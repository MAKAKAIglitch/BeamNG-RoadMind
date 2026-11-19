-- RoadMind low-level trajectory controller for BeamNG.drive.
-- Runs in Vehicle Lua. The GE coordinator supplies a short, safety-checked path.
-- This controller deliberately fails closed when its path feed becomes stale.

local M = {}

local UPDATE_INTERVAL = 0.05
local PATH_MAX_AGE = 0.75
local MAX_PATH_POINTS = 32
local MAX_SPEED_MPS = 8.0
local LOOKAHEAD_BASE_M = 4.0
local LOOKAHEAD_SPEED_FACTOR = 0.55
local LOOKAHEAD_MIN_M = 4.0
local LOOKAHEAD_MAX_M = 14.0
local WHEELBASE_M = 2.7
local MAX_STEER_ANGLE_RAD = 0.55
local MAX_THROTTLE = 0.55
local MAX_BRAKE = 1.0
local SPEED_KP = 0.18
local SPEED_KI = 0.035
local SPEED_INTEGRAL_LIMIT = 3.0

local state = {
  enabled = false,
  safetyLatched = false,
  path = {},
  pathAge = math.huge,
  updateTimer = 0,
  status = "idle",
  reason = "inactive",
  targetSpeed = MAX_SPEED_MPS,
  speedIntegral = 0,
  lastSteering = 0,
  lastThrottle = 0,
  lastBrake = 0,
  lastTargetDistance = nil
}

local function clamp(value, minimum, maximum)
  return math.max(minimum, math.min(maximum, value))
end

local function finite(value)
  return type(value) == "number" and value == value
    and value ~= math.huge and value ~= -math.huge
end

local function emit(name, value)
  if type(input) ~= "table" or type(input.event) ~= "function" then return false end
  local filter = FILTER_DIRECT or 2
  local ok = pcall(input.event, name, value, filter)
  return ok
end

local function applyInputs(steering, throttle, brake)
  steering = clamp(steering or 0, -1, 1)
  throttle = clamp(throttle or 0, 0, MAX_THROTTLE)
  brake = clamp(brake or 0, 0, MAX_BRAKE)

  emit("steering", steering)
  emit("throttle", throttle)
  emit("brake", brake)

  state.lastSteering = steering
  state.lastThrottle = throttle
  state.lastBrake = brake
end

local function failSafe(reason)
  state.status = "stopped"
  state.reason = reason or "safety_stop"
  state.speedIntegral = 0
  applyInputs(0, 0, 1)
end

local function readVehicleState()
  if not obj or type(obj.getPosition) ~= "function"
      or type(obj.getDirectionVector) ~= "function"
      or type(obj.getVelocity) ~= "function" then
    return nil
  end

  local ok, position, forward, velocity = pcall(function()
    return obj:getPosition(), obj:getDirectionVector(), obj:getVelocity()
  end)
  if not ok or not position or not forward or not velocity then return nil end

  local speed = velocity:length()
  local forwardLength = math.sqrt(forward.x * forward.x + forward.y * forward.y)
  if not finite(speed) or not finite(forwardLength) or forwardLength < 0.0001 then return nil end

  return position, forward, velocity, speed, forwardLength
end

local function nearestSegment(position)
  local bestIndex, bestT, bestDistanceSquared = nil, 0, math.huge
  for i = 1, #state.path - 1 do
    local a, b = state.path[i], state.path[i + 1]
    local dx, dy = b.x - a.x, b.y - a.y
    local lengthSquared = dx * dx + dy * dy
    if lengthSquared > 0.04 then
      local t = clamp(((position.x - a.x) * dx + (position.y - a.y) * dy) / lengthSquared, 0, 1)
      local px, py = a.x + dx * t, a.y + dy * t
      local ex, ey = position.x - px, position.y - py
      local distanceSquared = ex * ex + ey * ey
      if distanceSquared < bestDistanceSquared then
        bestIndex, bestT, bestDistanceSquared = i, t, distanceSquared
      end
    end
  end
  return bestIndex, bestT, bestDistanceSquared
end

local function pointAtDistance(startIndex, startT, distance)
  local first = state.path[startIndex]
  local second = state.path[startIndex + 1]
  if not first or not second then return nil end

  local dx, dy = second.x - first.x, second.y - first.y
  local segmentLength = math.sqrt(dx * dx + dy * dy)
  local remaining = segmentLength * (1 - startT) - distance
  if remaining <= 0 then
    distance = -remaining
    for i = startIndex + 1, #state.path - 1 do
      first, second = state.path[i], state.path[i + 1]
      dx, dy = second.x - first.x, second.y - first.y
      segmentLength = math.sqrt(dx * dx + dy * dy)
      if segmentLength >= distance and segmentLength > 0.001 then
        local t = distance / segmentLength
        return first.x + dx * t, first.y + dy * t
      end
      distance = distance - segmentLength
    end
    return state.path[#state.path].x, state.path[#state.path].y
  end

  local t = startT + distance / segmentLength
  return first.x + dx * t, first.y + dy * t
end

local function controlStep(dt)
  if not state.enabled then return end
  if state.safetyLatched then
    applyInputs(0, 0, 1)
    state.status = "stopped"
    return
  end
  if state.pathAge > PATH_MAX_AGE then
    failSafe("path_feed_stale")
    return
  end
  if #state.path < 2 then
    failSafe("path_unavailable")
    return
  end

  local position, forward, velocity, speed, forwardLength = readVehicleState()
  if not position then
    failSafe("vehicle_state_unavailable")
    return
  end

  local index, t, distanceSquared = nearestSegment(position)
  if not index or distanceSquared > 100 then
    failSafe("route_tracking_lost")
    return
  end

  local lookahead = clamp(LOOKAHEAD_BASE_M + speed * LOOKAHEAD_SPEED_FACTOR,
    LOOKAHEAD_MIN_M, LOOKAHEAD_MAX_M)
  local targetX, targetY = pointAtDistance(index, t, lookahead)
  if not targetX or not targetY then
    failSafe("lookahead_unavailable")
    return
  end

  local dx, dy = targetX - position.x, targetY - position.y
  local distance = math.sqrt(dx * dx + dy * dy)
  if distance < 0.5 then
    failSafe("target_too_close")
    return
  end

  local fx, fy = forward.x / forwardLength, forward.y / forwardLength
  local cross = fx * dy - fy * dx
  local dot = fx * dx + fy * dy
  local alpha = math.atan2(cross, dot)
  local curvature = 2 * math.sin(alpha) / math.max(1, distance)
  local steeringAngle = math.atan(WHEELBASE_M * curvature)
  -- BeamNG positive steering input is right; the planar cross product above is positive to the left.
  local steering = clamp(-steeringAngle / MAX_STEER_ANGLE_RAD, -1, 1)

  local targetSpeed = clamp(state.targetSpeed, 0, MAX_SPEED_MPS)
  local speedError = targetSpeed - speed
  state.speedIntegral = clamp(state.speedIntegral + speedError * dt,
    -SPEED_INTEGRAL_LIMIT, SPEED_INTEGRAL_LIMIT)
  local throttle = clamp(speedError * SPEED_KP + state.speedIntegral * SPEED_KI, 0, MAX_THROTTLE)
  local brake = clamp(-speedError * 0.22, 0, MAX_BRAKE)

  -- Reduce speed for tight curvature and sharp steering demand.
  local curveLimit = clamp(1.0 - math.abs(steering) * 0.55, 0.35, 1.0)
  targetSpeed = targetSpeed * curveLimit
  speedError = targetSpeed - speed
  throttle = clamp(speedError * SPEED_KP + state.speedIntegral * SPEED_KI, 0, MAX_THROTTLE)
  brake = clamp(-speedError * 0.22, 0, MAX_BRAKE)

  applyInputs(steering, throttle, brake)
  state.status = "active"
  state.reason = "tracking"
  state.lastTargetDistance = distance
end

function M.setPath(encodedPath)
  if type(encodedPath) ~= "string" or type(jsonDecode) ~= "function" then return false end
  local ok, decoded = pcall(jsonDecode, encodedPath)
  if not ok or type(decoded) ~= "table" or #decoded < 2 or #decoded > MAX_PATH_POINTS then
    return false
  end

  local path = {}
  for i = 1, #decoded do
    local point = decoded[i]
    if type(point) ~= "table" or not finite(point.x) or not finite(point.y) or not finite(point.z) then
      return false
    end
    path[i] = {x = point.x, y = point.y, z = point.z}
  end

  state.path = path
  state.pathAge = 0
  state.safetyLatched = false
  state.status = "ready"
  state.reason = "path_received"
  state.speedIntegral = 0
  return true
end

function M.setEnabled(enabled)
  state.enabled = enabled == true
  state.updateTimer = 0
  state.speedIntegral = 0
  if state.enabled then
    if #state.path < 2 then
      failSafe("path_unavailable")
      return false
    end
    state.status = "ready"
    state.reason = "armed"
    applyInputs(0, 0, 0)
    return true
  end

  state.status = "idle"
  state.reason = "disabled"
  applyInputs(0, 0, 0)
  return true
end

function M.setSafetyStop(reason)
  if state.enabled then
    state.safetyLatched = true
    failSafe(reason or "supervisor_stop")
  end
  return true
end

function M.setTargetSpeed(speed)
  if not finite(speed) then return false end
  state.targetSpeed = clamp(speed, 0, MAX_SPEED_MPS)
  return true
end

function M.getStatus()
  return {
    enabled = state.enabled,
    status = state.status,
    reason = state.reason,
    pathPoints = #state.path,
    pathAge = state.pathAge,
    targetSpeed = state.targetSpeed,
    steering = state.lastSteering,
    throttle = state.lastThrottle,
    brake = state.lastBrake,
    targetDistance = state.lastTargetDistance
  }
end

function M.updateGFX(dt)
  dt = math.max(0, tonumber(dt) or 0)
  state.pathAge = state.pathAge + dt
  if not state.enabled then return end

  state.updateTimer = state.updateTimer + dt
  if state.updateTimer < UPDATE_INTERVAL then return end
  local step = math.min(0.15, state.updateTimer)
  state.updateTimer = 0
  controlStep(step)
end

function M.onReset()
  state.enabled = false
  state.safetyLatched = false
  state.path = {}
  state.pathAge = math.huge
  state.updateTimer = 0
  state.speedIntegral = 0
  state.status = "idle"
  state.reason = "vehicle_reset"
  applyInputs(0, 0, 0)
end

function M.onExtensionUnloaded()
  M.onReset()
end

return M
