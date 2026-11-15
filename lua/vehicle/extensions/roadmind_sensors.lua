-- RoadMind forward-facing virtual LiDAR.
-- This file runs in the active vehicle Lua VM.

local M = {}

local CONFIG = {
  maxRange = 60,
  minRange = 0.35,
  horizontalFov = math.rad(120),
  verticalFov = math.rad(16),
  horizontalSamples = 17,
  verticalSamples = 3,
  scanInterval = 0.25,
  maxPoints = 51,

  -- Virtual mounting point near the front grille.
  forwardOffset = 2.0,
  upOffset = 0.95
}

local state = {
  enabled = true,
  timer = 0,
  sequence = 0,
  lastPointCount = 0
}

local function round2(value)
  return math.floor(value * 100 + 0.5) / 100
end

local function safeLength(value)
  if not value or type(value.length) ~= "function" then
    return 0
  end
  return value:length()
end

local function normalize(value)
  local length = safeLength(value)
  if length < 0.000001 then
    return nil
  end
  return value / length
end

local function getSensorFrame()
  if not obj then
    return nil
  end

  local position = obj:getPosition()
  local forward = normalize(obj:getDirectionVector())
  local up = normalize(obj:getDirectionVectorUp())

  if not position or not forward or not up then
    return nil
  end

  local right = normalize(forward:cross(up))
  if not right then
    return nil
  end

  local origin = position
    + forward * CONFIG.forwardOffset
    + up * CONFIG.upOffset

  return origin, forward, right, up
end

local function getStaticDistance(origin, direction)
  if type(castRayStatic) ~= "function" then
    return nil
  end

  local ok, distance = pcall(castRayStatic, origin, direction, CONFIG.maxRange)
  if ok and type(distance) == "number"
      and distance >= CONFIG.minRange and distance <= CONFIG.maxRange then
    return distance
  end

  return nil
end

local function getDynamicHit(origin, direction, maxDistance)
  if type(castRay) ~= "function" then
    return nil
  end

  local endpoint = origin + direction * maxDistance
  local ok, hit = pcall(castRay, origin, endpoint, true, true)

  if ok and type(hit) == "table" and type(hit.dist) == "number"
      and hit.dist >= CONFIG.minRange and hit.dist <= maxDistance then
    return hit
  end

  return nil
end

local function addHit(points, origin, direction, right, forward, up, distance, worldPoint)
  if type(distance) ~= "number" or distance < CONFIG.minRange
      or distance > CONFIG.maxRange then
    return
  end

  local point = worldPoint or (origin + direction * distance)
  local relative = point - origin

  points[#points + 1] = {
    x = round2(relative:dot(right)),
    y = round2(relative:dot(forward)),
    z = round2(relative:dot(up)),
    distance = round2(distance)
  }
end

local function scan()
  local origin, forward, right, up = getSensorFrame()
  if not origin then
    return nil
  end

  local points = {}
  local hCount = CONFIG.horizontalSamples
  local vCount = CONFIG.verticalSamples

  for h = 0, hCount - 1 do
    local horizontalAngle = -CONFIG.horizontalFov * 0.5
      + CONFIG.horizontalFov * h / math.max(1, hCount - 1)

    local cosH = math.cos(horizontalAngle)
    local sinH = math.sin(horizontalAngle)

    for v = 0, vCount - 1 do
      if #points >= CONFIG.maxPoints then
        break
      end

      local verticalAngle = -CONFIG.verticalFov * 0.5
        + CONFIG.verticalFov * v / math.max(1, vCount - 1)

      local cosV = math.cos(verticalAngle)
      local sinV = math.sin(verticalAngle)

      local direction = normalize(
        forward * (cosV * cosH)
          + right * (cosV * sinH)
          + up * sinV
      )

      if direction then
        local staticDistance = getStaticDistance(origin, direction)
        local dynamicLimit = staticDistance or CONFIG.maxRange
        local dynamicHit = getDynamicHit(origin, direction, dynamicLimit)

        local distance = staticDistance
        local worldPoint = nil

        if staticDistance then
          worldPoint = origin + direction * staticDistance
        end

        if dynamicHit and (not distance or dynamicHit.dist < distance) then
          distance = dynamicHit.dist
          worldPoint = dynamicHit.pt
        end

        if distance then
          addHit(
            points,
            origin,
            direction,
            right,
            forward,
            up,
            distance,
            worldPoint
          )
        end
      end
    end
  end

  state.sequence = state.sequence + 1
  state.lastPointCount = #points

  return {
    sequence = state.sequence,
    range = CONFIG.maxRange,
    horizontalFov = 120,
    verticalFov = 16,
    scanHz = 1 / CONFIG.scanInterval,
    points = points,
    pointCount = #points
  }
end

local function publish(frame)
  if not frame or not obj or type(obj.queueGameEngineLua) ~= "function"
      or type(jsonEncode) ~= "function" then
    return
  end

  local ok, encoded = pcall(jsonEncode, frame)
  if not ok or type(encoded) ~= "string" then
    return
  end

  local command = "extensions.roadmind.onSensorData("
    .. string.format("%q", encoded) .. ")"

  pcall(obj.queueGameEngineLua, obj, command)
end

function M.setEnabled(enabled)
  state.enabled = enabled == true
  state.timer = 0
  return state.enabled
end

function M.getStatus()
  return {
    enabled = state.enabled,
    sequence = state.sequence,
    pointCount = state.lastPointCount,
    scanHz = 1 / CONFIG.scanInterval,
    rayBudget = CONFIG.horizontalSamples * CONFIG.verticalSamples
  }
end

function M.onReset()
  state.timer = 0
  state.sequence = 0
  state.lastPointCount = 0
end

function M.updateGFX(dt)
  if not state.enabled then
    return
  end

  dt = tonumber(dt) or 0
  if dt < 0 then
    dt = 0
  end

  state.timer = state.timer + dt
  if state.timer < CONFIG.scanInterval then
    return
  end

  -- Do not run catch-up scans after a slow frame.
  state.timer = 0

  local frame = scan()
  if frame then
    publish(frame)
  end
end

return M
