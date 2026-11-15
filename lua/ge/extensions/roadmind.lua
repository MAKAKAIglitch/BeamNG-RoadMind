-- RoadMind world-space LiDAR visualizer and vehicle coordinator.
-- This is a Game Engine Lua extension: debugDrawer renders into the 3D world.

local M = {}

local CHECK_INTERVAL = 0.5
local MAX_FRAME_AGE = 0.8
local POINT_RADIUS = 0.075

local state = {
  activeVehicleId = nil,
  activeVehicle = nil,
  visualizationEnabled = false,
  latestData = nil,
  frameAge = math.huge
}

local checkTimer = 0

local function boolString(value)
  return value and "true" or "false"
end

local function getVehicleId(vehicle)
  if not vehicle or type(vehicle.getID) ~= "function" then
    return nil
  end

  local ok, id = pcall(vehicle.getID, vehicle)
  if ok then
    return id
  end

  return nil
end

local function queueVehicleCommand(vehicle, command)
  if not vehicle or type(vehicle.queueLuaCommand) ~= "function" then
    return false
  end

  local ok = pcall(vehicle.queueLuaCommand, vehicle, command)
  return ok
end

local function setVehicleSensorState(vehicle, enabled)
  if not vehicle then
    return
  end

  local command = "extensions.load('roadmind_sensors'); "
    .. "roadmind_sensors.setEnabled(" .. boolString(enabled) .. ")"

  queueVehicleCommand(vehicle, command)
end

local function getPlayerVehicle()
  if not be or type(be.getPlayerVehicle) ~= "function" then
    return nil
  end

  local ok, vehicle = pcall(be.getPlayerVehicle, be, 0)
  if ok then
    return vehicle
  end

  return nil
end

local function resetFrame()
  state.latestData = nil
  state.frameAge = math.huge
end

local function detachPreviousVehicle()
  if state.activeVehicle then
    queueVehicleCommand(
      state.activeVehicle,
      "if roadmind_sensors then roadmind_sensors.setEnabled(false) end"
    )
  end

  state.activeVehicle = nil
  state.activeVehicleId = nil
end

local function attachVehicle(vehicle, id)
  state.activeVehicle = vehicle
  state.activeVehicleId = id
  resetFrame()
  setVehicleSensorState(vehicle, state.visualizationEnabled)
end

local function syncPlayerVehicle()
  local vehicle = getPlayerVehicle()
  local id = getVehicleId(vehicle)

  if id == state.activeVehicleId then
    return
  end

  detachPreviousVehicle()

  if vehicle and id then
    attachVehicle(vehicle, id)
  end
end

local function toPoint3F(point)
  if type(point) ~= "table"
      or type(point.x) ~= "number"
      or type(point.y) ~= "number"
      or type(point.z) ~= "number" then
    return nil
  end

  return vec3(point.x, point.y, point.z):toPoint3F()
end

local function drawFrame()
  local drawer = debugDrawer
  local frame = state.latestData

  if not drawer or not frame or not frame.enabled
      or state.frameAge > MAX_FRAME_AGE
      or type(frame.origin) ~= "table"
      or type(frame.rays) ~= "table"
      or type(drawer.drawLine) ~= "function"
      or type(drawer.drawSphere) ~= "function" then
    return
  end

  local origin = toPoint3F(frame.origin)
  if not origin then
    return
  end

  -- A small marker identifies the virtual sensor's exact mounting position.
  drawer:drawSphere(origin, 0.11, ColorF(0.05, 0.85, 1.0, 1.0))

  for i = 1, #frame.rays do
    local ray = frame.rays[i]

    -- Show only beams that received a return. This keeps the scene readable
    -- and avoids drawing the full 60 m cone when no object was detected.
    if ray.hit == true then
      local hitPoint = toPoint3F(ray)
      if hitPoint then
        local distance = type(ray.distance) == "number" and ray.distance or 60
        local t = math.max(0, math.min(1, distance / 60))
        local lineColor = ColorF(0.1 + 0.9 * (1 - t), 0.82, 1.0, 0.9)
        local pointColor = ColorF(1.0, 0.35 + 0.45 * t, 0.08, 1.0)

        drawer:drawLine(origin, hitPoint, lineColor)
        drawer:drawSphere(hitPoint, POINT_RADIUS, pointColor)
      end
    end
  end
end

function M.onExtensionLoaded()
  if setExtensionUnloadMode then
    setExtensionUnloadMode(M, "manual")
  end
end

function M.toggleVisualization()
  state.visualizationEnabled = not state.visualizationEnabled
  resetFrame()

  if state.visualizationEnabled then
    syncPlayerVehicle()

    if state.activeVehicle then
      setVehicleSensorState(state.activeVehicle, true)
    end

    log("I", "roadmind", "3D LiDAR visualization enabled")
  else
    if state.activeVehicle then
      setVehicleSensorState(state.activeVehicle, false)
    end

    log("I", "roadmind", "3D LiDAR visualization disabled")
  end

  return state.visualizationEnabled
end

-- Receives serialized world-space ray endpoints from the active vehicle VM.
function M.onSensorData(encodedFrame)
  if not state.visualizationEnabled
      or type(encodedFrame) ~= "string"
      or type(jsonDecode) ~= "function" then
    return
  end

  local ok, frame = pcall(jsonDecode, encodedFrame)
  if not ok or type(frame) ~= "table"
      or type(frame.rays) ~= "table" then
    return
  end

  if frame.enabled == false then
    resetFrame()
    return
  end

  state.latestData = frame
  state.frameAge = 0
end

function M.onUpdate(dt)
  dt = tonumber(dt) or 0
  if dt < 0 then
    dt = 0
  end

  if not state.visualizationEnabled then
    return
  end

  state.frameAge = state.frameAge + dt
  checkTimer = checkTimer + dt

  if checkTimer >= CHECK_INTERVAL then
    checkTimer = 0
    syncPlayerVehicle()
  end

  -- debugDrawer primitives are transient, so redraw the last scan every GFX
  -- frame. Raycasts themselves remain rate-limited in Vehicle Lua to 4 Hz.
  drawFrame()
end

function M.onVehicleDestroyed(vehicleId)
  if vehicleId == state.activeVehicleId then
    detachPreviousVehicle()
    resetFrame()
  end
end

return M
