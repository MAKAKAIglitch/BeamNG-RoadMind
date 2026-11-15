-- RoadMind game-engine coordinator.
-- Attaches the vehicle-side sensor extension to the active player vehicle
-- and exposes the latest sensor frame to the UI app.

local M = {}

local CHECK_INTERVAL = 0.5

local state = {
  activeVehicleId = nil,
  activeVehicle = nil,
  sensorEnabled = true,
  latestData = {
    sequence = 0,
    range = 60,
    horizontalFov = 120,
    verticalFov = 16,
    points = {},
    pointCount = 0
  }
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

local function detachPreviousVehicle()
  if state.activeVehicle then
    queueVehicleCommand(
      state.activeVehicle,
      "if roadmind_sensors then roadmind_sensors.setEnabled(false) end"
    )
  end
end

local function attachVehicle(vehicle, id)
  state.activeVehicle = vehicle
  state.activeVehicleId = id

  state.latestData = {
    sequence = 0,
    range = 60,
    horizontalFov = 120,
    verticalFov = 16,
    points = {},
    pointCount = 0
  }

  setVehicleSensorState(vehicle, state.sensorEnabled)
end

function M.onExtensionLoaded()
  if setExtensionUnloadMode then
    setExtensionUnloadMode(M, "manual")
  end
end

function M.onUpdate(dt)
  dt = tonumber(dt) or 0
  if dt < 0 then
    dt = 0
  end

  checkTimer = checkTimer + dt
  if checkTimer < CHECK_INTERVAL then
    return
  end
  checkTimer = 0

  local vehicle = getPlayerVehicle()
  local id = getVehicleId(vehicle)

  if id ~= state.activeVehicleId then
    detachPreviousVehicle()

    if vehicle and id then
      attachVehicle(vehicle, id)
    else
      state.activeVehicle = nil
      state.activeVehicleId = nil
      state.latestData.points = {}
      state.latestData.pointCount = 0
    end
  end
end

function M.setSensorEnabled(enabled)
  state.sensorEnabled = enabled == true

  if state.activeVehicle then
    queueVehicleCommand(
      state.activeVehicle,
      "if roadmind_sensors then roadmind_sensors.setEnabled("
        .. boolString(state.sensorEnabled) .. ") end"
    )
  end

  return state.sensorEnabled
end

-- Called by the vehicle-side extension with a JSON-encoded sensor frame.
function M.onSensorData(encodedFrame)
  if type(encodedFrame) ~= "string" or type(jsonDecode) ~= "function" then
    return
  end

  local ok, frame = pcall(jsonDecode, encodedFrame)
  if not ok or type(frame) ~= "table" or type(frame.points) ~= "table" then
    return
  end

  state.latestData = frame
end

function M.getLatestData()
  return state.latestData
end

return M
