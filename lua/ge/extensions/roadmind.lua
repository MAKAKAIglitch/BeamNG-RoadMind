-- RoadMind world-space automotive sensor visualizer and coordinator.
-- All primitives are drawn directly into the BeamNG 3D world.

local M = {}

local CHECK_INTERVAL = 0.5
local MAX_FRAME_AGE = 1.1

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

local function safeField(object, key)
  if not object then return nil end
  local ok, value = pcall(function() return object[key] end)
  if ok then return value end
  return nil
end

local function round2(value)
  return math.floor((value or 0) * 100 + 0.5) / 100
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

local function setVehicleSensorState(vehicle, enabled, vehicleId)
  if not vehicle then
    return
  end

  local idArgument = type(vehicleId) == "number" and string.format("%.0f", vehicleId) or "nil"
  local command = "extensions.load('roadmind_sensors'); "
    .. "roadmind_sensors.setEnabled(" .. boolString(enabled) .. ", " .. idArgument .. ")"

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
      "if roadmind_sensors then roadmind_sensors.setEnabled(false, " .. tostring(state.activeVehicleId) .. ") end"
    )
  end

  state.activeVehicle = nil
  state.activeVehicleId = nil
  resetFrame()
end

local function attachVehicle(vehicle, id)
  state.activeVehicle = vehicle
  state.activeVehicleId = id
  resetFrame()
  setVehicleSensorState(vehicle, state.visualizationEnabled, id)
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

local function drawLine(drawer, a, b, color)
  if not drawer or type(drawer.drawLine) ~= "function" or not a or not b then
    return
  end
  pcall(drawer.drawLine, drawer, a, b, color)
end

local function drawSphere(drawer, position, radius, color)
  if not drawer or type(drawer.drawSphere) ~= "function" or not position then
    return
  end
  pcall(drawer.drawSphere, drawer, position, radius, color)
end

local function drawRayReturns(drawer, originTable, rays, color, pointColor, pointRadius)
  local origin = toPoint3F(originTable)
  if not origin or type(rays) ~= "table" then
    return
  end

  drawSphere(drawer, origin, 0.075, color)

  for i = 1, #rays do
    local ray = rays[i]
    if ray.hit == true then
      local hitPoint = toPoint3F(ray)
      if hitPoint then
        drawLine(drawer, origin, hitPoint, color)
        drawSphere(drawer, hitPoint, pointRadius, pointColor)
      end
    end
  end
end

local function drawLidar(drawer, lidar)
  if type(lidar) ~= "table" then
    return
  end

  drawRayReturns(
    drawer,
    lidar.origin,
    lidar.rays,
    ColorF(0.10, 0.76, 1.0, 0.88),
    ColorF(1.0, 0.45, 0.08, 1.0),
    0.065
  )
end

local function drawRadar(drawer, radar)
  if type(radar) ~= "table" then
    return
  end

  local mounts = radar.mounts or {}
  for i = 1, #mounts do
    local position = toPoint3F(mounts[i].position)
    drawSphere(drawer, position, 0.09, ColorF(0.15, 1.0, 0.35, 1.0))
  end

  local returns = radar.returns or {}
  for i = 1, #returns do
    local hit = returns[i]
    local origin = toPoint3F(hit.origin)
    local target = toPoint3F(hit)
    if origin and target then
      local approaching = type(hit.radialSpeed) == "number" and hit.radialSpeed < -0.5
      local color = approaching
        and ColorF(1.0, 0.18, 0.12, 0.95)
        or ColorF(0.95, 0.76, 0.12, 0.90)

      drawLine(drawer, origin, target, color)
      drawSphere(drawer, target, 0.13, color)
    end
  end
end

local function drawUltrasonic(drawer, ultrasonic)
  if type(ultrasonic) ~= "table" then
    return
  end

  local mounts = ultrasonic.mounts or {}
  for i = 1, #mounts do
    drawSphere(
      drawer,
      toPoint3F(mounts[i].position),
      0.065,
      ColorF(0.15, 0.95, 0.95, 1.0)
    )
  end

  local returns = ultrasonic.returns or {}
  for i = 1, #returns do
    local hit = returns[i]
    local origin = toPoint3F(hit.origin)
    local target = toPoint3F(hit)
    if origin and target then
      drawLine(drawer, origin, target, ColorF(0.10, 1.0, 0.82, 0.95))
      drawSphere(drawer, target, 0.085, ColorF(1.0, 0.88, 0.1, 1.0))
    end
  end
end

local function drawCameraFrustum(drawer, camera)
  local origin = toPoint3F(camera.position)
  local forward = camera.direction
  local up = camera.up

  if not origin or type(forward) ~= "table" or type(up) ~= "table" then
    return
  end

  local fwd = vec3(forward.x, forward.y, forward.z)
  local upVec = vec3(up.x, up.y, up.z)
  if fwd:length() < 0.00001 or upVec:length() < 0.00001 then
    return
  end
  fwd = fwd:normalized()
  upVec = upVec:normalized()

  local right = fwd:cross(upVec)
  if right:length() < 0.00001 then
    return
  end
  right = right:normalized()

  local range = type(camera.range) == "number" and camera.range or 25
  local hfov = math.rad(tonumber(camera.horizontalFov) or 90)
  local vfov = math.rad(tonumber(camera.verticalFov) or 55)
  local halfWidth = math.tan(hfov * 0.5) * range
  local halfHeight = math.tan(vfov * 0.5) * range

  local center = vec3(origin.x, origin.y, origin.z) + fwd * range
  local farLeftUp = center - right * halfWidth + upVec * halfHeight
  local farRightUp = center + right * halfWidth + upVec * halfHeight
  local farLeftDown = center - right * halfWidth - upVec * halfHeight
  local farRightDown = center + right * halfWidth - upVec * halfHeight

  local color = ColorF(0.28, 0.52, 1.0, 0.58)
  drawLine(drawer, origin, farLeftUp:toPoint3F(), color)
  drawLine(drawer, origin, farRightUp:toPoint3F(), color)
  drawLine(drawer, origin, farLeftDown:toPoint3F(), color)
  drawLine(drawer, origin, farRightDown:toPoint3F(), color)

  drawLine(drawer, farLeftUp:toPoint3F(), farRightUp:toPoint3F(), color)
  drawLine(drawer, farRightUp:toPoint3F(), farRightDown:toPoint3F(), color)
  drawLine(drawer, farRightDown:toPoint3F(), farLeftDown:toPoint3F(), color)
  drawLine(drawer, farLeftDown:toPoint3F(), farLeftUp:toPoint3F(), color)

  drawSphere(drawer, origin, 0.055, ColorF(0.25, 0.55, 1.0, 1.0))
end

local function drawLocalization(drawer, localization)
  if type(localization) ~= "table" then
    return
  end

  local imu = localization.imu
  if type(imu) == "table" and type(imu.position) == "table" then
    local origin = toPoint3F(imu.position)
    if origin then
      drawSphere(drawer, origin, 0.11, ColorF(0.9, 0.2, 1.0, 1.0))
    end
  end

  local gnss = localization.gnss
  if type(gnss) == "table" and type(gnss.position) == "table" then
    local origin = toPoint3F(gnss.position)
    if origin then
      drawSphere(drawer, origin, 0.12, ColorF(0.2, 1.0, 0.45, 1.0))
      drawSphere(drawer, origin, 0.22, ColorF(0.2, 1.0, 0.45, 0.35))
    end
  end
end

local function drawFrame()
  local drawer = debugDrawer
  local frame = state.latestData

  if not drawer or not frame or not frame.enabled
      or state.frameAge > MAX_FRAME_AGE then
    return
  end

  drawLidar(drawer, frame.lidar)
  drawRadar(drawer, frame.radar)
  drawUltrasonic(drawer, frame.ultrasonic)

  local cameras = frame.cameras or {}
  for i = 1, #cameras do
    drawCameraFrustum(drawer, cameras[i])
  end

  drawLocalization(drawer, frame.localization)
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
      setVehicleSensorState(state.activeVehicle, true, state.activeVehicleId)
    end

    log("I", "roadmind", "Automotive sensor suite visualization enabled")
  else
    if state.activeVehicle then
      setVehicleSensorState(state.activeVehicle, false, state.activeVehicleId)
    end

    log("I", "roadmind", "Automotive sensor suite visualization disabled")
  end

  return state.visualizationEnabled
end

local function raycastBeam(beam, fallbackOrigin, fallbackRange)
  if type(castRay) ~= "function" or type(beam) ~= "table" then return nil end

  local originData = beam.origin or fallbackOrigin
  local directionData = beam.direction
  local maxRange = type(beam.range) == "number" and beam.range or fallbackRange
  local minRange = type(beam.minRange) == "number" and beam.minRange or 0

  if type(originData) ~= "table" or type(directionData) ~= "table"
      or type(originData.x) ~= "number" or type(originData.y) ~= "number"
      or type(originData.z) ~= "number" or type(directionData.x) ~= "number"
      or type(directionData.y) ~= "number" or type(directionData.z) ~= "number"
      or type(maxRange) ~= "number" or maxRange <= minRange or minRange < 0 then
    return nil
  end

  local origin = vec3(originData.x, originData.y, originData.z)
  local direction = vec3(directionData.x, directionData.y, directionData.z)
  local directionLength = direction:length()
  if directionLength < 0.00001 then return nil end

  direction = direction / directionLength
  local target = origin + direction * maxRange
  -- Game Engine Lua castRay returns nil on a miss or a hit with dist/pt.
  local ok, hit = pcall(castRay, origin, target, true, true)
  if not ok or not hit then return nil end

  local distance = safeField(hit, "dist")
  if type(distance) ~= "number" or distance < minRange or distance > maxRange then
    return nil
  end

  local point = nil
  local pointData = safeField(hit, "pt")
  if pointData then
    local x, y, z = safeField(pointData, "x"), safeField(pointData, "y"), safeField(pointData, "z")
    if type(x) == "number" and type(y) == "number" and type(z) == "number" then
      point = vec3(x, y, z)
    end
  end
  if not point then point = origin + direction * distance end

  return {
    x = round2(point.x),
    y = round2(point.y),
    z = round2(point.z),
    distance = round2(distance)
  }
end

local function processLidar(lidar)
  if type(lidar) ~= "table" then return {rays = {}, pointCount = 0} end
  local returns = {}
  local beams = lidar.rays or {}
  for i = 1, #beams do
    local hit = raycastBeam(beams[i], lidar.origin, lidar.range)
    if hit then
      hit.hit = true
      returns[#returns + 1] = hit
    end
  end
  return {
    name = lidar.name,
    origin = lidar.origin,
    horizontalFov = lidar.horizontalFov,
    verticalFov = lidar.verticalFov,
    range = lidar.range,
    rays = returns,
    pointCount = #returns,
    rayCount = #beams
  }
end

local function processRadar(radar)
  if type(radar) ~= "table" then return {mounts = {}, returns = {}} end
  local returns = {}
  local beams = radar.beams or {}
  for i = 1, #beams do
    local beam = beams[i]
    local hit = raycastBeam(beam, nil, beam.range)
    if hit then
      hit.name = beam.name
      hit.origin = beam.origin
      returns[#returns + 1] = hit
    end
  end
  return {mounts = radar.mounts or {}, returns = returns, updateHz = radar.updateHz}
end

local function processUltrasonic(ultrasonic)
  if type(ultrasonic) ~= "table" then return {mounts = {}, returns = {}} end
  local nearestBySensor, sensorOrder = {}, {}
  local beams = ultrasonic.beams or {}
  for i = 1, #beams do
    local beam = beams[i]
    local hit = raycastBeam(beam, nil, beam.range)
    if hit then
      local current = nearestBySensor[beam.name]
      if not current then sensorOrder[#sensorOrder + 1] = beam.name end
      if not current or hit.distance < current.distance then
        hit.name = beam.name
        hit.origin = beam.origin
        nearestBySensor[beam.name] = hit
      end
    end
  end
  local returns = {}
  for i = 1, #sensorOrder do
    local hit = nearestBySensor[sensorOrder[i]]
    if hit then returns[#returns + 1] = hit end
  end
  return {mounts = ultrasonic.mounts or {}, returns = returns, updateHz = ultrasonic.updateHz}
end

local function mergeSensorFrame(frame)
  local updates = frame.updatedSensors or {}
  local latest = state.latestData or {
    enabled = true,
    lidar = {rays = {}, pointCount = 0},
    radar = {mounts = {}, returns = {}},
    ultrasonic = {mounts = {}, returns = {}},
    cameras = {}
  }
  latest.enabled = frame.enabled == true
  latest.sequence = frame.sequence

  if updates.lidar == true then latest.lidar = processLidar(frame.lidar) end
  if updates.radar == true then latest.radar = processRadar(frame.radar) end
  if updates.ultrasonic == true then latest.ultrasonic = processUltrasonic(frame.ultrasonic) end
  if type(frame.cameras) == "table" then latest.cameras = frame.cameras end
  if type(frame.localization) == "table" then latest.localization = frame.localization end
  if type(frame.mounts) == "table" then latest.mounts = frame.mounts end
  if type(frame.vehicleVelocity) == "table" then latest.vehicleVelocity = frame.vehicleVelocity end

  state.latestData = latest
  state.frameAge = 0
end

function M.onSensorData(encodedFrame)
  if not state.visualizationEnabled or type(encodedFrame) ~= "string"
      or type(jsonDecode) ~= "function" then return end
  local ok, frame = pcall(jsonDecode, encodedFrame)
  if not ok or type(frame) ~= "table" then return end
  -- Drop late messages from a vehicle that is no longer the active player car.
  if frame.sourceVehicleId ~= state.activeVehicleId then return end
  if frame.enabled == false then
    resetFrame()
    return
  end
  mergeSensorFrame(frame)
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

  -- Scene raycasts run only when Vehicle Lua sends a due sensor scan.
  -- This update callback redraws already cached geometry only.
  drawFrame()
end

function M.onVehicleDestroyed(vehicleId)
  if vehicleId == state.activeVehicleId then
    detachPreviousVehicle()
    resetFrame()
  end
end

return M
