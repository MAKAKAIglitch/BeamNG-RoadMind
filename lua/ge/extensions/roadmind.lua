-- RoadMind world-space automotive sensor visualizer and coordinator.
-- All primitives are drawn directly into the BeamNG 3D world.

local M = {}

local CHECK_INTERVAL = 0.5
local MAX_FRAME_AGE = 1.1
local SETTINGS_FILE = "/settings/roadmind.json"

local DEFAULT_SETTINGS = {
  preset = "economy",
  lidarEnabled = true, lidarQuality = "low", lidarPointCount = 9, lidarHz = 1, lidarRange = 45,
  radarEnabled = false, radarDensity = "low", radarHz = 1,
  ultrasonicEnabled = false, ultrasonicDensity = "single", ultrasonicHz = 1,
  poseHz = 2,
  lidarPersistenceEnabled = false,
  showLidar = true, showRadar = false, showUltrasonic = false,
  showCameras = false, showLocalization = false, showDetectedVehicles = true
}

local PRESETS = {
  economy = {
    lidarEnabled = true, lidarQuality = "low", lidarPointCount = 9, lidarHz = 1, lidarRange = 45,
    radarEnabled = false, radarDensity = "low", radarHz = 1,
    ultrasonicEnabled = false, ultrasonicDensity = "single", ultrasonicHz = 1, poseHz = 2
  },
  balanced = {
    lidarEnabled = true, lidarQuality = "balanced", lidarPointCount = 13, lidarHz = 2, lidarRange = 65,
    radarEnabled = true, radarDensity = "medium", radarHz = 1,
    ultrasonicEnabled = true, ultrasonicDensity = "single", ultrasonicHz = 1, poseHz = 4
  },
  high = {
    lidarEnabled = true, lidarQuality = "high", lidarPointCount = 17, lidarHz = 4, lidarRange = 65,
    radarEnabled = true, radarDensity = "high", radarHz = 2,
    ultrasonicEnabled = true, ultrasonicDensity = "wide", ultrasonicHz = 2, poseHz = 5
  }
}

local SENSOR_SETTING_KEYS = {
  lidarEnabled = true, lidarQuality = true, lidarPointCount = true, lidarHz = true, lidarRange = true,
  radarEnabled = true, radarDensity = true, radarHz = true,
  ultrasonicEnabled = true, ultrasonicDensity = true, ultrasonicHz = true,
  poseHz = true, showCameras = true, showLocalization = true
}

local MAX_PERSISTENT_LIDAR_POINTS = 1800
local PERSISTENT_POINT_CELL_SIZE = 0.35
local ROUTE_MAX_SPEED_MPS = 8.0
local ROUTE_STOP_DISTANCE = 8.0
local ROUTE_RESUME_DISTANCE = 14.0

local state = {
  activeVehicleId = nil,
  activeVehicle = nil,
  visualizationEnabled = false,
  settings = nil,
  latestData = nil,
  frameAge = math.huge,
  lidarPointCloud = {},
  lidarPointCells = {},
  lidarPointWriteIndex = 1,
  routePilotEnabled = false,
  routePilotStarted = false,
  routeSafetyStopped = false,
  routePilotVehicleId = nil
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

local function safeCall(object, methodName, ...)
  local method = safeField(object, methodName)
  if type(method) ~= "function" then return nil end
  local ok, value = pcall(method, object, ...)
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

local function startNativeRouteAI(vehicle)
  if not vehicle then return false end
  local speed = string.format("%.2f", ROUTE_MAX_SPEED_MPS)
  return queueVehicleCommand(vehicle,
    "if ai then " ..
    "if ai.setAggression then pcall(ai.setAggression, 0.35) end; " ..
    "if ai.setSpeedMode then pcall(ai.setSpeedMode, 'limit') end; " ..
    "if ai.setSpeed then pcall(ai.setSpeed, " .. speed .. ") end; " ..
    "if ai.setAvoidCars then pcall(ai.setAvoidCars, true) end; " ..
    "if ai.setMode then ai.setMode('span') end end"
  )
end

local function stopNativeRouteAI(vehicle)
  if not vehicle then return false end
  return queueVehicleCommand(vehicle,
    "if ai and ai.setMode then pcall(ai.setMode, 'stopping') end"
  )
end

local function clearPersistentLidarPoints()
  state.lidarPointCloud = {}
  state.lidarPointCells = {}
  state.lidarPointWriteIndex = 1
end

local function vehicleSettingsCommand()
  local encoded = "{}"
  if type(jsonEncode) == "function" then
    local ok, value = pcall(jsonEncode, state.settings or DEFAULT_SETTINGS)
    if ok and type(value) == "string" then encoded = value end
  end
  return "roadmind_sensors.setSettings(" .. string.format("%q", encoded) .. "); "
end

local function setVehicleSensorSettings(vehicle)
  if not vehicle then return end
  local command = "extensions.load('roadmind_sensors'); " .. vehicleSettingsCommand()
  queueVehicleCommand(vehicle, command)
end

local function setVehicleSensorState(vehicle, enabled, vehicleId)
  if not vehicle then return end
  local idArgument = type(vehicleId) == "number" and string.format("%.0f", vehicleId) or "nil"
  local command = "extensions.load('roadmind_sensors'); "
  if enabled then command = command .. vehicleSettingsCommand() end
  command = command .. "roadmind_sensors.setEnabled(" .. boolString(enabled) .. ", " .. idArgument .. ")"
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
    if state.routePilotEnabled then stopNativeRouteAI(state.activeVehicle) end
    queueVehicleCommand(
      state.activeVehicle,
      "if roadmind_sensors then roadmind_sensors.setEnabled(false, " .. tostring(state.activeVehicleId) .. ") end"
    )
  end

  state.routePilotEnabled = false
  state.routePilotStarted = false
  state.routeSafetyStopped = false
  state.routePilotVehicleId = nil
  state.activeVehicle = nil
  state.activeVehicleId = nil
  clearPersistentLidarPoints()
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

local function drawPersistentLidarCloud(drawer)
  if not state.settings or not state.settings.lidarPersistenceEnabled then return end
  for i = 1, #state.lidarPointCloud do
    local point = state.lidarPointCloud[i]
    if point then
      drawSphere(drawer, toPoint3F(point), 0.045, ColorF(0.95, 0.72, 0.20, 0.82))
    end
  end
end

local function drawDetectedVehicles(drawer, vehicles)
  if type(vehicles) ~= "table" then return end
  for i = 1, #vehicles do
    local target = vehicles[i]
    local position = toPoint3F(target.position)
    local origin = toPoint3F(target.sensorOrigin)
    if position then
      local color = type(target.radialSpeed) == "number" and target.radialSpeed < -1.0
        and ColorF(1.0, 0.18, 0.12, 1.0)
        or ColorF(0.30, 1.0, 0.40, 1.0)
      if origin then drawLine(drawer, origin, position, color) end
      drawSphere(drawer, position, 0.42, color)
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
  local settings = state.settings or DEFAULT_SETTINGS

  if not drawer or not frame or not frame.enabled
      or state.frameAge > MAX_FRAME_AGE then
    return
  end

  if settings.showLidar then
    drawLidar(drawer, frame.lidar)
    drawPersistentLidarCloud(drawer)
  end
  if settings.showRadar then drawRadar(drawer, frame.radar) end
  if settings.showUltrasonic then drawUltrasonic(drawer, frame.ultrasonic) end
  if settings.showDetectedVehicles then drawDetectedVehicles(drawer, frame.vehicles) end

  if settings.showCameras then
    local cameras = frame.cameras or {}
    for i = 1, #cameras do drawCameraFrustum(drawer, cameras[i]) end
  end

  if settings.showLocalization then drawLocalization(drawer, frame.localization) end
end

local function copySettings(source)
  local result = {}
  for key, value in pairs(source or {}) do result[key] = value end
  return result
end

local function enumValue(value, allowed)
  for i = 1, #allowed do
    if value == allowed[i] then return true end
  end
  return false
end

local function normalizeSettings(source)
  source = type(source) == "table" and source or {}
  local result = copySettings(DEFAULT_SETTINGS)

  local booleanKeys = {
    "lidarEnabled", "radarEnabled", "ultrasonicEnabled",
    "lidarPersistenceEnabled",
    "showLidar", "showRadar", "showUltrasonic", "showCameras",
    "showLocalization", "showDetectedVehicles"
  }
  for i = 1, #booleanKeys do
    local key = booleanKeys[i]
    if type(source[key]) == "boolean" then result[key] = source[key] end
  end

  local enums = {
    preset = {"economy", "balanced", "high", "custom"},
    lidarQuality = {"low", "balanced", "high"},
    radarDensity = {"low", "medium", "high"},
    ultrasonicDensity = {"single", "wide"}
  }
  for key, allowed in pairs(enums) do
    if enumValue(source[key], allowed) then result[key] = source[key] end
  end

  if type(source.lidarPointCount) == "number" and source.lidarPointCount >= 9
      and source.lidarPointCount <= 33 then
    result.lidarPointCount = math.floor((source.lidarPointCount - 9) / 4 + 0.5) * 4 + 9
  end
  if source.lidarHz == 1 or source.lidarHz == 2 or source.lidarHz == 4 then result.lidarHz = source.lidarHz end
  if type(source.lidarRange) == "number" and source.lidarRange >= 30 and source.lidarRange <= 160 then
    result.lidarRange = math.floor(source.lidarRange / 5 + 0.5) * 5
  end
  if source.radarHz == 1 or source.radarHz == 2 then result.radarHz = source.radarHz end
  if source.ultrasonicHz == 1 or source.ultrasonicHz == 2 then result.ultrasonicHz = source.ultrasonicHz end
  if source.poseHz == 2 or source.poseHz == 4 or source.poseHz == 5 then result.poseHz = source.poseHz end

  return result
end

local function loadSettings()
  local loaded = nil
  if type(jsonReadFile) == "function" then
    local ok, data = pcall(jsonReadFile, SETTINGS_FILE)
    if ok and type(data) == "table" then loaded = data end
  end
  state.settings = normalizeSettings(loaded or DEFAULT_SETTINGS)
end

local function saveSettings()
  if type(jsonWriteFile) ~= "function" then return false end
  local ok = pcall(jsonWriteFile, SETTINGS_FILE, state.settings, true)
  return ok
end

local function rayCountFor(settings)
  local lidarLayers = settings.lidarQuality == "low" and 1
    or (settings.lidarQuality == "balanced" and 2 or 3)
  local lidarCount = (settings.lidarPointCount or 9) * lidarLayers
  local radarCount = settings.radarDensity == "low" and 5
    or (settings.radarDensity == "medium" and 10 or 17)
  local ultrasonicCount = settings.ultrasonicDensity == "single" and 12 or 36

  local total = 0
  if settings.lidarEnabled then total = total + lidarCount * settings.lidarHz end
  if settings.radarEnabled then total = total + radarCount * settings.radarHz end
  if settings.ultrasonicEnabled then total = total + ultrasonicCount * settings.ultrasonicHz end
  return total
end

local function getGameLanguage()
  if type(settings) == "table" and type(settings.getValue) == "function" then
    local ok, language = pcall(settings.getValue, "userLanguage")
    if ok and type(language) == "string" and language ~= "" then
      return language
    end
  end
  return "en-US"
end

local function settingsResult()
  local settings = state.settings or DEFAULT_SETTINGS
  return {
    ok = true,
    language = getGameLanguage(),
    settings = copySettings(settings),
    status = {
      visualizationEnabled = state.visualizationEnabled,
      activeVehicle = state.activeVehicleId ~= nil,
      estimatedRaycastsPerSecond = state.visualizationEnabled and rayCountFor(settings) or 0,
      routePilotEnabled = state.routePilotEnabled,
      routeSafetyStopped = state.routeSafetyStopped,
      persistentLidarPointCount = #state.lidarPointCloud,
      detectedVehicleCount = state.latestData and #(state.latestData.vehicles or {}) or 0
    }
  }
end

function M.getSettings()
  if not state.settings then loadSettings() end
  return settingsResult()
end

function M.setSetting(key, value)
  if not state.settings then loadSettings() end
  if DEFAULT_SETTINGS[key] == nil then
    return {ok = false, error = "Unknown setting"}
  end

  local candidate = copySettings(state.settings)
  candidate[key] = value
  if key ~= "preset" then candidate.preset = "custom" end
  local normalized = normalizeSettings(candidate)

  if type(DEFAULT_SETTINGS[key]) == "boolean" and type(value) ~= "boolean" then
    return {ok = false, error = "Invalid boolean value"}
  end
  if type(DEFAULT_SETTINGS[key]) == "number" and type(value) ~= "number" then
    return {ok = false, error = "Invalid numeric value"}
  end
  if type(DEFAULT_SETTINGS[key]) == "string" and normalized[key] ~= value then
    return {ok = false, error = "Invalid option"}
  end
  if type(DEFAULT_SETTINGS[key]) == "number" and key ~= "lidarRange" and key ~= "lidarPointCount"
      and normalized[key] ~= value then
    return {ok = false, error = "Invalid numeric option"}
  end
  if key == "lidarRange" and (value < 30 or value > 160) then
    return {ok = false, error = "LiDAR range must be between 30 and 160 m"}
  end
  if key == "lidarPointCount"
      and (value < 9 or value > 33 or math.abs((value - 9) % 4) > 0.001) then
    return {ok = false, error = "LiDAR point count must be 9–33 in steps of 4"}
  end

  state.settings = normalized
  saveSettings()

  if state.visualizationEnabled and state.activeVehicle and SENSOR_SETTING_KEYS[key] then
    setVehicleSensorSettings(state.activeVehicle)
  end
  return settingsResult()
end

function M.setPreset(name)
  if not state.settings then loadSettings() end
  local preset = PRESETS[name]
  if not preset then return {ok = false, error = "Unknown performance preset"} end

  local candidate = copySettings(state.settings)
  candidate.preset = name
  for key, value in pairs(preset) do candidate[key] = value end
  state.settings = normalizeSettings(candidate)
  saveSettings()

  if state.visualizationEnabled and state.activeVehicle then
    setVehicleSensorSettings(state.activeVehicle)
  end
  return settingsResult()
end

function M.setVisualizationEnabled(enabled)
  if not state.settings then loadSettings() end
  local wanted = enabled == true

  if wanted then
    local previousId = state.activeVehicleId
    state.visualizationEnabled = true
    resetFrame()
    syncPlayerVehicle()
    if state.activeVehicle and state.activeVehicleId == previousId then
      setVehicleSensorState(state.activeVehicle, true, state.activeVehicleId)
    end
    if not state.activeVehicle then
      state.visualizationEnabled = false
      return {ok = false, error = "No active player vehicle"}
    end
  else
    if state.routePilotEnabled and state.activeVehicle then
      stopNativeRouteAI(state.activeVehicle)
    end
    state.routePilotEnabled = false
    state.routePilotStarted = false
    state.routeSafetyStopped = false
    state.routePilotVehicleId = nil
    state.visualizationEnabled = false
    if state.activeVehicle then
      setVehicleSensorState(state.activeVehicle, false, state.activeVehicleId)
    end
    resetFrame()
  end

  return settingsResult()
end

function M.setRoutePilotEnabled(enabled)
  if not state.settings then loadSettings() end

  if enabled ~= true then
    if state.routePilotEnabled and state.activeVehicle then
      stopNativeRouteAI(state.activeVehicle)
    end
    state.routePilotEnabled = false
    state.routePilotStarted = false
    state.routeSafetyStopped = false
    state.routePilotVehicleId = nil
    return settingsResult()
  end

  if not state.visualizationEnabled then
    local started = M.setVisualizationEnabled(true)
    if not started.ok then return started end
  end
  if not state.activeVehicle or not state.activeVehicleId then
    return {ok = false, error = "No active player vehicle"}
  end

  -- The route pilot explicitly switches to a bounded Balanced sensor baseline.
  -- It enables LiDAR, all radars and ultrasonics, plus pose telemetry.
  local candidate = copySettings(state.settings)
  for key, value in pairs(PRESETS.balanced) do candidate[key] = value end
  candidate.preset = "balanced"
  candidate.showLidar = true
  candidate.showRadar = true
  candidate.showUltrasonic = true
  candidate.showCameras = true
  candidate.showLocalization = true
  candidate.showDetectedVehicles = true
  state.settings = normalizeSettings(candidate)
  saveSettings()
  setVehicleSensorSettings(state.activeVehicle)

  state.routePilotEnabled = true
  state.routePilotStarted = false
  state.routeSafetyStopped = true
  state.routePilotVehicleId = state.activeVehicleId
  resetFrame()

  log("I", "roadmind", "Experimental road-network route pilot armed; waiting for fresh sensor data")
  return settingsResult()
end

function M.clearLidarMap()
  clearPersistentLidarPoints()
  return settingsResult()
end

function M.onExtensionLoaded()
  if setExtensionUnloadMode then
    setExtensionUnloadMode(M, "manual")
  end
  loadSettings()
end

function M.toggleVisualization()
  local result = M.setVisualizationEnabled(not state.visualizationEnabled)
  if not result.ok then
    log("W", "roadmind", result.error or "Could not toggle sensor suite")
    return false
  end

  log("I", "roadmind", state.visualizationEnabled
    and "Automotive sensor suite visualization enabled"
    or "Automotive sensor suite visualization disabled")
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
    forward = lidar.forward,
    right = lidar.right,
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

local function vectorFromTable(point)
  if type(point) ~= "table" or type(point.x) ~= "number"
      or type(point.y) ~= "number" or type(point.z) ~= "number" then
    return nil
  end
  return vec3(point.x, point.y, point.z)
end

local function pointCellKey(point)
  local size = PERSISTENT_POINT_CELL_SIZE
  return table.concat({
    tostring(math.floor(point.x / size)),
    tostring(math.floor(point.y / size)),
    tostring(math.floor(point.z / size))
  }, ":")
end

local function rememberLidarPoints(lidar)
  if not state.settings or not state.settings.lidarPersistenceEnabled
      or type(lidar) ~= "table" or type(lidar.rays) ~= "table" then
    return
  end

  for i = 1, #lidar.rays do
    local point = lidar.rays[i]
    if point.hit == true and type(point.x) == "number"
        and type(point.y) == "number" and type(point.z) == "number" then
      local key = pointCellKey(point)
      if not state.lidarPointCells[key] then
        local slot
        if #state.lidarPointCloud < MAX_PERSISTENT_LIDAR_POINTS then
          slot = #state.lidarPointCloud + 1
        else
          slot = state.lidarPointWriteIndex
          local previous = state.lidarPointCloud[slot]
          if previous then
            local previousKey = pointCellKey(previous)
            if state.lidarPointCells[previousKey] == slot then
              state.lidarPointCells[previousKey] = nil
            end
          end
          state.lidarPointWriteIndex = (slot % MAX_PERSISTENT_LIDAR_POINTS) + 1
        end

        state.lidarPointCloud[slot] = {x = point.x, y = point.y, z = point.z}
        state.lidarPointCells[key] = slot
      end
    end
  end
end

local function pointInsideSensorCone(targetPosition, mountPosition, direction, range, fovDegrees)
  local target = vectorFromTable(targetPosition)
  local origin = vectorFromTable(mountPosition)
  local forward = vectorFromTable(direction)
  if not target or not origin or not forward or type(range) ~= "number" then return false end

  local delta = target - origin
  local distance = delta:length()
  if distance < 0.5 or distance > range then return false end
  if forward:length() < 0.00001 then return false end

  local cosine = math.max(-1, math.min(1, delta:normalized():dot(forward:normalized())))
  local halfAngle = math.rad((tonumber(fovDegrees) or 0) * 0.5)
  return cosine >= math.cos(halfAngle), distance
end

local function detectNearbyVehicles(data)
  local detected = {}
  if not state.activeVehicle or type(getAllVehicles) ~= "function" then return detected end

  local ok, vehicles = pcall(getAllVehicles)
  if not ok or type(vehicles) ~= "table" then return detected end

  local settings = state.settings or DEFAULT_SETTINGS
  local playerVelocity = safeCall(state.activeVehicle, "getVelocity")
  local examined = 0

  for _, otherVehicle in pairs(vehicles) do
    examined = examined + 1
    if examined > 96 then break end

    local otherId = getVehicleId(otherVehicle)
    if otherId and otherId ~= state.activeVehicleId then
      local position = safeCall(otherVehicle, "getPosition")
      if position then
        local targetPosition = {x = position.x, y = position.y, z = position.z}
        local bestDistance = math.huge
        local bestOrigin = nil
        local detectedBy = {}

        if settings.lidarEnabled and data.lidar then
          local visible, distance = pointInsideSensorCone(
            targetPosition, data.lidar.origin, data.lidar.forward,
            data.lidar.range, data.lidar.horizontalFov or 120
          )
          if visible then
            detectedBy[#detectedBy + 1] = "LiDAR"
            bestDistance = distance
            bestOrigin = data.lidar.origin
          end
        end

        if settings.radarEnabled and data.radar and type(data.radar.mounts) == "table" then
          for i = 1, #data.radar.mounts do
            local mount = data.radar.mounts[i]
            local visible, distance = pointInsideSensorCone(
              targetPosition, mount.position, mount.direction, mount.range, mount.fov
            )
            if visible then
              detectedBy[#detectedBy + 1] = mount.name or "Radar"
              if distance < bestDistance then
                bestDistance = distance
                bestOrigin = mount.position
              end
            end
          end
        end

        if settings.ultrasonicEnabled and data.ultrasonic
            and type(data.ultrasonic.mounts) == "table" then
          for i = 1, #data.ultrasonic.mounts do
            local mount = data.ultrasonic.mounts[i]
            local visible, distance = pointInsideSensorCone(
              targetPosition, mount.position, mount.direction, mount.range, 16
            )
            if visible then
              detectedBy[#detectedBy + 1] = mount.name or "Ultrasonic"
              if distance < bestDistance then
                bestDistance = distance
                bestOrigin = mount.position
              end
            end
          end
        end

        if #detectedBy > 0 then
          local targetVelocity = safeCall(otherVehicle, "getVelocity")
          local radialSpeed = nil
          if targetVelocity and playerVelocity then
            local relativeVelocity = targetVelocity - playerVelocity
            local playerPosition = safeCall(state.activeVehicle, "getPosition")
            if playerPosition then
              local delta = position - playerPosition
              if delta:length() > 0.001 then
                radialSpeed = round2(relativeVelocity:dot(delta:normalized()))
              end
            end
          end

          detected[#detected + 1] = {
            id = otherId,
            position = targetPosition,
            sensorOrigin = bestOrigin,
            distance = round2(bestDistance),
            radialSpeed = radialSpeed,
            detectedBy = detectedBy
          }
        end
      end
    end
  end

  table.sort(detected, function(a, b) return a.distance < b.distance end)
  return detected
end

local function nearestForwardObstacle(data)
  if type(data) ~= "table" then return math.huge end

  local forward = data.lidar and vectorFromTable(data.lidar.forward) or nil
  local right = data.lidar and vectorFromTable(data.lidar.right) or nil
  local originFallback = data.lidar and vectorFromTable(data.lidar.origin) or nil

  if not forward then
    local mounts = data.radar and data.radar.mounts or {}
    for i = 1, #mounts do
      if mounts[i].name == "Front long-range" then
        forward = vectorFromTable(mounts[i].direction)
        right = forward and vec3(-forward.y, forward.x, 0) or nil
        originFallback = vectorFromTable(mounts[i].position)
        break
      end
    end
  end
  if not forward or forward:length() < 0.00001 then return math.huge end
  forward = forward:normalized()
  if not right or right:length() < 0.00001 then right = vec3(-forward.y, forward.x, 0) end
  if right:length() < 0.00001 then return math.huge end
  right = right:normalized()

  local minimum = math.huge
  local function inspect(point, originData)
    local target = vectorFromTable(point)
    local origin = vectorFromTable(originData) or originFallback
    if not target or not origin then return end
    local delta = target - origin
    local longitudinal = delta:dot(forward)
    local lateral = math.abs(delta:dot(right))
    if longitudinal > 0.5 and lateral <= 2.4 and delta.z > -0.85 then
      minimum = math.min(minimum, longitudinal)
    end
  end

  local lidarReturns = data.lidar and data.lidar.rays or {}
  for i = 1, #lidarReturns do inspect(lidarReturns[i], data.lidar.origin) end

  local radarReturns = data.radar and data.radar.returns or {}
  for i = 1, #radarReturns do inspect(radarReturns[i], radarReturns[i].origin) end

  local ultrasonicReturns = data.ultrasonic and data.ultrasonic.returns or {}
  for i = 1, #ultrasonicReturns do inspect(ultrasonicReturns[i], ultrasonicReturns[i].origin) end

  for i = 1, #(data.vehicles or {}) do
    inspect(data.vehicles[i].position, originFallback)
  end

  return minimum
end

local function updateRoutePilotSafety()
  if not state.routePilotEnabled then return end
  if not state.activeVehicle or state.routePilotVehicleId ~= state.activeVehicleId then
    state.routePilotEnabled = false
    state.routePilotStarted = false
    state.routeSafetyStopped = false
    state.routePilotVehicleId = nil
    return
  end

  if state.frameAge > MAX_FRAME_AGE or not state.latestData then
    if not state.routeSafetyStopped then
      stopNativeRouteAI(state.activeVehicle)
      state.routePilotStarted = false
      state.routeSafetyStopped = true
    end
    return
  end

  local clearance = nearestForwardObstacle(state.latestData)
  if clearance <= ROUTE_STOP_DISTANCE then
    if not state.routeSafetyStopped then
      stopNativeRouteAI(state.activeVehicle)
      state.routePilotStarted = false
      state.routeSafetyStopped = true
      log("W", "roadmind", "Route pilot paused by sensor obstacle guard")
    end
    return
  end

  if state.routeSafetyStopped then
    if clearance >= ROUTE_RESUME_DISTANCE then
      if startNativeRouteAI(state.activeVehicle) then
        state.routePilotStarted = true
        state.routeSafetyStopped = false
      end
    end
  elseif not state.routePilotStarted then
    if startNativeRouteAI(state.activeVehicle) then
      state.routePilotStarted = true
    end
  end
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

  if updates.lidar == true then
    latest.lidar = processLidar(frame.lidar)
    rememberLidarPoints(latest.lidar)
  end
  if updates.radar == true then latest.radar = processRadar(frame.radar) end
  if updates.ultrasonic == true then latest.ultrasonic = processUltrasonic(frame.ultrasonic) end
  if updates.pose == true then
    latest.cameras = type(frame.cameras) == "table" and frame.cameras or {}
    latest.localization = type(frame.localization) == "table" and frame.localization or nil
    latest.mounts = type(frame.mounts) == "table" and frame.mounts or {}
  end
  if type(frame.vehicleVelocity) == "table" then latest.vehicleVelocity = frame.vehicleVelocity end

  latest.vehicles = detectNearbyVehicles(latest)
  state.latestData = latest
  state.frameAge = 0
  updateRoutePilotSafety()
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

  if state.routePilotEnabled then
    updateRoutePilotSafety()
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
