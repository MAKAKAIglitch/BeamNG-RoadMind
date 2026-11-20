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
  ultrasonicEnabled = true, ultrasonicDensity = "single", ultrasonicHz = 1,
  cameraDetectionEnabled = true,
  autoScanOnVehicleSpawn = false,
  poseHz = 2,
  lidarPersistenceEnabled = false,
  lidarPointCloudLimit = 5000,
  visualizationInertiaEnabled = true,
  adaptiveFpsEnabled = false,
  performanceProfilerEnabled = false,
  targetFps = 60,
  mapCameraHeightM = 14,
  mapCameraDistanceM = 20,
  mapCameraLookAheadM = 20,
  mapForwardRangeM = 120,
  mapRadiusM = 500,
  mapMaxPolygons = 3000,
  mapRenderFps = 30,
  speedAdaptiveRateEnabled = true,
  adasEnabled = false,
  adasFCWEnabled = true,
  adasAEBEnabled = true,
  adasACCEnabled = false,
  adasBSMEnabled = true,
  adasParkingEnabled = true,
  adasLDWEnabled = true,
  driverTakeoverEnabled = true,
  showLidar = true, showRadar = false, showUltrasonic = false,
  showCameras = false, showLocalization = false, showDetectedVehicles = true,
  showCalibration = false
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
  cameraDetectionEnabled = true,
  poseHz = true, showCameras = true, showLocalization = true, showCalibration = true,
  adaptiveFpsEnabled = true, speedAdaptiveRateEnabled = true
}

local MAX_PERSISTENT_LIDAR_POINTS = 10000
local MIN_PERSISTENT_LIDAR_POINTS = 500
local DEFAULT_PERSISTENT_LIDAR_POINTS = 5000
local PERSISTENT_POINT_CELL_SIZE = 0.35
local MAX_LIDAR_RENDER_POINTS = 99
local MAX_LIDAR_FADE_POINTS = 198
local LIDAR_RENDER_SMOOTHING_SECONDS = 0.18
local LIDAR_POINT_FADE_SECONDS = 0.40
local LIDAR_RENDER_POINT_MAX_AGE = 2.25
local LIDAR_STATIC_MATCH_DISTANCE = 0.75
local LIDAR_OBJECT_MATCH_DISTANCE = 8.0
local LIDAR_TRACK_LOCAL_MATCH_DISTANCE = 1.75
local MAX_RENDERED_VEHICLE_BOXES = 64
local VEHICLE_BOX_CORNER_SIGNS = {
  {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
  {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}
}
local VEHICLE_BOX_EDGES = {
  {1, 2}, {2, 3}, {3, 4}, {4, 1},
  {5, 6}, {6, 7}, {7, 8}, {8, 5},
  {1, 5}, {2, 6}, {3, 7}, {4, 8}
}
local WORLD_MODEL_SCHEMA_VERSION = 2
local WORLD_MODEL_OBSERVATION_MAX_AGE = 1.5
local WORLD_MODEL_REGISTRY_MAX_AGE = 1.0
local WORLD_MODEL_OBSTACLE_CELL_SIZE = 0.75
local WORLD_MODEL_MAX_OBSERVATIONS = 512
local WORLD_MODEL_MAX_TRACKS = 128
local VEHICLE_REGISTRY_INTERVAL = 0.15
local VEHICLE_DETECTION_INTERVAL = 0.15
local VEHICLE_SLOW_UPDATE_INTERVAL = 0.30
local VEHICLE_FAST_UPDATE_INTERVAL = 0.10
local MAX_REGISTRY_VEHICLES = 256
local MAX_OCCLUSION_CHECKS_PER_CYCLE = 8
local OCCLUSION_CACHE_MAX_AGE = 0.75
local CAMERA_OCCLUSION_CACHE_MAX_AGE = 0.36
local TRACK_UPDATE_INTERVAL = 0.25
local TRACK_OBSERVATION_MAX_AGE = 0.75
local TRACK_MAX_AGE = 1.5
local TRACK_MAX_MISSED_SCANS = 6
local TRACK_MAX_COUNT = 128
local TRACK_SPATIAL_CELL_SIZE = 16.0
local TRACK_CONFIDENCE_MAX = 0.98
local ROUTE_MAX_SPEED_MPS = 8.0
local ROUTE_LOOKAHEAD_M = 550.0
local ROUTE_REPLAN_DISTANCE = 140.0
local ROUTE_REPLAN_COOLDOWN = 5.0
local ROUTE_STOP_DISTANCE = 12.0
local ROUTE_RESUME_DISTANCE = 20.0
local MAX_ROUTE_INPUT_POINTS = 12
local MAX_ROUTE_PATH_NODES = 256
local MAX_ROUTE_POINT_ROAD_DISTANCE = 100
local ROUTE_DESTINATION_REACHED_M = 12
local MAX_ROUTE_ENDPOINT_OFFSET_M = 60
local ROUTE_MAX_DEVIATION_M = 30
local ROUTE_MAX_LOST_UPDATES = 4
local ROUTE_SAFETY_INTERVAL = 0.10
local PLANNER_INTERVAL = 0.20
local PLANNER_HORIZON_M = 25.0
local PLANNER_SAMPLE_STEP_M = 1.25
local PLANNER_MAX_SAMPLES = 21
local PLANNER_MAX_CANDIDATES = 5
local PLANNER_ROAD_HALF_WIDTH_M = 3.5
local PLANNER_ROAD_EDGE_MARGIN_M = 0.35
local PLANNER_OBSTACLE_MARGIN_M = 0.65
local PLANNER_MAX_OBSTACLES = 128
local PLANNER_MAX_TARGETS = 96
local PLANNER_MAX_DATA_AGE = 1.0
local PLANNER_CANDIDATE_OFFSETS = {-1.25, -0.625, 0, 0.625, 1.25}
local SESSION_STATIC_MAP_MAX_CELLS = 20000
local SESSION_STATIC_MAP_MAX_IDS = 8192
local SESSION_STATIC_MAP_PRUNE_STEP_M = 12
local SESSION_STATIC_MAP_MAX_OUTPUT = 4000
local WORLD_MODEL_UPDATE_INTERVAL = 0.20

local state = {
  activeVehicleId = nil,
  activeVehicle = nil,
  visualizationEnabled = false,
  mapModeEnabled = false,
  mapPlanningEnabled = false,
  mapStartedSensors = false,
  sessionStaticMap = {},
  sessionStaticObservationIds = {},
  sessionStaticObservationOrder = {},
  sessionStaticObservationCursor = 1,
  sessionStaticMapOutputCache = nil,
  sessionStaticMapOutputCenter = nil,
  sessionStaticMapLastRadius = nil,
  sessionStaticMapCount = 0,
  sessionStaticMapRevision = 0,
  sessionStaticMapDistance = 0,
  sessionStaticMapLastPosition = nil,
  sessionStaticMapLastPrunePosition = nil,
  sessionStaticMapGeneration = 1,
  trafficLaneEvidence = {},
  levelMetadataLoaded = false,
  levelMetadataFile = nil,
  levelDirectory = nil,
  levelMetadataName = nil,
  levelRoadCache = nil,
  levelRoadCacheFile = nil,
  levelRoadCacheStatus = "unavailable",
  levelRoadCacheNodeCount = 0,
  levelRoadCacheRevision = 0,
  levelRoadLoadObjects = nil,
  levelRoadLoadCursor = nil,
  levelRoadLoadActive = false,
  mapRoadGeometryCache = nil,
  mapRoadGeometryCacheTime = -math.huge,
  mapRoadGeometryCachePosition = nil,
  mapRoadGeometryCacheForward = nil,
  mapRoadGeometryCacheRange = nil,
  mapRoadGeometryCacheRoadRevision = -1,
  mapRoadGeometryCacheRevision = 0,
  navGraphCache = nil,
  navGraphGeneration = 0,
  levelRoadRules = {rightHandDrive = false, turnOnRed = false},
  levelPOICache = {},
  levelPOIStatus = "unavailable",
  levelPOISource = "",
  levelSignalCache = {status = "unavailable", source = "", items = {}, timestamp = -math.huge},
  settings = nil,
  latestData = nil,
  frameAge = math.huge,
  vehicleGeometry = nil,
  vehicleCalibrationError = nil,
  vehicleRegistry = {},
  vehicleRegistryById = {},
  vehicleRegistrySnapshotValid = false,
  vehicleRegistryTimestamp = nil,
  vehicleIteratorContext = {},
  vehicleRegistryTimer = 0,
  vehicleRegistryGeneration = 0,
  vehicleRegistryTruncated = false,
  vehicleDetectionTimer = 0,
  currentTrafficUpdateInterval = VEHICLE_DETECTION_INTERVAL,
  vehicleCandidates = {},
  occlusionCache = {},
  occlusionSourceCursor = {},
  occlusionCursor = 1,
  simulationTime = 0,
  tracks = {},
  tracksBySourceId = {},
  lidarTrackGrid = {},
  nextTrackId = 1,
  trackCount = 0,
  occlusionRaycastsLastCycle = 0,
  occlusionRaycastsPerSecond = 0,
  lidarPointCloud = {},
  lidarPointCells = {},
  lidarPointWriteIndex = 1,
  lidarPointCount = 0,
  lidarPointCloudCapacity = DEFAULT_PERSISTENT_LIDAR_POINTS,
  lidarRenderPoints = {},
  lidarFadingPoints = {},
  lidarRenderOrigin = nil,
  lidarRenderOriginTarget = nil,
  lidarRenderOriginVelocity = nil,
  lidarRenderScanTimestamp = 0,
  vehicleRenderStates = {},
  visualizationRenderFrame = nil,
  visualizationRenderOverlays = nil,
  visualizationVectorStates = {},
  sensorHoverCandidates = {},
  sensorHover = nil,
  mouseCursorVisible = nil,
  mouseLocked = false,
  visualizationRenderClock = 0,
  visualizationRenderPruneTimer = 0,
  lidarRenderScanSequence = 0,
  worldModel = nil,
  worldModelGeneration = 0,
  worldModelDirty = true,
  worldModelUpdateTimer = 0,
  routePilotEnabled = false,
  routePilotStarted = false,
  routeSafetyStopped = false,
  routePilotVehicleId = nil,
  controllerPathTimestamp = nil,
  routePilotError = nil,
  routeRetryCooldown = 0,
  routePath = {},
  routePathEnd = nil,
  routeInputPoints = {},
  routeCompleted = false,
  routeProgressIndex = 1,
  routeProgressPercent = 0,
  routeProgressTimer = 0,
  routeDistanceToPath = nil,
  routeLostUpdates = 0,
  routeDestinationSnapDistance = nil,
  routeSafetyLastCheckAt = -math.huge,
  localPlanner = {
    status = "idle", reasonCode = "inactive", timestamp = nil,
    candidates = {}, selectedIndex = nil
  },
  roadUpdateTimer = 0,
  roadState = nil,
  performanceFps = nil,
  performanceFpsSamples = 0,
  performanceDecisionTimer = 0,
  performanceLowFpsTimer = 0,
  performanceHighFpsTimer = 0,
  adaptiveQualityTier = 4,
  adasTimer = 0,
  adas = {
    available = false, fcw = false, aeb = false, leftBlindSpot = false,
    rightBlindSpot = false, parkingDistance = nil, laneDeparture = false,
    leadDistance = nil, targetSpeedMps = ROUTE_MAX_SPEED_MPS,
    status = "disabled"
  },
  adasLastBrakeCommand = 0,
  adasLastCruiseCommand = nil
}

local checkTimer = 0

M.performanceProfiler = (function(ownerState, defaults)
  local HISTORY_SECONDS = 15
  local STAGES = {
    "vehicle_job", "vehicle_lidar", "vehicle_radar", "vehicle_ultrasonic",
    "vehicle_pose", "vehicle_detection_zones",
    "sensor_frame", "sensor_lidar", "sensor_radar", "sensor_ultrasonic",
    "vehicle_tracking", "world_model", "planner", "road_graph", "adas",
    "visualization", "debug_draw", "route_safety", "ge_update"
  }
  local clock = type(os) == "table" and type(os.clock) == "function" and os.clock or nil
  local data = {current = {}, history = {}, count = 0, cursor = 1, elapsed = 0, memoryKb = nil}

  local function bucket()
    return {calls = 0, total = 0, peak = 0, last = 0}
  end
  for i = 1, #STAGES do data.current[STAGES[i]] = bucket() end
  for i = 1, HISTORY_SECONDS do
    local sample = {duration = 0, stages = {}}
    for j = 1, #STAGES do sample.stages[STAGES[j]] = bucket() end
    data.history[i] = sample
  end

  local api = {}
  function api.enabled()
    return clock ~= nil and (ownerState.settings or defaults).performanceProfilerEnabled == true
  end
  function api.start()
    if not api.enabled() then return nil end
    return clock()
  end
  function api.record(key, elapsedMs)
    if not api.enabled() then return end
    local target = data.current[key]
    local value = tonumber(elapsedMs)
    if not target or not value or value < 0 or value ~= value then return end
    target.calls = target.calls + 1
    target.total = target.total + value
    target.last = value
    if value > target.peak then target.peak = value end
  end
  function api.stop(key, started)
    if started ~= nil and api.enabled() then
      api.record(key, math.max(0, (clock() - started) * 1000))
    end
  end
  function api.reset()
    data.count, data.cursor, data.elapsed, data.memoryKb = 0, 1, 0, nil
    for i = 1, #STAGES do
      local current = data.current[STAGES[i]]
      current.calls, current.total, current.peak, current.last = 0, 0, 0, 0
    end
    for i = 1, HISTORY_SECONDS do
      local sample = data.history[i]
      sample.duration = 0
      for j = 1, #STAGES do
        local target = sample.stages[STAGES[j]]
        target.calls, target.total, target.peak, target.last = 0, 0, 0, 0
      end
    end
  end
  function api.advance(dt)
    if not api.enabled() then return end
    data.elapsed = data.elapsed + math.max(0, tonumber(dt) or 0)
    if data.elapsed < 1 then return end
    local sample = data.history[data.cursor]
    sample.duration = data.elapsed
    data.elapsed = data.elapsed % 1
    for i = 1, #STAGES do
      local key = STAGES[i]
      local source, target = data.current[key], sample.stages[key]
      target.calls, target.total, target.peak, target.last =
        source.calls, source.total, source.peak, source.last
      source.calls, source.total, source.peak, source.last = 0, 0, 0, 0
    end
    data.cursor = data.cursor % HISTORY_SECONDS + 1
    data.count = math.min(HISTORY_SECONDS, data.count + 1)
    if type(collectgarbage) == "function" then
      local ok, value = pcall(collectgarbage, "count")
      if ok and type(value) == "number" then data.memoryKb = value end
    end
  end
  function api.snapshot()
    local enabled = api.enabled()
    local result = {
      enabled = enabled, available = clock ~= nil,
      historySeconds = data.count,
      memoryKb = data.memoryKb and math.floor(data.memoryKb * 100 + 0.5) / 100 or nil,
      stages = {}
    }
    if not enabled then return result end
    local oldest = ((data.cursor - data.count - 1) % HISTORY_SECONDS) + 1
    for i = 1, #STAGES do
      local key = STAGES[i]
      local calls, total, peak, last, duration = 0, 0, 0, nil, 0
      for offset = 0, data.count - 1 do
        local sample = data.history[(oldest + offset - 1) % HISTORY_SECONDS + 1]
        local item = sample.stages[key]
        calls, total = calls + item.calls, total + item.total
        peak = math.max(peak, item.peak)
        if item.calls > 0 then last = item.last end
        duration = duration + sample.duration
      end
      result.stages[#result.stages + 1] = {
        key = key, calls = calls,
        callsPerSecond = duration > 0 and math.floor(calls / duration * 100 + 0.5) / 100 or 0,
        avgMs = calls > 0 and math.floor(total / calls * 100 + 0.5) / 100 or 0,
        peakMs = math.floor(peak * 100 + 0.5) / 100,
        lastMs = last and math.floor(last * 100 + 0.5) / 100 or nil
      }
    end
    return result
  end
  return api
end)(state, DEFAULT_SETTINGS)


state._roadmindHelpers = state._roadmindHelpers or {}

-- Forward declarations used by the sensor-frame and world-model pipeline.
local buildLidarRenderPoints, buildWorldModel, trimPersistentLidarPoints, updateLocalTrajectoryPlanner

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
  local id = safeCall(vehicle, "getID")
  if id == nil then id = safeCall(vehicle, "getId") end
  if type(id) == "number" or type(id) == "string" then return id end
  return nil
end

local queueVehicleCommand

local function asVec3(value)
  local x, y, z = safeField(value, "x"), safeField(value, "y"), safeField(value, "z")
  if type(x) ~= "number" or type(y) ~= "number" or type(z) ~= "number" then return nil end
  return vec3(x, y, z)
end

local function normalizedVec3(value)
  local vector = asVec3(value)
  if not vector then return nil end
  local length = vector:length()
  if length < 0.00001 then return nil end
  return vector / length
end

state._roadmindHelpers.computeVehicleGeometry = function(vehicle)
  local box = safeCall(vehicle, "getSpawnWorldOOBB")
  if not box then return nil, "getSpawnWorldOOBB is unavailable" end

  local center = asVec3(safeCall(box, "getCenter"))
  local half = asVec3(safeCall(box, "getHalfExtents"))
  local axes = {
    normalizedVec3(safeCall(box, "getAxis", 0)),
    normalizedVec3(safeCall(box, "getAxis", 1)),
    normalizedVec3(safeCall(box, "getAxis", 2))
  }
  if not center or not half or not axes[1] or not axes[2] or not axes[3] then
    return nil, "The vehicle OBB did not return a valid center, axes and half-extents"
  end

  local position = asVec3(safeCall(vehicle, "getPosition"))
  local forward = normalizedVec3(safeCall(vehicle, "getDirectionVector"))
  local up = normalizedVec3(safeCall(vehicle, "getDirectionVectorUp"))
  if not position or not forward or not up then
    return nil, "The vehicle position or orientation is unavailable"
  end
  local right = normalizedVec3(forward:cross(up))
  if not right then return nil, "The vehicle coordinate basis is degenerate" end

  local basis = {forward, right, up}
  local halfValues = {half.x, half.y, half.z}
  local permutations = {
    {1, 2, 3}, {1, 3, 2}, {2, 1, 3},
    {2, 3, 1}, {3, 1, 2}, {3, 2, 1}
  }
  local best, bestScore = nil, -math.huge
  for i = 1, #permutations do
    local p = permutations[i]
    local score =
      math.abs(axes[p[1]]:dot(basis[1])) +
      math.abs(axes[p[2]]:dot(basis[2])) +
      math.abs(axes[p[3]]:dot(basis[3]))
    if score > bestScore then best, bestScore = p, score end
  end
  if not best then return nil, "Could not align the OBB axes with the vehicle coordinate basis" end

  local alignForward = math.abs(axes[best[1]]:dot(forward))
  local alignRight = math.abs(axes[best[2]]:dot(right))
  local alignUp = math.abs(axes[best[3]]:dot(up))
  if alignForward < 0.75 or alignRight < 0.75 or alignUp < 0.75 then
    return nil, "The OBB axes do not align reliably with the live vehicle orientation"
  end

  local centerOffset = center - position
  local centerForward = centerOffset:dot(forward)
  local centerLateral = centerOffset:dot(right)
  local centerUp = centerOffset:dot(up)
  local halfForward = halfValues[best[1]]
  local halfLateral = halfValues[best[2]]
  local halfUp = halfValues[best[3]]

  local profile = {
    schema = 1,
    valid = true,
    source = "getSpawnWorldOOBB",
    vehicleKey = safeCall(vehicle, "getJBeamFilename"),
    forwardMin = centerForward - halfForward,
    forwardMax = centerForward + halfForward,
    lateralMin = centerLateral - halfLateral,
    lateralMax = centerLateral + halfLateral,
    bottom = centerUp - halfUp,
    top = centerUp + halfUp,
    axisAlignment = {
      forward = alignForward,
      lateral = alignRight,
      up = alignUp
    }
  }

  profile.length = profile.forwardMax - profile.forwardMin
  profile.width = profile.lateralMax - profile.lateralMin
  profile.height = profile.top - profile.bottom

  if profile.length < 1.2 or profile.length > 30
      or profile.width < 0.5 or profile.width > 10
      or profile.height < 0.45 or profile.height > 9
      or profile.bottom >= profile.top then
    return nil, "The OBB dimensions are outside the supported safety limits"
  end

  return profile
end

state._roadmindHelpers.vehicleGeometryCommand = function(profile)
  local encoded = "{}"
  if type(jsonEncode) == "function" then
    local ok, value = pcall(jsonEncode, profile or {schema = 1, valid = false})
    if ok and type(value) == "string" then encoded = value end
  end
  return "extensions.load('roadmind_sensors'); roadmind_sensors.setVehicleGeometry("
    .. string.format("%q", encoded) .. ")"
end

state._roadmindHelpers.calibrateVehicleGeometry = function(vehicle)
  if not vehicle then
    state.vehicleGeometry = nil
    state.vehicleCalibrationError = "No active vehicle"
    return false
  end

  local profile, reason = state._roadmindHelpers.computeVehicleGeometry(vehicle)
  state.vehicleGeometry = profile
  state.vehicleCalibrationError = profile and nil or reason

  queueVehicleCommand(vehicle, state._roadmindHelpers.vehicleGeometryCommand(profile))
  if profile then
    log("I", "roadmind", string.format(
      "Vehicle geometry calibrated: %.2f m long, %.2f m wide, %.2f m high (%s)",
      profile.length, profile.width, profile.height, tostring(profile.vehicleKey or "unknown vehicle")
    ))
    return true
  end

  log("W", "roadmind", "Vehicle geometry calibration unavailable; using conservative fallback mounts: "
    .. tostring(reason))
  return false
end

queueVehicleCommand = function(vehicle, command)
  if not vehicle or type(vehicle.queueLuaCommand) ~= "function" then
    return false
  end

  local ok = pcall(vehicle.queueLuaCommand, vehicle, command)
  return ok
end

state._roadmindHelpers.buildForwardRoadPath = function(vehicle)
  if type(map) ~= "table" or type(map.findClosestRoad) ~= "function"
      or type(map.getMap) ~= "function" or type(map.getPath) ~= "function" then
    return nil, nil, "NavGraph route API is unavailable on this map"
  end

  local position = safeCall(vehicle, "getPosition")
  local direction = safeCall(vehicle, "getDirectionVector")
  if not position or not direction then
    return nil, nil, "Could not read vehicle position or heading"
  end

  local flatForward = vec3(direction.x, direction.y, 0)
  if flatForward:length() < 0.0001 then
    return nil, nil, "Vehicle heading is invalid"
  end
  flatForward = flatForward:normalized()

  local targetPosition = position + flatForward * ROUTE_LOOKAHEAD_M
  local okStart, startA, startB = pcall(map.findClosestRoad, position)
  local okTarget, targetA, targetB = pcall(map.findClosestRoad, targetPosition)
  if not okStart or not okTarget or not startA or not startB or not targetA or not targetB then
    return nil, nil, "Could not find connected road nodes near the vehicle and forward target"
  end

  local okMap, mapData = pcall(map.getMap)
  local nodes = okMap and type(mapData) == "table" and mapData.nodes or nil
  if type(nodes) ~= "table" then
    return nil, nil, "The current map has no loaded NavGraph nodes"
  end

  local starts = {startA, startB}
  local targets = {targetA, targetB}
  local bestIds, bestPoints, bestScore = nil, nil, math.huge

  for si = 1, #starts do
    for ti = 1, #targets do
      local fromId, toId = starts[si], targets[ti]
      if fromId ~= toId then
        local okPath, path = pcall(map.getPath, fromId, toId)
        if okPath and type(path) == "table" and #path >= 2 and #path <= 160 then
          local valid, points, length = true, {}, 0
          local previous = nil

          for i = 1, #path do
            local nodeId = path[i]
            local node = nodes[nodeId]
            local nodePosition = node and safeField(node, "pos")
            local nodeX = safeField(nodePosition, "x")
            local nodeY = safeField(nodePosition, "y")
            local nodeZ = safeField(nodePosition, "z")
            if (type(nodeId) ~= "string" and type(nodeId) ~= "number")
                or type(nodeX) ~= "number"
                or type(nodeY) ~= "number"
                or type(nodeZ) ~= "number" then
              valid = false
              break
            end

            if previous then
              local previousNode = nodes[previous]
              local edge = previousNode and previousNode.links and previousNode.links[nodeId]
              if not edge then
                local currentNode = nodes[nodeId]
                edge = currentNode and currentNode.links and currentNode.links[previous]
              end
              if not edge then
                valid = false
                break
              end
              if type(edge) == "table" and edge.oneWay and edge.inNode ~= nil
                  and edge.inNode ~= previous then
                valid = false
                break
              end

              local previousPosition = nodes[previous].pos
              local dx = nodePosition.x - previousPosition.x
              local dy = nodePosition.y - previousPosition.y
              length = length + math.sqrt(dx * dx + dy * dy)
            end

            points[#points + 1] = {
              x = nodeX,
              y = nodeY,
              z = nodeZ
            }
            previous = nodeId
          end

          if valid and #points >= 2 then
            local dx = points[2].x - points[1].x
            local dy = points[2].y - points[1].y
            local headingAlignment = (dx * flatForward.x + dy * flatForward.y)
              / math.max(0.0001, math.sqrt(dx * dx + dy * dy))
            local endDelta = vec3(
              points[#points].x - targetPosition.x,
              points[#points].y - targetPosition.y,
              points[#points].z - targetPosition.z
            )
            local targetError = endDelta:length()

            local startDelta = vec3(
              points[1].x - position.x,
              points[1].y - position.y,
              0
            )
            local startDistance = startDelta:length()
            local startAlignment = startDistance > 0.5
              and startDelta:dot(flatForward) / startDistance or 1

            -- Vehicle AI treats the first NavGraph ID as its next waypoint.
            -- Reject a candidate that would make it drive back to a node behind us.
            if startDistance <= 90 and (startDistance <= 3 or startAlignment >= -0.15)
                and headingAlignment > -0.15 and targetError <= 180 then
              local score = length + targetError * 2
                + startDistance * 0.5 - headingAlignment * 25 - startAlignment * 15
              if score < bestScore then
                bestIds, bestPoints, bestScore = path, points, score
              end
            end
          end
        end
      end
    end
  end

  if not bestIds or not bestPoints then
    return nil, nil, "No valid forward road path could be built from the current NavGraph"
  end

  return bestIds, bestPoints
end


state._roadmindHelpers.buildRoadRouteThroughPoints = function(vehicle, routePoints)
  if not vehicle then return nil, nil, "No active vehicle" end
  if type(map) ~= "table" or type(map.findClosestRoad) ~= "function"
      or type(map.getMap) ~= "function" or type(map.getPath) ~= "function" then
    return nil, nil, "NavGraph route API is unavailable on this map"
  end

  local startPosition = asVec3(safeCall(vehicle, "getPosition"))
  local startHeading = normalizedVec3(safeCall(vehicle, "getDirectionVector"))
  if not startPosition or not startHeading then return nil, nil, "Could not read vehicle position or heading" end
  startHeading = vec3(startHeading.x, startHeading.y, 0)
  if startHeading:length() < 0.0001 then return nil, nil, "Vehicle heading is invalid" end
  startHeading = startHeading:normalized()

  local okMap, mapData = pcall(map.getMap)
  local nodes = okMap and type(mapData) == "table" and mapData.nodes or nil
  if type(nodes) ~= "table" then return nil, nil, "The current map has no loaded NavGraph nodes" end

  local function closestPair(position)
    local ok, a, b, distance = pcall(map.findClosestRoad, position)
    if not ok or a == nil or b == nil or nodes[a] == nil or nodes[b] == nil then
      return nil, "Could not find a road segment for a route point"
    end
    if type(distance) == "number" and distance > MAX_ROUTE_POINT_ROAD_DISTANCE then
      return nil, "A route point is too far from the road network"
    end
    return {a, b}
  end

  local function buildSegment(fromPosition, toPosition, preferredHeading)
    local starts, startError = closestPair(fromPosition)
    if not starts then return nil, nil, startError end
    local targets, targetError = closestPair(toPosition)
    if not targets then return nil, nil, targetError end

    local bestIds, bestPoints, bestScore = nil, nil, math.huge
    for si = 1, 2 do
      for ti = 1, 2 do
        local fromId, toId = starts[si], targets[ti]
        if fromId ~= toId then
          local okPath, path = pcall(map.getPath, fromId, toId)
          if okPath and type(path) == "table" and #path >= 2 and #path <= 160 then
            local valid, points, length = true, {}, 0
            local previousId, previousPosition = nil, nil

            for i = 1, #path do
              local nodeId = path[i]
              local node = nodes[nodeId]
              local nodePosition = node and asVec3(safeField(node, "pos")) or nil
              if (type(nodeId) ~= "string" and type(nodeId) ~= "number") or not nodePosition then
                valid = false
                break
              end

              if previousId ~= nil then
                local previousNode = nodes[previousId]
                local links = previousNode and safeField(previousNode, "links")
                local edge = type(links) == "table" and links[nodeId] or nil
                if not edge then
                  local currentNode = nodes[nodeId]
                  local reverseLinks = currentNode and safeField(currentNode, "links")
                  edge = type(reverseLinks) == "table" and reverseLinks[previousId] or nil
                end
                if not edge then
                  valid = false
                  break
                end
                if type(edge) == "table" and safeField(edge, "oneWay") == true then
                  local incomingId = safeField(edge, "inNode")
                  if incomingId ~= nil and incomingId ~= previousId then
                    valid = false
                    break
                  end
                end

                local dx = nodePosition.x - previousPosition.x
                local dy = nodePosition.y - previousPosition.y
                length = length + math.sqrt(dx * dx + dy * dy)
              end

              points[#points + 1] = {x = nodePosition.x, y = nodePosition.y, z = nodePosition.z}
              previousId = nodeId
              previousPosition = nodePosition
            end

            if valid and #points >= 2 then
              local headingAlignment = 0
              local startDelta = vec3(
                points[1].x - fromPosition.x,
                points[1].y - fromPosition.y,
                0
              )
              local startDistance = startDelta:length()
              local startAlignment = startDistance > 0.5 and preferredHeading
                and startDelta:dot(preferredHeading) / startDistance or 1
              if preferredHeading and startDistance > 3 and startAlignment < -0.25 then
                valid = false
              end
              if preferredHeading then
                local dx = points[2].x - points[1].x
                local dy = points[2].y - points[1].y
                local segmentLength = math.sqrt(dx * dx + dy * dy)
                if segmentLength > 0.1 then
                  headingAlignment = (dx * preferredHeading.x + dy * preferredHeading.y) / segmentLength
                end
                if headingAlignment < -0.25 then valid = false end
              end

              if valid then
                local lastPoint = points[#points]
                local dx, dy, dz = lastPoint.x - toPosition.x, lastPoint.y - toPosition.y, lastPoint.z - toPosition.z
                local endpointError = math.sqrt(dx * dx + dy * dy + dz * dz)
                if endpointError <= MAX_ROUTE_ENDPOINT_OFFSET_M then
                  local score = length + endpointError * 2
                    + startDistance * 0.5 - headingAlignment * 25 - startAlignment * 15
                  if score < bestScore then
                    bestIds, bestPoints, bestScore = path, points, score
                  end
                end
              end
            end
          end
        end
      end
    end

    if not bestIds then return nil, nil, "No connected NavGraph path exists between route points" end
    return bestIds, bestPoints
  end

  local combinedIds, combinedPoints = {}, {}
  local currentPosition, preferredHeading = startPosition, startHeading
  for routeIndex = 1, #routePoints do
    local target = routePoints[routeIndex]
    local targetPosition = vec3(target.x, target.y, target.z)
    local segmentIds, segmentPoints, reason = buildSegment(currentPosition, targetPosition, preferredHeading)
    if not segmentIds then return nil, nil, reason end

    for i = 1, #segmentIds do
      local nodeId = segmentIds[i]
      if #combinedIds == 0 or tostring(combinedIds[#combinedIds]) ~= tostring(nodeId) then
        combinedIds[#combinedIds + 1] = nodeId
        combinedPoints[#combinedPoints + 1] = segmentPoints[i]
        if #combinedIds > MAX_ROUTE_PATH_NODES then
          return nil, nil, "Route exceeds the NavGraph waypoint limit"
        end
      end
    end

    currentPosition = targetPosition
    if #segmentPoints >= 2 then
      local a, b = segmentPoints[#segmentPoints - 1], segmentPoints[#segmentPoints]
      local dx, dy = b.x - a.x, b.y - a.y
      local magnitude = math.sqrt(dx * dx + dy * dy)
      if magnitude > 0.1 then preferredHeading = vec3(dx / magnitude, dy / magnitude, 0) end
    end
  end

  if #combinedIds < 2 then return nil, nil, "The route does not contain enough connected road nodes" end
  return combinedIds, combinedPoints
end

state._roadmindHelpers.sendControllerPath = function(vehicle, planner, force)
  if not vehicle or type(planner) ~= "table" or planner.status ~= "ready"
      or not planner.selectedIndex then
    return false, "No safe trajectory is selected"
  end
  local candidate = planner.candidates and planner.candidates[planner.selectedIndex]
  local path = candidate and candidate.path
  if type(path) ~= "table" or #path < 2 or #path > 32 then
    return false, "Selected trajectory is unavailable or exceeds the controller limit"
  end
  local timestamp = tonumber(planner.timestamp) or state.simulationTime
  if not force and state.controllerPathTimestamp == timestamp then return true end
  if type(jsonEncode) ~= "function" then return false, "JSON encoder is unavailable" end
  local ok, encoded = pcall(jsonEncode, path)
  if not ok or type(encoded) ~= "string" then return false, "Could not encode the selected trajectory" end
  local command =
    "if not roadmind_controller and not roadmind_controller_load_attempted then roadmind_controller_load_attempted = true; extensions.load('roadmind_controller') end; " ..
    "if type(roadmind_controller) == 'table' and type(roadmind_controller.setPath) == 'function' then " ..
    "if not roadmind_controller.setPath(" .. string.format("%q", encoded) .. ") then " ..
    "log('E', 'roadmind', 'Vehicle controller rejected the selected trajectory') end " ..
    "else log('E', 'roadmind', 'Vehicle route controller is unavailable; trajectory update skipped') end"
  if not queueVehicleCommand(vehicle, command) then
    return false, "Could not queue the controller trajectory"
  end
  state.controllerPathTimestamp = timestamp
  return true
end

state._roadmindHelpers.startNativeRouteController = function(vehicle)
  if not vehicle then return false, "No active vehicle" end

  local pathIds, pathPoints, pathError
  if #state.routeInputPoints > 0 then
    pathIds, pathPoints, pathError = state._roadmindHelpers.buildRoadRouteThroughPoints(vehicle, state.routeInputPoints)
  else
    pathIds, pathPoints, pathError = state._roadmindHelpers.buildForwardRoadPath(vehicle)
  end
  if not pathIds then return false, pathError end

  state.routePath = pathPoints
  state.routePathEnd = pathPoints[#pathPoints]
  state.localPlanner.timestamp = nil
  state.controllerPathTimestamp = nil
  state.routeProgressIndex = 1
  state.routeProgressPercent = 0
  state.routeProgressTimer = 0
  state.routeDistanceToPath = nil
  state.routeLostUpdates = 0
  state.routeCompleted = false
  state.routeDestinationSnapDistance = nil
  if #state.routeInputPoints > 0 then
    local destination = state.routeInputPoints[#state.routeInputPoints]
    local endpoint = state.routePathEnd
    if destination and endpoint then
      local dx, dy, dz = endpoint.x - destination.x, endpoint.y - destination.y, endpoint.z - destination.z
      state.routeDestinationSnapDistance = round2(math.sqrt(dx * dx + dy * dy + dz * dz))
    end
  end

  local planner = updateLocalTrajectoryPlanner()
  local queued, reason = state._roadmindHelpers.sendControllerPath(vehicle, planner, true)
  if not queued then return false, reason end
  local takeoverEnabled = state.settings and state.settings.driverTakeoverEnabled ~= false
  local command =
    "if type(roadmind_controller) == 'table' " ..
    "and type(roadmind_controller.setDriverTakeoverEnabled) == 'function' " ..
    "and type(roadmind_controller.setTargetSpeed) == 'function' " ..
    "and type(roadmind_controller.setEnabled) == 'function' then " ..
    "roadmind_controller.setDriverTakeoverEnabled(" .. boolString(takeoverEnabled) .. "); " ..
    "roadmind_controller.setTargetSpeed(" .. string.format("%.2f", ROUTE_MAX_SPEED_MPS) .. "); " ..
    "if not roadmind_controller.setEnabled(true) then log('E', 'roadmind', 'Vehicle controller could not arm') end " ..
    "else log('E', 'roadmind', 'Vehicle route controller is unavailable; route pilot was not armed') end"
  if not queueVehicleCommand(vehicle, command) then
    return false, "Could not queue the autonomous controller start command"
  end
  return true
end

local function stopNativeRouteController(vehicle)
  state.controllerPathTimestamp = nil
  if not vehicle then return false end
  return queueVehicleCommand(vehicle,
    "if not roadmind_controller and not roadmind_controller_load_attempted then roadmind_controller_load_attempted = true; extensions.load('roadmind_controller') end; " ..
    "if roadmind_controller then roadmind_controller.setSafetyStop('route_safety_supervisor') end"
  )
end

state._roadmindHelpers.disableNativeRouteController = function(vehicle)
  state.controllerPathTimestamp = nil
  if not vehicle then return false end
  return queueVehicleCommand(vehicle,
    "if not roadmind_controller and not roadmind_controller_load_attempted then roadmind_controller_load_attempted = true; extensions.load('roadmind_controller') end; " ..
    "if roadmind_controller then roadmind_controller.setEnabled(false) end"
  )
end

state._roadmindHelpers.clearLidarRenderState = function()
  state.lidarRenderPoints = {}
  state.lidarFadingPoints = {}
  state.lidarRenderOrigin = nil
  state.lidarRenderOriginTarget = nil
  state.lidarRenderOriginVelocity = nil
  state.lidarRenderScanTimestamp = 0
  state.lidarRenderScanSequence = 0
end

state._roadmindHelpers.clearPersistentLidarPoints = function()
  state.lidarPointCloud = {}
  state.lidarPointCells = {}
  state.lidarPointWriteIndex = 1
  state.lidarPointCount = 0
  state.lidarPointCloudCapacity = state.settings
    and state.settings.lidarPointCloudLimit or DEFAULT_PERSISTENT_LIDAR_POINTS
  state._roadmindHelpers.clearLidarRenderState()
end

state._roadmindHelpers.resetTrackingState = function(resetClock)
  state.tracks = {}
  state.tracksBySourceId = {}
  state.lidarTrackGrid = {}
  state.trackCount = 0
  state.vehicleRenderStates = {}
  state.worldModel = nil
  state.worldModelDirty = true
  state.worldModelUpdateTimer = 0
  state.occlusionCache = {}
  state.occlusionSourceCursor = {}
  state.occlusionCursor = 1
  state.vehicleCandidates = {}
  if resetClock then state.simulationTime = 0 end
  if state.latestData then
    state.latestData.vehicles = {}
    state.latestData.tracks = {}
  end
end

state._roadmindHelpers.allocateRoadMindTrackId = function()
  local id = string.format("RM-%06d", state.nextTrackId)
  state.nextTrackId = state.nextTrackId + 1
  return id
end

state._roadmindHelpers.copySensorSources = function(sources)
  local result = {}
  for i = 1, #(sources or {}) do
    result[i] = {
      sensor = sources[i].sensor,
      name = sources[i].name
    }
  end
  return result
end

local function vectorToPlainTable(value)
  if not value then return nil end
  return {x = value.x, y = value.y, z = value.z}
end

state._roadmindHelpers.makeTrackObservation = function(candidate)
  if candidate.visibility ~= "visible" then return nil end
  local now = state.simulationTime
  local visibilityTimestamp = tonumber(candidate.visibilityTimestamp)
  local observationTimestamp = tonumber(candidate.observationTimestamp)
  if not visibilityTimestamp or not observationTimestamp then return nil end
  if now - visibilityTimestamp > TRACK_OBSERVATION_MAX_AGE
      or now - observationTimestamp > TRACK_OBSERVATION_MAX_AGE then
    return nil
  end

  local position = asVec3(candidate.position)
  if not position or type(candidate.dimensions) ~= "table"
      or type(candidate.orientation) ~= "table" then return nil end

  return {
    sourceVehicleId = candidate.sourceVehicleId or candidate.id,
    vehicleObject = candidate.vehicleObject,
    centerOffsetLocal = candidate.centerOffsetLocal,
    sensorOrigin = asVec3(candidate.sensorOrigin),
    position = position,
    dimensions = candidate.dimensions,
    orientation = candidate.orientation,
    simulatorVelocity = asVec3(candidate.simulatorVelocity),
    observationTimestamp = observationTimestamp,
    visibilityTimestamp = visibilityTimestamp,
    visibilityAge = math.max(0, now - visibilityTimestamp),
    distance = tonumber(candidate.distance),
    sourceSensors = state._roadmindHelpers.copySensorSources(candidate.sensorSources),
    sourceKind = candidate.sourceKind or "simulator-assisted",
    measurementKind = candidate.measurementKind or "vehicle OBB registry"
  }
end

state._roadmindHelpers.estimateTrackVelocity = function(track, position, timestamp)
  local previousPosition = asVec3(track.position)
  local elapsed = timestamp - (track.lastObservedAt or timestamp)
  if not previousPosition or elapsed < 0.05 or elapsed > TRACK_MAX_AGE then return end

  local rawVelocity = (position - previousPosition) / elapsed
  local speed = rawVelocity:length()
  if speed > 100 then return end

  local previousVelocity = asVec3(track.estimatedVelocity)
  if previousVelocity then
    rawVelocity = previousVelocity * 0.35 + rawVelocity * 0.65
  end
  track.estimatedVelocity = vectorToPlainTable(rawVelocity)
  track.estimatedSpeed = round2(rawVelocity:length())
  track.velocitySource = "timestamped-position-difference"
end

state._roadmindHelpers.updateObjectTracks = function(candidates)
  local now = state.simulationTime
  local seenTracks = {}
  local playerPosition = state.activeVehicle and asVec3(safeCall(state.activeVehicle, "getPosition")) or nil
  local playerVelocity = state.activeVehicle and asVec3(safeCall(state.activeVehicle, "getVelocity")) or nil

  for i = 1, #(candidates or {}) do
    local observation = state._roadmindHelpers.makeTrackObservation(candidates[i])
    if observation then
      local sourceKey = tostring(observation.sourceVehicleId)
      local track = state.tracksBySourceId[sourceKey]

      if not track and state.trackCount < TRACK_MAX_COUNT then
        track = {
          trackId = state._roadmindHelpers.allocateRoadMindTrackId(),
          sourceVehicleId = observation.sourceVehicleId,
          confidence = 0.55,
          missedScans = 0,
          lastObservedAt = nil,
          lastUpdatedAt = now,
          velocitySource = "not-yet-available",
          positionSource = "simulator-obb-center",
          sourceKind = observation.sourceKind,
          measurementKind = observation.measurementKind
        }
        state.tracks[track.trackId] = track
        state.tracksBySourceId[sourceKey] = track
        state.trackCount = state.trackCount + 1
      end

      if track then
        local isFreshObservation =
          observation.observationTimestamp > (track.lastObservedAt or -math.huge) + 0.001

        if isFreshObservation then
          state._roadmindHelpers.estimateTrackVelocity(track, observation.position, observation.observationTimestamp)
          track.position = vectorToPlainTable(observation.position)
          track.orientation = observation.orientation
          track.vehicleObject = observation.vehicleObject
          track.centerOffsetLocal = observation.centerOffsetLocal
          track.dimensions = {
            length = observation.dimensions.length,
            width = observation.dimensions.width,
            height = observation.dimensions.height
          }
          track.lastObservedAt = observation.observationTimestamp
          track.lastVisibilityAt = observation.visibilityTimestamp
          track.distance = observation.distance
          track.confidence = math.min(TRACK_CONFIDENCE_MAX, (track.confidence or 0.55) + 0.08)
          track.missedScans = 0
          track.registryMissingSnapshots = 0
        end

        track.lastUpdatedAt = now
        track.sourceSensors = observation.sourceSensors
        track.sensorOrigin = vectorToPlainTable(observation.sensorOrigin)
        track.simulatorVelocity = vectorToPlainTable(observation.simulatorVelocity)
        track.sourceKind = observation.sourceKind
        track.measurementKind = observation.measurementKind
        track.visibilityAge = observation.visibilityAge
        track.registryGeneration = state.vehicleRegistryGeneration
        seenTracks[track.trackId] = true

        local estimatedVelocity = asVec3(track.estimatedVelocity)
        if estimatedVelocity and playerPosition and playerVelocity then
          local relativeDirection = observation.position - playerPosition
          if relativeDirection:length() > 0.001 then
            track.estimatedRelativeSpeed = round2(
              (estimatedVelocity - playerVelocity):dot(relativeDirection:normalized())
            )
            track.relativeSpeedSource = "timestamped-track-velocity-minus-simulator-player-velocity"
          end
        end
      end
    end
  end

  local result = {}
  for trackId, track in pairs(state.tracks) do
    if not seenTracks[trackId] then
      if state.vehicleRegistrySnapshotValid and not state.vehicleRegistryTruncated
          and (track.lastRegistryGeneration or -1) < state.vehicleRegistryGeneration then
        track.lastRegistryGeneration = state.vehicleRegistryGeneration
        if state.vehicleRegistryById[tostring(track.sourceVehicleId)] then
          track.registryMissingSnapshots = 0
        else
          track.registryMissingSnapshots = (track.registryMissingSnapshots or 0) + 1
        end
      end
      track.missedScans = (track.missedScans or 0) + 1
      track.confidence = math.max(0, (track.confidence or 0) - 0.07)
      track.lastUpdatedAt = now
    else
      track.lastRegistryGeneration = state.vehicleRegistryGeneration
    end

    local dataAge = math.max(0, now - (track.lastObservedAt or now))
    track.dataAge = round2(dataAge)
    track.timestamp = track.lastObservedAt
    track.predictionHorizon = math.min(dataAge, 0.5)

    local position = asVec3(track.position)
    local velocity = asVec3(track.estimatedVelocity)
    if position and velocity then
      track.predictedPosition = vectorToPlainTable(position + velocity * track.predictionHorizon)
    else
      track.predictedPosition = track.position
    end

    if dataAge <= 0.55 and (track.confidence or 0) >= 0.25 then
      track.status = "confirmed"
    elseif dataAge <= 1.0 then
      track.status = "coasting"
    else
      track.status = "stale"
    end

    local expired = dataAge > TRACK_MAX_AGE
      or (track.missedScans or 0) > TRACK_MAX_MISSED_SCANS
      or (track.registryMissingSnapshots or 0) >= 2
      or (track.confidence or 0) < 0.10

    if expired then
      state.tracks[trackId] = nil
      state.tracksBySourceId[tostring(track.sourceVehicleId)] = nil
      state.trackCount = math.max(0, state.trackCount - 1)
    else
      result[#result + 1] = track
    end
  end

  -- Build a coarse spatial index once per tracking update. A LiDAR hit can
  -- query one cell instead of testing every active target OBB.
  local trackGrid = {}
  local cellSize = TRACK_SPATIAL_CELL_SIZE
  for i = 1, #result do
    local track = result[i]
    if track.position and track.dimensions then
      local position = asVec3(track.position)
      local dimensions = track.dimensions
      local length = tonumber(dimensions.length) or 0
      local width = tonumber(dimensions.width) or 0
      local height = tonumber(dimensions.height) or 0
      local radius = math.sqrt(length * length + width * width + height * height) * 0.5 + 0.5
      if position and radius > 0 then
        local minX, maxX = math.floor((position.x - radius) / cellSize), math.floor((position.x + radius) / cellSize)
        local minY, maxY = math.floor((position.y - radius) / cellSize), math.floor((position.y + radius) / cellSize)
        local minZ, maxZ = math.floor((position.z - radius) / cellSize), math.floor((position.z + radius) / cellSize)
        for x = minX, maxX do
          for y = minY, maxY do
            for z = minZ, maxZ do
              local key = x .. ":" .. y .. ":" .. z
              local bucket = trackGrid[key]
              if not bucket then bucket = {}; trackGrid[key] = bucket end
              bucket[#bucket + 1] = track
            end
          end
        end
      end
    end
  end
  state.lidarTrackGrid = trackGrid

  table.sort(result, function(a, b)
    local origin = playerPosition or vec3(0, 0, 0)
    local aPosition = asVec3(a.position)
    local bPosition = asVec3(b.position)
    local aDistance = aPosition and (aPosition - origin):length() or math.huge
    local bDistance = bPosition and (bPosition - origin):length() or math.huge
    if math.abs(aDistance - bDistance) < 0.001 then
      return a.trackId < b.trackId
    end
    return aDistance < bDistance
  end)
  return result
end

state._roadmindHelpers.getEffectiveSensorSettings = function()
  local base = state.settings or DEFAULT_SETTINGS
  local effective = {}
  for key, value in pairs(base) do effective[key] = value end
  if base.adaptiveFpsEnabled ~= true then return effective end

  local tier = math.max(0, math.min(4, math.floor(tonumber(state.adaptiveQualityTier) or 4)))
  local basePoints = tonumber(base.lidarPointCount) or 9
  local pointScale = {[0] = 0.35, [1] = 0.50, [2] = 0.70, [3] = 0.85, [4] = 1.0}
  local scaledPoints = 9 + (basePoints - 9) * pointScale[tier]
  effective.lidarPointCount = math.max(9,
    math.min(basePoints, 9 + math.floor((scaledPoints - 9) / 4) * 4))

  local baseLayers = base.lidarQuality == "high" and 3
    or (base.lidarQuality == "balanced" and 2 or 1)
  local effectiveLayers = baseLayers
  if baseLayers == 3 then
    effectiveLayers = tier <= 0 and 1 or (tier <= 2 and 2 or 3)
  elseif baseLayers == 2 then
    effectiveLayers = tier <= 1 and 1 or 2
  end
  effective.lidarQuality = effectiveLayers == 1 and "low"
    or (effectiveLayers == 2 and "balanced" or "high")

  local function adaptiveRate(baseRate)
    if baseRate == 4 then
      return tier <= 0 and 1 or (tier <= 2 and 2 or 4)
    end
    if baseRate == 2 then return tier <= 0 and 1 or 2 end
    return 1
  end
  effective.lidarHz = adaptiveRate(base.lidarHz)
  effective.radarHz = adaptiveRate(base.radarHz)
  effective.ultrasonicHz = adaptiveRate(base.ultrasonicHz)

  if base.radarDensity == "high" then
    effective.radarDensity = tier <= 1 and "low" or (tier <= 3 and "medium" or "high")
  elseif base.radarDensity == "medium" then
    effective.radarDensity = tier <= 1 and "low" or "medium"
  end
  if base.ultrasonicDensity == "wide" then
    effective.ultrasonicDensity = tier <= 2 and "single" or "wide"
  end
  if base.poseHz == 5 then
    effective.poseHz = tier <= 1 and 2 or (tier <= 3 and 4 or 5)
  elseif base.poseHz == 4 then
    effective.poseHz = tier <= 1 and 2 or 4
  end
  return effective
end

state._roadmindHelpers.vehicleSettingsCommand = function()
  local encoded = "{}"
  if type(jsonEncode) == "function" then
    local ok, value = pcall(jsonEncode, state._roadmindHelpers.getEffectiveSensorSettings())
    if ok and type(value) == "string" then encoded = value end
  end
  return "roadmind_sensors.setSettings(" .. string.format("%q", encoded) .. "); "
end

state._roadmindHelpers.setVehicleSensorSettings = function(vehicle)
  if not vehicle then return end
  local command = "extensions.load('roadmind_sensors'); " .. state._roadmindHelpers.vehicleSettingsCommand()
  queueVehicleCommand(vehicle, command)
end

state._roadmindHelpers.setVehicleSensorState = function(vehicle, enabled, vehicleId)
  if not vehicle then return end
  local idArgument = type(vehicleId) == "number" and string.format("%.0f", vehicleId) or "nil"
  local command = "extensions.load('roadmind_sensors'); "
  if enabled then command = command .. state._roadmindHelpers.vehicleSettingsCommand() end
  command = command .. "roadmind_sensors.setEnabled(" .. boolString(enabled) .. ", " .. idArgument .. ")"
  queueVehicleCommand(vehicle, command)
end

state._roadmindHelpers.getPlayerVehicle = function()
  if not be or type(be.getPlayerVehicle) ~= "function" then
    return nil
  end

  local ok, vehicle = pcall(be.getPlayerVehicle, be, 0)
  if ok then
    return vehicle
  end

  return nil
end

state._roadmindHelpers.resetFrame = function()
  state.latestData = nil
  state.frameAge = math.huge
  state.worldModel = nil
  state.worldModelDirty = true
  state.worldModelUpdateTimer = 0
  state.visualizationRenderFrame = nil
  state.visualizationRenderOverlays = nil
  state.visualizationVectorStates = {}
  state.visualizationRenderClock = 0
  state.visualizationRenderPruneTimer = 0
  state._roadmindHelpers.clearLidarRenderState()
end

state._roadmindHelpers.detachPreviousVehicle = function()
  if state.activeVehicle then
    if (state.adasLastBrakeCommand or 0) > 0 then
      queueVehicleCommand(state.activeVehicle,
        "if not roadmind_controller and not roadmind_controller_load_attempted then roadmind_controller_load_attempted = true; extensions.load('roadmind_controller') end; " ..
        "if roadmind_controller then roadmind_controller.setAebBrake(0) end")
      state.adasLastBrakeCommand = 0
    end
    if state.routePilotEnabled then
      stopNativeRouteController(state.activeVehicle)
      state._roadmindHelpers.disableNativeRouteController(state.activeVehicle)
    end
    queueVehicleCommand(
      state.activeVehicle,
      "if roadmind_sensors then roadmind_sensors.setEnabled(false, " .. tostring(state.activeVehicleId) .. ") end"
    )
  end

  state.routePilotEnabled = false
  state.routePilotStarted = false
  state.routeSafetyStopped = false
  state.adasTimer = 0
  state.adasLastBrakeCommand = 0
  state.adasLastCruiseCommand = nil
  state.adas = {
    available = false, fcw = false, aeb = false, leftBlindSpot = false,
    rightBlindSpot = false, parkingDistance = nil, laneDeparture = false,
    leadDistance = nil, targetSpeedMps = ROUTE_MAX_SPEED_MPS,
    status = state.settings and state.settings.adasEnabled == true
      and "waiting_for_sensors" or "disabled"
  }
  state.routePilotVehicleId = nil
  state.routePilotError = nil
  state.routeRetryCooldown = 0
  state.routeSafetyLastCheckAt = -math.huge
  state.routePath = {}
  state.routePathEnd = nil
  state.localPlanner = {
    status = "idle", reasonCode = "inactive", timestamp = nil,
    candidates = {}, selectedIndex = nil
  }
  state.vehicleGeometry = nil
  state.vehicleCalibrationError = nil
  state._roadmindHelpers.resetTrackingState(true)
  state.vehicleRegistry = {}
  state.vehicleRegistryById = {}
  state.vehicleRegistrySnapshotValid = false
  state.vehicleRegistryTimestamp = nil
  state.vehicleRegistryTimer = 0
  state.vehicleRegistryGeneration = 0
  state.vehicleRegistryTruncated = false
  state.vehicleDetectionTimer = 0
  state.activeVehicle = nil
  state.activeVehicleId = nil
  state.vehicleRegistry = {}
  state.vehicleRegistryTimer = 0
  state.vehicleRegistryGeneration = 0
  state.vehicleRegistryTruncated = false
  state.vehicleDetectionTimer = 0
  state.vehicleCandidates = {}
  state.occlusionRaycastsLastCycle = 0
  state.occlusionRaycastsPerSecond = 0
  state._roadmindHelpers.clearPersistentLidarPoints()
  state._roadmindHelpers.resetFrame()
end

state._roadmindHelpers.attachVehicle = function(vehicle, id)
  state.activeVehicle = vehicle
  state.activeVehicleId = id
  state._roadmindHelpers.resetFrame()
  state._roadmindHelpers.calibrateVehicleGeometry(vehicle)
  state._roadmindHelpers.setVehicleSensorState(vehicle, state.visualizationEnabled, id)
end

state._roadmindHelpers.syncPlayerVehicle = function(allowStartOnChange)
  local vehicle = state._roadmindHelpers.getPlayerVehicle()
  local id = getVehicleId(vehicle)

  if id == state.activeVehicleId then
    return
  end

  local hadPreviousVehicle = state.activeVehicleId ~= nil
  local wasScanning = state.visualizationEnabled
  local autoResume = state.settings and state.settings.autoScanOnVehicleSpawn == true

  -- A manually requested start is always allowed. Otherwise, a running
  -- sensor suite only follows a replacement player vehicle when the opt-in
  -- setting is enabled. Traffic vehicles are never given their own scanners.
  if hadPreviousVehicle and wasScanning and not autoResume and allowStartOnChange ~= true then
    state.visualizationEnabled = false
  end

  state._roadmindHelpers.detachPreviousVehicle()

  if vehicle and id then
    state._roadmindHelpers.attachVehicle(vehicle, id)
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

local SENSOR_HOVER_KIND_LABELS = {
  camera = {ru = "Камера", en = "Camera"},
  lidar = {ru = "LiDAR", en = "LiDAR"},
  radar = {ru = "Радар", en = "Radar"},
  ultrasonic = {ru = "Ультразвуковой датчик", en = "Ultrasonic"}
}

local SENSOR_HOVER_NAME_RU = {
  ["Forward LiDAR"] = "Передний LiDAR",
  ["Front narrow"] = "Передняя узкоугольная",
  ["Front wide"] = "Передняя широкоугольная",
  ["Left side"] = "Левая боковая",
  ["Right side"] = "Правая боковая",
  ["Rear wide"] = "Задняя широкоугольная",
  ["Rear left"] = "Задняя левая",
  ["Rear right"] = "Задняя правая",
  ["Front long-range"] = "Передний дальнобойный",
  ["Front left corner"] = "Передний левый угловой",
  ["Front right corner"] = "Передний правый угловой",
  ["Rear left corner"] = "Задний левый угловой",
  ["Rear right corner"] = "Задний правый угловой",
  ["Front left outer"] = "Передний левый внешний",
  ["Front left inner"] = "Передний левый внутренний",
  ["Front right inner"] = "Передний правый внутренний",
  ["Front right outer"] = "Передний правый внешний",
  ["Rear left outer"] = "Задний левый внешний",
  ["Rear left inner"] = "Задний левый внутренний",
  ["Rear right inner"] = "Задний правый внутренний",
  ["Rear right outer"] = "Задний правый внешний",
  ["Front left side"] = "Передний левый боковой",
  ["Front right side"] = "Передний правый боковой",
  ["Rear left side"] = "Задний левый боковой",
  ["Rear right side"] = "Задний правый боковой"
}

local function sensorHoverLabel(kind, name)
  local language = state._roadmindHelpers.getGameLanguage()
  local russian = type(language) == "string" and language:sub(1, 2) == "ru"
  local kindLabels = SENSOR_HOVER_KIND_LABELS[kind] or {ru = "Датчик", en = "Sensor"}
  local typeLabel = russian and kindLabels.ru or kindLabels.en
  local nameLabel = tostring(name or "")
  if russian then nameLabel = SENSOR_HOVER_NAME_RU[nameLabel] or nameLabel end
  if nameLabel == "" then return typeLabel end
  return typeLabel .. " · " .. nameLabel
end

local function nextSensorHoverCandidate(candidates)
  local index = (candidates.count or 0) + 1
  candidates.count = index
  local candidate = candidates[index]
  if not candidate then
    candidate = {}
    candidates[index] = candidate
  end
  return candidate
end

local function addSensorHoverPoint(candidates, position, kind, name, radius)
  local point = asVec3(position)
  if not point then return end
  local candidate = nextSensorHoverCandidate(candidates)
  candidate.kind = kind
  candidate.name = name
  candidate.a = point
  candidate.b = nil
  candidate.radius = radius or 0.10
end

local function addSensorHoverSegment(candidates, a, b, kind, name, radius)
  if not a or not b then return end
  local candidate = nextSensorHoverCandidate(candidates)
  candidate.kind = kind
  candidate.name = name
  candidate.a = a
  candidate.b = b
  candidate.radius = radius or 0.075
end

local function sensorHoverDistance(rayOrigin, rayDirection, candidate)
  if not candidate.b then
    local depth = (candidate.a - rayOrigin):dot(rayDirection)
    if depth <= 0 then return nil end
    local nearestRayPoint = rayOrigin + rayDirection * depth
    return (candidate.a - nearestRayPoint):length(), depth, candidate.a
  end

  local segment = candidate.b - candidate.a
  local lengthSquared = segment:dot(segment)
  if lengthSquared < 0.000001 then
    local depth = (candidate.a - rayOrigin):dot(rayDirection)
    if depth <= 0 then return nil end
    local nearestRayPoint = rayOrigin + rayDirection * depth
    return (candidate.a - nearestRayPoint):length(), depth, candidate.a
  end

  local relative = rayOrigin - candidate.a
  local directionSegment = rayDirection:dot(segment)
  local directionRelative = rayDirection:dot(relative)
  local segmentRelative = segment:dot(relative)
  local denominator = lengthSquared - directionSegment * directionSegment
  local segmentT
  if math.abs(denominator) > 0.000001 then
    segmentT = (segmentRelative - directionSegment * directionRelative) / denominator
  else
    segmentT = segmentRelative / lengthSquared
  end
  segmentT = math.max(0, math.min(1, segmentT))

  local nearestSegmentPoint = candidate.a + segment * segmentT
  local depth = (nearestSegmentPoint - rayOrigin):dot(rayDirection)
  if depth <= 0 then return nil end
  local nearestRayPoint = rayOrigin + rayDirection * depth
  return (nearestSegmentPoint - nearestRayPoint):length(), depth, nearestSegmentPoint
end

local function findHoveredSensor(candidates)
  if state.mouseCursorVisible == false or state.mouseLocked == true then return nil end
  if type(getCameraMouseRay) ~= "function" then return nil end

  local imgui = rawget(_G, "ui_imgui")
  if imgui and type(imgui.GetIO) == "function" then
    local ok, io = pcall(imgui.GetIO)
    if ok and io and io.WantCaptureMouse == true then return nil end
  end

  local ok, ray = pcall(getCameraMouseRay)
  if not ok or not ray then return nil end
  local origin = asVec3(ray.pos)
  local direction = normalizedVec3(ray.dir)
  if not origin or not direction then return nil end

  local best = nil
  local bestScore = math.huge
  for i = 1, (candidates.count or #candidates) do
    local candidate = candidates[i]
    local distance, depth, position = sensorHoverDistance(origin, direction, candidate)
    if distance and depth then
      -- Scale the pick tolerance with distance so thin frustum edges remain
      -- easy to hover without making the whole volume selectable.
      local tolerance = math.max(candidate.radius or 0.075, depth * 0.0065)
      if distance <= tolerance then
        local score = distance / tolerance + depth * 0.000001
        if score < bestScore then
          bestScore = score
          best = {kind = candidate.kind, name = candidate.name, position = position}
        end
      end
    end
  end
  return best
end

local function drawSensorHoverTooltip(drawer, candidates)
  state.sensorHover = findHoveredSensor(candidates)
  local hover = state.sensorHover
  if not hover or not drawer or type(drawer.drawTextAdvanced) ~= "function" then return end

  local position = hover.position + vec3(0, 0, 0.12)
  pcall(
    drawer.drawTextAdvanced,
    drawer,
    position:toPoint3F(),
    String(sensorHoverLabel(hover.kind, hover.name)),
    ColorF(1.0, 1.0, 1.0, 1.0),
    true,
    false,
    ColorI(10, 17, 30, 225)
  )
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

local function drawLidar(drawer, lidar, hoverCandidates)
  if type(lidar) ~= "table" then return end
  local origin = toPoint3F(lidar.origin)
  if not origin or type(lidar.rays) ~= "table" then return end

  local beamColor = ColorF(0.10, 0.76, 1.0, 0.72)
  drawSphere(drawer, origin, 0.075, beamColor)
  addSensorHoverPoint(hoverCandidates, origin, "lidar", "Forward LiDAR", 0.15)
  for i = 1, #lidar.rays do
    local ray = lidar.rays[i]
    if ray.hit == true then
      local renderPoint = state.lidarRenderPoints[tonumber(ray.rayIndex)]
      local hitPoint = toPoint3F(renderPoint and renderPoint.renderPosition or ray)
      if hitPoint then
        local measurementOrigin = toPoint3F(ray.origin) or origin
        drawLine(drawer, measurementOrigin, hitPoint, beamColor)
        addSensorHoverSegment(
          hoverCandidates,
          vec3(measurementOrigin),
          vec3(hitPoint),
          "lidar",
          "Forward LiDAR"
        )
      end
    end
  end
end

local function drawLidarRenderPoints(drawer)
  for _, point in pairs(state.lidarRenderPoints) do
    local position = toPoint3F(point.renderPosition)
    if position then
      local alpha = math.max(0.08, math.min(1, point.opacity or 1))
      drawSphere(drawer, position, 0.06, ColorF(1.0, 0.45, 0.08, alpha))
    end
  end

  for i = 1, #state.lidarFadingPoints do
    local point = state.lidarFadingPoints[i]
    local position = toPoint3F(point.position)
    if position then
      local alpha = math.max(0, math.min(1, point.opacity or 0))
      if alpha > 0.01 then drawSphere(drawer, position, 0.055, ColorF(1.0, 0.58, 0.22, alpha)) end
    end
  end
end

local function drawRadar(drawer, radar, hoverCandidates)
  if type(radar) ~= "table" then
    return
  end

  local mounts = radar.mounts or {}
  for i = 1, #mounts do
    local mount = mounts[i]
    local position = toPoint3F(mount.position)
    drawSphere(drawer, position, 0.09, ColorF(0.15, 1.0, 0.35, 1.0))
    addSensorHoverPoint(hoverCandidates, position, "radar", mount.name or "Radar", 0.16)
  end

  local returns = radar.returns or {}
  for i = 1, #returns do
    local hit = returns[i]
    local origin = not hit.renderExpired and toPoint3F(hit.origin) or nil
    local target = not hit.renderExpired and toPoint3F(hit) or nil
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

local function drawUltrasonic(drawer, ultrasonic, hoverCandidates)
  if type(ultrasonic) ~= "table" then
    return
  end

  local mounts = ultrasonic.mounts or {}
  for i = 1, #mounts do
    local mount = mounts[i]
    local position = toPoint3F(mount.position)
    drawSphere(
      drawer,
      position,
      0.065,
      ColorF(0.15, 0.95, 0.95, 1.0)
    )
    addSensorHoverPoint(hoverCandidates, position, "ultrasonic", mount.name or "Ultrasonic", 0.14)
  end

  local returns = ultrasonic.returns or {}
  for i = 1, #returns do
    local hit = returns[i]
    local origin = not hit.renderExpired and toPoint3F(hit.origin) or nil
    local target = not hit.renderExpired and toPoint3F(hit) or nil
    if origin and target then
      drawLine(drawer, origin, target, ColorF(0.10, 1.0, 0.82, 0.95))
      drawSphere(drawer, target, 0.085, ColorF(1.0, 0.88, 0.1, 1.0))
    end
  end
end

local persistentLidarCloudColor = nil

local function drawPersistentLidarCloud(drawer)
  if not state.settings or not state.settings.lidarPersistenceEnabled then return end
  if not persistentLidarCloudColor then
    persistentLidarCloudColor = ColorF(0.95, 0.72, 0.20, 0.82)
  end
  local color = persistentLidarCloudColor
  for i = 1, (state.lidarPointCount or #state.lidarPointCloud) do
    local point = state.lidarPointCloud[i]
    if point then
      -- Persistent cloud points never move after insertion, so keep their
      -- engine Point3F conversion instead of rebuilding it on every draw.
      if not point.renderPoint then point.renderPoint = toPoint3F(point) end
      drawSphere(drawer, point.renderPoint, 0.045, color)
    end
  end
end

local vehicleObbCornerScratch = {}

local function drawVehicleObb(drawer, positionValue, orientation, dimensions, color)
  local center = asVec3(positionValue)
  local forward = orientation and normalizedVec3(orientation.forward) or nil
  local right = orientation and normalizedVec3(orientation.right) or nil
  local up = orientation and normalizedVec3(orientation.up) or nil
  local length = type(dimensions) == "table" and tonumber(dimensions.length) or nil
  local width = type(dimensions) == "table" and tonumber(dimensions.width) or nil
  local height = type(dimensions) == "table" and tonumber(dimensions.height) or nil
  if not center then return end
  if not forward or not right or not up or not length or not width or not height
      or length <= 0 or width <= 0 or height <= 0 then
    drawSphere(drawer, center:toPoint3F(), 0.42, color)
    return
  end
  local corners = vehicleObbCornerScratch
  local centerPoint = center:toPoint3F()
  for i = 1, #VEHICLE_BOX_CORNER_SIGNS do
    local sign = VEHICLE_BOX_CORNER_SIGNS[i]
    local point = center + forward * (sign[1] * length * 0.5)
      + right * (sign[2] * width * 0.5) + up * (sign[3] * height * 0.5)
    corners[i] = point:toPoint3F()
  end
  for i = 1, #VEHICLE_BOX_EDGES do
    local edge = VEHICLE_BOX_EDGES[i]
    drawLine(drawer, corners[edge[1]], corners[edge[2]], color)
  end
  drawSphere(drawer, centerPoint, 0.13, color)
  drawLine(drawer, centerPoint,
    (center + forward * math.min(length * 0.65, 2.8)):toPoint3F(), color)
end

local function drawDetectedVehicles(drawer, candidates, tracks)
  local rendered = 0
  local seen = state.detectedVehicleRenderSeen or {}
  state.detectedVehicleRenderSeen = seen
  for key in pairs(seen) do seen[key] = nil end

  -- A camera track may coast internally for ADAS/history, but it is drawn only
  -- while a matching camera candidate has a fresh, raycast-confirmed view.
  local cameraVisible = state.detectedCameraVisibleById or {}
  state.detectedCameraVisibleById = cameraVisible
  for key in pairs(cameraVisible) do cameraVisible[key] = nil end

  local function hasCameraSource(item)
    if item.sourceKind == "simulator-assisted-camera-visibility" then return true end
    local sources = item.sensorSources or item.sourceSensors or {}
    for i = 1, #sources do
      if sources[i].sensor == "camera" then return true end
    end
    return false
  end

  for i = 1, #(candidates or {}) do
    local candidate = candidates[i]
    if type(candidate) == "table" and hasCameraSource(candidate)
        and candidate.visibility == "visible" then
      local timestamp = tonumber(candidate.visibilityTimestamp)
      local age = timestamp and (state.simulationTime - timestamp) or math.huge
      local sourceId = candidate.sourceVehicleId or candidate.id
      if sourceId ~= nil and age >= -0.05 and age <= CAMERA_OCCLUSION_CACHE_MAX_AGE then
        cameraVisible[tostring(sourceId)] = true
      end
    end
  end

  local function drawCandidate(item, isTrack)
    if rendered >= MAX_RENDERED_VEHICLE_BOXES or type(item) ~= "table" then return end
    if item.visibility == "occluded" then return end
    local sourceId = item.sourceVehicleId or item.id
    local key = tostring(sourceId or item.trackId or "")
    if key == "" or seen[key] then return end

    local cameraSource = hasCameraSource(item)
    if cameraSource then
      -- Unverified camera candidates and stale/coasting camera tracks are not
      -- rendered as if the camera had actually seen the vehicle.
      if not cameraVisible[key] then return end
      if not isTrack and item.visibility ~= "visible" then return end
    end

    local renderState = state.vehicleRenderStates[key]
    local position = renderState and renderState.position or item.position
    local orientation = renderState and renderState.orientation or item.orientation
    local dimensions = renderState and renderState.dimensions or item.dimensions
    if not position then return end
    local itemState = isTrack and item.status or item.visibility
    local color
    if itemState == "visible" or itemState == "confirmed" then
      color = ColorF(0.20, 1.0, 0.38, 0.95)
    elseif itemState == "coasting" then
      color = ColorF(1.0, 0.72, 0.18, 0.80)
    else
      color = ColorF(1.0, 0.78, 0.16, 0.72)
    end
    drawVehicleObb(drawer, position, orientation, dimensions, color)
    local sensorOrigin = toPoint3F(item.sensorOrigin)
    local centerPoint = toPoint3F(position)
    if sensorOrigin and centerPoint then drawLine(drawer, sensorOrigin, centerPoint, color) end
    seen[key] = true
    rendered = rendered + 1
  end
  for i = 1, #(candidates or {}) do
    drawCandidate(candidates[i], false)
    if rendered >= MAX_RENDERED_VEHICLE_BOXES then return end
  end
  for i = 1, #(tracks or {}) do
    drawCandidate(tracks[i], true)
    if rendered >= MAX_RENDERED_VEHICLE_BOXES then return end
  end
end

local function drawRoutePath(drawer, renderPath)
  local path = renderPath or state.routePath
  if not state.routePilotEnabled or type(path) ~= "table" then return end
  local color = ColorF(0.95, 0.58, 0.12, 0.88)
  for i = 1, #path - 1 do
    local a = path[i]
    local b = path[i + 1]
    if a and b then
      drawLine(drawer, toPoint3F(a), toPoint3F(b), color)
    end
  end
end

local function drawLocalPlanner(drawer, renderPlanner)
  local planner = renderPlanner or state.localPlanner
  if not state.routePilotEnabled or type(planner) ~= "table"
      or type(planner.candidates) ~= "table" then return end
  for i = 1, #planner.candidates do
    local candidate = planner.candidates[i]
    local selected = i == planner.selectedIndex and candidate.safe == true
    local color
    if selected then
      color = ColorF(0.12, 1.0, 0.32, 0.95)
    elseif candidate.safe then
      color = ColorF(0.18, 0.62, 1.0, 0.45)
    else
      color = ColorF(1.0, 0.22, 0.14, 0.52)
    end
    local path = candidate.path or {}
    for pointIndex = 1, #path - 1 do
      drawLine(drawer, toPoint3F(path[pointIndex]), toPoint3F(path[pointIndex + 1]), color)
    end
    if selected then
      for pointIndex = 2, #path, 3 do
        drawSphere(drawer, toPoint3F(path[pointIndex]), 0.07, color)
      end
    end
  end
end

local function drawCalibrationGeometry(drawer, geometry, mounts)
  if type(geometry) == "table" and geometry.valid == true
      and type(geometry.corners) == "table" and #geometry.corners == 8 then
    local corners = {}
    for i = 1, 8 do corners[i] = toPoint3F(geometry.corners[i]) end
    local edges = {
      {1, 2}, {2, 3}, {3, 4}, {4, 1},
      {5, 6}, {6, 7}, {7, 8}, {8, 5},
      {1, 5}, {2, 6}, {3, 7}, {4, 8}
    }
    local boundsColor = ColorF(1.0, 0.70, 0.12, 0.9)
    for i = 1, #edges do
      local edge = edges[i]
      drawLine(drawer, corners[edge[1]], corners[edge[2]], boundsColor)
    end
    for i = 1, #corners do
      drawSphere(drawer, corners[i], 0.035, boundsColor)
    end
  end

  if type(geometry) == "table" and type(geometry.axes) == "table" then
    local origin = toPoint3F(geometry.axes.origin)
    drawLine(drawer, origin, toPoint3F(geometry.axes.forward), ColorF(0.2, 1.0, 0.25, 1.0))
    drawLine(drawer, origin, toPoint3F(geometry.axes.right), ColorF(1.0, 0.2, 0.2, 1.0))
    drawLine(drawer, origin, toPoint3F(geometry.axes.up), ColorF(0.25, 0.55, 1.0, 1.0))
  end

  if type(mounts) ~= "table" then return end
  for i = 1, #mounts do
    local mount = mounts[i]
    local position = toPoint3F(mount.position)
    local direction = asVec3(mount.direction)
    local color = ColorF(0.65, 0.8, 1.0, 0.95)
    if mount.kind == "lidar" then
      color = ColorF(0.1, 0.78, 1.0, 1.0)
    elseif mount.kind == "radar" then
      color = ColorF(0.2, 1.0, 0.35, 1.0)
    elseif mount.kind == "ultrasonic" then
      color = ColorF(0.1, 1.0, 0.82, 1.0)
    end
    if position then
      drawSphere(drawer, position, 0.07, color)
      if direction then
        local endpoint = vec3(position.x, position.y, position.z)
          + direction:normalized() * 0.65
        drawLine(drawer, position, endpoint:toPoint3F(), color)
      end
    end
  end
end

local function drawCameraFrustum(drawer, camera, hoverCandidates)
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

  local originVector = vec3(origin.x, origin.y, origin.z)
  local center = originVector + fwd * range
  local farLeftUp = center - right * halfWidth + upVec * halfHeight
  local farRightUp = center + right * halfWidth + upVec * halfHeight
  local farLeftDown = center - right * halfWidth - upVec * halfHeight
  local farRightDown = center + right * halfWidth - upVec * halfHeight

  local name = camera.name or "Camera"
  local edges = {
    {originVector, farLeftUp}, {originVector, farRightUp},
    {originVector, farLeftDown}, {originVector, farRightDown},
    {farLeftUp, farRightUp}, {farRightUp, farRightDown},
    {farRightDown, farLeftDown}, {farLeftDown, farLeftUp}
  }
  addSensorHoverPoint(hoverCandidates, originVector, "camera", name, 0.14)
  for i = 1, #edges do
    addSensorHoverSegment(hoverCandidates, edges[i][1], edges[i][2], "camera", name, 0.085)
  end

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
  local frame = state.visualizationRenderFrame or state.latestData
  local settings = state.settings or DEFAULT_SETTINGS

  if not drawer or not frame or not frame.enabled then
    state.sensorHover = nil
    return
  end

  local hoverCandidates = state.sensorHoverCandidates
  hoverCandidates.count = 0
  if settings.showLidar then
    drawLidar(drawer, frame.lidar, hoverCandidates)
    drawLidarRenderPoints(drawer)
    drawPersistentLidarCloud(drawer)
  end
  if settings.showRadar then drawRadar(drawer, frame.radar, hoverCandidates) end
  if settings.showUltrasonic then drawUltrasonic(drawer, frame.ultrasonic, hoverCandidates) end
  if settings.showDetectedVehicles then
    drawDetectedVehicles(drawer, frame.vehicles or state.vehicleCandidates, frame.tracks or {})
  end

  if settings.showCameras then
    local cameras = frame.cameras or {}
    for i = 1, #cameras do drawCameraFrustum(drawer, cameras[i], hoverCandidates) end
  end

  if settings.showLocalization then drawLocalization(drawer, frame.localization) end
  if settings.showCalibration then
    drawCalibrationGeometry(drawer, frame.geometryDebug, frame.mounts)
    for i = 1, #(frame.mounts or {}) do
      local mount = frame.mounts[i]
      addSensorHoverPoint(
        hoverCandidates,
        mount.position,
        mount.kind or "sensor",
        mount.name or "Sensor",
        0.15
      )
    end
  end
  local renderOverlays = state.visualizationRenderOverlays or {}
  drawRoutePath(drawer, renderOverlays.routePath)
  drawLocalPlanner(drawer, renderOverlays.planner)
  drawSensorHoverTooltip(drawer, hoverCandidates)
end

state._roadmindHelpers.copySettings = function(source)
  local result = {}
  for key, value in pairs(source or {}) do result[key] = value end
  return result
end

state._roadmindHelpers.enumValue = function(value, allowed)
  for i = 1, #allowed do
    if value == allowed[i] then return true end
  end
  return false
end

state._roadmindHelpers.normalizeSettings = function(source)
  source = type(source) == "table" and source or {}
  local result = state._roadmindHelpers.copySettings(DEFAULT_SETTINGS)

  local booleanKeys = {
    "lidarEnabled", "radarEnabled", "ultrasonicEnabled", "cameraDetectionEnabled",
    "autoScanOnVehicleSpawn", "driverTakeoverEnabled",
    "lidarPersistenceEnabled", "visualizationInertiaEnabled",
    "showLidar", "showRadar", "showUltrasonic", "showCameras",
    "showLocalization", "showDetectedVehicles", "showCalibration",
    "performanceProfilerEnabled",
    "adasEnabled", "adasFCWEnabled", "adasAEBEnabled", "adasACCEnabled",
    "adasBSMEnabled", "adasParkingEnabled", "adasLDWEnabled"
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
    if state._roadmindHelpers.enumValue(source[key], allowed) then result[key] = source[key] end
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
  if type(source.targetFps) == "number" and source.targetFps >= 30 and source.targetFps <= 165 then
    result.targetFps = math.floor((source.targetFps - 30) / 5 + 0.5) * 5 + 30
  end
  local mapNumericSettings = {
    mapCameraHeightM = {10, 20, 0.5},
    mapCameraDistanceM = {14, 36, 1},
    mapCameraLookAheadM = {12, 48, 1},
    mapForwardRangeM = {100, 180, 10},
    mapRadiusM = {100, 500, 25},
    mapMaxPolygons = {500, 8000, 500},
    mapRenderFps = {15, 60, 5}
  }
  for key, limits in pairs(mapNumericSettings) do
    local value = tonumber(source[key])
    if value and value >= limits[1] and value <= limits[2] then
      result[key] = limits[1] + math.floor((value - limits[1]) / limits[3] + 0.5) * limits[3]
    end
  end
  if type(source.lidarPointCloudLimit) == "number"
      and source.lidarPointCloudLimit >= MIN_PERSISTENT_LIDAR_POINTS
      and source.lidarPointCloudLimit <= MAX_PERSISTENT_LIDAR_POINTS then
    result.lidarPointCloudLimit =
      math.floor((source.lidarPointCloudLimit - MIN_PERSISTENT_LIDAR_POINTS) / 500 + 0.5) * 500
      + MIN_PERSISTENT_LIDAR_POINTS
  end
  return result
end

state._roadmindHelpers.loadSettings = function()
  local loaded = nil
  if type(jsonReadFile) == "function" then
    local ok, data = pcall(jsonReadFile, SETTINGS_FILE)
    if ok and type(data) == "table" then loaded = data end
  end
  state.settings = state._roadmindHelpers.normalizeSettings(loaded or DEFAULT_SETTINGS)
  if trimPersistentLidarPoints then trimPersistentLidarPoints() end
end

state._roadmindHelpers.saveSettings = function()
  if type(jsonWriteFile) ~= "function" then return false end
  local ok = pcall(jsonWriteFile, SETTINGS_FILE, state.settings, true)
  return ok
end

state._roadmindHelpers.rayCountFor = function(settings)
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

state._roadmindHelpers.vehicleDetectionEnabled = function(settings)
  return settings.cameraDetectionEnabled == true
    or settings.lidarEnabled or settings.radarEnabled or settings.ultrasonicEnabled
end

state._roadmindHelpers.getGameLanguage = function()
  if type(settings) == "table" and type(settings.getValue) == "function" then
    -- BeamNG exposes the selected game/UI language through userLanguage.
    local ok, language = pcall(settings.getValue, "userLanguage")
    if ok and type(language) == "string" and language ~= "" then
      local normalized = string.lower(language):gsub("_", "-")
      if normalized:sub(1, 2) == "ru" or normalized:find("russian", 1, true) then
        return "ru-RU"
      end
      if normalized:sub(1, 2) == "en" or normalized:find("english", 1, true) then
        return "en-US"
      end
    end
  end
  return "en-US"
end

state._roadmindHelpers.getSensorSpeedRateScale = function(settings)
  if settings.speedAdaptiveRateEnabled == false or not state.activeVehicle then return 1 end
  local velocity = asVec3(safeCall(state.activeVehicle, "getVelocity"))
  local speed = velocity and velocity:length() or 0
  if speed < 5 then return 1 / 1.5 end
  if speed >= 25 then return 1 / 0.75 end
  return 1
end

local function settingsResult()
  local settings = state.settings or DEFAULT_SETTINGS
  local effectiveSettings = state._roadmindHelpers.getEffectiveSensorSettings()
  local speedRateScale = state._roadmindHelpers.getSensorSpeedRateScale(settings)
  local trackedObjects = {}
  local tracks = state.latestData and state.latestData.tracks or {}
  local cameraZoneCount = 0
  local cameraCandidateCount = 0
  local cameraVisibleVehicleCount = 0
  local detectionZones = state.latestData and state.latestData.detectionZones
    and state.latestData.detectionZones.zones or {}
  for i = 1, #detectionZones do
    if detectionZones[i].sensor == "camera" then cameraZoneCount = cameraZoneCount + 1 end
  end
  local candidates = state.latestData and state.latestData.vehicles or {}
  for i = 1, #candidates do
    local candidate = candidates[i]
    local cameraSource = false
    for j = 1, #(candidate.sensorSources or {}) do
      if candidate.sensorSources[j].sensor == "camera" then
        cameraSource = true
        break
      end
    end
    if cameraSource then
      cameraCandidateCount = cameraCandidateCount + 1
      if candidate.visibility == "visible" then
        cameraVisibleVehicleCount = cameraVisibleVehicleCount + 1
      end
    end
  end
  for i = 1, math.min(#tracks, 8) do
    local track = tracks[i]
    local sensorNames = {}
    for j = 1, #(track.sourceSensors or {}) do
      sensorNames[#sensorNames + 1] = track.sourceSensors[j].name
    end
    trackedObjects[#trackedObjects + 1] = {
      trackId = track.trackId,
      status = track.status,
      distance = track.distance,
      estimatedSpeedKph = track.estimatedSpeed and round2(track.estimatedSpeed * 3.6) or nil,
      estimatedRelativeSpeedKph = track.estimatedRelativeSpeed
        and round2(track.estimatedRelativeSpeed * 3.6) or nil,
      dataAge = track.dataAge,
      confidencePercent = math.floor((track.confidence or 0) * 100 + 0.5),
      sensors = sensorNames
    }
  end

  return {
    ok = true,
    language = state._roadmindHelpers.getGameLanguage(),
    settings = state._roadmindHelpers.copySettings(settings),
    status = {
      visualizationEnabled = state.visualizationEnabled,
      activeVehicle = state.activeVehicleId ~= nil,
      estimatedRaycastsPerSecond = state.visualizationEnabled
        and (state._roadmindHelpers.rayCountFor(effectiveSettings) * speedRateScale + state.occlusionRaycastsPerSecond) or 0,
      currentFps = state.performanceFps and round2(state.performanceFps) or nil,
      targetFps = settings.targetFps or 60,
      adaptiveFpsEnabled = settings.adaptiveFpsEnabled == true,
      adaptiveQualityTier = settings.adaptiveFpsEnabled == true and state.adaptiveQualityTier or 4,
      effectiveLidarRayCount = (effectiveSettings.lidarPointCount or 9)
        * (effectiveSettings.lidarQuality == "high" and 3
          or (effectiveSettings.lidarQuality == "balanced" and 2 or 1)),
      performanceLimitReached = settings.adaptiveFpsEnabled == true
        and state.adaptiveQualityTier <= 0
        and state.performanceFps ~= nil
        and state.performanceFps < (settings.targetFps or 60) * 0.92,
      speedAdaptiveRateEnabled = settings.speedAdaptiveRateEnabled ~= false,
      profiler = M.performanceProfiler.snapshot(),
      trafficUpdateHz = 1 / math.max(0.01,
        state.currentTrafficUpdateInterval or VEHICLE_DETECTION_INTERVAL),
      vehicleRegistryCount = #state.vehicleRegistry,
      vehicleRegistryGeneration = state.vehicleRegistryGeneration,
      vehicleRegistryTruncated = state.vehicleRegistryTruncated,
      vehicleCandidateCount = #state.vehicleCandidates,
      cameraDetectionEnabled = settings.cameraDetectionEnabled == true,
      cameraZoneCount = cameraZoneCount,
      cameraCandidateCount = cameraCandidateCount,
      cameraVisibleVehicleCount = cameraVisibleVehicleCount,
      visibleVehicleCount = state.latestData and (function()
        local count = 0
        for i = 1, #(state.latestData.vehicles or {}) do
          if state.latestData.vehicles[i].visibility == "visible" then count = count + 1 end
        end
        return count
      end)() or 0,
      unverifiedVehicleCount = state.latestData and (function()
        local count = 0
        for i = 1, #(state.latestData.vehicles or {}) do
          if state.latestData.vehicles[i].visibility == "unverified" then count = count + 1 end
        end
        return count
      end)() or 0,
      occlusionRaycastsPerSecond = state.occlusionRaycastsPerSecond,
      routePilotEnabled = state.routePilotEnabled,
      routePilotStarted = state.routePilotStarted,
      routeSafetyStopped = state.routeSafetyStopped,
      adasStatus = state._roadmindHelpers.copySettings(state.adas or {}),
      adasEnabled = settings.adasEnabled == true,
      routePilotError = state.routePilotError,
      routeWaypointCount = #state.routePath,
      routeInputPointCount = #state.routeInputPoints,
      routeCompleted = state.routeCompleted,
      routeProgressIndex = state.routeProgressIndex,
      routeProgressPercent = math.floor((state.routeProgressPercent or 0) * 10 + 0.5) / 10,
      routeDistanceToPath = state.routeDistanceToPath and round2(state.routeDistanceToPath) or nil,
      routeLostUpdates = state.routeLostUpdates or 0,
      routeDestinationSnapDistance = state.routeDestinationSnapDistance,
      plannerStatus = state.localPlanner and state.localPlanner.status or "idle",
      plannerReasonCode = state.localPlanner and state.localPlanner.reasonCode or "inactive",
      plannerCandidateCount = state.localPlanner and state.localPlanner.candidateCount or 0,
      plannerRejectedCandidateCount = state.localPlanner and state.localPlanner.rejectedCount or 0,
      plannerSelectedScore = state.localPlanner and state.localPlanner.selectedScore or nil,
      plannerMinimumClearance = state.localPlanner and state.localPlanner.minimumClearance or nil,
      plannerMinimumRoadMargin = state.localPlanner and state.localPlanner.minimumRoadMargin or nil,
      plannerSelectedOffset = state.localPlanner and state.localPlanner.selectedOffset or nil,
      plannerHorizon = PLANNER_HORIZON_M,
      roadLocalizationQuality = state.roadState and state.roadState.quality or "unavailable",
      roadDistanceToSegment = state.roadState and state.roadState.distanceToSegment
        and round2(state.roadState.distanceToSegment) or nil,
      roadLateralOffset = state.roadState and state.roadState.lateralOffset
        and round2(state.roadState.lateralOffset) or nil,
      roadHeadingAlignment = state.roadState and state.roadState.headingAlignment
        and round2(state.roadState.headingAlignment) or nil,
      roadSegment = state.roadState and state.roadState.segmentStartNode
        and (state.roadState.segmentStartNode .. " -> " .. state.roadState.segmentEndNode) or nil,
      roadCorridorPointCount = state.roadState and #(state.roadState.corridor or {}) or 0,
      persistentLidarPointCount = state.lidarPointCount or #state.lidarPointCloud,
      persistentLidarPointLimit = settings.lidarPointCloudLimit or DEFAULT_PERSISTENT_LIDAR_POINTS,
      trackedVehicleCount = state.trackCount,
      trackedObjects = trackedObjects,
      coastingTrackCount = state.latestData and (function()
        local count = 0
        for i = 1, #(state.latestData.tracks or {}) do
          if state.latestData.tracks[i].status == "coasting"
              or state.latestData.tracks[i].status == "stale" then count = count + 1 end
        end
        return count
      end)() or 0,
      detectedVehicleCount = state.latestData and (function()
        local count = 0
        for i = 1, #(state.latestData.vehicles or {}) do
          if state.latestData.vehicles[i].visibility == "visible" then count = count + 1 end
        end
        return count
      end)() or 0,
      worldModelSchemaVersion = WORLD_MODEL_SCHEMA_VERSION,
      worldModelGeneration = state.worldModel and state.worldModel.generation or 0,
      worldModelTargetCount = state.worldModel and #(state.worldModel.targets or {}) or 0,
      worldModelObstacleCount = state.worldModel and #(state.worldModel.obstacles or {}) or 0,
      worldModelObservationCount = state.worldModel and (state.worldModel.observationCount or 0) or 0,
      worldModelVehicleObservationCount = state.worldModel
        and #(state.worldModel.observations.vehicleRegistry or {}) or 0,
      worldModelStaleSourceCount = state.worldModel and (state.worldModel.quality.staleSourceCount or 0) or 0,
      worldModelInvalidObservationCount = state.worldModel
        and (state.worldModel.quality.invalidObservationCount or 0) or 0,
      worldModelDroppedObservationCount = state.worldModel
        and (state.worldModel.quality.droppedObservationCount or 0) or 0,
      worldModelConflictingObservationCount = state.worldModel
        and (state.worldModel.quality.conflictingObservationCount or 0) or 0,
      worldModelTruncatedSourceCount = state.worldModel
        and (state.worldModel.quality.truncatedSourceCount or 0) or 0,
      worldModelRegistryEnabled = state.worldModel and state.worldModel.sourceStatus.vehicleRegistry
        and state.worldModel.sourceStatus.vehicleRegistry.enabled or false,
      worldModelRegistryFresh = state.worldModel and state.worldModel.sourceStatus.vehicleRegistry
        and state.worldModel.sourceStatus.vehicleRegistry.fresh or false,
      vehicleCalibration = state.vehicleGeometry and {
        calibrated = true,
        source = state.vehicleGeometry.source,
        vehicleKey = state.vehicleGeometry.vehicleKey,
        length = state.vehicleGeometry.length,
        width = state.vehicleGeometry.width,
        height = state.vehicleGeometry.height
      } or {
        calibrated = false,
        error = state.vehicleCalibrationError or "Using fallback sensor mounts"
      }
    }
  }
end

function M.getSettings()
  if not state.settings then state._roadmindHelpers.loadSettings() end
  if not state.activeVehicle then state._roadmindHelpers.syncPlayerVehicle() end
  return settingsResult()
end

function M.setSetting(key, value)
  if not state.settings then state._roadmindHelpers.loadSettings() end
  if DEFAULT_SETTINGS[key] == nil then
    return {ok = false, error = "Unknown setting"}
  end

  local candidate = state._roadmindHelpers.copySettings(state.settings)
  candidate[key] = value
  if key ~= "preset" and key ~= "performanceProfilerEnabled"
      and not tostring(key):match("^adas") and not tostring(key):match("^map") then
    candidate.preset = "custom"
  end
  local normalized = state._roadmindHelpers.normalizeSettings(candidate)

  if type(DEFAULT_SETTINGS[key]) == "boolean" and type(value) ~= "boolean" then
    return {ok = false, error = "Invalid boolean value"}
  end
  if type(DEFAULT_SETTINGS[key]) == "number" and type(value) ~= "number" then
    return {ok = false, error = "Invalid numeric value"}
  end
  if type(DEFAULT_SETTINGS[key]) == "string" and normalized[key] ~= value then
    return {ok = false, error = "Invalid option"}
  end
  if type(DEFAULT_SETTINGS[key]) == "number" and key ~= "lidarRange"
      and key ~= "lidarPointCount" and key ~= "lidarPointCloudLimit"
      and key ~= "targetFps" and normalized[key] ~= value then
    return {ok = false, error = "Invalid numeric option"}
  end
  if key == "lidarRange" and (value < 30 or value > 160) then
    return {ok = false, error = "LiDAR range must be between 30 and 160 m"}
  end
  if key == "lidarPointCount"
      and (value < 9 or value > 33 or math.abs((value - 9) % 4) > 0.001) then
    return {ok = false, error = "LiDAR point count must be 9–33 in steps of 4"}
  end
  if key == "lidarPointCloudLimit"
      and (value < MIN_PERSISTENT_LIDAR_POINTS or value > MAX_PERSISTENT_LIDAR_POINTS
        or math.abs((value - MIN_PERSISTENT_LIDAR_POINTS) % 500) > 0.001) then
    return {ok = false, error = "LiDAR cloud limit must be 500–10000 in steps of 500"}
  end
  if key == "targetFps"
      and (value < 30 or value > 165 or math.abs((value - 30) % 5) > 0.001) then
    return {ok = false, error = "Target FPS must be 30–165 in steps of 5"}
  end
  state.settings = normalized
  if key == "performanceProfilerEnabled" then M.performanceProfiler.reset() end
  if key == "driverTakeoverEnabled" and state.routePilotEnabled and state.activeVehicle then
    queueVehicleCommand(state.activeVehicle,
      "if not roadmind_controller and not roadmind_controller_load_attempted then roadmind_controller_load_attempted = true; extensions.load('roadmind_controller') end; " ..
      "if roadmind_controller then roadmind_controller.setDriverTakeoverEnabled(" .. boolString(value) .. ") end")
  end
  if key == "adasEnabled" or key == "adasAEBEnabled" then
    if normalized.adasEnabled then
      state.adas = {
        available = state.visualizationEnabled and state.frameAge <= MAX_FRAME_AGE,
        fcw = false, aeb = false, leftBlindSpot = false, rightBlindSpot = false,
        parkingDistance = nil, laneDeparture = false, leadDistance = nil,
        targetSpeedMps = ROUTE_MAX_SPEED_MPS,
        status = "waiting_for_sensors"
      }
    end
    if not normalized.adasEnabled or not normalized.adasAEBEnabled then
      if state.activeVehicle and (state.adasLastBrakeCommand or 0) > 0 then
        queueVehicleCommand(state.activeVehicle,
          "if not roadmind_controller and not roadmind_controller_load_attempted then roadmind_controller_load_attempted = true; extensions.load('roadmind_controller') end; " ..
          "if roadmind_controller then roadmind_controller.setAebBrake(0) end")
      end
      state.adasLastBrakeCommand = 0
      state.adas = {
        available = state.visualizationEnabled and state.frameAge <= MAX_FRAME_AGE,
        fcw = false, aeb = false, leftBlindSpot = false, rightBlindSpot = false,
        parkingDistance = nil, laneDeparture = false, leadDistance = nil,
        targetSpeedMps = ROUTE_MAX_SPEED_MPS,
        status = normalized.adasEnabled and "standby" or "disabled"
      }
    end
  end
  if (key == "adasEnabled" and value == false)
      or (key == "adasACCEnabled" and value == false) then
    state.adasLastCruiseCommand = ROUTE_MAX_SPEED_MPS
    if state.routePilotEnabled and state.activeVehicle then
      queueVehicleCommand(state.activeVehicle,
        "if not roadmind_controller and not roadmind_controller_load_attempted then roadmind_controller_load_attempted = true; extensions.load('roadmind_controller') end; " ..
        "if roadmind_controller then roadmind_controller.setTargetSpeed(" ..
        string.format("%.2f", ROUTE_MAX_SPEED_MPS) .. ") end")
    end
  end
  if key == "adaptiveFpsEnabled" and value == true then
    state.adaptiveQualityTier = 4
    state.performanceLowFpsTimer = 0
    state.performanceHighFpsTimer = 0
  elseif SENSOR_SETTING_KEYS[key] and key ~= "targetFps" then
    state.adaptiveQualityTier = 4
    state.performanceLowFpsTimer = 0
    state.performanceHighFpsTimer = 0
  end
  if key == "lidarPointCloudLimit" and trimPersistentLidarPoints then trimPersistentLidarPoints() end
  state._roadmindHelpers.saveSettings()

  if state.visualizationEnabled and state.activeVehicle and SENSOR_SETTING_KEYS[key] then
    state._roadmindHelpers.setVehicleSensorSettings(state.activeVehicle)
  end
  return settingsResult()
end

function M.setPreset(name)
  if not state.settings then state._roadmindHelpers.loadSettings() end
  local preset = PRESETS[name]
  if not preset then return {ok = false, error = "Unknown performance preset"} end

  local candidate = state._roadmindHelpers.copySettings(state.settings)
  candidate.preset = name
  for key, value in pairs(preset) do candidate[key] = value end
  state.settings = state._roadmindHelpers.normalizeSettings(candidate)
  state.adaptiveQualityTier = 4
  state.performanceLowFpsTimer = 0
  state.performanceHighFpsTimer = 0
  state._roadmindHelpers.saveSettings()

  if state.visualizationEnabled and state.activeVehicle then
    state._roadmindHelpers.setVehicleSensorSettings(state.activeVehicle)
  end
  return settingsResult()
end

function M.setVisualizationEnabled(enabled)
  if not state.settings then state._roadmindHelpers.loadSettings() end
  local wanted = enabled == true

  if wanted then
    local previousId = state.activeVehicleId
    state.visualizationEnabled = true
    state._roadmindHelpers.resetTrackingState(true)
    state._roadmindHelpers.clearLidarRenderState()
    state.vehicleRegistry = {}
    state.vehicleRegistryById = {}
    state.vehicleRegistrySnapshotValid = false
    state.vehicleRegistryTimestamp = nil
    state.vehicleRegistryTimer = 0
    state.vehicleDetectionTimer = 0
    state.currentTrafficUpdateInterval = VEHICLE_DETECTION_INTERVAL
    state.vehicleCandidates = {}
    state.adaptiveQualityTier = 4
    state.performanceLowFpsTimer = 0
    state.performanceHighFpsTimer = 0
    state.performanceDecisionTimer = 0
    state.occlusionRaycastsLastCycle = 0
    state.occlusionRaycastsPerSecond = 0
    state._roadmindHelpers.resetFrame()
    state._roadmindHelpers.syncPlayerVehicle(true)
    if state.activeVehicle and state.activeVehicleId == previousId then
      state._roadmindHelpers.setVehicleSensorState(state.activeVehicle, true, state.activeVehicleId)
    end
    if not state.activeVehicle then
      state.visualizationEnabled = false
      return {ok = false, error = "No active player vehicle"}
    end
  else
    if state.activeVehicle and (state.adasLastBrakeCommand or 0) > 0 then
      queueVehicleCommand(state.activeVehicle,
        "if not roadmind_controller and not roadmind_controller_load_attempted then roadmind_controller_load_attempted = true; extensions.load('roadmind_controller') end; " ..
        "if roadmind_controller then roadmind_controller.setAebBrake(0) end")
      state.adasLastBrakeCommand = 0
    end
    if state.routePilotEnabled and state.activeVehicle then
      stopNativeRouteController(state.activeVehicle)
      state._roadmindHelpers.disableNativeRouteController(state.activeVehicle)
    end
    state.adasTimer = 0
    state.adasLastCruiseCommand = nil
    state.adas = {
      available = false, fcw = false, aeb = false, leftBlindSpot = false,
      rightBlindSpot = false, parkingDistance = nil, laneDeparture = false,
      leadDistance = nil, targetSpeedMps = ROUTE_MAX_SPEED_MPS,
      status = state.settings.adasEnabled == true and "waiting_for_sensors" or "disabled"
    }
    state.vehicleRegistry = {}
    state.vehicleRegistryById = {}
    state.vehicleRegistrySnapshotValid = false
    state.vehicleRegistryTimestamp = nil
    state.vehicleRegistryTimer = 0
    state.vehicleRegistryTruncated = false
    state.vehicleDetectionTimer = 0
    state.vehicleCandidates = {}
    state._roadmindHelpers.resetTrackingState(false)
    state._roadmindHelpers.clearLidarRenderState()
    state.occlusionRaycastsLastCycle = 0
    state.occlusionRaycastsPerSecond = 0
    state.routePilotEnabled = false
    state.routePilotStarted = false
    state.routeSafetyStopped = false
    state.routePilotVehicleId = nil
    state.routePilotError = nil
    state.routeRetryCooldown = 0
    state.routeSafetyLastCheckAt = -math.huge
    state.routePath = {}
    state.routePathEnd = nil
    state.localPlanner = {
      status = "idle", reasonCode = "inactive", timestamp = nil,
      candidates = {}, selectedIndex = nil
    }
    state.visualizationEnabled = false
    if state.activeVehicle then
      state._roadmindHelpers.setVehicleSensorState(state.activeVehicle, false, state.activeVehicleId)
    end
    state._roadmindHelpers.resetFrame()
  end

  return settingsResult()
end


function M.setMapEnabled(enabled)
  if not state.settings then state._roadmindHelpers.loadSettings() end
  local wanted = enabled == true

  if wanted then
    state.mapModeEnabled = true
    state.mapStartedSensors = false
    state.mapPlanningEnabled = state.visualizationEnabled == true
    return {
      ok = true,
      enabled = true,
      scanning = state.visualizationEnabled == true,
      startedSensors = false
    }
  end

  -- The HUD widget is a view, not the owner of sensor lifecycle.
  local stopSensors = false
  state.mapModeEnabled = false
  state.mapPlanningEnabled = false
  state.mapStartedSensors = false
  if not state.routePilotEnabled then
    state.localPlanner = {
      status = "idle", reasonCode = "inactive", timestamp = nil,
      candidates = {}, selectedIndex = nil
    }
  end
  if stopSensors and state.visualizationEnabled then
    return M.setVisualizationEnabled(false)
  end
  return {ok = true, enabled = false, scanning = state.visualizationEnabled}
end


function M.setRoutePilotEnabled(enabled)
  if not state.settings then state._roadmindHelpers.loadSettings() end

  if enabled ~= true then
    if state.routePilotEnabled and state.activeVehicle then
      stopNativeRouteController(state.activeVehicle)
      state._roadmindHelpers.disableNativeRouteController(state.activeVehicle)
    end
    state.routePilotEnabled = false
    state.routePilotStarted = false
    state.routeSafetyStopped = false
    state.routePilotVehicleId = nil
    state.routePilotError = nil
    state.routeRetryCooldown = 0
    state.routeSafetyLastCheckAt = -math.huge
    state.routePath = {}
    state.routePathEnd = nil
    state.localPlanner = {
      status = "idle", reasonCode = "inactive", timestamp = nil,
      candidates = {}, selectedIndex = nil
    }
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
  local candidate = state._roadmindHelpers.copySettings(state.settings)
  for key, value in pairs(PRESETS.balanced) do candidate[key] = value end
  candidate.preset = "balanced"
  candidate.showLidar = true
  candidate.showRadar = true
  candidate.showUltrasonic = true
  candidate.showCameras = true
  candidate.showLocalization = true
  candidate.showDetectedVehicles = true
  state.settings = state._roadmindHelpers.normalizeSettings(candidate)
  state._roadmindHelpers.saveSettings()
  state._roadmindHelpers.setVehicleSensorSettings(state.activeVehicle)

  state.routePilotEnabled = true
  state.routePilotStarted = false
  state.routeSafetyStopped = true
  state.routePilotVehicleId = state.activeVehicleId
  state.routePilotError = "Waiting for fresh sensor data"
  state.routeRetryCooldown = 0
  state.routeSafetyLastCheckAt = -math.huge
  state.routePath = {}
  state.routePathEnd = nil
  state.localPlanner = {
    status = "waiting", reasonCode = "sensor_stale", timestamp = nil,
    candidates = {}, selectedIndex = nil
  }
  state.routeProgressIndex = 1
  state.routeProgressPercent = 0
  state.routeProgressTimer = 0
  state.routeDistanceToPath = nil
  state.routeCompleted = false
  state._roadmindHelpers.resetFrame()

  log("I", "roadmind", "Experimental road-network route pilot armed; waiting for fresh sensor data")
  return settingsResult()
end


function M.onDriverTakeover()
  if not state.routePilotEnabled then return settingsResult() end
  state.routePilotEnabled = false
  state.routePilotStarted = false
  state.routeSafetyStopped = false
  state.routePilotVehicleId = nil
  state.routePilotError = "Driver took control"
  state.routeRetryCooldown = 0
  state.controllerPathTimestamp = nil
  state.localPlanner = {
    status = "idle", reasonCode = "driver_takeover", timestamp = state.simulationTime,
    candidates = {}, selectedIndex = nil
  }
  log("I", "roadmind", "Manual driver input detected; route pilot handed over control")
  return settingsResult()
end

function M.setRoutePoints(encodedPoints)
  if type(encodedPoints) ~= "string" or type(jsonDecode) ~= "function" then
    return {ok = false, error = "Route points must be supplied as JSON"}
  end
  local okDecode, decoded = pcall(jsonDecode, encodedPoints)
  if not okDecode or type(decoded) ~= "table" or #decoded < 1 or #decoded > MAX_ROUTE_INPUT_POINTS then
    return {ok = false, error = "Route requires 1-12 ordered XYZ points"}
  end

  local points = {}
  for i = 1, #decoded do
    local item = decoded[i]
    local x, y, z = safeField(item, "x"), safeField(item, "y"), safeField(item, "z")
    if type(x) ~= "number" or type(y) ~= "number" or type(z) ~= "number"
        or x ~= x or y ~= y or z ~= z
        or math.abs(x) > 100000 or math.abs(y) > 100000 or math.abs(z) > 100000
        or x == math.huge or x == -math.huge
        or y == math.huge or y == -math.huge
        or z == math.huge or z == -math.huge then
      return {ok = false, error = "Route points must contain finite world X, Y and Z coordinates"}
    end
    points[#points + 1] = {x = x, y = y, z = z}
  end

  if not state.activeVehicle then state._roadmindHelpers.syncPlayerVehicle() end
  if not state.activeVehicle then return {ok = false, error = "No active player vehicle"} end

  local pathIds, pathPoints, reason = state._roadmindHelpers.buildRoadRouteThroughPoints(state.activeVehicle, points)
  if not pathIds then return {ok = false, error = reason or "Could not build the requested route"} end

  if state.routePilotEnabled then
    stopNativeRouteController(state.activeVehicle)
    state.routePilotStarted = false
    state.routeSafetyStopped = true
    state.routePilotError = "Waiting for fresh sensor data"
  end

  state.routeInputPoints = points
  state.routePath = pathPoints
  state.routePathEnd = pathPoints[#pathPoints]
  state.localPlanner = {
    status = "waiting", reasonCode = "sensor_stale", timestamp = nil,
    candidates = {}, selectedIndex = nil
  }
  state.routeProgressIndex = 1
  state.routeProgressPercent = 0
  state.routeProgressTimer = 0
  state.routeDistanceToPath = nil
  state.routeLostUpdates = 0
  state.routeCompleted = false
  state.routeDestinationSnapDistance = nil
  local destination, endpoint = points[#points], state.routePathEnd
  if destination and endpoint then
    local dx, dy, dz = endpoint.x - destination.x, endpoint.y - destination.y, endpoint.z - destination.z
    state.routeDestinationSnapDistance = round2(math.sqrt(dx * dx + dy * dy + dz * dz))
  end
  log("I", "roadmind", string.format("Loaded %d ordered route point(s), %d NavGraph nodes", #points, #pathIds))
  return settingsResult()
end

function M.clearRoutePoints()
  if state.routePilotEnabled and state.activeVehicle then
    stopNativeRouteController(state.activeVehicle)
    state._roadmindHelpers.disableNativeRouteController(state.activeVehicle)
  end
  state.routePilotEnabled = false
  state.routePilotStarted = false
  state.routeSafetyStopped = false
  state.routePilotError = nil
  state.routeInputPoints = {}
  state.routePath = {}
  state.routePathEnd = nil
  state.localPlanner = {
    status = "idle", reasonCode = "inactive", timestamp = nil,
    candidates = {}, selectedIndex = nil
  }
  state.routeProgressIndex = 1
  state.routeProgressPercent = 0
  state.routeProgressTimer = 0
  state.routeDistanceToPath = nil
  state.routeLostUpdates = 0
  state.routeDestinationSnapDistance = nil
  state.routeCompleted = false
  state.routePilotVehicleId = nil
  return settingsResult()
end

function M.clearLidarMap()
  state._roadmindHelpers.clearPersistentLidarPoints()
  return settingsResult()
end

function M.recalibrateVehicle()
  if not state.activeVehicle then state._roadmindHelpers.syncPlayerVehicle() end
  if not state.activeVehicle then
    return {ok = false, error = "No active player vehicle"}
  end

  local calibrated = state._roadmindHelpers.calibrateVehicleGeometry(state.activeVehicle)
  if state.visualizationEnabled then state._roadmindHelpers.setVehicleSensorSettings(state.activeVehicle) end
  state._roadmindHelpers.resetFrame()

  local result = settingsResult()
  result.calibrationSucceeded = calibrated
  return result
end

state._roadmindHelpers.invalidateVehicleGeometry = function(vehicleId)
  if vehicleId == nil or tostring(vehicleId) == tostring(state.activeVehicleId) then
    state._roadmindHelpers.resetTrackingState(false)
    state.vehicleRegistry = {}
    state.vehicleRegistryById = {}
    state.vehicleRegistrySnapshotValid = false
    state.vehicleRegistryTimer = 0
    if state.activeVehicle then M.recalibrateVehicle() end
  end
end

function M.onVehicleResetted(vehicleId)
  state._roadmindHelpers.invalidateVehicleGeometry(vehicleId)
end

function M.onVehicleReset(vehicleId)
  state._roadmindHelpers.invalidateVehicleGeometry(vehicleId)
end

function M.onVehicleConfigurationChanged(vehicleId)
  state._roadmindHelpers.invalidateVehicleGeometry(vehicleId)
end

function M.onVehicleConfigChanged(vehicleId)
  state._roadmindHelpers.invalidateVehicleGeometry(vehicleId)
end

function M.onExtensionLoaded()
  if setExtensionUnloadMode then
    setExtensionUnloadMode(M, "manual")
  end
  state._roadmindHelpers.loadSettings()
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
    distance = round2(distance),
    hitObjectId = getVehicleId(safeField(hit, "obj"))
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
      hit.name = lidar.name or "Forward LiDAR"
      hit.origin = lidar.origin
      hit.direction = beams[i].direction
      hit.rayIndex = i
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
    up = lidar.up,
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
      hit.direction = beam.direction
      hit.rayIndex = i
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
        hit.direction = beam.direction
        hit.beamIndex = i
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

state._roadmindHelpers.vectorFromTable = function(point)
  if type(point) ~= "table" or type(point.x) ~= "number"
      or type(point.y) ~= "number" or type(point.z) ~= "number" then
    return nil
  end
  return vec3(point.x, point.y, point.z)
end

state._roadmindHelpers.pointCellKey = function(point)
  local size = PERSISTENT_POINT_CELL_SIZE
  return table.concat({
    tostring(math.floor(point.x / size)),
    tostring(math.floor(point.y / size)),
    tostring(math.floor(point.z / size))
  }, ":")
end

trimPersistentLidarPoints = function()
  local settings = state.settings or DEFAULT_SETTINGS
  local newLimit = math.floor(tonumber(settings.lidarPointCloudLimit)
    or DEFAULT_PERSISTENT_LIDAR_POINTS)
  newLimit = math.max(MIN_PERSISTENT_LIDAR_POINTS,
    math.min(MAX_PERSISTENT_LIDAR_POINTS, newLimit))
  local oldLimit = math.max(1, tonumber(state.lidarPointCloudCapacity) or newLimit)
  local oldCount = math.max(0, math.min(
    tonumber(state.lidarPointCount) or #state.lidarPointCloud,
    #state.lidarPointCloud
  ))
  if oldLimit == newLimit then return end
  local oldWriteIndex = tonumber(state.lidarPointWriteIndex) or 1
  local oldStartIndex = oldCount >= oldLimit and oldWriteIndex or 1
  local keepCount = math.min(oldCount, newLimit)
  local firstOffset = oldCount - keepCount
  local nextCloud, nextCells = {}, {}
  for offset = firstOffset, oldCount - 1 do
    local oldIndex = ((oldStartIndex + offset - 1) % oldLimit) + 1
    local point = state.lidarPointCloud[oldIndex]
    if point then
      nextCloud[#nextCloud + 1] = point
      nextCells[state._roadmindHelpers.pointCellKey(point)] = #nextCloud
    end
  end
  state.lidarPointCloud = nextCloud
  state.lidarPointCells = nextCells
  state.lidarPointCount = #nextCloud
  state.lidarPointCloudCapacity = newLimit
  state.lidarPointWriteIndex = #nextCloud < newLimit and (#nextCloud + 1) or 1
end

state._roadmindHelpers.trackCanOwnLidarPoint = function(track)
  if type(track) ~= "table" then return false end
  local age = state.simulationTime - (track.lastObservedAt or -math.huge)
  return age >= 0 and age <= TRACK_OBSERVATION_MAX_AGE
    and (track.status == "confirmed" or track.status == "coasting")
    and (track.confidence or 0) >= 0.30
end

state._roadmindHelpers.localizeLidarPointOnTrack = function(point, track)
  if not state._roadmindHelpers.trackCanOwnLidarPoint(track) then return nil end
  local position = asVec3(track.position)
  local orientation = track.orientation or {}
  local forward = normalizedVec3(orientation.forward)
  local right = normalizedVec3(orientation.right)
  local up = normalizedVec3(orientation.up)
  if not position or not forward or not right or not up
      or type(track.dimensions) ~= "table" then return nil end

  local delta = asVec3(point)
  if not delta then return nil end
  delta = delta - position

  local localForward = delta:dot(forward)
  local localRight = delta:dot(right)
  local localUp = delta:dot(up)
  local dimensions = track.dimensions
  local length = tonumber(dimensions.length)
  local width = tonumber(dimensions.width)
  local height = tonumber(dimensions.height)
  if not length or not width or not height then return nil end

  local margin = 0.35
  if math.abs(localForward) > length * 0.5 + margin
      or math.abs(localRight) > width * 0.5 + margin
      or math.abs(localUp) > height * 0.5 + margin then
    return nil
  end

  return {
    forward = localForward,
    right = localRight,
    up = localUp
  }
end

state._roadmindHelpers.findTrackForLidarPoint = function(point)
  local sourceVehicleId = point and point.hitObjectId
  if sourceVehicleId ~= nil then
    local direct = state.tracksBySourceId[tostring(sourceVehicleId)]
    local localPosition = direct and state._roadmindHelpers.localizeLidarPointOnTrack(point, direct) or nil
    if localPosition then return direct, localPosition end
    -- The hit belongs to a known non-tracked object or an expired vehicle;
    -- do not spatially reassign it to a different target.
    return nil, nil
  end

  -- When the raycast does not expose an object handle, query only the coarse
  -- spatial cell covering the hit. Ambiguous overlapping OBBs remain static.
  local worldPoint = asVec3(point)
  if not worldPoint then return nil, nil end
  local cellSize = TRACK_SPATIAL_CELL_SIZE
  local key = math.floor(worldPoint.x / cellSize) .. ":"
    .. math.floor(worldPoint.y / cellSize) .. ":"
    .. math.floor(worldPoint.z / cellSize)
  local bucket = state.lidarTrackGrid[key] or {}
  local match, matchLocal = nil, nil
  for i = 1, #bucket do
    local track = bucket[i]
    if state._roadmindHelpers.trackCanOwnLidarPoint(track) then
      local localPosition = state._roadmindHelpers.localizeLidarPointOnTrack(point, track)
      if localPosition then
        if match then return nil, nil end
        match, matchLocal = track, localPosition
      end
    end
  end
  return match, matchLocal
end

local function isRegisteredVehicleHit(point)
  local sourceVehicleId = point and point.hitObjectId
  if sourceVehicleId ~= nil then
    local key = tostring(sourceVehicleId)
    return state.vehicleRegistryById[key] == true or state.tracksBySourceId[key] ~= nil
  end
  return state._roadmindHelpers.findTrackForLidarPoint(point) ~= nil
end

state._roadmindHelpers.pushLidarFadePoint = function(point)
  if type(point) ~= "table" then return end
  state.lidarFadingPoints[#state.lidarFadingPoints + 1] = {
    position = point.renderPosition,
    opacity = point.opacity or 1,
    remaining = LIDAR_POINT_FADE_SECONDS
  }
  while #state.lidarFadingPoints > MAX_LIDAR_FADE_POINTS do
    table.remove(state.lidarFadingPoints, 1)
  end
end

buildLidarRenderPoints = function(lidar)
  if type(lidar) ~= "table" or type(lidar.rays) ~= "table" then return end
  local nextPoints = {}
  local oldPoints = state.lidarRenderPoints
  local matchedOld = {}
  local pointCount = 0
  local inertiaEnabled = (state.settings or DEFAULT_SETTINGS).visualizationInertiaEnabled ~= false
  for i = 1, #lidar.rays do
    local rawPoint = lidar.rays[i]
    local rayIndex = tonumber(rawPoint.rayIndex)
    local worldPoint = asVec3(rawPoint)
    if rawPoint.hit == true and rayIndex and worldPoint and rayIndex >= 1 then
      local track, localPosition = state._roadmindHelpers.findTrackForLidarPoint(rawPoint)
      local objectId = rawPoint.hitObjectId
      local associationKey = track and ("track:" .. track.trackId)
        or (objectId ~= nil and ("object:" .. tostring(objectId)) or "static")
      local previous = oldPoints[rayIndex]
      local isSameReturn = false
      if previous and previous.associationKey == associationKey then
        if track and localPosition and previous.localPosition then
          local df = localPosition.forward - previous.localPosition.forward
          local dr = localPosition.right - previous.localPosition.right
          local du = localPosition.up - previous.localPosition.up
          isSameReturn = math.sqrt(df * df + dr * dr + du * du) <= LIDAR_TRACK_LOCAL_MATCH_DISTANCE
        elseif objectId ~= nil and previous.hitObjectId ~= nil
            and tostring(objectId) == tostring(previous.hitObjectId) then
          local oldMeasurement = asVec3(previous.worldMeasurement)
          isSameReturn = oldMeasurement ~= nil
            and (worldPoint - oldMeasurement):length() <= LIDAR_OBJECT_MATCH_DISTANCE
        elseif objectId == nil and previous.hitObjectId == nil then
          local oldMeasurement = asVec3(previous.worldMeasurement)
          isSameReturn = oldMeasurement ~= nil
            and (worldPoint - oldMeasurement):length() <= LIDAR_STATIC_MATCH_DISTANCE
        end
      end
      local renderPosition = vectorToPlainTable(worldPoint)
      local renderVelocity = {x = 0, y = 0, z = 0}
      local opacity = 1
      if isSameReturn then
        if inertiaEnabled then
          renderPosition = previous.renderPosition or renderPosition
          renderVelocity = previous.renderVelocity or renderVelocity
        end
        opacity = previous.opacity or 1
        matchedOld[rayIndex] = true
      elseif previous then
        state._roadmindHelpers.pushLidarFadePoint(previous)
      end
      pointCount = pointCount + 1
      if pointCount <= MAX_LIDAR_RENDER_POINTS then
        nextPoints[rayIndex] = {
          rayIndex = rayIndex,
          associationKey = associationKey,
          hitObjectId = objectId,
          trackId = track and track.trackId or nil,
          localPosition = localPosition,
          worldMeasurement = vectorToPlainTable(worldPoint),
          renderPosition = renderPosition,
          renderVelocity = renderVelocity,
          opacity = opacity,
          measurementTimestamp = tonumber(rawPoint.measurementTimestamp)
            or tonumber(lidar.scanTimestamp) or state.simulationTime
        }
      end
    end
  end
  for rayIndex, previous in pairs(oldPoints) do
    if not matchedOld[rayIndex] and not nextPoints[rayIndex] then
      local replaced = false
      for i = 1, #lidar.rays do
        if tonumber(lidar.rays[i].rayIndex) == rayIndex then replaced = true break end
      end
      if not replaced then state._roadmindHelpers.pushLidarFadePoint(previous) end
    end
  end
  state.lidarRenderPoints = nextPoints
  local origin = asVec3(lidar.origin)
  state.lidarRenderOriginTarget = origin and vectorToPlainTable(origin) or nil
  if not state.lidarRenderOrigin then
    state.lidarRenderOrigin = state.lidarRenderOriginTarget
    state.lidarRenderOriginVelocity = {x = 0, y = 0, z = 0}
  end
  state.lidarRenderScanTimestamp = tonumber(lidar.scanTimestamp) or state.simulationTime
  state.lidarRenderScanSequence = tonumber(lidar.scanSequence) or state.lidarRenderScanSequence
end

state._roadmindHelpers.getTrackedLidarWorldPosition = function(point)
  if not point.trackId or not point.localPosition then return asVec3(point.worldMeasurement) end
  local track = state.tracks[point.trackId]
  -- Once a point has been attached to a moving body, never fall back to its
  -- old world-space measurement when that body track expires. That would leave
  -- a detached ghost behind the vehicle.
  if not track or not state._roadmindHelpers.trackCanOwnLidarPoint(track) then return nil end
  local position = asVec3(track.position)
  local velocity = asVec3(track.estimatedVelocity)
  if position and velocity then
    local elapsed = math.max(0, state.simulationTime - (track.lastObservedAt or state.simulationTime))
    position = position + velocity * math.min(elapsed, 0.5)
  end
  local orientation = track.orientation or {}
  local forward = normalizedVec3(orientation.forward)
  local right = normalizedVec3(orientation.right)
  local up = normalizedVec3(orientation.up)
  if not position or not forward or not right or not up then return nil end
  local p = point.localPosition
  return position + forward * p.forward + right * p.right + up * p.up
end

state._roadmindHelpers.attachTrackedVehicleReturns = function(returns)
  for i = 1, #(returns or {}) do
    local hit = returns[i]
    local track, localPosition = state._roadmindHelpers.findTrackForLidarPoint(hit)
    if track and localPosition then
      hit.trackId = track.trackId
      hit.localPosition = localPosition
      hit.worldMeasurement = {x = hit.x, y = hit.y, z = hit.z}
    end
  end
end

state._roadmindHelpers.updateTrackedVehicleReturns = function(returns)
  for i = 1, #(returns or {}) do
    local hit = returns[i]
    if hit.trackId and hit.localPosition and hit.worldMeasurement then
      local track = state.tracks[hit.trackId]
      if not state._roadmindHelpers.trackCanOwnLidarPoint(track) then
        hit.renderExpired = true
      else
        local position = state._roadmindHelpers.getTrackedLidarWorldPosition(hit)
        if position then
          hit.x, hit.y, hit.z = position.x, position.y, position.z
        end
      end
    end
  end
end

state._roadmindHelpers.dampedSpringStep = function(currentPosition, currentVelocity, targetPosition, dt, smoothingSeconds, maxVelocity)
  if not currentPosition or not targetPosition then return targetPosition, vec3(0, 0, 0) end
  if dt <= 0 then return currentPosition, currentVelocity or vec3(0, 0, 0) end
  local omega = 2.0 / math.max(0.01, tonumber(smoothingSeconds) or LIDAR_RENDER_SMOOTHING_SECONDS)
  local velocity = currentVelocity or vec3(0, 0, 0)
  local displacement = currentPosition - targetPosition
  local temp = (velocity + displacement * omega) * dt
  local decay = math.exp(-omega * dt)
  local nextPosition = targetPosition + (displacement + temp) * decay
  local nextVelocity = (velocity - temp * omega) * decay
  local speed = nextVelocity:length()
  local velocityLimit = math.max(1, tonumber(maxVelocity) or 60)
  if speed > velocityLimit then nextVelocity = nextVelocity * (velocityLimit / speed) end
  return nextPosition, nextVelocity
end

state._roadmindHelpers.updateLidarRenderPoints = function(dt)
  dt = math.max(0, math.min(tonumber(dt) or 0, 0.1))
  local inertiaEnabled = (state.settings or DEFAULT_SETTINGS).visualizationInertiaEnabled ~= false
  for rayIndex, point in pairs(state.lidarRenderPoints) do
    local pointAge = state.simulationTime - (point.measurementTimestamp or state.simulationTime)
    local targetPosition = pointAge <= LIDAR_RENDER_POINT_MAX_AGE
      and state._roadmindHelpers.getTrackedLidarWorldPosition(point) or nil
    local currentPosition = asVec3(point.renderPosition)
    if not targetPosition or not currentPosition then
      state._roadmindHelpers.pushLidarFadePoint(point)
      state.lidarRenderPoints[rayIndex] = nil
    else
      local distance = (targetPosition - currentPosition):length()
      if distance > 18.0 then
        state._roadmindHelpers.pushLidarFadePoint(point)
        state.lidarRenderPoints[rayIndex] = nil
      elseif point.trackId then
        -- A LiDAR return attached to a tracked vehicle is expressed in that
        -- vehicle's local OBB frame. Never spring it through world space:
        -- it must stay on the same body surface while the vehicle moves.
        point.renderPosition = vectorToPlainTable(targetPosition)
        point.renderVelocity = {x = 0, y = 0, z = 0}
      elseif inertiaEnabled then
        local velocity = asVec3(point.renderVelocity) or vec3(0, 0, 0)
        local nextPosition, nextVelocity = state._roadmindHelpers.dampedSpringStep(currentPosition, velocity, targetPosition, dt)
        point.renderPosition = vectorToPlainTable(nextPosition)
        point.renderVelocity = vectorToPlainTable(nextVelocity)
      else
        point.renderPosition = vectorToPlainTable(targetPosition)
        point.renderVelocity = {x = 0, y = 0, z = 0}
      end
    end
  end
  local originTarget = asVec3(state.lidarRenderOriginTarget)
  local currentOrigin = asVec3(state.lidarRenderOrigin)
  if originTarget then
    if not currentOrigin or not inertiaEnabled then
      state.lidarRenderOrigin = vectorToPlainTable(originTarget)
      state.lidarRenderOriginVelocity = {x = 0, y = 0, z = 0}
    else
      local velocity = asVec3(state.lidarRenderOriginVelocity) or vec3(0, 0, 0)
      local nextPosition, nextVelocity = state._roadmindHelpers.dampedSpringStep(currentOrigin, velocity, originTarget, dt)
      state.lidarRenderOrigin = vectorToPlainTable(nextPosition)
      state.lidarRenderOriginVelocity = vectorToPlainTable(nextVelocity)
    end
  end
  for i = #state.lidarFadingPoints, 1, -1 do
    local point = state.lidarFadingPoints[i]
    point.remaining = point.remaining - dt
    point.opacity = math.max(0, point.remaining / LIDAR_POINT_FADE_SECONDS)
    if point.remaining <= 0 then table.remove(state.lidarFadingPoints, i) end
  end
end

state._roadmindHelpers.rememberLidarPoints = function(lidar)
  if not state.settings or not state.settings.lidarPersistenceEnabled
      or type(lidar) ~= "table" or type(lidar.rays) ~= "table" then return end
  trimPersistentLidarPoints()
  local limit = math.max(MIN_PERSISTENT_LIDAR_POINTS, math.min(
    MAX_PERSISTENT_LIDAR_POINTS,
    tonumber(state.settings.lidarPointCloudLimit) or DEFAULT_PERSISTENT_LIDAR_POINTS
  ))
  local count = state.lidarPointCount or #state.lidarPointCloud
  for i = 1, #lidar.rays do
    local point = lidar.rays[i]
    if point.hit == true and type(point.x) == "number"
        and type(point.y) == "number" and type(point.z) == "number"
        and point.x == point.x and point.y == point.y and point.z == point.z
        and point.x ~= math.huge and point.x ~= -math.huge
        and point.y ~= math.huge and point.y ~= -math.huge
        and point.z ~= math.huge and point.z ~= -math.huge then
      local key = state._roadmindHelpers.pointCellKey(point)
      if not state.lidarPointCells[key] then
        local slot
        if count < limit then
          slot = count + 1
          count = count + 1
        else
          slot = state.lidarPointWriteIndex
          local previous = state.lidarPointCloud[slot]
          if previous then
            local previousKey = state._roadmindHelpers.pointCellKey(previous)
            if state.lidarPointCells[previousKey] == slot then state.lidarPointCells[previousKey] = nil end
          end
        end
        state.lidarPointCloud[slot] = {x = point.x, y = point.y, z = point.z}
        state.lidarPointCells[key] = slot
        state.lidarPointWriteIndex = (slot % limit) + 1
      end
    end
  end
  state.lidarPointCount = count
end

state._roadmindHelpers.updateDetectedVehicleRenderStates = function(dt)
  -- Keep traffic OBBs attached to their real scene objects. The registry and
  -- occlusion work stay rate-limited, but the render pose is sampled each frame.
  local settings = state.settings or DEFAULT_SETTINGS
  if settings.showDetectedVehicles == false then return end

  local updated = state.updatedVehicleRenderKeys or {}
  state.updatedVehicleRenderKeys = updated
  for key in pairs(updated) do updated[key] = nil end

  local function writeVector(target, value)
    target = type(target) == "table" and target or {}
    target.x, target.y, target.z = value.x, value.y, value.z
    return target
  end

  local function writeDimensions(target, value)
    target = type(target) == "table" and target or {}
    target.length = tonumber(value and value.length) or target.length
    target.width = tonumber(value and value.width) or target.width
    target.height = tonumber(value and value.height) or target.height
    return target
  end

  local function attachedPose(vehicle, centerOffsetLocal)
    if not vehicle then return nil end
    local position = asVec3(safeCall(vehicle, "getPosition"))
    local forward = normalizedVec3(safeCall(vehicle, "getDirectionVector"))
    local up = normalizedVec3(safeCall(vehicle, "getDirectionVectorUp"))
    if not position or not forward or not up then return nil end
    local right = normalizedVec3(forward:cross(up))
    if not right then return nil end
    up = normalizedVec3(right:cross(forward)) or up

    local center = position
    if type(centerOffsetLocal) == "table" then
      local longitudinal = tonumber(centerOffsetLocal.forward) or 0
      local lateral = tonumber(centerOffsetLocal.right) or 0
      local vertical = tonumber(centerOffsetLocal.up) or 0
      center = position + forward * longitudinal + right * lateral + up * vertical
    end
    return center, {forward = forward, right = right, up = up}
  end

  local function predictPosition(track)
    local position = asVec3(track and track.position)
    if not position then return nil end
    local velocity = asVec3(track.simulatorVelocity) or asVec3(track.estimatedVelocity)
    local timestamp = tonumber(track.lastObservedAt)
    if not velocity or not timestamp then return position end
    local age = math.max(0, math.min(0.5, state.simulationTime - timestamp))
    return position + velocity * age
  end

  local function updatePosition(key, positionValue, orientationValue, dimensionsValue)
    local position = asVec3(positionValue)
    if not position then return false end

    local entry = state.vehicleRenderStates[key] or {}
    entry.position = writeVector(entry.position, position)
    entry.lastSeenAt = state.simulationTime

    local forward = orientationValue and normalizedVec3(orientationValue.forward) or nil
    local up = orientationValue and normalizedVec3(orientationValue.up) or nil
    if forward and up then
      local right = normalizedVec3(forward:cross(up))
      if right then
        up = normalizedVec3(right:cross(forward)) or up
        local orientation = entry.orientation or {}
        orientation.forward = writeVector(orientation.forward, forward)
        orientation.right = writeVector(orientation.right, right)
        orientation.up = writeVector(orientation.up, up)
        entry.orientation = orientation
      else
        entry.orientation = nil
      end
    else
      entry.orientation = nil
    end
    if type(dimensionsValue) == "table" then
      entry.dimensions = writeDimensions(entry.dimensions, dimensionsValue)
    end
    state.vehicleRenderStates[key] = entry
    updated[key] = true
    return true
  end

  local rendered = 0
  local candidates = state.latestData and state.latestData.vehicles or state.vehicleCandidates
  for i = 1, #(candidates or {}) do
    if rendered >= MAX_RENDERED_VEHICLE_BOXES then break end
    local candidate = candidates[i]
    if candidate.visibility ~= "occluded" then
      local key = tostring(candidate.sourceVehicleId or candidate.id)
      local track = state.tracksBySourceId[key]
      local vehicle = (track and track.vehicleObject) or candidate.vehicleObject
      local centerOffset = (track and track.centerOffsetLocal) or candidate.centerOffsetLocal
      local livePosition, liveOrientation = attachedPose(vehicle, centerOffset)
      local usedLivePose = livePosition ~= nil
      if not usedLivePose then
        livePosition = track and predictPosition(track) or candidate.position
        liveOrientation = track and track.orientation or candidate.orientation
      end
      if updatePosition(
        key,
        livePosition,
        liveOrientation,
        (track and track.dimensions) or candidate.dimensions
      ) then
        rendered = rendered + 1
      end
    end
  end

  local tracks = state.latestData and state.latestData.tracks or {}
  for i = 1, #tracks do
    if rendered >= MAX_RENDERED_VEHICLE_BOXES then break end
    local track = tracks[i]
    if track.status == "confirmed" or track.status == "coasting" then
      local key = tostring(track.sourceVehicleId)
      if not updated[key] then
        local livePosition, liveOrientation = attachedPose(track.vehicleObject, track.centerOffsetLocal)
        if not livePosition then
          livePosition = predictPosition(track)
          liveOrientation = track.orientation
        end
        if updatePosition(key, livePosition, liveOrientation, track.dimensions) then
          rendered = rendered + 1
        end
      end
    end
  end

  for key, entry in pairs(state.vehicleRenderStates) do
    if not updated[key]
        and state.simulationTime - (entry.lastSeenAt or -math.huge) > TRACK_MAX_AGE then
      state.vehicleRenderStates[key] = nil
    end
  end
end

local VISUALIZATION_RENDER_SMOOTHING_SECONDS = 0.16
local VISUALIZATION_RENDER_SNAP_DISTANCE = 60.0
local VISUALIZATION_RENDER_STATE_MAX_AGE = 2.5
local VISUALIZATION_RENDER_PRUNE_INTERVAL = 0.5
local RENDER_IDENTITY_FIELDS = {
  "trackId", "sourceVehicleId", "hitObjectId", "id",
  "name", "rayIndex", "beamIndex", "kind", "offset"
}

state._roadmindHelpers.getRenderArrayIdentity = function(item, index)
  if type(item) ~= "table" then return tostring(index) end
  local parts = {}
  for i = 1, #RENDER_IDENTITY_FIELDS do
    local field = RENDER_IDENTITY_FIELDS[i]
    local value = item[field]
    if type(value) == "string" or type(value) == "number" then
      parts[#parts + 1] = field .. "=" .. tostring(value)
    end
  end
  if #parts == 0 then return tostring(index) end
  return "[" .. table.concat(parts, "|") .. "]"
end

state._roadmindHelpers.smoothVisualizationValue = function(value, path, dt, now, inertiaEnabled, target)
  if type(value) ~= "table" then return value end

  -- Interpolate render-only world-space positions and directions.
  -- Source measurements and the fused World Model are never mutated.
  if type(value.x) == "number" and type(value.y) == "number" and type(value.z) == "number" then
    local targetPosition = vec3(value.x, value.y, value.z)
    target = type(target) == "table" and target or {}
    for key in pairs(target) do
      if key ~= "x" and key ~= "y" and key ~= "z" then target[key] = nil end
    end
    if not inertiaEnabled then
      state.visualizationVectorStates[path] = nil
      target.x, target.y, target.z = targetPosition.x, targetPosition.y, targetPosition.z
      return target
    end

    local entry = state.visualizationVectorStates[path]
    local current = entry and asVec3(entry.position) or nil
    local velocity = entry and asVec3(entry.velocity) or vec3(0, 0, 0)
    if not current or (targetPosition - current):length() > VISUALIZATION_RENDER_SNAP_DISTANCE then
      current = targetPosition
      velocity = vec3(0, 0, 0)
    else
      current, velocity = state._roadmindHelpers.dampedSpringStep(
        current, velocity, targetPosition, dt, VISUALIZATION_RENDER_SMOOTHING_SECONDS, 250
      )
    end
    entry = type(entry) == "table" and entry or {}
    local positionState = type(entry.position) == "table" and entry.position or {}
    positionState.x, positionState.y, positionState.z = current.x, current.y, current.z
    local velocityState = type(entry.velocity) == "table" and entry.velocity or {}
    velocityState.x, velocityState.y, velocityState.z = velocity.x, velocity.y, velocity.z
    entry.position, entry.velocity, entry.lastSeenAt = positionState, velocityState, now
    state.visualizationVectorStates[path] = entry
    target.x, target.y, target.z = current.x, current.y, current.z
    return target
  end

  target = type(target) == "table" and target or {}
  for key, child in pairs(value) do
    local childKey = tostring(key)
    if type(key) == "number" and type(child) == "table" then
      childKey = state._roadmindHelpers.getRenderArrayIdentity(child, key)
    end
    target[key] = state._roadmindHelpers.smoothVisualizationValue(
      child, path .. "/" .. childKey, dt, now, inertiaEnabled, target[key]
    )
  end
  for key in pairs(target) do
    if value[key] == nil then target[key] = nil end
  end
  return target
end
state._roadmindHelpers.captureRenderVehiclePose = function()
  local vehicle = state.activeVehicle
  if not vehicle then return nil end
  local position = asVec3(safeCall(vehicle, "getPosition"))
  local forward = normalizedVec3(safeCall(vehicle, "getDirectionVector"))
  local up = normalizedVec3(safeCall(vehicle, "getDirectionVectorUp"))
  if not position or not forward or not up then return nil end
  local right = normalizedVec3(forward:cross(up))
  if not right then return nil end
  return {
    position = vectorToPlainTable(position),
    forward = vectorToPlainTable(forward),
    right = vectorToPlainTable(right),
    up = vectorToPlainTable(up)
  }
end

state._roadmindHelpers.transformAttachedPoint = function(point, sourcePose, currentPose)
  local world = asVec3(point)
  local sourcePosition = asVec3(sourcePose and sourcePose.position)
  local sourceForward = normalizedVec3(sourcePose and sourcePose.forward)
  local sourceRight = normalizedVec3(sourcePose and sourcePose.right)
  local sourceUp = normalizedVec3(sourcePose and sourcePose.up)
  local targetPosition = asVec3(currentPose and currentPose.position)
  local targetForward = normalizedVec3(currentPose and currentPose.forward)
  local targetRight = normalizedVec3(currentPose and currentPose.right)
  local targetUp = normalizedVec3(currentPose and currentPose.up)
  if not world or not sourcePosition or not sourceForward or not sourceRight or not sourceUp
      or not targetPosition or not targetForward or not targetRight or not targetUp then
    return point
  end
  local delta = world - sourcePosition
  local localForward = delta:dot(sourceForward)
  local localRight = delta:dot(sourceRight)
  local localUp = delta:dot(sourceUp)
  local transformed = targetPosition
    + targetForward * localForward
    + targetRight * localRight
    + targetUp * localUp
  return vectorToPlainTable(transformed)
end

state._roadmindHelpers.transformAttachedDirection = function(direction, sourcePose, currentPose)
  local vector = asVec3(direction)
  local sourceForward = normalizedVec3(sourcePose and sourcePose.forward)
  local sourceRight = normalizedVec3(sourcePose and sourcePose.right)
  local sourceUp = normalizedVec3(sourcePose and sourcePose.up)
  local targetForward = normalizedVec3(currentPose and currentPose.forward)
  local targetRight = normalizedVec3(currentPose and currentPose.right)
  local targetUp = normalizedVec3(currentPose and currentPose.up)
  if not vector or not sourceForward or not sourceRight or not sourceUp
      or not targetForward or not targetRight or not targetUp then return direction end
  local transformed = targetForward * vector:dot(sourceForward)
    + targetRight * vector:dot(sourceRight)
    + targetUp * vector:dot(sourceUp)
  local normalized = normalizedVec3(transformed)
  return normalized and vectorToPlainTable(normalized) or direction
end

state._roadmindHelpers.makeAttachedPoseTransform = function(sourcePose, currentPose)
  local transform = {
    sourcePosition = asVec3(sourcePose and sourcePose.position),
    sourceForward = normalizedVec3(sourcePose and sourcePose.forward),
    sourceRight = normalizedVec3(sourcePose and sourcePose.right),
    sourceUp = normalizedVec3(sourcePose and sourcePose.up),
    targetPosition = asVec3(currentPose and currentPose.position),
    targetForward = normalizedVec3(currentPose and currentPose.forward),
    targetRight = normalizedVec3(currentPose and currentPose.right),
    targetUp = normalizedVec3(currentPose and currentPose.up)
  }
  transform.valid = transform.sourcePosition and transform.sourceForward
    and transform.sourceRight and transform.sourceUp and transform.targetPosition
    and transform.targetForward and transform.targetRight and transform.targetUp
  return transform
end

state._roadmindHelpers.transformAttachedPointWithBasis = function(point, transform)
  if not transform or not transform.valid then return point end
  local world = asVec3(point)
  if not world then return point end
  local delta = world - transform.sourcePosition
  local localForward = delta:dot(transform.sourceForward)
  local localRight = delta:dot(transform.sourceRight)
  local localUp = delta:dot(transform.sourceUp)
  local transformed = transform.targetPosition
    + transform.targetForward * localForward
    + transform.targetRight * localRight
    + transform.targetUp * localUp
  return vectorToPlainTable(transformed)
end

state._roadmindHelpers.transformAttachedDirectionWithBasis = function(direction, transform)
  if not transform or not transform.valid then return direction end
  local vector = asVec3(direction)
  if not vector then return direction end
  local transformed = transform.targetForward * vector:dot(transform.sourceForward)
    + transform.targetRight * vector:dot(transform.sourceRight)
    + transform.targetUp * vector:dot(transform.sourceUp)
  local normalized = normalizedVec3(transformed)
  return normalized and vectorToPlainTable(normalized) or direction
end

state._roadmindHelpers.reanchorAttachedVisuals = function(frame, sourcePose, currentPose)
  if type(frame) ~= "table" or type(sourcePose) ~= "table" or type(currentPose) ~= "table" then
    return nil
  end

  local transform = state._roadmindHelpers.makeAttachedPoseTransform(sourcePose, currentPose)
  local function transformMounts(items)
    for i = 1, #(items or {}) do
      local item = items[i]
      if type(item) == "table" then
        if item.position then item.position = state._roadmindHelpers.transformAttachedPointWithBasis(item.position, transform) end
        if item.origin then item.origin = state._roadmindHelpers.transformAttachedPointWithBasis(item.origin, transform) end
        if item.direction then item.direction = state._roadmindHelpers.transformAttachedDirectionWithBasis(item.direction, transform) end
        if item.forward then item.forward = state._roadmindHelpers.transformAttachedDirectionWithBasis(item.forward, transform) end
        if item.right then item.right = state._roadmindHelpers.transformAttachedDirectionWithBasis(item.right, transform) end
        if item.up then item.up = state._roadmindHelpers.transformAttachedDirectionWithBasis(item.up, transform) end
      end
    end
  end

  if type(frame.lidar) == "table" and frame.lidar.origin then
    frame.lidar.origin = state._roadmindHelpers.transformAttachedPointWithBasis(frame.lidar.origin, transform)
    if frame.lidar.forward then frame.lidar.forward = state._roadmindHelpers.transformAttachedDirectionWithBasis(frame.lidar.forward, transform) end
    if frame.lidar.right then frame.lidar.right = state._roadmindHelpers.transformAttachedDirectionWithBasis(frame.lidar.right, transform) end
    if frame.lidar.up then frame.lidar.up = state._roadmindHelpers.transformAttachedDirectionWithBasis(frame.lidar.up, transform) end
  end
  if type(frame.radar) == "table" then transformMounts(frame.radar.mounts) end
  if type(frame.ultrasonic) == "table" then transformMounts(frame.ultrasonic.mounts) end
  transformMounts(frame.cameras)
  transformMounts(frame.mounts)

  local detectionPose = frame.detectionZonesVehiclePose or sourcePose
  local detectionTransform = detectionPose == sourcePose
    and transform or state._roadmindHelpers.makeAttachedPoseTransform(detectionPose, currentPose)
  local function transformSensorOrigins(items)
    for i = 1, #(items or {}) do
      local item = items[i]
      if type(item) == "table" and item.sensorOrigin then
        item.sensorOrigin = state._roadmindHelpers.transformAttachedPointWithBasis(item.sensorOrigin, detectionTransform)
      end
    end
  end
  transformSensorOrigins(frame.vehicles)
  transformSensorOrigins(frame.tracks)

  local zones = frame.detectionZones and frame.detectionZones.zones
  transformMounts(zones)

  local geometry = frame.geometryDebug
  if type(geometry) == "table" then
    for i = 1, #(geometry.corners or {}) do
      geometry.corners[i] = state._roadmindHelpers.transformAttachedPointWithBasis(geometry.corners[i], transform)
    end
    local axes = geometry.axes
    if type(axes) == "table" then
      if axes.origin then axes.origin = state._roadmindHelpers.transformAttachedPointWithBasis(axes.origin, transform) end
      if axes.forward then axes.forward = state._roadmindHelpers.transformAttachedPointWithBasis(axes.forward, transform) end
      if axes.right then axes.right = state._roadmindHelpers.transformAttachedPointWithBasis(axes.right, transform) end
      if axes.up then axes.up = state._roadmindHelpers.transformAttachedPointWithBasis(axes.up, transform) end
    end
  end
  return transform
end
local ATTACHED_VISUAL_FIELDS = {
  lidar = true, radar = true, ultrasonic = true,
  cameras = true, mounts = true, geometryDebug = true,
  detectionZones = true, localization = true,
  vehicles = true, tracks = true
}

-- Reuse the render-side table tree instead of allocating a full deep copy on
-- every graphics frame. The source snapshot remains immutable.
state._roadmindHelpers.copyVisualizationValueInto = function(value, target)
  if type(value) ~= "table" then return value end
  target = type(target) == "table" and target or {}
  for key, item in pairs(value) do
    if type(item) == "table" then
      target[key] = state._roadmindHelpers.copyVisualizationValueInto(item, target[key])
    else
      target[key] = item
    end
  end
  for key in pairs(target) do
    if value[key] == nil then target[key] = nil end
  end
  return target
end

state._roadmindHelpers.updateVisualizationRenderState = function(dt)
  dt = math.max(0, math.min(tonumber(dt) or 0, 0.10))
  state.visualizationRenderClock = (state.visualizationRenderClock or 0) + dt
  local now = state.visualizationRenderClock
  local inertiaEnabled = (state.settings or DEFAULT_SETTINGS).visualizationInertiaEnabled ~= false
  if type(state.latestData) == "table" then
    local renderFrame = type(state.visualizationRenderFrame) == "table"
      and state.visualizationRenderFrame or {}
    -- Drop fields that disappeared from the latest source snapshot.
    for key in pairs(renderFrame) do
      if state.latestData[key] == nil then renderFrame[key] = nil end
    end
    for key, value in pairs(state.latestData) do
      -- Vehicle-mounted geometry and moving-object measurements must not be
      -- eased in world space before the ego-pose re-anchor. Doing so would
      -- introduce a visible lag even after converting back to vehicle-local
      -- coordinates. LiDAR object returns have their own track-local renderer.
      if ATTACHED_VISUAL_FIELDS[key] then
        renderFrame[key] = state._roadmindHelpers.copyVisualizationValueInto(value, renderFrame[key])
      else
        renderFrame[key] = state._roadmindHelpers.smoothVisualizationValue(
          value, "frame/" .. tostring(key), dt, now, inertiaEnabled, renderFrame[key]
        )
      end
    end
    local currentPose = state._roadmindHelpers.captureRenderVehiclePose()
    state._roadmindHelpers.reanchorAttachedVisuals(renderFrame, state.latestData.vehiclePose, currentPose)
    if type(renderFrame.radar) == "table" then
      state._roadmindHelpers.updateTrackedVehicleReturns(renderFrame.radar.returns)
    end
    if type(renderFrame.ultrasonic) == "table" then
      state._roadmindHelpers.updateTrackedVehicleReturns(renderFrame.ultrasonic.returns)
    end
    if type(renderFrame.localization) == "table" then
      local localization = renderFrame.localization
      if localization then
        local localizationPose = state.latestData.localizationVehiclePose or state.latestData.vehiclePose
        local localizationTransform = localizationPose == state.latestData.vehiclePose
          and state._roadmindHelpers.makeAttachedPoseTransform(state.latestData.vehiclePose, currentPose)
          or state._roadmindHelpers.makeAttachedPoseTransform(localizationPose, currentPose)
        if localization.gnss then
          if localization.gnss.position then
            localization.gnss.position = state._roadmindHelpers.transformAttachedPointWithBasis(
              localization.gnss.position, localizationTransform)
          end
          if localization.gnss.worldPosition then
            localization.gnss.worldPosition = state._roadmindHelpers.transformAttachedPointWithBasis(
              localization.gnss.worldPosition, localizationTransform)
          end
        end
        if localization.imu and localization.imu.position then
          localization.imu.position = state._roadmindHelpers.transformAttachedPointWithBasis(
            localization.imu.position, localizationTransform)
        end
      end
      renderFrame.localization = localization
    end
    renderFrame.vehiclePose = currentPose
    state.visualizationRenderFrame = renderFrame
  else
    state.visualizationRenderFrame = nil
  end

  -- Route and planner coordinates are smoothed only in a separate display copy.
  local overlays = {routePath = state.routePath, planner = state.localPlanner}
  state.visualizationRenderOverlays = state._roadmindHelpers.smoothVisualizationValue(
    overlays, "overlays", dt, now, inertiaEnabled, state.visualizationRenderOverlays
  )

  if not inertiaEnabled then
    state.visualizationVectorStates = {}
    state.visualizationRenderPruneTimer = 0
    return
  end

  state.visualizationRenderPruneTimer = state.visualizationRenderPruneTimer + dt
  if state.visualizationRenderPruneTimer >= VISUALIZATION_RENDER_PRUNE_INTERVAL then
    state.visualizationRenderPruneTimer = 0
    for key, entry in pairs(state.visualizationVectorStates) do
      if now - (entry.lastSeenAt or -math.huge) > VISUALIZATION_RENDER_STATE_MAX_AGE then
        state.visualizationVectorStates[key] = nil
      end
    end
  end
end

state._roadmindHelpers.worldObstacleCellKey = function(position)
  local size = WORLD_MODEL_OBSTACLE_CELL_SIZE
  return table.concat({
    tostring(math.floor(position.x / size)),
    tostring(math.floor(position.y / size)),
    tostring(math.floor(position.z / size))
  }, ":")
end

state._roadmindHelpers.isFiniteNumber = function(value)
  return type(value) == "number"
    and value == value
    and value ~= math.huge
    and value ~= -math.huge
end

state._roadmindHelpers.finiteWorldVector = function(value)
  local x = safeField(value, "x")
  local y = safeField(value, "y")
  local z = safeField(value, "z")
  if type(x) ~= "number" or type(y) ~= "number" or type(z) ~= "number"
      or x ~= x or y ~= y or z ~= z
      or x == math.huge or x == -math.huge
      or y == math.huge or y == -math.huge
      or z == math.huge or z == -math.huge then
    return nil
  end
  return {x = x, y = y, z = z}
end

state._roadmindHelpers.cloneWorldModelValue = function(value)
  if type(value) ~= "table" then return value end
  local result = {}
  for key, item in pairs(value) do
    result[key] = state._roadmindHelpers.cloneWorldModelValue(item)
  end
  return result
end

buildWorldModel = function()
  local latest = state.latestData
  local now = state.simulationTime
  local settings = state.settings or DEFAULT_SETTINGS
  state.worldModelGeneration = state.worldModelGeneration + 1

  local model = {
    schemaVersion = WORLD_MODEL_SCHEMA_VERSION,
    generation = state.worldModelGeneration,
    timestamp = now,
    coordinateFrame = "beamng-world",
    coordinateUnits = "meters",
    targets = {},
    obstacles = {},
    observations = {lidar = {}, radar = {}, ultrasonic = {}, vehicleRegistry = {}},
    observationCount = 0,
    sourceStatus = {},
    quality = {
      invalidObservationCount = 0,
      staleObservationCount = 0,
      staleSourceCount = 0,
      truncatedSourceCount = 0,
      lowConfidenceObservationCount = 0,
      conflictingObservationCount = 0,
      droppedObservationCount = 0,
      staleTargetCount = 0,
      unassociatedVehicleHitCount = 0,
      unverifiedVehicleCandidateCount = 0,
      occludedVehicleCandidateCount = 0
    }
  }

  latest = type(latest) == "table" and latest or {}
  local timestamps = latest.sensorTimestamps or {}
  local sources = {
    {
      key = "lidar",
      label = "Forward LiDAR",
      enabled = state.visualizationEnabled and settings.lidarEnabled == true,
      data = latest.lidar,
      hits = latest.lidar and latest.lidar.rays or {},
      confidence = 0.68
    },
    {
      key = "radar",
      label = "Automotive radar proxy",
      enabled = state.visualizationEnabled and settings.radarEnabled == true,
      data = latest.radar,
      hits = latest.radar and latest.radar.returns or {},
      confidence = 0.60
    },
    {
      key = "ultrasonic",
      label = "Ultrasonic raycast proxy",
      enabled = state.visualizationEnabled and settings.ultrasonicEnabled == true,
      data = latest.ultrasonic,
      hits = latest.ultrasonic and latest.ultrasonic.returns or {},
      confidence = 0.63
    }
  }

  local registryAge = state._roadmindHelpers.isFiniteNumber(state.vehicleRegistryTimestamp)
    and math.max(0, now - state.vehicleRegistryTimestamp) or nil
  local registryEnabled = state.visualizationEnabled and state._roadmindHelpers.vehicleDetectionEnabled(settings)
  local registryFresh = registryEnabled and state.vehicleRegistrySnapshotValid
    and registryAge ~= nil
    and state.vehicleRegistryTimestamp <= now + 0.05
    and registryAge <= WORLD_MODEL_REGISTRY_MAX_AGE
  model.sourceStatus.vehicleRegistry = {
    enabled = registryEnabled,
    timestamp = state.vehicleRegistryTimestamp,
    dataAge = registryAge and round2(registryAge) or nil,
    fresh = registryFresh == true,
    stale = registryEnabled and not registryFresh,
    truncated = state.vehicleRegistryTruncated == true,
    generation = state.vehicleRegistryGeneration,
    sourceKind = "simulator-assisted",
    measurementKind = "vehicle-oriented-bounding-box-snapshot"
  }
  if registryEnabled and not registryFresh then
    model.quality.staleSourceCount = model.quality.staleSourceCount + 1
  end
  if registryEnabled and state.vehicleRegistryTruncated then
    model.quality.truncatedSourceCount = model.quality.truncatedSourceCount + 1
  end

  local obstacleMap = {}
  local obstacleCount = 0

  local function addObstacleObservation(observation)
    local position = observation.position
    local key = state._roadmindHelpers.worldObstacleCellKey(position)
    local obstacle = obstacleMap[key]
    if not obstacle then
      obstacleCount = obstacleCount + 1
      obstacle = {
        obstacleId = "OBS:" .. key,
        coordinateFrame = "beamng-world",
        sumX = 0,
        sumY = 0,
        sumZ = 0,
        observationCount = 0,
        firstObservedAt = observation.timestamp,
        lastObservedAt = observation.timestamp,
        sourceSensors = {},
        sourceSensorKeys = {},
        sourceObservationIds = {},
        conflictingObservationCount = 0
      }
      obstacleMap[key] = obstacle
    end

    if obstacle.observationCount > 0 then
      local centroid = vec3(
        obstacle.sumX / obstacle.observationCount,
        obstacle.sumY / obstacle.observationCount,
        obstacle.sumZ / obstacle.observationCount
      )
      local delta = vec3(position.x, position.y, position.z) - centroid
      if delta:length() > WORLD_MODEL_OBSTACLE_CELL_SIZE * 1.1 then
        obstacle.conflictingObservationCount = obstacle.conflictingObservationCount + 1
        model.quality.conflictingObservationCount =
          model.quality.conflictingObservationCount + 1
      end
    end

    obstacle.sumX = obstacle.sumX + position.x
    obstacle.sumY = obstacle.sumY + position.y
    obstacle.sumZ = obstacle.sumZ + position.z
    obstacle.observationCount = obstacle.observationCount + 1
    obstacle.firstObservedAt = math.min(obstacle.firstObservedAt, observation.timestamp)
    obstacle.lastObservedAt = math.max(obstacle.lastObservedAt, observation.timestamp)

    local sourceKey = observation.sourceSensor .. ":" .. observation.sensorName
    if not obstacle.sourceSensorKeys[sourceKey] then
      obstacle.sourceSensorKeys[sourceKey] = true
      obstacle.sourceSensors[#obstacle.sourceSensors + 1] = {
        sensor = observation.sourceSensor,
        name = observation.sensorName
      }
    end
    obstacle.sourceObservationIds[#obstacle.sourceObservationIds + 1] = observation.observationId
  end

  for sourceIndex = 1, #sources do
    local source = sources[sourceIndex]
    local timestamp = tonumber(timestamps[source.key])
    local age = state._roadmindHelpers.isFiniteNumber(timestamp) and math.max(0, now - timestamp) or nil
    local fresh = source.enabled and age ~= nil
      and timestamp <= now + 0.05 and age <= WORLD_MODEL_OBSERVATION_MAX_AGE

    model.sourceStatus[source.key] = {
      enabled = source.enabled,
      timestamp = timestamp,
      dataAge = age and round2(age) or nil,
      fresh = fresh,
      stale = source.enabled and not fresh,
      sourceKind = "scene-raycast",
      measurementKind = "geometric-raycast-proxy"
    }

    if source.enabled and not fresh then
      model.quality.staleSourceCount = model.quality.staleSourceCount + 1
      model.quality.staleObservationCount =
        model.quality.staleObservationCount + #(source.hits or {})
    elseif fresh then
      local hits = source.hits or {}
      for hitIndex = 1, #hits do
        if model.observationCount >= WORLD_MODEL_MAX_OBSERVATIONS then
          model.quality.droppedObservationCount =
            model.quality.droppedObservationCount + (#hits - hitIndex + 1)
          break
        end

        local hit = hits[hitIndex]
        local position = state._roadmindHelpers.finiteWorldVector(hit)
        local measurementTimestamp = tonumber(hit.measurementTimestamp) or timestamp
        local dataAge = state._roadmindHelpers.isFiniteNumber(measurementTimestamp)
          and math.max(0, now - measurementTimestamp) or math.huge

        if not position or not state._roadmindHelpers.isFiniteNumber(measurementTimestamp)
            or measurementTimestamp > now + 0.05 then
          model.quality.invalidObservationCount = model.quality.invalidObservationCount + 1
        elseif dataAge > WORLD_MODEL_OBSERVATION_MAX_AGE then
          model.quality.staleObservationCount = model.quality.staleObservationCount + 1
        else
          local track = state._roadmindHelpers.findTrackForLidarPoint(hit)
          local sourceVehicleId = hit.hitObjectId
          local knownVehicle = track ~= nil
          if sourceVehicleId ~= nil then
            local vehicleKey = tostring(sourceVehicleId)
            knownVehicle = knownVehicle
              or state.vehicleRegistryById[vehicleKey] == true
              or state.tracksBySourceId[vehicleKey] ~= nil
          end

          local sensorName = type(hit.name) == "string" and hit.name or source.label
          local measurementIndex = hit.rayIndex or hit.beamIndex or hitIndex
          local observationId = string.format(
            "%s:%s:%s",
            source.key,
            tostring(hit.measurementTimestamp or timestamp),
            tostring(measurementIndex)
          )

          local observation = {
            observationId = observationId,
            position = position,
            coordinateFrame = "beamng-world",
            timestamp = measurementTimestamp,
            dataAge = round2(dataAge),
            sourceSensor = source.key,
            sensorName = sensorName,
            confidence = source.confidence,
            confidenceModel = "uncalibrated-raycast-proxy-heuristic",
            sourceKind = "scene-raycast-measurement",
            measurementKind = "geometric-scene-raycast-proxy",
            sensorOrigin = state._roadmindHelpers.finiteWorldVector(hit.origin),
            rayDirection = state._roadmindHelpers.finiteWorldVector(hit.direction),
            range = tonumber(hit.distance),
            rayIndex = tonumber(hit.rayIndex or hit.beamIndex),
            hitObjectId = sourceVehicleId,
            isPredicted = false,
            targetTrackId = track and track.trackId or nil,
            targetVehicleId = knownVehicle and sourceVehicleId or nil,
            orientation = track and {
              forward = vectorToPlainTable(asVec3(track.orientation.forward)),
              right = vectorToPlainTable(asVec3(track.orientation.right)),
              up = vectorToPlainTable(asVec3(track.orientation.up))
            } or nil,
            dimensions = track and {
              length = track.dimensions.length,
              width = track.dimensions.width,
              height = track.dimensions.height
            } or nil
          }

          if observation.confidence < 0.4 then
            model.quality.lowConfidenceObservationCount =
              model.quality.lowConfidenceObservationCount + 1
          end

          local bucket = model.observations[source.key]
          bucket[#bucket + 1] = observation
          model.observationCount = model.observationCount + 1

          if knownVehicle and not track then
            model.quality.unassociatedVehicleHitCount =
              model.quality.unassociatedVehicleHitCount + 1
          end
          if not knownVehicle then addObstacleObservation(observation) end
        end
      end
    end
  end

  -- Simulator-assisted vehicle observations use the same envelope as
  -- scene-raycast observations, while keeping their origin explicitly labelled.
  local vehicleCandidates = latest.vehicles or state.vehicleCandidates or {}
  for i = 1, #vehicleCandidates do
    local candidate = vehicleCandidates[i]
    if candidate.visibility == "occluded" then
      model.quality.occludedVehicleCandidateCount =
        model.quality.occludedVehicleCandidateCount + 1
    elseif candidate.visibility ~= "visible" then
      model.quality.unverifiedVehicleCandidateCount =
        model.quality.unverifiedVehicleCandidateCount + 1
    elseif registryFresh then
      local timestamp = tonumber(candidate.observationTimestamp or candidate.registryTimestamp)
      local visibilityTimestamp = tonumber(candidate.visibilityTimestamp)
      local position = state._roadmindHelpers.finiteWorldVector(candidate.position)
      local dataAge = state._roadmindHelpers.isFiniteNumber(timestamp) and math.max(0, now - timestamp) or math.huge
      local visibilityAge = state._roadmindHelpers.isFiniteNumber(visibilityTimestamp)
        and math.max(0, now - visibilityTimestamp) or math.huge
      local dimensions = candidate.dimensions
      local orientation = candidate.orientation
      local validDimensions = type(dimensions) == "table"
        and state._roadmindHelpers.isFiniteNumber(dimensions.length) and dimensions.length > 0 and dimensions.length <= 30
        and state._roadmindHelpers.isFiniteNumber(dimensions.width) and dimensions.width > 0 and dimensions.width <= 30
        and state._roadmindHelpers.isFiniteNumber(dimensions.height) and dimensions.height > 0 and dimensions.height <= 10
      local validOrientation = type(orientation) == "table"
        and state._roadmindHelpers.finiteWorldVector(orientation.forward) ~= nil
        and state._roadmindHelpers.finiteWorldVector(orientation.right) ~= nil
        and state._roadmindHelpers.finiteWorldVector(orientation.up) ~= nil

      if not position or not timestamp or timestamp > now + 0.05
          or not visibilityTimestamp or visibilityTimestamp > now + 0.05
          or dataAge > WORLD_MODEL_OBSERVATION_MAX_AGE
          or visibilityAge > TRACK_OBSERVATION_MAX_AGE
          or not validDimensions or not validOrientation then
        model.quality.invalidObservationCount = model.quality.invalidObservationCount + 1
      elseif model.observationCount >= WORLD_MODEL_MAX_OBSERVATIONS then
        model.quality.droppedObservationCount = model.quality.droppedObservationCount + 1
      else
        local sourceVehicleId = candidate.sourceVehicleId or candidate.id
        local track = sourceVehicleId ~= nil
          and state.tracksBySourceId[tostring(sourceVehicleId)] or nil
        local confidence = track and track.confidence or 0.68
        local observation = {
          observationId = string.format(
            "vehicleRegistry:%s:%s",
            tostring(sourceVehicleId),
            tostring(timestamp)
          ),
          targetTrackId = track and track.trackId or nil,
          sourceVehicleId = sourceVehicleId,
          position = position,
          orientation = {
            forward = state._roadmindHelpers.finiteWorldVector(orientation.forward),
            right = state._roadmindHelpers.finiteWorldVector(orientation.right),
            up = state._roadmindHelpers.finiteWorldVector(orientation.up)
          },
          dimensions = {
            length = dimensions.length,
            width = dimensions.width,
            height = dimensions.height
          },
          timestamp = timestamp,
          dataAge = round2(dataAge),
          visibilityTimestamp = visibilityTimestamp,
          visibilityAge = round2(visibilityAge),
          sourceSensor = "vehicleRegistry",
          sensorName = candidate.visibleFrom or candidate.bestSensorName or "Vehicle OBB registry",
          sourceSensors = state._roadmindHelpers.copySensorSources(candidate.sensorSources),
          confidence = confidence,
          confidenceModel = "tracked-observation-confidence-heuristic",
          sourceKind = candidate.sourceKind or "simulator-assisted",
          measurementKind = candidate.measurementKind or "vehicle-obb-center-and-extents",
          simulatorVelocity = state._roadmindHelpers.finiteWorldVector(candidate.simulatorVelocity),
          estimatedVelocity = track and state._roadmindHelpers.finiteWorldVector(track.estimatedVelocity) or nil,
          estimatedSpeed = track and track.estimatedSpeed or nil,
          relativeSpeedEstimate = track and track.estimatedRelativeSpeed or nil,
          visibility = "visible",
          coordinateFrame = "beamng-world",
          isPredicted = false
        }
        model.observations.vehicleRegistry[#model.observations.vehicleRegistry + 1] = observation
        model.observationCount = model.observationCount + 1
        if confidence < 0.4 then
          model.quality.lowConfidenceObservationCount =
            model.quality.lowConfidenceObservationCount + 1
        end
      end
    end
  end

  local tracks = latest.tracks or {}
  for i = 1, math.min(#tracks, WORLD_MODEL_MAX_TRACKS) do
    local track = tracks[i]
    local position = state._roadmindHelpers.finiteWorldVector(track.position)
    local timestamp = tonumber(track.timestamp or track.lastObservedAt)
    if not position or not state._roadmindHelpers.isFiniteNumber(timestamp) or timestamp > now + 0.05 then
      model.quality.invalidObservationCount = model.quality.invalidObservationCount + 1
    else
      local dataAge = math.max(0, now - timestamp)
      if track.status == "stale" or dataAge > TRACK_OBSERVATION_MAX_AGE then
        model.quality.staleTargetCount = model.quality.staleTargetCount + 1
      end
      if (track.confidence or 0) < 0.4 then
        model.quality.lowConfidenceObservationCount =
          model.quality.lowConfidenceObservationCount + 1
      end

      model.targets[#model.targets + 1] = {
        trackId = track.trackId,
        status = track.status,
        position = position,
        predictedPosition = state._roadmindHelpers.finiteWorldVector(track.predictedPosition),
        coordinateFrame = "beamng-world",
        timestamp = timestamp,
        dataAge = round2(dataAge),
        visibilityAge = round2(math.max(0, now - (track.lastVisibilityAt or timestamp))),
        confidence = track.confidence,
        positionSource = track.positionSource or "simulator-obb-center",
        sourceKind = track.sourceKind or "simulator-assisted",
        measurementKind = track.measurementKind or "vehicle OBB registry",
        dimensions = track.dimensions and {
          length = track.dimensions.length,
          width = track.dimensions.width,
          height = track.dimensions.height
        } or nil,
        orientation = track.orientation and {
          forward = vectorToPlainTable(asVec3(track.orientation.forward)),
          right = vectorToPlainTable(asVec3(track.orientation.right)),
          up = vectorToPlainTable(asVec3(track.orientation.up))
        } or nil,
        estimatedVelocity = state._roadmindHelpers.finiteWorldVector(track.estimatedVelocity),
        estimatedSpeed = track.estimatedSpeed,
        estimatedRelativeSpeed = track.estimatedRelativeSpeed,
        simulatorVelocity = state._roadmindHelpers.finiteWorldVector(track.simulatorVelocity),
        velocitySource = track.velocitySource or "not-yet-available",
        relativeSpeedSource = track.relativeSpeedSource,
        predictionHorizon = track.predictionHorizon,
        predictedPositionKind = track.estimatedVelocity and "short-horizon-linear-extrapolation" or nil,
        sourceSensors = state._roadmindHelpers.copySensorSources(track.sourceSensors),
        isPredicted = false
      }
    end
  end

  for _, obstacle in pairs(obstacleMap) do
    local count = math.max(1, obstacle.observationCount)
    local sourceCount = #obstacle.sourceSensors
    model.obstacles[#model.obstacles + 1] = {
      obstacleId = obstacle.obstacleId,
      type = "raycast-point-cluster",
      position = {
        x = obstacle.sumX / count,
        y = obstacle.sumY / count,
        z = obstacle.sumZ / count
      },
      coordinateFrame = "beamng-world",
      timestamp = obstacle.lastObservedAt,
      dataAge = round2(math.max(0, now - obstacle.lastObservedAt)),
      confidence = math.min(0.9, 0.55 + math.max(0, sourceCount - 1) * 0.12),
      confidenceModel = "sensor-agreement-heuristic-not-calibrated",
      sourceKind = "scene-raycast",
      measurementKind = "deduplicated-geometric-raycast-points",
      sourceSensors = obstacle.sourceSensors,
      sourceObservationIds = obstacle.sourceObservationIds,
      observationCount = obstacle.observationCount,
      conflictingObservationCount = obstacle.conflictingObservationCount
    }
  end

  table.sort(model.obstacles, function(a, b)
    if a.dataAge == b.dataAge then return a.obstacleId < b.obstacleId end
    return a.dataAge < b.dataAge
  end)


  -- Keep a rolling world-anchored spatial memory around the player.
  -- Leaving the 500 m circle evicts only out-of-range cells; nearby roads stay cached.
  local egoPosition = asVec3(safeCall(state.activeVehicle, "getPosition"))
  state.sessionStaticMap = state.sessionStaticMap or {}
  state.sessionStaticObservationIds = state.sessionStaticObservationIds or {}
  state.sessionStaticObservationOrder = state.sessionStaticObservationOrder or {}
  state.sessionStaticMapDistance = tonumber(state.sessionStaticMapDistance) or 0
  state.sessionStaticMapGeneration = tonumber(state.sessionStaticMapGeneration) or 1
  local mapRadius = math.max(100, math.min(500,
    tonumber(state.settings and state.settings.mapRadiusM) or 500))

  if egoPosition then
    local previousPosition = asVec3(state.sessionStaticMapLastPosition)
    if previousPosition then
      local travelledStep = (egoPosition - previousPosition):length()
      if travelledStep <= 75 then
        state.sessionStaticMapDistance = math.min(10000000,
          state.sessionStaticMapDistance + travelledStep)
      end
    end
    state.sessionStaticMapLastPosition = vectorToPlainTable(egoPosition)
  end

  local staticMapChanged = false
  local lastPrunePosition = asVec3(state.sessionStaticMapLastPrunePosition)
  local radiusChanged = tonumber(state.sessionStaticMapLastRadius) ~= mapRadius
  local shouldPrune = egoPosition and (not lastPrunePosition or radiusChanged
    or (egoPosition - lastPrunePosition):length() >= SESSION_STATIC_MAP_PRUNE_STEP_M)
  local staticCellCount = tonumber(state.sessionStaticMapCount) or 0
  if staticCellCount == 0 and next(state.sessionStaticMap) ~= nil then
    for _ in pairs(state.sessionStaticMap) do staticCellCount = staticCellCount + 1 end
  end
  if shouldPrune then
    local radiusSquared = mapRadius * mapRadius
    local retained = 0
    for key, cell in pairs(state.sessionStaticMap) do
      local point = asVec3(cell and cell.position)
      if point then
        local dx = point.x - egoPosition.x
        local dy = point.y - egoPosition.y
        if dx * dx + dy * dy <= radiusSquared then
          retained = retained + 1
        else
          state.sessionStaticMap[key] = nil
          staticMapChanged = true
        end
      else
        state.sessionStaticMap[key] = nil
        staticMapChanged = true
      end
    end
    staticCellCount = retained
    state.sessionStaticMapCount = retained
    state.sessionStaticMapLastPrunePosition = vectorToPlainTable(egoPosition)
  end
  state.sessionStaticMapLastRadius = mapRadius

  local staticCells = state.sessionStaticMap
  local processedIds = state.sessionStaticObservationIds
  local processedOrder = state.sessionStaticObservationOrder
  local staticMapSources = {"lidar", "ultrasonic"}

  for sourceIndex = 1, #staticMapSources do
    local sourceName = staticMapSources[sourceIndex]
    local sourceObservations = model.observations[sourceName] or {}
    for i = 1, #sourceObservations do
      local observation = sourceObservations[i]
      local observationId = observation and observation.observationId
      if observation and observation.targetTrackId == nil
          and observation.targetVehicleId == nil and observation.isPredicted ~= true then
        local stableId = type(observationId) == "string" and observationId
          or (sourceName .. ":" .. tostring(observation.timestamp or now) .. ":" .. tostring(i))
        if not processedIds[stableId] then
          processedIds[stableId] = true
          local processedCursor = math.max(1, tonumber(state.sessionStaticObservationCursor) or 1)
          processedOrder[#processedOrder + 1] = stableId
          if #processedOrder - processedCursor + 1 > SESSION_STATIC_MAP_MAX_IDS then
            local expiredId = processedOrder[processedCursor]
            processedOrder[processedCursor] = false
            processedCursor = processedCursor + 1
            if expiredId then processedIds[expiredId] = nil end
          end
          if processedCursor > 4096 and processedCursor > #processedOrder * 0.5 then
            local compactOrder = {}
            for orderIndex = processedCursor, #processedOrder do
              local retainedId = processedOrder[orderIndex]
              if retainedId then compactOrder[#compactOrder + 1] = retainedId end
            end
            processedOrder = compactOrder
            state.sessionStaticObservationOrder = processedOrder
            processedCursor = 1
          end
          state.sessionStaticObservationCursor = processedCursor

          local point = asVec3(observation.position)
          if point then
            local key = state._roadmindHelpers.worldObstacleCellKey(point)
            local cell = staticCells[key]
            local timestamp = tonumber(observation.timestamp) or now
            local confidence = tonumber(observation.confidence)
              or (sourceName == "ultrasonic" and 0.58 or 0.68)
            if not cell then
              staticCells[key] = {
                hitCount = 1, conflictCount = 0,
                sumX = point.x, sumY = point.y, sumZ = point.z,
                position = vectorToPlainTable(point),
                lastPosition = vectorToPlainTable(point),
                firstObservedAt = timestamp,
                lastObservedAt = timestamp,
                confidence = confidence,
                sourceMask = {[sourceName] = true}
              }
              staticCellCount = staticCellCount + 1
              staticMapChanged = true
            else
              local lastPosition = asVec3(cell.lastPosition or cell.position)
              local previousCenter = asVec3(cell.position)
              local hadSource = cell.sourceMask and cell.sourceMask[sourceName] == true
              local delta = lastPosition and (point - lastPosition):length() or math.huge
              if delta <= WORLD_MODEL_OBSTACLE_CELL_SIZE * 0.72 then
                cell.hitCount = (cell.hitCount or 0) + 1
                cell.sumX = (cell.sumX or 0) + point.x
                cell.sumY = (cell.sumY or 0) + point.y
                cell.sumZ = (cell.sumZ or 0) + point.z
                cell.position = {
                  x = cell.sumX / cell.hitCount,
                  y = cell.sumY / cell.hitCount,
                  z = cell.sumZ / cell.hitCount
                }
                cell.lastPosition = vectorToPlainTable(point)
                cell.lastObservedAt = math.max(cell.lastObservedAt or timestamp, timestamp)
                cell.confidence = math.max(tonumber(cell.confidence) or 0, confidence)
                cell.sourceMask = cell.sourceMask or {}
                cell.sourceMask[sourceName] = true
                local updatedCenter = asVec3(cell.position)
                if not hadSource or not previousCenter or
                    (updatedCenter - previousCenter):length() >= 0.05 then
                  staticMapChanged = true
                end
              elseif now - (cell.lastObservedAt or 0) > 1.25 then
                staticCells[key] = {
                  hitCount = 1,
                  conflictCount = (cell.conflictCount or 0) + 1,
                  sumX = point.x, sumY = point.y, sumZ = point.z,
                  position = vectorToPlainTable(point),
                  lastPosition = vectorToPlainTable(point),
                  firstObservedAt = timestamp,
                  lastObservedAt = timestamp,
                  confidence = confidence,
                  sourceMask = {[sourceName] = true}
                }
                staticMapChanged = true
              else
                cell.conflictCount = (cell.conflictCount or 0) + 1
              end
            end
          end
        end
      end
    end
  end

  -- If the hard cap is exceeded, find the oldest cells in one batch.
  -- This avoids a full 20,000-cell scan for every single eviction.
  if staticCellCount > SESSION_STATIC_MAP_MAX_CELLS then
    local oldest = {}
    for key, cell in pairs(staticCells) do
      oldest[#oldest + 1] = {
        key = key,
        timestamp = tonumber(cell.lastObservedAt) or -math.huge
      }
    end
    table.sort(oldest, function(a, b) return a.timestamp < b.timestamp end)
    local excess = staticCellCount - SESSION_STATIC_MAP_MAX_CELLS
    for i = 1, math.min(excess, #oldest) do
      staticCells[oldest[i].key] = nil
      staticCellCount = staticCellCount - 1
      staticMapChanged = true
    end
  end
  state.sessionStaticMapCount = staticCellCount

  local outputCenter = asVec3(state.sessionStaticMapOutputCenter)
  local outputCenterMoved = outputCenter and egoPosition
    and (egoPosition - outputCenter):length() or math.huge
  local outputNeedsRebuild = staticMapChanged or shouldPrune or radiusChanged
    or type(state.sessionStaticMapOutputCache) ~= "table"
    or outputCenterMoved >= SESSION_STATIC_MAP_PRUNE_STEP_M
  if outputNeedsRebuild then
    local staticGeometry = {}
    for key, cell in pairs(staticCells) do
      local point = asVec3(cell.position)
      local hits = tonumber(cell.hitCount) or 0
      if point and hits >= 1 then
        local distance = egoPosition and (point - egoPosition):length() or math.huge
        if distance <= mapRadius then
          local sourceSensors = {}
          for sourceName in pairs(cell.sourceMask or {}) do
            sourceSensors[#sourceSensors + 1] = sourceName
          end
          table.sort(sourceSensors)
          staticGeometry[#staticGeometry + 1] = {
            geometryId = key,
            position = vectorToPlainTable(point),
            classification = "stable-static-candidate",
            sourceKind = "session-map-multi-sensor",
            sourceSensors = sourceSensors,
            firstObservedAt = cell.firstObservedAt,
            lastObservedAt = cell.lastObservedAt,
            dataAge = round2(math.max(0, now - (cell.lastObservedAt or now))),
            hitCount = hits,
            conflictCount = cell.conflictCount or 0,
            confidence = math.min(0.92, math.max(
              tonumber(cell.confidence) or 0,
              0.45 + math.min(hits, 8) * 0.055
            ))
          }
        end
      end
    end
    table.sort(staticGeometry, function(a, b)
      local da = egoPosition and (asVec3(a.position) - egoPosition):length() or math.huge
      local db = egoPosition and (asVec3(b.position) - egoPosition):length() or math.huge
      return da < db
    end)
    while #staticGeometry > SESSION_STATIC_MAP_MAX_OUTPUT do
      table.remove(staticGeometry)
    end
    state.sessionStaticMapOutputCache = staticGeometry
    state.sessionStaticMapOutputCenter = egoPosition and vectorToPlainTable(egoPosition) or nil
  end
  model.staticGeometry = state.sessionStaticMapOutputCache or {}

  if staticMapChanged or outputNeedsRebuild then
    state.sessionStaticMapRevision = (state.sessionStaticMapRevision or 0) + 1
  end

  model.summary = {
    targetCount = #model.targets,
    obstacleCount = #model.obstacles,
    staticGeometryCount = #model.staticGeometry,
    observationCount = model.observationCount,
    staleSourceCount = model.quality.staleSourceCount
  }
  return model
end

state._roadmindHelpers.snapshotVehicleBounds = function(vehicle, vehicleId)
  local position = asVec3(safeCall(vehicle, "getPosition"))
  local velocity = asVec3(safeCall(vehicle, "getVelocity"))
  local box = safeCall(vehicle, "getSpawnWorldOOBB")
  if not position or not velocity or not box then return nil end

  local center = asVec3(safeCall(box, "getCenter"))
  local half = asVec3(safeCall(box, "getHalfExtents"))
  local axes = {
    normalizedVec3(safeCall(box, "getAxis", 0)),
    normalizedVec3(safeCall(box, "getAxis", 1)),
    normalizedVec3(safeCall(box, "getAxis", 2))
  }
  if not center or not half or not axes[1] or not axes[2] or not axes[3] then return nil end

  local hx, hy, hz = math.abs(half.x), math.abs(half.y), math.abs(half.z)
  if hx < 0.1 or hy < 0.1 or hz < 0.1
      or hx > 15 or hy > 15 or hz > 5 then
    return nil
  end

  local forward = normalizedVec3(safeCall(vehicle, "getDirectionVector"))
  local up = normalizedVec3(safeCall(vehicle, "getDirectionVectorUp"))
  local right = forward and up and normalizedVec3(forward:cross(up)) or nil
  if not forward or not right or not up then return nil end

  local function projectedHalfExtent(axis)
    return hx * math.abs(axes[1]:dot(axis))
      + hy * math.abs(axes[2]:dot(axis))
      + hz * math.abs(axes[3]:dot(axis))
  end

  local length = projectedHalfExtent(forward) * 2
  local width = projectedHalfExtent(right) * 2
  local height = projectedHalfExtent(up) * 2
  local radius = math.sqrt(hx * hx + hy * hy + hz * hz)
  local centerOffset = center - position

  return {
    id = vehicleId,
    vehicleObject = vehicle,
    center = center,
    position = position,
    velocity = velocity,
    forward = forward,
    right = right,
    up = up,
    centerOffsetLocal = {
      forward = centerOffset:dot(forward),
      right = centerOffset:dot(right),
      up = centerOffset:dot(up)
    },
    dimensions = {length = length, width = width, height = height},
    axes = axes,
    half = {hx, hy, hz},
    radius = radius,
    registryTimestamp = state.simulationTime
  }
end



local function roadMindReadLevelJson(filePath)
  if type(jsonReadFile) ~= "function" or type(filePath) ~= "string" then return nil end
  local ok, result = pcall(jsonReadFile, filePath)
  if ok and type(result) == "table" then return result end
  return nil
end

local function roadMindLevelContext()
  if type(getMissionFilename) ~= "function" then return nil end
  local ok, missionFile = pcall(getMissionFilename)
  if not ok or type(missionFile) ~= "string" or missionFile == "" then return nil end
  local levelDirectory, baseName = string.match(missionFile, "^(.*[/\\])([^/\\]+)$")
  if not levelDirectory or not baseName then return nil end
  local levelName = string.match(levelDirectory, "levels[/\\]([^/\\]+)[/\\]$")
  if not levelName then levelName = string.gsub(baseName, "%.[^%.]+$", "") end
  return missionFile, levelDirectory, levelName
end

local function roadMindPoint(value)
  if type(value) == "table" then
    local x = tonumber(safeField(value, "x") or safeField(value, 1))
    local y = tonumber(safeField(value, "y") or safeField(value, 2))
    local z = tonumber(safeField(value, "z") or safeField(value, 3))
    if x and y and z and x == x and y == y and z == z then
      return {x = x, y = y, z = z}
    end
  end
  local point = asVec3(value)
  return point and vectorToPlainTable(point) or nil
end

local function roadMindPoiText(item)
  local values = {}
  local function append(value)
    if type(value) == "string" or type(value) == "number" then
      values[#values + 1] = tostring(value)
    elseif type(value) == "table" then
      for key, entry in pairs(value) do
        if type(entry) == "string" or type(entry) == "number" then
          values[#values + 1] = tostring(entry)
        elseif type(key) == "string" and entry == true then
          values[#values + 1] = key
        end
      end
    end
  end
  if type(item) == "table" then
    append(safeField(item, "name"))
    append(safeField(item, "title"))
    append(safeField(item, "type"))
    append(safeField(item, "category"))
    append(safeField(item, "desc"))
    append(safeField(item, "description"))
    append(safeField(item, "tags"))
    append(safeField(item, "customFields"))
  end
  return string.lower(table.concat(values, " "))
end

local function roadMindPoiKind(item, fallback)
  local text = roadMindPoiText(item)
  if string.find(text, "charg", 1, true) or string.find(text, "electric", 1, true)
      or string.find(text, "ev_", 1, true) then return "charging" end
  if string.find(text, "fuel", 1, true) or string.find(text, "gas", 1, true)
      or string.find(text, "petrol", 1, true) then return "fuel" end
  if string.find(text, "parking", 1, true) or fallback == "parking" then return "parking" end
  if string.find(text, "garage", 1, true) or string.find(text, "repair", 1, true)
      or string.find(text, "service", 1, true) then return "service" end
  return fallback or "location"
end

state._roadmindHelpers.loadLevelMetadata = function(force)
  local missionFile, levelDirectory, levelName = roadMindLevelContext()
  if not missionFile then
    state.levelMetadataLoaded = false
    state.levelPOIStatus = "unavailable"
    state.levelPOISource = "The active level info.json path is unavailable"
    return false
  end
  if force ~= true and state.levelMetadataLoaded
      and state.levelMetadataFile == missionFile then
    return true
  end

  state.levelMetadataFile = missionFile
  state.levelDirectory = levelDirectory
  state.levelMetadataName = levelName
  state.levelRoadRules = {rightHandDrive = false, turnOnRed = false}
  state.levelPOICache = {}
  state.levelPOIStatus = "unavailable"
  state.levelPOISource = "Level .sites.json and BeamNGPointOfInterest data"

  local levelInfo = roadMindReadLevelJson(missionFile)
  if type(levelInfo) == "table" and type(levelInfo.roadRules) == "table" then
    state.levelRoadRules.rightHandDrive = levelInfo.roadRules.rightHandDrive == true
    state.levelRoadRules.turnOnRed = levelInfo.roadRules.turnOnRed == true
  end

  local poiById = {}
  local loadedSitesFiles = 0
  local function addPoi(item, fallbackKind, source, fallbackName)
    if type(item) ~= "table" or #state.levelPOICache >= 512 then return end
    local position = roadMindPoint(safeField(item, "pos") or safeField(item, "position"))
    if not position then return end
    local name = safeField(item, "name") or safeField(item, "title")
      or safeField(item, "id") or fallbackName or fallbackKind or "Location"
    name = tostring(name)
    local kind = roadMindPoiKind(item, fallbackKind)
    local id = string.lower(kind .. ":" .. name)
    if poiById[id] then return end
    poiById[id] = true
    local tags = safeField(item, "tags")
    state.levelPOICache[#state.levelPOICache + 1] = {
      id = id,
      name = name,
      kind = kind,
      position = position,
      radius = tonumber(safeField(item, "radius")),
      extent = roadMindPoint(safeField(item, "scl") or safeField(item, "size")),
      source = source,
      tags = type(tags) == "table"
        and state._roadmindHelpers.cloneWorldModelValue(tags) or nil
    }
  end

  local sitesManager = rawget(_G, "gameplay_sites_sitesManager")
  if not sitesManager and type(extensions) == "table" and type(extensions.load) == "function" then
    pcall(extensions.load, "gameplay_sites_sitesManager")
    sitesManager = rawget(_G, "gameplay_sites_sitesManager")
  end
  if type(sitesManager) == "table"
      and type(sitesManager.getCurrentLevelSitesFiles) == "function"
      and type(sitesManager.loadSites) == "function" then
    local okFiles, sitesFiles = pcall(sitesManager.getCurrentLevelSitesFiles)
    if okFiles and type(sitesFiles) == "table" then
      for _, sitesFile in ipairs(sitesFiles) do
        local fileName = tostring(sitesFile or "")
        local lowerPath = string.lower(fileName)
        if fileName ~= ""
            and not string.find(lowerPath, "/missions/", 1, true)
            and not string.find(lowerPath, "/quickrace/", 1, true)
            and not string.find(lowerPath, "/scenarios/", 1, true) then
          local okSites, sites = pcall(sitesManager.loadSites, fileName)
          if okSites and type(sites) == "table" then
            loadedSitesFiles = loadedSitesFiles + 1
            local sortedSpots = sites.parkingSpots and sites.parkingSpots.sorted
            if type(sortedSpots) == "table" then
              for _, spot in ipairs(sortedSpots) do
                local customFields = safeField(spot, "customFields")
                addPoi({
                  name = safeField(spot, "name"),
                  pos = safeField(spot, "pos"),
                  tags = customFields and safeField(customFields, "tags") or nil
                }, "parking", fileName, nil)
              end
            end
            for _, item in pairs(sites.locations or {}) do
              addPoi(item, nil, fileName, nil)
            end
          end
        end
      end
    end
  else
    -- Conservative fallback for levels exposing site files but not the manager API.
    local sitesFiles = {"city.sites.json", "facilities.sites.json", "garages.sites.json"}
    for i = 1, #sitesFiles do
      local fileName = sitesFiles[i]
      local sites = roadMindReadLevelJson(levelDirectory .. fileName)
      if type(sites) == "table" then
        loadedSitesFiles = loadedSitesFiles + 1
        for _, item in pairs(sites.locations or {}) do
          addPoi(item, nil, fileName, nil)
        end
        for _, item in pairs(sites.parkingSpots or {}) do
          addPoi(item, "parking", fileName, nil)
        end
      end
    end
  end

  -- Scene POIs include level-authored fuel, garage and service markers.
  if type(scenetree) == "table" and type(scenetree.findClassObjects) == "function"
      and type(scenetree.findObject) == "function" then
    local okNames, objectNames = pcall(scenetree.findClassObjects, "BeamNGPointOfInterest")
    if okNames and type(objectNames) == "table" then
      for i = 1, math.min(#objectNames, 1024) do
        local objectName = objectNames[i]
        local okObject, object = pcall(scenetree.findObject, objectName)
        if okObject and object then
          local title = safeField(object, "title")
          local desc = safeField(object, "desc")
          local objectType = safeField(object, "type")
          addPoi({
            name = title or objectName,
            title = title,
            desc = desc,
            type = objectType,
            pos = safeCall(object, "getPosition")
          }, nil, "BeamNGPointOfInterest", objectName)
        end
      end
    end
  end

  if #state.levelPOICache > 0 then
    state.levelPOIStatus = "available"
  elseif loadedSitesFiles > 0 then
    state.levelPOIStatus = "empty"
  else
    state.levelPOIStatus = "unavailable"
    state.levelPOISource = "No readable sites files or BeamNGPointOfInterest objects"
  end

  state.levelMetadataLoaded = true
  state.levelSignalCache = {
    status = "unavailable",
    source = "core_trafficSignals runtime state has not been sampled yet",
    items = {},
    timestamp = -math.huge
  }
  return true
end

state._roadmindHelpers.loadLevelRoadGeometry = function(force)
  local missionFile, levelDirectory = roadMindLevelContext()
  if not missionFile or not levelDirectory then
    state.levelRoadCacheStatus = "unavailable"
    return false
  end

  local roadFile = levelDirectory .. "items.level.json"
  local sameFile = state.levelRoadCacheFile == roadFile
  if force ~= true and sameFile and not state.levelRoadLoadActive then
    return state.levelRoadCacheStatus == "available"
  end

  if force == true or not sameFile then
    local levelItems = roadMindReadLevelJson(roadFile)
    if type(levelItems) ~= "table" then
      state.levelRoadCacheFile = roadFile
      state.levelRoadCache = {}
      state.levelRoadCacheStatus = "file-unavailable"
      state.levelRoadCacheNodeCount = 0
      state.levelRoadLoadObjects = nil
      state.levelRoadLoadCursor = nil
      state.levelRoadLoadActive = false
      return false
    end

    local objects = safeField(levelItems, "items") or safeField(levelItems, "objects") or levelItems
    if type(objects) ~= "table" then
      state.levelRoadCacheFile = roadFile
      state.levelRoadCache = {}
      state.levelRoadCacheStatus = "invalid-level-items"
      state.levelRoadLoadObjects = nil
      state.levelRoadLoadCursor = nil
      state.levelRoadLoadActive = false
      return false
    end

    state.levelRoadCacheFile = roadFile
    state.levelRoadCache = {}
    state.levelRoadCacheStatus = "loading"
    state.levelRoadCacheNodeCount = 0
    state.levelRoadLoadObjects = objects
    state.levelRoadLoadCursor = nil
    state.levelRoadLoadActive = true
    state.levelRoadCacheRevision = (state.levelRoadCacheRevision or 0) + 1
  end

  local objects = state.levelRoadLoadObjects
  if not state.levelRoadLoadActive or type(objects) ~= "table" then
    return state.levelRoadCacheStatus == "available"
  end

  local roads = state.levelRoadCache or {}
  local totalRoadNodes = state.levelRoadCacheNodeCount or 0
  local cursor = state.levelRoadLoadCursor
  local processedObjects, addedRoads = 0, 0
  local maxObjectsPerSnapshot = 128

  while processedObjects < maxObjectsPerSnapshot do
    local nextKey, item = next(objects, cursor)
    if nextKey == nil then
      state.levelRoadLoadActive = false
      state.levelRoadLoadObjects = nil
      state.levelRoadLoadCursor = nil
      break
    end
    cursor = nextKey
    processedObjects = processedObjects + 1

    if type(item) == "table" and safeField(item, "class") == "DecalRoad"
        and #roads < 4096 then
      local rawNodes = safeField(item, "nodes")
      local nodeCount = type(rawNodes) == "table" and #rawNodes or 0
      if nodeCount >= 2 and nodeCount <= 1024 then
        local nodes = {}
        local previousWidth = tonumber(safeField(item, "width"))
          or tonumber(safeField(item, "defaultWidth")) or 8
        for nodeIndex = 1, nodeCount do
          local rawNode = rawNodes[nodeIndex]
          local position = roadMindPoint(rawNode)
          local width = tonumber(safeField(rawNode, 4) or safeField(rawNode, "width"))
            or previousWidth
          if position and width >= 2.2 and width <= 60 then
            nodes[#nodes + 1] = {
              x = position.x, y = position.y, z = position.z,
              width = width
            }
            previousWidth = width
          else
            nodes = {}
            break
          end
        end

        if #nodes >= 2 then
          local minX, minY, maxX, maxY = math.huge, math.huge, -math.huge, -math.huge
          for nodeIndex = 1, #nodes do
            local node = nodes[nodeIndex]
            minX, minY = math.min(minX, node.x), math.min(minY, node.y)
            maxX, maxY = math.max(maxX, node.x), math.max(maxY, node.y)
          end
          totalRoadNodes = totalRoadNodes + #nodes
          local name = tostring(safeField(item, "name")
            or safeField(item, "persistentId") or ("DecalRoad-" .. tostring(#roads + 1)))
          roads[#roads + 1] = {
            id = tostring(safeField(item, "persistentId") or name),
            name = name,
            nodes = nodes,
            minX = minX, minY = minY, maxX = maxX, maxY = maxY,
            lanesLeft = tonumber(safeField(item, "lanesLeft")),
            lanesRight = tonumber(safeField(item, "lanesRight")),
            autoLanes = safeField(item, "autoLanes"),
            oneWay = safeField(item, "oneWay") == true,
            flipDirection = safeField(item, "flipDirection") == true,
            drivability = tonumber(safeField(item, "drivability")),
            smoothness = math.max(0, math.min(1,
              tonumber(safeField(item, "smoothness")) or 0.5)),
            improvedSpline = safeField(item, "improvedSpline") ~= false,
            material = tostring(safeField(item, "material") or "")
          }
          addedRoads = addedRoads + 1
        end
      end
    end
  end

  state.levelRoadLoadCursor = cursor
  state.levelRoadCache = roads
  state.levelRoadCacheNodeCount = totalRoadNodes
  if addedRoads > 0 then
    state.levelRoadCacheRevision = (state.levelRoadCacheRevision or 0) + 1
  end

  if not state.levelRoadLoadActive then
    state.levelRoadCacheStatus = #roads > 0 and "available" or "empty"
  else
    state.levelRoadCacheStatus = "loading"
  end
  return #roads > 0
end


state._roadmindHelpers.buildMapRoadGeometry = function(vehiclePosition, vehicleForward, detailRange)
  if not vehiclePosition then return {}, "no-vehicle-position" end
  state._roadmindHelpers.loadLevelRoadGeometry(false)
  local roads = state.levelRoadCache or {}
  if #roads == 0 then return {}, state.levelRoadCacheStatus or "unavailable" end
  if state.levelRoadLoadActive == true and not state.mapRoadGeometryCache then
    return {}, "loading"
  end

  local radiusM = math.max(100, math.min(500,
    tonumber(state.settings and state.settings.mapRadiusM) or 500))
  local nearDetailRange = math.max(60, math.min(220, tonumber(detailRange) or 120))
  local previousPosition = state.mapRoadGeometryCachePosition
  local movedEnough = not previousPosition
  if previousPosition then
    local dx, dy = vehiclePosition.x - previousPosition.x, vehiclePosition.y - previousPosition.y
    movedEnough = dx * dx + dy * dy >= 625
  end
  local radiusChanged = tonumber(state.mapRoadGeometryCacheRange) ~= radiusM
  local roadsChanged = tonumber(state.mapRoadGeometryCacheRoadRevision or -1)
    ~= tonumber(state.levelRoadCacheRevision or 0)
  if state.mapRoadGeometryCache and not movedEnough and not radiusChanged and roadsChanged
      and state.levelRoadLoadActive == true
      and state.simulationTime - (tonumber(state.mapRoadGeometryCacheTime) or -math.huge) < 1.5 then
    return state.mapRoadGeometryCache, "loading"
  end
  if state.mapRoadGeometryCache and not movedEnough and not radiusChanged and not roadsChanged then
    return state.mapRoadGeometryCache, "cached-" .. tostring(state.levelRoadCacheStatus or "available")
  end

  local maxRuns, maxSamples = 512, 8000
  local output, totalSamples = {}, 0

  local function distance3(a, b)
    local dx, dy, dz = a.x - b.x, a.y - b.y, a.z - b.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)
  end

  local function catmullScalar(v0, v1, v2, v3, t0, t1, t2, t3, t)
    local d10 = math.max(0.0001, t1 - t0)
    local d21 = math.max(0.0001, t2 - t1)
    local d32 = math.max(0.0001, t3 - t2)
    local d20 = math.max(0.0001, t2 - t0)
    local d31 = math.max(0.0001, t3 - t1)
    local a1 = ((t1 - t) / d10) * v0 + ((t - t0) / d10) * v1
    local a2 = ((t2 - t) / d21) * v1 + ((t - t1) / d21) * v2
    local a3 = ((t3 - t) / d32) * v2 + ((t - t2) / d32) * v3
    local b1 = ((t2 - t) / d20) * a1 + ((t - t0) / d20) * a2
    local b2 = ((t3 - t) / d31) * a2 + ((t - t1) / d31) * a3
    return ((t2 - t) / d21) * b1 + ((t - t1) / d21) * b2
  end

  local function interpolateRoadNode(p0, p1, p2, p3, amount, smooth)
    if smooth <= 0.02 then
      return {
        x = p1.x + (p2.x - p1.x) * amount,
        y = p1.y + (p2.y - p1.y) * amount,
        z = p1.z + (p2.z - p1.z) * amount,
        width = p1.width + (p2.width - p1.width) * amount
      }
    end

    local t0 = 0
    local t1 = t0 + math.sqrt(math.max(0.0001, distance3(p0, p1)))
    local t2 = t1 + math.sqrt(math.max(0.0001, distance3(p1, p2)))
    local t3 = t2 + math.sqrt(math.max(0.0001, distance3(p2, p3)))
    local t = t1 + (t2 - t1) * amount

    local blend = math.max(0, math.min(1, smooth))
    local linearX = p1.x + (p2.x - p1.x) * amount
    local linearY = p1.y + (p2.y - p1.y) * amount
    local linearZ = p1.z + (p2.z - p1.z) * amount
    local linearWidth = p1.width + (p2.width - p1.width) * amount
    local curveX = catmullScalar(p0.x, p1.x, p2.x, p3.x, t0, t1, t2, t3, t)
    local curveY = catmullScalar(p0.y, p1.y, p2.y, p3.y, t0, t1, t2, t3, t)
    local curveZ = catmullScalar(p0.z, p1.z, p2.z, p3.z, t0, t1, t2, t3, t)
    local curveWidth = catmullScalar(p0.width, p1.width, p2.width, p3.width, t0, t1, t2, t3, t)
    return {
      x = linearX + (curveX - linearX) * blend,
      y = linearY + (curveY - linearY) * blend,
      z = linearZ + (curveZ - linearZ) * blend,
      width = math.max(2.2, math.min(60, linearWidth + (curveWidth - linearWidth) * blend))
    }
  end

  local function pointVisible(point)
    local dx, dy = point.x - vehiclePosition.x, point.y - vehiclePosition.y
    return dx * dx + dy * dy <= (radiusM + 35) * (radiusM + 35)
  end

  local function laneLayout(road, averageWidth)
    local leftCount = tonumber(road.lanesLeft)
    local rightCount = tonumber(road.lanesRight)
    local hasManualCounts = leftCount ~= nil or rightCount ~= nil
    leftCount = math.max(0, math.min(8, math.floor(leftCount or 0)))
    rightCount = math.max(0, math.min(8, math.floor(rightCount or 0)))

    if road.flipDirection then leftCount, rightCount = rightCount, leftCount end
    if leftCount + rightCount == 0 and road.autoLanes ~= false then
      local estimatedTotal = math.max(1, math.min(8,
        math.floor(averageWidth / 3.5 + 0.5)))
      if road.oneWay then
        leftCount = estimatedTotal
      else
        leftCount = math.ceil(estimatedTotal * 0.5)
        rightCount = math.floor(estimatedTotal * 0.5)
      end
    end

    local total = leftCount + rightCount
    local fractions = {}
    if total > 1 then
      for index = 1, total - 1 do
        fractions[#fractions + 1] = -1 + (2 * index / total)
      end
    end
    local divider = nil
    if not road.oneWay and leftCount > 0 and rightCount > 0 then
      divider = -1 + (2 * leftCount / total)
    end
    return fractions, divider,
      hasManualCounts and "decalroad-lane-counts" or "decalroad-auto-lane-estimate"
  end

  local function appendRun(road, laneFractions, dividerFraction, laneSource, runPoints, runIndex)
    if #runPoints < 2 or #output >= maxRuns then return end
    output[#output + 1] = {
      id = road.id .. ":" .. tostring(runIndex),
      name = road.name,
      points = runPoints,
      laneFractions = laneFractions,
      centerDividerFraction = dividerFraction,
      laneSource = laneSource,
      source = "DecalRoad-spline",
      material = road.material,
      drivability = road.drivability
    }
  end

  local reachX = radiusM + 35
  local reachY = radiusM + 35
  local nearbyRoads = {}
  for roadIndex = 1, #roads do
    local road = roads[roadIndex]
    local roadMayIntersect = not road.minX
      or (road.maxX >= vehiclePosition.x - reachX and road.minX <= vehiclePosition.x + reachX
        and road.maxY >= vehiclePosition.y - reachY and road.minY <= vehiclePosition.y + reachY)
    if roadMayIntersect then
      local nearestX = road.minX and math.max(road.minX, math.min(road.maxX, vehiclePosition.x))
        or vehiclePosition.x
      local nearestY = road.minY and math.max(road.minY, math.min(road.maxY, vehiclePosition.y))
        or vehiclePosition.y
      local dx, dy = nearestX - vehiclePosition.x, nearestY - vehiclePosition.y
      nearbyRoads[#nearbyRoads + 1] = {
        road = road, distanceSquared = dx * dx + dy * dy
      }
    end
  end
  table.sort(nearbyRoads, function(a, b)
    if a.distanceSquared == b.distanceSquared then return tostring(a.road.id) < tostring(b.road.id) end
    return a.distanceSquared < b.distanceSquared
  end)

  for roadIndex = 1, #nearbyRoads do
    if #output >= maxRuns or totalSamples >= maxSamples then break end
    local road = nearbyRoads[roadIndex].road
    local nodes = road.nodes or {}
    if #nodes >= 2 then
      local averageWidth = 0
      for nodeIndex = 1, #nodes do averageWidth = averageWidth + nodes[nodeIndex].width end
      averageWidth = averageWidth / #nodes
      local laneFractions, dividerFraction, laneSource = laneLayout(road, averageWidth)
      local runPoints, runIndex = {}, 0

      local function flushRun()
        if #runPoints >= 2 then
          runIndex = runIndex + 1
          appendRun(road, laneFractions, dividerFraction, laneSource, runPoints, runIndex)
        end
        runPoints = {}
      end

      for nodeIndex = 1, #nodes - 1 do
        local p1, p2 = nodes[nodeIndex], nodes[nodeIndex + 1]
        local p0 = nodes[math.max(1, nodeIndex - 1)]
        local p3 = nodes[math.min(#nodes, nodeIndex + 2)]
        if nodeIndex == 1 then
          p0 = {
            x = p1.x * 2 - p2.x, y = p1.y * 2 - p2.y,
            z = p1.z * 2 - p2.z, width = p1.width * 2 - p2.width
          }
        end
        if nodeIndex + 1 == #nodes then
          p3 = {
            x = p2.x * 2 - p1.x, y = p2.y * 2 - p1.y,
            z = p2.z * 2 - p1.z, width = p2.width * 2 - p1.width
          }
        end

        local spanLength = distance3(p1, p2)
        local midX = (p1.x + p2.x) * 0.5 - vehiclePosition.x
        local midY = (p1.y + p2.y) * 0.5 - vehiclePosition.y
        local distanceFromEgo = math.sqrt(midX * midX + midY * midY)
        local sampleSpacing = distanceFromEgo <= nearDetailRange and 2.5
          or (distanceFromEgo <= 260 and 8 or 16)
        local steps = math.max(1, math.min(64, math.ceil(spanLength / sampleSpacing)))
        local firstSample = nodeIndex == 1 and 0 or 1
        for sampleIndex = firstSample, steps do
          if totalSamples >= maxSamples or #output >= maxRuns then break end
          local sample = interpolateRoadNode(
            p0, p1, p2, p3, sampleIndex / steps, road.smoothness or 0.5)
          if pointVisible(sample) then
            local previous = runPoints[#runPoints]
            if previous and distance3(previous, sample) > 9 then flushRun() end
            runPoints[#runPoints + 1] = {
              x = round2(sample.x), y = round2(sample.y), z = round2(sample.z),
              width = round2(sample.width)
            }
            totalSamples = totalSamples + 1
          else
            flushRun()
          end
        end
      end
      flushRun()
    end
  end

  state.mapRoadGeometryCache = output
  state.mapRoadGeometryCacheTime = state.simulationTime
  state.mapRoadGeometryCachePosition = {x = vehiclePosition.x, y = vehiclePosition.y, z = vehiclePosition.z}
  state.mapRoadGeometryCacheForward = nil
  state.mapRoadGeometryCacheRange = radiusM
  state.mapRoadGeometryCacheRoadRevision = state.levelRoadCacheRevision or 0
  state.mapRoadGeometryCacheRevision = (state.mapRoadGeometryCacheRevision or 0) + 1
  return output, state.levelRoadCacheStatus or "available"
end


local function roadMindSignalColor(stateName, stateData)
  local values = {string.lower(tostring(stateName or ""))}
  local lights = type(stateData) == "table" and stateData.lights or nil
  if type(lights) == "table" then
    for i = 1, #lights do values[#values + 1] = string.lower(tostring(lights[i])) end
  end
  local joined = table.concat(values, " ")
  if string.find(joined, "green", 1, true) then return "green" end
  if string.find(joined, "yellow", 1, true) or string.find(joined, "amber", 1, true) then return "amber" end
  if string.find(joined, "red", 1, true) then return "red" end
  if string.find(joined, "off", 1, true) or string.find(joined, "black", 1, true) then return "off" end
  return "unknown"
end

state._roadmindHelpers.refreshTrafficSignals = function()
  if not state.levelMetadataLoaded then state._roadmindHelpers.loadLevelMetadata(false) end
  local api = rawget(_G, "core_trafficSignals")
  if not api and type(extensions) == "table" and type(extensions.load) == "function" then
    pcall(extensions.load, "core_trafficSignals")
    api = rawget(_G, "core_trafficSignals")
  end

  local items = {}
  local runtimeAvailable = false
  if type(api) == "table" and type(api.getSignals) == "function" then
    local okSignals, signals = pcall(api.getSignals)
    if okSignals and type(signals) == "table" then
      runtimeAvailable = true
      for i = 1, math.min(#signals, 256) do
        local instance = signals[i]
        local position = roadMindPoint(safeField(instance, "pos"))
        if position then
          local stateName, stateData = nil, nil
          local getState = safeField(instance, "getState")
          if type(getState) == "function" then
            local okState
            okState, stateName, stateData = pcall(getState, instance)
            if not okState then stateName, stateData = nil, nil end
          end
          local road = safeField(instance, "road")
          local direction = roadMindPoint(safeField(instance, "dir") or safeField(instance, "direction"))
          items[#items + 1] = {
            id = tostring(safeField(instance, "id") or i),
            name = tostring(safeField(instance, "name") or ("Signal " .. i)),
            position = position,
            direction = direction,
            state = type(stateName) == "string" and stateName or "unknown",
            color = roadMindSignalColor(stateName, stateData),
            action = type(stateData) == "table" and stateData.action or nil,
            controllerId = safeField(instance, "controllerId"),
            sequenceId = safeField(instance, "sequenceId"),
            road = road and {
              n1 = safeField(road, "n1"),
              n2 = safeField(road, "n2"),
              useLane = safeField(instance, "useCurrentLane")
            } or nil,
            source = "core_trafficSignals.getState",
            live = type(stateName) == "string" and stateName ~= ""
          }
        end
      end
    end
  end

  if not runtimeAvailable then
    local signalData = state.levelDirectory
      and roadMindReadLevelJson(state.levelDirectory .. "signals.json") or nil
    for i, instance in ipairs(signalData and signalData.instances or {}) do
      local position = roadMindPoint(instance.pos or instance.position)
      if position then
        items[#items + 1] = {
          id = tostring(instance.id or i),
          name = tostring(instance.name or ("Signal " .. i)),
          position = position,
          direction = roadMindPoint(instance.dir or instance.direction),
          state = "unknown",
          color = "unknown",
          controllerId = instance.controllerId,
          sequenceId = instance.sequenceId,
          road = nil,
          source = "signals.json metadata; live phase unavailable",
          live = false
        }
      end
    end
  end

  state.levelSignalCache = {
    status = runtimeAvailable and "live" or (#items > 0 and "metadata-only" or "unavailable"),
    source = runtimeAvailable and "core_trafficSignals" or "levels/<level>/signals.json",
    items = items,
    timestamp = state.simulationTime
  }
end

state._roadmindHelpers.updateRoadUnderstanding = function()
  if state._roadmindHelpers.loadLevelMetadata then
    state._roadmindHelpers.loadLevelMetadata(false)
  end
  if not state.activeVehicle or type(map) ~= "table"
      or type(map.findClosestRoad) ~= "function" or type(map.getMap) ~= "function" then
    state.roadState = {valid = false, quality = "unavailable", timestamp = state.simulationTime, corridor = {}}
    return
  end

  local position = asVec3(safeCall(state.activeVehicle, "getPosition"))
  local heading = normalizedVec3(safeCall(state.activeVehicle, "getDirectionVector"))
  if not position or not heading then
    state.roadState = {valid = false, quality = "lost", timestamp = state.simulationTime, corridor = {}}
    return
  end

  local flatHeading = vec3(heading.x, heading.y, 0)
  if flatHeading:length() < 0.0001 then
    state.roadState = {valid = false, quality = "lost", timestamp = state.simulationTime, corridor = {}}
    return
  end
  flatHeading = flatHeading:normalized()

  local okRoad, nodeAId, nodeBId = pcall(map.findClosestRoad, position)
  if not okRoad or nodeAId == nil or nodeBId == nil then
    state.roadState = {valid = false, quality = "lost", timestamp = state.simulationTime, corridor = {}}
    return
  end

  local levelKey = tostring(state.levelMetadataName or state.levelDirectory or "unknown-level")
  local navCache = state.navGraphCache
  if not navCache or navCache.levelKey ~= levelKey or type(navCache.nodes) ~= "table" then
    local okMap, mapData = pcall(map.getMap)
    local loadedNodes = okMap and type(mapData) == "table" and mapData.nodes or nil
    if type(loadedNodes) == "table" and next(loadedNodes) ~= nil then
      state.navGraphGeneration = (state.navGraphGeneration or 0) + 1
      state.navGraphCache = {levelKey = levelKey, nodes = loadedNodes,
        generation = state.navGraphGeneration, timestamp = state.simulationTime}
      navCache = state.navGraphCache
    end
  end
  local nodes = navCache and navCache.nodes or nil
  local nodeA = type(nodes) == "table" and nodes[nodeAId] or nil
  local nodeB = type(nodes) == "table" and nodes[nodeBId] or nil
  local pointA = nodeA and asVec3(safeField(nodeA, "pos")) or nil
  local pointB = nodeB and asVec3(safeField(nodeB, "pos")) or nil
  if not pointA or not pointB then
    state.roadState = {valid = false, quality = "lost", timestamp = state.simulationTime, corridor = {}}
    return
  end

  local segment = pointB - pointA
  local segmentLengthSquared = segment:dot(segment)
  local flatSegment = vec3(segment.x, segment.y, 0)
  if segmentLengthSquared < 0.25 or flatSegment:length() < 0.5 then
    state.roadState = {valid = false, quality = "lost", timestamp = state.simulationTime, corridor = {}}
    return
  end

  local tangent = flatSegment:normalized()
  local alignment = tangent:dot(flatHeading)
  if alignment < 0 then
    nodeAId, nodeBId = nodeBId, nodeAId
    pointA, pointB = pointB, pointA
    segment = pointB - pointA
    tangent = -tangent
    alignment = -alignment
  end

  local projection = math.max(0, math.min(1, (position - pointA):dot(segment) / segmentLengthSquared))
  local nearest = pointA + segment * projection
  local offset = position - nearest
  local roadDistance = offset:length()
  local lateralOffset = offset.x * (-tangent.y) + offset.y * tangent.x

  local quality = "lost"
  if roadDistance <= 4 and alignment >= 0.70 then
    quality = "high"
  elseif roadDistance <= 10 and alignment >= 0.35 then
    quality = "medium"
  elseif roadDistance <= 25 then
    quality = "low"
  end

  local corridor = {
    {x = pointA.x, y = pointA.y, z = pointA.z, nodeId = tostring(nodeAId)},
    {x = pointB.x, y = pointB.y, z = pointB.z, nodeId = tostring(nodeBId)}
  }
  local roadHalfWidth = PLANNER_ROAD_HALF_WIDTH_M
  local widthSource = "conservative-navgraph-heuristic"
  local laneModel = {
    status = "unknown",
    source = "The current NavGraph edge does not expose lane-direction metadata"
  }
  local function readRoadEdge(fromId, toId)
    local fromNode = nodes[fromId]
    local links = fromNode and safeField(fromNode, "links") or nil
    local edge = type(links) == "table" and links[toId] or nil
    if type(edge) ~= "table" then
      local toNode = nodes[toId]
      local reverseLinks = toNode and safeField(toNode, "links") or nil
      edge = type(reverseLinks) == "table" and reverseLinks[fromId] or nil
    end
    return type(edge) == "table" and edge or nil
  end
  local rightHandDrive = state.levelRoadRules
    and state.levelRoadRules.rightHandDrive == true or false

  local function countNavGraphLanes(edge)
    local laneString = type(edge) == "table" and safeField(edge, "lanes") or nil
    if type(laneString) ~= "string" or laneString == "" then return 0 end
    local _, forward = string.gsub(laneString, "%+a", "")
    local _, reverse = string.gsub(laneString, "%-a", "")
    return forward + reverse
  end

  local function roadEdgeHalfWidth(fromNode, toNode, edge)
    local radiusA = tonumber(safeField(fromNode, "radius"))
    local radiusB = tonumber(safeField(toNode, "radius"))
    local validA = radiusA and radiusA >= 0.5 and radiusA <= 30
    local validB = radiusB and radiusB >= 0.5 and radiusB <= 30
    if validA and validB then
      return (radiusA + radiusB) * 0.5, "navgraph-node-radius"
    elseif validA then
      return radiusA, "navgraph-node-radius"
    elseif validB then
      return radiusB, "navgraph-node-radius"
    end

    -- NavGraph lane counts are authoritative even when node radius is absent.
    -- Match BeamNG traffic's narrow-road / normal-lane width convention.
    local laneCount = countNavGraphLanes(edge)
    if laneCount > 0 then
      local estimatedLaneWidth = laneCount == 1 and 2.4 or 3.05
      local estimatedFullWidth = laneCount * estimatedLaneWidth
      return math.max(1.2, math.min(18, estimatedFullWidth * 0.5)),
        "navgraph-lane-count-estimate"
    end
    return PLANNER_ROAD_HALF_WIDTH_M, "conservative-navgraph-heuristic"
  end

  local function buildDirectedLaneModel(edge, fromId, toId, halfWidth)
    local result = {
      status = "unknown", directionStatus = "unknown", directionAllowed = nil,
      forwardLaneOffsets = {}, laneBoundaryOffsets = {}, lanesForward = nil, lanesReverse = nil,
      laneCount = nil, laneWidth = nil,
      oneWay = type(edge) == "table" and safeField(edge, "oneWay") == true or false,
      source = "NavGraph lane metadata unavailable"
    }
    if type(edge) ~= "table" then return result end
    local incomingId = safeField(edge, "inNode")
    local fromMatches = incomingId ~= nil and tostring(incomingId) == tostring(fromId)
    local toMatches = incomingId ~= nil and tostring(incomingId) == tostring(toId)
    result.incomingNode = incomingId ~= nil and tostring(incomingId) or nil
    if incomingId ~= nil and not fromMatches and not toMatches then
      result.status, result.directionStatus, result.directionAllowed =
        "direction-restricted", "incoming-node-mismatch", false
      return result
    end
    local laneString = safeField(edge, "lanes")
    if type(laneString) ~= "string" or laneString == "" or incomingId == nil then
      result.directionStatus = "direction-unknown"
      result.source = "NavGraph connectivity only; lane direction unknown"
      if result.oneWay and not fromMatches then
        result.status, result.directionAllowed = "direction-restricted", false
      end
      return result
    end
    local _, reverseCount = string.gsub(laneString, "%-a", "")
    local _, forwardCount = string.gsub(laneString, "%+a", "")
    local forward = fromMatches and forwardCount or reverseCount
    local reverse = fromMatches and reverseCount or forwardCount
    result.lanesForward, result.lanesReverse = forward, reverse
    result.laneCount = forward + reverse
    result.directionStatus = "lane-metadata"
    if forward <= 0 then
      result.status, result.directionAllowed = "direction-restricted", false
      return result
    end
    if result.laneCount <= 0 or halfWidth <= 0 then return result end
    local laneWidth = (halfWidth * 2) / result.laneCount
    local offsets, boundaries = {}, {}
    if reverse == 0 then
      for i = 0, forward - 1 do
        offsets[#offsets + 1] = rightHandDrive and (-halfWidth + (i + 0.5) * laneWidth)
          or (halfWidth - (i + 0.5) * laneWidth)
      end
    else
      local divider = rightHandDrive and (-halfWidth + reverse * laneWidth)
        or (halfWidth - reverse * laneWidth)
      result.centerDividerOffset = divider
      for i = 0, forward - 1 do
        offsets[#offsets + 1] = rightHandDrive and (divider + (i + 0.5) * laneWidth)
          or (divider - (i + 0.5) * laneWidth)
      end
    end

    -- Draw every physical lane separator across both directions, not just
    -- the separators belonging to the current directed edge.
    for i = 1, result.laneCount - 1 do
      boundaries[#boundaries + 1] = rightHandDrive and (-halfWidth + i * laneWidth)
        or (halfWidth - i * laneWidth)
    end
    result.status, result.directionAllowed = "metadata", true
    result.source = "NavGraph processed DecalRoad/map.json lane data"
    result.laneWidth = laneWidth
    result.forwardLaneOffsets, result.laneBoundaryOffsets = offsets, boundaries
    result.direction = fromMatches and "inNode-to-outNode" or "outNode-to-inNode"
    result.rightHandDrive = rightHandDrive
    result.drivability = tonumber(safeField(edge, "drivability"))
    result.speedLimitMps = tonumber(safeField(edge, "speedLimit"))
    result.roadType = safeField(edge, "type")
    result.hiddenInNavi = safeField(edge, "hiddenInNavi") == true
    return result
  end

  local function edgeAllowsTravel(edge, fromId, laneInfo)
    if type(edge) ~= "table" or safeField(edge, "hiddenInNavi") == true then return false end
    local drivability = tonumber(safeField(edge, "drivability"))
    if drivability ~= nil and drivability <= 0 then return false end
    if laneInfo and laneInfo.status == "direction-restricted" then return false end
    if safeField(edge, "oneWay") == true then
      local incomingId = safeField(edge, "inNode")
      if incomingId == nil or tostring(incomingId) ~= tostring(fromId) then return false end
    end
    return true
  end

  local firstEdge = readRoadEdge(nodeAId, nodeBId)
  roadHalfWidth, widthSource = roadEdgeHalfWidth(nodeA, nodeB, firstEdge)
  laneModel = buildDirectedLaneModel(firstEdge, nodeAId, nodeBId, roadHalfWidth)
  if not edgeAllowsTravel(firstEdge, nodeAId, laneModel) then
    laneModel.status, laneModel.directionAllowed = "direction-restricted", false
  end
  local visited = {[tostring(nodeAId)] = true, [tostring(nodeBId)] = true}
  local currentId = nodeBId
  local currentPosition = pointB
  local corridorDirection = tangent

  -- Extend a short centerline by following the most forward connected graph edge.
  -- This is NavGraph geometry, not a lane-boundary or free-space estimate.
  for _ = 1, 16 do
    local currentNode = nodes[currentId]
    local links = currentNode and safeField(currentNode, "links") or nil
    if type(links) ~= "table" then break end

    local bestId, bestPosition, bestAlignment = nil, nil, 0.20
    for candidateId, edge in pairs(links) do
      if not visited[tostring(candidateId)] then
        local candidateNode = nodes[candidateId]
        local candidatePosition = candidateNode and asVec3(safeField(candidateNode, "pos")) or nil
        local oneWay = type(edge) == "table" and safeField(edge, "oneWay") == true
        local incomingId = type(edge) == "table" and safeField(edge, "inNode") or nil
        if candidatePosition and (not oneWay or incomingId == nil or incomingId == currentId) then
          local step = vec3(candidatePosition.x - currentPosition.x, candidatePosition.y - currentPosition.y, 0)
          if step:length() > 0.5 then
            local candidateAlignment = step:normalized():dot(corridorDirection)
            if candidateAlignment > bestAlignment then
              bestId, bestPosition, bestAlignment = candidateId, candidatePosition, candidateAlignment
            end
          end
        end
      end
    end

    if bestId == nil then break end
    corridor[#corridor + 1] = {
      x = bestPosition.x, y = bestPosition.y, z = bestPosition.z, nodeId = tostring(bestId)
    }
    visited[tostring(bestId)] = true
    local step = vec3(bestPosition.x - currentPosition.x, bestPosition.y - currentPosition.y, 0)
    if step:length() > 0.5 then corridorDirection = step:normalized() end
    currentId = bestId
    currentPosition = bestPosition
  end


  local laneGraph = {
    schemaVersion = 1, status = "unavailable", source = "BeamNG NavGraph",
    timestamp = state.simulationTime, cacheGeneration = navCache and navCache.generation or 0,
    currentLaneId = nil, currentLaneStatus = "unknown",
    segments = {}, junctions = {}, connections = {},
    nodeCount = 0, segmentCount = 0, laneCount = 0, inferredLaneCount = 0,
    junctionCount = 0, connectionCount = 0
  }
  local queue, queuedEdges, visitedEdges = {}, {}, {}
  local queueIndex = 1
  local maxSegments, maxDepth, maxDistance, maxBranches = 192, 32,
    math.max(150, math.min(500, tonumber(state.settings and state.settings.mapRadiusM) or 500)), 4
  local function edgeKey(a,b) return tostring(a)..">"..tostring(b) end
  local function enqueue(fromId,toId,depth,distance,current)
    local key=edgeKey(fromId,toId)
    if queuedEdges[key] or visitedEdges[key] then return end
    local fromNode,toNode=nodes[fromId],nodes[toId]
    if not fromNode or not toNode then return end
    local p1,p2=asVec3(safeField(fromNode,"pos")),asVec3(safeField(toNode,"pos"))
    if not p1 or not p2 then return end
    if (p1-position):length()>maxDistance and (p2-position):length()>maxDistance then return end
    local edge=readRoadEdge(fromId,toId)
    local width=roadEdgeHalfWidth(fromNode,toNode,edge)
    local info=buildDirectedLaneModel(edge,fromId,toId,width)
    if not edgeAllowsTravel(edge,fromId,info) then return end
    queuedEdges[key]=true
    queue[#queue+1]={from=fromId,to=toId,depth=depth,distance=distance,current=current==true}
  end
  enqueue(nodeAId,nodeBId,0,0,true)
  enqueue(nodeBId,nodeAId,0,0,false)

  while queueIndex<=#queue and #laneGraph.segments<maxSegments do
    local item=queue[queueIndex]
    queueIndex=queueIndex+1
    local fromId,toId=item.from,item.to
    local key=edgeKey(fromId,toId)
    if not visitedEdges[key] then
      visitedEdges[key]=true
      local fromNode,toNode=nodes[fromId],nodes[toId]
      local p1=fromNode and asVec3(safeField(fromNode,"pos")) or nil
      local p2=toNode and asVec3(safeField(toNode,"pos")) or nil
      local edge=readRoadEdge(fromId,toId)
      if p1 and p2 and edge then
        local delta=p2-p1
        local flat=vec3(delta.x,delta.y,0)
        local length=flat:length()
        local halfWidth,widthSource=roadEdgeHalfWidth(fromNode,toNode,edge)
        local laneInfo=buildDirectedLaneModel(edge,fromId,toId,halfWidth)
        local drivability=tonumber(safeField(edge,"drivability"))
        if length>0.5 and (drivability==nil or drivability>0) and safeField(edge,"hiddenInNavi")~=true then
          local tangent=flat/length
          local normal=vec3(-tangent.y,tangent.x,0)
          local lanes={}
          if laneInfo.status=="metadata" then
            for laneIndex=1,#(laneInfo.forwardLaneOffsets or {}) do
              local offset=tonumber(laneInfo.forwardLaneOffsets[laneIndex])
              if offset then
                lanes[#lanes+1]={
                  id=key..":lane:"..tostring(laneIndex),index=laneIndex,
                  fromNode=tostring(fromId),toNode=tostring(toId),
                  centerOffset=round2(offset),width=round2(laneInfo.laneWidth or 0),
                  start=vectorToPlainTable(p1+normal*offset),
                  finish=vectorToPlainTable(p2+normal*offset),direction="forward"
                }
                laneGraph.laneCount=laneGraph.laneCount+1
              end
            end
          end
          laneGraph.segments[#laneGraph.segments+1]={
            id=key,from=tostring(fromId),to=tostring(toId),
            start=vectorToPlainTable(p1),finish=vectorToPlainTable(p2),
            length=round2(length),roadHalfWidth=round2(halfWidth),widthSource=widthSource,
            laneModel=laneInfo,lanes=lanes,laneCount=#lanes,
            directionStatus=laneInfo.directionStatus or "unknown",
            oneWay=safeField(edge,"oneWay")==true,drivability=drivability,
            speedLimitMps=tonumber(safeField(edge,"speedLimit")),
            roadType=safeField(edge,"type"),depth=item.depth,current=item.current
          }
          local travelled=(item.distance or 0)+length
          if item.depth<maxDepth and travelled<maxDistance then
            local links=safeField(toNode,"links")
            if type(links)=="table" then
              local choices={}
              for candidateId in pairs(links) do
                if tostring(candidateId)~=tostring(fromId) then
                  local candidateNode=nodes[candidateId]
                  local candidatePosition=candidateNode and asVec3(safeField(candidateNode,"pos")) or nil
                  if candidatePosition and (candidatePosition-position):length()<=maxDistance then
                    local candidateEdge=readRoadEdge(toId,candidateId)
                    if candidateEdge then
                      local nextWidth=roadEdgeHalfWidth(toNode,candidateNode,candidateEdge)
                      local nextInfo=buildDirectedLaneModel(candidateEdge,toId,candidateId,nextWidth)
                      local allowed=edgeAllowsTravel(candidateEdge,toId,nextInfo)
                      local nextDrive=tonumber(safeField(candidateEdge,"drivability"))
                      if allowed and (nextDrive==nil or nextDrive>0) and safeField(candidateEdge,"hiddenInNavi")~=true then
                        local v=candidatePosition-p2
                        local fv=vec3(v.x,v.y,0)
                        if fv:length()>0.5 then
                          local alignment=tangent:dot(fv:normalized())
                          if alignment>-0.45 then choices[#choices+1]={id=candidateId,alignment=alignment} end
                        end
                      end
                    end
                  end
                end
              end
              table.sort(choices,function(a,b)
                if a.alignment==b.alignment then return tostring(a.id)<tostring(b.id) end
                return a.alignment>b.alignment
              end)
              for i=1,math.min(maxBranches,#choices) do enqueue(toId,choices[i].id,item.depth+1,travelled,false) end
            end
          end
        end
      end
    end
  end

  laneGraph.segmentCount=#laneGraph.segments

  -- Build joined road-surface corners from the same NavGraph topology used by AI.
  -- Degree-two nodes receive a shared miter join; real graph junctions get a
  -- compact surface patch so separate road strips do not leave holes.
  local surfaceNodes = {}
  local function registerSurfaceNeighbor(nodeId, neighborId, nodePosition, neighborPosition, halfWidth)
    local key, neighborKey = tostring(nodeId), tostring(neighborId)
    local entry = surfaceNodes[key]
    if not entry then
      entry = {
        position = asVec3(nodePosition), neighbors = {}, byNeighbor = {},
        maxHalfWidth = tonumber(halfWidth) or PLANNER_ROAD_HALF_WIDTH_M
      }
      surfaceNodes[key] = entry
    end
    entry.maxHalfWidth = math.max(entry.maxHalfWidth or 0,
      tonumber(halfWidth) or PLANNER_ROAD_HALF_WIDTH_M)
    local neighbor = entry.byNeighbor[neighborKey]
    if not neighbor then
      neighbor = {
        id = neighborKey, position = asVec3(neighborPosition),
        halfWidth = tonumber(halfWidth) or PLANNER_ROAD_HALF_WIDTH_M
      }
      entry.byNeighbor[neighborKey] = neighbor
      entry.neighbors[#entry.neighbors + 1] = neighbor
    else
      neighbor.halfWidth = math.max(neighbor.halfWidth or 0,
        tonumber(halfWidth) or PLANNER_ROAD_HALF_WIDTH_M)
    end
  end

  for i = 1, #laneGraph.segments do
    local segment = laneGraph.segments[i]
    local p1, p2 = asVec3(segment.start), asVec3(segment.finish)
    if p1 and p2 then
      local halfWidth = tonumber(segment.roadHalfWidth) or PLANNER_ROAD_HALF_WIDTH_M
      registerSurfaceNeighbor(segment.from, segment.to, p1, p2, halfWidth)
      registerSurfaceNeighbor(segment.to, segment.from, p2, p1, halfWidth)
    end
  end

  local function roadSurfaceOffset(nodeId, otherId, travelDirection, atStart, halfWidth)
    local segmentNormal = vec3(-travelDirection.y, travelDirection.x, 0)
    local fallbackOffset = segmentNormal * halfWidth
    local entry = surfaceNodes[tostring(nodeId)]
    if not entry or #entry.neighbors ~= 2 then return fallbackOffset end

    local otherNeighbor = nil
    for i = 1, #entry.neighbors do
      local candidate = entry.neighbors[i]
      if candidate.id ~= tostring(otherId) then
        otherNeighbor = candidate
        break
      end
    end
    local otherPosition = otherNeighbor and otherNeighbor.position
    if not otherPosition or not entry.position then return fallbackOffset end

    local toOther = vec3(
      otherPosition.x - entry.position.x,
      otherPosition.y - entry.position.y, 0
    )
    if toOther:length() < 0.1 then return fallbackOffset end
    toOther = toOther:normalized()

    local joinTangent = atStart and (travelDirection - toOther)
      or (travelDirection + toOther)
    if joinTangent:length() < 0.1 then return fallbackOffset end
    joinTangent = joinTangent:normalized()
    local miterNormal = vec3(-joinTangent.y, joinTangent.x, 0)
    local denominator = miterNormal:dot(segmentNormal)
    if denominator <= 0.45 then return fallbackOffset end

    local joinHalfWidth = (halfWidth + (tonumber(otherNeighbor.halfWidth) or halfWidth)) * 0.5
    local miterLength = joinHalfWidth / denominator
    if miterLength > joinHalfWidth * 2.25 then return fallbackOffset end
    return miterNormal * miterLength
  end

  laneGraph.surfaceJunctions = {}
  for nodeId, entry in pairs(surfaceNodes) do
    if #entry.neighbors >= 3 and entry.position then
      laneGraph.surfaceJunctions[#laneGraph.surfaceJunctions + 1] = {
        id = nodeId, position = vectorToPlainTable(entry.position),
        radius = round2(math.max(2.2, entry.maxHalfWidth or PLANNER_ROAD_HALF_WIDTH_M)),
        connections = #entry.neighbors
      }
    end
  end
  table.sort(laneGraph.surfaceJunctions, function(a, b) return a.id < b.id end)

  for i = 1, #laneGraph.segments do
    local segment = laneGraph.segments[i]
    local p1, p2 = asVec3(segment.start), asVec3(segment.finish)
    local delta = p1 and p2 and vec3(p2.x - p1.x, p2.y - p1.y, 0) or nil
    if delta and delta:length() > 0.5 then
      local direction = delta:normalized()
      local halfWidth = tonumber(segment.roadHalfWidth) or PLANNER_ROAD_HALF_WIDTH_M
      local startOffset = roadSurfaceOffset(segment.from, segment.to, direction, true, halfWidth)
      local finishOffset = roadSurfaceOffset(segment.to, segment.from, direction, false, halfWidth)
      segment.surfaceStartLeft = vectorToPlainTable(p1 + startOffset)
      segment.surfaceStartRight = vectorToPlainTable(p1 - startOffset)
      segment.surfaceFinishLeft = vectorToPlainTable(p2 + finishOffset)
      segment.surfaceFinishRight = vectorToPlainTable(p2 - finishOffset)
      local firstId, secondId = tostring(segment.from), tostring(segment.to)
      if firstId > secondId then firstId, secondId = secondId, firstId end
      segment.surfaceId = firstId .. "<>" .. secondId
    end
  end
  laneGraph.surfaceGeometryStatus = "mitered-navgraph-edges-with-junction-patches"

  -- Infer lane centres from repeated, confirmed traffic trajectories when
  -- NavGraph does not publish lane metadata. Direction alignment prevents
  -- opposite-direction traffic from being merged into one lane estimate.
  local trafficTargets = state.worldModel and state.worldModel.targets or {}
  state.trafficLaneEvidence = state.trafficLaneEvidence or {}
  local laneEvidenceBySegment = state.trafficLaneEvidence
  local activeSegmentIds = {}
  local function inferTrafficLanes(segment)
    if segment.laneModel and segment.laneModel.status == "metadata" then return end
    if #(segment.lanes or {}) > 0 then return end
    local p1, p2 = asVec3(segment.start), asVec3(segment.finish)
    if not p1 or not p2 then return end
    local delta = vec3(p2.x - p1.x, p2.y - p1.y, 0)
    local length = delta:length()
    if length < 2 then return end
    local tangent = delta / length
    local normal = vec3(-tangent.y, tangent.x, 0)
    local halfWidth = tonumber(segment.roadHalfWidth) or 3.5
    local evidence = laneEvidenceBySegment[segment.id]
    if not evidence then
      evidence = {samples = {}, lastByTrack = {}, lastUpdated = state.simulationTime}
      laneEvidenceBySegment[segment.id] = evidence
    end
    evidence.lastUpdated = state.simulationTime

    for targetIndex = 1, math.min(#trafficTargets, 48) do
      local target = trafficTargets[targetIndex]
      local age = tonumber(target and target.dataAge) or math.huge
      local orientation = target and target.orientation
      local heading = orientation and normalizedVec3(orientation.forward)
      local trackPosition = target and asVec3(target.position)
      local speed = tonumber(target and target.estimatedSpeed) or 0
      if target and target.status == "confirmed" and age <= 0.75
          and heading and trackPosition and speed >= 1.0 then
        local alignment = heading:dot(tangent)
        local relative = trackPosition - p1
        local along = relative:dot(tangent)
        local lateral = relative:dot(normal)
        if alignment >= 0.72 and along >= -3 and along <= length + 3
            and math.abs(lateral) <= math.max(1, halfWidth - 0.45) then
          local trackId = tostring(target.trackId or targetIndex)
          local lastSample = tonumber(evidence.lastByTrack[trackId]) or -math.huge
          if state.simulationTime - lastSample >= 1.0 then
            evidence.lastByTrack[trackId] = state.simulationTime
            evidence.samples[#evidence.samples + 1] = {
              offset = lateral, trackId = trackId, timestamp = state.simulationTime
            }
          end
        end
      end
    end

    local freshSamples = {}
    for i = 1, #evidence.samples do
      local sample = evidence.samples[i]
      if state.simulationTime - (tonumber(sample.timestamp) or 0) <= 60 then
        freshSamples[#freshSamples + 1] = sample
      end
    end
    while #freshSamples > 128 do table.remove(freshSamples, 1) end
    evidence.samples = freshSamples
    for trackId, lastSeen in pairs(evidence.lastByTrack) do
      if state.simulationTime - (tonumber(lastSeen) or 0) > 60 then
        evidence.lastByTrack[trackId] = nil
      end
    end
    local offsets = {}
    for i = 1, #freshSamples do offsets[i] = freshSamples[i] end
    table.sort(offsets, function(a, b) return a.offset < b.offset end)

    local clusters = {}
    for i = 1, #offsets do
      local sample = offsets[i]
      local cluster = clusters[#clusters]
      if not cluster or sample.offset - cluster.maxOffset > 1.15 then
        cluster = {sum = 0, count = 0, maxOffset = sample.offset, tracks = {}}
        clusters[#clusters + 1] = cluster
      end
      cluster.sum = cluster.sum + sample.offset
      cluster.count = cluster.count + 1
      cluster.maxOffset = sample.offset
      cluster.tracks[sample.trackId] = true
    end

    local inferred = {}
    for i = 1, #clusters do
      local cluster = clusters[i]
      if cluster.count >= 3 then
        local distinctTracks = 0
        for _ in pairs(cluster.tracks) do distinctTracks = distinctTracks + 1 end
        local centerOffset = cluster.sum / cluster.count
        if math.abs(centerOffset) <= halfWidth - 0.5 then
          inferred[#inferred + 1] = {
            centerOffset = centerOffset, sampleCount = cluster.count,
            distinctTracks = distinctTracks
          }
        end
      end
    end
    while #inferred > 6 do table.remove(inferred) end
    if #inferred == 0 then return end

    local laneWidth = math.max(2.8, math.min(4.2, (halfWidth * 2) / #inferred))
    local boundaries = {math.max(-halfWidth, inferred[1].centerOffset - laneWidth * 0.5)}
    local lanes = {}
    local hasIndependentTracks = false
    for i = 1, #inferred do
      local lane = inferred[i]
      local center = p1 + normal * lane.centerOffset
      local finish = p2 + normal * lane.centerOffset
      local confidence = math.min(0.82,
        0.30 + math.min(lane.distinctTracks, 4) * 0.09 + math.min(lane.sampleCount, 12) * 0.012)
      if lane.distinctTracks >= 2 then hasIndependentTracks = true end
      lanes[#lanes + 1] = {
        id = segment.id .. ":traffic-lane:" .. tostring(i), index = i,
        fromNode = segment.from, toNode = segment.to,
        centerOffset = round2(lane.centerOffset), width = round2(laneWidth),
        start = vectorToPlainTable(center), finish = vectorToPlainTable(finish),
        direction = "forward", source = "tracked-traffic-trajectories",
        confidence = round2(confidence), evidenceTracks = lane.distinctTracks,
        evidenceSamples = lane.sampleCount,
        status = lane.distinctTracks >= 2 and "inferred" or "provisional"
      }
      if i < #inferred then
        boundaries[#boundaries + 1] = (lane.centerOffset + inferred[i + 1].centerOffset) * 0.5
      end
    end
    boundaries[#boundaries + 1] = math.min(halfWidth,
      inferred[#inferred].centerOffset + laneWidth * 0.5)
    segment.lanes = lanes
    segment.laneCount = #lanes
    segment.laneModel = {
      status = hasIndependentTracks and "traffic-inferred" or "traffic-provisional",
      source = "Confirmed moving vehicle trajectories aligned with this directed NavGraph segment",
      laneWidth = round2(laneWidth),
      laneBoundaryOffsets = boundaries,
      evidenceWindowSeconds = 60
    }
    laneGraph.laneCount = laneGraph.laneCount + #lanes
    laneGraph.inferredLaneCount = (laneGraph.inferredLaneCount or 0) + #lanes
  end

  for i = 1, #laneGraph.segments do
    local segment = laneGraph.segments[i]
    activeSegmentIds[segment.id] = true
    inferTrafficLanes(segment)
  end
  for segmentId, evidence in pairs(laneEvidenceBySegment) do
    if not activeSegmentIds[segmentId]
        and state.simulationTime - (tonumber(evidence.lastUpdated) or 0) > 120 then
      laneEvidenceBySegment[segmentId] = nil
    end
  end
  -- Precompute mitered endpoints for every lane separator so markings
  -- remain connected at curved NavGraph edges just like the road surface.
  for i = 1, #laneGraph.segments do
    local segment = laneGraph.segments[i]
    local laneInfo = segment.laneModel or {}
    local boundaries = laneInfo.laneBoundaryOffsets or {}
    local p1, p2 = asVec3(segment.start), asVec3(segment.finish)
    local leftA, leftB = asVec3(segment.surfaceStartLeft), asVec3(segment.surfaceFinishLeft)
    local halfWidth = tonumber(segment.roadHalfWidth) or PLANNER_ROAD_HALF_WIDTH_M
    segment.laneBoundaryGeometry = {}

    if p1 and p2 and halfWidth > 0.1 and type(boundaries) == "table"
        and (laneInfo.status == "metadata" or laneInfo.status == "traffic-inferred"
          or laneInfo.status == "traffic-provisional") then
      local delta = vec3(p2.x - p1.x, p2.y - p1.y, 0)
      local tangent = delta:length() > 0.1 and delta:normalized() or nil
      local normal = tangent and vec3(-tangent.y, tangent.x, 0) or nil
      local startMiter = leftA and vec3(leftA.x - p1.x, leftA.y - p1.y, leftA.z - p1.z)
        or (normal and normal * halfWidth or nil)
      local finishMiter = leftB and vec3(leftB.x - p2.x, leftB.y - p2.y, leftB.z - p2.z)
        or (normal and normal * halfWidth or nil)

      if startMiter and finishMiter then
        local centerDivider = tonumber(laneInfo.centerDividerOffset)
        for boundaryIndex = 1, #boundaries do
          local offset = tonumber(boundaries[boundaryIndex])
          if offset then
            local ratio = math.max(-1, math.min(1, offset / halfWidth))
            local startPoint = p1 + startMiter * ratio
            local finishPoint = p2 + finishMiter * ratio
            segment.laneBoundaryGeometry[#segment.laneBoundaryGeometry + 1] = {
              offset = round2(offset),
              start = vectorToPlainTable(startPoint),
              finish = vectorToPlainTable(finishPoint),
              centerDivider = centerDivider ~= nil and math.abs(offset - centerDivider) < 0.05
            }
          end
        end
      end
    end
  end

  laneGraph.source = "BeamNG NavGraph with confirmed traffic-trajectory inference"

  local seenNodes,incomingByNode,outgoingByNode={},{},{}
  local function addSegment(bucket,nodeId,segment)
    local key=tostring(nodeId)
    bucket[key]=bucket[key] or {}
    bucket[key][#bucket[key]+1]=segment
  end
  for i=1,#laneGraph.segments do
    local seg=laneGraph.segments[i]
    seenNodes[seg.from],seenNodes[seg.to]=true,true
    addSegment(outgoingByNode,seg.from,seg)
    addSegment(incomingByNode,seg.to,seg)
  end
  for _ in pairs(seenNodes) do laneGraph.nodeCount=laneGraph.nodeCount+1 end

  for nodeId,incoming in pairs(incomingByNode) do
    local outgoing=outgoingByNode[nodeId] or {}
    local connectionsAtNode=0
    for ii=1,#incoming do
      local inSeg=incoming[ii]
      for il=1,#(inSeg.lanes or {}) do
        local inLane=inSeg.lanes[il]
        local is,ie=asVec3(inLane.start),asVec3(inLane.finish)
        local inDir=is and ie and vec3(ie.x-is.x,ie.y-is.y,0) or nil
        if inDir and inDir:length()>0.1 then
          inDir=inDir:normalized()
          for oi=1,#outgoing do
            local outSeg=outgoing[oi]
            if outSeg.id~=inSeg.id then
              local best,bestD,bestAlign=nil,math.huge,-1
              for ol=1,#(outSeg.lanes or {}) do
                local lane=outSeg.lanes[ol]
                local os,oe=asVec3(lane.start),asVec3(lane.finish)
                local outDir=os and oe and vec3(oe.x-os.x,oe.y-os.y,0) or nil
                if os and outDir and outDir:length()>0.1 then
                  outDir=outDir:normalized()
                  local align=inDir:dot(outDir)
                  if align>-0.45 then
                    local d=(asVec3(inLane.finish)-os):length()
                    if d<bestD then best,bestD,bestAlign=lane,d,align end
                  end
                end
              end
              local maxJoin=math.max(5,math.min(11,(inSeg.roadHalfWidth+outSeg.roadHalfWidth)*0.82))
              if best and bestD<=maxJoin and #laneGraph.connections<480 then
                local os,oe=asVec3(best.start),asVec3(best.finish)
                local od=vec3(oe.x-os.x,oe.y-os.y,0):normalized()
                local cross=inDir.x*od.y-inDir.y*od.x
                local turn="straight"
                if bestAlign<0.92 then
                  if cross>0.12 then turn="left" elseif cross< -0.12 then turn="right" else turn="curve" end
                end
                laneGraph.connections[#laneGraph.connections+1]={
                  id=inLane.id.."=>"..best.id,fromLaneId=inLane.id,toLaneId=best.id,
                  viaNode=tostring(nodeId),turnType=turn,endpointDistance=round2(bestD),
                  headingAlignment=round2(bestAlign),status="geometric-candidate",
                  legality="unverified",source="NavGraph connectivity and lane-centre geometry"
                }
                connectionsAtNode=connectionsAtNode+1
              end
            end
          end
        end
      end
    end
    if #incoming>0 and #outgoing>1 then
      local node=nodes[nodeId]
      local p=node and asVec3(safeField(node,"pos")) or nil
      laneGraph.junctions[#laneGraph.junctions+1]={
        nodeId=nodeId,position=p and vectorToPlainTable(p) or nil,
        incomingSegmentCount=#incoming,outgoingSegmentCount=#outgoing,
        connectionCount=connectionsAtNode,status="topology-candidate",
        source="NavGraph node connectivity"
      }
    end
  end
  laneGraph.junctionCount=#laneGraph.junctions
  laneGraph.connectionCount=#laneGraph.connections
  laneGraph.status=laneGraph.laneCount>0 and "lane-graph-available"
    or (laneGraph.segmentCount>0 and "road-graph-only" or "unavailable")

  local laneLocalization={status="unknown",laneId=nil,laneIndex=nil,lateralError=nil,confidence=0,
    source="No verified forward lane metadata on current road edge"}
  if laneModel.status=="metadata" and #(laneModel.forwardLaneOffsets or {})>0 then
    local bestIndex,bestError=nil,math.huge
    for i=1,#laneModel.forwardLaneOffsets do
      local offset=tonumber(laneModel.forwardLaneOffsets[i])
      if offset then
        local d=math.abs(lateralOffset-offset)
        if d<bestError then bestIndex,bestError=i,d end
      end
    end
    if bestIndex then
      local laneWidth=tonumber(laneModel.laneWidth) or 3.5
      local confidence=math.max(0,math.min(1,1-bestError/math.max(0.5,laneWidth*0.75)))
      laneLocalization={
        status=(quality=="high" or quality=="medium") and bestError<=laneWidth*0.75 and "localized" or "uncertain",
        laneId=edgeKey(nodeAId,nodeBId)..":lane:"..tostring(bestIndex),laneIndex=bestIndex,
        centerOffset=tonumber(laneModel.forwardLaneOffsets[bestIndex]),
        lateralError=round2(bestError),confidence=round2(confidence),
        source="Vehicle pose projected onto directed NavGraph lane centre"
      }
      if laneLocalization.status=="localized" then
        laneGraph.currentLaneId,laneGraph.currentLaneStatus=laneLocalization.laneId,"localized"
      else laneGraph.currentLaneStatus="uncertain" end
    end
  end
  if laneGraph.currentLaneStatus=="unknown" and laneLocalization.status=="uncertain" then
    laneGraph.currentLaneStatus="uncertain"
  end

  state.roadState = {
    valid = true,
    quality = quality,
    source = "BeamNG NavGraph nearest segment",
    timestamp = state.simulationTime,
    segmentStartNode = tostring(nodeAId),
    segmentEndNode = tostring(nodeBId),
    distanceToSegment = roadDistance,
    lateralOffset = lateralOffset,
    headingAlignment = alignment,
    roadDirection = {x = tangent.x, y = tangent.y, z = 0},
    corridor = corridor,
    roadHalfWidth = roadHalfWidth,
    widthSource = widthSource,
    laneModel = laneModel,
    laneLocalization = laneLocalization,
    laneGraph = laneGraph
  }
end

state._roadmindHelpers.setLocalPlannerState = function(status, reasonCode, reason, candidates, selectedIndex, details)
  state.localPlanner = {
    status = status,
    reasonCode = reasonCode,
    reason = reason,
    timestamp = state.simulationTime,
    candidates = candidates or {},
    selectedIndex = selectedIndex,
    candidateCount = #(candidates or {}),
    rejectedCount = 0,
    minimumClearance = nil,
    minimumRoadMargin = nil,
    horizon = PLANNER_HORIZON_M,
    roadBoundaryModel = "NavGraph centerline with conservative 3.5 m half-width heuristic",
    worldModelGeneration = state.worldModel and state.worldModel.generation or 0
  }
  for i = 1, #(candidates or {}) do
    if candidates[i].safe ~= true then
      state.localPlanner.rejectedCount = state.localPlanner.rejectedCount + 1
    end
  end
  if type(details) == "table" then
    for key, value in pairs(details) do state.localPlanner[key] = value end
  end
  return state.localPlanner
end

state._roadmindHelpers.ensurePlannerRoutePath = function()
  if type(state.routePath) == "table" and #state.routePath >= 2 then return true end
  local pathIds, pathPoints, reason
  if #state.routeInputPoints > 0 then
    pathIds, pathPoints, reason = state._roadmindHelpers.buildRoadRouteThroughPoints(state.activeVehicle, state.routeInputPoints)
  else
    pathIds, pathPoints, reason = state._roadmindHelpers.buildForwardRoadPath(state.activeVehicle)
  end
  if not pathIds or type(pathPoints) ~= "table" or #pathPoints < 2 then
    return false, reason or "No usable NavGraph path is available"
  end
  state.routePath = pathPoints
  state.routePathEnd = pathPoints[#pathPoints]
  state.routeProgressIndex = 1
  state.routeProgressPercent = 0
  state.routeDistanceToPath = nil
  state.routeLostUpdates = 0
  return true
end

state._roadmindHelpers.getPlannerVehicleFootprint = function()
  local profile = state.vehicleGeometry
  if type(profile) ~= "table" or profile.valid ~= true then return nil end
  local position = asVec3(safeCall(state.activeVehicle, "getPosition"))
  local forward = normalizedVec3(safeCall(state.activeVehicle, "getDirectionVector"))
  local up = normalizedVec3(safeCall(state.activeVehicle, "getDirectionVectorUp"))
  if not position or not forward or not up then return nil end
  local flatForward = vec3(forward.x, forward.y, 0)
  if flatForward:length() < 0.0001 then return nil end
  flatForward = flatForward:normalized()
  local right = normalizedVec3(flatForward:cross(up))
  if not right then return nil end

  local forwardMin, forwardMax = tonumber(profile.forwardMin), tonumber(profile.forwardMax)
  local lateralMin, lateralMax = tonumber(profile.lateralMin), tonumber(profile.lateralMax)
  local bottom, top = tonumber(profile.bottom), tonumber(profile.top)
  if not forwardMin or not forwardMax or not lateralMin or not lateralMax
      or not bottom or not top or forwardMax <= forwardMin
      or lateralMax <= lateralMin or top <= bottom then return nil end
  local length = forwardMax - forwardMin
  local width = lateralMax - lateralMin
  local height = top - bottom
  if length < 1.2 or length > 30 or width < 0.5 or width > 10
      or height < 0.45 or height > 9 then return nil end

  local center = position
    + forward * ((forwardMin + forwardMax) * 0.5)
    + right * ((lateralMin + lateralMax) * 0.5)
    + up * ((bottom + top) * 0.5)
  return {
    center = center, forward = flatForward, right = right, up = up,
    halfLength = length * 0.5, halfWidth = width * 0.5, halfHeight = height * 0.5,
    length = length, width = width, height = height
  }
end

state._roadmindHelpers.findClosestPlannerRouteProjection = function(path, position)
  local bestIndex, bestPoint, bestDistance = nil, nil, math.huge
  for i = 1, #path - 1 do
    local a, b = asVec3(path[i]), asVec3(path[i + 1])
    if a and b then
      local segment = vec3(b.x - a.x, b.y - a.y, 0)
      local lengthSquared = segment:dot(segment)
      if lengthSquared > 0.25 then
        local offset = vec3(position.x - a.x, position.y - a.y, 0)
        local t = math.max(0, math.min(1, offset:dot(segment) / lengthSquared))
        local point = a + (b - a) * t
        local dx, dy = position.x - point.x, position.y - point.y
        local distance = math.sqrt(dx * dx + dy * dy)
        if distance < bestDistance then
          bestIndex, bestPoint, bestDistance = i, point, distance
        end
      end
    end
  end
  return bestIndex, bestPoint, bestDistance
end

state._roadmindHelpers.samplePlannerRoute = function(path, startIndex, startPoint, distance)
  local remaining = math.max(0, distance)
  local segmentStart = startPoint
  for i = startIndex, #path - 1 do
    local finish = asVec3(path[i + 1])
    if not finish then return nil, nil end
    local delta = finish - segmentStart
    local flatDelta = vec3(delta.x, delta.y, 0)
    local length = flatDelta:length()
    if length > 0.25 then
      if remaining <= length then
        return segmentStart + delta * (remaining / length), flatDelta / length
      end
      remaining = remaining - length
    end
    segmentStart = finish
  end
  return nil, nil
end

state._roadmindHelpers.smoothPlannerRamp = function(distance)
  local t = math.max(0, math.min(1, (distance - 1.0) / 8.0))
  return t * t * (3 - 2 * t)
end

state._roadmindHelpers.plannerDirections = function(points, initialForward)
  local result = {}
  for i = 1, #points do
    if i == 1 then
      result[i] = initialForward
    else
      local a, b = points[i - 1], points[i]
      local delta = vec3(b.x - a.x, b.y - a.y, 0)
      result[i] = delta:length() > 0.05 and delta:normalized() or initialForward
    end
  end
  return result
end

state._roadmindHelpers.evaluatePlannerCandidate = function(candidate, footprint, obstacles, targets, centerlineSamples)
  local points = candidate.path
  local directions = state._roadmindHelpers.plannerDirections(points, footprint.forward)
  local minimumRadius = math.max(2.8, footprint.length * 0.55)
  local previousCurvature, curvatureChange, maximumCurvature = 0, 0, 0

  for i = 2, #points - 1 do
    local a, b, c = points[i - 1], points[i], points[i + 1]
    local first = vec3(b.x - a.x, b.y - a.y, 0)
    local second = vec3(c.x - b.x, c.y - b.y, 0)
    local firstLength, secondLength = first:length(), second:length()
    if firstLength > 0.1 and secondLength > 0.1 then
      local cosine = math.max(-1, math.min(1, first:dot(second) / (firstLength * secondLength)))
      local curvature = math.acos(cosine) / math.max(0.1, (firstLength + secondLength) * 0.5)
      if curvature > 0.0001 and 1 / curvature < minimumRadius then
        candidate.safe = false
        candidate.reasonCode = candidate.reasonCode or "turning_radius"
        candidate.reason = candidate.reason or "Trajectory exceeds the estimated vehicle turning limit"
      end
      curvatureChange = curvatureChange + math.abs(curvature - previousCurvature)
      previousCurvature = curvature
      maximumCurvature = math.max(maximumCurvature, curvature)
    end
  end

  candidate.minimumTurnRadius = minimumRadius
  candidate.maximumCurvature = maximumCurvature
  candidate.steeringChangeCost = curvatureChange
  candidate.minimumClearance = 12.0

  for i = 1, #points do
    local sample, direction = points[i], directions[i]
    local right = vec3(-direction.y, direction.x, 0)
    for j = 1, #obstacles do
      local position = obstacles[j]._position
      if position.z >= centerlineSamples[i].z + 0.15
          and position.z <= centerlineSamples[i].z + footprint.height + 0.50 then
        local delta = position - sample
        local longitudinalGap = math.max(0,
          math.abs(delta:dot(direction)) - footprint.halfLength - PLANNER_OBSTACLE_MARGIN_M)
        local lateralGap = math.max(0,
          math.abs(delta:dot(right)) - footprint.halfWidth - PLANNER_OBSTACLE_MARGIN_M)
        local clearance = math.sqrt(longitudinalGap * longitudinalGap + lateralGap * lateralGap)
        candidate.minimumClearance = math.min(candidate.minimumClearance, clearance)
        if longitudinalGap <= 0 and lateralGap <= 0 then
          candidate.safe = false
          candidate.reasonCode = candidate.reasonCode or "obstacle"
          candidate.reason = candidate.reason or "A scene-raycast obstacle overlaps the vehicle footprint"
        end
      end
    end

    for j = 1, #targets do
      local target = targets[j]
      local targetPosition, dimensions = target._position, target._dimensions
      local orientation = target._orientation
      local targetForward = orientation and normalizedVec3(orientation.forward) or direction
      local targetRight = orientation and normalizedVec3(orientation.right) or nil
      if not targetRight then targetRight = vec3(-targetForward.y, targetForward.x, 0) end
      if math.abs(targetPosition.z - sample.z)
          <= footprint.halfHeight + dimensions.height * 0.5 + 0.45 then
        local delta = targetPosition - sample
        local targetHalfLong = dimensions.length * 0.5 * math.abs(targetForward:dot(direction))
          + dimensions.width * 0.5 * math.abs(targetRight:dot(direction))
        local targetHalfLat = dimensions.length * 0.5 * math.abs(targetForward:dot(right))
          + dimensions.width * 0.5 * math.abs(targetRight:dot(right))
        local longitudinalGap = math.max(0,
          math.abs(delta:dot(direction)) - footprint.halfLength - targetHalfLong - PLANNER_OBSTACLE_MARGIN_M)
        local lateralGap = math.max(0,
          math.abs(delta:dot(right)) - footprint.halfWidth - targetHalfLat - PLANNER_OBSTACLE_MARGIN_M)
        local clearance = math.sqrt(longitudinalGap * longitudinalGap + lateralGap * lateralGap)
        candidate.minimumClearance = math.min(candidate.minimumClearance, clearance)
        if longitudinalGap <= 0 and lateralGap <= 0 then
          candidate.safe = false
          candidate.reasonCode = candidate.reasonCode or "vehicle"
          candidate.reason = candidate.reason or "A tracked vehicle overlaps the planned footprint"
        end
      end
    end
  end

  candidate.minimumClearance = math.min(12.0, candidate.minimumClearance)
  if candidate.safe then
    candidate.score = candidate.pathProgress * 0.1
      + candidate.minimumClearance * 1.35
      + candidate.minimumRoadMargin * 0.75
      - math.abs(candidate.targetOffset) * 1.5
      - maximumCurvature * 8.0
      - curvatureChange * 3.0
      - (candidate.laneChangeCost or 0)
  end
end

updateLocalTrajectoryPlanner = function()
  local previous = state.localPlanner or {}
  if previous.timestamp and state.simulationTime - previous.timestamp < PLANNER_INTERVAL then
    return previous
  end
  if not state.routePilotEnabled and not state.mapPlanningEnabled then
    return state._roadmindHelpers.setLocalPlannerState("idle", "inactive", "Map planning and route pilot are inactive", {}, nil)
  end
  local function waiting(code, reason)
    return state._roadmindHelpers.setLocalPlannerState("waiting", code, reason, {}, nil)
  end

  local now = state.simulationTime
  if state.frameAge > MAX_FRAME_AGE or type(state.latestData) ~= "table"
      or type(state.worldModel) ~= "table"
      or now - (tonumber(state.worldModel.timestamp) or -math.huge) > PLANNER_MAX_DATA_AGE then
    return waiting("sensor_stale", "Waiting for a fresh world-model snapshot")
  end

  local timestamps = state.latestData.sensorTimestamps or {}
  local settings = state.settings or DEFAULT_SETTINGS
  local function sourceFresh(key, enabled)
    local timestamp = tonumber(timestamps[key])
    return enabled == true and timestamp ~= nil
      and timestamp <= now + 0.05 and now - timestamp <= PLANNER_MAX_DATA_AGE
  end
  local freshSource = sourceFresh("lidar", settings.lidarEnabled)
    or sourceFresh("radar", settings.radarEnabled)
    or sourceFresh("ultrasonic", settings.ultrasonicEnabled)
  if not freshSource then
    return waiting("sensor_stale", "No enabled obstacle sensor has a fresh measurement")
  end

  if type(state.roadState) ~= "table" or state.roadState.valid ~= true
      or (state.roadState.quality ~= "high" and state.roadState.quality ~= "medium")
      or now - (tonumber(state.roadState.timestamp) or -math.huge) > PLANNER_MAX_DATA_AGE then
    return waiting("road_unavailable", "Road localization is not reliable enough for trajectory planning")
  end

  local laneModel = type(state.roadState) == "table" and state.roadState.laneModel or nil
  if type(laneModel) == "table" and laneModel.status == "direction-restricted" then
    return state._roadmindHelpers.setLocalPlannerState("blocked", "direction_restricted",
      "NavGraph does not define a forward lane on this directed road edge", {}, nil)
  end

  local footprint = state._roadmindHelpers.getPlannerVehicleFootprint()
  if not footprint then
    return waiting("vehicle_geometry", "Calibrated full vehicle footprint is unavailable")
  end
  local routeReady, routeError = state._roadmindHelpers.ensurePlannerRoutePath()
  if not routeReady then
    return waiting("route_unavailable", routeError or "No valid route corridor is available")
  end

  local startIndex, projection, distanceToPath =
    state._roadmindHelpers.findClosestPlannerRouteProjection(state.routePath, footprint.center)
  if not startIndex or not projection then
    return waiting("route_unavailable", "Could not project the vehicle onto the route")
  end
  local first, second = asVec3(state.routePath[startIndex]), asVec3(state.routePath[startIndex + 1])
  local initialSegment = first and second
    and vec3(second.x - first.x, second.y - first.y, 0) or nil
  if not initialSegment or initialSegment:length() < 0.1 then
    return waiting("route_unavailable", "The route has no usable forward direction")
  end
  local routeForward = initialSegment:normalized()
  local routeRight = vec3(-routeForward.y, routeForward.x, 0)
  local currentOffset = (footprint.center - projection):dot(routeRight)
  local vehicleCenterHeight = footprint.center.z - projection.z

  local centerlineSamples = {}
  for sampleIndex = 0, PLANNER_MAX_SAMPLES - 1 do
    local distance = sampleIndex * PLANNER_SAMPLE_STEP_M
    if distance > PLANNER_HORIZON_M then break end
    local centerline, tangent = state._roadmindHelpers.samplePlannerRoute(state.routePath, startIndex, projection, distance)
    if not centerline or not tangent then break end
    centerlineSamples[#centerlineSamples + 1] = {
      centerline = centerline, tangent = tangent, distance = distance, z = centerline.z
    }
  end
  if #centerlineSamples < 4 then
    return waiting("route_unavailable", "The NavGraph path is shorter than the planning horizon")
  end

  local nearbyObstacles = {}
  for i = 1, #(state.worldModel.obstacles or {}) do
    local obstacle = state.worldModel.obstacles[i]
    local position = asVec3(obstacle.position)
    local timestamp = tonumber(obstacle.timestamp)
    local age = timestamp and now - timestamp or math.huge
    if position and age >= 0 and age <= PLANNER_MAX_DATA_AGE
        and (position - footprint.center):length() <= PLANNER_HORIZON_M + 12 then
      nearbyObstacles[#nearbyObstacles + 1] = {_position = position}
      if #nearbyObstacles >= PLANNER_MAX_OBSTACLES then break end
    end
  end

  local nearbyTargets = {}
  for i = 1, #(state.worldModel.targets or {}) do
    local target = state.worldModel.targets[i]
    local targetPosition = target.status == "coasting"
      and asVec3(target.predictedPosition) or asVec3(target.position)
    local timestamp = tonumber(target.timestamp)
    local age = timestamp and now - timestamp or math.huge
    local dimensions = target.dimensions
    if targetPosition and age >= 0 and age <= PLANNER_MAX_DATA_AGE
        and (target.status == "confirmed" or target.status == "coasting")
        and (tonumber(target.confidence) or 0) >= 0.25
        and (targetPosition - footprint.center):length() <= PLANNER_HORIZON_M + 12 then
      local length = type(dimensions) == "table" and tonumber(dimensions.length) or 4.5
      local width = type(dimensions) == "table" and tonumber(dimensions.width) or 2.0
      local height = type(dimensions) == "table" and tonumber(dimensions.height) or 1.6
      nearbyTargets[#nearbyTargets + 1] = {
        _position = targetPosition,
        _dimensions = {
          length = math.max(1.0, math.min(30, length)),
          width = math.max(0.5, math.min(10, width)),
          height = math.max(0.45, math.min(9, height))
        },
        _orientation = target.orientation
      }
      if #nearbyTargets >= PLANNER_MAX_TARGETS then break end
    end
  end

  -- Prefer lane records from the connected local graph. Candidate lane changes
  -- are limited to an immediately adjacent lane and only considered when the
  -- current-lane localization is explicitly reliable.
  local candidateOffsets = {}
  local laneModel = state.roadState.laneModel or {}
  local laneGraph = state.roadState.laneGraph or {}
  local laneLocalization = state.roadState.laneLocalization or {}
  local roadStartNode = tostring(state.roadState.segmentStartNode or "")
  local roadEndNode = tostring(state.roadState.segmentEndNode or "")
  local graphSegment = nil
  for i = 1, #(laneGraph.segments or {}) do
    local segment = laneGraph.segments[i]
    if tostring(segment.from or "") == roadStartNode
        and tostring(segment.to or "") == roadEndNode then
      graphSegment = segment
      break
    end
  end

  -- Lane offsets are defined relative to the directed NavGraph edge. Convert
  -- graph lane centre points into the route's local right-vector frame so curved
  -- or slightly offset route corridors do not silently flip/shift lane choices.
  local graphSegmentProgress = nil
  if graphSegment then
    local graphStart = asVec3(graphSegment.start)
    local graphFinish = asVec3(graphSegment.finish)
    if graphStart and graphFinish then
      local graphDelta = vec3(graphFinish.x - graphStart.x, graphFinish.y - graphStart.y, 0)
      local graphLengthSquared = graphDelta:dot(graphDelta)
      if graphLengthSquared > 0.25 then
        local toVehicle = vec3(
          footprint.center.x - graphStart.x,
          footprint.center.y - graphStart.y,
          0
        )
        graphSegmentProgress = math.max(0, math.min(1,
          toVehicle:dot(graphDelta) / graphLengthSquared))
      end
    end
  end

  local laneOffsetSign = 1
  local graphForward = asVec3(state.roadState.roadDirection)
  if graphForward then
    local flatGraphForward = vec3(graphForward.x, graphForward.y, 0)
    if flatGraphForward:length() > 0.1
        and flatGraphForward:normalized():dot(routeForward) < 0 then
      laneOffsetSign = -1
    end
  end

  local laneRecords = {}
  local function addLaneRecord(laneId, laneIndex, laneOffset, laneWidth, laneStartPoint, laneFinishPoint)
    laneIndex = tonumber(laneIndex)
    laneOffset = tonumber(laneOffset)
    laneWidth = tonumber(laneWidth)
    if laneIndex and laneOffset then
      local routeOffset = laneOffset * laneOffsetSign
      local laneStart = asVec3(laneStartPoint)
      local laneFinish = asVec3(laneFinishPoint)
      if laneStart and laneFinish and graphSegmentProgress ~= nil then
        local lanePoint = laneStart + (laneFinish - laneStart) * graphSegmentProgress
        routeOffset = (lanePoint - projection):dot(routeRight)
      end
      laneRecords[#laneRecords + 1] = {
        id = laneId and tostring(laneId) or nil,
        index = laneIndex,
        offset = routeOffset,
        width = laneWidth
      }
    end
  end

  if graphSegment and type(graphSegment.lanes) == "table" then
    for i = 1, #graphSegment.lanes do
      local lane = graphSegment.lanes[i]
      if type(lane) == "table" then
        addLaneRecord(lane.id, lane.index, lane.centerOffset, lane.width, lane.start, lane.finish)
      end
    end
  end
  if #laneRecords == 0 and laneModel.status == "metadata"
      and type(laneModel.forwardLaneOffsets) == "table" then
    for i = 1, #laneModel.forwardLaneOffsets do
      addLaneRecord(
        tostring(state.roadState.segmentStartNode or "") .. ">"
          .. tostring(state.roadState.segmentEndNode or "") .. ":lane:" .. tostring(i),
        i, laneModel.forwardLaneOffsets[i], laneModel.laneWidth
      )
    end
  end

  local currentLaneId = laneGraph.currentLaneId or laneLocalization.laneId
  local currentLaneIndex = tonumber(laneLocalization.laneIndex)
  local closestLane, closestLaneError = nil, math.huge
  for i = 1, #laneRecords do
    local lane = laneRecords[i]
    local laneError = math.abs(lane.offset - currentOffset)
    if lane.id and currentLaneId and lane.id == tostring(currentLaneId) then
      currentLaneIndex = lane.index
      closestLane, closestLaneError = lane, laneError
      break
    end
    if laneError < closestLaneError then
      closestLane, closestLaneError = lane, laneError
    end
  end

  local lanePositionVerified = laneGraph.currentLaneStatus == "localized"
    or laneLocalization.status == "localized"
  if currentLaneIndex == nil and closestLane then
    local nearestWidth = closestLane.width or tonumber(laneModel.laneWidth) or 3.5
    if closestLaneError <= math.max(0.75, nearestWidth * 0.75) then
      currentLaneIndex = closestLane.index
      currentLaneId = closestLane.id
    end
  end

  if currentLaneIndex ~= nil then
    for i = 1, #laneRecords do
      local lane = laneRecords[i]
      local laneDelta = math.abs(lane.index - currentLaneIndex)
      local isCurrentLane = laneDelta == 0
      local isAdjacentLane = laneDelta == 1
      if isCurrentLane or (lanePositionVerified and isAdjacentLane) then
        local laneChange = not isCurrentLane
        candidateOffsets[#candidateOffsets + 1] = {
          offset = lane.offset,
          laneId = lane.id,
          laneIndex = lane.index,
          maneuver = laneChange and "adjacent-lane-change" or "keep-lane",
          laneChange = laneChange,
          laneChangeLegality = laneChange and "unverified" or "not-applicable",
          laneWidth = lane.width
        }
      end
    end
  elseif closestLane then
    -- Without a reliable lane localization, retain only the nearest plausible
    -- lane as a stay-in-lane preview; do not suggest a lane change.
    candidateOffsets[#candidateOffsets + 1] = {
      offset = closestLane.offset,
      laneId = closestLane.id,
      laneIndex = closestLane.index,
      maneuver = "keep-lane-unlocalized",
      laneChange = false,
      laneChangeLegality = "not-applicable",
      laneWidth = closestLane.width
    }
  end

  table.sort(candidateOffsets, function(a, b)
    local aDistance = math.abs(a.offset - currentOffset)
    local bDistance = math.abs(b.offset - currentOffset)
    if aDistance == bDistance then
      return (a.laneIndex or math.huge) < (b.laneIndex or math.huge)
    end
    return aDistance < bDistance
  end)

  if #candidateOffsets == 0 then
    -- Preserve the conservative corridor-offset fallback when NavGraph lane
    -- metadata is absent or cannot be associated with the current segment.
    for i = 1, #PLANNER_CANDIDATE_OFFSETS do
      candidateOffsets[#candidateOffsets + 1] = {
        offset = PLANNER_CANDIDATE_OFFSETS[i],
        maneuver = "corridor-offset-preview",
        laneChange = false,
        laneChangeLegality = "unknown"
      }
    end
  end

  local candidates, selectedIndex, bestScore = {}, nil, -math.huge
  local reasons = {obstacle = 0, vehicle = 0, road_edge = 0, turning_radius = 0}
  for candidateIndex = 1, math.min(PLANNER_MAX_CANDIDATES, #candidateOffsets) do
    local laneChoice = candidateOffsets[candidateIndex]
    local targetOffset = laneChoice.offset
    local laneChange = laneChoice.laneChange == true
    local candidate = {
      index = candidateIndex, targetOffset = targetOffset, safe = true, path = {},
      minimumClearance = 12.0, minimumRoadMargin = math.huge, pathProgress = 0,
      laneId = laneChoice.laneId, laneIndex = laneChoice.laneIndex,
      maneuver = laneChoice.maneuver or "corridor-offset-preview",
      laneChange = laneChange,
      laneChangeLegality = laneChoice.laneChangeLegality or "unknown",
      laneChangeCost = laneChange and 4.0 or 0.0
    }
    for sampleIndex = 1, #centerlineSamples do
      local sample = centerlineSamples[sampleIndex]
      local ramp = state._roadmindHelpers.smoothPlannerRamp(sample.distance)
      local lateralOffset = currentOffset * (1 - ramp) + targetOffset * ramp
      local normal = vec3(-sample.tangent.y, sample.tangent.x, 0)
      local point
      if sampleIndex == 1 then
        point = footprint.center
      else
        point = sample.centerline + normal * lateralOffset + vec3(0, 0, vehicleCenterHeight)
      end
      candidate.path[#candidate.path + 1] = vectorToPlainTable(point)
      candidate.pathProgress = sample.distance
      local projectedHalfWidth = footprint.halfLength * math.abs(footprint.forward:dot(normal))
        + footprint.halfWidth * math.abs(footprint.right:dot(normal))
      local roadHalfWidth = tonumber(state.roadState.roadHalfWidth) or PLANNER_ROAD_HALF_WIDTH_M
      roadHalfWidth = math.max(2.0, math.min(20.0, roadHalfWidth))
      local roadMargin = roadHalfWidth - math.abs(lateralOffset) - projectedHalfWidth
      candidate.minimumRoadMargin = math.min(candidate.minimumRoadMargin, roadMargin)
      if roadMargin < PLANNER_ROAD_EDGE_MARGIN_M then
        candidate.safe = false
        candidate.reasonCode = candidate.reasonCode or "road_edge"
        candidate.reason = candidate.reason or "The full vehicle width exceeds the conservative NavGraph corridor"
      end
    end

    if candidate.minimumRoadMargin == math.huge then candidate.minimumRoadMargin = 0 end
    state._roadmindHelpers.evaluatePlannerCandidate(candidate, footprint, nearbyObstacles, nearbyTargets, centerlineSamples)
    if candidate.safe then
      if candidate.score > bestScore then bestScore, selectedIndex = candidate.score, candidateIndex end
    else
      local code = candidate.reasonCode or "obstacle"
      reasons[code] = (reasons[code] or 0) + 1
    end
    candidates[#candidates + 1] = candidate
  end

  if selectedIndex then
    local selected = candidates[selectedIndex]
    return state._roadmindHelpers.setLocalPlannerState("ready", "selected", "A safe bounded trajectory was selected",
      candidates, selectedIndex, {
        selectedScore = round2(selected.score),
        minimumClearance = round2(selected.minimumClearance),
        minimumRoadMargin = round2(selected.minimumRoadMargin),
        selectedOffset = round2(selected.targetOffset),
        selectedLaneId = selected.laneId,
        selectedLaneIndex = selected.laneIndex,
        selectedManeuver = selected.maneuver,
        selectedLaneChange = selected.laneChange == true,
        selectedLaneChangeLegality = selected.laneChangeLegality,
        laneLocalizationStatus = laneLocalization.status or laneGraph.currentLaneStatus or "unknown",
        laneGraphStatus = laneGraph.status or "unavailable",
        routeProjectionDistance = round2(distanceToPath),
        sampleCount = #selected.path
      })
  end

  local reasonCode = "no_safe_candidate"
  if reasons.road_edge == #candidates then
    reasonCode = "road_edge"
  elseif reasons.turning_radius == #candidates then
    reasonCode = "turning_radius"
  elseif reasons.vehicle > 0 and reasons.obstacle == 0 then
    reasonCode = "vehicle"
  elseif reasons.obstacle > 0 then
    reasonCode = "obstacle"
  end
  local minimumClearance, minimumRoadMargin = 12.0, math.huge
  for i = 1, #candidates do
    minimumClearance = math.min(minimumClearance, candidates[i].minimumClearance or 12)
    minimumRoadMargin = math.min(minimumRoadMargin, candidates[i].minimumRoadMargin or 0)
  end
  return state._roadmindHelpers.setLocalPlannerState("blocked", reasonCode,
    "No candidate passed the footprint, obstacle, road-corridor and turning checks",
    candidates, nil, {
      minimumClearance = round2(minimumClearance),
      minimumRoadMargin = minimumRoadMargin == math.huge and nil or round2(minimumRoadMargin),
      routeProjectionDistance = round2(distanceToPath),
      sampleCount = #centerlineSamples
    })
end
state._roadmindHelpers.refreshVehicleRegistry = function()
  local registry = {}
  local examined = 0
  local truncated = false
  local iteratorAvailable = false

  local function addVehicle(vehicleId, vehicle)
    examined = examined + 1
    if examined > MAX_REGISTRY_VEHICLES * 4 then
      truncated = true
      return false
    end

    vehicleId = vehicleId or getVehicleId(vehicle)
    if vehicleId and tostring(vehicleId) ~= tostring(state.activeVehicleId)
        and safeCall(vehicle, "isDeleted") ~= true
        and safeCall(vehicle, "isDestroyed") ~= true then
      local entry = state._roadmindHelpers.snapshotVehicleBounds(vehicle, vehicleId)
      if entry then
        registry[#registry + 1] = entry
        if #registry >= MAX_REGISTRY_VEHICLES then
          truncated = true
          return false
        end
      end
    end
    return true
  end

  -- BeamNG's iterator reuses the engine vehicle cache and avoids creating a
  -- temporary list/table for every registry refresh.
  if type(vehiclesIterator) == "function" then
    local okIterator, iterator, context = pcall(vehiclesIterator, state.vehicleIteratorContext)
    if okIterator and type(iterator) == "function" then
      state.vehicleIteratorContext = context or state.vehicleIteratorContext
      iteratorAvailable = true
      for vehicleId, vehicle in iterator, state.vehicleIteratorContext do
        if not addVehicle(vehicleId, vehicle) then break end
      end
    end
  end

  -- Compatibility paths for older/custom game environments.
  if not iteratorAvailable then
    local vehicles = nil
    if type(getAllVehicles) == "function" then
      local ok, result = pcall(getAllVehicles)
      if ok and type(result) == "table" then vehicles = result end
    end

    if type(vehicles) == "table" then
      for i = 1, #vehicles do
        if not addVehicle(nil, vehicles[i]) then break end
      end
      iteratorAvailable = true
    elseif be and type(be.getObjectCount) == "function"
        and type(be.getObject) == "function" then
      local okCount, count = pcall(be.getObjectCount, be)
      if okCount and type(count) == "number" then
        iteratorAvailable = true
        for index = 0, math.min(count - 1, MAX_REGISTRY_VEHICLES * 2) do
          local okObject, vehicle = pcall(be.getObject, be, index)
          if okObject and vehicle and not addVehicle(nil, vehicle) then
            truncated = true
            break
          end
        end
      end
    end
  end

  if not iteratorAvailable then
    state.vehicleRegistry = {}
    state.vehicleRegistryById = {}
    state.vehicleRegistrySnapshotValid = false
    state.vehicleRegistryTimestamp = nil
    state.vehicleRegistryTruncated = false
    state.vehicleRegistryGeneration = state.vehicleRegistryGeneration + 1
    return false
  end

  local registryById = {}
  for i = 1, #registry do
    registryById[tostring(registry[i].id)] = true
  end

  state.vehicleRegistry = registry
  state.vehicleRegistryById = registryById
  state.vehicleRegistrySnapshotValid = true
  state.vehicleRegistryTimestamp = state.simulationTime
  state.vehicleRegistryTruncated = truncated
  state.vehicleRegistryGeneration = state.vehicleRegistryGeneration + 1
  return true
end

state._roadmindHelpers.projectedBoxRadius = function(target, axis)
  return target.half[1] * math.abs(target.axes[1]:dot(axis))
    + target.half[2] * math.abs(target.axes[2]:dot(axis))
    + target.half[3] * math.abs(target.axes[3]:dot(axis))
end

state._roadmindHelpers.buildSensorZones = function(data, settings)
  local zones = {}
  local source = type(data.detectionZones) == "table" and data.detectionZones.zones or nil
  if type(source) ~= "table" then return zones end

  for i = 1, #source do
    local raw = source[i]
    local enabled = (raw.sensor == "camera" and settings.cameraDetectionEnabled == true)
      or (raw.sensor == "lidar" and settings.lidarEnabled)
      or (raw.sensor == "radar" and settings.radarEnabled)
      or (raw.sensor == "ultrasonic" and settings.ultrasonicEnabled)
    if enabled then
      local zone = {
        name = raw.name or (raw.sensor .. " " .. i),
        sensor = raw.sensor,
        origin = asVec3(raw.origin),
        forward = normalizedVec3(raw.forward),
        right = normalizedVec3(raw.right),
        up = normalizedVec3(raw.up),
        range = tonumber(raw.range),
        minRange = tonumber(raw.minRange) or 0,
        horizontalFov = tonumber(raw.horizontalFov),
        verticalFov = tonumber(raw.verticalFov)
      }
      if zone.origin and zone.forward and zone.right and zone.up
          and zone.range and zone.range > zone.minRange
          and zone.horizontalFov and zone.horizontalFov > 0
          and zone.verticalFov and zone.verticalFov > 0 then
        zones[#zones + 1] = zone
      end
    end
  end

  return zones
end

state._roadmindHelpers.targetInsideSensorZone = function(target, zone)
  local delta = target.center - zone.origin
  local distanceSquared = delta:dot(delta)
  local maxDistance = zone.range + target.radius
  if distanceSquared > maxDistance * maxDistance then return false end

  local along = delta:dot(zone.forward)
  local lateral = delta:dot(zone.right)
  local vertical = delta:dot(zone.up)

  local radiusForward = state._roadmindHelpers.projectedBoxRadius(target, zone.forward)
  local radiusRight = state._roadmindHelpers.projectedBoxRadius(target, zone.right)
  local radiusUp = state._roadmindHelpers.projectedBoxRadius(target, zone.up)

  local nearDistance = along - radiusForward
  local farDistance = along + radiusForward
  if farDistance < zone.minRange or nearDistance > zone.range then return false end

  local usableDepth = math.max(0, math.min(zone.range, farDistance))
  local horizontalHalfWidth = usableDepth * math.tan(math.rad(zone.horizontalFov * 0.5))
  local verticalHalfHeight = usableDepth * math.tan(math.rad(zone.verticalFov * 0.5))
  if math.abs(lateral) > horizontalHalfWidth + radiusRight then return false end
  if math.abs(vertical) > verticalHalfHeight + radiusUp then return false end

  return true, math.sqrt(distanceSquared), nearDistance, farDistance
end

state._roadmindHelpers.checkTargetOcclusion = function(zone, target)
  if type(castRay) ~= "function" then return "unavailable", false end
  local delta = target.center - zone.origin
  local centerDistance = delta:length()
  if centerDistance < 0.01 then return "unverified", false end

  -- The ray starts at this exact sensor mount (for cameras: the camera lens)
  -- and ends inside the candidate vehicle. Only a hit on that same vehicle
  -- confirms visibility; a nearby hit is never treated as a positive result.
  local ok, hit = pcall(castRay, zone.origin, target.center, true, true)
  if not ok or not hit then return "unverified", true end

  local hitDistance = tonumber(safeField(hit, "dist"))
  local hitObject = safeField(hit, "obj") or safeField(hit, "object")
  local hitObjectId = getVehicleId(hitObject)
  if hitObjectId == nil and (type(hitObject) == "number" or type(hitObject) == "string") then
    hitObjectId = hitObject
  end
  if hitObject == target.vehicleObject
      or (hitObjectId ~= nil and tostring(hitObjectId) == tostring(target.id)) then
    return "visible", true
  end

  -- Any other identified scene object is the first thing seen by the sensor,
  -- so the candidate is behind an occluder (another vehicle, wall or prop).
  if hitObject ~= nil then return "occluded", true end

  local rayDirection = delta / centerDistance
  local expectedTargetSurface = math.max(0,
    centerDistance - state._roadmindHelpers.projectedBoxRadius(target, rayDirection))
  if hitDistance and hitDistance < expectedTargetSurface - 0.10 then
    return "occluded", true
  end

  -- A ray without a reliably identified target hit is not proof of visibility.
  return "unverified", true
end

state._roadmindHelpers.detectNearbyVehicles = function(data)
  local candidates = {}
  local settings = state.settings or DEFAULT_SETTINGS
  if not state.activeVehicle or not state._roadmindHelpers.vehicleDetectionEnabled(settings) then
    state.vehicleCandidates = {}
    state.occlusionRaycastsLastCycle = 0
    state.occlusionRaycastsPerSecond = 0
    return candidates
  end

  local zones = state._roadmindHelpers.buildSensorZones(data, settings)
  if #zones == 0 then
    state.vehicleCandidates = {}
    state.occlusionRaycastsLastCycle = 0
    state.occlusionRaycastsPerSecond = 0
    return candidates
  end

  local playerVelocity = safeCall(state.activeVehicle, "getVelocity")
  for i = 1, #state.vehicleRegistry do
    local target = state.vehicleRegistry[i]
    local sources = {}
    local cameraSources = {}
    local nearestDistance = math.huge
    local nearestZone = nil
    local nearestCameraDistance = math.huge
    local nearestCameraZone = nil
    local nearDistance = math.huge
    local farDistance = 0
    local cameraNearDistance = math.huge
    local cameraFarDistance = 0

    for zoneIndex = 1, #zones do
      local zone = zones[zoneIndex]
      local inside, distance, near, far = state._roadmindHelpers.targetInsideSensorZone(target, zone)
      if inside then
        local source = {sensor = zone.sensor, name = zone.name}
        sources[#sources + 1] = source
        if zone.sensor == "camera" then
          cameraSources[#cameraSources + 1] = source
          if distance < nearestCameraDistance then
            nearestCameraDistance = distance
            nearestCameraZone = zone
          end
          cameraNearDistance = math.min(cameraNearDistance, near)
          cameraFarDistance = math.max(cameraFarDistance, far)
        end
        if distance < nearestDistance then
          nearestDistance = distance
          nearestZone = zone
        end
        nearDistance = math.min(nearDistance, near)
        farDistance = math.max(farDistance, far)
      end
    end

    -- When a target is inside any camera frustum, create a camera-only
    -- observation. Radar/LiDAR/ultrasonic remain independent obstacle sensors
    -- and must not steal the vehicle-detection source attribution.
    local hasCameraSource = #cameraSources > 0
    if hasCameraSource then
      sources = cameraSources
      nearestDistance = nearestCameraDistance
      nearestZone = nearestCameraZone
      nearDistance = cameraNearDistance
      farDistance = cameraFarDistance
    end

    if #sources > 0 and nearestZone then
      local relativeSpeed = nil
      if playerVelocity then
        local relativeVelocity = target.velocity - playerVelocity
        local delta = target.center - nearestZone.origin
        if delta:length() > 0.001 then
          relativeSpeed = round2(relativeVelocity:dot(delta:normalized()))
        end
      end

      candidates[#candidates + 1] = {
        id = target.id,
        sourceVehicleId = target.id,
        vehicleObject = target.vehicleObject,
        centerOffsetLocal = target.centerOffsetLocal,
        position = {x = target.center.x, y = target.center.y, z = target.center.z},
        simulatorVelocity = {x = target.velocity.x, y = target.velocity.y, z = target.velocity.z},
        orientation = {
          forward = {x = target.forward.x, y = target.forward.y, z = target.forward.z},
          right = {x = target.right.x, y = target.right.y, z = target.right.z},
          up = {x = target.up.x, y = target.up.y, z = target.up.z}
        },
        dimensions = {
          length = target.dimensions.length,
          width = target.dimensions.width,
          height = target.dimensions.height
        },
        registryTimestamp = target.registryTimestamp,
        observationTimestamp = target.registryTimestamp,
        distance = round2(nearestDistance),
        nearDistance = round2(nearDistance),
        farDistance = round2(farDistance),
        relativeSpeedEstimate = relativeSpeed,
        sensorOrigin = {x = nearestZone.origin.x, y = nearestZone.origin.y, z = nearestZone.origin.z},
        bestSensorName = nearestZone.name,
        sensorSources = sources,
        sourceKind = hasCameraSource and "simulator-assisted-camera-visibility" or "simulator-assisted",
        measurementKind = hasCameraSource
          and "camera-frustum-and-line-of-sight-vehicle-OBB" or "vehicle OBB registry",
        visibility = "unverified",
        registryGeneration = state.vehicleRegistryGeneration,
        registryIndex = i
      }
    end
  end

  table.sort(candidates, function(a, b) return a.distance < b.distance end)

  local zonesByName = {}
  for i = 1, #zones do zonesByName[zones[i].name] = zones[i] end

  -- Reuse a recent visibility result only while the same OBB has moved very little.
  for i = 1, #candidates do
    local candidate = candidates[i]
    local target = state.vehicleRegistry[candidate.registryIndex]
    local cacheKey = tostring(candidate.sourceVehicleId)
    local cached = state.occlusionCache[cacheKey]
    candidate.visibility = "unverified"

    local cachedSourceStillPresent = false
    if cached and type(cached.sensorName) == "string" then
      for sourceIndex = 1, #candidate.sensorSources do
        if candidate.sensorSources[sourceIndex].name == cached.sensorName then
          cachedSourceStillPresent = true
          break
        end
      end
    end
    local cacheMaxAge = candidate.sourceKind == "simulator-assisted-camera-visibility"
      and CAMERA_OCCLUSION_CACHE_MAX_AGE or OCCLUSION_CACHE_MAX_AGE
    if target and cached and cachedSourceStillPresent
        and state.simulationTime - cached.timestamp <= cacheMaxAge
        and cached.position and (target.center - cached.position):length() <= 1.25 then
      candidate.visibility = cached.visibility
      candidate.visibilityTimestamp = cached.timestamp
      candidate.visibleFrom = cached.sensorName
      candidate.visibilityAge = math.max(0, state.simulationTime - cached.timestamp)
      local cachedZone = zonesByName[cached.sensorName]
      if cachedZone then
        candidate.sensorOrigin = vectorToPlainTable(cachedZone.origin)
      end
    end
  end

  local checked = 0
  local count = #candidates
  local cursor = count > 0 and ((state.occlusionCursor - 1) % count) + 1 or 1
  local nextCursor = cursor

  -- Rotate the starting candidate so traffic outside the nearest four is not starved.
  for offset = 0, count - 1 do
    if checked >= MAX_OCCLUSION_CHECKS_PER_CYCLE then break end
    local index = ((cursor + offset - 1) % count) + 1
    local candidate = candidates[index]
    local target = state.vehicleRegistry[candidate.registryIndex]
    if target and candidate.visibility ~= "visible" then
      local cacheKey = tostring(candidate.sourceVehicleId)
      local allOccluded = true
      local anyRaycastResult = false
      local anyUnverified = false
      local sourcesChecked = 0
      local lastSensorName = nil
      local sourceCount = #candidate.sensorSources
      local sourceCursor = state.occlusionSourceCursor[cacheKey] or 1
      if sourceCount > 0 then sourceCursor = ((sourceCursor - 1) % sourceCount) + 1 end

      -- Rotate source-camera checks as well as target checks. Check no more
      -- than two source zones per target and eight raycasts globally per cycle.
      for sourceOffset = 0, sourceCount - 1 do
        if checked >= MAX_OCCLUSION_CHECKS_PER_CYCLE or sourcesChecked >= 2 then
          anyUnverified = true
          break
        end
        local sourceIndex = ((sourceCursor + sourceOffset - 1) % sourceCount) + 1
        local source = candidate.sensorSources[sourceIndex]
        local zone = zonesByName[source.name]
        if zone then
          local visibility, didRaycast = state._roadmindHelpers.checkTargetOcclusion(zone, target)
          if didRaycast then
            checked = checked + 1
            sourcesChecked = sourcesChecked + 1
            anyRaycastResult = true
          end
          lastSensorName = zone.name
          state.occlusionSourceCursor[cacheKey] = (sourceIndex % math.max(1, sourceCount)) + 1
          if visibility == "visible" then
            candidate.visibility = "visible"
            candidate.visibleFrom = zone.name
            candidate.sensorOrigin = vectorToPlainTable(zone.origin)
            candidate.sourceKind = zone.sensor == "camera"
              and "simulator-assisted-camera-visibility" or "simulator-assisted"
            candidate.measurementKind = zone.sensor == "camera"
              and "camera-frustum-and-line-of-sight-vehicle-OBB" or "vehicle OBB registry"
            allOccluded = false
            anyUnverified = false
            break
          elseif visibility == "occluded" then
            anyRaycastResult = true
          else
            anyUnverified = true
            allOccluded = false
          end
        else
          anyUnverified = true
          allOccluded = false
        end
      end

      if candidate.visibility ~= "visible" then
        candidate.visibility = allOccluded and anyRaycastResult and not anyUnverified
          and "occluded" or "unverified"
      end

      if anyRaycastResult then
        local key = tostring(candidate.sourceVehicleId)
        state.occlusionCache[key] = {
          visibility = candidate.visibility,
          timestamp = state.simulationTime,
          position = vec3(target.center),
          sensorName = candidate.visibleFrom or lastSensorName
        }
        candidate.visibilityTimestamp = state.simulationTime
        candidate.visibilityAge = 0
      end
      nextCursor = (index % count) + 1
    end
  end

  state.occlusionCursor = nextCursor
  for key, cached in pairs(state.occlusionCache) do
    if not state.vehicleRegistryById[key]
        or state.simulationTime - cached.timestamp > TRACK_MAX_AGE then
      state.occlusionCache[key] = nil
    end
  end

  state.vehicleCandidates = candidates
  state.occlusionRaycastsLastCycle = checked
  state.occlusionRaycastsPerSecond = checked / math.max(
    0.01, state.currentTrafficUpdateInterval or VEHICLE_DETECTION_INTERVAL
  )
  return candidates
end

state._roadmindHelpers.getTrafficUpdateInterval = function()
  local settings = state.settings or DEFAULT_SETTINGS
  if settings.speedAdaptiveRateEnabled == false then
    return VEHICLE_DETECTION_INTERVAL
  end
  local velocity = state.activeVehicle
    and asVec3(safeCall(state.activeVehicle, "getVelocity")) or nil
  local speed = velocity and velocity:length() or 0
  if speed < 5 then return VEHICLE_SLOW_UPDATE_INTERVAL end
  if speed >= 20 then return VEHICLE_FAST_UPDATE_INTERVAL end
  return VEHICLE_DETECTION_INTERVAL
end

state._roadmindHelpers.updateVehicleDetection = function(dt)
  local elapsed = math.max(0, tonumber(dt) or 0)
  local settings = state.settings or DEFAULT_SETTINGS
  local updateInterval = state._roadmindHelpers.getTrafficUpdateInterval()
  state.currentTrafficUpdateInterval = updateInterval
  if not state.activeVehicle or not state._roadmindHelpers.vehicleDetectionEnabled(settings) then
    state.vehicleRegistry = {}
    state.vehicleRegistryById = {}
    state.vehicleRegistrySnapshotValid = false
    state.vehicleRegistryTimestamp = nil
    state.vehicleCandidates = {}
    state.vehicleRegistryTimer = 0
    state.vehicleRegistryTruncated = false
    state.vehicleDetectionTimer = 0
    state.occlusionCache = {}
    state.occlusionCursor = 1
    state.occlusionRaycastsLastCycle = 0
    state.occlusionRaycastsPerSecond = 0
    state._roadmindHelpers.resetTrackingState(false)
    return
  end

  state.vehicleRegistryTimer = state.vehicleRegistryTimer + elapsed
  if state.vehicleRegistryTimer >= updateInterval then
    state.vehicleRegistryTimer = state.vehicleRegistryTimer % updateInterval
    state._roadmindHelpers.refreshVehicleRegistry()
  end

  state.vehicleDetectionTimer = state.vehicleDetectionTimer + elapsed
  if state.vehicleDetectionTimer < updateInterval then return end
  state.vehicleDetectionTimer = state.vehicleDetectionTimer % updateInterval

  local candidates = {}
  local zoneTimestamp = state.latestData and state.latestData.sensorTimestamps
    and state.latestData.sensorTimestamps.detectionZones or nil
  local zonesFresh = type(zoneTimestamp) == "number"
    and (state.simulationTime - zoneTimestamp) <= TRACK_OBSERVATION_MAX_AGE

  if state.latestData and zonesFresh then
    candidates = state._roadmindHelpers.detectNearbyVehicles(state.latestData)
  else
    state.vehicleCandidates = {}
    state.occlusionRaycastsLastCycle = 0
    state.occlusionRaycastsPerSecond = 0
  end

  local tracks = state._roadmindHelpers.updateObjectTracks(candidates)
  if state.latestData then
    state.latestData.vehicles = candidates
    state.latestData.tracks = tracks
    state.worldModelDirty = true
  end
end

state._roadmindHelpers.nearestForwardObstacle = function(data)
  if type(data) ~= "table" or not state.activeVehicle then return math.huge end

  local settings = state.settings or DEFAULT_SETTINGS
  local timestamps = data.sensorTimestamps or {}
  local now = state.simulationTime

  local function sourceFresh(key, enabled)
    local timestamp = timestamps[key]
    return enabled == true and type(timestamp) == "number"
      and timestamp <= now + 0.05 and (now - timestamp) <= MAX_FRAME_AGE
  end

  local lidarFresh = sourceFresh("lidar", settings.lidarEnabled)
  local radarFresh = sourceFresh("radar", settings.radarEnabled)
  local ultrasonicFresh = sourceFresh("ultrasonic", settings.ultrasonicEnabled)

  -- Use the current vehicle basis for the forward corridor. A cached sensor
  -- pose must not rotate the safety corridor after the vehicle has moved.
  local forward = normalizedVec3(safeCall(state.activeVehicle, "getDirectionVector"))
  local up = normalizedVec3(safeCall(state.activeVehicle, "getDirectionVectorUp"))
  local right = forward and up and normalizedVec3(forward:cross(up)) or nil
  local originFallback = asVec3(safeCall(state.activeVehicle, "getPosition"))
  if not forward or not right or not originFallback then return math.huge end

  local playerVelocity = asVec3(safeCall(state.activeVehicle, "getVelocity"))
  local ownForwardSpeed = playerVelocity and math.max(0, playerVelocity:dot(forward)) or 0
  local minimum = math.huge
  local nearestHazard = nil

  local function recordHazard(distance, details)
    if type(distance) == "number" and distance > 0 and distance < minimum then
      minimum = distance
      nearestHazard = details
    end
  end

  local function inspect(point, originData)
    local target = state._roadmindHelpers.vectorFromTable(point)
    local origin = state._roadmindHelpers.vectorFromTable(originData) or originFallback
    if not target or not origin then return end
    local delta = target - origin
    local longitudinal = delta:dot(forward)
    local lateral = math.abs(delta:dot(right))
    if longitudinal > 0.5 and lateral <= 2.2 and delta:dot(up) > -0.85 then
      recordHazard(longitudinal, {
        kind = "scene-raycast",
        closingSpeed = ownForwardSpeed,
        source = "LiDAR/radar/ultrasonic"
      })
    end
  end

  if lidarFresh then
    local lidarReturns = data.lidar and data.lidar.rays or {}
    for i = 1, #lidarReturns do inspect(lidarReturns[i], data.lidar.origin) end
  end

  if radarFresh then
    local radarReturns = data.radar and data.radar.returns or {}
    for i = 1, #radarReturns do inspect(radarReturns[i], radarReturns[i].origin) end
  end

  if ultrasonicFresh then
    local ultrasonicReturns = data.ultrasonic and data.ultrasonic.returns or {}
    for i = 1, #ultrasonicReturns do inspect(ultrasonicReturns[i], ultrasonicReturns[i].origin) end
  end

  -- A cached "visible" OBB is useful only while both its registry snapshot and
  -- its visibility confirmation are recent.
  local registryTimestamp = state.vehicleRegistryTimestamp
  local registryFresh = state.vehicleRegistrySnapshotValid
    and type(registryTimestamp) == "number"
    and registryTimestamp <= now + 0.05
    and (now - registryTimestamp) <= WORLD_MODEL_REGISTRY_MAX_AGE
  local function inspectVehicleOBB(target)
    local center = state._roadmindHelpers.vectorFromTable(target.position)
    local dimensions = target.dimensions
    local orientation = target.orientation
    if not center or type(dimensions) ~= "table" or type(orientation) ~= "table"
        or type(dimensions.length) ~= "number" or type(dimensions.width) ~= "number"
        or type(dimensions.height) ~= "number" then
      inspect(target.position, originFallback)
      return
    end

    local targetForward = normalizedVec3(orientation.forward)
    local targetRight = normalizedVec3(orientation.right)
    local targetUp = normalizedVec3(orientation.up)
    if not targetForward or not targetRight or not targetUp then
      inspect(target.position, originFallback)
      return
    end

    local halfLength = math.max(0, dimensions.length * 0.5)
    local halfWidth = math.max(0, dimensions.width * 0.5)
    local halfHeight = math.max(0, dimensions.height * 0.5)
    local function projectedHalfExtent(axis)
      return halfLength * math.abs(targetForward:dot(axis))
        + halfWidth * math.abs(targetRight:dot(axis))
        + halfHeight * math.abs(targetUp:dot(axis))
    end

    local delta = center - originFallback
    local longitudinal = delta:dot(forward)
    local lateral = math.abs(delta:dot(right))
    local vertical = delta:dot(up)
    local forwardRadius = projectedHalfExtent(forward)
    local lateralRadius = projectedHalfExtent(right)
    local verticalRadius = projectedHalfExtent(up)
    local nearDistance = longitudinal - forwardRadius
    local farDistance = longitudinal + forwardRadius
    local lateralGap = math.max(0, lateral - lateralRadius)

    if farDistance > 0.5 and lateralGap <= 1.6
        and vertical + verticalRadius > -0.85 then
      local targetVelocity = state._roadmindHelpers.vectorFromTable(target.simulatorVelocity)
      local closingSpeed = ownForwardSpeed
      if playerVelocity and targetVelocity then
        closingSpeed = (playerVelocity - targetVelocity):dot(forward)
      end
      recordHazard(math.max(0.5, nearDistance), {
        kind = "vehicle",
        closingSpeed = math.max(0, closingSpeed),
        targetVelocity = targetVelocity and vectorToPlainTable(targetVelocity) or nil,
        sourceKind = target.sourceKind,
        trackId = target.trackId or target.sourceVehicleId
      })
    end
  end

  if registryFresh then
    for i = 1, #(data.vehicles or {}) do
      local target = data.vehicles[i]
      local visibilityTimestamp = tonumber(target.visibilityTimestamp)
      local visibilityFresh = visibilityTimestamp ~= nil
        and visibilityTimestamp <= now + 0.05
        and (now - visibilityTimestamp) <= OCCLUSION_CACHE_MAX_AGE
      local simulatorVehicleSource = target.sourceKind == "simulator-assisted"
        or target.sourceKind == "simulator-assisted-camera-visibility"
      if target.visibility == "visible" and simulatorVehicleSource and visibilityFresh then
        inspectVehicleOBB(target)
      end
    end
  end

  return minimum, nearestHazard
end


state._roadmindHelpers.setADASBrakeOutput = function(brake)
  brake = math.max(0, math.min(1, tonumber(brake) or 0))
  local previous = state.adasLastBrakeCommand or 0
  if math.abs(brake - previous) < 0.08 and (brake > 0) == (previous > 0) then return end
  state.adasLastBrakeCommand = brake
  if not state.activeVehicle then return end
  local command = "if not roadmind_controller and not roadmind_controller_load_attempted then roadmind_controller_load_attempted = true; extensions.load('roadmind_controller') end; " ..
    "if roadmind_controller then roadmind_controller.setAebBrake(" ..
    string.format("%.3f", brake) .. ") end"
  queueVehicleCommand(state.activeVehicle, command)
end

state._roadmindHelpers.getADASLeadVehicle = function(data, forward, right, origin)
  local ownLength = state.vehicleGeometry and tonumber(state.vehicleGeometry.length) or 4.5
  local ownWidth = state.vehicleGeometry and tonumber(state.vehicleGeometry.width) or 2.0
  local best = nil
  for i = 1, #(data.tracks or {}) do
    local track = data.tracks[i]
    local age = tonumber(track.dataAge) or math.huge
    local position = state._roadmindHelpers.vectorFromTable(track.predictedPosition or track.position)
    local dimensions = track.dimensions
    if position and (track.status == "confirmed" or track.status == "coasting")
        and age >= 0 and age <= TRACK_OBSERVATION_MAX_AGE
        and type(dimensions) == "table" then
      local delta = position - origin
      local longitudinal = delta:dot(forward)
      local lateral = math.abs(delta:dot(right))
      local halfLength = math.max(0.5, (tonumber(dimensions.length) or 4.5) * 0.5)
      local halfWidth = math.max(0.25, (tonumber(dimensions.width) or 2.0) * 0.5)
      local gap = longitudinal - ownLength * 0.5 - halfLength
      if gap > 0 and gap <= 90 and lateral <= ownWidth * 0.5 + halfWidth + 1.0 then
        if not best or gap < best.distance then
          best = {
            distance = gap,
            targetVelocity = state._roadmindHelpers.vectorFromTable(track.estimatedVelocity),
            trackId = track.trackId
          }
        end
      end
    end
  end
  return best
end

state._roadmindHelpers.getADASParkingDistance = function(data, now)
  local timestamp = tonumber((data.sensorTimestamps or {}).ultrasonic)
  if not state.settings.ultrasonicEnabled or not timestamp
      or timestamp > now + 0.05 or now - timestamp > MAX_FRAME_AGE then
    return nil
  end
  local nearest = math.huge
  for i = 1, #((data.ultrasonic and data.ultrasonic.returns) or {}) do
    local hit = data.ultrasonic.returns[i]
    local point = state._roadmindHelpers.vectorFromTable(hit)
    local origin = state._roadmindHelpers.vectorFromTable(hit.origin)
    if point and origin then nearest = math.min(nearest, (point - origin):length()) end
  end
  return nearest < math.huge and nearest or nil
end

state._roadmindHelpers.updateADAS = function(dt)
  local settings = state.settings or DEFAULT_SETTINGS
  if settings.adasEnabled ~= true then
    if (state.adasLastBrakeCommand or 0) > 0 then state._roadmindHelpers.setADASBrakeOutput(0) end
    if state.routePilotEnabled and state.activeVehicle
        and state.adasLastCruiseCommand ~= ROUTE_MAX_SPEED_MPS then
      queueVehicleCommand(state.activeVehicle,
        "if not roadmind_controller and not roadmind_controller_load_attempted then roadmind_controller_load_attempted = true; extensions.load('roadmind_controller') end; " ..
        "if roadmind_controller then roadmind_controller.setTargetSpeed(" ..
        string.format("%.2f", ROUTE_MAX_SPEED_MPS) .. ") end")
      state.adasLastCruiseCommand = ROUTE_MAX_SPEED_MPS
    end
    state.adas = {
      available = false, fcw = false, aeb = false, leftBlindSpot = false,
      rightBlindSpot = false, parkingDistance = nil, laneDeparture = false,
      leadDistance = nil, targetSpeedMps = ROUTE_MAX_SPEED_MPS, status = "disabled"
    }
    state.adasTimer = 0
    return
  end

  state.adasTimer = (state.adasTimer or 0) + math.max(0, tonumber(dt) or 0)
  if state.adasTimer < 0.10 then return end
  state.adasTimer = state.adasTimer % 0.10

  local data = state.latestData
  if not state.visualizationEnabled or state.frameAge > MAX_FRAME_AGE or type(data) ~= "table"
      or not state.activeVehicle then
    if (state.adasLastBrakeCommand or 0) > 0 then state._roadmindHelpers.setADASBrakeOutput(0) end
    state.adas = {
      available = false, fcw = false, aeb = false, leftBlindSpot = false,
      rightBlindSpot = false, parkingDistance = nil, laneDeparture = false,
      leadDistance = nil, targetSpeedMps = ROUTE_MAX_SPEED_MPS, status = "waiting_for_sensors"
    }
    return
  end

  local position = asVec3(safeCall(state.activeVehicle, "getPosition"))
  local forward = normalizedVec3(safeCall(state.activeVehicle, "getDirectionVector"))
  local up = normalizedVec3(safeCall(state.activeVehicle, "getDirectionVectorUp"))
  local right = forward and up and normalizedVec3(forward:cross(up)) or nil
  local velocity = asVec3(safeCall(state.activeVehicle, "getVelocity"))
  if not position or not forward or not right or not up or not velocity then
    if (state.adasLastBrakeCommand or 0) > 0 then state._roadmindHelpers.setADASBrakeOutput(0) end
    state.adas = {available = false, status = "vehicle_state_unavailable"}
    return
  end

  local speed = velocity:length()
  local now = state.simulationTime
  local timestamps = data.sensorTimestamps or {}
  local function freshSensor(key, enabled)
    local timestamp = tonumber(timestamps[key])
    return enabled == true and timestamp ~= nil
      and timestamp <= now + 0.05 and now - timestamp <= MAX_FRAME_AGE
  end

  local ultrasonicAvailable = freshSensor("ultrasonic", settings.ultrasonicEnabled)
  local registryAvailable = state.vehicleRegistrySnapshotValid
    and now - (state.vehicleRegistryTimestamp or -math.huge) <= WORLD_MODEL_REGISTRY_MAX_AGE
  local cameraAvailable = settings.cameraDetectionEnabled == true
    and registryAvailable and freshSensor("detectionZones", true)
  local forwardSensorAvailable = freshSensor("lidar", settings.lidarEnabled)
    or freshSensor("radar", settings.radarEnabled)
    or freshSensor("ultrasonic", settings.ultrasonicEnabled)
    or cameraAvailable
  local roadAvailable = type(state.roadState) == "table" and state.roadState.valid == true
    and (state.roadState.quality == "high" or state.roadState.quality == "medium")
    and now - (tonumber(state.roadState.timestamp) or -math.huge) <= PLANNER_MAX_DATA_AGE

  local needsForward = settings.adasFCWEnabled == true or settings.adasAEBEnabled == true
  local nearest, nearestHazard = math.huge, nil
  if needsForward and forwardSensorAvailable then
    nearest, nearestHazard = state._roadmindHelpers.nearestForwardObstacle(data)
  end
  local closingSpeed = nearestHazard and tonumber(nearestHazard.closingSpeed) or speed
  closingSpeed = math.max(0, closingSpeed or 0)
  local timeToCollision = closingSpeed > 0.5 and nearest < math.huge
    and nearest / closingSpeed or math.huge
  local fcw = settings.adasFCWEnabled == true and forwardSensorAvailable
    and nearest <= math.max(14, speed * 2.5 + 6)
  local aebThreshold = 3.5 + speed * 0.65 + speed * speed / 13.0
  local movingVehicleHazard = nearestHazard and nearestHazard.kind == "vehicle"
  local vehicleClosingEnough = not movingVehicleHazard
    or closingSpeed >= math.max(0.75, speed * 0.25)
  -- Static obstacles use ego stopping distance. Moving vehicles also need a
  -- meaningful closing rate; TTC catches high relative-speed approaches.
  local distanceTrigger = nearest <= aebThreshold and vehicleClosingEnough
  local ttcTrigger = closingSpeed > 0.5 and timeToCollision <= 1.8
  local aeb = settings.adasAEBEnabled == true and forwardSensorAvailable
    and speed >= 1.5 and nearest < math.huge
    and (distanceTrigger or ttcTrigger)
  local brakeLevel = 0
  if aeb then
    local distanceUrgency = (aebThreshold - nearest) / math.max(1, aebThreshold)
    local timeUrgency = timeToCollision < math.huge
      and (1.8 - timeToCollision) / 1.8 or 0
    local urgency = math.max(distanceUrgency, timeUrgency)
    brakeLevel = math.max(0.35, math.min(1.0, 0.35 + urgency * 0.65))
  end

  local leftBlindSpot, rightBlindSpot = false, false
  if settings.adasBSMEnabled == true and registryAvailable then
    local ownWidth = state.vehicleGeometry and tonumber(state.vehicleGeometry.width) or 2.0
    for i = 1, #(data.tracks or {}) do
      local track = data.tracks[i]
      local age = tonumber(track.dataAge) or math.huge
      local target = state._roadmindHelpers.vectorFromTable(track.predictedPosition or track.position)
      local dimensions = track.dimensions
      if target and type(dimensions) == "table" and age >= 0 and age <= TRACK_OBSERVATION_MAX_AGE
          and (track.status == "confirmed" or track.status == "coasting") then
        local delta = target - position
        local longitudinal = delta:dot(forward)
        local lateral = delta:dot(right)
        local halfWidth = math.max(0.25, (tonumber(dimensions.width) or 2) * 0.5)
        local sideDistance = math.abs(lateral) - ownWidth * 0.5
        if sideDistance <= halfWidth + 3.0 and sideDistance >= -0.5
            and longitudinal >= -9 and longitudinal <= 4 then
          if lateral < 0 then leftBlindSpot = true else rightBlindSpot = true end
        end
      end
    end
  end

  local parkingDistance = settings.adasParkingEnabled == true and ultrasonicAvailable
    and state._roadmindHelpers.getADASParkingDistance(data, now) or nil
  local laneDeparture = settings.adasLDWEnabled == true and roadAvailable and speed >= 5
    and math.abs(tonumber(state.roadState.lateralOffset) or 0) >= 1.65 or false

  local lead = settings.adasACCEnabled == true and state.routePilotStarted and registryAvailable
    and state._roadmindHelpers.getADASLeadVehicle(data, forward, right, position) or nil
  local targetSpeed = ROUTE_MAX_SPEED_MPS
  if settings.adasACCEnabled == true and state.routePilotStarted then
    if not registryAvailable then
      -- If traffic observations go stale, hold the current speed target rather
      -- than accelerating into an unobserved lead-vehicle gap.
      targetSpeed = math.min(targetSpeed, speed)
    elseif lead then
      local desiredGap = 5.0 + speed * 1.55
      local gapFactor = math.max(0, math.min(1, (lead.distance - 3.0) / math.max(1, desiredGap - 3.0)))
      targetSpeed = math.min(targetSpeed, math.max(0, speed * gapFactor))
      if lead.targetVelocity then
        local closingSpeed = (velocity - lead.targetVelocity):dot(forward)
        if closingSpeed > 0.5 then
          local ttc = lead.distance / closingSpeed
          if ttc < 3.0 then targetSpeed = math.min(targetSpeed, math.max(0, speed * ttc / 3.0)) end
        end
      end
    end
  end

  -- AEB can brake a manually driven car. In active route mode the route
  -- supervisor applies the stop, avoiding competing control sources.
  state._roadmindHelpers.setADASBrakeOutput(not state.routePilotStarted and aeb and brakeLevel or 0)

  local anyFeatureEnabled = settings.adasFCWEnabled or settings.adasAEBEnabled
    or settings.adasACCEnabled or settings.adasBSMEnabled
    or settings.adasParkingEnabled or settings.adasLDWEnabled
  local requiresSensors =
    ((settings.adasFCWEnabled or settings.adasAEBEnabled) and not forwardSensorAvailable)
    or (settings.adasParkingEnabled and not ultrasonicAvailable)
    or (settings.adasBSMEnabled and not registryAvailable)
    or (settings.adasACCEnabled and state.routePilotStarted and not registryAvailable)
    or (settings.adasLDWEnabled and not roadAvailable)

  local nextStatus = not anyFeatureEnabled and "standby"
    or (requiresSensors and "waiting_for_sensors" or "active")
  if aeb then nextStatus = "aeb"
  elseif fcw then nextStatus = "fcw"
  elseif leftBlindSpot or rightBlindSpot then nextStatus = "blind_spot"
  elseif laneDeparture then nextStatus = "lane_departure" end

  state.adas = {
    available = anyFeatureEnabled and not requiresSensors,
    requiresSensors = requiresSensors == true,
    fcw = fcw,
    aeb = aeb,
    aebDistance = aeb and round2(nearest) or nil,
    aebThreshold = settings.adasAEBEnabled == true and forwardSensorAvailable
      and aebThreshold or ROUTE_STOP_DISTANCE,
    aebClosingSpeed = nearestHazard and round2(closingSpeed) or nil,
    aebTimeToCollision = timeToCollision < math.huge and round2(timeToCollision) or nil,
    aebHazardKind = nearestHazard and nearestHazard.kind or nil,
    aebHazardSource = nearestHazard and nearestHazard.sourceKind or nil,
    leftBlindSpot = leftBlindSpot,
    rightBlindSpot = rightBlindSpot,
    parkingDistance = parkingDistance and round2(parkingDistance) or nil,
    laneDeparture = laneDeparture,
    leadDistance = lead and round2(lead.distance) or nil,
    targetSpeedMps = targetSpeed,
    status = nextStatus,
    sensorAvailable = forwardSensorAvailable,
    cameraAvailable = cameraAvailable == true,
    ultrasonicAvailable = ultrasonicAvailable,
    registryAvailable = registryAvailable,
    roadAvailable = roadAvailable,
    roadQuality = state.roadState and state.roadState.quality or "unavailable"
  }

  local cruiseTarget = settings.adasACCEnabled == true and targetSpeed or ROUTE_MAX_SPEED_MPS
  if state.routePilotStarted
      and math.abs(cruiseTarget - (state.adasLastCruiseCommand or ROUTE_MAX_SPEED_MPS)) >= 0.25 then
    local command = "if not roadmind_controller and not roadmind_controller_load_attempted then roadmind_controller_load_attempted = true; extensions.load('roadmind_controller') end; " ..
      "if roadmind_controller then roadmind_controller.setTargetSpeed(" ..
      string.format("%.2f", cruiseTarget) .. ") end"
    queueVehicleCommand(state.activeVehicle, command)
    state.adasLastCruiseCommand = cruiseTarget
  elseif not state.routePilotStarted then
    state.adasLastCruiseCommand = nil
  end
end

state._roadmindHelpers.updateRouteProgress = function(dt)
  if not state.routePilotStarted or not state.activeVehicle or #state.routePath < 2 then return end
  state.routeProgressTimer = (state.routeProgressTimer or 0) + math.max(0, tonumber(dt) or 0)
  if state.routeProgressTimer < 0.25 then return end
  state.routeProgressTimer = state.routeProgressTimer % 0.25

  local position = asVec3(safeCall(state.activeVehicle, "getPosition"))
  if not position then return end
  local firstIndex = math.max(1, state.routeProgressIndex or 1)
  local lastIndex = math.min(#state.routePath - 1, firstIndex + 12)
  local bestIndex, bestDistance = firstIndex, math.huge

  -- Measure distance to route segments, not isolated nodes, to avoid false
  -- route-loss reports on maps with long NavGraph edges.
  for i = firstIndex, lastIndex do
    local a, b = state.routePath[i], state.routePath[i + 1]
    local dx, dy = b.x - a.x, b.y - a.y
    local lengthSquared = dx * dx + dy * dy
    local projection = 0
    if lengthSquared > 0.0001 then
      projection = math.max(0, math.min(1,
        ((position.x - a.x) * dx + (position.y - a.y) * dy) / lengthSquared))
    end
    local nearestX, nearestY = a.x + dx * projection, a.y + dy * projection
    local offsetX, offsetY = position.x - nearestX, position.y - nearestY
    local distance = math.sqrt(offsetX * offsetX + offsetY * offsetY)
    if distance < bestDistance then bestDistance = distance; bestIndex = i end
  end

  state.routeProgressIndex = math.max(firstIndex, bestIndex)
  state.routeDistanceToPath = bestDistance
  state.routeProgressPercent = state.routeProgressIndex / (#state.routePath - 1) * 100
  if bestDistance > ROUTE_MAX_DEVIATION_M then
    state.routeLostUpdates = (state.routeLostUpdates or 0) + 1
  else
    state.routeLostUpdates = 0
  end
end

state._roadmindHelpers.updateRoutePilotSafety = function(dt)
  if not state.routePilotEnabled then return end

  dt = math.max(0, tonumber(dt) or 0)
  if state.simulationTime - (state.routeSafetyLastCheckAt or -math.huge) < ROUTE_SAFETY_INTERVAL then
    return
  end
  state.routeSafetyLastCheckAt = state.simulationTime
  state.routeRetryCooldown = math.max(0, (state.routeRetryCooldown or 0) - dt)

  if not state.activeVehicle or state.routePilotVehicleId ~= state.activeVehicleId then
    if state.activeVehicle then
      stopNativeRouteController(state.activeVehicle)
      state._roadmindHelpers.disableNativeRouteController(state.activeVehicle)
    end
    state.routePilotEnabled = false
    state.routePilotStarted = false
    state.routeSafetyStopped = false
    state.routePilotVehicleId = nil
    state.routePilotError = "The active vehicle changed"
    state.routePath = {}
    state.routePathEnd = nil
    return
  end

  state._roadmindHelpers.updateRouteProgress(dt)
  if state.routePilotStarted and (state.routeLostUpdates or 0) >= ROUTE_MAX_LOST_UPDATES then
    stopNativeRouteController(state.activeVehicle)
    state.routePilotStarted = false
    state.routeSafetyStopped = true
    state.routePilotError = "Route localization lost"
    state.routeRetryCooldown = 3.0
    log("W", "roadmind", "Route pilot stopped: vehicle left the planned route corridor")
    return
  end

  if state.frameAge > MAX_FRAME_AGE or not state.latestData then
    if state.routePilotStarted then
      stopNativeRouteController(state.activeVehicle)
      state.routePilotStarted = false
      state.routeSafetyStopped = true
    end
    state.routePilotError = "Waiting for fresh sensor data"
    return
  end

  local plannerStarted = M.performanceProfiler.start()
  local planner = updateLocalTrajectoryPlanner()
  M.performanceProfiler.stop("planner", plannerStarted)
  if planner.status ~= "ready" or not planner.selectedIndex then
    if state.routePilotStarted then
      stopNativeRouteController(state.activeVehicle)
      state.routePilotStarted = false
      log("W", "roadmind", "Local planner stopped the route pilot: " .. tostring(planner.reasonCode))
    end
    state.routeSafetyStopped = true
    state.routePilotError = "Local planner: " .. tostring(planner.reason or "No safe local trajectory")
    state.routeRetryCooldown = math.max(state.routeRetryCooldown or 0, 0.5)
    return
  end
  if type(state.routePilotError) == "string"
      and state.routePilotError:find("^Local planner:") then
    state.routePilotError = nil
  end

  -- Refresh the low-level controller only when the bounded planner publishes
  -- a new safe trajectory. This keeps command traffic below the planner's 5 Hz cap.
  if state.routePilotStarted then
    local sent, sendError = state._roadmindHelpers.sendControllerPath(state.activeVehicle, planner, false)
    if not sent then
      stopNativeRouteController(state.activeVehicle)
      state.routePilotStarted = false
      state.routeSafetyStopped = true
      state.routePilotError = sendError or "Could not refresh controller trajectory"
      return
    end
  end

  local clearance = state._roadmindHelpers.nearestForwardObstacle(state.latestData)
  local routeStopDistance = ROUTE_STOP_DISTANCE
  if state.settings and state.settings.adasEnabled == true
      and state.settings.adasAEBEnabled == true then
    routeStopDistance = math.max(routeStopDistance, tonumber(state.adas and state.adas.aebThreshold) or ROUTE_STOP_DISTANCE)
  end
  if clearance <= routeStopDistance then
    if state.routePilotStarted then
      stopNativeRouteController(state.activeVehicle)
      state.routePilotStarted = false
      log("W", "roadmind", "Route pilot paused by sensor obstacle guard")
    end
    state.routeSafetyStopped = true
    state.routePilotError = string.format("Obstacle inside %.0f m safety distance", routeStopDistance)
    return
  end

  if state.routePilotStarted and #state.routeInputPoints > 0 then
    local endpoint = state.routePathEnd
    local position = asVec3(safeCall(state.activeVehicle, "getPosition"))
    if position and endpoint then
      local dx, dy, dz = endpoint.x - position.x, endpoint.y - position.y, endpoint.z - position.z
      if math.sqrt(dx * dx + dy * dy + dz * dz) <= ROUTE_DESTINATION_REACHED_M then
        stopNativeRouteController(state.activeVehicle)
        state._roadmindHelpers.disableNativeRouteController(state.activeVehicle)
        state.routePilotEnabled = false
        state.routePilotStarted = false
        state.routeSafetyStopped = false
        state.routeCompleted = true
        state.routePilotError = nil
        log("I", "roadmind", "Route destination reached")
        return
      end
    end
  end

  -- A supplied route already contains its destination; only the automatic
  -- forward-driving mode appends another route segment.
  if state.routePilotStarted and #state.routeInputPoints > 0 then return end

  -- Extend the automatic route before its last NavGraph waypoint is reached.
  if state.routePilotStarted and state.routePathEnd then
    local position = safeCall(state.activeVehicle, "getPosition")
    if position then
      local endpoint = state.routePathEnd
      local dx, dy = endpoint.x - position.x, endpoint.y - position.y
      local distance = math.sqrt(dx * dx + dy * dy)
      if distance <= ROUTE_REPLAN_DISTANCE and state.routeRetryCooldown <= 0 then
        local started, reason = state._roadmindHelpers.startNativeRouteController(state.activeVehicle)
        state.routeRetryCooldown = ROUTE_REPLAN_COOLDOWN
        if started then
          state.routePilotError = nil
        else
          state.routePilotError = reason or "Could not extend the road route"
        end
      end
    end
    return
  end

  if state.routeSafetyStopped or not state.routePilotStarted then
    if clearance >= ROUTE_RESUME_DISTANCE and state.routeRetryCooldown <= 0 then
      local started, reason = state._roadmindHelpers.startNativeRouteController(state.activeVehicle)
      if started then
        state.routePilotStarted = true
        state.routeSafetyStopped = false
        state.routePilotError = nil
        state.routeRetryCooldown = ROUTE_REPLAN_COOLDOWN
        log("I", "roadmind", "NavGraph route started with sensor safety enabled")
      else
        state.routeSafetyStopped = true
        state.routePilotError = reason or "Could not build a forward road route"
        state.routeRetryCooldown = 3.0
      end
    end
  end
end

state._roadmindHelpers.mergeSensorFrame = function(frame)
  local updates = frame.updatedSensors or {}
  local latest = state.latestData or {
    enabled = true,
    lidar = {rays = {}, pointCount = 0},
    radar = {mounts = {}, returns = {}},
    ultrasonic = {mounts = {}, returns = {}},
    detectionZones = {timestamp = 0, zones = {}},
    sensorTimestamps = {},
    tracks = {},
    cameras = {}
  }
  latest.sensorTimestamps = latest.sensorTimestamps or {}
  latest.enabled = frame.enabled == true
  latest.sequence = frame.sequence

  if updates.lidar == true then
    local profileStarted = M.performanceProfiler.start()
    latest.lidar = processLidar(frame.lidar)
    latest.lidar.scanTimestamp = state.simulationTime
    latest.lidar.scanSequence = tonumber(frame.sequence) or 0
    for i = 1, #(latest.lidar.rays or {}) do
      latest.lidar.rays[i].measurementTimestamp = state.simulationTime
    end
    latest.sensorTimestamps.lidar = state.simulationTime
    state._roadmindHelpers.rememberLidarPoints(latest.lidar)
    buildLidarRenderPoints(latest.lidar)
    M.performanceProfiler.stop("sensor_lidar", profileStarted)
  end
  if updates.radar == true then
    local profileStarted = M.performanceProfiler.start()
    latest.radar = processRadar(frame.radar)
    state._roadmindHelpers.attachTrackedVehicleReturns(latest.radar.returns)
    latest.radar.scanTimestamp = state.simulationTime
    latest.radar.scanSequence = tonumber(frame.sequence) or 0
    for i = 1, #(latest.radar.returns or {}) do
      latest.radar.returns[i].measurementTimestamp = state.simulationTime
    end
    latest.sensorTimestamps.radar = state.simulationTime
    M.performanceProfiler.stop("sensor_radar", profileStarted)
  end
  if updates.ultrasonic == true then
    local profileStarted = M.performanceProfiler.start()
    latest.ultrasonic = processUltrasonic(frame.ultrasonic)
    state._roadmindHelpers.attachTrackedVehicleReturns(latest.ultrasonic.returns)
    latest.ultrasonic.scanTimestamp = state.simulationTime
    latest.ultrasonic.scanSequence = tonumber(frame.sequence) or 0
    for i = 1, #(latest.ultrasonic.returns or {}) do
      latest.ultrasonic.returns[i].measurementTimestamp = state.simulationTime
    end
    latest.sensorTimestamps.ultrasonic = state.simulationTime
    M.performanceProfiler.stop("sensor_ultrasonic", profileStarted)
  end
  if updates.detectionZones == true then
    latest.detectionZones = type(frame.detectionZones) == "table"
      and frame.detectionZones or {timestamp = 0, zones = {}}
    latest.sensorTimestamps.detectionZones = state.simulationTime
  end
  if updates.pose == true then
    latest.cameras = type(frame.cameras) == "table" and frame.cameras or {}
    latest.localization = type(frame.localization) == "table" and frame.localization or nil
    latest.localizationVehiclePose = type(frame.vehiclePose) == "table"
      and frame.vehiclePose or latest.vehiclePose
    latest.mounts = type(frame.mounts) == "table" and frame.mounts or {}
    latest.geometryDebug = type(frame.geometryDebug) == "table" and frame.geometryDebug or nil
  end
  if type(frame.vehiclePose) == "table" then latest.vehiclePose = frame.vehiclePose end
  if type(frame.attachedVisuals) == "table" then
    local attached = frame.attachedVisuals
    latest.lidar = latest.lidar or {rays = {}, pointCount = 0}
    if attached.lidarOrigin then latest.lidar.origin = attached.lidarOrigin end
    latest.radar = latest.radar or {mounts = {}, returns = {}}
    latest.radar.mounts = type(attached.radarMounts) == "table" and attached.radarMounts or {}
    latest.ultrasonic = latest.ultrasonic or {mounts = {}, returns = {}}
    latest.ultrasonic.mounts = type(attached.ultrasonicMounts) == "table" and attached.ultrasonicMounts or {}
    latest.cameras = type(attached.cameras) == "table" and attached.cameras or latest.cameras
    latest.mounts = type(attached.mounts) == "table" and attached.mounts or latest.mounts
    latest.geometryDebug = attached.geometryDebug
    if type(attached.detectionZones) == "table" then
      latest.detectionZones = attached.detectionZones
      latest.detectionZonesVehiclePose = type(frame.vehiclePose) == "table"
        and frame.vehiclePose or latest.detectionZonesVehiclePose
      latest.sensorTimestamps.detectionZones = state.simulationTime
    end
  end

  if type(frame.vehicleVelocity) == "table" then latest.vehicleVelocity = frame.vehicleVelocity end

  latest.vehicles = state.vehicleCandidates
  state.latestData = latest
  state.frameAge = 0
  if updates.lidar == true or updates.radar == true or updates.ultrasonic == true then
    state.worldModelDirty = true
  end
  local routeStarted = M.performanceProfiler.start()
  state._roadmindHelpers.updateRoutePilotSafety()
  M.performanceProfiler.stop("route_safety", routeStarted)
end

function M.onSensorData(encodedFrame)
  if not state.visualizationEnabled or type(encodedFrame) ~= "string"
      or type(jsonDecode) ~= "function" then return end
  local profileStarted = M.performanceProfiler.start()
  local ok, frame = pcall(jsonDecode, encodedFrame)
  if not ok or type(frame) ~= "table" then
    M.performanceProfiler.stop("sensor_frame", profileStarted)
    return
  end
  -- Drop late messages from a vehicle that is no longer the active player car.
  if frame.sourceVehicleId ~= state.activeVehicleId then
    M.performanceProfiler.stop("sensor_frame", profileStarted)
    return
  end
  if frame.enabled == false then
    state._roadmindHelpers.resetFrame()
    M.performanceProfiler.stop("sensor_frame", profileStarted)
    return
  end
  if type(frame.performance) == "table" then
    M.performanceProfiler.record("vehicle_job", frame.performance.elapsedMs)
    local job = frame.performance.job
    if type(job) == "string" then M.performanceProfiler.record("vehicle_" .. job, frame.performance.elapsedMs) end
  end
  state._roadmindHelpers.mergeSensorFrame(frame)
  M.performanceProfiler.stop("sensor_frame", profileStarted)
end



state._roadmindHelpers.updateAdaptivePerformance = function(dt)
  local elapsed = math.max(0, tonumber(dt) or 0)
  if elapsed > 0 and elapsed < 1.0 then
    local instantFps = math.min(300, 1 / elapsed)
    local alpha = 1 - math.exp(-elapsed / 1.5)
    if state.performanceFps == nil then
      state.performanceFps = instantFps
    else
      state.performanceFps = state.performanceFps + (instantFps - state.performanceFps) * alpha
    end
    state.performanceFpsSamples = (state.performanceFpsSamples or 0) + 1
  end

  local settings = state.settings or DEFAULT_SETTINGS
  if settings.adaptiveFpsEnabled ~= true then return end
  if state.performanceFps == nil or (state.performanceFpsSamples or 0) < 10 then return end

  state.performanceDecisionTimer = (state.performanceDecisionTimer or 0) + elapsed
  if state.performanceDecisionTimer < 0.5 then return end
  local decisionDt = state.performanceDecisionTimer
  state.performanceDecisionTimer = 0

  local target = tonumber(settings.targetFps) or 60
  local measured = state.performanceFps
  if measured < target * 0.92 then
    state.performanceLowFpsTimer = (state.performanceLowFpsTimer or 0) + decisionDt
    state.performanceHighFpsTimer = 0
  elseif measured > target * 1.08 then
    state.performanceHighFpsTimer = (state.performanceHighFpsTimer or 0) + decisionDt
    state.performanceLowFpsTimer = 0
  else
    state.performanceLowFpsTimer = 0
    state.performanceHighFpsTimer = 0
  end

  local oldTier = state.adaptiveQualityTier or 4
  local nextTier = oldTier
  if state.performanceLowFpsTimer >= 2.0 and oldTier > 0 then
    nextTier = oldTier - 1
    state.performanceLowFpsTimer = 0
    state.performanceHighFpsTimer = 0
  elseif state.performanceHighFpsTimer >= 4.0 and oldTier < 4 then
    nextTier = oldTier + 1
    state.performanceLowFpsTimer = 0
    state.performanceHighFpsTimer = 0
  end

  if nextTier ~= oldTier then
    state.adaptiveQualityTier = nextTier
    if state.visualizationEnabled and state.activeVehicle then
      state._roadmindHelpers.setVehicleSensorSettings(state.activeVehicle)
    end
    log("I", "roadmind", string.format(
      "Adaptive sensor quality changed to tier %d (FPS %.1f / target %d)",
      nextTier, measured, target
    ))
  end
end

function M.onUpdate(dt)
  dt = tonumber(dt) or 0
  if dt < 0 then dt = 0 end
  if not state.visualizationEnabled then return end

  -- Keep watching for a replacement player vehicle while auto-resume is enabled,
  -- even when the previous vehicle was destroyed and sensor data is stale.
  checkTimer = checkTimer + dt
  if checkTimer >= CHECK_INTERVAL then
    checkTimer = 0
    state._roadmindHelpers.syncPlayerVehicle()
  end

  if not state.visualizationEnabled or not state.activeVehicle then return end

  local geStarted = M.performanceProfiler.start()
  state._roadmindHelpers.updateAdaptivePerformance(dt)
  state.frameAge = state.frameAge + dt
  local visualFrameFresh = state.frameAge <= MAX_FRAME_AGE
  local routePilotActive = state.routePilotEnabled

  if visualFrameFresh or routePilotActive then
    state.simulationTime = state.simulationTime + dt
    -- The route-safety clock continues while sensors are stale, so stale data
    -- can trigger a stop. The visual clock freezes when scanning is paused.
    state.roadUpdateTimer = (state.roadUpdateTimer or 0) + dt
    if state.roadUpdateTimer >= 0.5 then
      state.roadUpdateTimer = state.roadUpdateTimer % 0.5
      local roadStarted = M.performanceProfiler.start()
      state._roadmindHelpers.updateRoadUnderstanding()
      M.performanceProfiler.stop("road_graph", roadStarted)
    end
    local trackingStarted = M.performanceProfiler.start()
    state._roadmindHelpers.updateVehicleDetection(dt)
    M.performanceProfiler.stop("vehicle_tracking", trackingStarted)
    if state.worldModelDirty and buildWorldModel then
      state.worldModelUpdateTimer = (state.worldModelUpdateTimer or 0) + dt
      if not state.worldModel or state.worldModelUpdateTimer >= WORLD_MODEL_UPDATE_INTERVAL then
        local worldModelStarted = M.performanceProfiler.start()
        state.worldModel = buildWorldModel()
        state.worldModelDirty = false
        state.worldModelUpdateTimer = 0
        M.performanceProfiler.stop("world_model", worldModelStarted)
      end
    else
      state.worldModelUpdateTimer = 0
    end
    if visualFrameFresh then
      state._roadmindHelpers.updateLidarRenderPoints(dt)
      state._roadmindHelpers.updateDetectedVehicleRenderStates(dt)
    end
    if state.mapPlanningEnabled and visualFrameFresh then
      local mapPlannerStarted = M.performanceProfiler.start()
      updateLocalTrajectoryPlanner()
      M.performanceProfiler.stop("map_planner_preview", mapPlannerStarted)
    end
    if routePilotActive then
      local safetyStarted = M.performanceProfiler.start()
      state._roadmindHelpers.updateRoutePilotSafety(dt)
      M.performanceProfiler.stop("route_safety", safetyStarted)
    end
  end
  -- Use the newest vehicle registry, camera visibility and tracks for ADAS.
  -- Keep this outside the freshness block so stale data still releases AEB.
  local adasStarted = M.performanceProfiler.start()
  state._roadmindHelpers.updateADAS(dt)
  M.performanceProfiler.stop("adas", adasStarted)

  -- Prepare smoothed sensor geometry outside the render-critical hook.
  local visualStarted = M.performanceProfiler.start()
  state._roadmindHelpers.updateVisualizationRenderState(dt)
  M.performanceProfiler.stop("visualization", visualStarted)
  M.performanceProfiler.stop("ge_update", geStarted)
  M.performanceProfiler.advance(dt)
end

-- debugDrawer queues primitives for the next render frame. Submitting them from
-- onPreRender keeps sensor dots visible during live Freeroam rendering.
function M.onPreRender(dtReal, dtSim, dtRaw)
  if not state.visualizationEnabled then return end
  drawFrame()
end

function M.onClientStartMission()
  state.sessionStaticMap = {}
  state.sessionStaticObservationIds = {}
  state.sessionStaticObservationOrder = {}
  state.sessionStaticObservationCursor = 1
  state.sessionStaticMapOutputCache = nil
  state.sessionStaticMapOutputCenter = nil
  state.sessionStaticMapLastRadius = nil
  state.sessionStaticMapCount = 0
  state.sessionStaticMapDistance = 0
  state.sessionStaticMapLastPosition = nil
  state.sessionStaticMapLastPrunePosition = nil
  state.sessionStaticMapGeneration = (state.sessionStaticMapGeneration or 0) + 1
  state.sessionStaticMapRevision = (state.sessionStaticMapRevision or 0) + 1
  state.trafficLaneEvidence = {}
  if state.worldModel then state.worldModel.staticGeometry = {} end
  state.levelMetadataLoaded = false
  state.levelMetadataName = nil
  state.levelDirectory = nil
  state.levelRoadCache = nil
  state.levelRoadCacheFile = nil
  state.levelRoadCacheStatus = "unavailable"
  state.levelRoadCacheNodeCount = 0
  state.levelRoadCacheRevision = (state.levelRoadCacheRevision or 0) + 1
  state.levelRoadLoadObjects = nil
  state.levelRoadLoadCursor = nil
  state.levelRoadLoadActive = false
  state.mapRoadGeometryCache = nil
  state.mapRoadGeometryCacheTime = -math.huge
  state.mapRoadGeometryCachePosition = nil
  state.mapRoadGeometryCacheForward = nil
  state.mapRoadGeometryCacheRange = nil
  state.mapRoadGeometryCacheRoadRevision = -1
  state.mapRoadGeometryCacheRevision = (state.mapRoadGeometryCacheRevision or 0) + 1
  state.navGraphCache = nil
  state.levelPOICache = {}
  state.levelSignalCache = {status = "unavailable", items = {}}
end

function M.onClientEndMission()
  state.sessionStaticMap = {}
  state.sessionStaticObservationIds = {}
  state.sessionStaticObservationOrder = {}
  state.sessionStaticObservationCursor = 1
  state.sessionStaticMapOutputCache = nil
  state.sessionStaticMapOutputCenter = nil
  state.sessionStaticMapLastRadius = nil
  state.sessionStaticMapCount = 0
  state.sessionStaticMapDistance = 0
  state.sessionStaticMapLastPosition = nil
  state.sessionStaticMapLastPrunePosition = nil
  state.sessionStaticMapGeneration = (state.sessionStaticMapGeneration or 0) + 1
  state.sessionStaticMapRevision = (state.sessionStaticMapRevision or 0) + 1
  state.trafficLaneEvidence = {}
  if state.worldModel then state.worldModel.staticGeometry = {} end
  state.levelMetadataLoaded = false
  state.levelMetadataName = nil
  state.levelDirectory = nil
  state.levelRoadCache = nil
  state.levelRoadCacheFile = nil
  state.levelRoadCacheStatus = "unavailable"
  state.levelRoadCacheNodeCount = 0
  state.levelRoadCacheRevision = (state.levelRoadCacheRevision or 0) + 1
  state.levelRoadLoadObjects = nil
  state.levelRoadLoadCursor = nil
  state.levelRoadLoadActive = false
  state.mapRoadGeometryCache = nil
  state.mapRoadGeometryCacheTime = -math.huge
  state.mapRoadGeometryCachePosition = nil
  state.mapRoadGeometryCacheForward = nil
  state.mapRoadGeometryCacheRange = nil
  state.mapRoadGeometryCacheRoadRevision = -1
  state.mapRoadGeometryCacheRevision = (state.mapRoadGeometryCacheRevision or 0) + 1
  state.navGraphCache = nil
  state.levelPOICache = {}
  state.levelSignalCache = {status = "unavailable", items = {}}
end

function M.clearSessionMap()
  state.sessionStaticMap = {}
  state.sessionStaticObservationIds = {}
  state.sessionStaticObservationOrder = {}
  state.sessionStaticObservationCursor = 1
  state.sessionStaticMapOutputCache = nil
  state.sessionStaticMapOutputCenter = nil
  state.sessionStaticMapLastRadius = nil
  state.sessionStaticMapCount = 0
  state.sessionStaticMapRevision = (state.sessionStaticMapRevision or 0) + 1
  state.sessionStaticMapDistance = 0
  state.sessionStaticMapLastPosition = nil
  state.sessionStaticMapLastPrunePosition = nil
  state.sessionStaticMapGeneration = (state.sessionStaticMapGeneration or 0) + 1
  if state.worldModel then state.worldModel.staticGeometry = {} end
  return {ok = true, cleared = true}
end

function M.onCursorVisibilityChanged(visible)
  state.mouseCursorVisible = visible == true
end

function M.onMouseLocked(locked)
  state.mouseLocked = locked == true
end

function M.onVehicleDestroyed(vehicleId)
  if vehicleId == state.activeVehicleId then
    local autoResume = state.settings and state.settings.autoScanOnVehicleSpawn == true
    state._roadmindHelpers.detachPreviousVehicle()
    if not autoResume then
      state.visualizationEnabled = false
    end
    state._roadmindHelpers.resetFrame()
    return
  end

  local key = tostring(vehicleId)
  local track = state.tracksBySourceId[key]
  if track then
    state.tracks[track.trackId] = nil
    state.tracksBySourceId[key] = nil
    state.trackCount = math.max(0, state.trackCount - 1)
  end
  state.vehicleRegistryById[key] = nil
  state.occlusionCache[key] = nil
  state.vehicleRenderStates[key] = nil
  state.lidarTrackGrid = {}
end

function M.getRouteState()
  return {
    schemaVersion = 1,
    coordinateFrame = "beamng-world",
    coordinateUnits = "meters",
    timestamp = state.simulationTime,
    road = state._roadmindHelpers.cloneWorldModelValue(state.roadState or {
      valid = false,
      quality = "unavailable",
      timestamp = state.simulationTime,
      corridor = {}
    }),
    route = {
      enabled = state.routePilotEnabled,
      started = state.routePilotStarted,
      safetyStopped = state.routeSafetyStopped,
      completed = state.routeCompleted,
      source = #state.routeInputPoints > 0 and "ordered-user-points" or "forward-navgraph",
      inputPoints = state._roadmindHelpers.cloneWorldModelValue(state.routeInputPoints),
      path = state._roadmindHelpers.cloneWorldModelValue(state.routePath),
      progressIndex = state.routeProgressIndex,
      progressPercent = state.routeProgressPercent,
      distanceToPath = state.routeDistanceToPath,
      destinationSnapDistance = state.routeDestinationSnapDistance,
      nextTarget = state._roadmindHelpers.cloneWorldModelValue(
        state.routePath[math.min(#state.routePath, (state.routeProgressIndex or 1) + 1)]
      ),
      error = state.routePilotError
    },
    planner = state._roadmindHelpers.cloneWorldModelValue(state.localPlanner or {
      status = "idle", reasonCode = "inactive", candidates = {}
    })
  }
end

function M.getWorldModel()
  if not state.worldModel and buildWorldModel then
    state.worldModel = buildWorldModel()
    state.worldModelDirty = false
    state.worldModelUpdateTimer = 0
  end
  if not state.worldModel then
    return {
      schemaVersion = WORLD_MODEL_SCHEMA_VERSION,
      coordinateFrame = "beamng-world",
      timestamp = state.simulationTime,
      generation = state.worldModelGeneration,
      coordinateUnits = "meters",
      targets = {},
      obstacles = {},
      observations = {lidar = {}, radar = {}, ultrasonic = {}, vehicleRegistry = {}},
      observationCount = 0,
      summary = {targetCount = 0, obstacleCount = 0, observationCount = 0},
      sourceStatus = {
        lidar = {enabled = false, fresh = false, stale = false},
        radar = {enabled = false, fresh = false, stale = false},
        ultrasonic = {enabled = false, fresh = false, stale = false},
        vehicleRegistry = {enabled = false, fresh = false, stale = false}
      },
      quality = {
        invalidObservationCount = 0,
        staleObservationCount = 0,
        staleSourceCount = 0,
        truncatedSourceCount = 0,
        lowConfidenceObservationCount = 0,
        conflictingObservationCount = 0,
        droppedObservationCount = 0,
        staleTargetCount = 0,
        unassociatedVehicleHitCount = 0,
        unverifiedVehicleCandidateCount = 0,
        occludedVehicleCandidateCount = 0
      }
    }
  end
  -- Callers receive an independent snapshot and cannot mutate GE-owned fusion state.
  return state._roadmindHelpers.cloneWorldModelValue(state.worldModel)
end

function M.getPerceptionSnapshot(worldModelOverride)
  local latest = state.latestData or {}
  local pose = latest.vehiclePose or latest.localizationVehiclePose
  local position = pose and asVec3(pose.position) or nil
  local forward = pose and normalizedVec3(pose.forward) or nil
  local right = pose and normalizedVec3(pose.right) or nil
  local up = pose and normalizedVec3(pose.up) or nil

  if state.activeVehicle then
    position = position or asVec3(safeCall(state.activeVehicle, "getPosition"))
    forward = forward or normalizedVec3(safeCall(state.activeVehicle, "getDirectionVector"))
    up = up or normalizedVec3(safeCall(state.activeVehicle, "getDirectionVectorUp"))
    right = right or (forward and up and normalizedVec3(forward:cross(up)) or nil)
  end

  local vehicle = nil
  if position and forward and right then
    vehicle = {
      position = vectorToPlainTable(position),
      forward = vectorToPlainTable(forward),
      right = vectorToPlainTable(right),
      up = vectorToPlainTable(up),
      dimensions = state.vehicleGeometry and {
        length = state.vehicleGeometry.length,
        width = state.vehicleGeometry.width,
        height = state.vehicleGeometry.height
      } or nil
    }
  end

  local model = type(worldModelOverride) == "table"
    and worldModelOverride or M.getWorldModel()
  local targets = {}
  local tracks = model and model.targets or latest.tracks or {}
  for i = 1, math.min(#tracks, 48) do
    local track = tracks[i]
    if type(track) == "table" and type(track.position) == "table" then
      targets[#targets + 1] = {
        trackId = track.trackId,
        status = track.status,
        position = state._roadmindHelpers.cloneWorldModelValue(track.position),
        predictedPosition = state._roadmindHelpers.cloneWorldModelValue(track.predictedPosition),
        dimensions = state._roadmindHelpers.cloneWorldModelValue(track.dimensions),
        orientation = state._roadmindHelpers.cloneWorldModelValue(track.orientation),
        sourceSensors = state._roadmindHelpers.copySensorSources(track.sourceSensors),
        cameraVisible = (function()
          for sourceIndex = 1, #(track.sourceSensors or {}) do
            if track.sourceSensors[sourceIndex].sensor == "camera" then return true end
          end
          return false
        end)(),
        dataAge = track.dataAge,
        visibilityAge = track.visibilityAge,
        confidence = track.confidence,
        estimatedSpeed = track.estimatedSpeed,
        estimatedRelativeSpeed = track.estimatedRelativeSpeed
      }
    end
  end

  local obstacles = {}
  if model then
    for i = 1, math.min(#(model.obstacles or {}), 64) do
      local obstacle = model.obstacles[i]
      obstacles[#obstacles + 1] = {
        obstacleId = obstacle.obstacleId,
        position = state._roadmindHelpers.cloneWorldModelValue(obstacle.position),
        dataAge = obstacle.dataAge,
        confidence = obstacle.confidence,
        sourceSensors = state._roadmindHelpers.copySensorSources(obstacle.sourceSensors)
      }
    end
  end

  local route = {}
  for i = 1, math.min(#(state.routePath or {}), 48) do
    local point = state.routePath[i]
    if type(point) == "table" then
      route[#route + 1] = {x = point.x, y = point.y, z = point.z}
    end
  end

  return {
    schemaVersion = 1,
    timestamp = state.simulationTime,
    frameAge = state.frameAge,
    vehicle = vehicle,
    targets = targets,
    obstacles = obstacles,
    road = state._roadmindHelpers.cloneWorldModelValue(state.roadState or {
      valid = false, quality = "unavailable", corridor = {}
    }),
    route = route,
    status = {
      scanning = state.visualizationEnabled == true,
      cameraDetectionEnabled = state.settings and state.settings.cameraDetectionEnabled == true,
      routePilotEnabled = state.routePilotEnabled == true,
      routePilotStarted = state.routePilotStarted == true,
      routeSafetyStopped = state.routeSafetyStopped == true,
      routePilotError = state.routePilotError,
      plannerStatus = state.localPlanner and state.localPlanner.status or "idle"
    }
  }
end


function M.getMapSnapshot(lastRoadRevision, lastSurfaceRevision, includeSurface)
  -- Recenter the cached session-map output even after scanning has been stopped.
  -- This exposes previously scanned cells around the new ego position without
  -- requiring new sensor observations.
  local latestPose = state.latestData
    and (state.latestData.vehiclePose or state.latestData.localizationVehiclePose) or nil
  local liveMapPosition = latestPose and asVec3(latestPose.position) or nil
  liveMapPosition = liveMapPosition or asVec3(safeCall(state.activeVehicle, "getPosition"))
  local configuredMapRadius = math.max(100, math.min(500,
    tonumber(state.settings and state.settings.mapRadiusM) or 500))
  local cachedOutputCenter = asVec3(state.sessionStaticMapOutputCenter)
  local outputCenterMoved = cachedOutputCenter and liveMapPosition
    and (liveMapPosition - cachedOutputCenter):length() or math.huge
  local outputCacheNeedsRefresh = type(state.sessionStaticMapOutputCache) ~= "table"
    or tonumber(state.sessionStaticMapLastRadius) ~= configuredMapRadius
    or outputCenterMoved >= SESSION_STATIC_MAP_PRUNE_STEP_M

  -- Sensor-driven updates keep their normal World Model cadence. While idle,
  -- refresh only when the compact static-map output window moves or radius changes.
  local worldModel = state.worldModel
  if buildWorldModel and (not worldModel or outputCacheNeedsRefresh) then
    worldModel = buildWorldModel()
    state.worldModel = worldModel
    state.worldModelDirty = false
    state.worldModelUpdateTimer = 0
  end
  worldModel = worldModel or {targets = {}, obstacles = {}, staticGeometry = {}}
  local perception = M.getPerceptionSnapshot(worldModel)
  local routeState = M.getRouteState()
  local vehicle = perception and perception.vehicle or nil
  local vehiclePosition = vehicle and asVec3(vehicle.position) or nil

  if vehicle and state.activeVehicle then
    local velocity = asVec3(safeCall(state.activeVehicle, "getVelocity"))
    vehicle.speedMps = velocity and round2(velocity:length()) or 0
  end

  local road = state._roadmindHelpers.cloneWorldModelValue(state.roadState or {
    valid = false, quality = "unavailable", corridor = {}
  })
  road.roadHalfWidth = tonumber(road.roadHalfWidth) or PLANNER_ROAD_HALF_WIDTH_M
  road.widthSource = road.widthSource or "conservative-navgraph-heuristic"
  road.laneModel = road.laneModel or {
    status = "unknown",
    source = "No verified lane metadata is available on the current road edge"
  }
  local laneGraph = state._roadmindHelpers.cloneWorldModelValue(road.laneGraph or {
    schemaVersion = 1, status = "unavailable", segments = {}, junctions = {}, connections = {}
  })
  road.laneGraph = nil

  local forwardRangeM = tonumber(state.settings and state.settings.mapForwardRangeM) or 120
  local decalRoads, decalRoadStatus = state._roadmindHelpers.buildMapRoadGeometry(
    vehiclePosition, vehicle and vehicle.forward or nil, forwardRangeM)
  local roadRevision = state.mapRoadGeometryCacheRevision or 0
  local includeRoadData = tonumber(lastRoadRevision) ~= roadRevision
  local surfaceRevision = state.sessionStaticMapRevision or 0
  local includeSurfaceData = includeSurface == true
    and tonumber(lastSurfaceRevision) ~= surfaceRevision
  local staticGeometry = nil
  if includeSurfaceData then
    staticGeometry = {}
    for i = 1, #(worldModel.staticGeometry or {}) do
      local item = worldModel.staticGeometry[i]
      local point = item and asVec3(item.position)
      local distance = vehiclePosition and point and (point - vehiclePosition):length() or math.huge
      if point and distance <= 90 and #staticGeometry < 2500 then
        staticGeometry[#staticGeometry + 1] = state._roadmindHelpers.cloneWorldModelValue(item)
      end
    end
  end

  local routePath = routeState and routeState.route and routeState.route.path or {}
  local route = {}
  for i = 1, math.min(#routePath, 180) do
    local point = routePath[i]
    if type(point) == "table" then
      route[#route + 1] = {x = point.x, y = point.y, z = point.z}
    end
  end

  if not state.levelMetadataLoaded then
    state._roadmindHelpers.loadLevelMetadata(false)
  end
  if not state.levelSignalCache or state.simulationTime - (tonumber(state.levelSignalCache.timestamp) or -math.huge) >= 0.75 then
    state._roadmindHelpers.refreshTrafficSignals()
  end

  local nearbyPOI = {}
  for i = 1, #(state.levelPOICache or {}) do
    local item = state.levelPOICache[i]
    local point = asVec3(item.position)
    local distance = vehiclePosition and point and (point - vehiclePosition):length() or math.huge
    if point and distance <= 350 then
      nearbyPOI[#nearbyPOI + 1] = {
        id = item.id, name = item.name, kind = item.kind,
        position = item.position, distance = round2(distance),
        source = item.source, tags = item.tags
      }
    end
  end
  table.sort(nearbyPOI, function(a, b) return a.distance < b.distance end)
  while #nearbyPOI > 80 do table.remove(nearbyPOI) end

  local nearbySignals = {}
  for i = 1, #((state.levelSignalCache and state.levelSignalCache.items) or {}) do
    local item = state.levelSignalCache.items[i]
    local point = asVec3(item.position)
    local distance = vehiclePosition and point and (point - vehiclePosition):length() or math.huge
    if point and distance <= 220 then
      local copy = state._roadmindHelpers.cloneWorldModelValue(item)
      copy.distance = round2(distance)
      local signalRoad = copy.road or {}
      local n1 = signalRoad.n1 ~= nil and tostring(signalRoad.n1) or nil
      local n2 = signalRoad.n2 ~= nil and tostring(signalRoad.n2) or nil
      copy.relevantToCurrentSegment = n1 ~= nil and n2 ~= nil and (
        (n1 == tostring(road.segmentStartNode) and n2 == tostring(road.segmentEndNode))
        or (n1 == tostring(road.segmentEndNode) and n2 == tostring(road.segmentStartNode))
      ) or false
      nearbySignals[#nearbySignals + 1] = copy
    end
  end
  table.sort(nearbySignals, function(a, b) return a.distance < b.distance end)
  while #nearbySignals > 48 do table.remove(nearbySignals) end

  local planner = routeState and routeState.planner or {
    status = "waiting", reasonCode = "not-ready", candidates = {}
  }
  local selectedPath = nil
  local selectedIndex = tonumber(planner.selectedIndex)
  if selectedIndex and type(planner.candidates) == "table" then
    local candidate = planner.candidates[selectedIndex]
    if type(candidate) == "table" and candidate.safe == true then
      selectedPath = state._roadmindHelpers.cloneWorldModelValue(candidate.path)
    end
  end

  return {
    schemaVersion = 2,
    timestamp = state.simulationTime,
    coordinateFrame = "beamng-world",
    coordinateUnits = "meters",
    vehicle = state._roadmindHelpers.cloneWorldModelValue(vehicle),
    road = road,
    laneGraph = laneGraph,
    roadRevision = roadRevision,
    decalRoads = includeRoadData and decalRoads or nil,
    roadGeometryStatus = {
      source = "items.level.json DecalRoad nodes",
      status = decalRoadStatus,
      roadCount = #decalRoads,
      cacheRoadCount = #(state.levelRoadCache or {}),
      cacheNodeCount = state.levelRoadCacheNodeCount or 0,
      loadActive = state.levelRoadLoadActive == true
    },
    vehicles = state._roadmindHelpers.cloneWorldModelValue(perception.targets or {}),
    staticGeometry = staticGeometry,
    sessionMap = {
      cellCount = state.sessionStaticMapCount or 0,
      generation = state.sessionStaticMapGeneration or 1,
      revision = surfaceRevision,
      distanceTravelledM = round2(state.sessionStaticMapDistance or 0),
      radiusM = tonumber(state.settings and state.settings.mapRadiusM) or 500,
      scope = "rolling-circular-radius",
      coordinateFrame = "beamng-world"
    },
    mapSettings = {
      cameraHeightM = tonumber(state.settings and state.settings.mapCameraHeightM) or 14,
      cameraDistanceM = tonumber(state.settings and state.settings.mapCameraDistanceM) or 20,
      cameraLookAheadM = tonumber(state.settings and state.settings.mapCameraLookAheadM) or 20,
      forwardRangeM = forwardRangeM,
      radiusM = tonumber(state.settings and state.settings.mapRadiusM) or 500,
      maxPolygons = tonumber(state.settings and state.settings.mapMaxPolygons) or 3000,
      renderFps = tonumber(state.settings and state.settings.mapRenderFps) or 30
    },
    route = route,
    planner = state._roadmindHelpers.cloneWorldModelValue(planner),
    selectedTrajectory = selectedPath,
    sourceStatus = state._roadmindHelpers.cloneWorldModelValue(worldModel.sourceStatus or {}),
    worldModelQuality = state._roadmindHelpers.cloneWorldModelValue(worldModel.quality or {}),
    poi = {
      status = state.levelPOIStatus or "unavailable",
      source = state.levelPOISource or "No level-sites adapter is connected",
      items = nearbyPOI
    },
    semantics = {
      status = "unavailable",
      source = "Regular Drive camera mounts are geometric coverage models, not RGB/semantic detections",
      items = {}
    },
    signals = {
      status = state.levelSignalCache and state.levelSignalCache.status or "unavailable",
      source = state.levelSignalCache and state.levelSignalCache.source or "Traffic signal API unavailable",
      items = nearbySignals,
      timestamp = state.levelSignalCache and state.levelSignalCache.timestamp or nil
    },
    status = {
      enabled = state.mapModeEnabled == true,
      scanning = state.visualizationEnabled == true,
      frameAge = state.frameAge,
      roadQuality = road.quality or "unavailable",
      laneStatus = road.laneModel.status or "unknown",
      laneLocalization = laneGraph.currentLaneStatus or "unknown",
      laneGraphStatus = laneGraph.status or "unavailable",
      laneGraphSegments = laneGraph.segmentCount or 0,
      laneGraphLanes = laneGraph.laneCount or 0,
      laneGraphJunctions = laneGraph.junctionCount or 0,
      laneGraphConnections = laneGraph.connectionCount or 0,
      decalRoadStatus = decalRoadStatus,
      decalRoadSegments = #decalRoads,
      plannerStatus = planner.status or "waiting",
      plannerReason = planner.reason or nil,
      sessionMapCells = state.sessionStaticMapCount or 0,
      poiStatus = state.levelPOIStatus or "unavailable",
      signalStatus = state.levelSignalCache and state.levelSignalCache.status or "unavailable"
    }
  }
end

return M
