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
  poseHz = true, showCameras = true, showLocalization = true, showCalibration = true
}

local MAX_PERSISTENT_LIDAR_POINTS = 600
local PERSISTENT_POINT_CELL_SIZE = 0.35
local MAX_LIDAR_RENDER_POINTS = 99
local MAX_LIDAR_FADE_POINTS = 198
local LIDAR_RENDER_SMOOTHING_SECONDS = 0.10
local LIDAR_POINT_FADE_SECONDS = 0.40
local LIDAR_RENDER_POINT_MAX_AGE = 2.25
local LIDAR_STATIC_MATCH_DISTANCE = 0.60
local LIDAR_TRACK_LOCAL_MATCH_DISTANCE = 0.85
local VEHICLE_REGISTRY_INTERVAL = 0.5
local VEHICLE_DETECTION_INTERVAL = 0.25
local MAX_REGISTRY_VEHICLES = 256
local MAX_OCCLUSION_CHECKS_PER_CYCLE = 4
local OCCLUSION_CACHE_MAX_AGE = 0.75
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

local state = {
  activeVehicleId = nil,
  activeVehicle = nil,
  visualizationEnabled = false,
  settings = nil,
  latestData = nil,
  frameAge = math.huge,
  vehicleGeometry = nil,
  vehicleCalibrationError = nil,
  vehicleRegistry = {},
  vehicleRegistryById = {},
  vehicleRegistrySnapshotValid = false,
  vehicleIteratorContext = {},
  vehicleRegistryTimer = 0,
  vehicleRegistryGeneration = 0,
  vehicleRegistryTruncated = false,
  vehicleDetectionTimer = 0,
  vehicleCandidates = {},
  occlusionCache = {},
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
  lidarRenderPoints = {},
  lidarFadingPoints = {},
  lidarRenderScanTimestamp = 0,
  lidarRenderScanSequence = 0,
  routePilotEnabled = false,
  routePilotStarted = false,
  routeSafetyStopped = false,
  routePilotVehicleId = nil,
  routePilotError = nil,
  routeRetryCooldown = 0,
  routePath = {},
  routePathEnd = nil
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

local function computeVehicleGeometry(vehicle)
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

local function vehicleGeometryCommand(profile)
  local encoded = "{}"
  if type(jsonEncode) == "function" then
    local ok, value = pcall(jsonEncode, profile or {schema = 1, valid = false})
    if ok and type(value) == "string" then encoded = value end
  end
  return "extensions.load('roadmind_sensors'); roadmind_sensors.setVehicleGeometry("
    .. string.format("%q", encoded) .. ")"
end

local function calibrateVehicleGeometry(vehicle)
  if not vehicle then
    state.vehicleGeometry = nil
    state.vehicleCalibrationError = "No active vehicle"
    return false
  end

  local profile, reason = computeVehicleGeometry(vehicle)
  state.vehicleGeometry = profile
  state.vehicleCalibrationError = profile and nil or reason

  queueVehicleCommand(vehicle, vehicleGeometryCommand(profile))
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

local function buildForwardRoadPath(vehicle)
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

            if headingAlignment > -0.15 and targetError <= 180 then
              local score = length + targetError * 2 - headingAlignment * 25
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

local function startNativeRouteAI(vehicle)
  if not vehicle then return false, "No active vehicle" end

  local pathIds, pathPoints, pathError = buildForwardRoadPath(vehicle)
  if not pathIds then return false, pathError end

  local serializedNodes = {}
  for i = 1, #pathIds do
    serializedNodes[i] = string.format("%q", tostring(pathIds[i]))
  end

  local speed = string.format("%.2f", ROUTE_MAX_SPEED_MPS)
  local command =
    "if not ai then extensions.load('ai') end; " ..
    "if ai and ai.driveUsingPath then " ..
    "ai.driveUsingPath({wpTargetList={" .. table.concat(serializedNodes, ",") .. "}, " ..
    "driveInLane='on', avoidCars='on', routeSpeedMode='limit', " ..
    "routeSpeed=" .. speed .. ", aggression=0.35}) " ..
    "else error('RoadMind: vehicle AI path API unavailable') end"

  if not queueVehicleCommand(vehicle, command) then
    return false, "Could not queue the vehicle AI route command"
  end

  state.routePath = pathPoints
  state.routePathEnd = pathPoints[#pathPoints]
  return true
end

local function stopNativeRouteAI(vehicle)
  if not vehicle then return false end
  return queueVehicleCommand(vehicle,
    "if not ai then extensions.load('ai') end; " ..
    "if ai and ai.setMode then pcall(ai.setMode, 'stopping') end"
  )
end

local function clearLidarRenderState()
  state.lidarRenderPoints = {}
  state.lidarFadingPoints = {}
  state.lidarRenderScanTimestamp = 0
  state.lidarRenderScanSequence = 0
end

local function clearPersistentLidarPoints()
  state.lidarPointCloud = {}
  state.lidarPointCells = {}
  state.lidarPointWriteIndex = 1
  clearLidarRenderState()
end

local function resetTrackingState(resetClock)
  state.tracks = {}
  state.tracksBySourceId = {}
  state.lidarTrackGrid = {}
  state.trackCount = 0
  state.occlusionCache = {}
  state.occlusionCursor = 1
  state.vehicleCandidates = {}
  if resetClock then state.simulationTime = 0 end
  if state.latestData then
    state.latestData.vehicles = {}
    state.latestData.tracks = {}
  end
end

local function allocateRoadMindTrackId()
  local id = string.format("RM-%06d", state.nextTrackId)
  state.nextTrackId = state.nextTrackId + 1
  return id
end

local function copySensorSources(sources)
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

local function makeTrackObservation(candidate)
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
    sensorOrigin = asVec3(candidate.sensorOrigin),
    position = position,
    dimensions = candidate.dimensions,
    orientation = candidate.orientation,
    simulatorVelocity = asVec3(candidate.simulatorVelocity),
    observationTimestamp = observationTimestamp,
    visibilityTimestamp = visibilityTimestamp,
    visibilityAge = math.max(0, now - visibilityTimestamp),
    distance = tonumber(candidate.distance),
    sourceSensors = copySensorSources(candidate.sensorSources),
    sourceKind = candidate.sourceKind or "simulator-assisted",
    measurementKind = candidate.measurementKind or "vehicle OBB registry"
  }
end

local function estimateTrackVelocity(track, position, timestamp)
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

local function updateObjectTracks(candidates)
  local now = state.simulationTime
  local seenTracks = {}
  local playerPosition = state.activeVehicle and asVec3(safeCall(state.activeVehicle, "getPosition")) or nil
  local playerVelocity = state.activeVehicle and asVec3(safeCall(state.activeVehicle, "getVelocity")) or nil

  for i = 1, #(candidates or {}) do
    local observation = makeTrackObservation(candidates[i])
    if observation then
      local sourceKey = tostring(observation.sourceVehicleId)
      local track = state.tracksBySourceId[sourceKey]

      if not track and state.trackCount < TRACK_MAX_COUNT then
        track = {
          trackId = allocateRoadMindTrackId(),
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
          estimateTrackVelocity(track, observation.position, observation.observationTimestamp)
          track.position = vectorToPlainTable(observation.position)
          track.orientation = observation.orientation
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
  clearLidarRenderState()
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
  state.routePilotError = nil
  state.routeRetryCooldown = 0
  state.routePath = {}
  state.routePathEnd = nil
  state.vehicleGeometry = nil
  state.vehicleCalibrationError = nil
  resetTrackingState(true)
  state.vehicleRegistry = {}
  state.vehicleRegistryById = {}
  state.vehicleRegistrySnapshotValid = false
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
  clearPersistentLidarPoints()
  resetFrame()
end

local function attachVehicle(vehicle, id)
  state.activeVehicle = vehicle
  state.activeVehicleId = id
  resetFrame()
  calibrateVehicleGeometry(vehicle)
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
  if type(lidar) ~= "table" then return end
  local origin = toPoint3F(lidar.origin)
  if not origin or type(lidar.rays) ~= "table" then return end

  local beamColor = ColorF(0.10, 0.76, 1.0, 0.72)
  drawSphere(drawer, origin, 0.075, beamColor)
  for i = 1, #lidar.rays do
    local ray = lidar.rays[i]
    if ray.hit == true then
      local hitPoint = toPoint3F(ray)
      if hitPoint then drawLine(drawer, origin, hitPoint, beamColor) end
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

local function drawDetectedVehicles(drawer, tracks)
  if type(tracks) ~= "table" then return end
  for i = 1, #tracks do
    local track = tracks[i]
    if track.sourceKind == "simulator-assisted" then
      local displayPosition = track.status == "confirmed"
        and track.position or (track.predictedPosition or track.position)
      local position = toPoint3F(displayPosition)
      local origin = toPoint3F(track.sensorOrigin)
      if position then
        local color
        if type(track.estimatedRelativeSpeed) == "number" and track.estimatedRelativeSpeed < -1.0 then
          color = ColorF(1.0, 0.18, 0.12, 1.0)
        elseif track.status == "confirmed" then
          color = ColorF(0.30, 1.0, 0.40, 1.0)
        else
          color = ColorF(1.0, 0.72, 0.18, 0.8)
        end
        if origin then drawLine(drawer, origin, position, color) end
        local radius = track.status == "confirmed" and 0.42 or 0.32
        drawSphere(drawer, position, radius, color)
      end
    end
  end
end

local function drawRoutePath(drawer)
  if not state.routePilotEnabled or type(state.routePath) ~= "table" then return end
  local color = ColorF(0.95, 0.58, 0.12, 0.88)
  for i = 1, #state.routePath - 1 do
    local a = state.routePath[i]
    local b = state.routePath[i + 1]
    if a and b then
      drawLine(drawer, toPoint3F(a), toPoint3F(b), color)
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
    drawLidarRenderPoints(drawer)
    drawPersistentLidarCloud(drawer)
  end
  if settings.showRadar then drawRadar(drawer, frame.radar) end
  if settings.showUltrasonic then drawUltrasonic(drawer, frame.ultrasonic) end
  if settings.showDetectedVehicles then drawDetectedVehicles(drawer, frame.tracks or {}) end

  if settings.showCameras then
    local cameras = frame.cameras or {}
    for i = 1, #cameras do drawCameraFrustum(drawer, cameras[i]) end
  end

  if settings.showLocalization then drawLocalization(drawer, frame.localization) end
  if settings.showCalibration then
    drawCalibrationGeometry(drawer, frame.geometryDebug, frame.mounts)
  end
  drawRoutePath(drawer)
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
    "showLocalization", "showDetectedVehicles", "showCalibration"
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

local function vehicleDetectionEnabled(settings)
  return settings.lidarEnabled or settings.radarEnabled or settings.ultrasonicEnabled
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
  local trackedObjects = {}
  local tracks = state.latestData and state.latestData.tracks or {}
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
    language = getGameLanguage(),
    settings = copySettings(settings),
    status = {
      visualizationEnabled = state.visualizationEnabled,
      activeVehicle = state.activeVehicleId ~= nil,
      estimatedRaycastsPerSecond = state.visualizationEnabled
        and (rayCountFor(settings) + state.occlusionRaycastsPerSecond) or 0,
      vehicleRegistryCount = #state.vehicleRegistry,
      vehicleRegistryGeneration = state.vehicleRegistryGeneration,
      vehicleRegistryTruncated = state.vehicleRegistryTruncated,
      vehicleCandidateCount = #state.vehicleCandidates,
      visibleVehicleCount = state.latestData and (function()
        local count = 0
        for i = 1, #(state.latestData.vehicles or {}) do
          if state.latestData.vehicles[i].visibility == "visible" then count = count + 1 end
        end
        return count
      end)() or 0,
      occlusionRaycastsPerSecond = state.occlusionRaycastsPerSecond,
      routePilotEnabled = state.routePilotEnabled,
      routeSafetyStopped = state.routeSafetyStopped,
      routePilotError = state.routePilotError,
      routeWaypointCount = #state.routePath,
      persistentLidarPointCount = #state.lidarPointCloud,
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
  if not state.settings then loadSettings() end
  if not state.activeVehicle then syncPlayerVehicle() end
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
    resetTrackingState(true)
    clearLidarRenderState()
    state.vehicleRegistry = {}
    state.vehicleRegistryById = {}
    state.vehicleRegistrySnapshotValid = false
    state.vehicleRegistryTimer = 0
    state.vehicleDetectionTimer = 0
    state.vehicleCandidates = {}
    state.occlusionRaycastsLastCycle = 0
    state.occlusionRaycastsPerSecond = 0
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
    state.vehicleRegistry = {}
    state.vehicleRegistryById = {}
    state.vehicleRegistrySnapshotValid = false
    state.vehicleRegistryTimer = 0
    state.vehicleRegistryTruncated = false
    state.vehicleDetectionTimer = 0
    state.vehicleCandidates = {}
    resetTrackingState(false)
    clearLidarRenderState()
    state.occlusionRaycastsLastCycle = 0
    state.occlusionRaycastsPerSecond = 0
    state.routePilotEnabled = false
    state.routePilotStarted = false
    state.routeSafetyStopped = false
    state.routePilotVehicleId = nil
    state.routePilotError = nil
    state.routeRetryCooldown = 0
    state.routePath = {}
    state.routePathEnd = nil
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
    state.routePilotError = nil
    state.routeRetryCooldown = 0
    state.routePath = {}
    state.routePathEnd = nil
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
  state.routePilotError = "Waiting for fresh sensor data"
  state.routeRetryCooldown = 0
  state.routePath = {}
  state.routePathEnd = nil
  resetFrame()

  log("I", "roadmind", "Experimental road-network route pilot armed; waiting for fresh sensor data")
  return settingsResult()
end

function M.clearLidarMap()
  clearPersistentLidarPoints()
  return settingsResult()
end

function M.recalibrateVehicle()
  if not state.activeVehicle then syncPlayerVehicle() end
  if not state.activeVehicle then
    return {ok = false, error = "No active player vehicle"}
  end

  local calibrated = calibrateVehicleGeometry(state.activeVehicle)
  if state.visualizationEnabled then setVehicleSensorSettings(state.activeVehicle) end
  resetFrame()

  local result = settingsResult()
  result.calibrationSucceeded = calibrated
  return result
end

local function invalidateVehicleGeometry(vehicleId)
  if vehicleId == nil or tostring(vehicleId) == tostring(state.activeVehicleId) then
    resetTrackingState(false)
    state.vehicleRegistry = {}
    state.vehicleRegistryById = {}
    state.vehicleRegistrySnapshotValid = false
    state.vehicleRegistryTimer = 0
    if state.activeVehicle then M.recalibrateVehicle() end
  end
end

function M.onVehicleResetted(vehicleId)
  invalidateVehicleGeometry(vehicleId)
end

function M.onVehicleReset(vehicleId)
  invalidateVehicleGeometry(vehicleId)
end

function M.onVehicleConfigurationChanged(vehicleId)
  invalidateVehicleGeometry(vehicleId)
end

function M.onVehicleConfigChanged(vehicleId)
  invalidateVehicleGeometry(vehicleId)
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

local function trackCanOwnLidarPoint(track)
  if type(track) ~= "table" then return false end
  local age = state.simulationTime - (track.lastObservedAt or -math.huge)
  return age >= 0 and age <= TRACK_OBSERVATION_MAX_AGE
    and (track.status == "confirmed" or track.status == "coasting")
    and (track.confidence or 0) >= 0.30
end

local function localizeLidarPointOnTrack(point, track)
  if not trackCanOwnLidarPoint(track) then return nil end
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

local function findTrackForLidarPoint(point)
  local sourceVehicleId = point and point.hitObjectId
  if sourceVehicleId ~= nil then
    local direct = state.tracksBySourceId[tostring(sourceVehicleId)]
    local localPosition = direct and localizeLidarPointOnTrack(point, direct) or nil
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
    if trackCanOwnLidarPoint(track) then
      local localPosition = localizeLidarPointOnTrack(point, track)
      if localPosition then
        if match then return nil, nil end
        match, matchLocal = track, localPosition
      end
    end
  end
  return match, matchLocal
end

local function pushLidarFadePoint(point)
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

  for i = 1, #lidar.rays do
    local rawPoint = lidar.rays[i]
    local rayIndex = tonumber(rawPoint.rayIndex)
    local worldPoint = asVec3(rawPoint)
    if rawPoint.hit == true and rayIndex and worldPoint and rayIndex >= 1 then
      local track, localPosition = findTrackForLidarPoint(rawPoint)
      local associationKey = track and ("track:" .. track.trackId) or "static"
      local previous = oldPoints[rayIndex]
      local isSameReturn = false

      if previous and previous.associationKey == associationKey then
        if track and localPosition and previous.localPosition then
          local deltaForward = localPosition.forward - previous.localPosition.forward
          local deltaRight = localPosition.right - previous.localPosition.right
          local deltaUp = localPosition.up - previous.localPosition.up
          local localDistance = math.sqrt(
            deltaForward * deltaForward + deltaRight * deltaRight + deltaUp * deltaUp
          )
          isSameReturn = localDistance <= LIDAR_TRACK_LOCAL_MATCH_DISTANCE
        elseif not track and previous.worldMeasurement then
          isSameReturn = (worldPoint - previous.worldMeasurement):length() <= LIDAR_STATIC_MATCH_DISTANCE
        end
      end

      local renderPosition = vectorToPlainTable(worldPoint)
      local opacity = 1
      if isSameReturn then
        renderPosition = previous.renderPosition
        opacity = previous.opacity or 1
        matchedOld[rayIndex] = true
      elseif previous then
        pushLidarFadePoint(previous)
      end

      pointCount = pointCount + 1
      local worldMeasurement = nil
      if not track then worldMeasurement = vectorToPlainTable(worldPoint) end
      if pointCount <= MAX_LIDAR_RENDER_POINTS then
        nextPoints[rayIndex] = {
          rayIndex = rayIndex,
          associationKey = associationKey,
          trackId = track and track.trackId or nil,
          localPosition = localPosition,
          worldMeasurement = worldMeasurement,
          renderPosition = renderPosition,
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
      if not replaced then pushLidarFadePoint(previous) end
    end
  end

  state.lidarRenderPoints = nextPoints
  state.lidarRenderScanTimestamp = tonumber(lidar.scanTimestamp) or state.simulationTime
  state.lidarRenderScanSequence = tonumber(lidar.scanSequence) or state.lidarRenderScanSequence
end

local function getTrackedLidarWorldPosition(point)
  if not point.trackId or not point.localPosition then return asVec3(point.worldMeasurement) end
  local track = state.tracks[point.trackId]
  if not track or not trackCanOwnLidarPoint(track) then return nil end

  local position = asVec3(track.position)
  local velocity = asVec3(track.estimatedVelocity)
  if position and velocity then
    local elapsed = math.max(0, state.simulationTime - (track.lastObservedAt or state.simulationTime))
    local predictionHorizon = math.min(elapsed, 0.5)
    position = position + velocity * predictionHorizon
  end

  local orientation = track.orientation or {}
  local forward = normalizedVec3(orientation.forward)
  local right = normalizedVec3(orientation.right)
  local up = normalizedVec3(orientation.up)
  if not position or not forward or not right or not up then return nil end

  local localPosition = point.localPosition
  return position
    + forward * localPosition.forward
    + right * localPosition.right
    + up * localPosition.up
end

local function updateLidarRenderPoints(dt)
  dt = math.max(0, math.min(tonumber(dt) or 0, 0.1))
  local smoothing = 1 - math.exp(-dt / LIDAR_RENDER_SMOOTHING_SECONDS)

  for rayIndex, point in pairs(state.lidarRenderPoints) do
    local pointAge = state.simulationTime - (point.measurementTimestamp or state.simulationTime)
    local targetPosition = pointAge <= LIDAR_RENDER_POINT_MAX_AGE
      and getTrackedLidarWorldPosition(point) or nil
    local currentPosition = asVec3(point.renderPosition)
    if not targetPosition or not currentPosition then
      pushLidarFadePoint(point)
      state.lidarRenderPoints[rayIndex] = nil
    else
      local delta = targetPosition - currentPosition
      local distance = delta:length()
      if distance > 4.0 then
        -- Large discontinuities are likely a changed surface or a broken association.
        pushLidarFadePoint(point)
        state.lidarRenderPoints[rayIndex] = nil
      else
        local step = delta * smoothing
        local stepLength = step:length()
        if stepLength > 0.75 then step = step * (0.75 / stepLength) end
        point.renderPosition = vectorToPlainTable(currentPosition + step)
      end
    end
  end

  for i = #state.lidarFadingPoints, 1, -1 do
    local point = state.lidarFadingPoints[i]
    point.remaining = point.remaining - dt
    point.opacity = math.max(0, point.remaining / LIDAR_POINT_FADE_SECONDS)
    if point.remaining <= 0 then table.remove(state.lidarFadingPoints, i) end
  end
end

local function rememberLidarPoints(lidar)
  if not state.settings or not state.settings.lidarPersistenceEnabled
      or type(lidar) ~= "table" or type(lidar.rays) ~= "table" then
    return
  end

  for i = 1, #lidar.rays do
    local point = lidar.rays[i]
    if point.hit == true and type(point.x) == "number"
        and type(point.y) == "number" and type(point.z) == "number"
        and not findTrackForLidarPoint(point) then
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

local function snapshotVehicleBounds(vehicle, vehicleId)
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

  return {
    id = vehicleId,
    center = center,
    position = position,
    velocity = velocity,
    forward = forward,
    right = right,
    up = up,
    dimensions = {length = length, width = width, height = height},
    axes = axes,
    half = {hx, hy, hz},
    radius = radius,
    registryTimestamp = state.simulationTime
  }
end

local function refreshVehicleRegistry()
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
      local entry = snapshotVehicleBounds(vehicle, vehicleId)
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
  state.vehicleRegistryTruncated = truncated
  state.vehicleRegistryGeneration = state.vehicleRegistryGeneration + 1
  return true
end

local function projectedBoxRadius(target, axis)
  return target.half[1] * math.abs(target.axes[1]:dot(axis))
    + target.half[2] * math.abs(target.axes[2]:dot(axis))
    + target.half[3] * math.abs(target.axes[3]:dot(axis))
end

local function buildSensorZones(data, settings)
  local zones = {}
  local source = type(data.detectionZones) == "table" and data.detectionZones.zones or nil
  if type(source) ~= "table" then return zones end

  for i = 1, #source do
    local raw = source[i]
    local enabled = (raw.sensor == "lidar" and settings.lidarEnabled)
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

local function targetInsideSensorZone(target, zone)
  local delta = target.center - zone.origin
  local distanceSquared = delta:dot(delta)
  local maxDistance = zone.range + target.radius
  if distanceSquared > maxDistance * maxDistance then return false end

  local along = delta:dot(zone.forward)
  local lateral = delta:dot(zone.right)
  local vertical = delta:dot(zone.up)

  local radiusForward = projectedBoxRadius(target, zone.forward)
  local radiusRight = projectedBoxRadius(target, zone.right)
  local radiusUp = projectedBoxRadius(target, zone.up)

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

local function checkTargetOcclusion(zone, target)
  if type(castRay) ~= "function" then return "unavailable", false end
  local delta = target.center - zone.origin
  local centerDistance = delta:length()
  if centerDistance < 0.01 then return "unverified", false end

  local ok, hit = pcall(castRay, zone.origin, target.center, true, true)
  if not ok or not hit then return "unverified", true end

  local hitDistance = safeField(hit, "dist")
  local hitObject = safeField(hit, "obj")
  local hitObjectId = getVehicleId(hitObject)
  if hitObjectId and tostring(hitObjectId) == tostring(target.id) then
    return "visible", true
  end

  local rayDirection = delta / centerDistance
  local expectedTargetSurface = math.max(0,
    centerDistance - projectedBoxRadius(target, rayDirection))
  if type(hitDistance) == "number" and hitDistance < expectedTargetSurface - 0.4 then
    return "occluded", true
  end
  if type(hitDistance) == "number" and hitDistance <= centerDistance + 0.4 then
    return "visible", true
  end
  return "unverified", true
end

local function detectNearbyVehicles(data)
  local candidates = {}
  local settings = state.settings or DEFAULT_SETTINGS
  if not state.activeVehicle or not vehicleDetectionEnabled(settings) then
    state.vehicleCandidates = {}
    state.occlusionRaycastsLastCycle = 0
    state.occlusionRaycastsPerSecond = 0
    return candidates
  end

  local zones = buildSensorZones(data, settings)
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
    local nearestDistance = math.huge
    local nearestZone = nil
    local nearDistance = math.huge
    local farDistance = 0

    for zoneIndex = 1, #zones do
      local zone = zones[zoneIndex]
      local inside, distance, near, far = targetInsideSensorZone(target, zone)
      if inside then
        sources[#sources + 1] = {sensor = zone.sensor, name = zone.name}
        if distance < nearestDistance then
          nearestDistance = distance
          nearestZone = zone
        end
        nearDistance = math.min(nearDistance, near)
        farDistance = math.max(farDistance, far)
      end
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
        sourceKind = "simulator-assisted",
        measurementKind = "vehicle OBB registry",
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

    if target and cached and state.simulationTime - cached.timestamp <= OCCLUSION_CACHE_MAX_AGE
        and cached.position and (target.center - cached.position):length() <= 1.25 then
      candidate.visibility = cached.visibility
      candidate.visibilityTimestamp = cached.timestamp
      candidate.visibleFrom = cached.sensorName
      candidate.visibilityAge = math.max(0, state.simulationTime - cached.timestamp)
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
    if target then
      local allOccluded = true
      local anyRaycastResult = false
      local anyUnverified = false
      local sourcesChecked = 0
      local lastVisibility = "unverified"
      local lastSensorName = nil

      -- A second sensor may see a target hidden from the nearest sensor.
      for sourceIndex = 1, #candidate.sensorSources do
        if checked >= MAX_OCCLUSION_CHECKS_PER_CYCLE or sourcesChecked >= 2 then
          if sourceIndex <= #candidate.sensorSources then anyUnverified = true end
          break
        end
        local source = candidate.sensorSources[sourceIndex]
        local zone = zonesByName[source.name]
        if zone then
          local visibility, didRaycast = checkTargetOcclusion(zone, target)
          if didRaycast then
            checked = checked + 1
            sourcesChecked = sourcesChecked + 1
            anyRaycastResult = true
          end
          lastVisibility = visibility
          lastSensorName = zone.name
          if visibility == "visible" then
            candidate.visibility = "visible"
            candidate.visibleFrom = zone.name
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
  state.occlusionRaycastsPerSecond = checked / VEHICLE_DETECTION_INTERVAL
  return candidates
end

local buildLidarRenderPoints

local function updateVehicleDetection(dt)
  local elapsed = math.max(0, tonumber(dt) or 0)
  local settings = state.settings or DEFAULT_SETTINGS
  if not state.activeVehicle or not vehicleDetectionEnabled(settings) then
    state.vehicleRegistry = {}
    state.vehicleRegistryById = {}
    state.vehicleRegistrySnapshotValid = false
    state.vehicleCandidates = {}
    state.vehicleRegistryTimer = 0
    state.vehicleRegistryTruncated = false
    state.vehicleDetectionTimer = 0
    state.occlusionCache = {}
    state.occlusionCursor = 1
    state.occlusionRaycastsLastCycle = 0
    state.occlusionRaycastsPerSecond = 0
    resetTrackingState(false)
    return
  end

  state.vehicleRegistryTimer = state.vehicleRegistryTimer + elapsed
  if state.vehicleRegistryTimer >= VEHICLE_REGISTRY_INTERVAL then
    state.vehicleRegistryTimer = state.vehicleRegistryTimer % VEHICLE_REGISTRY_INTERVAL
    refreshVehicleRegistry()
  end

  state.vehicleDetectionTimer = state.vehicleDetectionTimer + elapsed
  if state.vehicleDetectionTimer < VEHICLE_DETECTION_INTERVAL then return end
  state.vehicleDetectionTimer = state.vehicleDetectionTimer % VEHICLE_DETECTION_INTERVAL

  local candidates = {}
  local zoneTimestamp = state.latestData and state.latestData.sensorTimestamps
    and state.latestData.sensorTimestamps.detectionZones or nil
  local zonesFresh = type(zoneTimestamp) == "number"
    and (state.simulationTime - zoneTimestamp) <= TRACK_OBSERVATION_MAX_AGE

  if state.latestData and zonesFresh then
    candidates = detectNearbyVehicles(state.latestData)
  else
    state.vehicleCandidates = {}
    state.occlusionRaycastsLastCycle = 0
    state.occlusionRaycastsPerSecond = 0
  end

  local tracks = updateObjectTracks(candidates)
  if state.latestData then
    state.latestData.vehicles = candidates
    state.latestData.tracks = tracks
  end
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
    local target = data.vehicles[i]
    if target.visibility == "visible" and target.sourceKind == "simulator-assisted" then
      inspect(target.position, originFallback)
    end
  end

  return minimum
end

local function updateRoutePilotSafety(dt)
  if not state.routePilotEnabled then return end

  dt = math.max(0, tonumber(dt) or 0)
  state.routeRetryCooldown = math.max(0, (state.routeRetryCooldown or 0) - dt)

  if not state.activeVehicle or state.routePilotVehicleId ~= state.activeVehicleId then
    state.routePilotEnabled = false
    state.routePilotStarted = false
    state.routeSafetyStopped = false
    state.routePilotVehicleId = nil
    state.routePilotError = "The active vehicle changed"
    state.routePath = {}
    state.routePathEnd = nil
    return
  end

  if state.frameAge > MAX_FRAME_AGE or not state.latestData then
    if state.routePilotStarted then
      stopNativeRouteAI(state.activeVehicle)
      state.routePilotStarted = false
      state.routeSafetyStopped = true
    end
    state.routePilotError = "Waiting for fresh sensor data"
    return
  end

  local clearance = nearestForwardObstacle(state.latestData)
  if clearance <= ROUTE_STOP_DISTANCE then
    if state.routePilotStarted then
      stopNativeRouteAI(state.activeVehicle)
      state.routePilotStarted = false
      log("W", "roadmind", "Route pilot paused by sensor obstacle guard")
    end
    state.routeSafetyStopped = true
    state.routePilotError = string.format("Obstacle inside %.0f m safety distance", ROUTE_STOP_DISTANCE)
    return
  end

  -- Extend the route before its last NavGraph waypoint is reached.
  if state.routePilotStarted and state.routePathEnd then
    local position = safeCall(state.activeVehicle, "getPosition")
    if position then
      local endpoint = state.routePathEnd
      local dx, dy = endpoint.x - position.x, endpoint.y - position.y
      local distance = math.sqrt(dx * dx + dy * dy)
      if distance <= ROUTE_REPLAN_DISTANCE and state.routeRetryCooldown <= 0 then
        local started, reason = startNativeRouteAI(state.activeVehicle)
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
      local started, reason = startNativeRouteAI(state.activeVehicle)
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

local function mergeSensorFrame(frame)
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
    latest.lidar = processLidar(frame.lidar)
    latest.lidar.scanTimestamp = state.simulationTime
    latest.lidar.scanSequence = tonumber(frame.sequence) or 0
    for i = 1, #(latest.lidar.rays or {}) do
      latest.lidar.rays[i].measurementTimestamp = state.simulationTime
    end
    latest.sensorTimestamps.lidar = state.simulationTime
    rememberLidarPoints(latest.lidar)
    buildLidarRenderPoints(latest.lidar)
  end
  if updates.radar == true then
    latest.radar = processRadar(frame.radar)
    latest.sensorTimestamps.radar = state.simulationTime
  end
  if updates.ultrasonic == true then
    latest.ultrasonic = processUltrasonic(frame.ultrasonic)
    latest.sensorTimestamps.ultrasonic = state.simulationTime
  end
  if updates.detectionZones == true then
    latest.detectionZones = type(frame.detectionZones) == "table"
      and frame.detectionZones or {timestamp = 0, zones = {}}
    latest.sensorTimestamps.detectionZones = state.simulationTime
  end
  if updates.pose == true then
    latest.cameras = type(frame.cameras) == "table" and frame.cameras or {}
    latest.localization = type(frame.localization) == "table" and frame.localization or nil
    latest.mounts = type(frame.mounts) == "table" and frame.mounts or {}
    latest.geometryDebug = type(frame.geometryDebug) == "table" and frame.geometryDebug or nil
  end
  if type(frame.vehicleVelocity) == "table" then latest.vehicleVelocity = frame.vehicleVelocity end

  latest.vehicles = state.vehicleCandidates
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

  state.simulationTime = state.simulationTime + dt
  state.frameAge = state.frameAge + dt
  checkTimer = checkTimer + dt

  if checkTimer >= CHECK_INTERVAL then
    checkTimer = 0
    syncPlayerVehicle()
  end

  -- Refresh the player identity before building targets so a vehicle switch
  -- cannot briefly register the new player car as an external target.
  updateVehicleDetection(dt)
  updateLidarRenderPoints(dt)

  if state.routePilotEnabled then
    updateRoutePilotSafety(dt)
  end

  -- Scene raycasts run only when Vehicle Lua sends a due sensor scan.
  -- This update callback redraws already cached geometry only.
  drawFrame()
end

function M.onVehicleDestroyed(vehicleId)
  if vehicleId == state.activeVehicleId then
    detachPreviousVehicle()
    resetFrame()
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
  state.lidarTrackGrid = {}
end

return M
