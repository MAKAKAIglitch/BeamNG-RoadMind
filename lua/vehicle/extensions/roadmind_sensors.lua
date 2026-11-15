-- RoadMind virtual automotive sensor suite.
-- Vehicle Lua calculates sensor poses and ray directions. Game Engine Lua
-- performs scene raycasts and renders the resulting geometry in BeamNG.drive.

local M = {}

local CONFIG = {
  lidar = {
    maxRange = 65,
    minRange = 0.35,
    horizontalFov = math.rad(120),
    verticalFov = math.rad(16),
    horizontalSamples = 17,
    verticalSamples = 3,
    interval = 0.25,
    forwardOffset = 0.85,
    lateralOffset = 0,
    upOffset = 1.35
  },

  radarInterval = 0.5,
  ultrasonicInterval = 0.5,
  poseInterval = 0.2,
  ultrasonicRange = 5.0,
  ultrasonicSpread = math.rad(7)
}

local sensorSettings = {
  lidarEnabled = true,
  lidarQuality = "low",
  lidarHz = 1,
  lidarRange = 45,
  radarEnabled = true,
  radarDensity = "low",
  radarHz = 1,
  ultrasonicEnabled = true,
  ultrasonicDensity = "single",
  ultrasonicHz = 1,
  poseHz = 2,
  cameraFrustumsEnabled = false,
  localizationEnabled = false
}

-- Vehicle-local lateral convention follows the sensor basis derived from
-- getDirectionVector() and getDirectionVectorUp().
local RADARS = {
  {name = "Front long-range", forward = 2.15, lateral = 0, up = 0.48, yaw = 0, fov = 28, range = 160, rays = 5},
  {name = "Front left corner", forward = 1.90, lateral = -0.98, up = 0.50, yaw = -52, fov = 58, range = 65, rays = 3},
  {name = "Front right corner", forward = 1.90, lateral = 0.98, up = 0.50, yaw = 52, fov = 58, range = 65, rays = 3},
  {name = "Rear left corner", forward = -1.90, lateral = -0.98, up = 0.50, yaw = -128, fov = 58, range = 50, rays = 3},
  {name = "Rear right corner", forward = -1.90, lateral = 0.98, up = 0.50, yaw = 128, fov = 58, range = 50, rays = 3}
}

local ULTRASONICS = {
  {name = "Front left outer", forward = 2.20, lateral = -0.82, yaw = -4},
  {name = "Front left inner", forward = 2.25, lateral = -0.30, yaw = 0},
  {name = "Front right inner", forward = 2.25, lateral = 0.30, yaw = 0},
  {name = "Front right outer", forward = 2.20, lateral = 0.82, yaw = 4},

  {name = "Rear left outer", forward = -2.20, lateral = -0.82, yaw = -176},
  {name = "Rear left inner", forward = -2.25, lateral = -0.30, yaw = 180},
  {name = "Rear right inner", forward = -2.25, lateral = 0.30, yaw = 180},
  {name = "Rear right outer", forward = -2.20, lateral = 0.82, yaw = 176},

  {name = "Front left side", forward = 1.25, lateral = -0.98, yaw = -90},
  {name = "Front right side", forward = 1.25, lateral = 0.98, yaw = 90},
  {name = "Rear left side", forward = -1.25, lateral = -0.98, yaw = -90},
  {name = "Rear right side", forward = -1.25, lateral = 0.98, yaw = 90}
}

-- Camera frustums are geometric coverage visualizations. BeamNG.drive's
-- regular mod Lua API does not provide the BeamNG.tech RGB camera stream.
local CAMERAS = {
  {name = "Front narrow", forward = 1.20, lateral = 0, up = 1.18, yaw = 0, hfov = 38, vfov = 24, range = 85},
  {name = "Front wide", forward = 1.10, lateral = 0, up = 1.12, yaw = 0, hfov = 120, vfov = 62, range = 32},
  {name = "Left side", forward = 0.25, lateral = -1.02, up = 1.02, yaw = -90, hfov = 100, vfov = 58, range = 24},
  {name = "Right side", forward = 0.25, lateral = 1.02, up = 1.02, yaw = 90, hfov = 100, vfov = 58, range = 24},
  {name = "Rear wide", forward = -1.60, lateral = 0, up = 1.15, yaw = 180, hfov = 120, vfov = 62, range = 30},
  {name = "Rear left", forward = -1.25, lateral = -0.92, up = 1.05, yaw = -140, hfov = 90, vfov = 55, range = 22},
  {name = "Rear right", forward = -1.25, lateral = 0.92, up = 1.05, yaw = 140, hfov = 90, vfov = 55, range = 22}
}

local state = {
  enabled = false,
  sourceVehicleId = nil,
  lidarTimer = 0,
  radarTimer = 0,
  ultrasonicTimer = 0,
  poseTimer = 0,
  sequence = 0,
  lastLidarCount = 0,
  lastRadarCount = 0,
  lastUltrasonicCount = 0,
  lastForward = nil,
  lastPoseTime = 0,
  elapsed = 0,
  pendingReset = false,
  frame = nil
}

local function round2(value)
  return math.floor((value or 0) * 100 + 0.5) / 100
end

local function safeField(object, key)
  if not object then
    return nil
  end

  local ok, value = pcall(function()
    return object[key]
  end)

  if ok then
    return value
  end

  return nil
end

local function safeCall(object, methodName, ...)
  local method = safeField(object, methodName)
  if type(method) ~= "function" then
    return nil
  end

  local ok, result = pcall(method, object, ...)
  if ok then
    return result
  end

  return nil
end

local function normalize(value)
  if not value or type(value.length) ~= "function" then
    return nil
  end

  local length = value:length()
  if length < 0.000001 then
    return nil
  end

  return value / length
end

local function vectorToTable(value)
  return {
    x = round2(value.x),
    y = round2(value.y),
    z = round2(value.z)
  }
end

-- Keep directions at full precision to avoid angular drift at long range.
local function vectorToPreciseTable(value)
  return {
    x = value.x,
    y = value.y,
    z = value.z
  }
end



local function getVehicleFrame()
  if not obj then
    return nil
  end

  local position = safeCall(obj, "getPosition")
  local forward = normalize(safeCall(obj, "getDirectionVector"))
  local up = normalize(safeCall(obj, "getDirectionVectorUp"))
  local velocity = safeCall(obj, "getVelocity")

  if not position or not forward or not up or not velocity then
    return nil
  end

  local right = normalize(forward:cross(up))
  if not right then
    return nil
  end

  return {
    position = position,
    forward = forward,
    right = right,
    up = up,
    velocity = velocity,
    speed = velocity:length()
  }
end

local function makeMount(frame, forwardOffset, lateralOffset, upOffset, yawDegrees)
  local yaw = math.rad(yawDegrees or 0)
  local direction = normalize(
    frame.forward * math.cos(yaw) + frame.right * math.sin(yaw)
  )

  if not direction then
    return nil
  end

  local sensorRight = normalize(direction:cross(frame.up))
  if not sensorRight then
    return nil
  end

  local position = frame.position
    + frame.forward * (forwardOffset or 0)
    + frame.right * (lateralOffset or 0)
    + frame.up * (upOffset or 0)

  return {
    position = position,
    direction = direction,
    right = sensorRight,
    up = frame.up
  }
end



local function scanLidar(frame)
  local cfg = CONFIG.lidar
  local mount = makeMount(frame, cfg.forwardOffset, cfg.lateralOffset, cfg.upOffset, 0)
  if not mount then
    return {origin = nil, rays = {}, rayCount = 0}
  end

  local rays = {}
  for h = 0, cfg.horizontalSamples - 1 do
    local horizontalAngle = -cfg.horizontalFov * 0.5
      + cfg.horizontalFov * h / math.max(1, cfg.horizontalSamples - 1)
    local cosH = math.cos(horizontalAngle)
    local sinH = math.sin(horizontalAngle)

    for verticalIndex = 0, cfg.verticalSamples - 1 do
      local verticalAngle = -cfg.verticalFov * 0.5
        + cfg.verticalFov * verticalIndex / math.max(1, cfg.verticalSamples - 1)
      local cosV = math.cos(verticalAngle)
      local sinV = math.sin(verticalAngle)

      local direction = normalize(
        mount.direction * (cosV * cosH)
          + mount.right * (cosV * sinH)
          + mount.up * sinV
      )
      if direction then
        rays[#rays + 1] = {
          direction = vectorToPreciseTable(direction),
          minRange = cfg.minRange,
          maxRange = cfg.maxRange
        }
      end
    end
  end

  state.lastLidarCount = #rays
  return {
    name = "Forward LiDAR",
    origin = vectorToTable(mount.position),
    horizontalFov = math.deg(cfg.horizontalFov),
    verticalFov = math.deg(cfg.verticalFov),
    range = cfg.maxRange,
    rays = rays,
    rayCount = #rays
  }
end

local function scanRadars(frame)
  local mounts, beams = {}, {}
  for i = 1, #RADARS do
    local cfg = RADARS[i]
    local mount = makeMount(frame, cfg.forward, cfg.lateral, cfg.up, cfg.yaw)
    if mount then
      mounts[#mounts + 1] = {
        name = cfg.name,
        position = vectorToTable(mount.position),
        direction = vectorToPreciseTable(mount.direction),
        range = cfg.range,
        fov = cfg.fov
      }

      local sampleCount = cfg.rays
      if sensorSettings.radarDensity == "low" then
        sampleCount = 1
      elseif sensorSettings.radarDensity == "medium" then
        sampleCount = math.min(2, cfg.rays)
      end

      for rayIndex = 0, sampleCount - 1 do
        local angle = 0
        if sampleCount > 1 then
          angle = math.rad(-cfg.fov * 0.5
            + cfg.fov * rayIndex / (sampleCount - 1))
        end
        local direction = normalize(
          mount.direction * math.cos(angle) + mount.right * math.sin(angle)
        )
        if direction then
          beams[#beams + 1] = {
            name = cfg.name,
            origin = vectorToTable(mount.position),
            direction = vectorToPreciseTable(direction),
            range = cfg.range,
            minRange = 0.75
          }
        end
      end
    end
  end

  state.lastRadarCount = #beams
  return {mounts = mounts, beams = beams, updateHz = 1 / CONFIG.radarInterval}
end

local function scanUltrasonics(frame)
  local mounts, beams = {}, {}
  for i = 1, #ULTRASONICS do
    local cfg = ULTRASONICS[i]
    local mount = makeMount(frame, cfg.forward, cfg.lateral, 0.42, cfg.yaw)
    if mount then
      mounts[#mounts + 1] = {
        name = cfg.name,
        position = vectorToTable(mount.position),
        direction = vectorToPreciseTable(mount.direction),
        range = CONFIG.ultrasonicRange
      }
      local firstOffset, lastOffset = -1, 1
      if sensorSettings.ultrasonicDensity == "single" then
        firstOffset, lastOffset = 0, 0
      end
      for offsetIndex = firstOffset, lastOffset do
        local angle = CONFIG.ultrasonicSpread * offsetIndex
        local direction = normalize(
          mount.direction * math.cos(angle) + mount.right * math.sin(angle)
        )
        if direction then
          beams[#beams + 1] = {
            name = cfg.name,
            origin = vectorToTable(mount.position),
            direction = vectorToPreciseTable(direction),
            range = CONFIG.ultrasonicRange,
            minRange = 0.08
          }
        end
      end
    end
  end
  state.lastUltrasonicCount = #beams
  return {mounts = mounts, beams = beams, updateHz = 1 / CONFIG.ultrasonicInterval}
end

local function buildCameras(frame)
  local cameras = {}
  if not sensorSettings.cameraFrustumsEnabled then
    return cameras
  end

  for i = 1, #CAMERAS do
    local cfg = CAMERAS[i]
    local mount = makeMount(
      frame,
      cfg.forward,
      cfg.lateral,
      cfg.up,
      cfg.yaw
    )

    if mount then
      cameras[#cameras + 1] = {
        name = cfg.name,
        position = vectorToTable(mount.position),
        direction = vectorToPreciseTable(mount.direction),
        up = vectorToPreciseTable(mount.up),
        horizontalFov = cfg.hfov,
        verticalFov = cfg.vfov,
        range = cfg.range
      }
    end
  end

  return cameras
end

local function readElectrics()
  if type(electrics) == "table" and type(electrics.values) == "table" then
    return electrics.values
  end
  return {}
end

local function updateLocalization(frame, dt)
  local values = readElectrics()
  local position = frame.position
  local forward = frame.forward
  local up = frame.up
  local velocity = frame.velocity

  local yawRate = nil
  if state.lastForward and dt > 0.001 then
    local cross = state.lastForward:cross(forward)
    local dot = math.max(-1, math.min(1, state.lastForward:dot(forward)))
    yawRate = math.atan2(cross:dot(up), dot) / dt
  end

  state.lastForward = vec3(forward)

  local imuPosition = position + up * 0.45
  local gnssPosition = position + up * 1.0

  local function readNumber(key)
    local value = values[key]
    if type(value) == "number" then
      return round2(value)
    end
    return nil
  end

  return {
    gnss = {
      position = vectorToTable(gnssPosition),
      worldPosition = vectorToTable(position),
      altitude = round2(position.z),
      source = "BeamNG world coordinates; no latitude/longitude map reference"
    },
    imu = {
      position = vectorToTable(imuPosition),
      acceleration = {
        x = readNumber("accXSmooth"),
        y = readNumber("accYSmooth"),
        z = readNumber("accZSmooth")
      },
      estimatedYawRate = yawRate and round2(yawRate) or nil,
      velocity = vectorToTable(velocity)
    },
    odometry = {
      wheelSpeed = readNumber("wheelspeed"),
      odometerMeters = readNumber("odometer"),
      airSpeed = readNumber("airspeed"),
      speedFromVelocity = round2(frame.speed)
    }
  }
end

local function buildEmptyFrame()
  return {
    enabled = false,
    sourceVehicleId = state.sourceVehicleId,
    sequence = state.sequence,
    updatedSensors = {},
    lidar = {rays = {}, rayCount = 0},
    radar = {mounts = {}, beams = {}},
    ultrasonic = {mounts = {}, beams = {}},
    cameras = {},
    localization = nil,
    vehicleVelocity = nil
  }
end

local function publish()
  local frame = state.frame
  if not frame or not obj
      or type(obj.queueGameEngineLua) ~= "function"
      or type(jsonEncode) ~= "function" then
    return
  end

  state.sequence = state.sequence + 1
  frame.enabled = true
  frame.sourceVehicleId = state.sourceVehicleId
  frame.sequence = state.sequence

  local updates = frame.updatedSensors or {}
  local payload = {
    enabled = true,
    sourceVehicleId = state.sourceVehicleId,
    sequence = state.sequence,
    updatedSensors = updates
  }

  if updates.lidar then payload.lidar = frame.lidar end
  if updates.radar then payload.radar = frame.radar end
  if updates.ultrasonic then payload.ultrasonic = frame.ultrasonic end
  if updates.pose then
    payload.cameras = frame.cameras or {}
    payload.localization = frame.localization
    payload.mounts = frame.mounts or {}
  end

  local okPayload, compactEncoded = pcall(jsonEncode, payload)
  if not okPayload or type(compactEncoded) ~= "string" then return end

  local command = "extensions.roadmind.onSensorData("
    .. string.format("%q", compactEncoded) .. ")"

  pcall(obj.queueGameEngineLua, obj, command)
end

local function clearPublishedFrame()
  local empty = buildEmptyFrame()
  empty.enabled = false

  if obj and type(obj.queueGameEngineLua) == "function"
      and type(jsonEncode) == "function" then
    local ok, encoded = pcall(jsonEncode, empty)
    if ok and type(encoded) == "string" then
      local command = "extensions.roadmind.onSensorData("
        .. string.format("%q", encoded) .. ")"
      pcall(obj.queueGameEngineLua, obj, command)
    end
  end
end

local function applySensorSettings(source)
  if type(source) ~= "table" then return false end

  local booleanKeys = {
    "lidarEnabled", "radarEnabled", "ultrasonicEnabled",
    "cameraFrustumsEnabled", "localizationEnabled"
  }
  for i = 1, #booleanKeys do
    local key = booleanKeys[i]
    if type(source[key]) == "boolean" then sensorSettings[key] = source[key] end
  end

  if type(source.showCameras) == "boolean" then
    sensorSettings.cameraFrustumsEnabled = source.showCameras
  end
  if type(source.showLocalization) == "boolean" then
    sensorSettings.localizationEnabled = source.showLocalization
  end

  if source.lidarQuality == "low" or source.lidarQuality == "balanced"
      or source.lidarQuality == "high" then
    sensorSettings.lidarQuality = source.lidarQuality
  end
  if source.radarDensity == "low" or source.radarDensity == "medium"
      or source.radarDensity == "high" then
    sensorSettings.radarDensity = source.radarDensity
  end
  if source.ultrasonicDensity == "single" or source.ultrasonicDensity == "wide" then
    sensorSettings.ultrasonicDensity = source.ultrasonicDensity
  end

  if source.lidarHz == 1 or source.lidarHz == 2 or source.lidarHz == 4 then
    sensorSettings.lidarHz = source.lidarHz
  end
  if type(source.lidarRange) == "number" and source.lidarRange >= 30
      and source.lidarRange <= 160 then
    sensorSettings.lidarRange = math.floor(source.lidarRange / 5 + 0.5) * 5
  end
  if source.radarHz == 1 or source.radarHz == 2 then sensorSettings.radarHz = source.radarHz end
  if source.ultrasonicHz == 1 or source.ultrasonicHz == 2 then
    sensorSettings.ultrasonicHz = source.ultrasonicHz
  end
  if source.poseHz == 2 or source.poseHz == 4 or source.poseHz == 5 then
    sensorSettings.poseHz = source.poseHz
  end

  local lidarResolution = {
    low = {horizontal = 9, vertical = 1},
    balanced = {horizontal = 13, vertical = 2},
    high = {horizontal = 17, vertical = 3}
  }
  local resolution = lidarResolution[sensorSettings.lidarQuality]
  CONFIG.lidar.horizontalSamples = resolution.horizontal
  CONFIG.lidar.verticalSamples = resolution.vertical
  CONFIG.lidar.maxRange = sensorSettings.lidarRange
  CONFIG.lidar.interval = 1 / sensorSettings.lidarHz
  CONFIG.radarInterval = 1 / sensorSettings.radarHz
  CONFIG.ultrasonicInterval = 1 / sensorSettings.ultrasonicHz
  CONFIG.poseInterval = 1 / sensorSettings.poseHz

  state.lidarTimer = 0
  state.radarTimer = 0
  state.ultrasonicTimer = 0
  state.poseTimer = 0
  state.pendingReset = true
  return true
end

function M.setSettings(encodedSettings)
  if type(encodedSettings) ~= "string" or type(jsonDecode) ~= "function" then
    return false
  end
  local ok, settings = pcall(jsonDecode, encodedSettings)
  if not ok or type(settings) ~= "table" then return false end

  local applied = applySensorSettings(settings)
  if applied and state.enabled then
    state.frame = state.frame or buildEmptyFrame()
  end
  return applied
end

function M.setEnabled(enabled, sourceVehicleId)
  state.enabled = enabled == true
  if type(sourceVehicleId) == "number" then
    state.sourceVehicleId = sourceVehicleId
  end
  state.lidarTimer = 0
  state.radarTimer = 0
  state.ultrasonicTimer = 0
  state.poseTimer = 0
  state.lastForward = nil
  state.lastLidarCount = 0
  state.lastRadarCount = 0
  state.lastUltrasonicCount = 0
  state.pendingReset = true

  if state.enabled then
    state.frame = buildEmptyFrame()
  else
    state.frame = nil
    clearPublishedFrame()
  end

  return state.enabled
end

function M.getStatus()
  return {
    enabled = state.enabled,
    sequence = state.sequence,
    lidarRays = state.lastLidarCount,
    radarRays = state.lastRadarCount,
    ultrasonicRays = state.lastUltrasonicCount,
    cameraCount = #CAMERAS,
    radarCount = #RADARS,
    ultrasonicCount = #ULTRASONICS,
    lidarRayBudget = CONFIG.lidar.horizontalSamples * CONFIG.lidar.verticalSamples
  }
end

function M.onReset()
  state.lidarTimer = 0
  state.radarTimer = 0
  state.ultrasonicTimer = 0
  state.poseTimer = 0
  state.lastForward = nil
  state.pendingReset = true
  state.frame = state.enabled and buildEmptyFrame() or nil
end

function M.updateGFX(dt)
  if not state.enabled then
    return
  end

  dt = tonumber(dt) or 0
  if dt < 0 then
    dt = 0
  end

  state.elapsed = state.elapsed + dt
  if sensorSettings.lidarEnabled then state.lidarTimer = state.lidarTimer + dt end
  if sensorSettings.radarEnabled then state.radarTimer = state.radarTimer + dt end
  if sensorSettings.ultrasonicEnabled then state.ultrasonicTimer = state.ultrasonicTimer + dt end
  if sensorSettings.cameraFrustumsEnabled or sensorSettings.localizationEnabled then
    state.poseTimer = state.poseTimer + dt
  end

  local frame = getVehicleFrame()
  if not frame then
    return
  end

  local changed = false
  local updatedSensors = {}

  if state.pendingReset then
    state.frame = state.frame or buildEmptyFrame()
    state.frame.lidar = {origin = nil, rays = {}, rayCount = 0}
    state.frame.radar = {mounts = {}, beams = {}}
    state.frame.ultrasonic = {mounts = {}, beams = {}}
    state.frame.cameras = {}
    state.frame.localization = nil
    updatedSensors.lidar = true
    updatedSensors.radar = true
    updatedSensors.ultrasonic = true
    updatedSensors.pose = true
    state.pendingReset = false
    changed = true
  end

  if sensorSettings.lidarEnabled and state.lidarTimer >= CONFIG.lidar.interval then
    state.lidarTimer = state.lidarTimer % CONFIG.lidar.interval
    state.frame = state.frame or buildEmptyFrame()
    state.frame.lidar = scanLidar(frame)
    updatedSensors.lidar = true
    changed = true
  end

  if sensorSettings.radarEnabled and state.radarTimer >= CONFIG.radarInterval then
    state.radarTimer = state.radarTimer % CONFIG.radarInterval
    state.frame = state.frame or buildEmptyFrame()
    state.frame.radar = scanRadars(frame)
    updatedSensors.radar = true
    changed = true
  end

  if sensorSettings.ultrasonicEnabled and state.ultrasonicTimer >= CONFIG.ultrasonicInterval then
    state.ultrasonicTimer = state.ultrasonicTimer % CONFIG.ultrasonicInterval
    state.frame = state.frame or buildEmptyFrame()
    state.frame.ultrasonic = scanUltrasonics(frame)
    updatedSensors.ultrasonic = true
    changed = true
  end

  if (sensorSettings.cameraFrustumsEnabled or sensorSettings.localizationEnabled)
      and state.poseTimer >= CONFIG.poseInterval then
    local poseDt = state.poseTimer
    state.poseTimer = state.poseTimer % CONFIG.poseInterval
    state.frame = state.frame or buildEmptyFrame()
    state.frame.cameras = buildCameras(frame)
    state.frame.localization = sensorSettings.localizationEnabled
      and updateLocalization(frame, poseDt) or nil
    state.frame.mounts = state.frame.localization and {
      imu = state.frame.localization.imu.position,
      gnss = state.frame.localization.gnss.position
    } or {}
    updatedSensors.pose = true
    changed = true
  end

  if changed and state.frame then
    state.frame.enabled = true
    state.frame.updatedSensors = updatedSensors
    publish()
    state.frame.updatedSensors = {}
  end
end

return M
