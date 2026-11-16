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
    horizontalSamples = 9,
    verticalSamples = 1,
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
  lidarPointCount = 9,
  lidarHz = 1,
  lidarRange = 45,
  radarEnabled = false,
  radarDensity = "low",
  radarHz = 1,
  ultrasonicEnabled = false,
  ultrasonicDensity = "single",
  ultrasonicHz = 1,
  poseHz = 2,
  speedAdaptiveRateEnabled = true,
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
  detectionZoneTimer = 0,
  schedulerCursor = 1,
  sequence = 0,
  lastLidarCount = 0,
  lastRadarCount = 0,
  lastUltrasonicCount = 0,
  lastForward = nil,
  lastPoseTime = 0,
  elapsed = 0,
  pendingReset = false,
  frame = nil,
  vehicleGeometry = nil,
  sensorLayout = nil
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

local function buildVehicleSensorLayout(geometry)
  if type(geometry) ~= "table" or geometry.valid ~= true then return nil end

  local fMin, fMax = geometry.forwardMin, geometry.forwardMax
  local lMin, lMax = geometry.lateralMin, geometry.lateralMax
  local bottom, top = geometry.bottom, geometry.top
  local length = geometry.length
  local height = geometry.height
  local centerLateral = (lMin + lMax) * 0.5

  local function clamp(value, minimum, maximum)
    return math.max(minimum, math.min(maximum, value))
  end

  local roofFront = fMin + length * 0.72
  local roofHeight = top + math.max(0.08, math.min(0.18, height * 0.04))
  local midHeight = bottom + height * 0.52
  local bumperHeight = bottom + math.max(0.16, math.min(0.32, height * 0.23))
  local mirrorHeight = bottom + height * 0.76
  local front = fMax + 0.07
  local rear = fMin - 0.07
  local left = lMin - 0.055
  local right = lMax + 0.055
  local frontInnerLeft = lMin * 0.28 + centerLateral * 0.72
  local frontOuterLeft = lMin * 0.76 + centerLateral * 0.24
  local frontInnerRight = lMax * 0.28 + centerLateral * 0.72
  local frontOuterRight = lMax * 0.76 + centerLateral * 0.24

  local layout = {
    lidar = {
      maxRange = CONFIG.lidar.maxRange,
      minRange = CONFIG.lidar.minRange,
      horizontalFov = CONFIG.lidar.horizontalFov,
      verticalFov = CONFIG.lidar.verticalFov,
      horizontalSamples = CONFIG.lidar.horizontalSamples,
      verticalSamples = CONFIG.lidar.verticalSamples,
      interval = CONFIG.lidar.interval,
      forwardOffset = clamp(roofFront, fMin + length * 0.35, fMax - length * 0.08),
      lateralOffset = centerLateral,
      upOffset = roofHeight
    },
    radars = {
      {name = "Front long-range", forward = front, lateral = centerLateral, up = midHeight, yaw = 0, fov = 28, range = 160, rays = 5},
      {name = "Front left corner", forward = front, lateral = left, up = midHeight, yaw = -52, fov = 58, range = 65, rays = 3},
      {name = "Front right corner", forward = front, lateral = right, up = midHeight, yaw = 52, fov = 58, range = 65, rays = 3},
      {name = "Rear left corner", forward = rear, lateral = left, up = midHeight, yaw = -128, fov = 58, range = 50, rays = 3},
      {name = "Rear right corner", forward = rear, lateral = right, up = midHeight, yaw = 128, fov = 58, range = 50, rays = 3}
    },
    ultrasonics = {
      {name = "Front left outer", forward = front, lateral = frontOuterLeft, up = bumperHeight, yaw = -4},
      {name = "Front left inner", forward = front, lateral = frontInnerLeft, up = bumperHeight, yaw = 0},
      {name = "Front right inner", forward = front, lateral = frontInnerRight, up = bumperHeight, yaw = 0},
      {name = "Front right outer", forward = front, lateral = frontOuterRight, up = bumperHeight, yaw = 4},
      {name = "Rear left outer", forward = rear, lateral = frontOuterLeft, up = bumperHeight, yaw = -176},
      {name = "Rear left inner", forward = rear, lateral = frontInnerLeft, up = bumperHeight, yaw = 180},
      {name = "Rear right inner", forward = rear, lateral = frontInnerRight, up = bumperHeight, yaw = 180},
      {name = "Rear right outer", forward = rear, lateral = frontOuterRight, up = bumperHeight, yaw = 176},
      {name = "Front left side", forward = fMin + length * 0.72, lateral = left, up = bumperHeight, yaw = -90},
      {name = "Front right side", forward = fMin + length * 0.72, lateral = right, up = bumperHeight, yaw = 90},
      {name = "Rear left side", forward = fMin + length * 0.28, lateral = left, up = bumperHeight, yaw = -90},
      {name = "Rear right side", forward = fMin + length * 0.28, lateral = right, up = bumperHeight, yaw = 90}
    },
    cameras = {
      {name = "Front narrow", forward = roofFront, lateral = centerLateral, up = roofHeight, yaw = 0, hfov = 38, vfov = 24, range = 85},
      {name = "Front wide", forward = roofFront - length * 0.08, lateral = centerLateral, up = roofHeight, yaw = 0, hfov = 120, vfov = 62, range = 32},
      {name = "Left side", forward = fMin + length * 0.72, lateral = left, up = mirrorHeight, yaw = -90, hfov = 100, vfov = 58, range = 24},
      {name = "Right side", forward = fMin + length * 0.72, lateral = right, up = mirrorHeight, yaw = 90, hfov = 100, vfov = 58, range = 24},
      {name = "Rear wide", forward = rear, lateral = centerLateral, up = roofHeight, yaw = 180, hfov = 120, vfov = 62, range = 30},
      {name = "Rear left", forward = fMin + length * 0.12, lateral = left, up = mirrorHeight, yaw = -140, hfov = 90, vfov = 55, range = 22},
      {name = "Rear right", forward = fMin + length * 0.12, lateral = right, up = mirrorHeight, yaw = 140, hfov = 90, vfov = 55, range = 22}
    }
  }

  return layout
end

local function currentSensorLayout()
  return state.sensorLayout or {
    lidar = CONFIG.lidar,
    radars = RADARS,
    ultrasonics = ULTRASONICS,
    cameras = CAMERAS
  }
end

local function buildGeometryDebug(frame)
  local geometry = state.vehicleGeometry
  if not geometry or geometry.valid ~= true then return nil end

  local fMin, fMax = geometry.forwardMin, geometry.forwardMax
  local lMin, lMax = geometry.lateralMin, geometry.lateralMax
  local bottom, top = geometry.bottom, geometry.top
  local function worldPoint(forward, lateral, up)
    local p = frame.position + frame.forward * forward + frame.right * lateral + frame.up * up
    return vectorToPreciseTable(p)
  end

  local corners = {
    worldPoint(fMin, lMin, bottom), worldPoint(fMax, lMin, bottom),
    worldPoint(fMax, lMax, bottom), worldPoint(fMin, lMax, bottom),
    worldPoint(fMin, lMin, top), worldPoint(fMax, lMin, top),
    worldPoint(fMax, lMax, top), worldPoint(fMin, lMax, top)
  }
  return {
    valid = true,
    corners = corners,
    axes = {
      origin = vectorToPreciseTable(frame.position),
      forward = vectorToPreciseTable(frame.position + frame.forward * 1.8),
      right = vectorToPreciseTable(frame.position + frame.right * 1.2),
      up = vectorToPreciseTable(frame.position + frame.up * 1.2)
    }
  }
end

local function buildDetectionZones(frame)
  local layout = currentSensorLayout()
  local zones = {}

  local function add(name, sensor, config, horizontalFov, verticalFov, range, minRange)
    local mount = makeMount(frame, config.forward, config.lateral, config.up or 0, config.yaw)
    if not mount then return end
    zones[#zones + 1] = {
      name = name,
      sensor = sensor,
      origin = vectorToTable(mount.position),
      forward = vectorToPreciseTable(mount.direction),
      right = vectorToPreciseTable(mount.right),
      up = vectorToPreciseTable(mount.up),
      range = range,
      minRange = minRange,
      horizontalFov = horizontalFov,
      verticalFov = verticalFov
    }
  end

  if sensorSettings.lidarEnabled then
    local cfg = layout.lidar
    add("Forward LiDAR", "lidar", {
      forward = cfg.forwardOffset, lateral = cfg.lateralOffset,
      up = cfg.upOffset, yaw = 0
    }, math.deg(cfg.horizontalFov), math.deg(cfg.verticalFov), cfg.maxRange, cfg.minRange)
  end

  if sensorSettings.radarEnabled then
    for i = 1, #layout.radars do
      local cfg = layout.radars[i]
      add(cfg.name, "radar", cfg, cfg.fov, 45, cfg.range, 0.75)
    end
  end

  if sensorSettings.ultrasonicEnabled then
    for i = 1, #layout.ultrasonics do
      local cfg = layout.ultrasonics[i]
      add(cfg.name, "ultrasonic", cfg, 16, 20, CONFIG.ultrasonicRange, 0.08)
    end
  end

  return {
    timestamp = state.elapsed,
    updateHz = sensorSettings.poseHz,
    zones = zones
  }
end

local function buildSensorMountDebug(frame)
  local layout = currentSensorLayout()
  local mounts = {}

  local function add(name, kind, config)
    local mount = makeMount(frame, config.forward, config.lateral, config.up, config.yaw)
    if mount then
      mounts[#mounts + 1] = {
        name = name,
        kind = kind,
        position = vectorToTable(mount.position),
        direction = vectorToPreciseTable(mount.direction)
      }
    end
  end

  add("Forward LiDAR", "lidar", {
    forward = layout.lidar.forwardOffset,
    lateral = layout.lidar.lateralOffset,
    up = layout.lidar.upOffset,
    yaw = 0
  })
  for i = 1, #layout.radars do
    local config = layout.radars[i]
    add(config.name, "radar", config)
  end
  for i = 1, #layout.ultrasonics do
    local config = layout.ultrasonics[i]
    add(config.name, "ultrasonic", {
      forward = config.forward,
      lateral = config.lateral,
      up = config.up or 0.42,
      yaw = config.yaw
    })
  end
  for i = 1, #layout.cameras do
    local config = layout.cameras[i]
    add(config.name, "camera", config)
  end

  return mounts
end



local function scanLidar(frame)
  local cfg = currentSensorLayout().lidar
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
    forward = vectorToPreciseTable(mount.direction),
    right = vectorToPreciseTable(mount.right),
    up = vectorToPreciseTable(mount.up),
    rays = rays,
    rayCount = #rays
  }
end

local function scanRadars(frame)
  local mounts, beams = {}, {}
  local radars = currentSensorLayout().radars
  for i = 1, #radars do
    local cfg = radars[i]
    local mount = makeMount(frame, cfg.forward, cfg.lateral, cfg.up, cfg.yaw)
    if mount then
      mounts[#mounts + 1] = {
        name = cfg.name,
        position = vectorToTable(mount.position),
        direction = vectorToPreciseTable(mount.direction),
        right = vectorToPreciseTable(mount.right),
        up = vectorToPreciseTable(mount.up),
        range = cfg.range,
        fov = cfg.fov,
        verticalFov = 45,
        minRange = 0.75
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
  local ultrasonics = currentSensorLayout().ultrasonics
  for i = 1, #ultrasonics do
    local cfg = ultrasonics[i]
    local mount = makeMount(frame, cfg.forward, cfg.lateral, cfg.up or 0.42, cfg.yaw)
    if mount then
      mounts[#mounts + 1] = {
        name = cfg.name,
        position = vectorToTable(mount.position),
        direction = vectorToPreciseTable(mount.direction),
        right = vectorToPreciseTable(mount.right),
        up = vectorToPreciseTable(mount.up),
        range = CONFIG.ultrasonicRange,
        fov = 16,
        verticalFov = 20,
        minRange = 0.08
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

  local camerasConfig = currentSensorLayout().cameras
  for i = 1, #camerasConfig do
    local cfg = camerasConfig[i]
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
    mounts = {},
    geometryDebug = nil,
    detectionZones = {},
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
  if updates.detectionZones then payload.detectionZones = frame.detectionZones or {} end
  if updates.pose then
    payload.cameras = frame.cameras or {}
    payload.localization = frame.localization
    payload.mounts = frame.mounts or {}
    payload.geometryDebug = frame.geometryDebug
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
    "lidarEnabled", "radarEnabled", "ultrasonicEnabled", "speedAdaptiveRateEnabled",
    "cameraFrustumsEnabled", "localizationEnabled", "calibrationDebugEnabled"
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
  if type(source.showCalibration) == "boolean" then
    sensorSettings.calibrationDebugEnabled = source.showCalibration
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

  if type(source.lidarPointCount) == "number" and source.lidarPointCount >= 9
      and source.lidarPointCount <= 33 then
    sensorSettings.lidarPointCount = math.floor((source.lidarPointCount - 9) / 4 + 0.5) * 4 + 9
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
    low = {vertical = 1},
    balanced = {vertical = 2},
    high = {vertical = 3}
  }
  local resolution = lidarResolution[sensorSettings.lidarQuality]
  CONFIG.lidar.horizontalSamples = sensorSettings.lidarPointCount
  CONFIG.lidar.verticalSamples = resolution.vertical
  CONFIG.lidar.maxRange = sensorSettings.lidarRange
  CONFIG.lidar.interval = 1 / sensorSettings.lidarHz
  CONFIG.radarInterval = 1 / sensorSettings.radarHz
  CONFIG.ultrasonicInterval = 1 / sensorSettings.ultrasonicHz
  CONFIG.poseInterval = 1 / sensorSettings.poseHz

  if state.sensorLayout and state.sensorLayout.lidar then
    state.sensorLayout.lidar.maxRange = CONFIG.lidar.maxRange
    state.sensorLayout.lidar.minRange = CONFIG.lidar.minRange
    state.sensorLayout.lidar.horizontalFov = CONFIG.lidar.horizontalFov
    state.sensorLayout.lidar.verticalFov = CONFIG.lidar.verticalFov
    state.sensorLayout.lidar.horizontalSamples = CONFIG.lidar.horizontalSamples
    state.sensorLayout.lidar.verticalSamples = CONFIG.lidar.verticalSamples
    state.sensorLayout.lidar.interval = CONFIG.lidar.interval
  end

  state.lidarTimer = 0
  state.radarTimer = 0
  state.ultrasonicTimer = 0
  state.poseTimer = 0
  state.detectionZoneTimer = 0
  state.schedulerCursor = 1
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
  state.detectionZoneTimer = 0
  state.schedulerCursor = 1
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

function M.setVehicleGeometry(encodedGeometry)
  if type(encodedGeometry) ~= "string" or type(jsonDecode) ~= "function" then return false end
  local ok, geometry = pcall(jsonDecode, encodedGeometry)
  if not ok or type(geometry) ~= "table" then return false end

  local keys = {
    "forwardMin", "forwardMax", "lateralMin", "lateralMax",
    "bottom", "top", "length", "width", "height"
  }
  for i = 1, #keys do
    local value = geometry[keys[i]]
    if type(value) ~= "number" or value ~= value or value == math.huge or value == -math.huge then
      state.vehicleGeometry = nil
      state.sensorLayout = nil
      state.pendingReset = true
      return false
    end
  end

  if geometry.valid ~= true
      or geometry.length < 1.2 or geometry.length > 30
      or geometry.width < 0.5 or geometry.width > 10
      or geometry.height < 0.45 or geometry.height > 9
      or geometry.forwardMin >= geometry.forwardMax
      or geometry.lateralMin >= geometry.lateralMax
      or geometry.bottom >= geometry.top then
    state.vehicleGeometry = nil
    state.sensorLayout = nil
    state.pendingReset = true
    return false
  end

  state.vehicleGeometry = {
    valid = true,
    source = geometry.source == "getSpawnWorldOOBB" and geometry.source or "validated OBB",
    vehicleKey = type(geometry.vehicleKey) == "string" and geometry.vehicleKey or nil,
    forwardMin = geometry.forwardMin,
    forwardMax = geometry.forwardMax,
    lateralMin = geometry.lateralMin,
    lateralMax = geometry.lateralMax,
    bottom = geometry.bottom,
    top = geometry.top,
    length = geometry.length,
    width = geometry.width,
    height = geometry.height
  }
  state.sensorLayout = buildVehicleSensorLayout(state.vehicleGeometry)
  state.pendingReset = true
  state.lidarTimer, state.radarTimer, state.ultrasonicTimer, state.poseTimer = 0, 0, 0, 0
  return state.sensorLayout ~= nil
end

function M.getStatus()
  return {
    enabled = state.enabled,
    vehicleGeometryCalibrated = state.vehicleGeometry ~= nil,
    vehicleLength = state.vehicleGeometry and state.vehicleGeometry.length or nil,
    vehicleWidth = state.vehicleGeometry and state.vehicleGeometry.width or nil,
    vehicleHeight = state.vehicleGeometry and state.vehicleGeometry.height or nil,
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
  state.detectionZoneTimer = 0
  state.schedulerCursor = 1
  state.lastForward = nil
  state.pendingReset = true
  state.frame = state.enabled and buildEmptyFrame() or nil
end

function M.updateGFX(dt)
  if not state.enabled then return end

  dt = tonumber(dt) or 0
  if dt < 0 then dt = 0 end
  state.elapsed = state.elapsed + dt

  local frame = getVehicleFrame()
  if not frame then return end

  -- Let stationary vehicles use a lower sensor cadence. At higher speed,
  -- tighten the intervals because the scene changes more between scans.
  local speedMultiplier = 1.0
  if sensorSettings.speedAdaptiveRateEnabled ~= false then
    if frame.speed < 5 then
      speedMultiplier = 1.5
    elseif frame.speed >= 25 then
      speedMultiplier = 0.75
    end
  end

  local lidarInterval = CONFIG.lidar.interval * speedMultiplier
  local radarInterval = CONFIG.radarInterval * speedMultiplier
  local ultrasonicInterval = CONFIG.ultrasonicInterval * speedMultiplier
  local poseInterval = CONFIG.poseInterval * speedMultiplier

  if sensorSettings.lidarEnabled then state.lidarTimer = state.lidarTimer + dt end
  if sensorSettings.radarEnabled then state.radarTimer = state.radarTimer + dt end
  if sensorSettings.ultrasonicEnabled then state.ultrasonicTimer = state.ultrasonicTimer + dt end
  if sensorSettings.cameraFrustumsEnabled or sensorSettings.localizationEnabled
      or sensorSettings.calibrationDebugEnabled then
    state.poseTimer = state.poseTimer + dt
  end
  if sensorSettings.lidarEnabled or sensorSettings.radarEnabled or sensorSettings.ultrasonicEnabled then
    state.detectionZoneTimer = state.detectionZoneTimer + dt
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
    state.frame.mounts = {}
    state.frame.geometryDebug = nil
    state.frame.detectionZones = {}
    updatedSensors.detectionZones = true
    updatedSensors.lidar = true
    updatedSensors.radar = true
    updatedSensors.ultrasonic = true
    updatedSensors.pose = true
    state.pendingReset = false
    changed = true
  end

  local lidarDue = sensorSettings.lidarEnabled and state.lidarTimer >= lidarInterval
  local radarDue = sensorSettings.radarEnabled and state.radarTimer >= radarInterval
  local ultrasonicDue = sensorSettings.ultrasonicEnabled
    and state.ultrasonicTimer >= ultrasonicInterval
  local poseDue = (sensorSettings.cameraFrustumsEnabled or sensorSettings.localizationEnabled
      or sensorSettings.calibrationDebugEnabled) and state.poseTimer >= poseInterval
  local zonesDue = state.detectionZoneTimer >= poseInterval
    and (sensorSettings.lidarEnabled or sensorSettings.radarEnabled or sensorSettings.ultrasonicEnabled)

  -- Round-robin over five independent jobs. A graphics frame handles at most
  -- one due scan/pose/zone job, spreading simultaneous deadlines over frames.
  local selectedJob = nil
  for offset = 0, 4 do
    local job = ((state.schedulerCursor + offset - 1) % 5) + 1
    if job == 1 and lidarDue then
      state.lidarTimer = state.lidarTimer % lidarInterval
      state.frame = state.frame or buildEmptyFrame()
      state.frame.lidar = scanLidar(frame)
      updatedSensors.lidar = true
      selectedJob = job
    elseif job == 2 and radarDue then
      state.radarTimer = state.radarTimer % radarInterval
      state.frame = state.frame or buildEmptyFrame()
      state.frame.radar = scanRadars(frame)
      updatedSensors.radar = true
      selectedJob = job
    elseif job == 3 and ultrasonicDue then
      state.ultrasonicTimer = state.ultrasonicTimer % ultrasonicInterval
      state.frame = state.frame or buildEmptyFrame()
      state.frame.ultrasonic = scanUltrasonics(frame)
      updatedSensors.ultrasonic = true
      selectedJob = job
    elseif job == 4 and poseDue then
      local poseDt = state.poseTimer
      state.poseTimer = state.poseTimer % poseInterval
      state.frame = state.frame or buildEmptyFrame()
      state.frame.cameras = buildCameras(frame)
      state.frame.localization = sensorSettings.localizationEnabled
        and updateLocalization(frame, poseDt) or nil
      state.frame.geometryDebug = sensorSettings.calibrationDebugEnabled
        and buildGeometryDebug(frame) or nil
      state.frame.mounts = sensorSettings.calibrationDebugEnabled
        and buildSensorMountDebug(frame) or {}
      updatedSensors.pose = true
      selectedJob = job
    elseif job == 5 and zonesDue then
      state.detectionZoneTimer = state.detectionZoneTimer % poseInterval
      state.frame = state.frame or buildEmptyFrame()
      state.frame.detectionZones = buildDetectionZones(frame)
      updatedSensors.detectionZones = true
      selectedJob = job
    end

    if selectedJob ~= nil then break end
  end

  if selectedJob ~= nil then
    state.schedulerCursor = (selectedJob % 5) + 1
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
