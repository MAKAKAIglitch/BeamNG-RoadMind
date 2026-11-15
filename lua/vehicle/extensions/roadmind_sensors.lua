-- RoadMind virtual automotive sensor suite.
-- Vehicle Lua performs all raycasts; Game Engine Lua renders the returned
-- sensor geometry in world space. This targets BeamNG.drive, not BeamNG.tech.

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
    forwardOffset = 2.15,
    lateralOffset = 0,
    upOffset = 0.78
  },

  radarInterval = 0.5,
  ultrasonicInterval = 0.5,
  poseInterval = 0.2,
  ultrasonicRange = 5.0,
  ultrasonicSpread = math.rad(7)
}

-- Vehicle-local lateral convention follows the sensor basis derived from
-- getDirectionVector() and getDirectionVectorUp().
local RADARS = {
  {name = "Front long-range", forward = 2.15, lateral = 0, up = 0.48, yaw = 0, fov = 28, range = 160, rays = 5},
  {name = "Front left corner", forward = 1.65, lateral = -0.88, up = 0.48, yaw = -52, fov = 58, range = 65, rays = 3},
  {name = "Front right corner", forward = 1.65, lateral = 0.88, up = 0.48, yaw = 52, fov = 58, range = 65, rays = 3},
  {name = "Rear left corner", forward = -1.65, lateral = -0.88, up = 0.48, yaw = -128, fov = 58, range = 50, rays = 3},
  {name = "Rear right corner", forward = -1.65, lateral = 0.88, up = 0.48, yaw = 128, fov = 58, range = 50, rays = 3}
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
  {name = "Front narrow", forward = 2.05, lateral = 0, up = 0.82, yaw = 0, hfov = 38, vfov = 24, range = 85},
  {name = "Front wide", forward = 2.05, lateral = 0, up = 0.76, yaw = 0, hfov = 120, vfov = 62, range = 32},
  {name = "Left side", forward = 0.35, lateral = -0.96, up = 0.78, yaw = -90, hfov = 100, vfov = 58, range = 24},
  {name = "Right side", forward = 0.35, lateral = 0.96, up = 0.78, yaw = 90, hfov = 100, vfov = 58, range = 24},
  {name = "Rear wide", forward = -2.05, lateral = 0, up = 0.80, yaw = 180, hfov = 120, vfov = 62, range = 30},
  {name = "Rear left", forward = -1.45, lateral = -0.86, up = 0.80, yaw = -140, hfov = 90, vfov = 55, range = 22},
  {name = "Rear right", forward = -1.45, lateral = 0.86, up = 0.80, yaw = 140, hfov = 90, vfov = 55, range = 22}
}

local state = {
  enabled = false,
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

local function pointFromHit(hit, origin, direction, distance)
  local hitPoint = safeField(hit, "pt")
  if hitPoint then
    local x = safeField(hitPoint, "x")
    local y = safeField(hitPoint, "y")
    local z = safeField(hitPoint, "z")

    if type(x) == "number" and type(y) == "number" and type(z) == "number" then
      return vec3(x, y, z)
    end
  end

  return origin + direction * distance
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

local function traceRay(origin, direction, maxRange, minRange, vehicleVelocity)
  local staticDistance = nil
  local dynamicHit = nil

  if type(castRayStatic) == "function" then
    local ok, distance = pcall(castRayStatic, origin, direction, maxRange)
    if ok and type(distance) == "number"
        and distance >= minRange and distance <= maxRange then
      staticDistance = distance
    end
  end

  if type(castRay) == "function" then
    local dynamicLimit = staticDistance or maxRange
    local endpoint = origin + direction * dynamicLimit
    local ok, hit = pcall(castRay, origin, endpoint, true, true)

    if ok and hit then
      local hitDistance = safeField(hit, "dist")
      if type(hitDistance) == "number"
          and hitDistance >= minRange and hitDistance <= dynamicLimit then
        dynamicHit = hit
      end
    end
  end

  local distance = staticDistance
  local point = staticDistance and (origin + direction * staticDistance) or nil
  local object = nil

  if dynamicHit then
    local dynamicDistance = safeField(dynamicHit, "dist")
    if not distance or dynamicDistance < distance then
      distance = dynamicDistance
      point = pointFromHit(dynamicHit, origin, direction, dynamicDistance)
      object = safeField(dynamicHit, "obj") or safeField(dynamicHit, "object")
    end
  end

  if not distance or not point then
    return nil
  end

  local targetVelocity = safeCall(object, "getVelocity")
  local relativeRadialSpeed = nil

  if targetVelocity and type(targetVelocity.dot) == "function" then
    local relativeVelocity = targetVelocity - (vehicleVelocity or vec3(0, 0, 0))
    relativeRadialSpeed = relativeVelocity:dot(direction)
  end

  local objectId = safeCall(object, "getID")
    or safeField(dynamicHit, "objectId")
    or safeField(dynamicHit, "objectID")
    or safeField(dynamicHit, "cid")

  return {
    point = point,
    distance = distance,
    objectId = type(objectId) == "number" and objectId or nil,
    dynamic = object ~= nil or objectId ~= nil,
    radialSpeed = relativeRadialSpeed
  }
end

local function scanLidar(frame)
  local cfg = CONFIG.lidar
  local mount = makeMount(
    frame,
    cfg.forwardOffset,
    cfg.lateralOffset,
    cfg.upOffset,
    0
  )

  if not mount then
    return {origin = nil, rays = {}, pointCount = 0}
  end

  local rays = {}
  local hits = 0

  for h = 0, cfg.horizontalSamples - 1 do
    local horizontalAngle = -cfg.horizontalFov * 0.5
      + cfg.horizontalFov * h / math.max(1, cfg.horizontalSamples - 1)

    local cosH = math.cos(horizontalAngle)
    local sinH = math.sin(horizontalAngle)

    for v = 0, cfg.verticalSamples - 1 do
      local verticalAngle = -cfg.verticalFov * 0.5
        + cfg.verticalFov * v / math.max(1, cfg.verticalSamples - 1)

      local cosV = math.cos(verticalAngle)
      local sinV = math.sin(verticalAngle)

      local direction = normalize(
        mount.direction * (cosV * cosH)
          + mount.right * (cosV * sinH)
          + mount.up * sinV
      )

      if direction then
        local result = traceRay(mount.position, direction, cfg.maxRange, cfg.minRange, frame.velocity)
        if result then
          hits = hits + 1
          rays[#rays + 1] = {
            x = round2(result.point.x),
            y = round2(result.point.y),
            z = round2(result.point.z),
            hit = true,
            distance = round2(result.distance)
          }
        end
      end
    end
  end

  state.lastLidarCount = hits

  return {
    name = "Forward LiDAR",
    origin = vectorToTable(mount.position),
    horizontalFov = 120,
    verticalFov = 16,
    range = cfg.maxRange,
    rays = rays,
    pointCount = hits
  }
end

local function scanRadars(frame)
  local mounts = {}
  local returns = {}

  for i = 1, #RADARS do
    local cfg = RADARS[i]
    local mount = makeMount(
      frame,
      cfg.forward,
      cfg.lateral,
      cfg.up,
      cfg.yaw
    )

    if mount then
      mounts[#mounts + 1] = {
        name = cfg.name,
        position = vectorToTable(mount.position),
        direction = vectorToTable(mount.direction),
        range = cfg.range,
        fov = cfg.fov
      }

      local sampleCount = cfg.rays
      for rayIndex = 0, sampleCount - 1 do
        local angle = math.rad(-cfg.fov * 0.5
          + cfg.fov * rayIndex / math.max(1, sampleCount - 1))

        local direction = normalize(
          mount.direction * math.cos(angle)
            + mount.right * math.sin(angle)
        )

        if direction then
          local result = traceRay(mount.position, direction, cfg.range, 0.75, frame.velocity)
          if result then
            returns[#returns + 1] = {
              name = cfg.name,
              origin = vectorToTable(mount.position),
              x = round2(result.point.x),
              y = round2(result.point.y),
              z = round2(result.point.z),
              distance = round2(result.distance),
              dynamic = result.dynamic,
              objectId = result.objectId,
              radialSpeed = result.radialSpeed
            }
          end
        end
      end
    end
  end

  state.lastRadarCount = #returns

  return {
    mounts = mounts,
    returns = returns,
    updateHz = 1 / CONFIG.radarInterval
  }
end

local function scanUltrasonics(frame)
  local mounts = {}
  local returns = {}

  for i = 1, #ULTRASONICS do
    local cfg = ULTRASONICS[i]
    local mount = makeMount(frame, cfg.forward, cfg.lateral, 0.42, cfg.yaw)

    if mount then
      mounts[#mounts + 1] = {
        name = cfg.name,
        position = vectorToTable(mount.position),
        direction = vectorToTable(mount.direction),
        range = CONFIG.ultrasonicRange
      }

      local nearest = nil
      local nearestDistance = CONFIG.ultrasonicRange

      for offsetIndex = -1, 1 do
        local angle = CONFIG.ultrasonicSpread * offsetIndex
        local direction = normalize(
          mount.direction * math.cos(angle)
            + mount.right * math.sin(angle)
        )

        if direction then
          local result = traceRay(
            mount.position,
            direction,
            CONFIG.ultrasonicRange,
            0.08,
            frame.velocity
          )

          if result and result.distance < nearestDistance then
            nearest = result
            nearestDistance = result.distance
          end
        end
      end

      if nearest then
        returns[#returns + 1] = {
          name = cfg.name,
          origin = vectorToTable(mount.position),
          x = round2(nearest.point.x),
          y = round2(nearest.point.y),
          z = round2(nearest.point.z),
          distance = round2(nearest.distance)
        }
      end
    end
  end

  state.lastUltrasonicCount = #returns

  return {
    mounts = mounts,
    returns = returns,
    updateHz = 1 / CONFIG.ultrasonicInterval
  }
end

local function buildCameras(frame)
  local cameras = {}

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
        direction = vectorToTable(mount.direction),
        up = vectorToTable(mount.up),
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
    sequence = state.sequence,
    lidar = {rays = {}, pointCount = 0},
    radar = {mounts = {}, returns = {}},
    ultrasonic = {mounts = {}, returns = {}},
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
  frame.sequence = state.sequence

  local ok, encoded = pcall(jsonEncode, frame)
  if not ok or type(encoded) ~= "string" then
    return
  end

  local command = "extensions.roadmind.onSensorData("
    .. string.format("%q", encoded) .. ")"

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

function M.setEnabled(enabled)
  state.enabled = enabled == true
  state.lidarTimer = 0
  state.radarTimer = 0
  state.ultrasonicTimer = 0
  state.poseTimer = 0
  state.lastForward = nil
  state.lastLidarCount = 0
  state.lastRadarCount = 0
  state.lastUltrasonicCount = 0

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
    lidarPoints = state.lastLidarCount,
    radarReturns = state.lastRadarCount,
    ultrasonicReturns = state.lastUltrasonicCount,
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
  state.lidarTimer = state.lidarTimer + dt
  state.radarTimer = state.radarTimer + dt
  state.ultrasonicTimer = state.ultrasonicTimer + dt
  state.poseTimer = state.poseTimer + dt

  local frame = getVehicleFrame()
  if not frame then
    return
  end

  local changed = false

  if state.lidarTimer >= CONFIG.lidar.interval then
    state.lidarTimer = 0
    state.frame = state.frame or buildEmptyFrame()
    state.frame.lidar = scanLidar(frame)
    changed = true
  end

  if state.radarTimer >= CONFIG.radarInterval then
    state.radarTimer = 0
    state.frame = state.frame or buildEmptyFrame()
    state.frame.radar = scanRadars(frame)
    changed = true
  end

  if state.ultrasonicTimer >= CONFIG.ultrasonicInterval then
    state.ultrasonicTimer = 0
    state.frame = state.frame or buildEmptyFrame()
    state.frame.ultrasonic = scanUltrasonics(frame)
    changed = true
  end

  if state.poseTimer >= CONFIG.poseInterval then
    local poseDt = state.poseTimer
    state.poseTimer = 0
    state.frame = state.frame or buildEmptyFrame()
    state.frame.cameras = buildCameras(frame)
    state.frame.localization = updateLocalization(frame, poseDt)
    state.frame.mounts = {
      imu = state.frame.localization.imu.position,
      gnss = state.frame.localization.gnss.position
    }
    changed = true
  end

  if changed and state.frame then
    state.frame.enabled = true
    publish()
  end
end

return M
