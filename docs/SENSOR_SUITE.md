# RoadMind sensor suite

## 1. Design rule

The sensor configuration and roles defined in **ROADMAP_BRANCH_TESLA_MAP_STYLE.md** are the baseline for this branch. Keep the current layout unless a measured test demonstrates a specific need to change it.

Each sensor is responsible for a distinct type of evidence. Sensors may corroborate one another, but one sensor must not silently impersonate another.

## 2. Baseline inventory

| Sensor group | Count | Layout / coverage | Primary role |
|---|---:|---|---|
| LiDAR | 1 | Front-facing, 120° horizontal sector; up to 51 directions and 65 m configured maximum | Geometric surface sampling and front road/obstacle reconstruction |
| RADAR | 5 | One front long-range unit and four corner units | Independent target/range and relative-motion channel |
| Ultrasonic | 12 | Front, rear and corner/side near-field units | Very close surfaces, parking distance and near-field free space |
| Cameras | 7 | Front narrow/wide, left/right sides, rear wide and rear corners | Surrounding coverage; semantic perception with a real image backend |
| IMU-like | 1 | Ego motion source | Acceleration and angular-motion support |
| GNSS-like | 1 | BeamNG world position | Local map positioning; not necessarily geographic coordinates |
| Wheel odometry | 1 source | Vehicle wheel speed/odometer signals when exposed | Distance and ego-motion support |

The LiDAR remains forward-only. Do not turn it into a 360-degree scanner. Combined camera coverage is intended to surround the vehicle; it does not imply that the regular Drive backend receives camera pixels.

## 3. Sensor responsibilities

### 3.1 Forward LiDAR

**Purpose:** geometric measurement of surfaces in the forward sector: road surface where sampled, boundaries, barriers, walls and obstacle surfaces.

Rules:
- preserve the single front-facing unit and its forward sector;
- use ray hits as geometric returns, not semantic classes;
- attach the originating sensor, ray index, timestamp and hit-object identity when available;
- keep dynamic-vehicle returns associated with the relevant dynamic track rather than permanently inserting them into static geometry;
- do not assume rays pass through vehicles or other objects;
- validate self-hit rejection, road/ground filtering, sensor height, pitch and vehicle-specific mounting in game.

The Drive implementation uses Game Engine scene raycasts. It is a geometric proxy, not a physical laser propagation/intensity/reflectivity model. The implementation has configurable density and cadence; actual settings and caps must be read from the current runtime configuration rather than assumed from this document.

### 3.2 Cameras

The seven configured mounts provide front, side, rear and rear-corner coverage. Their intended system role is the primary semantic channel for visible vehicles, signs, traffic signals, road markings, lane boundaries and visible infrastructure.

Backend distinction is mandatory:

- **Regular BeamNG.drive:** current frustums and simulator-assisted vehicle OBB visibility/occlusion checks describe geometric coverage. They do not produce RGB frames and do not perform pixel-level semantic recognition.
- **BeamNG.tech or another real image source:** a future camera adapter may provide RGB, depth and semantic annotations where the installed backend supports them. Only detections derived from actual frames may use the native-camera provenance.

A vehicle can be reported as currently camera-visible only when it is within a valid camera frustum and the visibility test indicates an unobstructed line of sight. Occluded candidates must not be rendered as currently visible. Coasting predictions must remain visually and semantically distinct from current sightings.

Camera coverage is not a substitute for the front LiDAR or RADAR. It supplies a different type of evidence and should operate independently when the backend supports it.

### 3.3 RADAR

The baseline has five units: one front long-range sensor plus four corner sensors. Their intended job is to find and track dynamic targets, estimate range and relative movement, and provide information about closing traffic.

In regular Drive, the current implementation uses scene-raycast geometry and timestamped/simulator-assisted vehicle motion. This is not RF/FMCW simulation and does not produce physical Doppler measurements. A relative-speed estimate from tracked positions must be labelled as an estimate. Simulator velocity must remain a separate provenance field.

Ideal RADAR, if used in a future BeamNG.tech/debug adapter, is ground-truth/kinematic assistance and must never be displayed as a physical RADAR measurement.

### 3.4 Ultrasonic parking sensors

The twelve sensors cover the front, rear and corner/side near-field zones. Their responsibility is short-range surface distance and nearby free-space checks for parking and low-speed manoeuvring.

They can return a nearby vehicle surface if it lies within the sensor's beam, but they are not a second vehicle classification/tracking system. Keep their processing focused on close surfaces, curbs, posts, walls and bumper-to-object distance.

The current Drive implementation uses short-range scene raycasts. It does not simulate acoustic propagation, transducer response or material-dependent echoes.

### 3.5 IMU and ego motion

Use available vehicle motion signals to support orientation, motion compensation and scan alignment:
- vehicle pose and orientation;
- speed and velocity;
- acceleration and yaw-rate estimate;
- wheel speed and odometer distance;
- simulation timestamps.

Regular Drive values derived from electrics or pose differences are virtual/simulator-derived signals, not calibrated hardware-grade IMU measurements. If a native Advanced IMU backend is added, preserve its source and timestamp independently.

### 3.6 GNSS-like position and odometry

Use the vehicle's position in the BeamNG world/map coordinate frame to anchor the local model. The world frame is not a geodetic reference. Do not invent latitude/longitude for a level without a verified geographic transform.

Wheel-speed and odometer electrics support ego-motion estimation. They do not supply road geometry or discover map POIs.

## 4. Vehicle-specific mounting and calibration

Every configured sensor mount must be defined in vehicle-local coordinates and transformed using the active vehicle pose.

A vehicle/configuration profile should record:
- supported vehicle/configuration key;
- sensor inventory and whether each sensor is fitted/enabled;
- mount position and yaw/pitch/roll;
- horizontal/vertical field of view;
- usable minimum/maximum range;
- update frequency and measurement resolution;
- role and coverage zone;
- calibration source and confidence;
- known blind spots and self-occlusion.

Prefer verified vehicle-specific installation points. A shared body-class fallback may be used when exact data is unavailable, but it must be labelled as estimated. Do not imply that every vehicle factory-fits every sensor. Provide explicit override/calibration support for modified vehicles.

On vehicle/configuration change, discard old transforms, pending frames, tracks tied to the old vehicle and cached geometry. Recompute the active sensor layout once, then update world transforms at the due sensor/pose cadence rather than repeatedly measuring the body every ray or frame.

Consider self-occlusion from the body, glass, bumper covers, mirrors, roof attachments and trailers. A nominal frustum/range is not proof of effective visibility.

## 5. Observation and visibility rules

- Every sensor frame carries a simulation timestamp and source identity.
- Sensor-local coordinates are transformed through the vehicle frame into the common world/map frame.
- A delayed frame from a previous player vehicle must be rejected.
- A current detection requires range/FOV validity and any applicable occlusion test.
- A coasting track is not a fresh sighting.
- If the object leaves the sensor coverage or becomes occluded, its visibility state changes; it must not remain marked as currently visible.
- Data from simulator object registries must remain marked simulator-assisted even when a camera zone or sensor zone is used as a geometric filter.
- No sensor or source may create a semantic object class without evidence appropriate to that class.

## 6. Backend compatibility

### Regular BeamNG.drive

Current baseline:
- scene-raycast proxies for LiDAR, RADAR and ultrasonic;
- geometric camera frustums plus simulator-assisted target candidates/occlusion checks;
- vehicle state and electrics for ego-motion signals;
- vehicle OBB registry for candidate vehicle geometry;
- NavGraph and available level metadata for road context.

These are useful deterministic inputs, but they are not equivalent to the native physical-style sensors in BeamNG.tech.

### BeamNG.tech

A separate adapter may use native Camera, LiDAR, RADAR, Ideal RADAR, Ultrasonic, GPS, IMU/Advanced IMU and Roads Sensor APIs when supported by the installed version. Verify the exact API and data transfer path before implementation. Do not assume Tech-only shared-memory or sensor interfaces exist in ordinary Drive.

All backend adapters must publish into the same versioned observation interface while retaining backend and provenance metadata.

## 7. Validation checklist

- [ ] Sensor mount origins and orientations checked on multiple vehicle classes.
- [ ] Vehicle/configuration changes replace the sensor inventory and transforms correctly.
- [ ] LiDAR remains forward-facing and does not become a 360-degree scanner.
- [ ] Beam directions, range, self-hits and surface returns are checked in game.
- [ ] Radar targets and relative-motion estimates are separated from simulator velocity.
- [ ] Ultrasonic returns are restricted to the near field and do not become vehicle classification.
- [ ] Camera visibility respects FOV, range, self-occlusion and external occlusion.
- [ ] Camera visibility works independently of LiDAR where supported by the selected backend.
- [ ] Stale frames and old-vehicle frames are rejected.
- [ ] Sensor load and visualization cost are measured separately.
