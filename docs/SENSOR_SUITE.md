# RoadMind virtual sensor suite

This document describes the current Drive-compatible sensor layout and the exact boundary between simulated measurements and native sensor APIs.

## Layout overview

| System | Quantity | Placement | Coverage / range | Update |
|---|---:|---|---|---:|
| Forward LiDAR | 1 | Front roof / windshield area | 120° × 16°, 30–160 m configurable | 1–4 Hz |
| Front long-range radar | 1 | Front center | 28° horizontal, 160 m | 1–2 Hz |
| Corner radars | 4 | Front-left/right, rear-left/right | 58°, 50–65 m | 1–2 Hz |
| Front ultrasonic | 4 | Front bumper | 5 m, 1/3-ray beam | 1–2 Hz |
| Rear ultrasonic | 4 | Rear bumper | 5 m, 1/3-ray beam | 1–2 Hz |
| Corner ultrasonic | 4 | Vehicle corners | 5 m, 1/3-ray beam | 1–2 Hz |
| Cameras | 7 | Windshield, side mirrors, rear glass and rear corners | Individual frustums, 22–85 m visualization range | 2–5 Hz pose refresh |
| IMU | 1 | Near vehicle center | Acceleration signals + estimated yaw rate | 2–5 Hz |
| GNSS-like position | 1 | Roof | BeamNG world position and altitude | 2–5 Hz |
| Wheel odometry | 1 data source | Vehicle electrics | Wheel speed and odometer | 2–5 Hz sample |

## Vehicle detection zones

RoadMind keeps one Game Engine vehicle registry made from oriented bounding boxes (OBBs). The registry refreshes every 0.5 s. It excludes the player vehicle, invalid entries and entries reported as deleted/destroyed; a vehicle without valid OBB data is not registered.

At 0.25 s intervals, the latest calibrated sensor mounts define independent detection volumes:

- LiDAR uses its 120° horizontal and 16° vertical coverage plus configured range.
- Each radar uses its own horizontal/vertical coverage and range.
- Each ultrasonic unit uses its close-range coverage and configured minimum/maximum range.

A target's complete OBB is projected onto the sensor's forward/right/up axes. Broad-phase range filtering happens before the detailed bounds test. Multiple sensors observing the same vehicle produce one candidate record with a list of sensor zones.

Up to four candidate targets per detection cycle receive an occlusion raycast (maximum 16 additional queries/s). Candidates are classified as **visible**, **occluded** or **unverified**. Confirmed-visible targets feed the experimental obstacle guard. The 3D visualizer also draws non-occluded candidates before their visibility check completes: green OBB wireframes are confirmed visible, while amber/yellow wireframes are still unverified. Explicitly occluded candidates remain hidden.

This is simulator-assisted vehicle discovery based on the game's vehicle registry and OBB data. It is not a raw physical-style LiDAR or radar point return, and camera frustums do not participate in vehicle recognition. Confirmed-visible candidates feed the implemented session-scoped RoadMind tracker described below; stable identity and visibility behavior still need live-traffic validation.

## Vehicle tracks and motion estimates

The GE vehicle registry provides simulator-assisted OBB observations. When a candidate is inside one or more current sensor coverage zones and scene-raycast visibility is confirmed, RoadMind creates or updates a session-scoped track ID such as `RM-000001`. Repeated observations from several sensor zones are merged into one candidate and then one track.

Track fields include:

- Latest OBB-center position, dimensions and vehicle orientation.
- Sensor-zone sources that geometrically cover the target, plus a separate visibility status from the bounded occlusion checks.
- Last-observed simulation timestamp, data age, missed-scan count, confidence and status (`confirmed`, `coasting` or `stale`).
- Estimated velocity from timestamped position differences, plus estimated relative speed.
- Simulator-provided velocity stored separately from RoadMind's timestamp-derived estimate.
- A short predicted display position kept separate from the last observed position.

The tracker uses a 0.75 s recent-visibility cache and retains a missed target for at most 1.5 s, six missed detection cycles, or two complete registry snapshots without that vehicle. An explicit vehicle-destroyed event removes its track immediately. The active store is limited to 128 tracks.

All vehicle positions, dimensions, orientation and the simulator-provided velocity originate from BeamNG's vehicle OBB/object data. The estimated velocities are derived values, not radar Doppler measurements. This is a simulator-assisted tracking prototype, not camera recognition or physical sensor-only tracking.

## Unified world model

The GE extension exports `extensions.roadmind.getWorldModel()`. It returns schema version 2 as an independent snapshot in `beamng-world` coordinates and metres.

The model has separate `observations.lidar`, `observations.radar`, `observations.ultrasonic` and `observations.vehicleRegistry` arrays. Raycast observations keep their sensor origin, ray direction, range, timestamp, data age and source. Vehicle OBB observations are explicitly labelled simulator-assisted and are included only after a candidate is confirmed visible.

The model exposes one tracked target per RoadMind track ID, plus spatially clustered non-vehicle raycast obstacles. A target's observed `position` is distinct from its short-horizon `predictedPosition`; simulator velocity and timestamp-derived estimated velocity are separate fields. Each source has a freshness/status record and the model reports invalid, stale, conflicting, low-confidence, truncated and dropped data counters.

Vehicle targets are merged by RoadMind track ID. Non-vehicle returns are grouped into 0.75 m spatial cells; these clusters are not semantic object identities. Confidence values are heuristics, not calibrated probabilities. The complete schema and consumer guidance are in [WORLD_MODEL.md](WORLD_MODEL.md).

## Time-consistent LiDAR point display

Raw scan hits are timestamped in the GE simulation clock and retain their ray index and hit object ID when the engine supplies one. The raw array remains the measurement source for downstream logic.

The renderer keeps a separate point state:

- A recent tracked vehicle hit is stored in that vehicle's local OBB coordinates and follows its timestamp-derived track motion between scans.
- A target can be extrapolated for display by at most 0.5 s. This extrapolation is render-only, not a new sensor measurement.
- Static scene points stay in world coordinates. Known hit object IDs help avoid mixing adjacent surfaces; unknown static hits are continued only when their world point stays within 0.35 m on the same ray.
- A changed/missing return or lost object association causes the old marker to fade, not travel toward an unrelated new hit.
- The live renderer caps the active ray layer at 99 points and the fading layer at 198 markers. When inertia is enabled, same-ray returns use a critically damped spring so changed measurements glide instead of snapping.
- A separate session-only world-space point map deduplicates all LiDAR hits (including other-vehicle surfaces) into 0.35 m cells. Its user-configurable capacity is 500–10,000 points; old points are overwritten only after the selected capacity is reached. The accumulated map does not age out and remains drawn while the simulation is paused.

The optional accumulated scan map stores every accepted world-space LiDAR hit, including vehicle surfaces. Moving vehicles therefore leave a spatial scan trail in the map; it is not a live deforming mesh. The render layer separately smooths moving returns and vehicle OBB outlines. This visual layer performs no raycasts and never feeds its interpolated coordinates back into the raw LiDAR measurements used by sensor safety/planning.

## LiDAR

The forward LiDAR uses 9–33 horizontal samples and 1–3 vertical elevations (up to 99 ray directions; the High preset uses 51, while Economy uses 9). Each ray is evaluated by the Game Engine scene-raycast API, which returns the closest scene hit or no return. Valid hits are rendered as a beam from the front sensor origin and a point at the hit.

The scanner is intentionally forward-facing, not a 360° roof scanner.

## Radar

Five radar units approximate a modern front-and-corner layout:

- Front long-range unit for forward traffic and obstacles.
- Front-left and front-right corner units.
- Rear-left and rear-right corner units.

Each radar samples a small horizontal ray fan. Returns contain range and world hit position. The raw radar return does not create an independent radar track or measure Doppler. When the scene raycast exposes a vehicle object ID, the world model can associate that hit with the separate simulator-assisted RoadMind vehicle track. Relative speed is estimated from timestamped target positions and simulator player velocity, not from radar signal processing.

This is a kinematic raycast proxy. It does not simulate RF propagation, frequency-modulated chirps, antenna arrays, RCS, multipath or Doppler processing.

## Ultrasonic

Twelve short-range units cover the front, rear and corners. Each sensor uses three Game Engine scene-raycasts around its center direction and returns the nearest hit up to 5 m. This creates a compact cone approximation suitable for visual parking/close-obstacle feedback.

The model does not simulate acoustic wave propagation, surface absorption or material-dependent sensitivity.

## Cameras

Seven camera mounts show the intended view coverage:

1. Front narrow
2. Front wide
3. Left side
4. Right side
5. Rear wide
6. Rear left
7. Rear right

Each mount has its own 3D frustum using its position, orientation, horizontal FOV, vertical FOV and visualization distance.

In regular BeamNG.drive, this prototype does not read a camera image stream or perform image recognition. Native rendered camera/depth/semantic sensor access belongs to the BeamNG.tech sensor system.

## Localization and motion

### IMU-like motion data

- Acceleration: BeamNG electrics `accXSmooth`, `accYSmooth`, `accZSmooth`.
- Yaw rate: estimated from the change in vehicle forward direction over the sample interval.
- Mount: near the vehicle center.

The values are suitable for prototyping and visualization; they are not a calibrated six-axis IMU model with bias, noise, temperature drift or sensor fusion.

### GNSS-like position

- Position: `obj:getPosition()` in BeamNG world coordinates.
- Altitude: world Z coordinate.
- Mount: virtual roof antenna point.

There is no latitude/longitude conversion because the mod has no geodetic reference for the current map.

### Wheel odometry

Uses BeamNG vehicle electrics where available:

- `wheelspeed` — wheel-measured vehicle speed in m/s.
- `odometer` — vehicle odometer in metres.
- `airspeed` — vehicle speed relative to the world.

These are simulator vehicle signals, not a per-wheel encoder model.

## Visualization legend

- Cyan/blue: forward LiDAR and sensor origin.
- Green/yellow: radar units and returns.
- Turquoise/yellow: ultrasonic units and close returns.
- Blue: camera frustums.
- Magenta: IMU mount.
- Green: GNSS-like antenna position.

## BeamNG.tech native upgrade path

When a BeamNG.tech installation is available, the sensor adapter can be replaced or extended to create native camera, LiDAR, RADAR, ultrasonic, GPS and IMU sensors through `extensions.tech_sensors`. The Drive-compatible raycast backend remains useful for basic gameplay installs.
