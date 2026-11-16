# Roadmap

RoadMind aims to become a lightweight, modular autonomous-driving system for BeamNG.drive. Development is organized around a reliable perception foundation first, followed by road understanding, route planning and vehicle control.

**Current status:** virtual sensor-suite prototype. The sensor pipeline and in-game settings exist; calibration across vehicle classes, object tracking, autonomy and in-game performance validation remain future work.

## Development priorities

1. Finish vehicle geometry detection and sensor-mount calibration.
2. Build optimized vehicle detection zones and a unified target tracker.
3. Add time-consistent LiDAR visualization and sensor fusion.
4. Add road understanding, route planning and autonomous control.
5. Add parking, data recording and optional learning workflows.

The current priority is the perception foundation. Route following, parking and learning must consume stable, timestamped sensor/world-model data rather than depend directly on individual sensor implementations.

## Phase 1 — Forward LiDAR and world visualization

- [x] Forward virtual LiDAR with 120° horizontal field of view.
- [x] Configurable 9/26/51-ray scan density and scan frequency up to 4 Hz.
- [x] Static and dynamic scene-raycast returns.
- [x] In-world ray and hit-point visualization.
- [x] Controls binding for the sensor-suite visualization.
- [ ] Validate LiDAR pose and coverage on multiple vehicle sizes and configurations.
- [ ] Validate point placement, self-hit rejection and ground/overhead filtering.
- [ ] Add repeatable LiDAR test scenes and measurement exports.

**Acceptance criteria:** LiDAR returns correspond to scene-raycast measurements, the sensor is correctly positioned on supported vehicle classes, and visualization can be disabled independently of acquisition.

## Phase 2 — Multi-sensor automotive layout

- [x] Five virtual radar units: one front long-range unit and four corner units.
- [x] Twelve virtual ultrasonic sensors around the bumpers and corners.
- [x] Seven camera mounts with independent 3D fields of view.
- [x] IMU-like acceleration and estimated yaw-rate data.
- [x] GNSS-like world-position data.
- [x] Wheel-speed and odometer data from vehicle electrics where available.
- [x] Per-sensor enable/disable controls and scan-density profiles.
- [x] Bilingual in-game settings with saved performance profiles.
- [x] Independent sensor-overlay visibility controls.
- [ ] Validate radar coverage and return stability in moving traffic.
- [ ] Validate ultrasonic coverage against curbs, posts and parked vehicles.
- [ ] Add scan-health indicators, data timestamps and stale-data status.
- [ ] Measure sensor and visualization overhead on light and dense maps.

**Acceptance criteria:** all configured sensor groups report their own data and status; sensor models and simulation limitations are identified accurately; disabling the suite stops acquisition.

## Phase 3 — Vehicle geometry and automatic sensor calibration

**Goal:** place sensors relative to each vehicle's actual dimensions instead of relying on universal fixed offsets.

- [x] Read the vehicle's oriented bounding geometry once when attached; provide manual recalibration and reset/configuration callback entry points.
- [x] Determine vehicle-local center, forward/rear extents, left/right extents, height and coordinate basis from the OBB and live vehicle basis.
- [x] Define a versioned vehicle geometry profile containing dimensions, local bounds and calibration state.
- [x] Derive LiDAR, radar, ultrasonic and camera mount positions from the vehicle geometry plus centralized clearances.
- [x] Keep sensor layout positions in vehicle-local coordinates.
- [x] Cache the geometry profile and calculated sensor layout; do not recalculate vehicle dimensions on every scan.
- [x] Update world-space sensor transforms from the vehicle's current pose when a scan/pose frame is due.
- [ ] Validate that reset/configuration callback dispatch invalidates the cache on every supported BeamNG version; vehicle switches and manual recalibration already rebuild it.
- [x] Provide conservative fallback offsets when geometry data is unavailable or invalid.
- [ ] Allow per-vehicle manual mount overrides without changing the shared defaults.
- [x] Add optional debug visualization for vehicle bounds, local axes and all sensor mounts.
- [ ] Validate placement on small cars, sedans, vans, pickups, SUVs and large vehicles.

**Acceptance criteria:** sensor mounts remain outside the relevant body surfaces with sensible coverage; vehicle dimensions are not queried during every scan; switching vehicles produces a fresh calibration without stale mounts.

## Phase 4 — Optimized vehicle detection zones

**Goal:** identify candidate vehicles efficiently and expose them as targets to the sensor and tracking systems.

- [ ] Add a Game Engine vehicle registry/snapshot service using the available vehicle iteration APIs.
- [ ] Exclude the player's vehicle, destroyed vehicles and invalid/stale entries.
- [ ] Update the registry on a controlled schedule rather than independently scanning the complete vehicle list for every sensor.
- [ ] Define sensor-specific detection volumes using each calibrated mount, orientation, range and field of view.
- [ ] Use cheap distance and broad-phase bounds checks before detailed geometry tests.
- [ ] Test the target vehicle's bounding box against the detection volume; do not rely only on its center point.
- [ ] Apply sensor-specific horizontal/vertical coverage and minimum/maximum range.
- [ ] Check occlusion for candidate targets with a bounded number of scene-raycasts where visibility matters.
- [ ] Record which sensors can currently observe each target.
- [ ] Deduplicate detections from multiple sensors into one tracked target.
- [ ] Keep simulator-assisted vehicle detection explicitly separate from physical-style sensor returns.
- [ ] Continue using LiDAR scene raycasts for actual LiDAR point measurements and scene raycasts for non-vehicle obstacles.

**Acceptance criteria:** one nearby vehicle produces one target record even when several sensors detect it; objects outside sensor coverage or behind occluders are not reported as visible; the detection service does not run once per LiDAR ray or in the render loop.

## Phase 5 — Object tracking and motion estimates

**Goal:** maintain stable target identities and motion data between sensor updates.

- [ ] Introduce a RoadMind tracking ID that is separate from the transient BeamNG vehicle ID.
- [ ] Store target position, dimensions, orientation, estimated velocity, source sensors and last-observed timestamp.
- [ ] Associate new detections with existing tracks and prevent duplicate tracks.
- [ ] Estimate target speed and relative speed from timestamped position changes.
- [ ] Distinguish measured values from estimated or predicted values.
- [ ] Add confidence, data age and track status fields.
- [ ] Handle brief occlusion and missed scans with a bounded track-retention period.
- [ ] Expire tracks after despawn, invalidation or timeout.
- [ ] Reset or re-associate tracks safely when the player changes vehicle or the simulation resets.

**Acceptance criteria:** moving traffic keeps stable tracking identities across scans; stale targets expire predictably; timestamps and confidence allow consumers to reject old or uncertain data.

## Phase 6 — Time-consistent LiDAR visualization

**Goal:** remove visible point teleportation without changing the measurements used by autonomy.

- [ ] Keep raw LiDAR measurements separate from render-only point state.
- [ ] Timestamp every scan and point set.
- [ ] Associate measured points with tracked moving vehicles when the association is reliable.
- [ ] Store associated moving-object points in the target's local coordinate frame where possible.
- [ ] Keep static-scene points in world coordinates.
- [ ] Interpolate render positions between scans using elapsed time and bounded smoothing.
- [ ] Use target motion to update associated points between scans without treating the result as a new measurement.
- [ ] Fade or expire points that are no longer observed.
- [ ] Avoid connecting unrelated hits when a ray changes from one surface/object to another.
- [ ] Keep the point count and interpolation work bounded.
- [ ] Ensure the planner and controller receive raw/timestamped measurements or an explicitly defined world model, never render-only interpolated values.

**Acceptance criteria:** moving targets and their associated visual points move smoothly between scan updates; missing or replaced returns do not create persistent false geometry; visual smoothing cannot alter the sensor data consumed by autonomy.

## Phase 7 — Sensor fusion and unified world model

**Goal:** provide one consistent interface for objects and obstacles detected by different sensors.

- [ ] Define a common observation format with position, orientation/size when available, timestamp, source sensor, confidence and data age.
- [ ] Define a common target/world-model format for tracked vehicles and other detected objects.
- [ ] Merge observations of the same target without creating duplicate objects.
- [ ] Preserve source-specific measurements alongside fused estimates.
- [ ] Reconcile differences in update rate, coordinate frame and measurement age.
- [ ] Add quality checks for invalid, stale, conflicting or low-confidence observations.
- [ ] Keep non-vehicle obstacles from LiDAR and scene raycasts in the world model.
- [ ] Document which fields are measured, simulator-assisted, estimated or predicted.

**Acceptance criteria:** future planning modules can consume a single documented world-model interface without depending on LiDAR, radar or ultrasonic implementation details.

## Phase 8 — Road understanding and route input

- [ ] Read the map road graph and identify the road segment under the vehicle.
- [ ] Build a local road corridor and estimate road/lane direction where supported.
- [ ] Accept ordered route points and a destination.
- [ ] Track route progress and select the next target point.
- [ ] Handle intersections, road splits, route loss and invalid route points.
- [ ] Add a route preview in the 3D world.
- [ ] Expose route state and localization quality to the planner.

**Acceptance criteria:** RoadMind can follow a defined route at the planning level and identify when the route or localization state is invalid.

## Phase 9 — Local trajectory planning

- [ ] Generate a bounded set of candidate trajectories ahead of the vehicle.
- [ ] Evaluate candidates against the unified world model and road boundaries.
- [ ] Check the vehicle's full footprint, not only its center point.
- [ ] Score route progress, obstacle clearance, road-boundary distance, curvature and steering smoothness.
- [ ] Account for vehicle dimensions and turning constraints.
- [ ] Select a safe candidate and expose the reason for the decision.
- [ ] Request a controlled stop when no candidate is safe.
- [ ] Visualize the selected trajectory and rejected alternatives.
- [ ] Add limits on candidate count and planning frequency.

**Acceptance criteria:** the planner selects bounded, collision-checked trajectories and produces a safe stop request when the available space is insufficient.

## Phase 10 — Autonomous control and parking

- [ ] Implement trajectory tracking with a lightweight geometric controller such as Pure Pursuit or Stanley.
- [ ] Add speed control with acceleration and braking limits.
- [ ] Reduce target speed for tight curvature, poor visibility and low obstacle clearance.
- [ ] Add smooth start, stop, gear changes and reverse handling.
- [ ] Add an independent safety supervisor for stale sensors, invalid routes, blocked paths and controller faults.
- [ ] Add a dedicated autopilot enable/disable binding.
- [ ] Keep manual control available and make driver handover immediate.
- [ ] Add parking-space boundary and occupancy estimation.
- [ ] Plan low-speed parking maneuvers, including reverse and multi-point maneuvers where needed.
- [ ] Check the swept vehicle footprint during parking, not just the final vehicle position.
- [ ] Validate parking clearance using vehicle geometry and close-range sensor data.

**Acceptance criteria:** autonomy can be disengaged immediately; unsafe or stale inputs lead to a controlled stop; parking maneuvers account for the entire vehicle body throughout the maneuver.

## Phase 11 — Data recording and learning experiments

- [ ] Add optional recording of timestamped sensor measurements, tracked targets, vehicle state, route, control inputs and planner decisions.
- [ ] Define a versioned data format with vehicle/configuration and map/scenario metadata.
- [ ] Add bounded recording, file rotation and explicit start/stop controls.
- [ ] Keep recording local by default; export datasets only when requested.
- [ ] Add deterministic replay of recorded scenarios for debugging and evaluation.
- [ ] Establish measurable baseline metrics before introducing machine learning.
- [ ] Explore offline training and evaluation only after a reliable deterministic perception and control baseline exists.
- [ ] Keep model runtimes optional and disabled by default.

**Acceptance criteria:** recorded sessions can be replayed for comparison; datasets distinguish raw measurements from estimates and control actions; machine learning is not required for the base mod.

## Phase 12 — Validation, performance and robustness

- [ ] Test on multiple maps and vehicle classes.
- [ ] Test stationary obstacles, moving traffic, roadworks, narrow roads, intersections and parking areas.
- [ ] Test sensor occlusion, missed returns, vehicle damage, despawn and vehicle switching.
- [ ] Add repeatable test routes and scenario scripts.
- [ ] Record sensor, tracking, route, planner and controller telemetry.
- [ ] Profile with RoadMind disabled, scanning only, detection/tracking enabled and visualization enabled.
- [ ] Measure CPU time, frame time, FPS, Lua garbage-collection pressure and raycast counts.
- [ ] Test light and dense traffic on comparable hardware and settings.
- [ ] Add regression checks for stale targets, duplicate tracks, sensor mount positions and point-cloud continuity.
- [ ] Document supported BeamNG.drive versions and known limitations.
- [ ] Report measured results separately from calculated operation/query budgets.

**Acceptance criteria:** performance and behavior are repeatable on documented test scenarios; optimizations are supported by measurements; sensor or planner failures are visible and handled safely.

## Phase 13 — Optional BeamNG.tech sensors and advanced perception

- [ ] Add a native BeamNG.tech sensor adapter for rendered camera, LiDAR, RADAR, ultrasonic, GPS and IMU where supported.
- [ ] Keep the upper-level observation and world-model interfaces independent of the sensor backend.
- [ ] Add camera-based lane, sign and traffic-light perception when native image data is available.
- [ ] Explore advanced sensor fusion and machine learning after a deterministic baseline exists.
- [ ] Keep BeamNG.tech-only features and advanced model runtimes optional.
- [ ] Clearly identify capabilities unavailable in regular BeamNG.drive.

---

## Performance principles

- Never run scene raycasts in the render loop.
- Never recalculate vehicle dimensions or local sensor mount geometry on every scan.
- Cache vehicle-local geometry and sensor poses; update world transforms from the current vehicle pose only when needed.
- Use one shared vehicle registry and detection pass instead of repeating full vehicle-list scans per sensor or per ray.
- Apply cheap distance and bounds filtering before expensive visibility checks.
- Use scene raycasts for physical-style scene measurements and occlusion checks, with explicit per-frame/query budgets.
- Keep sensor acquisition, target tracking, world-model fusion, visualization, planning and control as separate layers.
- Run radar and ultrasonic updates at lower configurable rates than LiDAR where appropriate.
- Reuse tables and temporary objects where practical; avoid unnecessary serialization between Lua environments.
- Cache visualization data and keep point, target and trajectory counts bounded.
- Make visualization optional and never use render-only interpolation as autonomy input.
- Keep advanced model runtimes optional and disabled by default.
- Treat configured query counts as estimates, not measured FPS or CPU savings.
- Make performance claims only from repeatable measurements on defined maps, vehicles, traffic and hardware.

## Current simulation boundaries

- The base project targets regular BeamNG.drive.
- LiDAR uses Game Engine scene raycasts and returns geometric scene hits.
- Radar is a geometric raycast proxy, not an RF/FMCW simulation; Doppler, antenna behavior and physical radar signal processing are not implemented.
- Relative target speed may be estimated from timestamped observations; it must not be described as physical Doppler measurement.
- Ultrasonic uses short-range raycasts and does not simulate acoustic propagation or material-dependent reflections.
- Camera frustums are coverage visualizations; the regular BeamNG.drive mod does not provide the BeamNG.tech RGB camera stream.
- GNSS-like positioning reports BeamNG world coordinates, not geodetic latitude/longitude.
- IMU-like values are derived from simulator signals and are not calibrated hardware-grade outputs.
- Game Engine vehicle-list/bounds data can assist target discovery and tracking, but it is simulator-assisted information, not a LiDAR/camera measurement. Keep that distinction explicit in data and documentation.
- Autonomous route planning, trajectory control, parking and learning workflows are future roadmap items and are not considered implemented until validated in-game.
