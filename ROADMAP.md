# Roadmap

RoadMind aims to become a lightweight, modular autonomous-driving system for BeamNG.drive. Development is organized around a reliable perception foundation first, followed by road understanding, route planning and vehicle control.

**Current status:** virtual sensor suite, simulator-assisted vehicle tracking, NavGraph route input, bounded local trajectory planning and a first native Lua trajectory controller. The controller now owns steering/throttle/brake inputs and no longer relies on the built-in Vehicle AI for route following. Sensor accuracy, steering polarity, vehicle-class coverage, controller tuning, traffic interaction, route behavior and performance still require repeatable in-game validation. Parking and robust intersection/traffic negotiation are not implemented.

## Development priorities

1. Validate vehicle geometry and sensor mounting across vehicle classes.
2. Validate detection zones, visibility checks, track identity and motion estimates in live traffic.
3. Validate LiDAR point continuity, sensor freshness and world-model quality.
4. Complete Phase 8 route behavior on intersections, splits and multiple maps.
5. Validate and harden the native Lua trajectory controller and independent safety supervisor.
6. Add parking, recording and optional learning only after deterministic route control is validated.

Phase 9 is implemented as a bounded local planner. Phase 10 now has its first native Vehicle Lua controller prototype; the next milestone is in-game validation and controller hardening, not adding more complexity before basic tracking and braking are proven.

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

- [x] Add a Game Engine vehicle registry/snapshot service using `getAllVehicles()`, with an object-pool fallback.
- [x] Exclude the player's vehicle, invalid OBBs and entries reported as deleted/destroyed; each new snapshot replaces stale entries.
- [x] Refresh the registry on a controlled 0.5 s schedule instead of scanning the list from each sensor.
- [x] Define sensor-specific detection volumes from calibrated mount position, orientation, range and horizontal/vertical FOV.
- [x] Use a distance broad phase and projected OBB radii before detailed sensor-volume checks.
- [x] Test the target vehicle's complete OBB against each detection volume, not only its center.
- [x] Apply per-sensor horizontal/vertical coverage and minimum/maximum range.
- [x] Run at most four occlusion raycasts per 0.25 s detection cycle; classify candidates as visible, occluded or unverified.
- [x] Record all sensor zones that currently contain each vehicle OBB.
- [x] Deduplicate multiple sensor-zone matches into one candidate record per vehicle.
- [x] Label these results as simulator-assisted vehicle-registry observations, separate from physical-style sensor returns.
- [x] Keep LiDAR scene raycasts as the source of actual LiDAR point measurements and scene raycasts for non-vehicle obstacles.
- [ ] Validate registry completeness, OBB overlap and occlusion behavior in live traffic scenarios.

**Acceptance criteria:** one nearby vehicle produces one target record even when several sensors detect it; objects outside sensor coverage or behind occluders are not reported as visible; the detection service does not run once per LiDAR ray or in the render loop.

## Phase 5 — Object tracking and motion estimates

**Goal:** maintain stable target identities and motion data between sensor updates.

- [x] Introduce session-scoped RoadMind tracking IDs, separate from transient BeamNG vehicle IDs.
- [x] Store target OBB-center position, dimensions, orientation, sensor-zone sources and last-observed simulation timestamp.
- [x] Associate repeated observations by source vehicle ID and merge multiple sensor-zone matches into one track.
- [x] Estimate target speed from successive timestamped positions and derive relative speed separately from simulator player velocity.
- [x] Distinguish simulator-provided velocity, timestamp-derived estimates and short-horizon predicted display positions.
- [x] Add confidence, data age, missed-scan count and confirmed/coasting/stale track states.
- [x] Handle brief visibility loss with a 0.75 s visibility cache and bounded track coasting.
- [x] Expire tracks after 1.5 s, more than six missed detection cycles, two complete registry snapshots without the source vehicle, or explicit despawn.
- [x] Clear or re-associate tracking state on player-vehicle switch/reset and scanner shutdown.
- [ ] Validate ID stability, velocity estimates, temporary occlusion and despawn expiry in moving-traffic scenarios.

**Acceptance criteria:** moving traffic keeps stable tracking identities across scans; stale targets expire predictably; timestamps and confidence allow consumers to reject old or uncertain data.

## Phase 6 — Time-consistent LiDAR visualization

**Goal:** remove visible point teleportation without changing the measurements used by autonomy.

- [x] Keep raw LiDAR measurements separate from render-only point state.
- [x] Timestamp every scan and point set in the GE simulation clock.
- [x] Associate measured points with recent tracked moving vehicles by hit object ID, with an unambiguous OBB spatial fallback.
- [x] Store associated moving-object points in the target's vehicle-local OBB frame.
- [x] Keep static-scene points in world coordinates.
- [x] Smooth render positions between scans using elapsed GE time and a bounded critically damped spring; make visualization inertia independently switchable.
- [x] Add a bounded session-only world-space LiDAR map with configurable 500–10,000 point capacity, vehicle returns included and pause-safe display.
- [x] Use timestamp-derived target motion to move associated render points between scans, with a maximum 0.5 s prediction horizon and no new sensor measurement.
- [x] Fade missed/replaced points and expire render points whose source scan is older than 2.25 s.
- [x] Keep each ray's previous hit separate when the hit object ID, track ID or world-space location changes; do not interpolate between unrelated returns.
- [x] Bound active render points to 99 and fading points to 198.
- [x] Keep planner/safety inputs on raw timestamped `latestData.lidar.rays`; render-only positions are never written back to sensor measurements.
- [ ] Validate moving-object point continuity, surface switching and missed-return fading in live BeamNG traffic and repeatable scenes.

**Acceptance criteria:** moving targets and their associated visual points move smoothly between scan updates; missing or replaced returns do not create persistent false geometry; visual smoothing cannot alter the sensor data consumed by autonomy.

## Phase 7 — Sensor fusion and unified world model

**Goal:** provide one consistent interface for objects and obstacles detected by different sensors.

- [x] Define a common observation format with position, orientation/size when available, timestamp, source sensor, confidence and data age.
- [x] Define a common target/world-model format for tracked vehicles and other detected objects.
- [x] Merge observations of the same vehicle by RoadMind track ID; aggregate non-vehicle ray hits into bounded spatial obstacle clusters.
- [x] Preserve source-specific LiDAR, radar, ultrasonic and vehicle-registry observations alongside tracked/fused target records.
- [x] Reconcile sensor update rates, coordinate frames and measurement ages with GE simulation timestamps and per-source freshness.
- [x] Add quality checks and counters for invalid, future, stale, conflicting, low-confidence, truncated and dropped data.
- [x] Keep non-vehicle obstacles from LiDAR, radar and ultrasonic scene raycasts in the world model.
- [x] Document measured scene-raycast data, simulator-assisted OBB observations, estimated motion and predicted positions in `docs/WORLD_MODEL.md`.

**Acceptance criteria:** future planning modules can consume a single documented world-model interface without depending on LiDAR, radar or ultrasonic implementation details.

## Phase 8 — Road understanding and route input

- [x] Read the map road graph and identify the road segment under the vehicle.
- [x] Build a short local road-centerline corridor and estimate segment direction/alignment where NavGraph data supports it. This is not a lane-boundary estimate.
- [x] Accept 1–12 ordered world-coordinate route points (X/Y/Z) and a destination through the RoadMind API and settings UI.
- [x] Build a connected NavGraph path through the supplied points, validate node data/links, and bound the total route to 256 nodes.
- [x] Track progress along the active path and expose the next-progress index and distance to the path.
- [ ] Handle complex intersections, road splits and complete recovery across different maps. Finite coordinate validation, connected-path checks and a route-deviation stop/replan are implemented; live behavior remains unverified.
- [x] Preview the generated route in the 3D world while the route pilot is active.
- [x] Expose road-localization quality, distance to the current segment, lateral offset, heading alignment and corridor size in status data.

Code-level implementation is present. Route choice, one-way behavior, map-to-map compatibility and progress tracking still require in-game validation.

**Acceptance criteria:** RoadMind can follow a defined route at the planning level and identify when the route or localization state is invalid.

## Phase 9 — Local trajectory planning

- [x] Generate five bounded lateral-offset candidate trajectories up to 25 m ahead, sampled at 2.5 m intervals.
- [x] Evaluate candidates against fresh schema-v2 obstacle clusters and confirmed/coasting vehicle targets.
- [x] Check the calibrated full vehicle footprint along each sampled trajectory, including projected target-vehicle OBB extents.
- [x] Score route progress, obstacle clearance, NavGraph corridor margin, curvature and steering smoothness.
- [x] Account for cached vehicle length/width/height and a conservative length-based turning-radius estimate.
- [x] Select the highest-scoring safe candidate and expose candidate counts, selected offset, clearance and rejection reason.
- [x] Stop the built-in route AI when data/road geometry is insufficient or no candidate passes the safety checks.
- [x] Visualize the selected candidate in green and rejected alternatives in red/blue.
- [x] Limit planning to five candidates, thirteen samples per candidate and a maximum 5 Hz update rate.

**Implementation status:** the code-level planner is present. The NavGraph road band uses a conservative 3.5 m half-width heuristic because the current Drive-mode road API does not provide verified lane boundaries. This is not a formal collision guarantee, and the selected path is preview-only until Phase 10 adds trajectory tracking and low-level control.

**Acceptance criteria:** the planner selects bounded, collision-checked trajectories and produces a safe stop request when the available space is insufficient. Live traffic, curb/road-edge behavior and tight intersections still require repeatable in-game validation.

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
- [x] Add an optional FPS-feedback sensor-quality controller with a target FPS slider and a visible minimum-quality limit state.
- [x] Spread due Vehicle Lua sensor/pose jobs over graphics frames and adapt scan and traffic update cadence to vehicle speed.
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

## Phase 10 — native Lua trajectory control

- [x] Add a Vehicle Lua controller that consumes a bounded, timestamped local trajectory from the GE planner.
- [x] Track the selected trajectory with a Pure Pursuit-style lateral controller.
- [x] Add bounded PI speed control with steering-dependent speed reduction and a conservative 8 m/s cap.
- [x] Apply steering, throttle and brake through BeamNG's Vehicle Lua input events.
- [x] Add a latched full-brake supervisor response for stale trajectory feeds, missing vehicle state and lost route tracking.
- [x] Keep controller updates at 20 Hz and refresh trajectory commands only when the 5 Hz planner publishes a new candidate.
- [ ] Validate steering direction, steering limits and speed response on several stock vehicle classes.
- [ ] Tune wheelbase/steering geometry per vehicle and verify braking distances under different tire/road conditions.
- [ ] Validate manual takeover, pause/resume, vehicle reset, route completion and all supervisor stop/restart paths.
- [ ] Validate intersections, traffic, lane changes and obstacle avoidance in repeatable scenarios.

**Status:** code-level prototype committed; not yet verified in a live BeamNG.drive session. The planner's NavGraph corridor remains a heuristic and the safety supervisor is not a collision guarantee.

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
- Route planning and a first native trajectory controller exist as experimental code; they are not considered validated until repeatable in-game tests pass. Parking and learning remain future roadmap items.
