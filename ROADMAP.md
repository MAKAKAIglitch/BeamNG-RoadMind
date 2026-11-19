# Roadmap

RoadMind aims to become a lightweight, modular autonomous-driving system for BeamNG.drive. Development is organized around a reliable perception foundation first, followed by road understanding, route planning and vehicle control.

**Current status:** RoadMind has a virtual sensor suite, simulator-assisted vehicle tracking, NavGraph route input, a bounded local trajectory planner, an experimental native Lua trajectory controller and an initial configurable ADAS layer. Existing ADAS prototypes include FCW, AEB, route-pilot ACC, blind-spot monitoring, parking-distance warnings and NavGraph-centreline lane-departure warnings. Sensor accuracy, controller tuning, ADAS thresholds, driver handover, visual alignment and performance still require repeatable in-game validation. The next roadmap extensions separate the ADAS interface, make vehicle perception camera-frustum-based around the whole vehicle (including rear cameras, without requiring LiDAR), add traffic-sign/signal awareness, trailer-aware sensor availability, vehicle-specific OEM-style sensor layouts and a lightweight perception display. Realistic mounting profiles must describe where each fitted sensor belongs on each vehicle, its orientation and effective field of view; generic placements remain clearly identified fallbacks. These are planned features, not implemented capabilities.

## Development priorities

1. Validate vehicle geometry, realistic per-vehicle sensor mounting profiles and trailer-aware coverage across vehicle classes.
2. Validate camera-frustum vehicle visibility around the full vehicle, occlusion, track identity and motion estimates in live traffic.
3. Validate LiDAR direction, point continuity, sensor freshness and world-model quality.
4. Complete Phase 8 route behavior on intersections, splits and multiple maps.
5. Validate and harden the native Lua trajectory controller, user takeover and independent safety supervisor.
6. Validate existing ADAS thresholds, brake blending, blind-spot tracking and pose-locked visualization in repeatable scenarios.
7. Separate the ADAS interface from core sensor settings before expanding assistance controls.
8. Add sign/signal awareness, lane-level perception and the perception display only on top of a measured, provenance-aware world model.

Phase 9 is implemented as a bounded local planner. Phase 10 now has its first native Vehicle Lua controller prototype; the next milestone is in-game validation and controller hardening, not adding more complexity before basic tracking and braking are proven.

## Phase 1 — Forward LiDAR and world visualization

- [x] Forward virtual LiDAR with 120° horizontal field of view.
- [x] Configurable 9/26/51-ray scan density and scan frequency up to 4 Hz.
- [x] Make vertical LiDAR layers predictable and useful: one layer points straight ahead horizontally; two layers add a downward ray while retaining the horizontal ray; three layers add an upward ray as well.
- [x] Keep horizontal-only as the lowest-cost default; do not make the one-layer mode point at the ground.
- [ ] Verify the generated directions and displayed beams on vehicles with different roof heights and pitch angles.
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

**Goal:** place sensors relative to each vehicle's actual dimensions and realistic equipment layout instead of relying on universal fixed offsets.

### Vehicle-specific OEM-style sensor placement

- [ ] Add a versioned per-vehicle sensor-layout profile, separate from the vehicle geometry profile. Record which sensor types are fitted, each mount's vehicle-local position, yaw/pitch/roll, field of view, range, role and calibration source/status.
- [ ] Prefer a vehicle-model/configuration-specific layout when one is available; support shared body-class defaults only as a fallback, and clearly label inferred or manually calibrated placements as estimates.
- [ ] Keep sensor inventory vehicle-specific: do not silently equip every car with roof LiDAR, surround-view cameras, corner radars or parking sensors when that equipment is not present in its selected profile. Allow optional virtual equipment only through an explicit configuration.
- [ ] Place the primary forward ADAS camera behind the upper windshield, typically near the rear-view mirror, using the actual vehicle profile rather than a universal roof/front offset.
- [ ] Place front/rear parking and surround-view cameras at the vehicle-specific factory locations: for example, in the grille/emblem, bumper, tailgate or at/above the license-plate area. A plate-area camera is a parking/rear-view camera where fitted, not a substitute for the main windshield ADAS camera.
- [ ] Place front radar behind the grille, emblem or front bumper cover according to the vehicle profile; place side/rear corner radars behind the appropriate bumper corners or rear-quarter areas where fitted.
- [ ] Support LiDAR at realistic vehicle-specific locations: roof/roofline units where the vehicle uses an external roof sensor, or integrated front/windshield/bumper locations where the design calls for concealed installation. Do not force every LiDAR onto the roof.
- [ ] Place side/surround cameras in mirror housings, fenders or other factory mounting points; place the rear camera near the tailgate or license-plate area and rear-corner cameras only where the vehicle actually has them.
- [ ] Place ultrasonic sensors flush with the appropriate front/rear bumper surfaces and corners, matching the count and locations in the vehicle's equipment profile.
- [ ] Keep GNSS antenna placement near the roof where appropriate, the IMU near the vehicle's structural center, and wheel-odometry sources tied to available wheel/electrics data rather than arbitrary exterior mounts.
- [ ] Apply each mount's actual orientation and calibrated camera/radar/LiDAR/ultrasonic coverage; mounting position alone must not imply that the sensor can see through body panels, glass, bumpers or other occluders.
- [ ] Account for self-occlusion from the vehicle body, attachments, roof racks and trailers when calculating effective sensor visibility and coverage.
- [ ] Provide a clear profile provenance state (vehicle-specific, manually overridden, body-class fallback or generic estimate) so the UI/debug overlay never presents an inferred mount as verified factory equipment.
- [ ] Preserve a conservative generic fallback for unsupported vehicles, but keep it visibly marked as estimated and avoid presenting it as an exact real-world installation.
- [ ] Validate that changing vehicle/configuration switches the active sensor inventory and mount profile without stale sensors or transforms.

- [x] Read the vehicle's oriented bounding geometry once when attached; provide manual recalibration and reset/configuration callback entry points.
- [x] Determine vehicle-local center, forward/rear extents, left/right extents, height and coordinate basis from the OBB and live vehicle basis.
- [x] Define a versioned vehicle geometry profile containing dimensions, local bounds and calibration state.
- [x] Derive LiDAR, radar, ultrasonic and camera mount positions from the vehicle geometry plus centralized clearances.
- [ ] Detect an attached trailer or other towed vehicle using verified coupler/attachment state where available, with a bounded fallback if the active vehicle API does not expose it reliably.
- [ ] Recalculate the effective rear sensor coverage when a trailer is attached; identify which rear ultrasonic/radar/camera mounts are physically blocked by the trailer's actual bounds.
- [ ] Suppress or mark unavailable only rear sensors whose field of view is obstructed by the trailer; do not blindly disable front or side sensors, or rear sensors that remain genuinely unobstructed.
- [ ] Restore the normal rear sensor layout after detachment without requiring a game restart.
- [x] Keep sensor layout positions in vehicle-local coordinates.
- [x] Cache the geometry profile and calculated sensor layout; do not recalculate vehicle dimensions on every scan.
- [x] Update world-space sensor transforms from the vehicle's current pose when a scan/pose frame is due.
- [ ] Validate that reset/configuration callback dispatch invalidates the cache on every supported BeamNG version; vehicle switches and manual recalibration already rebuild it.
- [x] Provide conservative fallback offsets when geometry data is unavailable or invalid.
- [ ] Allow per-vehicle manual mount overrides without changing the shared defaults.
- [x] Add optional debug visualization for vehicle bounds, local axes and all sensor mounts.
- [ ] Validate placement on small cars, sedans, vans, pickups, SUVs and large vehicles.

**Acceptance criteria:** sensor mounts remain outside the relevant body surfaces with sensible coverage; supported vehicles use their own fitted-sensor inventory and realistic model-specific mounting points/orientations; inferred fallback positions are identified as estimates; bodywork and attachments correctly limit visibility; vehicle dimensions are not queried during every scan; switching vehicles produces a fresh calibration without stale mounts.

## Phase 4 — Optimized vehicle detection zones

**Goal:** identify candidate vehicles efficiently and expose them as targets to the sensor and tracking systems.

- [x] Add a Game Engine vehicle registry/snapshot service using `getAllVehicles()`, with an object-pool fallback.
- [x] Exclude the player's vehicle, invalid OBBs and entries reported as deleted/destroyed; each new snapshot replaces stale entries.
- [x] Refresh the registry on a controlled 0.5 s schedule instead of scanning the list from each sensor.
- [x] Define sensor-specific detection volumes from calibrated mount position, orientation, range and horizontal/vertical FOV.
- [x] Add camera-specific vehicle observations for every configured camera: windshield, side/mirror, rear and rear-corner views; a vehicle behind the player must be eligible for detection by the rear cameras.
- [x] Require a target vehicle OBB to intersect at least one active camera's range and frustum before publishing a camera observation; merge overlapping camera observations into one tracked vehicle with multiple camera sources.
- [x] Make camera-based vehicle detection independent of LiDAR being enabled. LiDAR remains an obstacle/geometry sensor and is not required to discover or track other vehicles.
- [x] Use the vehicle registry only as an efficient candidate broad phase; camera visibility must still pass per-camera frustum and line-of-sight/occlusion checks before a target is marked visible.
- [x] Cache visibility results and spread bounded occlusion checks over time; never cast a ray for every vehicle-camera pair every render frame.
- [x] Keep camera-frustum/registry observations explicitly labelled simulator-assisted, not RGB image recognition; retain optional radar observations as a separate source.
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
- [ ] Validate multi-camera handoff (front to side to rear), overlapping fields of view, traffic approaching from behind, short occlusions and vehicles passing alongside.
- [ ] Preserve source-camera identity and visibility age on each track so the UI and ADAS can report which camera currently sees a vehicle.

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
- [x] Add configurable driver-intervention takeover for route pilot/autopilot: when the player deliberately applies steering, throttle or brake, transfer control back to the player immediately.
- [x] Enable manual takeover by default, while allowing the user to disable it explicitly.
- [x] Detect actual input/action values and analog axes through BeamNG's input system so wheel, gamepad, keyboard and user-remapped bindings work; do not hard-code key names or assume default bindings.
- [x] Ignore camera look/orbit input when deciding whether the driver has taken control; camera movement is not vehicle-control input.
- [ ] Define unambiguous controller ownership and re-engagement rules so RoadMind never fights the player's steering/throttle/brake inputs.
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

### Phase 10 implementation detail — native Lua trajectory control

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

## Phase 14 — configurable ADAS and attached-vehicle visualization

- [x] Add a persistent ADAS master switch and separate FCW, AEB, ACC, BSM, parking-distance and LDW toggles.
- [x] Add an ADAS status panel with sensor-freshness state and active warnings in Russian and English.
- [x] Add forward-obstacle FCW and AEB logic using currently enabled, fresh sensor observations.
- [x] Blend AEB brake input during manual driving and preserve route-controller actuator ownership during autonomous route control.
- [x] Add route-pilot-only adaptive cruise speed targeting from a tracked lead vehicle.
- [x] Add blind-spot alerts from simulator-assisted vehicle tracks and ultrasonic parking-distance display.
- [x] Add NavGraph-centreline lane-departure warning (not camera/painted-line recognition).
- [x] Publish vehicle pose and pose-only mount updates separately from sensor raycast returns.
- [x] Re-anchor sensor origins, camera frustums, calibration geometry and detection zones to the live vehicle pose.
- [x] Use tracked/predicted moving-vehicle poses for traffic OBB visualization instead of independent world-space smoothing.
- [ ] Validate each ADAS feature with controlled in-game scenarios and check false-positive/false-negative behavior.
- [ ] Verify AEB driver-brake blending and manual takeover across vehicle classes and input devices.
- [ ] Tune thresholds from repeatable braking-distance and traffic-gap measurements.
- [ ] Add trailer-aware rear-assistance behavior: disable or mark unavailable rear parking/ultrasonic/camera functions only when the attached trailer physically blocks their view; keep unobstructed side/rear coverage active.
- [ ] Keep trailer-camera support as an optional future extension, not a requirement for the first trailer-aware release.
- [ ] Add coordinated status/configuration for vehicle stability aids (ABS, ESC, traction control/TCS) only where the active vehicle exposes a reliable control or state interface; report unsupported systems instead of showing nonfunctional universal switches.
- [x] Add a dedicated pause-menu entry named **RoadMind ADAS** (subtitle: Advanced Driver Assistance Systems), separate from the existing RoadMind settings entry.
- [x] Move ADAS/autopilot controls and their status/warnings into the dedicated RoadMind ADAS page; keep core sensor layout, calibration, scan density and performance controls in RoadMind settings.
- [ ] Include clear independent controls for FCW, AEB, ACC, blind-spot/rear-cross-traffic alerts, parking assistance, LDW/LKA, lane centering, stability-aid integration and autopilot takeover behavior as each feature becomes available.
- [x] Keep the main RoadMind settings page focused on sensors, acquisition, calibration, road/route data, diagnostics and performance.
- [ ] Add manual-driving ACC only after reliable lead-vehicle tracking and speed-control arbitration are validated.

**Status:** code-level prototype committed; ADAS and visualization behavior have not yet been verified in a live BeamNG.drive session. Assistance thresholds are heuristics, not a collision guarantee.


## Phase 15 — Surround-view camera vehicle perception

**Goal:** make RoadMind detect and track surrounding traffic through the vehicle's camera layout, including vehicles alongside and behind the player, without requiring LiDAR.

- [ ] Treat each calibrated camera as an independent field-of-view sensor with its own origin, orientation, horizontal/vertical FOV, range, freshness and availability.
- [ ] Cover forward, front-wide, left/right side, rear-wide and rear-corner zones; validate gaps and overlaps against each vehicle's calibrated geometry.
- [ ] Discover potential vehicles through the shared BeamNG vehicle registry, then test the complete target OBB against the relevant camera frustum.
- [ ] Confirm that the line between the camera and the visible portion of the target is not blocked by terrain, buildings, barriers, other vehicles or the player's own body; occluded vehicles must not be reported as currently visible.
- [ ] Use a bounded, cached and staggered line-of-sight budget. Reuse recent valid visibility results for a short configured interval, mark them stale when they expire and avoid raycasts in the render loop.
- [ ] Publish one observation per camera/target pair and merge observations from several cameras into one stable RoadMind vehicle track with a list of currently contributing cameras.
- [ ] Preserve tracking through camera handoffs and short occlusions with explicit confirmed/coasting/stale states; do not present a coasting prediction as a fresh camera sighting.
- [ ] Keep camera-derived vehicle discovery operational with LiDAR disabled. Use LiDAR for physical-style obstacle geometry and point returns, not as a prerequisite for seeing traffic.
- [ ] Optionally fuse radar range/relative-motion information with camera vehicle tracks when radar is enabled, while preserving each source's provenance and data age.
- [ ] Add repeatable tests for oncoming traffic, cars in front, cars passing on either side, vehicles following from behind, dense traffic, blind spots, road crests, walls and partial/full occlusion.
- [ ] Add per-camera health/coverage diagnostics and counters for candidates, visible, occluded, unverified and stale targets.

**Simulation boundary:** in regular BeamNG.drive, this phase initially uses game vehicle data as a candidate source plus camera-frustum geometry and scene visibility checks. It is a simulator-assisted camera-visibility model, not image/AI recognition from an RGB frame. It must never see through solid objects. A native rendered-image perception backend remains an optional BeamNG.tech path.

**Acceptance criteria:** a vehicle is reported as camera-visible only while at least one active camera covers it and its view is unobstructed; rear and side traffic are tracked without LiDAR; overlapping camera observations do not create duplicate vehicles; query work stays bounded and outside rendering.

## Phase 16 — Traffic signs, road signals and speed awareness

**Goal:** give ADAS an understanding of posted road rules and traffic-control devices using reliable map/game information first, with image recognition as an optional backend.

- [ ] Inventory BeamNG level data, road-signal definitions, object classes/names and available runtime APIs for traffic signs and signal states before choosing an implementation.
- [ ] Build a versioned sign/signal classification layer with explicit source and confidence. Prefer authoritative level or simulator state when available; do not infer a sign from an unrelated nearby object.
- [ ] Detect and locate speed-limit signs, stop signs, parking/no-parking signs, no-entry signs, priority signs and directional/turn-ahead warning signs.
- [ ] Detect traffic signal heads in the active camera field of view and expose the actual supported state (red, yellow/amber, green, unknown); verify state transitions and direction/lane association on several maps.
- [ ] Apply the same camera range, frustum and occlusion rules to signs and traffic lights; objects behind walls or outside the camera's view must not be displayed as seen.
- [ ] Associate a detected sign or signal with the relevant road/route direction and avoid applying a sign from a parallel road or the opposite carriageway.
- [ ] Add an ADAS option to respect posted speed limits and use a recognized speed limit as a configurable target-speed cap.
- [ ] Add an explicit option to ignore speed-limit information for speed control while still allowing the sign to be shown in the perception display; do not let this option silently disable hazard warnings or emergency braking.
- [ ] Add stop-line/red-light approach warnings and later rule-aware stopping only after signal state, lane association and stopping-distance behavior are validated.
- [ ] Add sign/signal labels to the ADAS status view and the future perception display, using readable text labels rather than relying only on colors or emoji.
- [ ] Validate sign orientation, duplicates, temporary roadworks signs, occlusion, distance, state changes and map-to-map differences.

**Implementation path:** in regular BeamNG.drive, use dependable level/simulator metadata and object state when exposed. Pixel-level sign/light recognition requires an actual image source and belongs to the optional BeamNG.tech camera backend. If a map does not expose reliable semantics, report unknown rather than inventing a detection.

**Acceptance criteria:** displayed signs and signals have a defensible source, correct direction/road association and visibility state; speed-limit compliance can be enabled or disabled independently; unsupported or ambiguous cases remain unknown.

## Phase 17 — Lane-level road understanding and lane assistance

**Goal:** progress from road-centreline following to lane-aware assistance while preserving a reliable map-based fallback.

- [ ] Investigate which regular BeamNG.drive map/road structures expose road boundaries, lane count, lane direction, markings and lane connectivity at runtime.
- [ ] Extend NavGraph localization with lane/road attributes where the map actually provides them; keep road-centreline estimates distinct from painted lane-boundary detections.
- [ ] Add camera-frustum lane-marking observations only when a suitable image or semantic source exists; retain a documented simulator-assisted/map-geometry fallback for regular Drive.
- [ ] Estimate the current lane, lane centre, left/right boundaries, lane width, confidence and source freshness where data supports it.
- [ ] Track lane geometry over time and across camera transitions; reject discontinuous or implausible lane changes.
- [ ] Add lane-departure warning (LDW) based on lane boundaries when available, retaining the current NavGraph-centreline warning as a separately labelled fallback.
- [ ] Add lane-keeping/lane-centering assistance (LKA/LCC) with configurable activation, steering limits, speed range and driver override.
- [ ] Use GNSS-like world position and NavGraph route context to identify the correct road and direction, not as a substitute for lane markings.
- [ ] Validate intersections, merges, splits, multi-lane roads, narrow roads, junctions, worn/missing markings, temporary markings and roads without lane metadata.

**Acceptance criteria:** lane-level functions state whether they use map geometry, camera image perception or a lower-confidence road-centreline fallback; the controller does not claim lane centering when only a road centreline is known.

## Phase 18 — RoadMind perception display

**Goal:** provide a lightweight in-game mini-application that visualizes the interpreted environment, similar in purpose to a modern vehicle perception display, without duplicating sensor simulation or rendering a second heavy scene.

- [x] Add a separate compact **RoadMind Perception** application/window with a dark, restrained, high-contrast interface.
- [x] Provide a rear-follow-style 2D projection and a switchable top-down/overhead road view. This is a perception-map projection, not a live game camera.
- [x] Draw stylized vehicle markers, the NavGraph road-centreline corridor, the active route and surrounding detected targets in a flat SVG view; lane boundaries are not fabricated.
- [x] Visualize tracked cars as vehicle-shaped OBB markers and raycast obstacle clusters as generic obstacle points; unsupported object classes are not invented.
- [ ] Show traffic signs and traffic lights with concise text/state labels (for example, speed limit, STOP, parking restriction, turn warning, SIGNAL: RED) and their current camera/visibility status.
- [ ] Show lane lines only when measured or supported by map geometry; render an explicit unknown/unavailable state when lane data is missing.
- [ ] Display object distance, relative speed when available, track confidence/age and source sensor/camera on selection or in a compact details strip.
- [ ] Use textual statuses such as VISIBLE, OCCLUDED, PREDICTED, STALE and UNKNOWN alongside restrained visual accents; do not make meaning depend on color or emoji alone.
- [x] Reuse the latest bounded World Model and road/route snapshots. The application performs no scene raycasts, vehicle-registry scans, physics queries or perception work in its UI/render loop.
- [x] Throttle UI polling, reuse compact snapshots and cap targets, obstacles and displayed road points.
- [x] Avoid a redundant second raw-LiDAR/obstacle debug layer: show the fused interpreted scene and object classes, while the existing sensor visualization remains the detailed diagnostic view.
- [ ] Add controls for view mode, labels, object categories and display scale; keep the display optional and independently disableable.
- [ ] Validate readability, object motion, camera handoff, stale-data presentation and UI overhead in dense traffic and on low-end hardware.

**Acceptance criteria:** the mini-app shows only data already present in RoadMind's world/road model, distinguishes measurements from predictions and unknowns, performs no acquisition work itself and remains inexpensive when hidden or disabled.


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
- Camera frustums are coverage visualizations; the regular BeamNG.drive mod does not provide the BeamNG.tech RGB camera stream. The planned Drive-compatible vehicle-perception mode is a simulator-assisted approximation using vehicle candidates, camera frustums and occlusion checks; it is not RGB/image recognition and must not require LiDAR.
- Traffic-sign and signal awareness in regular Drive must use verified level/simulator metadata or state where available. Pixel-level sign, lane-marking and signal recognition requires a real image/semantic source such as the optional BeamNG.tech sensor backend.
- A NavGraph road centreline is not a painted lane boundary. Lane-keeping features must disclose which source (map geometry, image perception or centreline fallback) supports the result.
- GNSS-like positioning reports BeamNG world coordinates, not geodetic latitude/longitude.
- IMU-like values are derived from simulator signals and are not calibrated hardware-grade outputs.
- Game Engine vehicle-list/bounds data can assist target discovery and tracking, but it is simulator-assisted information, not a LiDAR/camera measurement. Keep that distinction explicit in data and documentation.
- Route planning, a first native trajectory controller and initial ADAS features exist as experimental code; they are not considered validated until repeatable in-game tests pass. Manual ACC, full lane-level perception, traffic-sign/signal awareness, trailer-aware rear sensor gating and the RoadMind Perception display remain roadmap work.
