# Performance notes

RoadMind is designed to minimize its load on BeamNG.drive.

## Configurable profiles

The settings page is available at **ESC → Mods → RoadMind** in BeamNG.drive 0.39+.

| Profile | LiDAR | Radar | Ultrasonic | Estimated queries/s |
|---|---|---|---|---:|
| Economy (default) | 9 rays × 1 Hz | Disabled | Disabled | **9** |
| Balanced | 26 rays × 2 Hz | 10 rays × 1 Hz | 12 rays × 1 Hz | **74** |
| High | 51 rays × 4 Hz | 17 rays × 2 Hz | 36 rays × 2 Hz | **310** |

The default is Economy: only LiDAR scans (9 queries/s). Radar and ultrasonic are disabled until enabled manually or by choosing Balanced/High. Only the LiDAR overlay is drawn; camera frustums and localization markers also start hidden. The page also allows each sensor group to be disabled, LiDAR range to be changed (30–160 m), and individual 3D overlays to be shown. Hiding an overlay does not disable the underlying scan; disable the sensor itself to reduce raycast work.

These figures are calculated from configured ray counts and frequencies. They are not measured FPS or CPU percentages.

## Vehicle geometry cost

Vehicle dimensions are sampled once from the active vehicle's oriented bounding box during attach or explicit recalibration. The resulting local geometry and sensor layout are cached in Vehicle Lua. Each due sensor frame transforms the cached local mounts into world space using the current vehicle basis. The OBB is not queried for every ray or on the render loop.

The bounds/axes/mount overlay is off by default. When enabled, it adds one pose/debug update at the configured pose rate and a bounded set of debug lines/points; it does not add scene raycasts.

## Vehicle registry and detection budget

- The shared GE vehicle OBB registry refreshes at 3.3–10 Hz depending on player speed, rather than being rebuilt by individual sensors.
- Sensor-zone/OBB tests run at up to 4 Hz against the cached registry.
- At most 2 occlusion raycasts run per detection cycle: at the fastest 10 Hz cadence, a hard upper bound of 20 additional scene-raycast queries/s. The actual count is lower when fewer than two candidates enter sensor zones.
- The settings page adds the actual recent occlusion-raycast rate to the scheduled sensor-raycast estimate. This is a workload estimate, not an FPS or CPU measurement.

## Object tracking budget

- The target registry contains at most 256 valid vehicle OBB snapshots.
- Sensor-zone detection and tracker ageing run at 3.3–10 Hz against the cached registry and latest zone frame.
- The track store is capped at 128 objects. Association uses a source-ID index; the tracker does not perform scene raycasts.
- Timestamped velocity estimation runs only when a new registry observation arrives. Cached visibility and repeated frames do not count as new measurements.
- Tracks use a 0.75 s visibility-cache limit and a 1.5 s maximum observation age, with an additional missed-scan and registry-removal check.
- A coasting track's predicted display position is limited to a 0.5 s horizon; it is not fed back as a new measurement.

## Unified world-model budget

- World-model schema v2 contains separate LiDAR, radar, ultrasonic and simulator-assisted vehicle-registry observation streams.
- The snapshot is bounded to 512 observations and 128 tracked targets.
- Building the model reuses already computed raycast hits and track data; it performs no additional scene raycasts.
- The builder runs on incoming sensor frames and the 0.25 s detection/tracking cadence, not in the render loop.
- `getWorldModel()` returns a deep-copied snapshot so consumers cannot mutate the live fusion state.
- Obstacle clusters use a 0.75 m spatial cell. They are geometric hit clusters, not semantic object identities.
- Confidence fields are uncalibrated heuristics. Freshness and provenance must be considered by consumers.

## Adaptive workload and scheduling

- The FPS controller samples the Game Engine graphics update delta and uses a smoothed estimate. It changes sensor quality in one tier at a time, requiring sustained low FPS before reducing quality and a longer recovery window before raising it.
- Adaptive quality only scales down from the user's selected profile; it does not exceed that profile's ray count or scan frequency.
- Speed adaptation multiplies sensor intervals by 1.5 below 5 m/s, keeps the base interval from 5 to 25 m/s, and multiplies by 0.75 at or above 25 m/s.
- Traffic OBB registry and tracking updates use a separate speed-based cadence: 0.30 s below 5 m/s, 0.15 s from 5–20 m/s and 0.10 s at or above 20 m/s. These updates use BeamNG vehicle-object position/OBB data rather than raycasts.
- Detection-zone transforms refresh independently at 0.15/0.10/0.075 s (about 6.7–13.3 Hz) for low/medium/high vehicle speed. This recomputes sensor-zone geometry only; it adds no scene raycasts.
- The Vehicle Lua scheduler executes only one due sensor/pose/detection-zone job per graphics frame in round-robin order. It spreads simultaneously due jobs across successive frames instead of issuing all scans in the same update.
- Target FPS is best-effort. If quality reaches its minimum and measured FPS remains below target, the game or the rest of the scene is the limiting factor.

## Scheduling

- Sensor scan frequencies are controlled by the selected profile and can be customized, then optionally scaled by the FPS and vehicle-speed controllers.
- Camera frustums, IMU, GNSS-like position and odometry use the selected 2–5 Hz pose rate when enabled.
- The complete suite is disabled until the player activates it from the settings page or Controls binding.
- Sensor data is cached between updates.
- Game Engine scene raycasts run only on incoming sensor-scan updates; the render loop redraws cached data and performs no raycasts.

## Route planning and safety workload

When the experimental route pilot is active, the forward stop guard evaluates at most 10 times per second, including immediate opportunities after incoming sensor frames. It checks only enabled, fresh sensor streams and recent vehicle-registry visibility. Confirmed nearby vehicle obstacles are tested using their projected OBB footprint.

The local trajectory planner runs at up to 5 Hz. It evaluates at most five candidates and thirteen samples per candidate, with up to 128 nearby obstacle clusters and 96 tracked vehicle targets considered per plan. It uses the cached vehicle geometry profile, not new per-ray OBB queries. The planner performs no scene raycasts. Its current road boundary is a 3.5 m half-width NavGraph heuristic, not a verified lane polygon. When sensor/road data is stale or no candidate is safe, it requests a stop from the built-in Vehicle AI. The selected trajectory is visualized but not yet tracked by steering control; that belongs to Phase 10.

The controller receives only the selected candidate path and does not perform scene raycasts. It parses at most 32 path points (currently up to 13) only when the planner publishes a new candidate, at a maximum of 5 Hz. Vehicle Lua computes steering and speed control at 20 Hz with no per-frame path serialization or new scene queries. The full-brake timeout is 0.75 s without a fresh path. These are configured workload bounds, not measured CPU or FPS results.

The combined stop guard, planner and controller are bounded geometric/control logic, not a collision guarantee. Actual CPU cost, control stability and braking performance have not yet been measured in-game.

## ADAS and vehicle-attached rendering workload

- ADAS evaluates cached world-model/tracking and sensor-freshness data at up to 10 Hz. Its calculations add no independent scene raycasts.
- AEB sends an input update only when its requested brake level changes materially; it does not queue a brake command on every graphics frame.
- Vehicle-local sensor mounts, camera frustums, calibration geometry and detection-zone origins are refreshed with the existing pose/detection-zone jobs. The detection-zone cadence is 6.7–13.3 Hz depending on speed; this adds transform/table work and a small pose payload, not raycast work.
- Game Engine Lua re-anchors mounted debug geometry to the current vehicle pose during rendering. Dynamic traffic OBBs and tracked LiDAR/radar/ultrasonic surface returns use predicted vehicle-local positions rather than damped world-space interpolation.
- Static-scene returns and the accumulated point cloud remain fixed in world coordinates. Bounded visual-frame tables are copied before transforms so the source observations are not mutated.

The extra Lua/table and payload cost has not yet been measured in-game. The existing scene-raycast estimates do not represent total CPU, serialization or rendering cost.

## Allocation-conscious rendering

- The render snapshot reuses its table tree between graphics frames. Attached sensor data is still copied before pose transforms, so the source measurements and World Model remain immutable, but existing destination tables are updated in place instead of allocating a fresh deep-copy tree each frame.
- Inertia output tables and spring-state position/velocity tables are reused. This preserves the same critically damped spring constants and snap threshold while reducing short-lived Lua table creation.
- The vehicle pose basis is converted and normalized once per render frame, then reused for mounted geometry, sensor origins and localization transforms.
- Vehicle OBB corner storage and duplicate-suppression keys are scratch tables reused by the renderer.
- Persistent LiDAR cloud points are immutable after insertion, so their engine Point3F representation is cached rather than recreated for every draw.
- These changes do not alter sensor ray counts, scan cadence, range, tracking cadence, planner candidate/sample limits, controller cadence or ADAS thresholds. They reduce Lua allocation and repeated conversion work; the actual FPS/CPU gain still needs an in-game before/after measurement.

## Rendering budget

The High preset scans 51 LiDAR directions (17 × 3); the horizontal ray count and range are profile-controlled rather than exposed as sliders. Radar and ultrasonic limits remain 17 and 36 directions respectively.

The live LiDAR renderer is capped at 99 active ray returns plus 198 fading markers. Inertia uses a critically damped spring and adds no raycasts. A separate session-only point map can retain 500–10,000 deduplicated world-space returns; at the 10,000-point limit it issues up to 10,000 debug-sphere draw calls per graphics frame, so a lower cap is recommended on weaker systems. The map is not aged or cleared by pause; it is cleared only by the explicit clear action or a vehicle switch. Vehicle OBB outlines are limited to the 64 nearest non-occluded candidates. These debug draws are separate from sensor query budgets and can materially affect rendering performance.

## In-game Lua profiler

The optional **Detailed Lua profiler** lives in **ESC → Mods → RoadMind → Performance** and is off by default. Enabling it adds high-resolution timer reads around existing work; it does not change sensor settings, ray counts, update intervals, planner limits, ADAS thresholds or controller cadence.

The profiler keeps a fixed 15-second ring of one-second aggregates in memory. It reports:

- Vehicle Lua sensor scheduler total time and separate LiDAR, radar, ultrasonic, pose and detection-zone jobs.
- GE sensor-frame decode/merge and per-sensor post-processing.
- Vehicle detection/tracking, World Model construction, route planner, NavGraph localization and route safety.
- ADAS, render preparation, 3D Debug Draw and the complete RoadMind GE update callback.
- Current Game Engine Lua heap estimate from `collectgarbage("count")`.

Each stage shows average, peak and most recent execution time plus call frequency. The total GE callback includes nested stages; do not add the displayed stage times together. The heap estimate is for GE Lua, not total game RAM. These timings do not include other extensions such as `gameplayInterface`, other mods, the GPU or the game's renderer outside RoadMind's callback.

The profiler uses no disk logging and clears its rolling history when toggled. Leave it disabled outside a measurement session to avoid timing overhead.

## Profiling procedure

1. Record a baseline with RoadMind disabled.
2. Compare Economy (LiDAR only), Balanced (all three scan groups), and High (all groups at maximum configured density). Turn overlays on separately when measuring rendering cost.
3. Compare the same vehicle, map, traffic and time of day.
4. Record CPU use, average frame time and low-percentile frame rate.
5. Change only one scan rate or ray-count setting at a time.

Actual CPU cost depends on map complexity, vehicle geometry, traffic and game version. No fixed CPU percentage is claimed until measured in BeamNG.
