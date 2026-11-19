# RoadMind architecture

RoadMind is a layered sensor and driving-stack prototype for BeamNG.drive. Current development follows the perception-first roadmap: vehicle geometry, calibrated sensors, target observations and a timestamped world model come before custom trajectory planning and control.

## Vehicle geometry and calibration

The Game Engine Lua coordinator reads the active vehicle's oriented spawn bounding box through `getSpawnWorldOOBB()` when a vehicle is attached, or when the user requests recalibration. It compares the OBB axes with the vehicle's live forward/right/up basis and projects the OBB center and half-extents into vehicle-local coordinates.

The cached geometry profile contains:

- Local forward/rear, left/right and lower/upper bounds relative to the vehicle reference position.
- Length, width and height.
- OBB-axis alignment quality and the geometry source.
- A vehicle key when the engine exposes one.

The profile is sent to Vehicle Lua through the vehicle command queue. Vehicle Lua derives a cached sensor layout from those local bounds. LiDAR, radar, ultrasonic and camera mounts therefore use vehicle-relative positions; their world-space transforms are calculated from the current vehicle pose only when a sensor or pose frame is due.

If the OBB is missing, invalid or does not align with the live vehicle basis, the sensor backend uses its conservative fallback mount layout and reports calibration as unavailable. The settings page provides an explicit recalibration action. Reset/configuration callback entry points are present, but their dispatch still requires live validation on supported BeamNG versions.

The optional calibration overlay draws the body bounds, forward/right/up axes and all sensor mount positions. It is disabled by default and requests pose updates only while enabled.

Per-vehicle manual mount overrides and validation across vehicle classes remain unfinished roadmap work.

## Vehicle Lua sensor backend

Location: `lua/vehicle/extensions/roadmind/sensors.lua`

The Vehicle Lua extension owns vehicle-local scan geometry and scheduling:

- Forward LiDAR: configurable horizontal sample count and 1/2/3 vertical layers, up to 51 rays in the High profile.
- Five radar ray fans: 17 directions total in the High profile.
- Twelve ultrasonic units: up to 36 short-range directions.
- Seven camera mount poses and FOV values. In regular Drive these are coverage frustums, not RGB image sources.
- IMU-like acceleration and estimated yaw-rate data.
- GNSS-like world-position and wheel-odometry data.

Sensor layout is calculated and cached when the GE sends a validated geometry profile. The Vehicle Lua module updates world-space mount transforms from the current vehicle frame when a scan/pose update is due. It does not call the Game Engine-only scene-raycast API.

## Game Engine Lua coordinator and renderer

Location: `lua/ge/extensions/roadmind.lua`

The Game Engine Lua extension:

- Attaches the sensor backend to the active player vehicle and caches its geometry profile.
- Receives serialized sensor frames through the cross-VM command queue.
- Performs scheduled scene raycasts through `castRay`.
- Rejects delayed frames from a vehicle that is no longer active.
- Merges raycast returns and sensor state into the current frame.
- Draws LiDAR returns, radar returns, ultrasonic returns, camera frustums, localization markers, detected-vehicle markers and optional calibration geometry.
- Redraws cached geometry each graphics frame because debug primitives are transient.

Scene raycasts run only when a sensor scan is due, not in the render loop.

## Phase 4 — shared vehicle registry and detection zones

The GE coordinator maintains one shared registry of nearby vehicle OBB snapshots:

- The vehicle list is collected on a 0.5 s schedule, not once per sensor or LiDAR ray.
- The player vehicle, invalid entries and entries reported as deleted/destroyed are excluded. Entries without usable world OBB data are not registered; each fresh snapshot replaces the previous one.
- Each registered vehicle stores its ID, OBB center, three axes, half-extents, position and simulator velocity.
- Detection runs on a separate 0.25 s schedule using the latest sensor frame.
- Calibrated LiDAR, radar and ultrasonic mounts define their own horizontal/vertical coverage volumes, minimum range and maximum range.
- A cheap range check precedes OBB projection tests against each sensor volume. One vehicle is emitted once with all sensor zones that contain its OBB.
- At most four candidate-to-target occlusion raycasts run per detection cycle (up to 16/s). Candidates are classified as visible, occluded or unverified. Only visible targets are drawn and used by the experimental route safety guard.

This service is **simulator-assisted**: the game provides the list of vehicles and their OBBs. It is not a claim that a physical LiDAR/radar/camera independently recognized the target. Confirmed candidates feed the session-scoped tracker and schema-v2 world model described in [WORLD_MODEL.md](WORLD_MODEL.md).

## Phase 5 — object tracking

The GE coordinator associates each confirmed-visible simulator-assisted OBB observation with a session-scoped RoadMind track ID (`RM-000001`, etc.). The transient BeamNG vehicle ID is retained only as an internal association key; it is not used as the RoadMind track identifier.

Each track stores the OBB center, dimensions, vehicle orientation, sensor-zone sources, simulator-provided velocity (separately labelled), last-observed simulation timestamp, confidence, data age, missed-scan count and status.

Estimated target velocity is derived from successive timestamped OBB-center positions. Estimated relative speed subtracts the current simulator vehicle velocity and projects along the player-to-target direction. Neither value is physical radar Doppler. A short predicted position (capped to a 0.5 s horizon) is exposed separately from the latest observed position.

Tracks remain briefly in a coasting state when a candidate is occluded or missed. Recent visibility cache lifetime is 0.75 s; tracks expire after 1.5 s of observation age, more than six missed detection cycles, two complete registry snapshots without the source vehicle, or an explicit vehicle-destroyed event. The tracker is bounded to 128 tracks. Player-vehicle switches, resets and scanner shutdown clear the active tracking state.

The UI exposes up to eight closest track summaries, including RoadMind ID, state, estimated speeds, data age, confidence and sensor-zone sources. In-world markers distinguish confirmed tracks from coasting predictions.

These are code-level tracking semantics. Stable identity and motion accuracy still require live-traffic validation in BeamNG.drive.

## Phase 6 — time-consistent LiDAR render layer

Raw LiDAR returns remain in `state.latestData.lidar.rays`. Each raw scan and each valid point carries the GE simulation timestamp; every hit also retains its ray index and, when exposed by the raycast, the hit object's ID.

A separate render-only state is keyed by ray index:

- Moving-object returns associate with a recent RoadMind track by source vehicle ID, or by a unique hit-inside-OBB check when the raycast has no vehicle object ID.
- Associated points are stored in the tracked vehicle's local OBB frame. Their render positions are reconstructed from the latest timestamped track position, orientation and finite-difference velocity.
- Track motion can be extrapolated for rendering only up to 0.5 s. This is not a new measurement and is never written to the raw LiDAR frame.
- Static-scene returns remain in world coordinates. Static hits with a known object ID keep that identity; unknown static hits are continued only when the same ray's world point remains within a 0.35 m tolerance.
- If a ray changes object/track association, jumps to a different surface, misses, or loses its track, the previous marker fades separately instead of being interpolated to the new hit.
- Active markers are capped at 99, fading markers at 198, and render points expire after 2.25 s without a new scan. Fades last 0.4 s.
- Render interpolation runs from the GE update loop using elapsed time and bounded per-frame movement. It performs no raycasts.

Raw LiDAR returns, timestamped measurements, persistent static scan points, and render-only point state are separate. The route safety guard continues to inspect raw raycast returns and confirmed target observations, never smoothed render positions.

This is a code-level visualization implementation. Smoothness, object association and surface-switch behavior still require in-game verification.

## Phase 7 — unified world model

The GE coordinator exposes `extensions.roadmind.getWorldModel()`, which returns an independent deep-copied schema-v2 snapshot in BeamNG world coordinates and metres.

The model contains:

- A common observation envelope with position, source, sensor mount, timestamp, data age, confidence and provenance.
- Separate raw observation streams for LiDAR, radar, ultrasonic and simulator-assisted vehicle OBB detections.
- One target record per RoadMind track ID, with observed and predicted positions kept separate.
- Spatially grouped non-vehicle scene-raycast obstacle clusters, retaining contributing observation IDs and sensor sources.
- Per-source freshness/status and quality counters for invalid, stale, conflicting, low-confidence, truncated and dropped data.

Raw scene-raycast hits remain source-specific. Vehicle OBB observations are explicitly labelled simulator-assisted. Estimated motion and predicted display positions are not described as physical radar Doppler or new measurements. See [WORLD_MODEL.md](WORLD_MODEL.md) for the complete versioned schema, fields, limits and consumer rules.

The world-model builder runs on sensor/detection updates, has a bounded observation and target budget, and performs no additional scene raycasts.

## Phase 8 — road understanding and route input

The GE coordinator samples the active vehicle against BeamNG's current NavGraph at a 0.5 s cadence:

- map.findClosestRoad(position) identifies the closest road segment and its endpoint nodes.
- The segment's node positions are projected against the vehicle pose to estimate distance, lateral offset and heading alignment.
- A short local centerline corridor is extended through forward-connected graph links where the map exposes usable link data. This is NavGraph centerline geometry, not detected lane boundaries or verified free drivable space.
- Localization quality is reported as high, medium, low, lost or unavailable using geometric distance and heading-alignment thresholds. These are engineering heuristics, not probabilistic localization covariance.

The route manager accepts 1–12 ordered XYZ points in BeamNG world coordinates (metres). It resolves each point to nearby road nodes, joins successive points with map.getPath, validates node positions and graph links, and bounds the combined path to 256 nodes. The settings UI accepts one X, Y, Z point per line. The final input point is snapped to a nearby NavGraph node; the UI reports the snap distance, so "destination reached" means that final graph node was reached, not necessarily the exact entered coordinate. Route progress is measured against nearby path segments at 4 Hz. A deviation above 30 m for four consecutive checks stops the AI, marks route localization as lost and schedules a replan attempt after a cooldown.

The GE API function extensions.roadmind.getRouteState() returns a cloned snapshot containing the current road segment, local corridor, localization quality, ordered input points, route polyline, progress, next target, endpoint snap distance, safety state and local-planner snapshot. Callers cannot mutate the coordinator's internal tables through the returned snapshot.

### Phase 9 — bounded local trajectory planner

While the experimental route pilot is armed, the GE coordinator evaluates five lateral-offset trajectory candidates up to 25 m ahead, at a 5 Hz maximum planning rate. The path sampler uses the current NavGraph route polyline and the calibrated vehicle-local OBB profile. Each sample evaluates the complete ego footprint against fresh non-vehicle obstacle clusters and confirmed/coasting tracked vehicle OBBs. Candidates are rejected when their footprint intersects an obstacle/target, exceeds the conservative NavGraph corridor band, or requires a turning radius below a length-based vehicle heuristic.

The selected safe candidate is shown in green; rejected candidates are shown in red, while other valid candidates are blue. Planner state, selected offset, road margin, obstacle clearance and rejection reason are exposed through settings status and the cloned route-state API. If all candidates fail or required road/sensor geometry is stale, the coordinator stops the built-in Vehicle AI route pilot. The selected trajectory is currently a preview and safety-decision input only; it does not steer or control pedals. Low-level trajectory tracking belongs to Phase 10.

The road boundary is an explicit approximation: a 3.5 m half-width band around the NavGraph centerline, not a detected lane boundary or exact drivable polygon. The turning-radius estimate is derived from vehicle length rather than measured steering geometry. These assumptions require in-game validation and must not be treated as a collision guarantee.

The active route is previewed in world space and passed as a waypoint list to the built-in Vehicle Lua ai.driveUsingPath. The base Drive-mode route pilot remains experimental: it uses the game's AI for lane following, and RoadMind runs a best-effort stop guard at up to 10 Hz using only enabled, fresh sensor returns plus recently confirmed vehicle OBBs. Vehicle obstacles are checked by their projected footprint, not only their center. This is not a collision guarantee or a replacement for the future swept-footprint trajectory planner. Camera frustums do not provide image measurements. Complex intersections, road splits, map compatibility, coordinate entry and progress still need live validation on several maps.

## Input and settings

The sensor-suite binding is defined in `lua/ge/extensions/core/input/actions/roadmind_actions.json`. The in-game settings page is under `ui/ui-vue/mods/roadmind/`.

The settings page supports performance profiles, sensor density/rates, point-cloud visualization, vehicle-geometry status, manual recalibration and the optional calibration overlay. It follows the game's language (Russian/English).

## Sensor model boundaries

The base mod targets regular BeamNG.drive. LiDAR, radar and ultrasonic use geometric scene raycasts; radar is not RF/FMCW simulation and ultrasonic is not an acoustic propagation model. Camera mounts display FOV frustums but do not return RGB frames. IMU-like values use vehicle electrics and heading differences. GNSS-like positioning uses simulator world coordinates. Wheel speed and odometer come from vehicle electrics.

BeamNG.tech provides a separate native suite with rendered camera, LiDAR, RADAR, ultrasonic, GPS and IMU sensors. A future adapter can provide native measurements without changing the upper-level observation interface.

## Current stack boundary and next phases

Implemented code-level layers: vehicle registry/detection zones, session-scoped object tracking, timestamped sensor fusion/world model, NavGraph road context and route input/progress. These layers still need repeatable in-game validation.

The route pilot now uses the native `lua/vehicle/extensions/roadmind_controller.lua` Vehicle Lua extension rather than BeamNG's built-in route AI. The GE coordinator sends the selected planner path as a compact JSON snapshot. The vehicle controller parses it once, then performs bounded trajectory tracking at 20 Hz.

### Phase 10 — native trajectory controller

- **Lateral control:** Pure Pursuit-style geometric tracking against the selected local path, with a speed-dependent lookahead bounded to 4–14 m.
- **Longitudinal control:** bounded PI speed control, a conservative 8 m/s target-speed ceiling and additional speed reduction for high steering demand.
- **Actuation:** direct Vehicle Lua `input.event` calls for steering, throttle and brake.
- **Supervisor behavior:** path-feed timeout (0.75 s), missing vehicle state, unavailable path and route-tracking loss latch a full-brake stop. A new approved planner path is required before motion resumes.
- **Bounded work:** at most 32 path points are accepted; the current planner sends at most 13 samples. The controller computes at 20 Hz and receives a new path only when the planner publishes a new result (maximum 5 Hz). No new scene raycasts are added by the controller.
- **Vehicle assumptions:** the first prototype uses a nominal 2.7 m wheelbase and conservative steering scaling. Per-vehicle geometry calibration and dynamic steering limits remain future tuning work.

The controller is an initial code-level implementation, not a validated autonomous-driving system. BeamNG input polarity, vehicle-specific steering response, braking performance, route tracking, manual takeover and reset/recovery behavior still require in-game tests. The NavGraph corridor is still a 3.5 m half-width heuristic, not a detected lane polygon. The supervisor is best-effort and cannot guarantee collision avoidance.

Next implementation phases:

1. Validate and tune the native controller across stock vehicle classes and road conditions.
2. Verify manual takeover, route loss, stale sensors, resets, pause/resume and fault recovery.
3. Add robust intersection negotiation, lane changes and parking only after baseline control is repeatable.
4. Add bounded local recording and deterministic replay before optional offline learning.
