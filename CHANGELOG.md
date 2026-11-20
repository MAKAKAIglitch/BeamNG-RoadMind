# Changelog

## Unreleased — Tesla FSD-style 3D perception map

- Set a forward-looking FSD-style perspective with the ego vehicle low in the foreground, a 120 m default road preview, and independent camera height, follow distance and look-ahead controls.
- Build the primary road ribbon from cached `DecalRoad` nodes in `items.level.json`, including per-node width, lane counts and direction settings; sample the spline at approximately 2.5 m intervals with Catmull–Rom interpolation blended by road smoothness.
- Render continuous road ribbons and lane separators from the precomputed spline points, with a distinct opposing-flow divider; retain the expanded NavGraph network as a fallback when DecalRoad data is not available.
- Cache world-space road samples and refresh them on movement, heading changes or cache expiry; reject distant road objects by bounding box and prioritize nearby roads within fixed run/sample budgets.
- Publish fresh LiDAR, radar and ultrasonic hit points with their sensor origins/directions, plus geometric camera/radar/LiDAR/ultrasonic coverage zones.
- Render sensor-specific point colours, proximity rays and translucent coverage wedges; distinguish vehicles confirmed by camera visibility from other tracked traffic.
- State explicitly that regular BeamNG camera models provide geometric frustums and vehicle visibility candidates, not RGB image recognition or semantic segmentation.

## Unreleased — NavGraph road surface and lane geometry

- Build connected road strips from the local BeamNG NavGraph using shared miter joins at degree-two nodes and filled surface patches at graph junctions.
- Render each physical road edge once even when the NavGraph exposes both travel directions; use the connected corner geometry so asphalt edges do not split into separate rectangular pieces.
- Use the NavGraph node radius as road half-width; when it is missing, estimate total width as 2.4 m for one lane or 3.05 m per lane for two or more lanes, with a conservative fallback when both sources are missing. Preserve narrow widths in the renderer instead of forcing every road to at least 4 m.
- Generate separators for all lanes across both travel directions, distinguish the opposing-flow centre divider, and join marking endpoints across bends.
- Keep confirmed traffic trajectories as a lower-confidence fallback for lane locations only when directional NavGraph lane metadata is unavailable.

## Unreleased — solid graphite scanned-surface rendering

- Render reconstructed scan coverage as a uniform dark graphite surface instead of mint/teal triangles.
- Hide every triangle border and mesh edge so the internal tessellation is not visible in the map.
- Remove edge-only geometry generation, cache storage, spatial reindexing and draw submission; retain only the face data required for the filled surface.


## Unreleased — RoadMind Map circular memory and lane-aware rendering

- Replace the whole-map reset after 1 km of driving with a moving circular world-space cache (default radius 500 m); remove out-of-radius cells incrementally as the player moves.
- Extend the local directed NavGraph neighbourhood up to the configured map-memory radius while retaining segment, hop and branch caps.
- Infer provisional lane centres from repeated, confirmed and direction-aligned traffic tracks when NavGraph lane metadata is unavailable; preserve evidence counts and source confidence.
- Render inferred lane centres and boundaries with a separate dashed style so estimated lanes remain distinguishable from authoritative NavGraph metadata.
- Add RoadMind Map camera height, follow distance, look-ahead, map radius, polygon budget and render FPS settings.
- Keep a dark perspective grid available when vehicle pose is missing; distinguish the road surface from the grid and pin the player's vehicle marker above the geometry layers.
- Bound polygon, line, marker and surface-point work; remove a duplicate map geometry rendering pass.
- Set the map widget render cadence to 30 FPS by default, configurable from 15 to 60 FPS.


## Unreleased — camera priority and ADAS safety fixes

- Give camera-frustum vehicle observations priority over overlapping LiDAR/radar/ultrasonic vehicle zones; retain camera-only provenance when a target is inside a camera frustum.
- Increase the shared, rotating and cached occlusion budget to eight scene raycasts per detection cycle (bounded to 80/s at the fastest 10 Hz traffic cadence).
- Keep the primary LiDAR layer level to world +Z regardless of chassis pitch/roll; use explicit horizontal, downward and upward elevation layers.
- Allow camera-confirmed vehicle candidates into the forward ADAS hazard guard and mark camera/registry observations as an available forward-perception source.
- Calculate vehicle-relative closing speed and time-to-collision for FCW/AEB; expose hazard distance, threshold, TTC and source in the ADAS status.
- Read driver takeover inputs from Vehicle Lua electrics input channels rather than treating the `input` API table as a numeric input table.
- Add live camera-zone, in-frustum candidate and camera-visible vehicle counters to RoadMind settings.
- Add cursor-ray hover picking for visible sensor geometry, with localized type/name tooltips for cameras, LiDAR, radar and ultrasonic mounts; UI mouse capture is respected and no scene raycasts are added.
- Require camera visibility rays to hit the candidate vehicle itself before confirming it; treat other scene hits as occlusion and ambiguous hits as unverified.
- Render camera-detected vehicles only while a matching camera candidate has a fresh confirmed line of sight; hide occluded, unverified and stale/coasting camera overlays.
- Shorten camera visibility-cache lifetime to 0.36 s, aligned with RoadMind's 0.10–0.30 s traffic-detection cadence.

## Unreleased — surround-view perception and ADAS interface

- Add seven-camera vehicle candidate zones spanning front, side, rear and rear-corner coverage, independent of LiDAR and camera debug overlays.
- Pass camera candidates through the shared bounded, rotating scene-occlusion checks and preserve the actually visible camera source in the World Model.
- Fix LiDAR elevation order: horizontal first, then downward, then upward.
- Add dedicated **RoadMind ADAS** and **RoadMind Perception** pause-menu pages; move ADAS and route-pilot controls out of core sensor settings.
- Add a lightweight dark 2D environment display backed by a bounded cached World Model snapshot, with top-down and rear-follow-style projection modes.
- Add default-on route-controller driver takeover for detected steering, throttle or brake input; camera/view movement is ignored.
- Document that regular BeamNG.drive uses simulator-assisted OBB/frustum visibility, not RGB image recognition, and that the NavGraph display is a road centreline rather than lane markings.

## Unreleased — native Lua autonomous controller

- Place the Vehicle Lua controller at `lua/vehicle/extensions/roadmind/controller.lua`, matching the `roadmind_controller` extension name resolved by BeamNG; guard missing-controller commands to avoid fatal nil dereferences and repeated load attempts.

- Add a native Vehicle Lua trajectory controller with Pure Pursuit-style steering and bounded PI speed control.
- Route the selected safe planner trajectory directly to the vehicle controller; route following no longer delegates to BeamNG's built-in Vehicle AI.
- Add direct steering, throttle and brake actuation through Vehicle Lua input events.
- Add a 20 Hz controller cadence, bounded path input (32 points maximum), and planner-driven path refresh capped at 5 Hz.
- Add latched full-brake handling for stale path feeds, missing vehicle state and lost route tracking.
- Keep the controller's work independent from scene-raycast acquisition; no additional sensor queries are introduced.
- Document the 8 m/s speed cap, nominal wheelbase assumption and outstanding in-game validation requirements.
- Add configurable ADAS master and independent FCW, AEB, ACC, BSM, parking-distance and LDW switches with Russian/English status reporting.
- Add forward-obstacle warning/braking, tracked-vehicle blind-spot checks, ultrasonic parking distance and NavGraph-based lane-departure warning.
- Add route-pilot-only adaptive cruise speed targeting; keep AEB actuator ownership separate between manual and autonomous driving.
- Publish vehicle pose and pose-only sensor-mount updates independently from raycast returns.
- Re-anchor mounted sensors, camera frustums, calibration geometry and detection zones to the live vehicle pose during rendering.
- Remove independent world-space smoothing from tracked traffic OBB centres so boxes follow their tracked/predicted vehicle poses.
- Bypass world-space easing for vehicle-mounted overlays and render LiDAR points on tracked vehicle surfaces directly from their OBB-local coordinates.
- Reproject radar and ultrasonic hit points on tracked vehicles from OBB-local surface coordinates; expire attached returns when the target track is lost.
- Deep-copy attached visual frames before render transforms so the source measurements and World Model remain immutable.
- Reuse render snapshot, smoothing, spring-state and traffic-overlay tables to reduce per-frame Lua garbage without changing the displayed geometry.
- Cache the vehicle pose transform basis once per render frame and reuse it for attached sensor geometry and localization.
- Cache immutable persistent LiDAR cloud Point3F values and reuse OBB/deduplication scratch tables.
- Keep sensor ray counts, scan rates, range, tracking/planner/controller cadence and ADAS thresholds unchanged during this optimization pass.

- Bind traffic OBB outlines to each source vehicle's live SceneObject position and orientation every graphics frame using a stored vehicle-local center offset; keep a bounded velocity-prediction fallback when an object transform cannot be read.
- Keep traffic OBB render dimensions synchronized with the stored vehicle dimensions and remove destroyed objects' render states; no extra sensor raycasts or changes to detection cadence.
- Route Vue settings calls through BeamNG's `runRaw` bridge with native Lua serialization, a method allowlist and a timeout so a missing callback releases the busy state and surfaces an error.



## Unreleased

- Add NavGraph road localization, short road-centerline context and route-state export.
- Add a bounded Phase 9 local trajectory planner with five candidates, full ego/target footprint checks, obstacle and road-corridor costs, preview visualization and a fail-safe route-AI stop.
- Add ordered world-coordinate route points, connected NavGraph path construction and route progress reporting.
- Add route-deviation detection and a stop/replan response; use only current enabled sensor streams in the experimental obstacle guard.
- Reject route candidates whose first waypoint would send the vehicle backwards to a node behind it.
- Check nearby vehicles by their projected OBB footprint in the route stop guard, and throttle guard evaluation to a maximum 10 Hz.
- Fix the missing vehicle OBB snapshot function declaration.
- Add a BeamNG 0.39+ Pause → Mods → RoadMind settings page.
- Add Russian/English interface text based on the game's selected language.
- Add persistent Economy, Balanced and High performance profiles.
- Add live LiDAR resolution, range and scan-rate controls.
- Add independent radar and ultrasonic density, frequency and enable controls.
- Add independent 3D overlay visibility controls and green/yellow OBB outlines for confirmed/unverified vehicle candidates.
- Add a session-only accumulated LiDAR map with a configurable 500–10,000 point cap, including vehicle hits and pause-safe rendering.
- Add optional critically damped inertia for eligible world-space LiDAR endpoints; vehicle-attached geometry and tracked traffic surfaces remain body-locked.
- Remove the manual LiDAR range and horizontal-point sliders; these remain controlled by performance profiles.
- Add speed-adaptive traffic OBB updates (3.3–10 Hz) independently from scene-raycast work, with a bounded rotating occlusion-check budget.
- Add a round-robin Vehicle Lua scheduler that handles at most one due LiDAR/radar/ultrasonic/pose/detection-zone job per graphics frame.
- Add optional FPS-target feedback that scales sensor ray counts, resolution and rates down from the selected profile with hysteresis; report when the minimum sensor tier still cannot meet the target.
- Add speed-adaptive sensor scan intervals and desired-FPS/current-FPS diagnostics in the performance section.
- Normalize English/Russian UI language from BeamNG's selected language setting and refresh it while the settings page remains open.
- Default to LiDAR-only Economy scanning (about 9 queries/s) and the LiDAR overlay to reduce overhead.
- Keep sensor scanning disabled until explicitly started.

- Move scene raycasts to Game Engine Lua, where BeamNG exposes `castRay`.
- Send only due sensor beams to the Game Engine to preserve the 4 Hz / 2 Hz scan schedules.
- Keep missed rays empty instead of inventing returns at maximum range.
- Preserve full floating-point precision for beam directions.
- Fix a missing newline between the adaptive FPS controller and the GE onUpdate callback.
- Clarify that radar radial velocity and stable target tracking are not implemented yet.

## 0.3.0 — Virtual automotive sensor suite

- Added five virtual automotive radar units.
- Added twelve short-range ultrasonic sensor proxies.
- Added seven camera mounts with world-space FOV frustums.
- Added IMU-like acceleration and estimated yaw-rate telemetry.
- Added GNSS-like world-position and vehicle-state output.
- Added wheel-speed and odometer telemetry from vehicle electrics.
- Replaced LiDAR-only overlay with one-key full sensor-suite visualization.
- Added a complete sensor inventory and updated architecture, roadmap and performance docs.

Radar and ultrasonic remain raycast approximations; camera mounts do not provide RGB image frames in the regular BeamNG.drive build.
