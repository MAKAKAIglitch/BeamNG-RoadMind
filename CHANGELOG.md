# Changelog

## Unreleased — native Lua autonomous controller

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
