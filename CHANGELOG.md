# Changelog

## Unreleased

- Add NavGraph road localization, short road-centerline context and route-state export for future planners.
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
- Add optional critically damped inertia for LiDAR endpoints and detected vehicle outlines.
- Remove the manual LiDAR range and horizontal-point sliders; these remain controlled by performance profiles.
- Default to LiDAR-only Economy scanning (about 9 queries/s) and the LiDAR overlay to reduce overhead.
- Keep sensor scanning disabled until explicitly started.

- Move scene raycasts to Game Engine Lua, where BeamNG exposes `castRay`.
- Send only due sensor beams to the Game Engine to preserve the 4 Hz / 2 Hz scan schedules.
- Keep missed rays empty instead of inventing returns at maximum range.
- Preserve full floating-point precision for beam directions.
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
