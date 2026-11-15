# Roadmap

RoadMind aims to become a lightweight autonomous-driving system for BeamNG.drive. Development is split into small milestones so sensor behavior, planning and vehicle control can be tested independently.

Status: **early prototype — world-space LiDAR visualization milestone implemented; in-game BeamNG validation is still required.**

## Phase 1 — Sensor foundation and world visualization

**Status: implemented in the current prototype; requires in-game validation.**

- [x] Forward-facing virtual sensor with 120° horizontal field of view.
- [x] 16° vertical field of view.
- [x] Capped 17 × 3 ray pattern (51 directions).
- [x] Static and dynamic ray-hit collection.
- [x] World-space hit coordinates sent from Vehicle Lua to Game Engine Lua.
- [x] 3D debug lines from the virtual sensor origin to each hit.
- [x] 3D hit markers and virtual sensor-origin marker.
- [x] Toggle through a normal BeamNG Controls binding.
- [x] No dedicated UI app or external runtime.
- [ ] Verify ray orientation and hit coordinates on multiple vehicle types.
- [ ] Verify visual persistence, color and point scale across BeamNG graphics settings.

**Acceptance criteria:** a player can assign one key, press it while driving, see sensor-origin rays and hit points in the 3D world, and press the same key to stop scanning and hide the overlay.

## Phase 2 — Sensor accuracy and performance

- [ ] Calibrate the virtual mounting position using each vehicle's dimensions rather than one fixed offset.
- [ ] Add adjustable sensor height, forward offset, range and field of view.
- [ ] Add low / medium / high ray-density presets with a hard ray budget.
- [ ] Measure CPU and frame-time overhead on light and busy maps.
- [ ] Test static objects, traffic, parked vehicles, barriers and road edges.
- [ ] Add point filtering for ground returns, overhead objects and self-hits.
- [ ] Add scan age and sensor-health reporting.
- [ ] Separate sensor activation from visualization activation when the autopilot is introduced.

**Acceptance criteria:** sensor placement is repeatable across vehicle sizes; performance has a recorded baseline; changing visual settings does not cause extra raycasts.

## Phase 3 — Road understanding and route input

- [ ] Read the map road graph and identify the road segment under the vehicle.
- [ ] Build a local road corridor and estimate lane direction.
- [ ] Accept ordered route points and a destination.
- [ ] Track route progress and select the next target point.
- [ ] Handle intersections, road splits and route loss.
- [ ] Add a route preview in the 3D world using debug drawing.

**Acceptance criteria:** RoadMind can keep a valid route corridor and advance through ordered waypoints without using the built-in BeamNG driving AI as the planner.

## Phase 4 — Local trajectory planner

- [ ] Generate a bounded set of candidate trajectories in front of the vehicle.
- [ ] Test each candidate against LiDAR obstacle points.
- [ ] Score progress, obstacle clearance, road-boundary distance, curvature and steering smoothness.
- [ ] Choose a safe candidate and expose the reason for the choice.
- [ ] Detect when every candidate is blocked and request a controlled stop.
- [ ] Visualize the selected trajectory and rejected alternatives in the 3D world.

**Acceptance criteria:** the planner selects a collision-free candidate in repeatable static-obstacle scenarios and reports BLOCKED when no candidate passes the collision checks.

## Phase 5 — Autonomous vehicle control

- [ ] Implement trajectory tracking with a lightweight geometric controller (Pure Pursuit or Stanley).
- [ ] Add speed control with acceleration and braking limits.
- [ ] Reduce target speed for tight curvature and low obstacle clearance.
- [ ] Add smooth start, stop and gear handling.
- [ ] Add a safety supervisor for stale sensor frames, invalid routes and blocked paths.
- [ ] Add a dedicated enable/disable autopilot binding.
- [ ] Keep manual control available and make handover immediate.

**Acceptance criteria:** the vehicle follows a route at a configurable low speed, stops for a blocked path, and releases control cleanly when autopilot is disabled.

## Phase 6 — Robustness and scenario testing

- [ ] Test on multiple maps and vehicle classes.
- [ ] Test stationary obstacles, moving traffic, roadworks, narrow roads and intersections.
- [ ] Test sensor occlusion, missing returns and vehicle damage.
- [ ] Add deterministic test routes and repeatable scenario scripts.
- [ ] Record route, sensor, planner and controller telemetry for debugging.
- [ ] Profile CPU and frame-time overhead with visualization both enabled and disabled.
- [ ] Document known limitations and supported BeamNG versions.

**Acceptance criteria:** each core behavior has a reproducible test scenario and failures are reported clearly rather than silently continuing.

## Phase 7 — Optional advanced perception

- [ ] Track moving objects across scans using position and velocity estimates.
- [ ] Add camera-based lane and traffic-light perception only where LiDAR and map data are insufficient.
- [ ] Explore optional machine learning only after a deterministic baseline exists.
- [ ] Keep any model runtime optional and disabled by default.

**Acceptance criteria:** advanced perception must improve a measured driving task enough to justify its CPU, memory and maintenance cost.

---

## Performance principles

- No raycasts on every render frame.
- Hard caps on rays, points and candidate trajectories.
- Sensor acquisition, world visualization and vehicle control use separate update rates.
- Visualization is optional and must not be required for autopilot operation.
- No neural network or external process in the baseline installation.
- CPU claims are made only after profiling on defined maps, vehicles and traffic levels.

## Current non-goals

- No 360° scanner in the baseline vehicle setup.
- No visible sensor mesh is required.
- No HTML/CEF mini-app is required for the LiDAR visualization.
- No use of the built-in BeamNG AI as the core route planner.
