# Roadmap

RoadMind aims to become a lightweight autonomous-driving system for BeamNG.drive, using a sensor suite, route understanding, local trajectory planning and vehicle control.

Status: **virtual sensor-suite prototype; in-game BeamNG validation is required.**

## Phase 1 — Forward LiDAR and world visualization

- [x] Forward virtual LiDAR with 120° horizontal field of view.
- [x] 51-ray scan budget and 4 Hz scan rate.
- [x] Static and dynamic raycast returns.
- [x] In-world ray and hit-point visualization.
- [x] Single Controls binding for the visualization.
- [ ] Validate sensor pose on multiple vehicle sizes and configurations.

## Phase 2 — Multi-sensor automotive layout

- [x] Five virtual radar units: front long-range plus four corner radars.
- [x] Twelve virtual ultrasonic sensors around the bumpers and corners.
- [x] Seven camera mounts with independent 3D FOV frustums.
- [x] IMU-like acceleration and yaw-rate data.
- [x] GNSS-like world-position data.
- [x] Wheel speed and odometer data from vehicle electrics.
- [x] One-key sensor-suite visualization.
- [ ] Calibrate all virtual sensor positions against vehicle dimensions.
- [ ] Validate radar object hits and relative-speed estimates in traffic.
- [ ] Validate ultrasonic coverage against curbs, posts and parked vehicles.
- [x] Add per-sensor enable/disable controls and scan-density profiles.
- [x] Add bilingual pause-menu settings with saved performance profiles.
- [ ] Add scan-health indicators and measured CPU/FPS profiling.
- [ ] Measure CPU overhead on light and dense maps.

**Acceptance criteria:** one key displays the complete virtual sensor layout; all measurement sources are identified honestly; scanning stops when the suite is disabled.

## Phase 3 — Sensor data quality

- [ ] Add vehicle-specific virtual mount calibration from bounding-box dimensions.
- [ ] Add point filtering for ground, overhead geometry and self-hits.
- [ ] Add radar return association and short-term target tracking.
- [ ] Add confidence and data-age fields to every sensor.
- [x] Add optional low/medium/high sensor-density profiles.
- [ ] Add repeatable sensor test scenes and debug exports.

## Phase 4 — Road understanding and route input

- [ ] Read the map road graph and identify the road segment under the vehicle.
- [ ] Build a local road corridor and estimate lane direction.
- [ ] Accept ordered route points and a destination.
- [ ] Track route progress and select the next target point.
- [ ] Handle intersections, road splits and route loss.
- [ ] Add a route preview in the 3D world.

## Phase 5 — Local trajectory planner

- [ ] Generate a bounded set of candidate trajectories ahead of the vehicle.
- [ ] Test each candidate against LiDAR, radar and ultrasonic obstacle data.
- [ ] Score route progress, obstacle clearance, road-boundary distance, curvature and steering smoothness.
- [ ] Choose a safe candidate and expose the reason for the choice.
- [ ] Request a controlled stop when every candidate is blocked.
- [ ] Visualize the selected trajectory and rejected alternatives in the 3D world.

## Phase 6 — Autonomous vehicle control

- [ ] Implement trajectory tracking with a lightweight geometric controller (Pure Pursuit or Stanley).
- [ ] Add speed control with acceleration and braking limits.
- [ ] Reduce target speed for tight curvature and low obstacle clearance.
- [ ] Add smooth start, stop and gear handling.
- [ ] Add a safety supervisor for stale sensors, invalid routes and blocked paths.
- [ ] Add a dedicated autopilot enable/disable binding.
- [ ] Keep manual control available and make handover immediate.

## Phase 7 — Validation and robustness

- [ ] Test on multiple maps and vehicle classes.
- [ ] Test stationary obstacles, moving traffic, roadworks, narrow roads and intersections.
- [ ] Test sensor occlusion, missing returns and vehicle damage.
- [ ] Add deterministic test routes and scenario scripts.
- [ ] Record sensor, route, planner and controller telemetry.
- [ ] Profile with the sensor suite disabled, scanning only and visualization enabled.
- [ ] Document supported BeamNG versions and known limitations.

## Phase 8 — Optional BeamNG.tech native sensors and advanced perception

- [ ] Add a native BeamNG.tech sensor adapter for rendered camera, LiDAR, RADAR, ultrasonic, GPS and IMU.
- [ ] Add camera-based lane, sign and traffic-light perception when native image data is available.
- [ ] Explore sensor fusion and machine learning only after a deterministic baseline exists.
- [ ] Keep advanced model runtimes optional and disabled by default.

---

## Performance principles

- No raycasts on every render frame.
- Balanced profile target: about 74 scene-raycast queries per second; Economy: 26; High: 310.
- These are calculated query budgets, not measured FPS or CPU results.
- Hard caps on rays, points and trajectory candidates.
- Radar and ultrasonic scans run at lower rates than LiDAR.
- Camera frustums are geometry only; no image rendering in the base Drive build.
- Sensor acquisition, visualization and vehicle control remain separate.
- Visualization is optional and not required for future autopilot operation.
- CPU claims are made only after profiling on defined maps, vehicles and traffic levels.

## Current limitations

- Radar and ultrasonic are raycast approximations, not full RF/acoustic physics.
- Camera frustums do not contain RGB pixels in the regular BeamNG.drive build.
- GNSS-like output is in world coordinates, not latitude/longitude.
- IMU values are derived from simulator signals and are not calibrated hardware-grade outputs.
- Autonomous path planning and vehicle control are not implemented yet.
