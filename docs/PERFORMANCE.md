# Performance notes

RoadMind starts with a small sensor budget because BeamNG already performs demanding vehicle-physics simulation.

## Current defaults

| Setting | Default |
|---|---:|
| Horizontal field of view | 120° |
| Vertical field of view | 16° |
| Horizontal samples | 17 |
| Vertical samples | 3 |
| Maximum ray directions per scan | 51 |
| Scan frequency | 4 Hz |
| Maximum range | 60 m |
| Visualization | Off until toggled |

The default sensor budget is 51 ray directions × 4 scans per second = 204 ray directions per second. Depending on the scene, one direction can cause both a static and a dynamic raycast. The approximate upper bound is therefore 408 raycast calls per second.

This is a workload estimate, not a measured CPU percentage. Real cost depends on scene complexity, BeamNG version, hardware and active vehicles.

## Rendering behavior

- World-space visualization is disabled until the player presses the RoadMind binding.
- While enabled, the latest scan is redrawn with debugDrawer each graphics frame because debug primitives are transient.
- The render loop does not perform any raycasts.
- Only valid return beams and hit points are drawn; no-hit beams are omitted.
- At most 51 hit markers can be drawn per scan.

## Rules for future features

- Keep sensor scanning independent of the render frame rate.
- Never rebuild or re-raycast a point cloud just because the camera frame changed.
- Cap the number of rays, points and trajectory candidates.
- Keep visualization optional so the driving stack can use sensor data without drawing it.
- Profile on a busy map with traffic before increasing ray density or scan rate.
- Compare frame times with the sensor disabled, scanning only and world visualization enabled.

## Tuning order

Change one setting at a time. Record a baseline on a repeatable map and vehicle, then compare scanning-only and visualization-enabled runs. Increase ray density only when a measured planner limitation requires it.
