# Performance notes

RoadMind starts with conservative limits because BeamNG already performs a demanding vehicle-physics simulation.

## Prototype defaults

| Setting | Default |
|---|---:|
| Horizontal field of view | 120° |
| Vertical field of view | 16° |
| Horizontal samples | 17 |
| Vertical samples | 3 |
| Maximum rays per scan | 51 |
| Scan frequency | 4 Hz |
| Maximum range | 60 m |
| UI polling | 5 Hz |

The default sensor budget is 51 ray directions × 4 scans per second = 204 ray directions per second. A direction may require both a static and a dynamic raycast, so the theoretical upper bound is approximately 408 raycast calls per second.

This is a ray budget, not a guaranteed CPU percentage. Real cost depends on scene complexity, BeamNG version, hardware and the number of active vehicles.

## Rules for future features

- Keep the sensor scan rate independent of the render frame rate.
- Never rebuild or re-raycast the point cloud just because the UI redraws.
- Cap the number of rays and points.
- Keep UI polling slower than the render loop.
- Avoid per-frame allocations in the future planner where practical.
- Profile on a busy map with traffic before raising scan density.
- Keep visualization optional so the driving stack can use sensor data without drawing it.

## Tuning order

When profiling, change one setting at a time. First measure the baseline with the viewer closed, then with the viewer open, then with traffic enabled. Increase horizontal resolution only if the planner demonstrably needs more samples.
