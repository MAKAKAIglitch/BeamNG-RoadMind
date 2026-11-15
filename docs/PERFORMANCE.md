# Performance notes

RoadMind is designed to minimize its load on BeamNG.drive.

## Current target budgets

| Subsystem | Workload | Update rate | Ray directions / second |
|---|---:|---:|---:|
| Forward LiDAR | 51 rays | 4 Hz | 204 |
| Five radars | 17 rays total | 2 Hz | 34 |
| Twelve ultrasonics | 36 rays total | 2 Hz | 72 |
| Cameras | 7 frustums | 5 Hz pose refresh | 0 |
| IMU / GNSS / odometry | vehicle-state sampling | 5 Hz | 0 |
| **Total** | | | **310** |

A ray direction may use one static and one dynamic query. The rough upper bound is therefore about 620 raycast calls per second. This is a workload estimate, not a measured CPU percentage.

## Scheduling

- LiDAR runs at 4 Hz.
- Radar runs at 2 Hz.
- Ultrasonic runs at 2 Hz.
- Camera poses, IMU, GNSS-like position and odometry refresh at 5 Hz.
- The complete suite is disabled until the player activates it.
- Sensor data is cached between updates.
- The Game Engine renderer redraws cached data but performs no raycasts.

## Rendering budget

A maximum of 51 LiDAR hits, 17 radar ray directions and 36 ultrasonic ray directions are processed by the sensor backend. The number of visible return markers is bounded by the number of valid hits. Seven camera frustums add a fixed number of debug lines.

## Profiling procedure

1. Record a baseline with RoadMind disabled.
2. Enable the suite but hide no sensor types (default full visualization).
3. Compare the same vehicle, map, traffic and time of day.
4. Record CPU use, average frame time and low-percentile frame rate.
5. Change only one scan rate or ray-count setting at a time.

Actual CPU cost depends on map complexity, vehicle geometry, traffic and game version. No fixed CPU percentage is claimed until measured in BeamNG.
