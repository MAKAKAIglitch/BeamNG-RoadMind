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

## Scheduling

- Sensor scan frequencies are controlled by the selected profile and can be customized.
- Camera frustums, IMU, GNSS-like position and odometry use the selected 2–5 Hz pose rate when enabled.
- The complete suite is disabled until the player activates it from the settings page or Controls binding.
- Sensor data is cached between updates.
- Game Engine scene raycasts run only on incoming sensor-scan updates; the render loop redraws cached data and performs no raycasts.

## Rendering budget

A maximum of 51 LiDAR hits, 17 radar ray directions and 36 ultrasonic ray directions are processed by the sensor backend. The number of visible return markers is bounded by the number of valid hits. Seven camera frustums add a fixed number of debug lines.

## Profiling procedure

1. Record a baseline with RoadMind disabled.
2. Compare Economy (LiDAR only), Balanced (all three scan groups), and High (all groups at maximum configured density). Turn overlays on separately when measuring rendering cost.
3. Compare the same vehicle, map, traffic and time of day.
4. Record CPU use, average frame time and low-percentile frame rate.
5. Change only one scan rate or ray-count setting at a time.

Actual CPU cost depends on map complexity, vehicle geometry, traffic and game version. No fixed CPU percentage is claimed until measured in BeamNG.
