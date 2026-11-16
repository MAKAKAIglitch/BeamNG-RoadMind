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

## Vehicle registry and detection budget

- The shared GE vehicle OBB registry refreshes at 2 Hz (once per 0.5 s), rather than being rebuilt by individual sensors.
- Sensor-zone/OBB tests run at up to 4 Hz against the cached registry.
- At most 4 occlusion raycasts run per detection cycle: a hard upper bound of 16 additional scene-raycast queries/s. The actual count is lower when fewer than four candidates enter sensor zones.
- The settings page adds the actual recent occlusion-raycast rate to the scheduled sensor-raycast estimate. This is a workload estimate, not an FPS or CPU measurement.

## Object tracking budget

- The target registry contains at most 256 valid vehicle OBB snapshots.
- Sensor-zone detection and tracker ageing run at 4 Hz (0.25 s cadence) against the cached registry and latest zone frame.
- The track store is capped at 128 objects. Association uses a source-ID index; the tracker does not perform scene raycasts.
- Timestamped velocity estimation runs only when a new registry observation arrives. Cached visibility and repeated frames do not count as new measurements.
- Tracks use a 0.75 s visibility-cache limit and a 1.5 s maximum observation age, with an additional missed-scan and registry-removal check.
- A coasting track's predicted display position is limited to a 0.5 s horizon; it is not fed back as a new measurement.

## Scheduling

- Sensor scan frequencies are controlled by the selected profile and can be customized.
- Camera frustums, IMU, GNSS-like position and odometry use the selected 2–5 Hz pose rate when enabled.
- The complete suite is disabled until the player activates it from the settings page or Controls binding.
- Sensor data is cached between updates.
- Game Engine scene raycasts run only on incoming sensor-scan updates; the render loop redraws cached data and performs no raycasts.

## Rendering budget

The High preset scans 51 LiDAR directions (17 × 3); the configurable slider allows up to 99 directions (33 × 3). Radar and ultrasonic limits remain 17 and 36 directions respectively.

The time-consistent LiDAR renderer is capped at 99 active points plus 198 fading points. It updates at most 297 render markers per graphics frame, with a 0.5 s maximum target-motion prediction horizon and a 0.75 m per-frame movement cap. The fading lifetime is 0.4 s and an unobserved render point expires after 2.25 s. This work performs no scene raycasts and does not alter raw sensor hits. Seven camera frustums add a fixed number of debug lines.

## Profiling procedure

1. Record a baseline with RoadMind disabled.
2. Compare Economy (LiDAR only), Balanced (all three scan groups), and High (all groups at maximum configured density). Turn overlays on separately when measuring rendering cost.
3. Compare the same vehicle, map, traffic and time of day.
4. Record CPU use, average frame time and low-percentile frame rate.
5. Change only one scan rate or ray-count setting at a time.

Actual CPU cost depends on map complexity, vehicle geometry, traffic and game version. No fixed CPU percentage is claimed until measured in BeamNG.
