# RoadMind settings

Open the in-game page at **ESC → Mods → RoadMind** in BeamNG.drive 0.39+.

The page reads the selected game language and displays Russian or English. Changes are saved to `/settings/roadmind.json` and applied to the active vehicle while scanning is enabled.

## Performance profiles

| Profile | LiDAR | Radar | Ultrasonic | Estimated scene queries/s |
|---|---|---|---|---:|
| Economy (default) | 9 rays × 1 Hz | Disabled | Disabled | 9 |
| Balanced | 26 rays × 2 Hz | up to 2 rays per radar × 1 Hz | 1 ray per sensor × 1 Hz | 74 |
| High | 51 rays × 4 Hz | 17 rays × 2 Hz | 3 rays per sensor × 2 Hz | 310 |

These are calculated query budgets, not measured FPS or CPU percentages. Actual cost depends on the map, vehicle geometry, traffic and hardware.

## Individual sensor settings

- **LiDAR:** enable/disable, low (9×1), balanced (13×2) or high (17×3) resolution, 1/2/4 Hz, 30–160 m range.
- **Radar:** enable/disable, 1/2 Hz, low (one ray per radar), medium (up to two per radar) or high (all configured rays).
- **Ultrasonic:** enable/disable, 1/2 Hz, single-ray or three-ray beam.
- **3D display:** independently show or hide LiDAR, radar, ultrasonic, camera frustums and IMU/GNSS markers.

Hiding a 3D overlay does not stop its sensor scans. Turn off the sensor group itself to reduce raycast work. Camera frustums and localization sampling can be disabled independently.

## Defaults

RoadMind starts with scanning off. When started, the default Economy profile runs only the forward LiDAR (about 9 scene queries/s). Radar and ultrasonic are disabled until enabled manually or by switching to Balanced/High. Only the LiDAR overlay is visible initially; radar, ultrasonic, camera and localization overlays are hidden to keep debug drawing light.
