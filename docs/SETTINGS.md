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

## Experimental performance controls

- **Automatic sensor quality:** optional FPS feedback controller with a 30–165 FPS target (5 FPS steps). It lowers ray count, sensor resolution and scan frequency when the smoothed FPS remains below target; it restores detail more slowly when FPS has headroom.
- **Speed-adaptive sensor cadence:** reduces sensor scan frequency below 5 m/s and raises it above 25 m/s. Traffic OBB position refresh independently adapts between 0.30 s, 0.15 s and 0.10 s based on player speed; it does not add scene raycasts.
- **Asynchronous work:** Vehicle Lua processes at most one of LiDAR, radar, ultrasonic, pose or detection-zone jobs per graphics frame. Simultaneously due jobs are spread across successive frames.
- The FPS target is an optimization goal, not a frame-rate lock or guarantee. With minimum sensor settings active, RoadMind cannot force BeamNG.drive to meet a target it cannot sustain.

## Individual sensor settings

- **LiDAR:** enable/disable, profile-controlled density and frequency (1/2/4 Hz). Range and horizontal sample count are kept inside the performance profiles rather than exposed as manual sliders.
- **Radar:** enable/disable, 1/2 Hz, low (one ray per radar), medium (up to two per radar) or high (all configured rays).
- **Ultrasonic:** enable/disable, 1/2 Hz, single-ray or three-ray beam.
- **Point-cloud accumulation:** optional session-only world-space map, with a configurable 500–10,000 point cap (500-point steps), including hits on other vehicles; clear it at any time.
- **Visualization inertia:** an independent switch for damped movement of LiDAR endpoints and detected-vehicle outlines.
- **3D display:** independently show or hide LiDAR, radar, ultrasonic, detected vehicle OBBs, camera frustums and IMU/GNSS markers. The last valid overlays remain frozen while the simulation is paused.

Hiding a 3D overlay does not stop its sensor scans. Turn off the sensor group itself to reduce raycast work. Camera frustums and localization sampling can be disabled independently.

## Experimental road route pilot

The route section uses the current map's NavGraph. Enter 1–12 ordered world-coordinate points in metres, one per line as `X, Y, Z`. The route builder snaps each point to a nearby road segment and shows the final-node snap distance. The pilot sends NavGraph waypoint IDs to BeamNG's built-in Vehicle Lua AI, with a 29 km/h speed cap.

When the pilot starts, RoadMind switches to the Balanced sensor profile. Enabled sensor streams are checked for freshness before their returns enter the stop guard; recently confirmed nearby vehicles are checked by their projected OBB footprint. The guard is best-effort, not a collision guarantee. This is not a custom trajectory planner: RoadMind does not yet steer around obstacles, calculate lane boundaries or provide its own throttle/brake/steering controller. Test only in a controlled, clear area; route following and map compatibility remain experimental.

## Defaults

RoadMind starts with scanning off. When started, the default Economy profile runs only the forward LiDAR (about 9 scene queries/s). Radar and ultrasonic are disabled until enabled manually or by switching to Balanced/High. Only the LiDAR overlay is visible initially; radar, ultrasonic, camera and localization overlays are hidden to keep debug drawing light.
