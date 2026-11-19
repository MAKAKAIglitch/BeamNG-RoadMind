# RoadMind settings

Open the three in-game pages under **ESC → Mods** in BeamNG.drive 0.39+:

- **RoadMind** — sensor acquisition, calibration, 3D overlays and performance.
- **RoadMind ADAS** — driver-assistance switches, ADAS status, route pilot and manual takeover.
- **RoadMind Perception** — lightweight top-down/rear-follow-style 2D view of the cached world model.

The pages read the selected game language and display Russian or English. Changes are saved to `/settings/roadmind.json` and applied while the relevant feature is active.

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
- **Detailed Lua profiler:** disabled by default. When enabled, records RoadMind Vehicle Lua sensor-job time and Game Engine Lua processing, tracking, World Model, planner, NavGraph, ADAS, visualization, Debug Draw and total update times. The panel shows average/peak/last milliseconds, calls per second and GE Lua heap size over a rolling 15-second window. It stores no files and reports RoadMind timings only, not whole-game CPU or GPU usage.
- **Asynchronous work:** Vehicle Lua processes at most one of LiDAR, radar, ultrasonic, pose or detection-zone jobs per graphics frame. Simultaneously due jobs are spread across successive frames.
- The FPS target is an optimization goal, not a frame-rate lock or guarantee. With minimum sensor settings active, RoadMind cannot force BeamNG.drive to meet a target it cannot sustain.

## Individual sensor settings

- **LiDAR:** enable/disable, profile-controlled density and frequency (1/2/4 Hz). Range and horizontal sample count are kept inside the performance profiles rather than exposed as manual sliders.
- **Radar:** enable/disable, 1/2 Hz, low (one ray per radar), medium (up to two per radar) or high (all configured rays).
- **Ultrasonic:** enable/disable, 1/2 Hz, single-ray or three-ray beam.
- **Camera vehicle detection:** enabled by default; includes front, side and rear camera frustums and bounded scene-occlusion checks. It works independently of LiDAR but is a simulator-assisted OBB visibility model, not RGB image recognition.
- **Point-cloud accumulation:** optional session-only world-space map, with a configurable 500–10,000 point cap (500-point steps), including hits on other vehicles; clear it at any time.
- **Visualization inertia:** an independent switch for eligible world-space LiDAR returns and route/planner overlays. Vehicle-mounted origins and geometry, traffic OBBs, and tracked LiDAR/radar/ultrasonic points on moving cars remain body-locked.
- **3D display:** independently show or hide LiDAR, radar, ultrasonic, detected vehicle OBBs, camera frustums and IMU/GNSS markers. The last valid overlays remain frozen while the simulation is paused.

Hiding a 3D overlay does not stop its sensor scans. Turn off the sensor group itself to reduce raycast work. Camera frustums and localization sampling can be disabled independently.

## ADAS driver assistance

Open **ESC → Mods → RoadMind ADAS**. ADAS is **off by default**. Enable the master switch to expose the individual feature switches. Every feature can be disabled independently; changes are saved in `/settings/roadmind.json`. The page also hosts the route-point editor and route-pilot controls.

| Setting | Behavior | Requirements / limits |
|---|---|---|
| Forward collision warning (FCW) | Warns when a fresh forward obstacle measurement enters the speed-dependent warning distance. | LiDAR, radar or ultrasonic must be enabled and delivering fresh data. |
| Automatic emergency braking (AEB) | Applies brake input during manual driving when a forward obstacle enters the estimated stopping threshold. | Fresh forward sensor data; while the RoadMind route controller is active, its safety supervisor handles the stop instead of a competing brake input. |
| Adaptive cruise control (ACC) | Reduces the route controller's target speed to maintain a time gap behind a tracked lead vehicle. | Currently works only while the RoadMind route controller is running; it is not manual-driving cruise control. |
| Blind-spot monitoring (BSM) | Flags recent tracked vehicles near the left or right rear quarter. | Requires current vehicle-registry observations and valid tracks; detection is simulator-assisted. |
| Parking distance warning | Displays the nearest fresh ultrasonic return. | Requires ultrasonic sensors to be enabled. It is a distance display, not automatic parking or automatic braking. |
| Lane departure warning (LDW) | Warns when lateral offset from the current road graph centreline becomes large. | Requires medium/high NavGraph localization. It does not detect painted lane markings. |

The ADAS status panel reports whether measurements are fresh and shows active warnings. A missing or stale required sensor stream puts the relevant function into a waiting state. Thresholds are initial engineering heuristics, not calibrated safety distances or a collision guarantee.

## Experimental road route pilot

The route controls are in **RoadMind ADAS**. Enter 1–12 ordered world-coordinate points in metres, one per line as `X, Y, Z`. The route builder snaps each point to a nearby road segment and shows route progress. RoadMind's own Vehicle Lua controller tracks the selected safe local trajectory and controls steering, throttle and brake, with a conservative 8 m/s (28.8 km/h) speed cap. Manual takeover is enabled by default and returns control when a steering, throttle or brake input is detected.

When the pilot starts, RoadMind switches to the Balanced sensor profile. Enabled sensor streams are checked for freshness; recent confirmed vehicle OBB footprints and scene-raycast obstacle clusters feed the bounded local planner. The controller stops with full brake if the planner cannot approve a path, the route is lost, vehicle state is unavailable or the trajectory feed is stale for 0.75 s. This is an experimental first controller: NavGraph road width is still a heuristic, and behavior on traffic, intersections, different vehicles and different road surfaces requires in-game validation. Test in a controlled, clear area.

## Defaults

RoadMind starts with scanning off. When started, the default Economy profile runs only the forward LiDAR (about 9 scene queries/s). Radar and ultrasonic are disabled until enabled manually or by switching to Balanced/High. Only the LiDAR overlay is visible initially; radar, ultrasonic, camera and localization overlays are hidden to keep debug drawing light.


## RoadMind Perception

Open **ESC → Mods → RoadMind Perception**. The page displays the current cached World Model in a lightweight dark 2D view, with a top-down mode and a rear-follow-style projection. It reads tracked vehicle OBBs, raycast obstacle clusters, the NavGraph road centreline and the active route. It does not run scene queries or render a live RGB camera feed. The NavGraph line is not lane marking.
