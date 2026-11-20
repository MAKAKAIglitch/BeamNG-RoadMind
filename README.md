# BeamNG-RoadMind

**A lightweight virtual automotive sensor suite and autonomous-driving research mod for BeamNG.drive.**

RoadMind models the sensor layout of a modern assisted/autonomous vehicle and visualizes its coverage directly in the game world. The long-term goal is a custom driving stack that plans routes, avoids obstacles and controls the vehicle without relying on a permanent neural-network runtime.

> **Development status: experimental perception and autonomy prototype.** The current build includes virtual LiDAR/radar/ultrasonic raycast proxies, vehicle OBB tracking, a timestamped world model, NavGraph routing, a bounded local trajectory planner and the first native Lua controller for steering, throttle and braking. Sensor quality, controller behavior, traffic interaction and performance still need repeatable in-game testing. Regular BeamNG.drive Lua does not expose BeamNG.tech's rendered RGB camera stream or a physical RF radar simulation.

## Sensor suite

| System | Count | Current model |
|---|---:|---|
| Forward LiDAR | 1 | 120° forward ray fan, 51 directions, 65 m |
| Automotive radar | 5 | 1 front long-range + 4 corner radars; 3–5 rays each |
| Ultrasonic | 12 | Front, rear and corner sensors; 3-ray cone approximation |
| Cameras | 7 | Front narrow/wide, side, rear wide and rear-corner coverage frustums |
| IMU | 1 | Chassis acceleration signals and estimated yaw rate |
| GNSS-like localization | 1 | Vehicle world position and altitude; no latitude/longitude reference |
| Wheel odometry | 1 data source | BeamNG wheel speed and odometer signals |

### In-game settings

Open **ESC → Mods → RoadMind** (BeamNG.drive 0.39+). The bilingual settings page follows the game's language and includes:

- Economy, Balanced and High performance profiles.
- An optional in-game Lua profiler with per-sensor, tracking, World Model, planner, ADAS and Debug Draw timings (off by default).
- LiDAR enable, profile-controlled resolution, frequency and range.
- Radar and ultrasonic enable, ray density and frequency.
- Independent switches for 3D sensor overlays.
- A dedicated ADAS panel with a master switch and separate FCW, AEB, ACC, blind-spot, parking-distance and lane-departure toggles.
- Live ADAS state and warning indicators, with missing/stale sensor data reported explicitly.
- OBB-based vehicle geometry status, manual recalibration and an optional bounds/axes/sensor-mount overlay.
- A tracked-object list showing RoadMind IDs, track state, estimated speed, relative speed, data age, confidence and sensor-zone sources.
- Green/yellow oriented vehicle-box outlines for visible and not-yet-verified vehicle candidates.
- A session-only accumulated LiDAR map with a 500–10,000 point limit, vehicle-surface returns included, a clear button and pause-safe display.
- A 3D RoadMind Perception view that triangulates nearby LiDAR points into a bounded spatial mesh, with a rear-follow camera, elevated top view, NavGraph road surface, tracked vehicle geometry and route overlay. The mesh is display-only and does not feed sensor fusion or planning.
- Optional critically damped inertia for eligible world-space LiDAR returns and route/planner overlays; vehicle-mounted geometry and tracked vehicle-surface points stay body-locked.
- A live estimate of scheduled scene-raycast queries per second.
- Optional adaptive sensor quality with a desired FPS setting, plus speed-adaptive scan intervals.
- Asynchronous round-robin sensor scheduling (at most one due sensor/pose job per graphics frame) and 3.3–10 Hz traffic OBB refresh without extra sensor raycasts.

Settings are saved to the BeamNG user settings folder and applied while scanning is active. Scanning is stopped by default. Press **Start scanning** in the page, or use the existing **RoadMind: Toggle Sensor Suite** Controls binding as a quick toggle.

The 3D beams and returns are drawn directly into the game world; there is no separate HUD app.

### Experimental road route pilot

The Mods page includes an experimental NavGraph route pilot. It accepts up to 12 ordered world-coordinate points (`X, Y, Z` in metres), builds a connected path on the current map and reports route progress and road localization. The pilot switches to the Balanced sensor profile; enabled, fresh sensor streams and confirmed vehicle OBB footprints feed a best-effort safety supervisor. A bounded local planner evaluates five candidate trajectories against obstacle and vehicle footprints.

### ADAS driver assistance

ADAS is off by default and is configured independently in **ESC → Mods → RoadMind**. The master switch and each feature can be toggled separately:

- **FCW:** warns about forward obstacles using fresh enabled sensor returns.
- **AEB:** can apply brake input during manual driving. During an active RoadMind route it extends the route supervisor's stopping threshold instead of competing with the route controller.
- **ACC:** adjusts the target speed to keep a gap behind a tracked lead vehicle, currently only while the RoadMind route controller is running.
- **BSM:** reports confirmed/coasting tracked vehicles in the left and right blind-spot zones.
- **Parking distance:** displays the nearest fresh ultrasonic return; ultrasonic sensors must be enabled.
- **LDW:** warns when the vehicle moves laterally away from the current NavGraph road centreline. It does not detect painted lane markings.

FCW/AEB depend on fresh forward sensor data; BSM depends on recent vehicle tracking; LDW depends on reliable NavGraph localization. ADAS is an experimental assistance layer, not a collision guarantee. Its thresholds and vehicle tracking still need in-game validation.

Vehicle-mounted sensor origins, mounts, frustums, calibration geometry and detection zones are re-anchored to the live vehicle pose for display. Dynamic vehicle OBBs and LiDAR/radar/ultrasonic returns that are associated with a tracked moving car are reconstructed from its current predicted pose and local body coordinates; stale attached returns fade/disappear rather than remaining behind. Static-scene returns stay in world coordinates. Rendering uses copied data and does not rewrite the raw sensor observations or World Model.

The first native RoadMind Vehicle Lua controller now tracks the selected safe trajectory with Pure Pursuit-style lateral control and a bounded PI speed controller. It sends steering, throttle and brake inputs directly through BeamNG's Vehicle Lua input system; it no longer delegates route following to the built-in Vehicle AI. The controller runs at 20 Hz, accepts short planner paths (up to 32 points), and latches a full-brake stop when its path feed becomes stale or route tracking is lost. Target speed is capped at 8 m/s (28.8 km/h). The road corridor and turning limits remain conservative NavGraph heuristics, not detected lane boundaries or collision guarantees. This is an initial control foundation, not a validated production autopilot: steering polarity, vehicle-class handling, braking, intersections, traffic behavior and recovery must be tested in-game before relying on it.

## Important simulation boundaries

This project targets **BeamNG.drive**. BeamNG.tech has a separate native sensor stack with rendered RGB/depth cameras, LiDAR, RADAR, ultrasonic, GPS and IMU APIs. The regular BeamNG.drive mod API does not expose the same complete native suite.

Therefore:

- LiDAR uses BeamNG's Game Engine scene-raycast API and produces a sparse forward point cloud.
- Radar uses Game Engine scene-raycast returns. It is a geometric radar proxy, not an RF/FMCW waveform simulation. Vehicle tracks and relative-speed estimates are simulator-assisted and timestamp-derived; they are not physical Doppler measurements.
- Ultrasonic uses three short-range scene-raycasts per sensor and keeps the nearest hit. It approximates the sensor beam, not acoustic propagation or material-dependent reflections.
- Camera cones show mounting positions and fields of view. They do not produce RGB frames or semantic object detections in this Drive build.
- IMU acceleration comes from BeamNG vehicle electrics; yaw rate is estimated from heading changes.
- GNSS-like localization reports BeamNG world coordinates. It is not geodetic latitude/longitude.
- Odometry reads the vehicle wheel-speed and odometer electrics where present.
- Vehicle candidates are found from a shared Game Engine vehicle OBB registry (0.30–0.10 s speed-adaptive refresh) and tested against fresh calibrated sensor zones. A maximum of two occlusion raycasts are used per detection cycle (up to 20 additional checks/s at 10 Hz). A session-scoped RoadMind track ID associates repeated observations; speed is estimated from timestamped positions, while simulator velocity is kept as a separate field. Tracks coast briefly through occlusion and expire after bounded time/missed snapshots. These observations are simulator-assisted registry results, not raw physical-style sensor measurements.

These boundaries are kept explicit so a simulated geometric measurement is not mistaken for a full physical sensor model.

## Performance

- Economy (default): about 9 scene-raycast queries/s (LiDAR only; radar and ultrasonic are off).
- Balanced: about 74 queries/s.
- High: about 310 queries/s.
- Individual sensors can be disabled; the 3D overlays can also be hidden. By default, only the LiDAR overlay is drawn.
- Camera frustums are geometry only; no image rendering.
- Camera/localization data updates at the selected pose rate.
- Scanning stays off until explicitly started.
- Raycasts run only when a sensor scan is due, not in the render loop. Debug drawing redraws cached results.
- LiDAR render smoothing performs no new scene raycasts and never changes the raw measurement array used by sensor safety logic.
- Vehicle OBB dimensions are read during vehicle attach/manual recalibration, not once per ray or each scan. The calibration overlay is optional.

The figures are calculated from ray counts and scan frequencies. They are not measured FPS or CPU percentages. Actual cost depends on the map, vehicle, traffic, hardware and game version.

## Documentation

- [Full sensor inventory](docs/SENSOR_SUITE.md)
- [Roadmap](ROADMAP.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Unified world model schema](docs/WORLD_MODEL.md)
- [Performance notes](docs/PERFORMANCE.md)
- [Settings and profile details](docs/SETTINGS.md)

## Design principles

- **Drive-compatible:** no BeamNG.tech-only API dependencies in the base mod.
- **Honest sensor models:** simulated proxies are labelled as proxies.
- **Low overhead:** off-by-default scanning, configurable scan rates and ray density, and no raycasts in the render loop.
- **World-space first:** sensor mounts, camera fields of view, beams and returns are inspectable in-game.
- **Modular:** sensing, planning, vehicle control and visualization remain independent.
- **Vehicle-agnostic:** no per-vehicle JBeam edits are required for the virtual sensor layout.

## License

MIT. See [LICENSE](LICENSE).
