# BeamNG-RoadMind

**A lightweight virtual automotive sensor suite and autonomous-driving research mod for BeamNG.drive.**

RoadMind models the sensor layout of a modern assisted/autonomous vehicle and visualizes its coverage directly in the game world. The long-term goal is a custom driving stack that plans routes, avoids obstacles and controls the vehicle without relying on a permanent neural-network runtime.

> **Development status: sensor-suite prototype.** The current build contains virtual LiDAR, raycast-based radar and ultrasonic proxies, camera coverage frustums, and vehicle-state localization/odometry data. Autonomous steering, throttle and braking are not implemented yet. RGB camera capture and physical RF radar simulation are not provided by the regular BeamNG.drive Lua API.

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

### One-key visualization

1. Download the repository as a ZIP.
2. Put it in your BeamNG.drive user folder's `mods` directory.
3. Start BeamNG.drive and open **Options → Controls**.
4. Search for **RoadMind: Toggle Sensor Suite** and assign a keyboard key or controller button.
5. Enter a vehicle and press the binding. Press it again to hide the suite and stop sensor updates.

No separate UI app is required. Sensors are invisible virtual mounts; their 3D coverage, beams and returns are drawn directly into the game world.

## Important simulation boundaries

This project targets **BeamNG.drive**. BeamNG.tech has a separate native sensor stack with rendered RGB/depth cameras, LiDAR, RADAR, ultrasonic, GPS and IMU APIs. The regular BeamNG.drive mod API does not expose the same complete native suite.

Therefore:

- LiDAR uses BeamNG's Game Engine scene-raycast API and produces a sparse forward point cloud.
- Radar uses Game Engine scene-raycast returns. It is a geometric radar proxy, not an RF/FMCW waveform simulation; target radial velocity and stable target tracking are not implemented yet.
- Ultrasonic uses three short-range scene-raycasts per sensor and keeps the nearest hit. It approximates the sensor beam, not acoustic propagation or material-dependent reflections.
- Camera cones show mounting positions and fields of view. They do not produce RGB frames or semantic object detections in this Drive build.
- IMU acceleration comes from BeamNG vehicle electrics; yaw rate is estimated from heading changes.
- GNSS-like localization reports BeamNG world coordinates. It is not geodetic latitude/longitude.
- Odometry reads the vehicle wheel-speed and odometer electrics where present.

These boundaries are kept explicit so a simulated geometric measurement is not mistaken for a full physical sensor model.

## Performance

- LiDAR: 51 directions at 4 Hz.
- Radar: 17 directions total at 2 Hz.
- Ultrasonic: 36 short-range directions at 2 Hz.
- Camera frustums: geometry only; no image rendering.
- IMU/localization/odometry: sampled at 5 Hz.
- The whole suite stays off until the Controls binding is pressed.
- Raycasts run only when a sensor scan is due, not in the render loop. Debug drawing redraws cached results.

The target workload is 310 Game Engine scene-raycast queries per second. This is an estimate, not a measured CPU percentage. Actual cost depends on the map, vehicle, traffic, hardware and game version.

## Documentation

- [Full sensor inventory](docs/SENSOR_SUITE.md)
- [Roadmap](ROADMAP.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Performance notes](docs/PERFORMANCE.md)

## Design principles

- **Drive-compatible:** no BeamNG.tech-only API dependencies in the base mod.
- **Honest sensor models:** simulated proxies are labelled as proxies.
- **Low overhead:** fixed scan rates, hard ray budgets and no raycasts in the render loop.
- **World-space first:** sensor mounts, camera fields of view, beams and returns are inspectable in-game.
- **Modular:** sensing, planning, vehicle control and visualization remain independent.
- **Vehicle-agnostic:** no per-vehicle JBeam edits are required for the virtual sensor layout.

## License

MIT. See [LICENSE](LICENSE).
