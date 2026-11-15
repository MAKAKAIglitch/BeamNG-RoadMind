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

### In-game settings

Open **ESC → Mods → RoadMind** (BeamNG.drive 0.39+). The bilingual settings page follows the game's language and includes:

- Economy, Balanced and High performance profiles.
- LiDAR enable, resolution, frequency and range.
- Radar and ultrasonic enable, ray density and frequency.
- Independent switches for 3D sensor overlays.
- A live estimate of scheduled scene-raycast queries per second.

Settings are saved to the BeamNG user settings folder and applied while scanning is active. Scanning is stopped by default. Press **Start scanning** in the page, or use the existing **RoadMind: Toggle Sensor Suite** Controls binding as a quick toggle.

The 3D beams and returns are drawn directly into the game world; there is no separate HUD app.

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

- Economy: about 26 scene-raycast queries/s.
- Economy (default): about 26 queries/s.
- Balanced: about 74 queries/s.
- High: about 310 queries/s.
- Individual sensors can be disabled; the 3D overlays can also be hidden. By default, only the LiDAR overlay is drawn.
- Camera frustums are geometry only; no image rendering.
- Camera/localization data updates at the selected pose rate.
- Scanning stays off until explicitly started.
- Raycasts run only when a sensor scan is due, not in the render loop. Debug drawing redraws cached results.

The figures are calculated from ray counts and scan frequencies. They are not measured FPS or CPU percentages. Actual cost depends on the map, vehicle, traffic, hardware and game version.

## Documentation

- [Full sensor inventory](docs/SENSOR_SUITE.md)
- [Roadmap](ROADMAP.md)
- [Architecture](docs/ARCHITECTURE.md)
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
