# BeamNG-RoadMind

**RoadMind is a sensor-grounded environment model and compact 3D driving map for BeamNG.drive.**

The project reconstructs a useful local view of the road and nearby traffic from RoadMind sensor observations, vehicle motion, and explicitly identified simulator/map metadata. Its central deliverable in this development branch is a reliable sensor foundation, a time-consistent World Model, and RoadMind Map. Full autonomous vehicle control is a later project stage, not the current objective.

## Source of truth

The detailed branch specification is **ROADMAP_BRANCH_TESLA_MAP_STYLE.md**. It defines the intended sensor roles, environment reconstruction, lane-aware road structure, map appearance, data provenance, performance principles, and acceptance criteria.

The technical documents in **docs/** explain how those requirements map to the system architecture. When a technical detail is not available in regular BeamNG.drive, it must be labelled as unavailable, estimated, simulator-assisted, or future work. Do not silently replace a missing sensor measurement with omniscient simulator data.

## What RoadMind is building

1. **Sensor foundation.** Preserve the configured sensor suite and its individual responsibilities. Validate physical placement, orientation, vehicle-specific calibration, range, coverage, occlusion and update cadence.
2. **World Model.** Transform timestamped observations into a common local/world coordinate frame. Preserve the source and freshness of each observation, fuse compatible measurements, track dynamic objects separately, and expire stale information.
3. **Road understanding.** Build a compact road and lane-level representation from supported geometry and metadata. A road centreline is not a lane boundary; unknown lane structure stays unknown.
4. **RoadMind Map.** Display the reconstructed local environment as a dark, restrained 3D map that follows the player's car. Use low-poly vehicle models, show only supported road structure and render the active calculated route/trajectory in blue.
5. **Performance and validation.** Keep acquisition, fusion, planning and rendering on separate bounded update paths. Validate behavior in repeatable in-game scenarios and report measured performance separately from theoretical query budgets.

## Sensor inventory and roles

| Source | Configured layout | Primary responsibility |
|---|---:|---|
| Forward LiDAR | 1 | Front-facing geometric surface measurements and local 3D reconstruction |
| RADAR | 5 | Independent target/range and relative-motion channel, within the limits of the backend |
| Ultrasonic | 12 | Very short-range surface distance and parking proximity |
| Cameras | 7 | Surrounding coverage; semantic perception when a real image/semantic backend exists |
| IMU-like motion data | 1 | Ego-motion and orientation-change support |
| GNSS-like position | 1 | Position in the BeamNG world/map coordinate frame |
| Wheel odometry | 1 data source | Ego-motion and distance estimation |

The LiDAR remains **front-facing**; it is not a 360-degree scanner. The combined camera layout provides surrounding coverage, but regular BeamNG.drive frustums and OBB visibility checks are not RGB image recognition.

## BeamNG simulation boundaries

The current regular BeamNG.drive implementation uses geometric scene-raycast proxies for LiDAR, RADAR and ultrasonic measurements. These are not complete physical laser, RF/FMCW or acoustic propagation models. Vehicle OBB registry data and simulator velocity can assist candidate discovery and tracking, but such observations must retain explicit simulator-assisted provenance.

Camera mounts and fields of view are geometric coverage models in the regular Drive build. They do not provide the rendered RGB/depth/semantic frames available through the separate BeamNG.tech sensor stack. IMU-like and GNSS-like outputs derived from Drive state are simulator-derived; world coordinates must not be presented as real latitude/longitude.

BeamNG.tech may be supported later through a separate sensor adapter. Its native readings must enter the same observation and World Model interfaces without being confused with Drive proxy data.

## Map design target

- One compact, rounded rectangular map with a dark visual style.
- A rear-following elevated 3D perspective; the map follows and rotates with the player's vehicle.
- Automatic framing/zoom based on speed, nearby road geometry and important upcoming route sections.
- A simple low-poly model of the player's current vehicle and low-poly miniatures of other tracked vehicles; do not use plain hitboxes as the final visual representation.
- Road surface, supported boundaries, lane structure, intersections, confirmed obstacles and useful POIs.
- A blue line for the calculated route or active planned trajectory.
- No speed, gear, sensor diagnostic, telemetry or ADAS status panels inside the map. Sensor beams and raw diagnostic overlays remain separate tools.
- No separate RoadMind Vision application is part of this branch's target.

The map must show what RoadMind has observed or what a clearly identified structural prior supports. It must not become a hidden rendering of the complete game scene.

## Development scope

This branch prioritizes sensors → World Model → road/lane representation → RoadMind Map → validation and performance. Route and trajectory data may be prepared for the map and future planning. The existing experimental route/controller code is not evidence of a validated autopilot and is not the focus of this branch.

Full autonomous steering, throttle and braking, lane changes, overtaking and expanded ADAS behavior are deferred until the perception and map foundation is reliable.

## Documentation

- [Branch specification](ROADMAP_BRANCH_TESLA_MAP_STYLE.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Sensor suite](docs/SENSOR_SUITE.md)
- [World Model](docs/WORLD_MODEL.md)
- [Performance](docs/PERFORMANCE.md)
- [Settings and runtime controls](docs/SETTINGS.md)

## Development status

RoadMind contains an experimental BeamNG.drive sensor and tracking prototype. The presence of code does not mean that its behavior is validated. Every capability must be described as one of:

- **Implemented, unvalidated** — code exists, but repeatable in-game tests have not passed.
- **Validated** — defined test scenarios pass on documented vehicles/maps/game versions.
- **Planned** — requirements exist, but implementation is incomplete or absent.
- **Unavailable in the current backend** — the regular Drive API does not provide the required native measurement.

Do not describe the project as production-ready autonomous driving software.
