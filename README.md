# BeamNG-RoadMind

**Lightweight autonomous-driving research mod for BeamNG.drive.**

RoadMind is being built around a forward-facing automotive sensor model, world-space LiDAR visualization and a custom driving stack. The goal is a vehicle that can follow route points, understand nearby obstacles and choose its own safe trajectory without requiring a permanent neural-network runtime.

> **Development status: early prototype.** The current build provides a forward virtual LiDAR sensor and an in-world 3D ray/point visualizer activated by a configurable key binding. Autonomous steering, throttle and braking are not implemented yet.

## Current prototype

- Forward virtual LiDAR with **120° horizontal FOV** and 16° vertical FOV.
- 17 × 3 ray layout: up to **51 rays per scan**.
- Default scan frequency: **4 Hz**, range: **60 m**.
- Static scenery and dynamic-object raycasts.
- Sensor origin positioned virtually near the front of the active vehicle.
- Each valid return is drawn directly in the 3D game world:
  - a line from the sensor origin to the hit;
  - a small sphere at the hit position;
  - a small cyan marker at the virtual sensor origin.
- No separate UI app, HTML canvas, external Python process or neural-network runtime.
- Sensor scanning runs only while the 3D visualization toggle is enabled.

## Installation and key binding

1. Download the repository as a ZIP.
2. Put the ZIP in your BeamNG.drive user folder's `mods` directory.
3. Start BeamNG.drive and open **Options → Controls**.
4. Search for **RoadMind: Toggle 3D LiDAR** and assign a keyboard key or controller button.
5. Enter a vehicle and press the assigned binding. Press it again to hide the visualization and stop scanning.

The binding is a regular BeamNG action, so it is available without opening or keeping any UI app on screen.

## Performance design

- Maximum of 51 ray directions per scan.
- Sensor scans are rate-limited to 4 Hz; no catch-up bursts.
- The sensor is off until the visualization binding is enabled.
- Debug drawing reuses the latest scan; it does not perform new raycasts each render frame.
- Only valid hit beams and hit points are drawn, rather than a dense empty cone.
- The visualization uses the game's world debug drawer, not a continuously polling HTML canvas.

These are workload limits, not a measured CPU guarantee. Actual cost depends on the map, vehicle, game version and scene complexity.

See [ROADMAP.md](ROADMAP.md), [Architecture](docs/ARCHITECTURE.md) and [Performance notes](docs/PERFORMANCE.md).

## Roadmap

See [ROADMAP.md](ROADMAP.md) for the full development plan and acceptance criteria.

## Design principles

- **Lua-first:** keep real-time sensor and driving logic inside BeamNG.
- **Low overhead:** rate-limit scans, cap ray counts and avoid per-frame raycasts.
- **World-space first:** sensor data should be inspectable directly in the game world.
- **Modular:** sensors, route planning, vehicle control and debugging are independent components.
- **Explainable:** start with deterministic planning and conventional control algorithms before considering machine learning.
- **Vehicle-agnostic:** do not require editing every vehicle's JBeam files.

## License

MIT. See [LICENSE](LICENSE).
