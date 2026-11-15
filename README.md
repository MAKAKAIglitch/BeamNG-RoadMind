# BeamNG-RoadMind

**A lightweight autonomous-driving research mod for BeamNG.drive.**

RoadMind is being built around a forward-facing automotive sensor model, a visible point-cloud viewer and a custom driving stack. The goal is a vehicle that can follow route points, understand nearby obstacles and choose its own safe trajectory — without relying on a neural-network runtime.

> **Development status: early prototype.** The first milestone adds the forward LiDAR sensor pipeline and its in-game viewer. The custom route planner and autonomous vehicle controller are planned next; they are not implemented yet.

## Current prototype

- Forward-facing virtual LiDAR: **120° horizontal field of view**, not a 360° scanner.
- 16° vertical field of view.
- 17 × 3 ray layout: up to **51 points per scan**.
- Default scan rate: **4 Hz**.
- Static scenery and dynamic-object raycasts.
- Sensor points are displayed in the **RoadMind Sensor Viewer** UI app as a top-down view.
- Sensor data is kept in vehicle-local coordinates; height (z) is retained for future 3D visualization and filtering.
- No external Python process, neural-network runtime or additional library is required by this prototype.

The sensor is virtual and currently has an invisible mounting origin near the front of the vehicle. A physical sensor mesh is not required.

## Installation

1. Download the repository as a ZIP.
2. Put the ZIP in your BeamNG.drive user folder's mods directory.
3. Start BeamNG.drive and open the UI Apps selector.
4. Add **RoadMind - Sensor Viewer**.
5. Use **Sensor ON/OFF** to control scanning and **Points ON/OFF** to hide or show the point cloud.

The viewer starts the RoadMind game-engine extension and attaches the sensor extension to the currently driven vehicle.

## Performance approach

RoadMind deliberately starts with a small ray budget and a low scan frequency. Sensor acquisition and visualization are separate: closing or hiding the point display does not need to change the sensor API used by future driving logic.

Default budget: 51 ray directions × 4 scans per second. Each direction checks static geometry and, when needed, dynamic objects. Actual CPU cost depends on the map, vehicle, game version and scene complexity; it must be measured in-game before increasing resolution.

See docs/PERFORMANCE.md and docs/ARCHITECTURE.md.

## Roadmap

- [x] Initial forward LiDAR scan and point data pipeline
- [x] UI point-cloud viewer
- [x] Low-cost scan limits and sensor toggle
- [ ] Sensor calibration and visualization validation across vehicle classes
- [ ] Road graph route input and ordered destination points
- [ ] Local trajectory planner with obstacle avoidance
- [ ] Steering and speed controllers with safe stopping behavior
- [ ] Optional lane and traffic-signal perception
- [ ] Replayable test scenarios and profiling tools

## Design principles

- **Lua-first:** keep the real-time sensor and driving logic inside BeamNG.
- **Low overhead:** rate-limit scans, cap ray counts and avoid per-frame point-cloud rebuilding.
- **Modular:** sensors, route planning, vehicle control and visualization are independent components.
- **Explainable:** prefer a deterministic trajectory planner and conventional control algorithms before considering machine learning.
- **Vehicle-agnostic:** do not require editing every vehicle's JBeam files.

## License

MIT. See LICENSE.
