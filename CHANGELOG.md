# Changelog

## 0.2.0 — World-space LiDAR visualization

- Replaced the separate HTML/Canvas viewer with in-world BeamNG debug drawing.
- Added a regular Controls action: RoadMind: Toggle 3D LiDAR.
- Added 3D beams from the virtual sensor origin to valid ray hits.
- Added hit-point spheres and a sensor-origin marker.
- Made scanning start only while visualization is enabled.
- Kept raycasts rate-limited to 4 Hz and capped at 51 rays per scan.
- Added a full project roadmap and updated architecture/performance docs.

The autonomous route planner and vehicle controller are not implemented yet.
