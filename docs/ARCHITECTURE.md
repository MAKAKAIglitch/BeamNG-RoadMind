# RoadMind architecture

RoadMind is split into four independent layers. This keeps point rendering from becoming part of the vehicle-control loop.

## 1. Sensor layer

Location: lua/vehicle/extensions/roadmind_sensors.lua

The prototype uses one virtual sensor mounted near the front of the vehicle:

- Horizontal field of view: 120°.
- Vertical field of view: 16°.
- Ray layout: 17 azimuth samples × 3 elevation samples.
- Maximum range: 60 m.
- Default update rate: 4 scans per second.
- Maximum output: 51 hit points per scan.

Each point is converted into sensor-local coordinates:

- x: lateral offset, positive to the vehicle's right.
- y: forward distance.
- z: height relative to the sensor.
- distance: straight-line distance from the sensor origin.

The scan uses static raycasts and checks dynamic objects along the same ray. The closest valid hit is retained. The sensor origin is virtual; no JBeam part or visible sensor model is needed.

## 2. Game-engine bridge

Location: lua/ge/extensions/roadmind.lua

The game-engine extension watches for the active player vehicle, loads the vehicle sensor extension and receives serialized scan data. It exposes the latest sensor frame to the UI app.

Only the active player vehicle is attached by this prototype.

## 3. Visualization

Location: ui/modules/apps/roadmind/

The UI app polls the latest frame at 5 Hz and draws the points on a top-down sensor view. Visualization is not involved in the sensor raycast loop or future control calculations.

The point data retains height, so a later 3D view or height-based filtering can use it without changing the sensor interface.

## 4. Autonomous-driving stack — planned

The intended driving stack is:

1. **Route manager:** receives ordered destination points and obtains a road-following corridor from the map/road graph.
2. **Local planner:** generates a small set of candidate trajectories ahead of the car. It scores progress, clearance from LiDAR points, road boundaries, curvature and steering smoothness.
3. **Motion controller:** tracks the selected trajectory and regulates speed using vehicle speed, curvature and stopping distance.
4. **Safety supervisor:** brakes and disengages when sensor data is stale, no safe trajectory exists or the vehicle leaves the supported road corridor.

The first planner should be deterministic and computationally small: a bicycle-model trajectory sampler with collision scoring is a better starting point than running a neural network every frame. A learned perception model can remain an optional future experiment.

## Important distinction

The current prototype does **not** autonomously drive the vehicle. It only provides the first sensor and visualization milestone. Route following, obstacle avoidance and actuation must be implemented and tested as separate milestones.
