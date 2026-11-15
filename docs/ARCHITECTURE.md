# RoadMind architecture

RoadMind is split into independent layers so sensor acquisition, world visualization, route planning and vehicle control do not depend on one another.

## 1. Sensor layer — Vehicle Lua

Location: lua/vehicle/extensions/roadmind_sensors.lua

The current prototype uses one virtual sensor mounted near the front of the active vehicle:

- Horizontal field of view: 120°.
- Vertical field of view: 16°.
- Ray pattern: 17 azimuth samples × 3 elevation samples.
- Maximum range: 60 m.
- Default scan rate: 4 scans per second.
- Maximum ray directions: 51 per scan.

For each ray, the sensor checks static geometry and dynamic objects, then keeps the closest hit. It sends the world-space endpoint of each ray to Game Engine Lua. Valid returns are also converted into sensor-local coordinates for future planning.

The sensor origin is virtual; no visible sensor mesh or JBeam edit is required.

## 2. Input binding

Location: lua/ge/extensions/core/input/actions/roadmind_actions.json

RoadMind contributes a regular BeamNG action named **RoadMind: Toggle 3D LiDAR**. It runs in Game Engine Lua and calls the RoadMind coordinator. The player assigns a keyboard key or controller button in Options → Controls.

No UI app is needed.

## 3. World visualization — Game Engine Lua

Location: lua/ge/extensions/roadmind.lua

The game-engine extension attaches the sensor extension to the active player vehicle when visualization is enabled. Vehicle Lua sends serialized scan frames to the coordinator.

The coordinator uses BeamNG's world debug drawer:

- a line from the virtual sensor origin to each valid hit;
- a small sphere at each hit position;
- a cyan marker at the sensor origin.

Debug primitives are transient, so the last received scan is redrawn each graphics frame while visualization is active. Raycasts themselves remain rate-limited to 4 Hz. No new raycasts are performed by the render loop.

Only valid hit beams are displayed; missed rays are not drawn to the 60 m range limit.

## 4. Local planner — planned

The local planner will consume sensor-local points and route-corridor information. It should produce a small set of candidate trajectories, reject collision paths and select the route-compatible candidate with the best clearance and smoothness score.

The initial planner should be deterministic and inexpensive. A lattice or bicycle-model trajectory sampler with collision checks is preferred over a neural network.

## 5. Vehicle controller — planned

The controller will track the selected path and manage steering, throttle, braking and gears. A safety supervisor will request a stop when sensor data is stale, route confidence is lost or every candidate path is blocked.

## Important distinction

The current prototype does **not** autonomously drive the vehicle. It provides sensor acquisition and the requested in-world visualization. Route planning and autonomous actuation are separate future milestones.
