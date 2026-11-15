# RoadMind architecture

RoadMind is a layered sensor and driving stack for BeamNG.drive.

## Vehicle Lua sensor backend

Location: lua/vehicle/extensions/roadmind_sensors.lua

The Vehicle Lua extension owns vehicle-local sensor geometry and scan scheduling:

- Forward LiDAR: 51 rays, 4 Hz, 120° horizontal FOV, 65 m.
- Five radar ray fans: 17 directions total, 2 Hz.
- Twelve ultrasonic units: 36 short-range rays total, 2 Hz.
- Seven camera mount poses and FOV values, refreshed at 5 Hz.
- IMU-like acceleration and estimated yaw rate, refreshed at 5 Hz.
- GNSS-like world position and vehicle odometry signals, refreshed at 5 Hz.

The extension serializes only the rays for sensors whose scan interval has elapsed. It does not call the Game Engine-only `castRay` API.

## Game Engine Lua coordinator and renderer

Location: lua/ge/extensions/roadmind.lua

The Game Engine Lua extension:

- Attaches the sensor backend to the active player vehicle when the sensor-suite binding is enabled.
- Receives the latest serialized sensor frame and performs scheduled scene raycasts through `castRay`.
- Merges raycast hits into the latest sensor frame.
- Draws LiDAR returns, radar returns, ultrasonic close-range returns, camera frustums, IMU and GNSS mount markers using the game's world debug drawer.
- Redraws cached geometry each graphics frame because debug primitives are transient.

Scene raycasts run only on incoming scan updates, not on every render frame.

## Input

Location: lua/ge/extensions/core/input/actions/roadmind_actions.json

The regular Controls action **RoadMind: Toggle Sensor Suite** calls the Game Engine Lua coordinator. The user assigns a keyboard key or controller button in Options → Controls.

## Sensor model boundaries

The base mod targets BeamNG.drive. LiDAR, radar and ultrasonic use geometric raycasts; camera mounts display FOV frustums but do not return RGB images. IMU-like values use vehicle electrics and heading differences. GNSS-like positioning uses simulator world coordinates. Wheel speed and odometer come from vehicle electrics.

BeamNG.tech provides a separate native suite with rendered camera, LiDAR, RADAR, ultrasonic, GPS and IMU sensors. The planned native adapter can use those APIs without changing the upper-level route-planning interface.

## Planned autonomy stack

1. **Route manager:** road graph, route points and local road corridor.
2. **Object tracker:** radar returns and LiDAR points associated over time.
3. **Local planner:** candidate trajectory generation, collision checks and scoring.
4. **Vehicle controller:** steering, throttle, braking and gear management.
5. **Safety supervisor:** controlled stop on stale sensors, invalid route or blocked path.
