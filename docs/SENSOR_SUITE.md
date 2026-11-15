# RoadMind virtual sensor suite

This document describes the current Drive-compatible sensor layout and the exact boundary between simulated measurements and native sensor APIs.

## Layout overview

| System | Quantity | Placement | Coverage / range | Update |
|---|---:|---|---|---:|
| Forward LiDAR | 1 | Front roof / windshield area | 120° × 16°, 65 m | 4 Hz |
| Front long-range radar | 1 | Front center | 28° horizontal, 160 m | 2 Hz |
| Corner radars | 4 | Front-left/right, rear-left/right | 58°, 50–65 m | 2 Hz |
| Front ultrasonic | 4 | Front bumper | 5 m, 3-ray beam | 2 Hz |
| Rear ultrasonic | 4 | Rear bumper | 5 m, 3-ray beam | 2 Hz |
| Corner ultrasonic | 4 | Vehicle corners | 5 m, 3-ray beam | 2 Hz |
| Cameras | 7 | Windshield, side mirrors, rear glass and rear corners | Individual frustums, 22–85 m visualization range | 5 Hz pose refresh |
| IMU | 1 | Near vehicle center | Acceleration signals + estimated yaw rate | 5 Hz |
| GNSS-like position | 1 | Roof | BeamNG world position and altitude | 5 Hz |
| Wheel odometry | 1 data source | Vehicle electrics | Wheel speed and odometer | 5 Hz sample |

## LiDAR

The forward LiDAR samples 17 horizontal azimuths and 3 vertical elevations (51 ray directions). Each ray is evaluated by the Game Engine scene-raycast API, which returns the closest scene hit or no return. Valid hits are rendered as a beam from the front sensor origin and a point at the hit.

The scanner is intentionally forward-facing, not a 360° roof scanner.

## Radar

Five radar units approximate a modern front-and-corner layout:

- Front long-range unit for forward traffic and obstacles.
- Front-left and front-right corner units.
- Rear-left and rear-right corner units.

Each radar samples a small horizontal ray fan. Returns contain range and world hit position. Stable target IDs and radial velocity are not extracted yet.

This is a kinematic raycast proxy. It does not simulate RF propagation, frequency-modulated chirps, antenna arrays, RCS, multipath or Doppler processing.

## Ultrasonic

Twelve short-range units cover the front, rear and corners. Each sensor uses three Game Engine scene-raycasts around its center direction and returns the nearest hit up to 5 m. This creates a compact cone approximation suitable for visual parking/close-obstacle feedback.

The model does not simulate acoustic wave propagation, surface absorption or material-dependent sensitivity.

## Cameras

Seven camera mounts show the intended view coverage:

1. Front narrow
2. Front wide
3. Left side
4. Right side
5. Rear wide
6. Rear left
7. Rear right

Each mount has its own 3D frustum using its position, orientation, horizontal FOV, vertical FOV and visualization distance.

In regular BeamNG.drive, this prototype does not read a camera image stream or perform image recognition. Native rendered camera/depth/semantic sensor access belongs to the BeamNG.tech sensor system.

## Localization and motion

### IMU-like motion data

- Acceleration: BeamNG electrics `accXSmooth`, `accYSmooth`, `accZSmooth`.
- Yaw rate: estimated from the change in vehicle forward direction over the sample interval.
- Mount: near the vehicle center.

The values are suitable for prototyping and visualization; they are not a calibrated six-axis IMU model with bias, noise, temperature drift or sensor fusion.

### GNSS-like position

- Position: `obj:getPosition()` in BeamNG world coordinates.
- Altitude: world Z coordinate.
- Mount: virtual roof antenna point.

There is no latitude/longitude conversion because the mod has no geodetic reference for the current map.

### Wheel odometry

Uses BeamNG vehicle electrics where available:

- `wheelspeed` — wheel-measured vehicle speed in m/s.
- `odometer` — vehicle odometer in metres.
- `airspeed` — vehicle speed relative to the world.

These are simulator vehicle signals, not a per-wheel encoder model.

## Visualization legend

- Cyan/blue: forward LiDAR and sensor origin.
- Green/yellow: radar units and returns.
- Turquoise/yellow: ultrasonic units and close returns.
- Blue: camera frustums.
- Magenta: IMU mount.
- Green: GNSS-like antenna position.

## BeamNG.tech native upgrade path

When a BeamNG.tech installation is available, the sensor adapter can be replaced or extended to create native camera, LiDAR, RADAR, ultrasonic, GPS and IMU sensors through `extensions.tech_sensors`. The Drive-compatible raycast backend remains useful for basic gameplay installs.
