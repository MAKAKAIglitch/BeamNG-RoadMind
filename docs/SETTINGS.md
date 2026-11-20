# RoadMind settings and runtime controls

## 1. Purpose

Runtime settings configure sensor acquisition, calibration, diagnostics and performance. They must not change the fundamental sensor responsibilities or cause the map to invent data.

The target user-facing application for this branch is RoadMind Map. Sensor diagnostics remain separate from the map so that the map stays focused on the reconstructed road environment.

## 2. Sensor activation

The current prototype starts sensor scanning disabled. Acquisition is started explicitly from the RoadMind settings page or the existing Controls binding. Keep this fail-idle behavior unless a deliberate design change is validated.

The `autoScanOnVehicleSpawn` setting defaults to `false`. When the active player vehicle is destroyed, replaced or respawned, a running sensor suite stops and stays stopped until the user starts it again. When the setting is enabled, RoadMind may resume scanning on the replacement player vehicle, but only if scanning was already active before the vehicle change. It does not start separate scanners on newly spawned traffic vehicles. An explicit manual start always takes precedence over this setting.

Hiding a debug overlay is not the same as stopping a sensor:
- disabling a sensor group stops its acquisition and reduces its query work;
- hiding a 3D overlay only hides its debug rendering;
- the map consumes the latest valid World Model snapshot and reports stale/unavailable state when data stops updating.

## 3. Performance profiles

The current prototype exposes Economy, Balanced and High profiles. Existing documentation gives these calculated query budgets:

| Profile | LiDAR | RADAR | Ultrasonic | Estimated scene queries/s |
|---|---|---|---|---:|
| Economy | 9 rays × 1 Hz | Off | Off | 9 |
| Balanced | 26 rays × 2 Hz | Up to 2 rays per unit × 1 Hz | 1 ray per unit × 1 Hz | 74 |
| High | 51 rays × 4 Hz | 17 directions × 2 Hz | 36 directions × 2 Hz | 310 |

These values are configuration-derived estimates, not measured FPS, CPU percentages or complete workload. Recalculate them from the active settings after any sensor layout or scheduler change.

## 4. Individual controls

The existing prototype includes controls for:
- LiDAR enablement, profile-controlled density and scan frequency;
- RADAR enablement, density and scan frequency;
- ultrasonic enablement, beam density and scan frequency;
- camera-visibility processing and pose/frustum updates;
- optional session-only point-cloud accumulation and its point limit;
- sensor/debug overlay visibility;
- optional adaptive sensor quality and speed-adaptive cadence;
- optional detailed RoadMind Lua profiling;
- vehicle-geometry status, manual recalibration and calibration overlay;
- optional automatic scan resumption after a player-vehicle spawn/replacement (off by default).

The exact control set and ranges are defined by the current runtime implementation. Do not document a control as available unless it exists in the branch code.

## 5. Diagnostics versus map content

Raw sensor rays, hit points, frustums, mount markers, OBB diagnostics and profiler values are engineering/debug views. They are not the visual design of RoadMind Map.

RoadMind Map should display the fused environment representation: supported road geometry and lane structure, low-poly tracked vehicles, relevant obstacles/semantic objects, POIs where available and the active calculated route/trajectory in blue.

Do not add permanent speed, gear, sensor health, ADAS or telemetry panels to the map.

## 6. Legacy experimental controls

The current codebase contains experimental route-pilot and ADAS controls. They are not the deliverable of this branch and must not be expanded at the expense of sensor validation, World Model correctness or RoadMind Map. Any existing control behavior must be labelled experimental until repeatable in-game validation is complete.

## 7. Configuration integrity

- Keep per-vehicle sensor mounts and calibration state separate from global performance profiles.
- Do not reset a user's sensor selections when the active vehicle changes.
- Keep automatic scan resumption after a player-vehicle replacement opt-in and disabled by default.
- Revalidate mount transforms when the vehicle/configuration changes.
- Mark unsupported sensors unavailable rather than showing a working but inert control.
- Preserve the source and freshness of sensor data when quality adapts.
- Keep optional debug drawing independent from data acquisition.
