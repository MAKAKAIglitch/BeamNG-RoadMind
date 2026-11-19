# RoadMind architecture

## 1. Purpose and scope

RoadMind is being reorganized around three connected products:

1. a dependable, clearly calibrated sensor foundation;
2. a timestamped, provenance-aware World Model;
3. a compact 3D RoadMind Map built from that model.

The map is the user-facing representation of the reconstructed environment. Lane-level road structure and route/trajectory data are part of the foundation because the map must represent how the road can be driven, not merely where the road centreline is.

This branch is not an autopilot implementation project. Existing route, ADAS and Vehicle Lua controller prototypes are legacy experimental functionality; they must not dictate the new perception architecture or be treated as validated control.

## 2. Architectural invariants

- Every observation has a source, timestamp, coordinate frame, validity/freshness state and provenance.
- A simulator-assisted result is never relabelled as a physical sensor measurement.
- Camera coverage geometry is not an RGB image and is not semantic image recognition.
- The single configured LiDAR stays front-facing. The camera arrangement provides surrounding coverage.
- RADAR, LiDAR, cameras and ultrasonic sensors keep distinct responsibilities. Their measurements may corroborate one another but are not interchangeable.
- Static environment memory, road/lane structure, dynamic objects, ego state, route data and predictions remain separate model layers.
- Predictions are not fresh observations. Render smoothing is never fed back into fusion or planning.
- Unknown data stays unknown. The system must not fabricate lane boundaries, traffic-light phases, POIs or object classes.
- Acquisition, fusion, road modelling, planning and rendering run at separately bounded rates.
- The UI consumes compact snapshots; it does not run sensor queries or rebuild the World Model.

## 3. High-level data flow

    Sensor backends
        ↓
    Vehicle-specific mounts and calibration
        ↓
    Scheduled acquisition / simulator-assisted candidate sources
        ↓
    Timestamping and coordinate transformation
        ↓
    Observation validation and provenance
        ↓
    Tracking and multi-sensor fusion
        ↓
    ┌──────────────────────┬──────────────────────┐
    │ Spatial World Model  │ Road / Lane Model    │
    │ static + dynamic     │ topology + attributes│
    └──────────────────────┴──────────────────────┘
                    ↓
          Route / trajectory context
                    ↓
          Versioned compact map snapshot
                    ↓
              RoadMind Map UI

The data flow is a target architecture. Each boundary must be implemented and tested incrementally; a box in this diagram does not imply that the corresponding capability is already complete.

## 4. Runtime layers

### 4.1 Sensor backend adapters

The backend layer provides measurements through a common interface while preserving backend-specific semantics.

- **Regular BeamNG.drive:** existing scene-raycast proxies, vehicle state/electrics, vehicle OBB registry, NavGraph and available level metadata.
- **BeamNG.tech (future adapter):** native camera, LiDAR, RADAR, ultrasonic, GPS and IMU sensor outputs where supported by the installed version.
- A backend must declare its capabilities. The consumer must not assume that a Drive proxy supports the same fields or physical semantics as a native Tech sensor.

### 4.2 Vehicle geometry and sensor calibration

A vehicle profile defines the local coordinate frame, dimensions, sensor inventory and mount transforms for the active vehicle/configuration.

Each mount records position, orientation, field of view, usable range, update rate, role, source and calibration confidence. Vehicle-specific verified data takes priority. A generic/body-class fallback is allowed only when it is explicitly marked as estimated. Changing vehicle or configuration invalidates old transforms and sensor state.

The body, attached equipment and trailer geometry must be considered when evaluating self-occlusion. A mount's nominal field of view does not imply that it can see through the vehicle body, glass, bumper or an attachment.

### 4.3 Acquisition and scheduling

The sensor scheduler runs scans only when a job is due. Sensor acquisition must not be tied to the map's frame rate. Jobs are staggered and bounded; expensive scene queries never run in the UI/render loop.

Each sensor job publishes a compact frame with sequence number, simulation timestamp, sensor identity, mount transform and measurements. Missing or delayed jobs update health/freshness state instead of silently reusing old values as current.

### 4.4 Ego motion and coordinate transformation

Measurements are first expressed in the sensor-local frame, then transformed through the calibrated vehicle frame into the common BeamNG world/map frame.

The ego-motion layer uses the available vehicle pose, orientation, velocity, acceleration/yaw estimates and wheel/odometer signals. In regular Drive these are simulator-derived signals or virtual equivalents, not calibrated hardware measurements.

A scan must be transformed using the ego pose corresponding to its timestamp, not an unrelated later render pose. Motion compensation is required before historical measurements are accumulated into a stable map.

### 4.5 Observation validation and provenance

Every source produces observations with explicit provenance. Recommended provenance classes:

- **raycast_proxy** — geometric scene-raycast result;
- **camera_visibility_proxy** — vehicle candidate inside a camera frustum and not occluded according to simulator-assisted checks;
- **native_camera** — detection derived from a real RGB/depth/semantic frame;
- **native_lidar / native_radar / native_ultrasonic** — native backend measurement;
- **simulator_state** — position, velocity, OBB or object state obtained from BeamNG;
- **map_metadata** — NavGraph, DecalRoad, signal or POI metadata;
- **derived / fused / predicted** — output computed from other observations.

The system stores the source measurements separately from derived entities. Consumers can inspect why an object or road feature exists and whether its evidence is fresh.

### 4.6 Tracking and fusion

The tracker associates repeated dynamic-object observations across time and sensor handoffs. It preserves stable RoadMind IDs independently of transient BeamNG object IDs.

Fusion is source-aware:
- combine measurements only when their frames, timestamps and semantic meaning are compatible;
- retain per-source contributions and last-observed times;
- do not interpret a simulator OBB as independent camera or RADAR confirmation;
- do not use predicted positions as new measurements;
- keep unknown geometric clusters separate from classified semantic objects.

The static geometry map and dynamic track store have different update and expiration policies.

### 4.7 World Model and spatial memory

The World Model is the canonical runtime representation for the map and future consumers. It contains independent layers for ego state, observations, static geometry, road topology/lane geometry, dynamic tracks, semantic objects, POIs, signals and route/trajectory context.

All collections have explicit caps and lifetimes. Current-session mapping is the initial persistence target; long-term cross-session map storage is deferred.

### 4.8 Road and lane model

Road structure is assembled from verified sources such as NavGraph, DecalRoad and level metadata, with sensor observations used to confirm visible geometry and boundaries. These sources are not equivalent:

- NavGraph supplies road topology/centreline relationships, not necessarily lane polygons.
- DecalRoad supplies road-surface/decal structure and may not encode all painted markings as separate geometry.
- Traffic AI trajectories may help infer or validate plausible lane paths, but they are additional evidence/priors, not the RoadMind planner and not proof of visible lane markings.
- Camera-based lane markings require an actual image/semantic backend. A frustum alone cannot detect paint.

The model must distinguish measured lane boundaries, map-derived lane structure, inferred lane corridors and unknown lane data.

### 4.9 Route and trajectory context

The route layer connects roads and, when supported, lane-level graph edges. It publishes route geometry and candidate local trajectories to the map as data. The selected route/trajectory is blue in the target map.

A future planner must consider the full vehicle footprint and its swept corridor, not only the centreline. It must account for road boundaries, lane connectivity, obstacles and predicted traffic. This branch prepares and visualizes this foundation; full actuator control and autonomous manoeuvre execution are deferred.

### 4.10 RoadMind Map renderer

The map is a read-only consumer of a versioned snapshot. It draws a single dark, rounded 3D map view that follows the player's vehicle, with a low-poly ego model, low-poly tracked traffic, supported road/lane structure and the blue route/trajectory.

The map does not:
- run scene raycasts or inspect the complete game scene;
- own sensor scheduling, tracking or fusion;
- show raw diagnostic beams as its primary content;
- contain speed/gear/telemetry/ADAS/sensor-status panels;
- invent road markings, object classes or hidden objects;
- alter BeamNG vehicle physics or the original game world.

Raw sensor debug drawing remains a separate diagnostic facility. A separate RoadMind Vision app is not part of this branch target.

## 5. Common observation contract

An observation should carry, where applicable:

| Field | Meaning |
|---|---|
| id | Stable observation identifier within a session |
| source_id | Specific sensor, backend or simulator source |
| source_type | LiDAR, RADAR, camera, ultrasonic, ego, map, simulator, derived |
| provenance | Exact origin class; never infer it from display type |
| timestamp | Simulation timestamp of the source measurement |
| frame | Sensor-local, vehicle-local or world/map frame |
| position / geometry | Point, ray, surface patch, OBB or supported polygon |
| orientation | Measured or source-derived orientation, when available |
| confidence | Explicitly heuristic unless calibrated |
| freshness / age | Time since last valid source observation |
| status | observed, coasting, predicted, stale, occluded, invalid or unknown |
| contributors | Source observation IDs used by a fused result |

The exact serialized schema may evolve. Changes must be versioned and documented; old consumers must not silently interpret a new schema as the old one.

## 6. Dynamic and static state ownership

Static geometry is accumulated in world/map coordinates and corrected by later observations. Dynamic vehicles and moving obstacles live in a separate track store with bounded prediction horizons. The ego vehicle is represented independently with its own pose and footprint.

When a dynamic object moves, its old position must be removed from the active dynamic layer. A brief coasting track may remain for continuity, but it must be labelled as a prediction/coasting state and expire after its configured timeout. It must not leave a permanent trail in static geometry.

Rendering interpolation is a presentation-only transform. It cannot update observation timestamps, track state, obstacle positions used for safety decisions or the raw measurement store.

## 7. Errors, missing data and graceful degradation

- Missing sensor frames mark that source stale/unavailable.
- Failed calibration marks affected coverage uncertain; do not display a false verified mount.
- A source that is unavailable in regular Drive remains unavailable unless an explicit adapter is installed.
- A lane graph without lane-level metadata may provide road topology but must not be promoted to measured lane boundaries.
- Unknown traffic-signal state remains unknown.
- A missing POI database means no POI result; do not invent locations from GPS.
- If the map snapshot is stale or invalid, the UI should show a restrained unavailable state rather than plausible fabricated geometry.

## 8. Validation boundary

Code presence is not validation. Sensor origins, direction, range, occlusion, timestamp alignment, motion compensation, track handoff, stale-data expiry, lane topology, map snapshot size and UI cost must be tested in live BeamNG scenarios across different vehicles, maps and traffic densities.

Performance budgets and acceptance tests are defined in [PERFORMANCE.md](PERFORMANCE.md) and the branch specification.
