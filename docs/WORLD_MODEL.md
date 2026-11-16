# RoadMind world model

The Game Engine Lua extension exposes a versioned, read-only snapshot for route planning and other consumers:

```lua
local snapshot = extensions.roadmind.getWorldModel()
```

The returned value is a deep copy. Callers may inspect it without being able to mutate RoadMind's internal sensor, tracking or fusion state.

## Coordinate contract

| Field | Contract |
|---|---|
| `schemaVersion` | Current schema: `2` |
| `coordinateFrame` | `beamng-world` |
| `coordinateUnits` | `meters` |
| `timestamp` | GE simulation time in seconds |
| `generation` | Increases whenever RoadMind rebuilds the snapshot |

Every world position is expressed in BeamNG world coordinates. Observation timestamps and data ages use the same GE simulation clock; they are not wall-clock timestamps.

## Top-level shape

```text
WorldModel {
  schemaVersion, generation, timestamp,
  coordinateFrame, coordinateUnits,
  targets[], obstacles[],
  observations {
    lidar[], radar[], ultrasonic[], vehicleRegistry[]
  },
  observationCount,
  sourceStatus {},
  quality {},
  summary {}
}
```

The snapshot has four observation streams. All use a common envelope with an observation ID, position, coordinate frame, timestamp, data age, source sensor, source kind, measurement kind, confidence, and confidence-model label. Orientation, dimensions, sensor origin, ray direction, range, target association and velocity fields are present when they are available.

## Observation sources

### LiDAR, radar and ultrasonic

These arrays contain individual scene-raycast hit measurements. Each record can include:

- `position`: measured scene hit point.
- `sensorOrigin` and `rayDirection`: the ray's world-space origin and direction.
- `range`, `rayIndex`: returned distance and original ray/beam index.
- `timestamp`, `dataAge`: scan time and age at model-build time.
- `sourceSensor`, `sensorName`: sensor family and individual mount.
- `hitObjectId`: simulator object ID when the raycast exposes one.
- `targetTrackId`: RoadMind track association when the hit belongs to a tracked vehicle.
- `orientation`, `dimensions`: attached from a tracked vehicle OBB when available.
- `sourceKind = "scene-raycast-measurement"` and `measurementKind = "geometric-scene-raycast-proxy"`.

The raw measurements remain separate by sensor. A radar or ultrasonic raycast proxy is not an RF radar or an acoustic measurement.

### `vehicleRegistry`

This stream contains only vehicle candidates whose visibility has been confirmed by the bounded occlusion checks. Records share the common observation envelope and can include the OBB center, orientation, dimensions, source sensor zones, simulator velocity and RoadMind track association.

These records are explicitly labelled:

- `sourceKind = "simulator-assisted"`
- `measurementKind = "vehicle-obb-center-and-extents"`
- `sourceSensor = "vehicleRegistry"`

The position and geometry come from BeamNG's vehicle OBB/object data. This is not a physical LiDAR, radar or camera detection. Unverified and occluded candidates are counted in quality metrics instead of being published as confirmed vehicle observations.

## Targets

`targets[]` contains one entry per active RoadMind vehicle track, keyed by a session-scoped `trackId` such as `RM-000001`. Multiple sensor-zone detections and ray hits for the same tracked vehicle refer to that one target.

Important fields:

| Field | Meaning |
|---|---|
| `position` | Latest observed OBB-center position; not predicted |
| `predictedPosition` | Separate short-horizon display/planning estimate |
| `predictionHorizon` | Maximum extrapolation horizon used by the tracker (0.5 s) |
| `predictedPositionKind` | `short-horizon-linear-extrapolation` when available |
| `timestamp`, `dataAge` | Time of last accepted observation and its current age |
| `status` | `confirmed`, `coasting` or `stale` |
| `confidence` | Tracker confidence heuristic, not a calibrated probability |
| `simulatorVelocity` | Velocity reported by BeamNG, kept separate |
| `estimatedVelocity` | Finite-difference estimate from timestamped positions |
| `estimatedRelativeSpeed` | Track velocity relative to the player vehicle |
| `velocitySource`, `relativeSpeedSource` | Explicit provenance of derived velocity fields |
| `sourceKind`, `measurementKind`, `positionSource` | Simulator-assisted provenance |

The latest observed `position` is never overwritten by the predicted display position. Consumers that require measured state should use `position`, `timestamp` and `dataAge`, and decide whether to accept the separate prediction.

## Obstacles

`obstacles[]` contains non-vehicle scene-raycast hits grouped into 0.75 m world-space cells. Each cluster has a spatial `obstacleId`, averaged position, first/last observation timestamps, data age, confidence heuristic and the source sensors/observation IDs that contributed to it.

These are geometric point clusters, not semantic object identities. Two clusters in adjacent cells may represent different parts of the same physical object; no semantic classification or persistent non-vehicle object tracker is claimed. Conflicting hits within a cluster are retained and counted rather than silently treated as exact agreement.

Vehicle hits that associate with the vehicle registry/tracker are kept as vehicle observations/targets and are not duplicated as non-vehicle obstacle clusters.

## Source freshness

`sourceStatus` contains entries for `lidar`, `radar`, `ultrasonic` and `vehicleRegistry`. Each entry reports whether the source is enabled, its latest timestamp, data age and freshness. A stale source is not silently treated as a current measurement.

- Scene-raycast source data is considered fresh for up to 1.5 s.
- The vehicle OBB registry snapshot is considered fresh for up to 1.0 s.
- A registry may be marked `truncated` when its configured vehicle limit was reached.

The scan groups run at different rates. The world model retains each source's own timestamp and data age; it does not pretend that all sensors sampled at the same instant.

## Quality counters

`quality` reports:

- `invalidObservationCount`: malformed coordinates, invalid dimensions/orientation, or timestamps outside the accepted simulation-time range.
- `staleObservationCount` and `staleSourceCount`: old measurements and enabled sources without fresh data.
- `truncatedSourceCount`: a vehicle registry snapshot reached its configured cap.
- `lowConfidenceObservationCount`: records below the current confidence heuristic threshold.
- `conflictingObservationCount`: spatially grouped hit points that disagree with the current cluster centroid.
- `droppedObservationCount`: records omitted after the bounded observation budget was reached.
- `staleTargetCount`: tracked targets whose observation age/status is stale.
- `unassociatedVehicleHitCount`: a scene ray hit a known registered vehicle but did not associate with an active RoadMind track.
- `unverifiedVehicleCandidateCount` and `occludedVehicleCandidateCount`: candidate OBBs that are not confirmed-visible.

Confidence values are heuristics, not calibrated probabilities. A planner should consider source provenance, freshness, confidence and quality counters together.

## Bounds and consumer rules

The model is bounded to 512 observations and 128 tracked targets. Per-sensor observations are preserved while the bounded target/obstacle aggregates provide convenient planning inputs.

- Use `targets` for tracked vehicles and `obstacles` for non-vehicle scene hits.
- Use the `observations` arrays when source-specific measurements are needed.
- Reject stale or invalid data according to the consumer's safety policy.
- Treat `simulator-assisted`, `scene-raycast-measurement`, `estimated` and `predicted` values according to their explicit provenance.
- Never use render-only interpolated LiDAR point positions as sensor measurements. They are maintained in a separate visualization layer.

## Local planner consumer contract

Phase 9 consumes timestamped targets, obstacle clusters, per-source freshness and a cloned route-state snapshot. It rejects planning when all enabled obstacle sensor sources are stale, road localization is not high/medium quality, the active vehicle has no valid cached OBB calibration, or no connected route corridor is available.

Vehicle targets are checked by projecting their OBB length/width onto each candidate tangent/lateral axis. Non-vehicle obstacles are point clusters and receive a conservative geometric margin; the planner does not infer semantic object shape from them. Only recent obstacle clusters and confirmed/coasting targets are considered. Render-smoothed LiDAR points are never used.

The current Drive-mode road context is a NavGraph centerline, not a lane-boundary polygon. Phase 9 therefore applies a conservative 3.5 m half-width corridor heuristic and a vehicle-length-derived minimum turn radius. Both are explicit approximations and require in-game validation. The planner can stop the built-in route AI if no candidate is safe, but it does not control steering/throttle/brakes; that is Phase 10.

The regular BeamNG.drive build has no RGB camera stream. Camera frustums therefore do not create camera observations in this schema.
