# RoadMind World Model

## 1. Purpose

The World Model is RoadMind's canonical, time-consistent representation of the nearby environment. It combines sensor observations, supported road metadata and ego-motion into a compact local model consumed by RoadMind Map and, later, route/lane planning.

It is not a copy of the complete BeamNG scene. Every element must have an identifiable evidence source. Simulator-assisted data, map metadata, sensor measurements, fused estimates and predictions must remain distinguishable.

## 2. Coordinate and time model

Use a common BeamNG world/map coordinate frame in metres for the current Drive backend. Sensor measurements begin in sensor-local coordinates and pass through the calibrated vehicle-local transform into the common frame.

Every observation must retain the simulation timestamp at which it was measured. When accumulating scans, use the ego pose corresponding to the measurement timestamp. Do not transform an old scan with the current render pose.

Keep these concepts separate:
- **measurement time** — when the sensor/source produced the data;
- **processing time** — when RoadMind consumed it;
- **display time** — when the map rendered it;
- **prediction horizon** — how far a derived state is projected beyond the last observation.

Prediction and display interpolation never create a new measurement timestamp.

## 3. Common observation envelope

Each observation should contain, as applicable:

| Field | Purpose |
|---|---|
| observation_id | Unique ID within the current session |
| source_id / source_type | Concrete sensor/backend/source |
| provenance | raycast proxy, native sensor, simulator state, map metadata, fused, derived or predicted |
| timestamp | Source measurement timestamp |
| frame_id | Sensor-local, vehicle-local or world/map frame |
| pose_at_measurement | Ego/sensor pose used for transform |
| geometry | Point, ray, surface sample, OBB, polygon or other supported primitive |
| semantic_class | Known class only; otherwise unknown |
| confidence | Heuristic unless calibrated and validated |
| freshness / age | Time since the latest valid source update |
| status | observed, coasting, predicted, stale, occluded, unavailable or invalid |
| contributors | IDs of observations contributing to a fused result |

The exact field names may change with schema version. Any schema change must be explicit and versioned.

## 4. World Model layers

### 4.1 Ego vehicle

Store the player's pose, orientation, dimensions, speed/velocity and available motion signals. The ego footprint is derived from the active vehicle geometry profile and is used to establish transforms and later evaluate the swept path of the full vehicle body.

### 4.2 Raw observation store

Keep bounded source-specific measurements for LiDAR, RADAR, ultrasonic, camera/image detections, simulator-assisted vehicle candidates and map metadata. Raw records are immutable after publication; filtering/fusion produces derived records rather than rewriting source data.

### 4.3 Static geometry memory

Accumulate stable geometric evidence in world/map coordinates:
- road and pavement samples;
- supported road edges and boundaries;
- curbs, barriers, walls and other static surfaces;
- unknown obstacle/surface clusters where semantic class is not established.

Repeated scans can refine or reject earlier geometry. Use spatial indexing, bounded range and memory caps. Do not permanently insert moving vehicles into static geometry. A geometric hit is not automatically a semantic object such as a tree, building or sign.

The initial mapping target is session-local memory. Persistent cross-session maps are deferred.

### 4.4 Road and lane model

Keep road topology and lane structure separate from raw surface points.

Potential inputs, each with explicit provenance:
- NavGraph nodes and links for road connectivity and centreline context;
- DecalRoad nodes, widths and road-surface attributes where available;
- verified map/level metadata such as lane count, direction, road width, restrictions and junction relationships;
- sensor-derived boundaries/markings when a real suitable sensor source provides evidence;
- AI traffic trajectories as supplementary lane/path priors or validation references.

Important distinctions:
- NavGraph centreline is not a lane polygon.
- DecalRoad is projected road/decal structure and may not expose every painted marking as independent geometry.
- A centreline-derived corridor is an estimate, not an observed lane boundary.
- AI traffic movement is a prior/reference, not a substitute for RoadMind perception or its future planner.
- If lane direction, boundaries, width or connectivity are not supported, publish unknown/estimated rather than fabricate them.

The target road representation is a lane-level graph: directed lanes/segments, widths, boundaries where known, allowed transitions, junction connections and confidence/provenance. It should support lane localization and future lane selection rather than only road-centre following.

### 4.5 Dynamic objects and tracking

Vehicles and other moving objects are held in a separate track store. A track may contain:
- stable RoadMind track ID;
- latest observed position and orientation;
- length, width, height and oriented footprint;
- estimated velocity and direction;
- simulator-provided velocity as a separate field where available;
- last observed timestamp, age and missed-update count;
- contributing sensors/sources and provenance;
- confidence and observed/coasting/predicted/stale status.

Associate new observations with existing tracks using source IDs, geometry and motion consistency. Camera handoffs must not create duplicate tracks. Coasting is allowed only for a bounded interval. Once a track expires, remove its active footprint and any render attachment so it cannot leave a ghost behind.

Estimated target velocity from timestamped positions is not physical radar Doppler. Keep prediction separate from latest measured state.

### 4.6 Semantic objects and infrastructure

Signs, signals, markings and infrastructure need a suitable semantic source. A LiDAR surface hit may create an unknown geometric surface, but cannot independently prove the class of a sign, tree, building or traffic light.

For traffic signals, use a verified live runtime signal state when available, associate the signal with its actual approach/direction/lane group, and retain timestamp and provenance. A signal's phase must not be guessed from its presence or from an assumed cycle. Camera colour recognition requires a real image frame. Unsupported or ambiguous state is unknown.

### 4.7 Points of interest

POIs are loaded from verified level metadata when present, not created by GPS. Candidate sources include:
- level .sites.json data: Locations, Zones and Parking Spots;
- BeamNGPointOfInterest records;
- facilities metadata such as fuel stations, charging stations and other supported facilities;
- parking areas/spaces exposed by level tooling.

Cache static POIs and build a spatial index when a level loads. The current ego position is used to find nearby points and connect a selected POI to the road/lane graph. Preserve POI name, type, bounds/position, orientation, metadata source and confidence. If metadata is missing, report that no POI data is available.

For un-georeferenced levels, all positioning remains in local BeamNG coordinates. Do not manufacture latitude/longitude.

### 4.8 Route and trajectory layers

Keep route topology, selected lane, candidate trajectories and active trajectory distinct. The selected route/trajectory is rendered in blue on RoadMind Map.

A future planner must compare multiple feasible lane-aware candidates and check the full ego vehicle footprint along the swept path. It must also account for the dimensions and predicted motion of other vehicles, road edges, lane connectivity and relevant traffic controls. A centreline-only path check is insufficient.

Full steering/throttle/brake actuation, autonomous lane changes and overtaking are deferred. Existing experimental route/controller code is not proof that this planning/control target is complete.

## 5. Fusion rules

1. Transform source measurements using the timestamp-aligned sensor and ego poses.
2. Validate range, field of view, calibration, occlusion and source freshness.
3. Preserve raw observations and their provenance.
4. Fuse only compatible measurements of the same physical feature or target.
5. Keep source-specific confidence and contribution IDs.
6. Separate static geometry from dynamic tracks.
7. Keep observed, coasting and predicted state separate.
8. Reject outliers and discontinuous associations rather than forcing a match.
9. Expire stale data under explicit per-layer policies.
10. Never allow render smoothing or UI state to mutate the source store or fused model.

A missing source is not negative evidence unless the sensor was active, healthy and had valid coverage of the region.

## 6. Visibility and memory

Visibility and memory are different concepts.

- **Visible now:** a fresh measurement confirms the object is currently within valid sensor coverage and not occluded.
- **Coasting:** a recently observed track is temporarily not refreshed but is retained for a short bounded period.
- **Predicted:** a derived future position or state, never a new measurement.
- **Stale:** evidence is too old for a current claim.
- **Unknown / unavailable:** the source or information needed to decide is absent.

Static geometry can persist in the current session after it leaves the field of view, but should carry observation age and confidence. Dynamic targets must not persist as static objects. The first version is a bounded session map; long-term storage and cross-session relocalization are future work.

## 7. Runtime API and snapshots

Expose a versioned, read-only snapshot for RoadMind Map and future consumers. The UI should receive only the local region and relevant route/POI/signal/track information needed for the current view.

Snapshots should:
- be compact and bounded;
- include schema version, sequence and simulation timestamp;
- publish incremental changes where practical;
- avoid sending the full raw point history or complete level graph every UI update;
- be copied or immutable from the consumer's perspective.

The current implementation contains an experimental schema-v2 World Model prototype. Treat it as a starting point to reconcile with this target, not as a reason to preserve incomplete semantics or silently break existing consumers.

## 8. Acceptance criteria

- [ ] Every rendered object has a known evidence source and freshness state.
- [ ] Sensor-local measurements are transformed with timestamp-consistent ego poses.
- [ ] Static geometry and dynamic tracks cannot contaminate one another.
- [ ] Repeated observations refine geometry without creating unbounded duplicates.
- [ ] Occluded or out-of-view targets are not labelled currently visible.
- [ ] Coasting/predicted objects are visually distinct from fresh detections.
- [ ] Traffic signals are associated with the correct approach/lane and remain unknown when unsupported.
- [ ] Lane structure distinguishes map-derived, sensor-observed, inferred and unknown data.
- [ ] POIs are sourced from level/facility metadata and cached rather than guessed from position.
- [ ] UI snapshots are versioned, bounded and read-only.
