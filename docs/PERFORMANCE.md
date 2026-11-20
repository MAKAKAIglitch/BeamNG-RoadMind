# RoadMind performance principles

## 1. Performance objective

Optimize for a fresh, coherent and useful World Model at the lowest practical CPU, memory and rendering cost. Maximizing the ray count or update rate is not the objective.

Optimization is a design constraint from the first implementation step. It must preserve source provenance, visibility correctness, timestamp alignment and safety-relevant geometry. Reducing work must not silently convert stale or incomplete data into apparently valid results.

## 2. Separate update rates

Do not run every subsystem at the graphics frame rate.

| Subsystem | Scheduling rule |
|---|---|
| Sensor acquisition | Run only when that sensor's configured scan is due |
| Ego pose / motion | Update at a bounded pose cadence appropriate to motion |
| Vehicle candidate registry | One shared cached pass, not a scan per sensor/ray |
| Target visibility / tracking | Bounded cadence and shared visibility budget |
| World Model fusion | On new observations or scheduled maintenance |
| Static map maintenance | Incremental updates; no full rebuild every frame |
| Road/lane graph | Cache level structure; update the local region and route-relevant links |
| Route / trajectory evaluation | Bounded planner cadence, independent of rendering |
| Map snapshot | Compact and throttled/incremental |
| Map rendering | May run at display refresh, but consumes cached snapshots only |

Scene raycasts, vehicle-registry scans, road graph discovery and fusion must not run from the UI/render loop.

## 3. Acquisition and query budgets

- Keep the existing one-front-LiDAR, five-RADAR, twelve-ultrasonic and seven-camera layout as the baseline.
- Preserve the current Economy/Balanced/High approach, but treat settings as configurable ceilings rather than a promise of quality or frame rate.
- Stagger jobs when multiple sensors become due together. The existing Vehicle Lua scheduler spreads due sensor/pose/detection-zone jobs across graphics frames; retain bounded scheduling rather than issuing every job in one burst.
- Use one shared vehicle OBB registry and candidate-detection pass. Do not enumerate all vehicles independently for each sensor or each ray.
- Apply cheap distance/FOV/OBB tests before any expensive line-of-sight raycast.
- Share and cap occlusion checks across cameras/candidates.
- Keep raycasts outside the renderer and avoid redundant queries when cached data remains valid.

The existing settings documentation reports approximate scene-query budgets of 9 queries/s for Economy, 74 for Balanced and 310 for High. These are calculated from configured ray counts and frequencies, not measured CPU cost, FPS or total workload. Recalculate them whenever sensor settings change, and report actual in-game measurements separately.

## 4. Geometry, memory and tracking bounds

- Cache vehicle geometry and sensor mount layout after attach/calibration; do not query full dimensions for every ray.
- Cache static level data such as NavGraph relationships, road metadata and POIs when the level loads.
- Build only the nearby/local road and lane graph needed around the ego vehicle and active route.
- Use spatial indexing for static geometry, dynamic tracks and POIs.
- Bound observation counts, track counts, point-cloud history, displayed objects and prediction horizon.
- Expire dynamic tracks and stale observations explicitly.
- Deduplicate repeated static returns and avoid unbounded insertion of nearly identical points.
- Reuse tables, vectors and render resources when safe; avoid per-frame allocation of deep copies.
- Send compact versioned snapshots and incremental changes to the UI instead of full internal stores.

Limits must be configurable or centrally defined and documented. Every cap needs a defined overflow policy: drop least relevant/stale data, aggregate it or reduce detail without corrupting timestamps and provenance.

## 5. Rendering efficiency

- Use low-poly vehicle models and simplified road geometry.
- Limit map range and object count; use distance-based detail/LOD.
- Reuse static geometry and update only changed dynamic objects.
- Simplify/merge nearby surface points before UI transfer.
- Keep raw diagnostic beams and sensor overlays separate from the map.
- Do not create large numbers of new render objects every frame.
- Make map visibility and diagnostic overlays independently switchable.
- Rendering interpolation may improve visual continuity but must never change World Model observations or planning inputs.

The target map is a single compact view, not a second full-resolution copy of the game scene.

## 6. Adaptive quality

Adaptive quality may reduce scan density, frequency or display detail when the measured load is excessive. It must:
- lower one quality tier at a time and recover more conservatively;
- never exceed the user's selected profile;
- preserve minimum useful coverage and correct sensor roles;
- expose when a source is degraded or stale;
- avoid interpreting reduced coverage as evidence that an area is empty;
- keep the blue planned route and critical nearby dynamic objects prioritized in the map.

An FPS target is a best-effort adaptation threshold, not a frame-rate lock. RoadMind cannot guarantee the game will meet it.

## 7. Measurement and reporting

Measure the same vehicle, map, traffic density, weather/time settings and graphics configuration when comparing changes.

Record at minimum:
- average and percentile frame time/FPS;
- RoadMind CPU time per subsystem where instrumentation is available;
- scene-raycast query rate;
- vehicle registry and visibility-check counts;
- observation/track/map point counts and memory;
- UI snapshot size and update rate;
- rendering cost with map hidden/shown and diagnostic overlays hidden/shown.

Separate:
- **calculated workload** — query counts derived from configured rates;
- **RoadMind profiler data** — time spent in instrumented RoadMind code;
- **whole-game performance** — game-wide FPS/CPU/GPU and frame-time data.

Do not claim a CPU/FPS improvement from fewer raycasts or fewer allocations without a before/after test.

## 8. Graceful degradation

When under load:
1. reduce optional display detail;
2. reduce non-critical scan density/frequency within the selected profile;
3. reduce map distance and low-priority object detail;
4. preserve the freshness/provenance state of all retained data.

Do not conceal sensor failure, remove critical near-field coverage without reporting it, or show stale data as current merely to keep the map visually full.

## 9. Validation checklist

- [ ] No scene raycasts from the UI/render loop.
- [ ] Shared vehicle registry and bounded occlusion budget are used.
- [ ] All collections and prediction horizons have explicit caps.
- [ ] Map snapshots are compact and incremental or throttled.
- [ ] Sensor acquisition, fusion, planning and rendering can be profiled separately.
- [ ] Economy/Balanced/High query counts are recalculated and labelled as estimates.
- [ ] Before/after performance is measured on repeatable scenarios.
- [ ] Reduced quality is reflected in sensor freshness/coverage state.


## 10. RoadMind Map circular memory and render budgets

The static surface cache is world-anchored inside a configurable circular radius around the active player vehicle (default 500 m, configurable 100–500 m). The backend prunes out-of-radius cells after approximately 12 m of ego movement instead of deleting the entire map after a fixed driven distance. The existing hard cap on stored cells remains in place; the compact map snapshot is limited to 4,000 static surface points.

The UI cache independently prunes points, edges and faces outside the selected circle. It rebuilds spatial indexes only after the player has moved about 25 m or the radius has changed, and caps cached surface points at 12,000. This prevents the UI cache from growing forever while preserving nearby mapped areas.

Map drawing uses a configurable polygon budget (default 3,000; range 500–8,000), with separately bounded lines and markers. Map render cadence defaults to 30 FPS and can be adjusted from 15 to 60 FPS. This is a widget rendering limit only; it does not change game FPS, sensor sampling frequency, world-model fusion or route planning cadence.

Camera height, follow distance and look-ahead are configurable from RoadMind settings. Missing vehicle pose yields a dark perspective grid rather than fabricated road geometry. The player's car marker is drawn above map layers to remain visible when the polygon budget is reached.

The limits reduce worst-case rendering and cache growth, but they are not proof of an FPS improvement. Validate with repeatable in-game before/after measurements and inspect road detail at several polygon budgets.


## 11. Scanned surface appearance

The local surface reconstruction still uses triangles internally to connect measured static points, but the RoadMind Map must not expose that triangulation visually. Faces are filled with a uniform opaque graphite-grey tone (#343a40) and are drawn without polygon outlines or mesh-edge strokes. Edge-only geometry is not generated, cached or submitted for drawing. This makes scanned coverage read as a continuous reconstructed surface against the darker unknown-area grid, rather than a mint wireframe.


## 12. NavGraph road surface and lane rendering

The fallback road and lane graph is sourced from the cached local NavGraph (map.getMap()), the same navigation graph BeamNG uses for AI and traffic routing. The primary FSD-style surface is now sourced from cached DecalRoad splines in items.level.json; see section 13. RoadMind does not add sensor raycasts for map rendering.

- **Width priority:** use the NavGraph node radius directly as road half-width. If radius is absent but directional lane counts are present, estimate total width as 2.4 m for a single-lane segment or 3.05 m per lane for segments with two or more lanes, then use half as the edge half-width. These are BeamNG Traffic-style lane-width estimates, not exact measured widths. If neither source exists, use the conservative planner-width fallback. The widthSource field preserves which path was used. The renderer accepts road half-widths down to 1.2 m so narrow roads are not artificially widened.
- **Connected road surface:** build one strip per physical NavGraph edge, share mitered corner offsets across degree-two nodes, and add bounded surface patches at nodes with three or more road connections. Degenerate or very sharp joins fall back to the local segment normal to avoid long spikes.
- **Lane separators:** use NavGraph forward/reverse lane counts to position separators across the entire road, including the centre divider on two-way roads. Precompute their world-space endpoints from the same joined corner geometry so markings continue through bends.
- **Traffic fallback:** confirmed, direction-aligned traffic trajectories can infer provisional lane centres only where NavGraph lane metadata is missing. Their boundaries retain a visually distinct provisional style and are not promoted to authoritative metadata.
- **Bounds:** the fallback NavGraph renderer keeps a 192-segment cap and bounded traversal; the UI fallback renderer consumes the same 192-segment ceiling. Primary DecalRoad sampling is separately capped at 128 visible runs and 1,800 samples, with the configured look-ahead and cached road-object bounds. Both paths reuse level data instead of rebuilding the complete level graph each UI frame.

These are static geometry and source-selection checks. Road width estimates and junction appearance still require visual validation on several BeamNG levels, especially roads with missing radius/lane metadata, unusual lane widths, and complex intersections.


## 13. FSD-style road map source and sensor display

The map is organized into two views. The in-game mini-map keeps a forward-looking perspective around the ego vehicle. Clicking it opens a full-screen top-down map covering the all-directions road network and lane markings around the current vehicle position.

### Road geometry and loading

- **Primary source:** active-level `DecalRoad` objects from `items.level.json`. Each accepted node uses `[x, y, z, width]`; road-level settings include `lanesLeft`, `lanesRight`, `autoLanes`, `oneWay`, `flipDirection` and `smoothness`.
- **Background preload:** level objects are parsed incrementally, up to 128 objects per map request. The loader caches road nodes, widths, lane counts and object bounds; the network is then sampled in all directions within the configured map radius (default 500 m).
- **Detail levels:** sample spacing is about 2.5 m within 120 m of the vehicle, 8 m from 120–260 m, and 16 m in the outer ring. The radial road window includes a 35 m continuity margin at its edge. This keeps lane geometry dense close to the car without using the same vertex density across the whole level.
- **Curve:** use centripetal Catmull–Rom interpolation blended from linear interpolation by the road's smoothness value. Interpolate road width along the same curve. Manual lane counts take priority; automatic lane estimates are used only when manual counts are absent.
- **Geometry bounds:** at most 512 road runs and 8,000 spline samples are retained for the current view. Candidate roads are filtered by bounding boxes and processed nearest-first. During level preload, an existing geometry cache is reused; the full view is rebuilt after loading completes or when the vehicle/radius/road cache meaningfully changes.
- **Fallback:** if DecalRoad data is unavailable, use the connected NavGraph geometry and its lane metadata/traffic inference. A missing level file remains an explicit data-source limitation, not fabricated road coverage.

DecalRoad is a road-surface decal projected onto terrain or static surfaces, not a standalone elevated road mesh. The map reconstructs its ribbon from node positions and widths; it does not copy clipped material textures or the exact terrain/static-mesh edge geometry. Bridges and unusual elevated structures still require in-game validation.

### Map interaction and display

- **Mini-map:** remains a compact FSD-style forward-perspective view. It uses a budget of 650 polygons, 1,200 lines and 32 markers and does not triangulate the scanned surface.
- **Large map:** opens full-screen from the mini-map. It uses a top-down view, drag-to-pan, wheel-to-zoom, a recenter control and Escape/close to return. Its per-draw budget is 8,000 polygons, 12,000 lines and 900 markers.
- **Scanned detail:** only the large map reconstructs the nearby measured surface. It consumes up to 2,500 scan points from within 90 m of the vehicle, submits at most 1,800 faces in one draw, and keeps a client-side face cache capped at 16,000 faces. New data is merged by the session-map revision rather than by every UI poll.
- **No sensor overlays:** LiDAR point clouds, radar/ultrasonic returns and sensor/camera coverage cones are not drawn on either map view. Sensor acquisition, object fusion and ADAS processing remain active according to their own settings; only the map visualization is removed.
- **Persistent scan memory:** stopping scanning does not clear the session map. The cached world-space scan geometry remains available to the large map; only explicit map clearing or a level transition resets the corresponding memory.
- **Idle recentering:** the map snapshot checks the live player-vehicle position and refreshes the compact static-geometry output after about 12 m of movement or when the configured radius changes, even with scanning disabled. The refresh reads the existing session cache and does not require new sensor observations.
- **Point revision updates:** when the session-map revision changes, the client updates the world position of an existing cached point and moves its spatial-bucket index if its cell changed. This keeps the triangulated surface aligned with the updated backend geometry.
- **View-only lifecycle:** opening or closing the map does not automatically start or stop scanning. The map polls the current vehicle pose and map revisions independently of sensor activation.

## 14. Map responsiveness and update cadence

The map no longer performs a full draw on every animation frame. It marks the canvas dirty only when a new map snapshot arrives, the canvas is resized, or the user pans/zooms. The animation-frame loop then draws the changed view and stays idle between changes.

- **Snapshot cadence:** when road preload is complete, the map requests a new compact snapshot every 100 ms. While road objects are being loaded, polling is reduced to 260 ms for the mini-map and 220 ms for the large map so background parsing does not dominate the game loop.
- **Incremental payloads:** the client sends the latest road and session-surface revisions. Unchanged road geometry and scan points are not serialized repeatedly; the browser/UI retains its last copy until a revision changes.
- **World Model reuse:** `getMapSnapshot` reads the GE-owned World Model once and passes that same read-only model to the perception snapshot builder. It does not create two full deep copies per map request. The World Model itself is rebuilt only when sensor/tracking data marks it dirty, with a 0.20 s update interval.
- **Road sampling:** the map snapshot calls the road-geometry builder once. During background level preload it reuses the previous geometry cache for up to 1.5 s while the road cache revision changes, then performs a complete rebuild when loading is finished.
- **Surface limits:** the mini-map omits the surface reconstruction pass entirely. The large map uses the nearby-point, face submission and persistent-face limits described in section 13.
- **Validation status:** these limits and update paths are statically checked. They do not establish a measured FPS improvement. Final performance still needs an in-game comparison using the same vehicle, level, weather, traffic density and scan settings.

