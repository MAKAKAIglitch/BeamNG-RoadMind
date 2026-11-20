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
