# RoadMind local lane graph

The local lane graph turns a bounded neighbourhood of the BeamNG NavGraph into directed road segments and lane records. A geometric connection is not proof of a legal turn.

## Sources and cache

- `map.getMap()` supplies NavGraph nodes and links; the node table is cached for the active mission and cleared on mission start/end.
- `map.findClosestRoad()` identifies the road segment beneath the player.
- Node `pos` and `radius` provide geometry and width metadata.
- Edge `lanes`, `inNode`, `oneWay`, `drivability`, `hiddenInNavi` and `speedLimit` retain their NavGraph provenance.
- Lane direction is decoded from processed `+a` / `-a` tokens relative to `inNode`.

## Bounds

The graph is bounded to 96 directed segments, 4 graph hops, 125 metres from the player and 8 outgoing branches per expansion. It refreshes at the existing 2 Hz road-understanding cadence, not every render frame.

## Lane and junction records

Segments expose endpoints, width provenance, direction status and forward lane centres. Each lane has a segment-scoped ID, width, centre offset and world-space endpoints. Current-lane localization is published only when nearest-centre distance and road-quality checks pass.

Connections join incoming/outgoing lane centres sharing a NavGraph node and satisfying geometric distance and heading checks. They are tagged `geometric-candidate` and `legality=unverified`; they are not permission to turn, change lanes or overtake. Turn restrictions, lane arrows, markings and signal-to-lane assignment remain future work.

## Preview planner integration

The preview planner consumes the lane records on the vehicle's current directed segment. It keeps the localized lane as the baseline and may evaluate only the immediately adjacent lane on either side when lane localization is explicitly reliable. When localization is uncertain, it evaluates only the nearest plausible lane and does not propose a lane change.

Each candidate retains its lane ID/index and maneuver classification. Adjacent-lane candidates are marked `legality=unverified`; NavGraph connectivity does not prove that a lane change is legal. The existing vehicle-footprint, road-margin, turning-radius, obstacle and tracked-vehicle checks still decide whether a preview candidate is safe. Lane changes receive a score penalty so staying in the current lane is preferred when both options are otherwise comparable.

This is still a local preview along the existing route corridor. The planner does not yet choose among junction branches or follow lane-to-lane topology over the full route, and it does not actuate steering, throttle or brakes.
