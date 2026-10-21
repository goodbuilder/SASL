# Integration Flow

## Action-to-postcondition traces

`Shell polling -> parsed instance/device map -> panel latest topology snapshot -> workspace
preview` is read-only. `combo edit -> per-side draft -> command payload -> worker voter-gap guard
-> PARAM_SET with timeout retries -> final PARAM readback -> visible confirmed/draft state` is
the write path. On failure, stale readback is invalidated and the draft stays in the view.
`vehicle_magnetometer/ATTITUDE` remains the independent active postcondition used by Apply & Verify.

## Transition tables

| State | Preview | Apply |
|---|---|---|
| PARAM + topology live | candidate and actual shown | enabled if existing gates pass |
| PARAM live, topology waiting | candidate waiting, actual shown if available | unchanged |
| topology shows CAN2 instance 4 | CAN2 candidate visible but not stock candidate | not blocked |
| staged edit + periodic status | actual readback updates separately | draft remains unchanged |
| proven v1.17 voter gap | gap IDs and filler advice shown | disabled before any PARAM write |
| PARAM_VALUE timeout | retry visible in operation log | retry up to three total attempts |
| apply fails after partial write | draft retained; old cache invalidated | fresh readback reconciles actual values |
| actual differs after apply | candidate/actual mismatch shown | existing verification continues |

## Failure and recovery flows

## Cross-feature contracts and dependency direction
