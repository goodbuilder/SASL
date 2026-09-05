# Integration Flow

## Action-to-postcondition traces

`Shell polling -> parsed instance/device map -> panel latest topology snapshot -> workspace
preview` is read-only. `combo edit -> role/raw priority payload -> PARAM worker -> PARAM readback`
remains the existing write path. `vehicle_magnetometer/ATTITUDE` remains the independent active
postcondition used by Apply & Verify.

## Transition tables

| State | Preview | Apply |
|---|---|---|
| PARAM + topology live | candidate and actual shown | enabled if existing gates pass |
| PARAM live, topology waiting | candidate waiting, actual shown if available | unchanged |
| topology shows CAN2 instance 4 | CAN2 candidate visible but not stock candidate | not blocked |
| actual differs after apply | candidate/actual mismatch shown | existing verification continues |

## Failure and recovery flows

## Cross-feature contracts and dependency direction
