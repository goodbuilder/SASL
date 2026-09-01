# Integration Flow

## Action-to-postcondition traces

```text
selected P1/P2 GNSS callbacks -> authoritative NED baseline
P1/P2 ATTITUDE callbacks      -> authoritative yaw/quaternion
existing 250 ms visible poll  -> EvidenceAdapter composite snapshot
                              -> fixed numeric values + top-view arrows
```

## Transition tables

| Baseline | P1 yaw | P2 yaw | Group state |
| --- | --- | --- | --- |
| missing/undefined | any | any | WAITING |
| current | current | current | CURRENT |
| current | current | missing/stale | PARTIAL |
| current | missing/stale | current | PARTIAL |
| stale with finite direction | any finite | any finite | STALE |
| current | missing/stale | missing/stale | STALE or WAITING according to retained evidence |

## Failure and recovery flows

Invalid inputs produce no invented angle. A side can recover independently. Fresh callbacks and the
next poll restore CURRENT/PARTIAL without user action. Device recovery remains on existing controls.

## Cross-feature contracts and dependency direction

Domain pure function -> EvidenceAdapter read model -> Pixhawk workspace/plot. There is no reverse
dependency, command, persistence write, or access to Alignment/Measurement widgets.
