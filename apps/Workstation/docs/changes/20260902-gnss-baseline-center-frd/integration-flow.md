# Integration Flow

## Action-to-postcondition traces

```text
existing MAVLink receive
  -> selected P1/P2 GpsPositionState + Drone GpsPositionState
  -> pure GNSS midpoint/local-level FRD estimator
  -> DroneTelemetryController immutable snapshot mapping
  -> visible Telemetry one-row table + reason/contract
```

No click, ACK, write, connection, or persistence postcondition is introduced.

## Transition tables

| Prior | New input | Next |
|---|---|---|
| WAITING | three valid/current/synchronized positions | CURRENT |
| CURRENT | missing/degenerate input | WAITING |
| CURRENT | valid but old/future/skewed | STALE |
| STALE | qualifying input | CURRENT |
| any | low fix with otherwise qualifying position/timing | same timing-derived state; fix evidence changes |

## Failure and recovery flows

- Missing Drone/P1/P2 -> WAITING reason -> source returns -> recompute.
- Horizontal P1/P2 direction undefined -> WAITING without division -> valid baseline returns.
- Age/future/skew breach -> STALE with retained values -> timing recovers -> CURRENT.
- UI missing/malformed mapping -> one WAITING row; connection, position and RTCM groups continue.

## Cross-feature contracts and dependency direction

`MAVLink/GNSS source owner -> pure geometry -> Telemetry read model -> UI` only. The scalar `Rg`
may feed existing ideal disparity. There is no dependency from Measurement/Recording/Alignment
back into this feature and no access from the widget to panel-private GPS state.
