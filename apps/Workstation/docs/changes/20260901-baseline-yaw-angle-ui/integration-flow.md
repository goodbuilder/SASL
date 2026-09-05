# Integration Flow

## Action-to-postcondition traces

```text
selected P1/P2/Drone GNSS callbacks -> authoritative NED positions
existing 250 ms visible poll        -> GNSS midpoint/local-level geometry
                                    -> Telemetry snapshot mapping
                                    -> fixed F/R/D/Rg + angle/90° error columns
```

## Transition tables

| P1/P2/Drone GNSS | horizontal Drone vector | Group state |
| --- | --- | --- |
| missing/invalid | any | WAITING |
| current | finite non-zero | CURRENT |
| stale/future/spread breach | finite | STALE |
| current | zero horizontal vector | WAITING angle / geometry remains explicit |

## Failure and recovery flows

Invalid inputs produce no invented angle. Fresh callbacks and the next poll restore CURRENT/STALE
without user action. Device recovery remains on existing controls.

## Cross-feature contracts and dependency direction

GNSS pure geometry -> DroneTelemetryController snapshot -> Telemetry workspace table. There is no
ATTITUDE dependency, reverse dependency, command, persistence write, or access to Alignment/Measurement
widgets.
