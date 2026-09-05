# Backend Contract

## Commands and validation

No command. The pure estimator accepts P1, P2, and Drone `GpsPositionState`, application
receive-monotonic `now_s`, maximum age 2.0 s, and maximum timestamp spread 1.000 s.

Coordinate contract, metres:

```text
b = P2 - P1 in local NED [North, East, Down]
t = Drone - P1 in the same local NED
v = t - 0.5*b                         # Drone from GNSS antenna midpoint
Right = normalize([b.North, b.East, 0])
Down = [0, 0, 1]                      # NED Down
Forward = Right x Down = [Right.East, -Right.North, 0]
F = dot(v, Forward)
R = dot(v, Right)
D = dot(v, Down)
Rg = sqrt(F^2 + R^2 + D^2)
```

Frame identifier:
`gnss_baseline_center_local_level_frd`; vector contract:
`gnss_baseline_center_local_level_frd:[forward,right,down,range]`.

## State, ownership, and concurrency

One immutable pure result owns geometry/state/reason/provenance. `DroneTelemetryController` owns
the snapshot mapping. No widget calculation and no mutable shared state. Existing subscriber and
GPS selection remain sole source owners.

## Worker resource budgets, scheduling, and backpressure

No worker or queue. O(1) arithmetic only; GPU 0, new threads 0, new processes 0, I/O 0. Existing
status polling/coalescing is unchanged.

## External boundaries and workers

No new external boundary. Inputs are already-decoded in-memory MAVLink GNSS states. MSL altitude
and application receive-monotonic timestamps retain their existing contracts.

## Results, errors, timeout, retry, and cancellation

- `CURRENT`: valid geometry, all ages in [0,2.0] s, max(timestamp)-min(timestamp) <=1.000 s.
- `STALE`: geometry valid but a source is old/future or spread exceeds 1.000 s. Preserve finite
  values and actual timing.
- `WAITING`: source missing/non-finite, horizontal baseline norm is effectively zero, or geometry
  cannot be formed. Do not emit numerical zero placeholders.
- Fix types/labels are carried as evidence and never gate state.
- No timeout/retry/cancel lifecycle; later snapshots naturally recover.

## Persistence, restart, and shutdown

No persistence or migration. Restart starts WAITING until live data returns. Shutdown owns no new
resource. Existing recording/session/public measurement schemas are unchanged.
