# Backend Contract

## Commands and validation

No commands. The pure calculation accepts finite NED North/East plus optional finite P1/P2 yaw.
Horizontal magnitude at or below numerical epsilon has no azimuth.

## State, ownership, and concurrency

The existing panel owns latest baseline/GPS/ATTITUDE states. The evidence adapter creates an
immutable snapshot on the GUI thread at the bounded existing poll; the plot owns presentation-only
state. No shared mutable owner or lock is added.

## Worker resource budgets, scheduling, and backpressure

No worker or backpressure change. Calculation and formatting must remain constant-time and must not
scan files, histories, or queues.

## External boundaries and workers

MAVLink and GNSS callbacks remain unchanged. No device/API call is introduced.

## Results, errors, timeout, retry, and cancellation

Pure output fields:
`baseline_azimuth_deg`, `p1_yaw_deg`, `p2_yaw_deg`,
`p1_relative_deg`, `p2_relative_deg`, and `p2_minus_p1_yaw_deg`.
Compass values normalize to `[0,360)`; signed values wrap to `[-180,180)`. Positive relative angle
means the baseline is clockwise/right of that Pixhawk Forward direction. Freshness and partial
states come from existing source ages/status contracts, not from a new timeout/retry owner.

## Persistence, restart, and shutdown

No persistence or migration. Restart begins WAITING until existing live sources arrive. Shutdown
adds no work and retains the existing lifecycle order.

## 2026-09-03 corrected amendment

The GNSS baseline-center geometry adds presentation-only `baseline_drone_angle_deg` and
`baseline_drone_angle_error_deg` values. They are derived from existing midpoint `Forward/Right`
components as `atan2(|Forward|,|Right|)` and `|90°−angle|`; NED Down is excluded from the horizontal
angle. The values are returned by the existing Telemetry snapshot mapping and are not added to the
recording row/schema. Drone ATTITUDE/yaw is not read by this feature.
