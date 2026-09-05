# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-09-04 — Show raw and PX4-selected yaw together

- Decision: show a diagnostic raw sensor-axis yaw for every complete MAG vector and show the
  firmware `ATTITUDE.yaw` only for the row whose active device identity is proven.
- Rationale: the raw field is useful for sensor-by-sensor A/B diagnosis, while the PX4 value is
  the actual estimator output after firmware calibration/rotation and tilt-aware fusion.
- Preserved: priority workflow, active-device verification, raw axis/unit contract, and all
  existing connection/calibration behavior.

## 2026-09-04 — Never infer calibrated per-sensor identity

- Decision: standard `SCALED_IMU*` rows without an exact physical device ID do not receive a copied
  PX4 yaw. Inferred slot labels remain evidence only.
- Rationale: `SCALED_IMU*` stream numbering is not a guaranteed `CAL_MAG` device mapping, and PX4
  EKF yaw may include fusion/history beyond an instantaneous host calculation.
- Consequence: the UI may show RAW yaw while the corrected/reference cell remains `identity 확인
  대기`; this is an intentional safety state, not a missing implementation.

## 2026-09-04 — Scope and README approval

- Approval: the user approved the exact RAW + exact active-device PX4 yaw/delta scope and confirmed
  `apps/Workstation/README.md` as the documentation target.
- Conditions: preserve priority/firmware/stream/persistence/workflow behavior and stop for any
  required scope expansion.

## 2026-09-04 — Stable firmware RM3100 source

- Decision: use the existing `DEBUG_VECT` `CAN1_MAG`/`CAN2_MAG` compatibility messages for
  RM3100 presentation; do not use `SCALED_IMU*` and do not add a private `SASL_CAN_MAG` request.
- Rationale: stable firmware does not provide the private message, while `DEBUG_VECT` is already
  part of the stock MAVLink stream request profile and the decoder preserves CAN port and RM3100
  identity.
- Consequence: the IMU view has five fixed rows per Pixhawk (three `SCALED_IMU*` plus two RM3100
  `DEBUG_VECT` rows). If the named DEBUG_VECT sample is absent, its row remains WAITING/unavailable.

## 2026-09-05 — Stable PX4 Shell source for RM3100

- Decision: add a bounded read-only `SERIAL_CONTROL` Shell polling source using PX4
  `listener sensor_mag`, while retaining MAVLink `DEBUG_VECT`/dedicated-message paths unchanged.
- Rationale: stock PX4 exposes per-instance RM3100 `sensor_mag` data and `device_id` through uORB
  Shell, but does not provide the project-specific per-CAN MAVLink message required by the current
  live path.
- Contract: Shell samples keep `px4-shell/sensor_mag` provenance, known RM3100 identity, producer
  sensor axes, and local receive age. Active `vehicle_magnetometer` identity is read separately;
  PX4 yaw is shown only on exact identity plus fresh ATTITUDE as before.
- Consequence: each Pixhawk can populate five fixed magnetometer rows and five row-level yaw
  projections without custom firmware; the Shell path is diagnostic-rate, not a replacement for
  high-rate MAVLink telemetry.

## 2026-09-05 — Launcher connection postcondition

- Decision: `--start-pixhawk` must invoke the existing guarded MAVLink Connect owner after selecting
  the Pixhawk page; it must not only navigate to the page.
- Rationale: Shell RM3100 polling starts only after a real MAVLink connection. The previous
  screen-only behavior contradicted the launcher help and left the UI with no Pixhawk data even
  after a requested Pixhawk start.
- Preserved: manual per-side Connect/Disconnect, endpoint/transport selection, exclusive receive
  ownership, and all radio/Ethernet failure gates.

## 2026-09-05 — Shell polling status semantics

- Decision: expose automatic RM3100 Shell polling as `SHELL SYNC`, not `PARAM SYNC`.
- Rationale: both operations reserve the same receive owner, but Shell `sensor_mag` readback does
  not query CAL_MAG parameters. The previous generic status would falsely report parameter lookup
  immediately after every connection/poll.
