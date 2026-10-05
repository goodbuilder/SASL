# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-09-05 — Keep CAN2 editable; show candidate versus actual

- User decision: the UI must not block CAN2 priority selection and must not show a topology
  rejection/reason dialog.
- Accepted behavior: keep the requested priority payload editable and visible, while showing
  the priority candidate separately from the actual PX4 `vehicle_magnetometer` device.
- Consequence: topology is explanatory read-only state; active-device/ATTITUDE verification
  remains the independent success oracle.

## 2026-09-05 — Prefer the newest uORB timestamp in queued Shell output

- Radio `SERIAL_CONTROL` may deliver an older `vehicle_magnetometer` listener block before the
  current block. The active-device parser now chooses the greatest PX4 `timestamp`, using block
  order only as a tie-breaker.
- This preserves the existing Shell polling contract and makes UI active verification align with
  the direct PX4 topic readback without changing priority semantics.
