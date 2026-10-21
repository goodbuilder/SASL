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

## 2026-10-12 — Preserve edits and prevent known v1.17 voter gaps

- User approved the follow-up fix after the RCA and concrete proposal.
- Priority combos are staged editor state owned by `MagnetometerView`; periodic PARAM readback
  cannot overwrite dirty slots. Explicit Discard restores the latest confirmed values.
- Retry PARAM_SET only on a missing acknowledgement (three total attempts); a mismatched
  acknowledgement is a definitive mismatch. After an apply failure, stale readback is removed
  so a subsequent fresh read can reconcile partial writes.
- A confirmed PX4 v1.17 sparse-voter gap blocks Apply before the first write and names the lower
  disabled instances. Filler values are not silently rewritten. If topology is unavailable,
  retain existing Apply gates and show that the guard could not be evaluated.
- Apply remains runtime-only. No automatic `param save`, reboot, physical hardware operation, or
  firmware patch is in scope. The earlier “do not block topology” decision remains for CAN2/out-
  of-range topology; it is superseded only for the proven v1.17 voter-gap condition.
