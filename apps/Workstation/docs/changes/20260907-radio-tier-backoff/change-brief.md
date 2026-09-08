# Radio optional-stream tier backoff and liveness log noise

## Symptom
`run.log` on 2026-09-07 (Radio 57600 links, P2 and Drone):
- `Radio optional stream tier=` cycled CRITICAL → OPTIONAL_25 → OPTIONAL_50
  → STOCK → `ATTITUDE STALE` → CRITICAL about once a minute, forever. Every
  cycle re-probed the tier that had just saturated the link, so ATTITUDE
  dropped again and RTCM/shell traffic shared a link in recovery most of the
  time (14 Drone `ATTITUDE STALE` lines in 25 minutes).
- `오른쪽 MAVLink liveness: QUALIFYING; qualification=00:10/00:10` followed
  by `... liveness: LIVE` twice per 2 s sensor_mag shell poll (74 `LIVE`
  lines in one session) although the side never lost qualification.

## Root cause
- `RadioStreamPolicy` had no memory of which tier failed: after any
  regression it restarted from CRITICAL and climbed back with the same
  30 s / 15 s dwell, re-requesting the failed tier's optional rates.
- The shell poll opens its exclusive command with
  `preserve_qualification=True`; the liveness model still reports
  `QUALIFYING / ACTIVE_COMMAND` while the command owns the receive path, and
  `_emit_liveness_transition` logged every such blip as a state change.

## Change
1. `workstation_core/pixhawk_radio_policy.py`: a regression observed while
   tier T was active sets `tier_cap = T - 1`. Escalation past the cap needs
   `RADIO_CAP_LIFT_S` (300 s) of continuous healthy dwell at the cap instead
   of the ordinary 15 s (30 s from CRITICAL), and lifts the cap one tier.
   `reset()` (new transport/connection) forgets the cap; the snapshot carries
   `tier_cap` and `next_transition_in_s` reflects the longer dwell.
2. `status_model.py`: the Radio detail line shows `optional-cap=<tier>` while
   a cap below STOCK is holding escalation back.
3. `workstation_core/pixhawk_attitude.py::_emit_liveness_transition`: a
   LIVE → QUALIFYING transition whose reason is `ACTIVE_COMMAND` and whose
   qualification elapsed is still complete is not logged (and not recorded as
   the last state), so the return to LIVE is silent too. Commands that reset
   qualification (PARAM readback) still log `qualification=00:00/00:10`.
