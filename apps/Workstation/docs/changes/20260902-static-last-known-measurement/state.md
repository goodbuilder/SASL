---
schema: modify-gui-change-state-v1
change_id: 20260902-static-last-known-measurement
status: approved
mode: backend-only-gui-contract
updated: 2026-09-03
baseline_ref: 3288aee5f5fb7af5ab30024f93e838f76724a0b0
last_completed: "Implemented symmetric attitude/GPS retention; focused and regression tests pass"
next_question: "Field verification with sequential P1/P2 USB-C connection"
read_next:
  - docs/changes/20260902-static-last-known-measurement/change-brief.md
  - docs/changes/20260902-static-last-known-measurement/current-behavior.md
  - docs/changes/20260902-static-last-known-measurement/decisions.md
  - docs/changes/20260902-static-last-known-measurement/impact-analysis.md
affected_product_docs:
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/07_COMMAND_MODEL.md
  - docs/ui/10_DESIGN_AUDIT.md
blocking_questions: 0
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "Automatic process-local last-valid Pixhawk attitude/GPS fallback exactly as approval-summary.md"
  conditions: []
  recorded_at: "2026-09-02"
---

# Resume Notes

README target confirmed as `./README.md` and updated, along with `docs/ui/01_USER_WORKFLOW.md`,
`06_STATUS_MODEL.md`, and `07_COMMAND_MODEL.md`.

Implemented in `measurement_view_controller.py`:

- `_attitude_state_is_valid_for_measurement_cache()` / `_cached_measurement_attitude_states()`
  mirror the existing GPS cache, giving both sensor halves the same retention rule.
- `_measurement_live_readiness()` consumes the retained attitude selection; the
  `attitude not received in current Measurement` hard block is gone and reuse is an advisory.
- `_freeze_measurement_pair_inputs()` keeps the time-matched `attitude_buffers.nearest()` sample
  as the preferred per-detection source and falls back to the retained sample only when the
  buffer yields nothing; attitude ages are computed against the detection timestamp in both
  cases; `reused_inputs` merges GPS and attitude provenance.
- `_gps_state_is_valid_for_measurement_cache()` follows the configured `minimum_fix_type`.

Sequential-connection follow-up (P1/P2 cannot share USB at the same time):

- `_measurement_correction_frame_token()` binds the retained GPS pair to the correction source
  identity, the Base coordinate profile hash, and the observed RTCM 1005/1006 reference station;
  `_cached_measurement_gps_states()` drops a side when that token changes and refuses to re-admit
  a sample observed before the change.
- `gnss_protocol.parse_rtcm3_reference_station_arp()` decodes 1005/1006 into exact 0.1 mm
  integers and `_record_gnss_base_rtcm_frame()` publishes `panel.latest_rtcm_reference_station`.
- `_measurement_live_readiness()` reports the retained pair's capture span as an advisory once it
  exceeds the configured `maximum_left_right_timestamp_delta_ms`. It never blocks.
- `gnss_runtime_controller._resolve_gga_sentence()`: `PER_ROVER_SESSION` now sends the GGA from
  whichever rover currently delivers a fresh position (left preferred), so a VRS caster keeps
  issuing RTCM while only one board is connected. The operator must select `세션별 rover` as the
  GGA policy; `SYSTEM_CENTER` cannot work on a rig that connects one board at a time.

Remaining work is hardware verification, which must be reported separately from automated tests.
It must cover a sequential P1-then-P2 fix under one unchanged Base session, and confirm on
hardware that a P2 session against the VRS mountpoint reports a different RTCM reference station
than the P1 session did (the caster behaviour was measured off-rig on 2026-09-03; the per-session
GGA difference between the two boards has not yet been observed end to end).
