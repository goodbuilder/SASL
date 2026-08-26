# Implementation Plan

## Approval status

Approved on 2026-08-26 16:32 KST for recovery policy 1 only. Automatic reboot, PX4 firmware/config
changes, and `MAV_2_RATE` changes remain outside scope.

Software slices 1–6 are implemented and their focused/full Workstation tests pass. Slice 7 remains
formally blocked only by the repository's 12 pre-existing portability findings; no finding belongs
to this ETH packet. Hardware Slice 8 and conditional documentation Slice 9 are not started.

## Slices and dependencies

상세한 cross-session 실행 순서, 현재 evidence, A/B 분기, hardware closure gate는
`PLAN.md`를 authoritative handoff로 사용한다.

1. Add pure per-side liveness/qualification model and focused transition tests.
2. Add GCS identity and 1 Hz heartbeat lifecycle tests.
3. Replace default request list with consumer-proven Stock profile; retain measurement rates and
   make unsupported custom diagnostics opt-in.
4. Integrate liveness with status projection and remove false host reconnect recovery.
5. Add RTCM enqueue/send-boundary fail-close and per-side counters.
6. Add the approved recovery action and lifecycle/identity guards.
7. Update implementation-facing product contracts, run focused/regression suites, then execute
   P1/P2 hardware soak.
8. Only after three 10-minute cold-boot cycles, a 60-minute dual-link full-load soak, one-side
   isolation, and verified recovery all succeed, update `Pixhawk.md` with the cause, remedy,
   evidence boundary, and residual firmware limitation. On failure, leave product truth unchanged
   and record the failure only in this packet.

## File and component map

- `workstation_core/pixhawk_attitude.py`: identity, heartbeat, timestamps, stream profile, snapshot.
- `device_runtime_controller.py`: stall/recovery orchestration and measurement guards.
- `device_runtime_workers.py`: RTCM send-boundary eligibility.
- `gnss_runtime_controller.py`: routing/readiness/evidence projection.
- `status_model.py`, `workspaces.py`/existing status surface: state/detail/action.
- `tests/test_workstation_pixhawk_gnss.py`, `tests/test_rtcm_pipeline_workers.py`,
  `tests/test_status_model_pixhawk.py`, lifecycle/UI contract tests.
- `README.md`, `Pixhawk.md`, `docs/ui/01_USER_WORKFLOW.md`, `06_STATUS_MODEL.md`,
  `07_COMMAND_MODEL.md`, `09_IMPLEMENTATION_PLAN.md`.

## Migration and rollback steps

No data migration. Commit/slice boundaries keep heartbeat, stream profile, state projection, RTCM
isolation, and recovery reversible. Rollback must not touch user calibration/config or PX4 media.

## Stopping conditions

- Stop before source changes until the exact scope and recovery policy are approved.
- Stop hardware recovery if USB identity is absent/ambiguous or measurement/finalization is active.
- Do not claim resolved if unit tests pass but 10-minute dual-Pixhawk soak fails.
- Do not add a resolved-cause/remedy entry to `Pixhawk.md` before hardware acceptance succeeds.
- If stall recurs after mitigations, preserve evidence and escalate to PX4 firmware patch evaluation;
  do not mask it with endless reconnect/reboot loops.
