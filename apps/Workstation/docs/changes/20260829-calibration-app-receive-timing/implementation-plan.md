# Implementation Plan

## Approval status

Approved by the user on 2026-08-29, implemented, and verified.

## Slices and dependencies

1. **Contract / PASS**: added host-receive proxy class and invariants.
2. **Pure timing / PASS**: exact/owned compatibility receive time, signed delta, threshold, failures.
3. **Persistence / PASS**: per-side receive ns, exact signed ns, basis, class, flags and detail in capture JSON.
4. **Input snapshot / PASS**: proxy non-precision semantics and mixed/legacy compatibility.
5. **UI / PASS**: existing tooltips/contracts/status wording updated without layout changes.
6. **Focused integration / PASS**: manual, Auto, worker JSON, FTP/exposure preserved, legacy read.
7. **Documentation / PASS**: approved product docs, workflow and confirmed `./README.md` updated.
8. **Regression/application / PASS**: compile, diff, launcher and clean full suite pass;
   `1630 passed, 1 skipped` after the user closed the independent Workstation process.

Each dependent slice starts only after the previous required slice passes.

## File and component map

- `calibration_controller.py`
- `workstation_core/calibration_timing.py`
- `workstation_core/calibration_input.py`
- `workspaces.py`, `standalone_gui.py`
- `tests/test_workstation_calibration.py`, `tests/test_calibration_input_snapshot.py`,
  `tests/test_wiris_ftp_capture_source.py` as required
- `README.md`, `docs/ui/01_USER_WORKFLOW.md`, `docs/ui/06_STATUS_MODEL.md`,
  `docs/ui/09_IMPLEMENTATION_PLAN.md`
- this change packet

## Migration and rollback steps

No migration. Existing files are read-only. Rollback reverts only the approved scoped hunks; newly written JSON
continues to expose its allow/reject booleans to the old pair-usage reader.

## Stopping conditions

- Exact app-receive origin cannot be established at the native-frame boundary.
- Implementation would require raw RTP capture/mapping, a new worker, setting/schema migration, or FTP/Measurement
  behavior changes.
- Dirty overlapping hunks cannot be preserved safely.
- Focused test failure reveals a material unapproved behavior change.
