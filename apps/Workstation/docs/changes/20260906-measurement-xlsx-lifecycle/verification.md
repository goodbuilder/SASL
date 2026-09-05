# Verification

## Requirement-to-evidence matrix

| Requirement | Evidence |
| --- | --- |
| Poor-quality pairs are measured, undefined geometry is N/A with inputs kept | `test_measurement_geometry_policy.py` (negative depth, large gap, large Δt, class mismatch, multiple candidates, zero disparity, zero baseline, parallel rays) |
| Pairing never drops for Δt / ambiguity / class | `test_detection_pair_coordinator.py` (over-window pairs with `pair_delta_exceeds_window`), `test_field_measurement_pairing.py`, `test_measurement_canonical_evidence.py`, `test_workstation_camera.py::test_noncoordinator_app_pair_path_keeps_100ms_plus_one_ns_as_advisory` |
| Subset submission, per-outcome emission, no timeout, crash → FAILED, per-method backpressure, close → FAILED | `test_method_evaluation.py` (`test_subset_submission_runs_only_requested_methods`, `test_slow_primary_is_never_timed_out_and_completes`, `test_crashed_worker_yields_failed_outcome_and_restarts`, `test_saturated_method_gets_backpressure_while_other_methods_stay_admitted`, `test_slow_method_backlog_only_blocks_that_method`, `test_close_marks_unfinished_methods_failed_instead_of_dropping_them`, `test_bundle_accepts_canonical_subsets_and_verifies_hashes`) |
| Lifecycle: pairs per run, Stop, late outcome to the stopped record only, B starts while A drains, shutdown | `test_measurement_record.py`, `test_measurement_run_controller.py` (`test_start_and_stop_open_and_close_the_measurement_record_after_state_release`, `test_start_never_retires_previous_measurement_workers`, `test_persistence_exceptions_never_gate_start_stop_or_next_run`) |
| Workbook layout, N/A, exact integers, hyperlinks, atomic replace, writer thread, storage failure events | `test_measurement_workbook.py` |
| Standalone reproduction (bit-exact and tolerance) | `test_measurement_workbook_reproduction.py` |
| Telemetry captured per side/tab without downsampling, ns/us as text | `test_measurement_record.py::test_telemetry_rows_route_to_side_specific_sheets_without_downsampling`, `test_measurement_workbook.py::test_detection_calibration_alignment_summary_and_flat_sheets` |
| Storage failure never gates Start / next Measurement / other files | `test_measurement_record.py::test_workbook_allocation_failure_never_blocks_start`, `test_measurement_workbook.py::test_writer_thread_writes_on_stop_and_late_outcome_and_reports_failures`, `test_measurement_run_controller.py::test_real_panel_ignores_old_session_and_prepares_type_owned_paths` |
| Data > Status workbook actions | `test_workstation_ui.py::test_sessionless_data_workspace_exposes_stable_measurement_workbook_actions` |

## Commands

```bash
cd "$SASL_ROOT"
.venv/bin/python -m pytest apps/Workstation/tests/test_measurement_geometry_policy.py \
  apps/Workstation/tests/test_measurement_record.py apps/Workstation/tests/test_measurement_workbook.py \
  apps/Workstation/tests/test_measurement_workbook_reproduction.py -q
.venv/bin/python -m pytest apps/Workstation/tests/test_method_evaluation.py \
  apps/Workstation/tests/test_measurement_run_controller.py apps/Workstation/tests/test_detection_pair_coordinator.py \
  apps/Workstation/tests/test_field_measurement_pairing.py apps/Workstation/tests/test_measurement_canonical_evidence.py \
  apps/Workstation/tests/test_measurement_data_store.py -q
QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest apps/Workstation/tests -q
.venv/bin/python -m pytest apps/StereoWIRIS/tests/test_stereo_geometry_common.py -q
python3 tools/check_repository_policy.py
```

## Results (2026-09-06)

Focused slices pass (geometry 10, record/workbook/reproduction 26, method evaluation 67,
pairing/timing 69, run controller + data store + UI 145). Full Workstation suite
(`QT_QPA_PLATFORM=offscreen`, 7 min 29 s): 1988 passed, 2 skipped, 1 deselected
(`test_baseline_yaw_alignment_ui::test_pixhawk_workspace_shows_separate_baseline_p1_and_baseline_p2_degrees`,
a known load-flaky GPS-age gate test unrelated to this change); the two policy tests that
failed on the first run (generated delegate stubs, neutral colour literals) were fixed and pass.
`apps/StereoWIRIS/tests/test_stereo_geometry_common.py`: 9 passed (shared kernel unchanged).
`./apps/Workstation/run.sh --check`: runtime check ok. `tools/check_repository_policy.py`: PASS.
`tools/check_portable_paths.py` reports 25 pre-existing violations in older docs packets; none in
this packet.

## Hardware-dependent items not verified here

- Real PX4 sample rates versus 30 s workbook checkpoint cost on the field notebook.
- NVENC MP4 segments written under the Measurement prefix on the real camera path.
- Section D evidence for Mountain/Terrain/Drone methods with real optical-pose candidates.
