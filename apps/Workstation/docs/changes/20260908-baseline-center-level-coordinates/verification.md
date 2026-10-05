# Verification

All approved software criteria passed. No hardware, device configuration, or live
widget behavior was changed. Existing unrelated worktree edits were preserved.

| Slice | Status | Evidence |
|---|---|---|
| Coordinate contract | PASS | Left optical RDF -> method midpoint -> body FRD -> frozen P1 NED -> horizontal FRD. Relative methods use the exact -R.T @ T origin. |
| Pure math | PASS | Hand-derived 8-degree pitch, 90-degree roll/yaw, translated/height-offset midpoint, independent origins and common heading. |
| Failure isolation | PASS | Missing/nonfinite source, invalid quaternion, zero baseline, vertical forward return null only for added outputs. Missing P1 does not alter original worker triangulation. |
| Worker/persistence | PASS | Worker result -> canonical primary/retry -> thin/flat method rows -> JSONL and CSV mirror. |
| Workbook/journal | PASS | Coordinate and Method tables plus 90_Flat match stored values. Journal rebuild runs with recomputation forbidden, preserves derived fields and exact WGS84 replay. |
| Compatibility | PASS | Existing WGS84, geometry, recording and UI tests; historical missing fields remain unknown. |
| Diff hygiene | PASS | git diff --check on affected tracked source/schema files. |

Commands (run from Workstation):

```
../../.venv/bin/python -m pytest tests/test_target_level_coordinates.py tests/test_target_coordinates.py tests/test_target_coordinate_storage.py tests/test_method_evaluation.py tests/test_measurement_report.py tests/test_measurement_workbook.py tests/test_readable_measurement_records.py tests/test_measurement_journal.py tests/test_measurement_canonical_evidence.py tests/test_measurement_geometry_policy.py tests/test_measurement_run_storage.py tests/test_measurement_workbook_reproduction.py -q -o cache_dir=/tmp/workstation-level-pytest-cache
```
169 passed. After adding reference evidence directly to thin result rows:

```
../../.venv/bin/python -m pytest tests/test_target_level_coordinates.py tests/test_target_coordinate_storage.py tests/test_target_coordinates_ui.py tests/test_measurement_geometry_policy.py tests/test_measurement_run_storage.py -q -o cache_dir=/tmp/workstation-level-pytest-cache
```
39 passed (overlaps the earlier set). Initial pure test run passed with a warning
that repository-root pytest cache was read-only; subsequent runs use /tmp cache.

Evidence is automated real math, real serialization/XLSX readback, and controlled
fixture/controller/UI execution. No live hardware run, app restart, or field
accuracy claim. The pre-existing attitude warnings/age remain authoritative;
AVAILABLE indicates computability, not fresh or accurate attitude certification.
Legacy preview filtering can differ from raw geometry: the additive range matches
the raw geometric range, not any separately filtered display value.

The lasting contract is in docs/RECORDING_SCHEMA.md. README.md was not modified by
this change; the previously requested README target confirmation remains unanswered.
