# Verification

## Requirement-to-evidence matrix

| Requirement | Evidence |
| --- | --- |
| All 16 fixed JSONL streams have paired CSV paths | mapping test + snapshot/UI table test |
| CSV row preserves the canonical JSONL record | independent JSONL/CSV parse and `record_json` equality |
| JSONL remains authoritative on CSV failure | broken CSV path test; Measurement stays available |
| Retry/restart/close are bounded and recoverable | storage lifecycle/retry tests |
| UI exposes path/state without layout/focus regression | offscreen Qt model/state/geometry tests |
| Coordinates/clocks are copied, not converted | schema/fixture review and existing coordinate tests |

## Focused test evidence

Planned focused commands after approval:

```bash
source .venv/bin/activate
pytest -q apps/Workstation/tests/test_measurement_data_store.py
pytest -q apps/Workstation/tests/test_workstation_ui.py -k 'data or storage or measurement'
```

Add independent fixtures for nested/list fields, large integers, Unicode, empty streams, existing
valid headers, foreign files, JSONL failure, CSV failure/retry, restart append, queue overflow,
and shutdown.

## Regression and application evidence

Run the Workstation affected regression suite and the documented launcher check from the repository
root after focused slices. Preserve warnings/skips and distinguish fake/offscreen UI evidence from
real file-system behavior.

## Performance, load, and target-hardware evidence

Measure a bounded burst of high-rate camera/PX4/GNSS records with and without CSV mirroring for
enqueue latency, written/dropped counts, queue depth, disk time, and shutdown. No camera, Pixhawk,
GPU, or field hardware claim is required for this file-format slice.

## Feature isolation and contract evidence

Break a CSV path while writing a priority `measurements` row and verify JSONL/Measurement remains
available. Break one bulk mirror and verify unrelated streams continue. Check that the UI consumes
only the public storage snapshot and that existing latest XLSX generation still reads JSONL.

## Visual authenticity and dynamic-layout stability evidence

Inspect the existing Data table at 1920×1080 and 1366×768/offscreen with normal, retrying, failed,
long-path, and recovery states. Assert fixed status/table geometry, unchanged button positions,
focus, selection, and scroll.

## Visual and hardware/manual evidence

Visual/manual file opening is useful but not required for software acceptance. Hardware/manual
camera/Pixhawk/GNSS evidence is out of scope; CSV mirroring copies already-produced records.

## Residual conditions

CSV is not transactionally atomic with JSONL across a process crash, and historical JSONL is not
backfilled in this slice. JSONL remains the recovery oracle. Any residual writer overhead or mirror
drop behavior must be recorded from fresh tests rather than inferred.

## 2026-09-04 implementation evidence

| Slice | Command / evidence | Result |
| --- | --- | --- |
| Storage contract and persistence | `../../.venv/bin/python -m pytest -q tests/test_measurement_data_store.py` | PASS — 31 tests; fresh CSV, same-stem mapping, UTF-8/nested `record_json`, restart append, invalid header, independent failure/retry, bounded CSV drops, and controller boundary |
| Data Status projection | `../../.venv/bin/python -m pytest -q tests/test_workstation_ui.py -k 'data or storage or measurement'` | PASS — 8 tests; CSV path/state/drop columns and degraded-but-available behavior |
| Directly coupled XLSX/lifecycle/evidence consumers | `../../.venv/bin/python -m pytest -q tests/test_latest_measurement_xlsx.py tests/test_recording_controller_save_lifecycle.py tests/test_measurement_canonical_evidence.py` | PASS — 53 tests |
| Syntax and diff checks | `../../.venv/bin/python -m py_compile workstation_core/measurement_data_store.py workspaces.py tests/test_measurement_data_store.py tests/test_workstation_ui.py`; `git diff --check -- <affected files>` | PASS |
| Workstation regression suite | `../../.venv/bin/python -m pytest -q` from `apps/Workstation` | 1,899 passed, 29 failed, 2 skipped; failures are pre-existing dirty-worktree calibration/network/telemetry/settings API mismatches, with no failure in the focused CSV/storage tests |
| Repository-wide regression | `.venv/bin/python -m pytest -q` from repository root | BLOCKED at collection by duplicate `test_px4_attitude_cal_manager` module names between StereoWIRIS and Workstation; repository root pytest cache is also read-only |
| Portability check | `/usr/bin/python3 tools/check_portable_paths.py` | FAIL — 21 pre-existing absolute/dated-path findings in user-owned runtime evidence and other change packets; no new machine-specific path is required by the CSV implementation |

The implementation intentionally does not rewrite or backfill existing JSONL. New sessionless
records create same-stem CSV files under the resolved data root, while JSONL remains authoritative.
