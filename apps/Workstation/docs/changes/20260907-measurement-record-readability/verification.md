# Verification

## Result

Implementation and affected-workflow verification passed. No real camera, Pixhawk, GNSS or logger commands were issued for these tests.

| Slice | Status | Independent evidence |
|---|---|---|
| Start identity | PASS | Integer epoch boundary/collision fixtures; whole-run suffix reservation |
| Raw routing | PASS | Original/next-run file contents, late append/replace index allocation, nested identity and long Unicode values |
| CSV recovery | PASS | Rebuild after retry-reserve overflow; RTCM index repair preserves binary bytes and does not duplicate next append |
| Journal/rebuild | PASS | Rebuild with geometry derivation deliberately disabled; read-only interrupted journal; restarted controller chooses collision successor |
| Binary outputs | PASS | Native PNG, RTCM bytes and ULog bytes verified independently; all owned paths carry the original stamp |
| Workbook | PASS | All ten Method tables agree with dedicated R,t records; expanded columns and links checked in XML; row-limit continuations retain exact reproduction |
| Data UI | PASS | Real Qt widget with fake backend: click, busy, error, stable geometry and status height |
| Visual | PASS | Synthetic 39-sheet workbook rendered with Artifact Tool; all sheets inspected, plus final Method/Rt/Definitions regions |
| Documentation/paths | PASS for changed scope | README and schema/status/command docs updated; 60 changed-scope files have zero portable-path violations |

## Commands and results

- `../../.venv/bin/python -m pytest -q -p no:cacheprovider tests`
  - Final full suite: **2044 passed, 2 skipped in 378.26s**.
  - Earlier full suite: 2041 passed, 2 failed, 2 skipped. Corrected the report palette to the existing gray/red theme and aligned an old settings-test expectation with the already-existing RuntimeError prefix. Production command error behavior was preserved.
- Final presentation follow-up: `../../.venv/bin/python -m pytest -q -p no:cacheprovider tests/test_workstation_ui.py tests/test_measurement_workbook.py tests/test_measurement_report.py tests/test_measurement_workbook_reproduction.py`
  - **43 passed in 45.80s**, after clarifying REBUILD FAILED and wrapping long definitions.
- Additional focused binary/report/exact checks: **9 passed**.
- `./run.sh --check`: **PASS**. The host reports unavailable NVIDIA driver / optional nvh264dec and selects the existing CPU decoder fallback. This task did not change the host driver.
- `git diff --check -- .`: **PASS**.
- `../../.venv/bin/python ../../tools/check_portable_paths.py`: existing repository-wide **26 violations** in historical documentation/Pixhawk configuration. These are outside this change and were preserved.
- The same checker on this task's 60 source/test/document paths: **PASS, zero violations**.

## Visual evidence

Synthetic data only; no field-measurement conclusions are drawn. Rendering used the supplied cached Artifact Tool, not a replacement application export engine. The app continues using XlsxWriter.

All 39 sheets were rendered and visually inspected. Summary/comparison, metric formats, missing markers, method titles, R,t, source tables, original integer text, exact values, provenance and definitions were inspected. Artifact Tool's imported preview can display numeric-looking text timestamps in scientific notation; original XLSX XML and the independent stdlib reader verify that the saved nanosecond cells remain exact strings. No spreadsheet data formulas/errors were introduced; the renderer's error scan matched zero cells.

Temporary QA files are under `/tmp/measurement-workbook-qa/`; representative images are copied into this packet's `evidence/` directory. Data UI screenshots use a controlled fake backend and fixed viewport. Normal and error-state geometry was verified programmatically.

## Coordinate contract

Canonical R,T: optical RDF, `X_right = R_right_from_left * X_left + T`, T in right-camera metres. Kernel inverse uses `R^T` and `C=-R^T T`. Result coordinates remain `baseline_center_frd`. GNSS/Pixhawk NED, body FRD and binary ULog contents retain their producer contracts. Tests verify distinct off-diagonal rotation entries, signed translation components, and bit-exact workbook reproduction including continuation sheets.

## Limits

Existing bounded-queue/storage failure behavior remains non-gating. Missing unrecoverable source records are reported. The two full-suite skips and real-device/driver qualification are not represented as hardware passes. Existing repository-wide path-policy violations remain outside the approved scope.

Representative views: [Summary](evidence/xlsx-summary-synthetic.png), [expanded Method R,t](evidence/method-rt-expanded.png), [R,t table](evidence/rt-table.png), [definitions](evidence/definitions.png), [Data rebuild error](evidence/data-rebuild-error.png).
