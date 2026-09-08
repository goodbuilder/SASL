# Measurement result charts

Final verification after the approved Alignment test follow-up: **2122 passed,
2 skipped**, with no failures. The earlier failure reports below are historical
checkpoints; their resolution is recorded in the follow-up section.

Scope: finish the interrupted addition of twelve native XLSX charts, authorized
on 2026-09-08. The `05_그래프` sheet compares Methods for each existing result
field. The summary links to this sheet. This is a workbook projection of frozen
records; measurement calculations, stored outcomes and live GUI graphs retain
their existing contracts.

## Coordinate contract and sources

- Camera frame: `forward_m`, `right_m`, `down_m`, `range_m`, in metres, retain
  each Method's existing optical midpoint and `baseline_center_frd` axes.
- Horizontal frame: `target_level_forward_m`, `target_level_right_m`,
  `target_level_down_m`, `target_level_range_m`, in metres, retain the Method
  midpoint and pair-frozen P1 horizontal Forward / Right / NED Down axes.
- Geographic outputs: `target_latitude_deg`, `target_longitude_deg` in degrees;
  `target_altitude_ellipsoid_m`, `target_altitude_msl_m` in metres with distinct
  height datums. The charts never infer a missing height or transform coordinates.
- X is elapsed seconds from the run's monotonic start to the registered pair.
  Only finite values from `VALID` / `ESTIMATED` results are plotted. Missing or
  failed results remain gaps. A single point is visible; absent data retains an
  empty, labelled chart. Method colour and dash style remain consistent.

Sources: the accepted twelve-field request, `measurement_record.py` sorted
snapshots, `target_level_coordinates.py`, `target_coordinates.py`, and
`docs/RECORDING_SCHEMA.md`. Independent oracle: distinct hand-authored values
for every field/Method, read back from native chart references in the XLSX ZIP.

## Verification slices

| Slice | Observable output and boundaries | Evidence | Status |
| --- | --- | --- | --- |
| Contract | Twelve fields, three frames, correct units and pair time | Source/schema review above | PASS |
| Logic and XLSX boundary | Field references, status gaps, absent coordinates, one point, stable Method style | Final focused run: 19 passed, including the axis/hidden-data checks | PASS |
| Persistence and integration | Atomic export, journal rebuild and exact values survive extra charts | Final workbook, target-storage and journal run: 19 passed | PASS |
| Visual output | Summary navigation; all twelve native charts render with readable axes/legends | Corrected export: all three chart pages, single-point/empty first pages and summary inspected | PASS |
| Relevant regression | Workbook/report/recording behavior after adding charts | Relevant tests passed; wider workspace results separately below | PASS |
| Operations | Sheet order, usage, missing values and chart-only sampling documented | Workstation README and recording schema; diff and scoped portability checks | PASS |

The interrupted run's temporary previews were lost during reboot. The resumed work
regenerated them with the production exporter and completed fresh verification.
No live device operation was used for these projections of recorded coordinates.

Initial rendering found two presentation defects: automatic axis crossing placed
elapsed-time labels inside negative Down plots, and an empty chart received a
meaningless numeric scale. Time labels now remain below the plot, the horizontal
axis crosses at the minimum Y, and empty charts hide their axes. Native chart
settings follow the [XlsxWriter axis API](https://xlsxwriter.readthedocs.io/chart.html#chart.set_x_axis).
The XLSX tests also verify hidden-sheet chart values remain visible to consumers.

## Fresh evidence on 2026-09-08

Run from `apps/Workstation` using the repository virtual environment:

```bash
PYTHONPATH=..:../../common/python ../../.venv/bin/python -m pytest -q -o cache_dir=/tmp/workstation-12-charts-resume-0brbonf_/focused-cache tests/test_measurement_result_charts.py tests/test_measurement_report.py tests/test_measurement_workbook.py tests/test_target_coordinate_storage.py tests/test_measurement_journal.py
```

Result after the final code change: **19 passed in 2.71s**. Field/Method sentinel
values independently verify native chart references, blocked-result gaps,
per-field availability, precise geographic formats, one-point markers, blank
slots, stable styles and visibility of the hidden backing data. Export/rebuild
tests cover original numeric values, row continuation and atomic publishing.

Visual QA used synthetic frozen records with ten Methods, thirty pairs, a blocked
pair, negative Down values and unavailable MSL; additional single-result and empty
records cover the boundary views. The existing application writer produced each
XLSX. QA-only copies exposed the graph sheet for LibreOffice headless conversion;
Poppler rendered the resulting three-page A4 landscape PDF. All twelve charts
were inspected after the axis fix, with no clipped titles, axis labels or legends.
The summary's chart link and the single-point/empty first-page views also passed.
This verifies the LibreOffice rendering and native OOXML references, not a live
Microsoft Excel session or sensor accuracy.

`git diff --check` and the repository portable-path scanner applied to the seven
resumed implementation/test/document files passed. The source, tests and this
record remain in the worktree; no commit was created. New required files from the
interrupted work include `workstation_core/measurement_report.py`,
`tests/test_measurement_report.py` and `tests/test_measurement_result_charts.py`;
they are currently untracked and must be included when the work is committed.

## Wider workspace test results

```bash
PYTHONPATH=..:../../common/python ../../.venv/bin/python -m pytest -q -o cache_dir=/tmp/workstation-12-charts-resume-0brbonf_/pytest-cache
```

Full run: **2118 passed, 4 failed, 2 skipped in 155.72s**. This was the broad
baseline check; the final chart-only axis changes then passed the focused run
above. Failure triage:

- `tests/test_drone_network_discovery.py::test_passive_discovery_reports_source_port_listen_port_and_sysid`:
  sandbox denied UDP socket creation; passed outside the sandbox.
- `tests/test_workstation_ntrip.py::test_gga_provider_uses_the_operator_selected_gps2_source`:
  its two wall-clock GGA timestamps straddled a 0.01-second boundary; passed on retry.
- `tests/test_alignment_live_rt_ui.py::test_baseline_setup_keeps_live_rows_and_return_keeps_readout`:
  still fails because index 1 now selects Baseline Yaw90, so the test does not leave
  that page as intended after the earlier tab reorder.
- `tests/test_workstation_lifecycle.py::test_workstation_shell_navigation_and_close`:
  still expects Horizon first and Baseline Yaw90 last, while the approved earlier
  reorder puts Mountain first and Baseline Yaw90 second.

Exact retry command (outside the sandbox for the UDP test):

```bash
PYTHONPATH=..:../../common/python ../../.venv/bin/python -m pytest -q -o cache_dir=/tmp/workstation-12-charts-resume-0brbonf_/retry-cache --tb=short tests/test_alignment_live_rt_ui.py::test_baseline_setup_keeps_live_rows_and_return_keeps_readout tests/test_drone_network_discovery.py::test_passive_discovery_reports_source_port_listen_port_and_sysid tests/test_workstation_lifecycle.py::test_workstation_shell_navigation_and_close tests/test_workstation_ntrip.py::test_gga_provider_uses_the_operator_selected_gps2_source
```

Retry at the chart-completion checkpoint: **2 passed, 2 failed**. The two
Alignment expectation failures were outside the original workbook request, so
those files were unchanged at that checkpoint. The suite was not yet green.
The subsequently approved test fix is recorded below. The R,t-profile removal
remains on user hold.

Coordinate contract checked: camera midpoint FRD, Method midpoint / P1 local-level
FRD, WGS84 degrees with separate ellipsoid/MSL metres, and monotonic elapsed seconds.

## Approved Alignment test follow-up

The user subsequently authorized fixing the two stale Alignment tests. Scope is
test expectations and navigation only. Source of truth: the accepted order is
Mountain + GNSS, Baseline Yaw90, Horizon, Terrain PnP, Drone Rotation, Drone
Multi-view, Combine. A test returning from Baseline setup must select the target
Method by its label and assert it left setup before checking readout placement.

Slices: expectation/selection changes PASS; focused navigation/readout tests
PASS; full regression PASS. No production UI, coordinate transform, R,t value,
device operation or stored record changes. README.md update not required for
this change.

`test_baseline_setup_keeps_live_rows_and_return_keeps_readout` now finds Mountain
by tab label and asserts both entry into and exit from the setup pane.
`test_workstation_shell_navigation_and_close` checks the approved complete order.
The live-readout placement/status and navigation/shutdown assertions remain intact.

```bash
PYTHONPATH=..:../../common/python ../../.venv/bin/python -m pytest -q -o cache_dir=/tmp/workstation-alignment-test-followup-cache tests/test_alignment_live_rt_ui.py tests/test_workstation_lifecycle.py
```

Focused result: **39 passed in 4.92s**. Full suite command from the Workstation
directory, outside the sandbox so the existing local UDP test can create its socket:

```bash
../../.venv/bin/python -m pytest -q -o cache_dir=/tmp/workstation-alignment-test-followup-cache --tb=short
```

Full result: **2122 passed, 2 skipped in 161.99s**. Both previously failing
Alignment tests now pass, together with the final twelve-chart export tests.
`git diff --check` and a portable-path scan of the two test files and this record
also passed. `tests/test_alignment_live_rt_ui.py` remains an untracked source file
from the earlier live-R,t work and must be included when committing that work.

Coordinate contract checked: the follow-up changes only test tab selection/order;
live optical RDF R,t readouts and all coordinate assertions remain in place.
