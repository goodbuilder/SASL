# Report workbook supplement

## Output and scope

- Input: `apps/Simulation/recordings/stereo_v1_1/paper1_v1_2_001/outputs/20261021-postfreeze-summary-review/stereo_v1_2_summary_postfreeze_reviewed.xlsx` (SHA-256 `17a2d78240ad99e91b707323b62d9169e0d7d51d0fdf357467638d8c37f032bf`).
- Output: sibling `stereo_v1_2_summary_report_ready.xlsx` (SHA-256 `999587fd1457c53b95d7d9f84f06abaa420ddeb23781b83d41438b8d1ea86a9c`). Input and frozen analysis files were not changed. No simulation or calibration rerun was performed.
- Added `11_방법계약`, `12_P2선정추적`, `13_QC시간간격`, `14_P95짝비교`; appended five rows to `10_계산추적`; clarified three labels in `06_리뷰판정`.

## Method and result contracts

- P2 score follows frozen `analyze_campaign.py:1645-1700`: each converged G0 trial-0 RPY_IMU calibration result yields the arithmetic mean of five V0-target predicted 3D sigma values; a path's score is the median of 15 run×method means. Tie order is average reference duration, then path ID. Workbook formulas use `AVERAGE`, `MEDIAN`, `LARGE` and source-value differences. Frozen covariance propagation outputs remain fixed inputs; Excel does not rerun that solver. Selection is not a numeric performance pass or a claim of global optimality.
- QC cadence values come from each selected run's original `qc_report.json`. The 30 ms simulation gap threshold is a diagnostic warning, while full reference-slot coverage and science-window coverage are separate checks.
- Paired P95 uses `attitude_variant_comparisons.csv`, with `COMPARABLE_CASE_METRIC` as the comparable denominator. Unavailable cases remain in the total-case count. Group means and medians are fixed code-derived aggregates; the improved share is an Excel formula.
- Positions and target geometry use world ENU `[east,north,up]` metres. Optical RDF, camera-link FLU, PX4 body FRD and PX4 local NED are documented separately; the supplement does not transform stored values between frames.

## Verification

- Reopened and rendered the four added sheets; inspected labels, units, numerical displays and formula outputs.
- Five selection scores and P95 values reproduce frozen `path_selection.json` within `1e-10` m; P2 remains first. The 39 QC rows give 39 covered science windows, zero missing full reference slots, 30 diagnostic 30 ms exceedances, 36 ms maximum simulation gap and 52 ms maximum left/right camera gaps.
- Independent Python CSV calculation for existing-yaw G0-P2 all_data A gives 3,500 comparable cases, 2,331 lower P95, 1,169 higher P95, mean delta `-0.016194940640086707` m and median `-0.016843944208714712` m; workbook row 17 matches.
- OOXML comparison: original 13 sheets' cell contents are unchanged except `06_리뷰판정!A24,D26,E26` and appended `10_계산추적!A14:G18`. Chart count remains 22; all 17 sheets have zero error-typed cells.
- `python3 tools/check_portable_paths.py`: PASS. The linked three-reviewer HTML has zero broken local links. The output XLSX is inside the repository's ignored `recordings/` tree, so it must be copied explicitly when sharing or moving machines; the builder source and this ledger are untracked until added to Git.

## Limits

- The new method sheet describes the frozen design; it does not exhaustively audit every MC trial's shared sensor-observation hash.
- PX4 RNG and independent physical-flight repetition remain unverified. Empirical MC percentiles are not confidence intervals for independent flights.
- The full large checksum manifest and raw video corpus were not rehashed for this supplement.

## Review-driven v2

- Output: sibling `stereo_v1_2_summary_report_ready_v2.xlsx` (SHA-256 `d6d69fa9bd48c9c1041966d44d5c45675936a15d375cc928314a890d613d58a8`). The first `report_ready.xlsx` remains as the independently reviewed predecessor.
- `12_P2선정추적` now labels the five-target support value as the second-largest sigma rather than a conventional P95. It shows A/B/C medians for every path and a live `combined−A` difference. For the frozen data, every difference is zero; B and C individually favor P4 over P2. The detailed 75-row table moved from rows 20–94 to 30–104, and `10_계산추적` follows the new addresses.
- `11_방법계약` distinguishes the RTK-bracket requirement for the common image-pair mask from C's RTK-free estimator input, and distinguishes synthetic sensor truth, estimator input and V0 GT evaluation. `13_QC시간간격` names `counted_in_campaign` as the inclusion flag.
- `00_요약` now calls final evaluation a metric-output completion without a numeric accuracy pass. The GPS paired graph/table names its improved share as 3D RMSE, while the P95 direction remains on `14_P95짝비교`. `99_정의` no longer promises that raw data is delivered with the workbook. The three current explanatory HTML pages link to v2, and the V0 posture-error wording in `stereo-calibration-explained.html` is corrected.
- Reopened and rendered the changed summary, method, selection, selection-detail and GPS graph areas. OOXML inspection found 17 sheets, 22 charts and zero error-typed cells. Comparing v2 with its predecessor found changed cells only in the intended summary, graph labels, definition, review note, Formula Map, method, selection and QC-note areas; the paired P95 sheet and numerical result tables remain unchanged.
- `python3 tools/check_portable_paths.py`: PASS. The three changed HTML pages have zero broken relative links. Frozen analysis inputs and earlier XLSX files were not modified; no new SITL, calibration or MC run was performed.

## Independent v2 re-review

Three read-only reviewers used the common `reviewer.toml` and the v2 workbook plus explicitly assigned frozen analysis, selected QC/GT evidence, selection code and explanatory HTML. Prior review reports were not shared. The review report is `apps/Simulation/docs/changes/20261021-three-reviewer-results/supplement_v2_review_report.html`; all four of its local links resolve.

- No confirmed arithmetic discrepancy: 75 selection-detail rows, 39 QC rows, 96 P95 paired groups from 277,200 CSV rows, 4,664 MC distribution groups and 9,788 GT-invalid frame rows matched assigned sources within their stated calculation scopes.
- Remaining interpretation issues: P2/P4 selection score difference is small and the combined median equals method A's median for all five paths; P0 has lower all-data A/B V0 RMSE and shorter reference duration than P2. Route-level plots should show the five physical run pairs and distinguish MC variability from flight variability.
- Remaining documentation issues: specify the two tracking P95 source definitions and relative-path roots; repeat denominators when exporting selected charts. The details of the earlier `감사 통과` label and physical-flight stochastic independence were not verified in the assigned review scope.
- The v2 workbook was not edited after the three reviewers began. It remains under the ignored `recordings/` tree and must be copied explicitly for handoff.

## Existing-data v3 improvement

- Output: sibling `stereo_v1_2_summary_report_ready_v3.xlsx` (final SHA-256 `b2c7cf8b66c3eb3fc214999bdaa21b43f9129a0de42a6e14559d93c31778a361`). The v2 input remains at `d6d69fa9bd48c9c1041966d44d5c45675936a15d375cc928314a890d613d58a8`. The frozen analysis and raw recording inputs were not changed.
- Added `15_선정민감도`: each of the five G0 repetitions is excluded from the trial-0 A/B/C prediction set in turn; 12 values remain per path. P2 ranks first in two exclusions and P4 in three. This is post-freeze sensitivity, not a replacement path selection or evidence of independent flights.
- Added `16_실행별V0`: 100 G0 groups (= 5 paths × 5 repetitions × A/B × two scopes), each averaging 7 yaw × 100 MC case rows within the corresponding calibration/V0 run pair. Added a 5-run plot and paired P0/P2 run differences. A separate Python CSV calculation for G0-P2-R03/A/all_data gave 700 rows and mean 1.2144334700790311 m; the saved XLSX gave 1.2144334700790325 m.
- Added `17_추종P95출처`: original workbook recorded flight-state P95 and the original GT-alignment recomputation are shown side by side. Thirty-one runs have both; eight V0 source summaries intentionally do not expose the pre-freeze tracking metric. Both metrics use world ENU 3D distance and type-7 P95. The recorded source reads saved flight-state GT/reference and rejects GT offsets >40 ms; the alignment source interpolates raw GT at science control timestamps. The two values must not be silently substituted.
- `05_그래프` now states the full-frame versus science-pair detection denominators, points route means to the run-level sheet, and states that P2 is not the observed all-data time–RMSE leader. `10_계산추적` names the Git-root and campaign-root relative path conventions and traces the three new sheets. `00_요약!C10` limits the audit PASS claim to the actual coverage/schema/provenance audit record, whose SHA-256 matches `98_출처!B6`.
- Reopened and rendered summary, detection denominator, run-level plot and table, leave-one-out selection and two-P95 source views. OOXML inspection found 20 sheets, 23 charts and zero error-typed cells. The new chart binds only to `16_실행별V0!A5:E9`. Previous 17 sheets' cell contents differ only at intended summary/graph/definition/formula-map locations. Three method/operations HTML pages now link to v3 and have zero broken relative links. `python3 tools/check_portable_paths.py`: PASS.
- No SITL, calibration, MC generation or new physical repeat was run. Source per-case evaluation outputs and QC values were reaggregated or displayed without changing the frozen analysis.

## Independent v3 review and targeted correction

Three read-only reviewers used `reviewer.toml`, the v3 workbook and explicitly assigned frozen analysis, source QC/GT, production code and explanatory HTML. They found no confirmed arithmetic error in the new 75-value leave-one-repetition sensitivity, 100 run-level V0 aggregates, 39 recorded tracking P95 values or 31 available recomputed tracking P95 values. The review report is `apps/Simulation/docs/changes/20261021-three-reviewer-results/supplement_v3_review_report.html`.

One reviewer identified a label error in the initial v3 chart 11: it included G0 paths with 5 run pairs × Y0–Y6 and G1 diagnostic paths with 3 run pairs × primary-zero, while the title said only five pairs. The final v3 title and `05_그래프!A91,A100` distinguish those groups, and `16_실행별V0` is labeled G0-only. The same review found that `15_선정민감도` omitted a clarification of the per-path omission and of the unused tie-break; both were added. `10_계산추적!A2` now includes the Git-root-relative `13_QC시간간격!K` column. Two reviewers re-read the final file at the final SHA-256 and confirmed these specific corrections against chart data and path prefixes.

Remaining scientific limits: the original P2–P4 score difference is about 0.073 mm, P4 wins three of five leave-one-repetition conditions, and P2 has higher all-data A/B RMSE than P0 in each of five paired run IDs. The workbook presents these as descriptive frozen-data findings. Independent physical-flight repetition, PX4 RNG effects, and frame-ID identity across paired attitude cases remain unverified.

## V0 start-coordinate guard (pending workbook rebuild)

- The v1.2 V0 science reference starts at world ENU `[E,N,U]=[0,600,218.382]m` and ends at `[0,400,145.588]m`, using `U=N*tan(20°)`. N=600m is the science-window start target; entry and stabilization occur before it. This does not claim every recorded run reached `science_start`.
- `build_report_workbook_v3.mjs` now checks the frozen target frame and exact northing sequence `[600,550,500,450,400]`, and prints the V0 start/end points on `16_실행별V0`.
- The v3 SHA-256 above identifies the previously rendered workbook. This builder update has not been rendered: the ignored campaign `recordings/` inputs and the Node `@oai/artifact-tool` package are absent from this checkout.

## Report v3 execution path

- Added the RQT/control action `report_v3` and `build_report_workbook_v3.sh`. It is pinned to the archived `paper1_v1_2_001` report and requires the v2 workbook plus the `analysis_9cfd7c6cd10b` freeze inputs. The JavaScript builder verifies `analysis_latest.json` still selects that exact freeze and that its freeze hash matches before aggregating.
- The XLSX is exported to a process-specific temporary sibling and atomically renamed to `stereo_v1_2_summary_report_ready_v3.xlsx`; the v2 source is read-only. This is a report-only action and does not run SITL.
- The current checkout cannot rebuild it: those ignored campaign inputs and the Artifact Tool runtime are absent. A second run of the 39-slot launcher, with session-only overrides for the available ROS/PX4 workspaces, stopped before live captures. The installed world and Simulation overlays are absent; the selected controller Python cannot import `px4_msgs`; the isolated YOLO environment cannot import `scipy`. The repo-owned YOLO model now passes its pinned SHA-256 check. `nvidia-smi` cannot reach an NVIDIA driver, while the frozen campaign config requires `cuda:0`; the CPU-only fallback was not selected. No `run_status.json` or attempt directory was created, and no manifest slot started.
- The active filesystem had 14,179,639,296 bytes free. The full-campaign storage check remains `NOT_MEASURED_BECAUSE_STATIC_PREFLIGHT_FAILED`, because the required short raw-capture probe was skipped. No campaign storage estimate or v1.2 data collection is claimed.
- At the time of this entry, `report_v3` remained report-only for archived analysis `analysis_9cfd7c6cd10b`, and fresh `xlsx_all` output was not connected to V3. The later “Fresh SWEEP-to-V3 implementation” section records the new campaign flow; the archived route remains separate.
- Coordinate contract checked: the v1.2 manifest requires world ENU `[east,north,up]` metres and V0 science targets from `[0,600,218.382]` to `[0,400,145.588]` m, with `U=N*tan(20°)`. N=600 m is the planned science-window start after entry and stabilization. The report builder requires northing targets `[600,550,500,450,400]`; it performs no coordinate transform and does not assert that a recorded run reached N=600 m.

## Latest non-flight runtime recovery — 2026-10-21

This section updates the earlier runtime status above; it does not replace the frozen analysis or report inputs.

- Recreated the ignored app `.venv` with CUDA PyTorch, the pinned `yolov8n_uav.pt` runtime, postprocessing packages, `pymavlink==2.4.50`, and PX4 `kconfiglib==13.7.0`. The reproducible setup now builds a venv without pip first, then uses its existing pip bootstrap path, so Ubuntu Python installations without `python3-venv` do not need a machine-level package install. The v1.2 runner, preflight, and PX4 launcher consistently select this app-owned interpreter.
- The no-flight probe is `preflight/short_capture_G0-P0-R01_a012`. It passed with left/right image counts 201/213, 50 Hz ground truth and no gaps at or above 40 ms. The observed camera position errors were below `3e-15` m and orientation errors were 0 degrees. The pause/step/resume probe observed 20/20/4 ms clock advances. Both saved 1280×720 RGB frames are non-uniform and visibly contain the target near image center.
- The probe-only vehicle fixture uses world ENU `[east,north,up] = [0,400,145.58809370648095]` m and holds the target static without modifying the source workbook or arming PX4. The recorded `vehicle_reference_link` Z is approximately 145.229 m because the x500_2m SDF defines that reference 0.35910692405 m below `base_link`; the spawn Z and published semantic reference are distinct contracts.
- Storage measurement is now available: 422,025.8 compressed bytes/s; 39 planned rows; 1,203.133 reference-science seconds; 22,425 seconds of declared entry/readiness/landing/finalization allowance; factor 1.25; projected captures 12,464,602,288 bytes; plus a 10,000,000,000-byte reserve. Free space after the probe was 630,133,886,976 bytes, so the storage gate passes.
- The environment report now has exactly one required failure: `isolated_yolo_python_and_device`. All Python imports, PX4, model hash, overlays, live capture, step control, and campaign storage checks pass. The CUDA build is installed (`torch 2.14.1+cu126`), but PyTorch reports zero CUDA devices; no campaign manifest slot was started.
- The archived report-v3 builder still cannot recreate its pinned historical workbook from this checkout: its precheck lists the v2 source workbook and five frozen-analysis inputs as missing. The historical v3 hash above does not identify a file present here. The fresh campaign route and its current execution blocker are recorded below.
- Verification after the recovery: 88 focused tests passed; Python compilation, both touched shell scripts, `git diff --check`, and `python3 tools/check_portable_paths.py` passed.
- Coordinate contract checked: workbook/reference path and Gazebo spawn use world ENU `[east,north,up]`; the ground-truth adapter publishes the model-defined `vehicle_reference_link` offset; camera checks preserve the declared optical/world rotations; PX4 local NED fields were not relabeled or rewritten.

## Fresh SWEEP-to-V3 implementation — 2026-10-21

- `xlsx_all` now performs the 39 workbook runs and, only after all 39 slots are source-QC-valid, runs the v1.2 analysis with exactly 100 Monte Carlo trials, verifies the analysis against the campaign source/checksum manifest, runs the v1.2 analysis audit, then creates a fresh report. `xlsx_resume` follows the same downstream chain when it completes an incomplete campaign. `xlsx_plan` remains plan-only; `report_v3` remains pinned to the archived analysis.
- The fresh exporter writes `recordings/stereo_v1_1/<output_id>/reports/stereo_v1_2_summary_report_ready_v3.xlsx` and a sibling `.manifest.json`. It builds the 20 V3 sheets and 22 dashboard charts plus the five-run P0/P2 A/B chart. `16_실행별V0` enforces the V3 100 G0 groups × 700 yaw/MC cases and includes the paired scope table; `17_추종P95출처` preserves the 14-column recorded/recomputed comparison. Its workbook and manifest include hashes for the analysis inputs, QC/alignment summaries, observation summaries, controller outputs, and GT/flight-state files used for post-freeze tracking. Excel metric and chart cells retain numeric types.
- `campaign_report_v3.py` requires a valid pre-V0 freeze and the V0 target geometry in world ENU `[east,north,up]` at northings `[600,550,500,450,400]`. Post-freeze tracking compares interpolated world-ENU GT with `reference_position_world_enu_m`; it does not mix PX4 local NED, body FRD or optical RDF fields. V0 planning remains N=600 m to N=400 m on a 20° glide.
- Targeted verification: the report, control, analysis orchestration, workbook campaign, analysis audit, and V0 reevaluation test set passes (108 passed; one legacy-workbook test deselected because its untracked `multi_flight_parameters_v1_1_numeric_v2.xlsx` input is absent). The report fixture confirms 20 sheets, 23 charts, the five-run chart range, 100 × 700 V0 aggregation, numeric XLSX cells, and GT/flight-state source hashes. Python compilation, `git diff --check`, and `python3 tools/check_portable_paths.py` pass.
- No fresh simulation or campaign XLSX was produced in this verification pass. The runtime preflight still reports `isolated_yolo_python_and_device` because the installed CUDA build sees zero devices; the flight action therefore stops before starting a manifest slot. A fresh V3 file will appear after a complete 39-run SWEEP succeeds on a CUDA-capable runtime.
- Coordinate contract checked: `[E,N,U]` world ENU targets and GT/reference values in `campaign_report_v3.py`; camera optical RDF, vehicle body FRD, and PX4 local NED remain separate; fixture and analysis-audit tests cover the freeze and target-frame gates.
