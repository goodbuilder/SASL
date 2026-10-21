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
