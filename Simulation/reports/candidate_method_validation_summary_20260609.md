# Candidate Method Validation Summary - 2026-06-09

This file records the tested StereoDistance correction candidates so the next
session can resume without re-running already rejected branches.

## Pause / Resume Snapshot

- Paused by user after offline aggregate and guard-sweep validation. Do not
  treat any later partial ROS2/Gazebo output as validation evidence.
- After resuming offline-only work, `bbox_anchor_cell_guard_cross_validation.py`
  was extended with a stricter `report_trial` holdout mode. This excludes both
  the held-out report and the same validation-trial number from Down-policy
  training.
- 2026-06-11 resume follow-up: the vertical-global `report_trial` Down policy
  was replayed across all 18 bbox-anchor reports and failed `15/90` held-out
  report/trial gates. A much stricter all-family micro Down guard passed
  deterministic `90/90` non-regression, but it only applied Down correction to
  `83/33570` samples and should be treated as near-no-op evidence, not a solved
  Down method.
- Completed comparisons in this file use the right-camera attitude-error
  scenario with roll/pitch/yaw increased by `0.1 deg` each.
- Deltas are `raw RMSE - corrected RMSE` in meters. Positive means improvement;
  `0.000` usually means protected raw/no-op behavior for that axis.
- No final method has been declared complete. No candidate has yet proven
  strict all-axis improvement on five new random overlap-safe validation
  trajectories.

| Resume rank | Candidate | Evidence kept | Right delta | Down delta | Forward delta | Position delta | Resume decision |
| ---: | --- | --- | ---: | ---: | ---: | ---: | --- |
| 1 | bbox-anchor axis guard, `rg=0.1`, `dg=0`, `fg=1`, `rmin=1100`, `fmin=500` | 18 reports / 90 validation trials; bootstrap all/trial/range-bin groups pass | min 0.000, mean 0.0418 | min 0.000, mean 0.000 | min 11.032, mean 40.541 | min 10.916, mean 40.465 | Best offline-stable non-no-op candidate. Resume here first if Right/Forward correction is the priority; Down is still raw/no-op. |
| 2 | bbox-anchor all-family `report_trial` micro Down guard, `dg=0.15`, `min_samples=20`, `min_train_down=0.02` | 18 reports / 90 stricter report+trial holdouts; bootstrap200 rows pass fraction `0.975`, trial-cluster pass fraction `1.000` | min 0.000, mean 0.0418 | min 0.000, mean 0.000174 | min 11.032, mean 40.541 | min 10.916, mean 40.465 | Deterministic all-family non-regression, but only `83/33570` samples apply Down correction. Treat as near-no-op guard sanity baseline, not a Down solution. |
| 3 | bbox-anchor vertical-global `report_trial` range-bin Down guard, `dg=0.25`, `min_samples=20`, `min_train_down=0.01` | 3 vertical-global reports / 15 stricter report+trial holdouts; bootstrap all/trial/range-bin groups pass; all-family replay fails `15/90` gates | min 0.000, mean 0.0355 | min 0.000, mean 0.0111 on vertical-only; all-family min `-0.0909` | min 11.939, mean 39.559 | min 11.812, mean 39.477 | Best narrow bbox-derived Down-improving branch, but all-family replay shows it does not generalize across bbox-anchor families. |
| 4 | bbox-anchor vertical-global LORO range-bin Down guard, `dg=0.2`, `min_samples=20`, `min_train_down=0.005` | 3 vertical-global reports / 15 leave-one-report-out trials; bootstrap all/trial/range-bin groups pass in the older report-only split | min 0.000, mean 0.0355 | min 0.000, mean 0.00898 | min 11.939, mean 39.559 | positive Position | Superseded by the stricter report+trial holdout candidate above. |
| 5 | bbox-anchor global LORO range/cell Down guard, `dg=0.2`, `min_samples=20`, `min_train_down=0.005` | 4 global reports / 20 leave-one-report-out trials; bootstrap all/trial/range-bin groups pass | min 0.000, mean 0.0348 | min 0.000, mean 0.00682 | min 12.059, mean 40.697 | positive Position | Second-best bbox-derived Down branch; 7/20 trials keep Down no-op. |
| 6 | bbox-anchor disparity-bin LORO cell Down guard, `dg=0.075`, `min_samples=400`, `min_train_down=0.005` | 3 disparity-bin reports / 15 leave-one-report-out trials; bootstrap all/trial/range-bin groups mostly pass | min 0.000, mean 0.0371 | min 0.000, mean 0.00561 | min 11.653, mean 37.958 | min 11.529, mean 37.880 | Earlier best bbox-derived Down branch; narrower than vertical-global and 7/15 trials still use Down no-op. |
| 7 | `range_only_forward_fusion` | 30 validation trials across completed range-only reports | min 0.000, mean 0.000, max 0.000 | min 0.000, mean 0.000, max 0.000 | min 1.538, mean 20.898, max 92.698 | min 1.524, mean 20.877, max 92.601 | Stable fallback baseline. Use as the minimum bar for new methods. |
| 8 | guarded ROI Down + range Forward + tiny correction guard `0.001 m` | 1 long ROI seed only; smoke C/D/E checked; bootstrap subgroup gate clear; evidence/coverage gate still fails | min 0.000, mean 0.000 | min 0.009, mean 0.015 | min 0.147, mean 3.151 | min 0.142, mean 3.146 | Best image-observation Down branch, but under-evidenced. Resume only with denser ROI capture and new multi-seed validation. |
| 9 | extrinsic global-bias axis fusion | 3 reports / 15 trials checked; axis guard and LORO range guard tried | raw/no-op | best safe Down is no-op; LORO mean only `+0.000258` | min `+11.704`, mean `+38.919` when Forward only | positive Forward | Useful as a Forward baseline, not a Down solution. |
| 10 | multi-source axis LORO fusion | 3 aligned bbox-anchor global + extrinsic global-bias reports / 15 trials checked | raw or bbox guard only | pass-all candidates choose Down raw/no-op | min `+11.942`, mean `+39.700` for best range-bin Forward selector | positive Forward | Does not solve Down; useful only as another Forward selector baseline. |
| 11 | tracking/Kalman filter | 6 existing tracking reports plus 1280-config offline sweep over 3 reports / 15 trials | regressions remain | regressions remain | mixed | mixed | Rejected as a correction source for now; keep only as temporal smoothing after a spatial method is validated. |
| 12 | bbox non-center / feature residual / known-size / KNN candidates | Completed or ad-hoc checks summarized below | includes negative lateral or Forward deltas | includes regressions | mixed | mixed | Rejected for now. Do not re-run unless the observation model or acceptance criterion changes. |

Primary artifacts to open first on resume:

- `reports/bbox_anchor_axis_guard_fixed_rg0p1_down0_all_anchor_reports`
- `reports/bbox_anchor_axis_guard_fixed_rg0p1_down0_all_anchor_reports_bootstrap/bootstrap_summary.csv`
- `reports/bbox_anchor_cell_guard_loro_down_sweep_vertical_global_report_trial_holdout_dg0p25_min0p01`
- `reports/bbox_anchor_cell_guard_loro_down_sweep_vertical_global_report_trial_holdout_dg0p25_min0p01_bootstrap/bootstrap_summary.csv`
- `reports/bbox_anchor_cell_guard_report_trial_all_anchor_dg0p25_min0p01_20260611`
- `reports/bbox_anchor_cell_guard_report_trial_all_anchor_micro_down_guard_20260611`
- `reports/bbox_anchor_cell_guard_report_trial_all_anchor_micro_down_guard_20260611_bootstrap200/bootstrap_summary.csv`
- `reports/bbox_anchor_cell_guard_loro_down_sweep_all_anchor_reports`
- `reports/bbox_anchor_cell_guard_loro_down_sweep_vertical_global_reports`
- `reports/bbox_anchor_cell_guard_loro_down_sweep_vertical_global_reports_bootstrap/bootstrap_summary.csv`
- `reports/bbox_anchor_cell_guard_loro_down_sweep_global_reports`
- `reports/bbox_anchor_cell_guard_loro_down_sweep_global_reports_bootstrap/bootstrap_summary.csv`
- `reports/bbox_anchor_cell_guard_loro_down_sweep_disparity_reports`
- `reports/bbox_anchor_cell_guard_loro_down_sweep_disparity_reports_bootstrap/bootstrap_summary.csv`
- `reports/validation_gate_global_aggregate_20260609`
- `reports/roi_guarded_down_range_forward_fusion_tiny0p001_seed20260634`
- `reports/roi_guarded_down_range_forward_fusion_tiny0p001_stability_gate_seed20260634`

## Validation Context

- Scenario: 15 m UAM/drone target, right camera roll/pitch/yaw attitude error
  increased by 0.1 deg each.
- Primary acceptance target used in this session: each validation trajectory
  should reduce or at least not regress RMSE on all RDF axes compared with raw
  estimated output. Scalar range error was treated as a separate metric.
- Important interpretation: reported RDF deltas below are `raw RMSE -
  corrected RMSE`, so positive is improvement and negative is regression.
- Profile and validation trajectories were separated where a full run completed.
- Current stable baseline is conservative: it improves Forward/Position
  through range-only correction and intentionally leaves Right/Down unchanged.
- Current strongest offline-stable non-no-op candidate is bbox-anchor axis
  guard across all existing bbox-anchor families: Right is corrected with gain
  `0.1` only beyond `1100 m`, Forward is corrected beyond `500 m`, and Down
  remains raw/no-op. It passes 18 existing reports / 90 validation trials plus
  all populated bootstrap trial/range subgroups. It is still not a complete
  all-axis improvement method because Down is intentionally no-op.
- Later leave-one-report-out bbox-anchor family sweeps found limited but
  repeatable Down improvement inside several profile families. A stricter
  report+trial holdout check then replaced the earlier vertical-global pick
  with a more conservative policy: `feature=range_bin`, `dg=0.25`,
  `min_samples=20`, `min_train_down=0.01`. It passes 15/15 held-out
  report/trial pairs and all populated bootstrap trial/range-bin subgroups.
  Range bins 0 and 1 remain Down no-op; bins 2/3/4 improve. It is still not
  final because it has not been proven on five new random trajectories.
- Replaying that vertical-global policy against all 18 existing bbox-anchor
  report families failed `15/90` stricter report+trial holdouts, with worst
  Down delta `-0.090940841 m`. A stricter all-family micro guard passed
  deterministic `90/90`, but applied Down to only `83/33570` samples and had
  mean Down delta `+0.000174318 m`, so it is too close to no-op to promote as a
  Down solution.
- Current best Down-improving investigation candidate remains the guarded ROI
  `top_center` Down + range Forward + tiny correction guard branch, but it has
  only one long ROI report and is not stable enough to declare complete.

## Current Recommendation

Continue from `range_only_forward_fusion` as the baseline to beat.

It is not a complete all-axis improvement method because Right and Down are
protected as no-op. However, across the completed validation sets it showed
stable non-regression:

| Method | Completed validation trials | Right delta | Down delta | Forward delta | Position delta | Verdict |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| `range_only_forward_fusion` | 30 | min 0.000, mean 0.000, max 0.000 | min 0.000, mean 0.000, max 0.000 | min 1.538, mean 20.898, max 92.698 | min 1.524, mean 20.877, max 92.601 | Best stable baseline so far |

Evidence report directories:

- `reports/range_only_reprojection_forward_right_att_0p1_seed20260625`
- `reports/range_only_reprojection_forward_right_att_0p1_seed20260626`
- `reports/range_only_reprojection_forward_right_att_0p1_seed20260626_ultralong`
- `reports/range_only_reprojection_forward_fullscreen_right_att_0p1_seed20260625`
- `reports/range_only_reprojection_forward_screen_coverage_right_att_0p1_seed20260624`
- `reports/range_only_reprojection_forward_right_att_0p1_seed20260623`

Current non-no-op branch to resume:

| Method | Evidence state | Right delta | Down delta | Forward delta | Position delta | Verdict |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| bbox-anchor axis guard | 18 reports / 90 trials across all bbox-anchor families; bootstrap all/trial/range-bin groups pass | min 0.000, mean 0.0418 | min 0.000, mean 0.000 | min 11.032, mean 40.541 | min 10.916, mean 40.465 | Best offline-stable non-no-op candidate; Down is no-op |
| bbox-anchor all-family `report_trial` micro Down guard | 18 reports / 90 stricter holdouts; bootstrap200 rows pass fraction `0.975`, trial-cluster pass fraction `1.000` | min 0.000, mean 0.0418 | min 0.000, mean 0.000174 | min 11.032, mean 40.541 | min 10.916, mean 40.465 | Deterministic pass-all but effectively near-no-op for Down; only `83/33570` samples apply Down |
| bbox-anchor vertical-global `report_trial` range-bin Down guard | 3 reports / 15 stricter holdouts; bootstrap all/trial/range-bin groups pass | min 0.000, mean 0.0355 | min 0.000, mean 0.0111 | min 11.939, mean 39.559 | min 11.812, mean 39.477 | Best bbox-derived Down-improving branch; narrow evidence, not final |
| bbox-anchor vertical-global LORO range-bin Down guard | 3 reports / 15 leave-one-report-out trials; older report-only split | min 0.000, mean 0.0355 | min 0.000, mean 0.00898 | min 11.939, mean 39.559 | positive | Superseded by stricter report+trial holdout |
| bbox-anchor global LORO range/cell Down guard | 4 reports / 20 leave-one-report-out trials; bootstrap all/trial/range-bin groups pass | min 0.000, mean 0.0348 | min 0.000, mean 0.00682 | min 12.059, mean 40.697 | positive | Second-best bbox-derived Down branch; 7/20 trials Down no-op |
| bbox-anchor disparity-bin LORO cell Down guard | 3 disparity-bin reports / 15 leave-one-report-out trials; bootstrap all/trial/range-bin groups checked | min 0.000, mean 0.0371 | min 0.000, mean 0.00561 | min 11.653, mean 37.958 | min 11.529, mean 37.880 | Earlier bbox-derived Down branch; narrow evidence |
| guarded ROI Down + range Forward | 1 long seed, bootstrap subgroup gate clear, stability gate fails only on evidence/coverage sufficiency | min 0.000, mean 0.000 | min 0.009, mean 0.015 | min 0.147, mean 3.151 | min 0.142, mean 3.146 | Previous best; superseded by tiny guard for strict-gate hygiene |
| guarded ROI Down + range Forward + tiny correction guard `0.001 m` | 1 long seed, smoke C/D/E checked, bootstrap subgroup gate clear, stability gate fails only on evidence/coverage sufficiency | min 0.000, mean 0.000 | min 0.009, mean 0.015 | min 0.147, mean 3.151 | min 0.142, mean 3.146 | Best next multi-seed candidate; not final |

## Rejected Or Unstable Candidates

### Tracking / Kalman Filter

Tracking was checked because temporal filtering may still be useful after a
spatial correction method is selected. As a standalone correction source, it is
not stable enough under the current acceptance gate.

Existing report summary:

- Reports:
  - `reports/tracking_anchor_global_right_att_0p1_seed20260625`
  - `reports/tracking_anchor_global_right_att_0p1_seed20260626`
  - `reports/tracking_anchor_global_right_att_0p1_seed20260626_ultralong`
  - the corresponding `_fast_open` variants.
- Default tracking over anchor input failed all normal/ultralong reports:
  - seed `20260625`: `0/5` trials pass; min RDF
    `Right=-6.710`, `Down=-2.634`, `Forward=-199.621`.
  - seed `20260626`: `0/5` trials pass; min RDF
    `Right=-5.129`, `Down=-3.946`, `Forward=-57.832`.
  - ultralong seed `20260626`: `0/5` trials pass; min RDF
    `Right=-4.388`, `Down=-6.954`, `Forward=-49.483`.
- `_fast_open` reduced the damage but still failed:
  - seed `20260625_fast_open`: `2/5` trials pass; min RDF
    `Right=-0.012`, `Down=-0.105`, `Forward=+25.418`.
  - seed `20260626_fast_open`: `1/5` trials pass; min RDF
    `Right=-0.418`, `Down=-0.092`, `Forward=+10.866`.
  - ultralong `_fast_open`: `2/5` trials pass; min RDF
    `Right=-0.404`, `Down=-0.057`, `Forward=+10.814`.

Additional offline parameter sweep:

- Scope: `1280` Kalman configurations over the three
  `bbox_anchor_global_right_att_0p1` reports, `15` validation trials total.
- Sources tested: `raw` and `anchor`.
- Varied:
  `process_accel_std_mps2`, `initial_position_std_m`,
  `max_normalized_innovation`, and min lateral/forward measurement std.
- Result: no pass-all candidate.
- Best-looking family still regressed at least one axis:
  - source `anchor`, process accel `256`, max innovation open, min std
    `(0.05 m lateral, 0.5 m forward)`.
  - min RDF:
    `Right=-0.416857`, `Down=-0.007050`, `Forward=+11.958293`.
  - mean RDF:
    `Right=+0.555575`, `Down=+0.039743`, `Forward=+40.270828`.

Conclusion: use Kalman/tracking only after a spatial correction has already
passed RDF gates. It should not be the main correction method or the source of
axis RMSE improvement claims.

### Extrinsic Refinement Baselines

Extrinsic refinement was checked because the real problem is unknown right
camera extrinsic error. The physical direction is valid, but the current
single-trial fit/global-bias implementations are not stable enough as final
methods.

Existing global-bias report summary:

- `extrinsic_refinement_right_att_0p1_seed20260625_global_bias`:
  `4/5` trials pass; min RDF
  `Right=-0.032`, `Down=+0.024`, `Forward=+27.312`.
- `extrinsic_refinement_right_att_0p1_seed20260626_global_bias`:
  `4/5` trials pass; min RDF
  `Right=-0.418`, `Down=+0.029`, `Forward=+11.704`.
- `extrinsic_refinement_right_att_0p1_seed20260626_ultralong_global_bias`:
  `3/5` trials pass; min RDF
  `Right=-0.380`, `Down=-0.020`, `Forward=+12.265`.

Axis-wise follow-up:

- Right was forced to raw/no-op while Down/Forward used the extrinsic
  corrected residual with gain/range guards.
- No candidate with positive Down minimum was found.
- Best pass-all axis-fusion candidate is effectively Down no-op:
  `Right=raw`, `Down=raw`, `Forward=global-bias corrected`.
  It has min/mean Forward improvement `+11.703761 / +38.918708 m` across the
  3 reports / 15 trials.
- A leave-one-report-out raw-range Down guard was also checked. The best safe
  result applied Down only to the ultralong holdout and had mean Down
  improvement `+0.000258 m`, which is too close to no-op to count as a useful
  Down method.

Rotation-only and true-rotation baselines:

- Rotation-only failed because Right and/or Down regressed despite strong
  Forward improvements.
- True-rotation also failed validation gates. This reinforces that the YOLO
  bbox center is not a stable physical correspondence point even when the
  attitude error model is known.

Conclusion: keep extrinsic/global-bias methods as baselines and for future
calibration work, but do not promote the current implementation as the main
runtime correction.

### Multi-Source Axis LORO Fusion

Because bbox-anchor and extrinsic global-bias reports share the same
validation trajectories, an additional row-aligned source-selection check was
run without launching new simulation.

Inputs:

- `bbox_anchor_global_right_att_0p1_seed20260625` +
  `extrinsic_refinement_right_att_0p1_seed20260625_global_bias`
- `bbox_anchor_global_right_att_0p1_seed20260626` +
  `extrinsic_refinement_right_att_0p1_seed20260626_global_bias`
- `bbox_anchor_global_right_att_0p1_seed20260626_ultralong` +
  `extrinsic_refinement_right_att_0p1_seed20260626_ultralong_global_bias`

The row counts and timestamps align across paired reports, so the following
sources were compared per axis:

- raw
- bbox anchor
- center-rot triangulation
- extrinsic global-bias corrected

LORO policies tested:

- Hold out one report, choose per-axis source from the other two reports.
- Feature keys: global `all` and raw-range `range_bin`.
- Source selection threshold: require training RMSE improvement over raw.
- Right modes:
  - learned source selection, which failed because Right overfit and regressed
    on the holdout.
  - raw/no-op or the conservative bbox Right guard.

Results:

- If Right is also learned, the best-looking policies still fail with Right
  regression as low as `-0.441937 m`.
- If Right is protected as raw/no-op or conservative bbox guard, all pass-all
  policies choose Down raw/no-op.
- Best pass-all range-bin Forward selector:
  - Right: raw/no-op
  - Down: raw/no-op
  - Forward: raw for range bin `0`, center-rot for range bin `1`, bbox anchor
    for range bins `2-4`.
  - min/mean Forward improvement: `+11.942224 / +39.699596 m`.
  - min/mean Right improvement: `0 / 0 m`.
  - min/mean Down improvement: `0 / 0 m`.
- Best pass-all global Forward selector:
  - Forward source switched between center-rot and bbox anchor by held-out
    report.
  - min/mean Forward improvement: `+12.023983 / +39.511270 m`.
  - Down remained raw/no-op.

Conclusion: source selection among existing bbox/extrinsic/center-rot outputs
does not reveal a stable Down correction. The result reinforces that the next
meaningful work needs either more disparity-bin evidence for the limited bbox
Down branch or denser ROI/image observations.

### Bbox Non-Center Geometry Candidates

Source summary:
`reports/bbox_non_center_axis_gain_sweep_multi_0p1/global_candidate_aggregate_summary.csv`

- Tested 152 candidate/method groups, 30 validation rows per group.
- Strict all-report improvement: 0 groups.
- All-report non-regression: 4 groups, but all were actual no-op gains
  `0/0/0`, so they do not provide a useful correction.

Representative result:

| Candidate | Method | Min Right | Min Down | Min Forward | Mean Forward | Verdict |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `learned_origin_anchor` | `triangulated` | -0.417 | -0.433 | 12.024 | 43.877 | Forward improves, lateral axes regress |
| `inner_edges_top` | `triangulated_axis_gain` | 0.000 | 0.000 | 0.000 | 0.000 | Passes only because gain is no-op |
| `left_center_to_right_edge_top` | `triangulated_axis_gain` | 0.000 | 0.000 | 0.000 | 0.000 | Passes only because gain is no-op |

Conclusion: bbox geometry-only non-center anchors are not reliable enough for
RDF all-axis correction under 0.1 deg right-camera attitude error.

### Bbox Feature Residual Ridge Profiles

Source summary:
`reports/bbox_feature_residual_longprofile_seed20260625/candidate_aggregate_summary.csv`

- Tested 28 candidates.
- All-trial non-regression: 0.
- All-trial strict improvement: 0.

Representative result:

| Candidate | Lambda | Min Right | Min Down | Min Forward | Mean Position | Verdict |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| `size_shape_compact` | 0.001 | -7.540 | -2.074 | 8.876 | 38.534 | Forward improves, lateral axes regress |
| `size_shape_raw_rdf` | 1000 | -6.677 | -0.861 | -78.665 | not selected | Unstable, includes Forward regression |

Conclusion: feature residual regression overfits or generalizes poorly in this
setup. Do not use as the next main path without a stronger observation model.

### Known-Size Range From Bbox Width/Height/Area

Width-based completed evidence:
`reports/bbox_non_center_size_range_longprofile_seed20260625/validation_gate_summary.csv`

Width known-size result by validation trial:

| Trial | Right delta | Down delta | Forward delta | Verdict |
| ---: | ---: | ---: | ---: | --- |
| 2 | 0.445 | 0.290 | 36.568 | Pass |
| 3 | 0.276 | -0.096 | -4.448 | Fail |
| 4 | -3.486 | -0.476 | -331.094 | Fail |
| 5 | -2.587 | -0.010 | -184.085 | Fail |
| 6 | -0.083 | -0.073 | -35.480 | Fail |

Quick height/area checks were also tried but were not persisted as full report
sets:

| Variant | Blend | Min Right | Min Down | Min Forward | Verdict |
| --- | ---: | ---: | ---: | ---: | --- |
| bbox height range | 0.05 | -0.134 | 0.001 | 1.921 | Small Right regression |
| bbox height range | 0.10 | -0.270 | 0.001 | 1.918 | Right regression |
| sqrt bbox area range | 0.05 | -0.135 | 0.001 | -0.776 | Right and Forward regression |

Conclusion: the YOLO bbox does not consistently represent the physical 15 m
target extent. Bbox-size-derived metric range is not stable enough as the main
correction signal.

### KNN Median Residual

Quick ad-hoc checks were run against size/shape/disparity residual features.
They were not persisted as full report sets.

Best-looking examples still failed on small lateral regressions:

| K | Blend | Min Right | Min Down | Min Forward | Mean Forward | Verdict |
| ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 11 | 0.05 | -0.016 | 0.002 | 1.049 | 2.742 | Small Right regression |
| 161 | 0.05 | -0.003 | -0.015 | 1.278 | not selected | Small Right/Down regression |

Conclusion: even conservative local residual lookup did not meet RDF all-axis
stability. Avoid spending more time here unless the validation criterion is
changed or stronger image observations are added.

## Bbox Anchor Axis-Guard Candidate

This branch was added after re-aggregating all existing
`validation_gate_summary.csv` files. It is important because it found a
stronger non-ROI candidate than pure range-only fusion without requiring new
ROS/Gazebo runs.

Global validation-gate aggregate:

- Script: `scripts/validation_gate_aggregate.py`
- Output: `reports/validation_gate_global_aggregate_20260609`
- Inputs loaded: `72` validation summaries.
- Trial rows aggregated: `459`.
- Method rows aggregated: `62`.
- With `--min-reports 2`, only range-only families passed all RDF trials
  without any lateral correction.
- Strict non-noop all-axis candidates with positive min Right, Down, and
  Forward improvements across at least 2 reports: `0`.

All-family bbox-anchor axis guard:

- Script: `scripts/bbox_anchor_axis_guard_sweep.py`
- Strongest robust output:
  `reports/bbox_anchor_axis_guard_fixed_rg0p1_down0_all_anchor_reports`
- Bootstrap output:
  `reports/bbox_anchor_axis_guard_fixed_rg0p1_down0_all_anchor_reports_bootstrap/bootstrap_summary.csv`
- Input reports: all 18 existing `reports/bbox_anchor_*right_att_0p1_seed*`
  validation reports, including disparity-bin, global, fullscreen,
  screen-coverage, screen-grid, vertical-global, and ultralong variants.
- Policy:
  - Right: bbox-anchor correction with gain `0.1`, but raw/no-op below raw
    Forward `1100 m`.
  - Down: raw/no-op, gain `0.0`.
  - Forward: bbox-anchor correction with gain `1.0`, but raw/no-op below raw
    Forward `500 m`.
  - Guard inputs are runtime-available: raw Forward/range only.
- Candidate summary:
  - reports: `18`; validation trials: `90`.
  - pass_all_nonregression: yes.
  - min RDF improvements: Right `0.000000000`, Down `0.000000000`,
    Forward `11.032455590`.
  - mean improvements: Right `0.041759338`, Down `0.000000000`,
    Forward `40.540801288`, Position `40.464589990`.
  - guarded rows: Right range guard `27378`, Forward range guard `5932`.
- Group summary:
  - all populated report, trial, and range-bin groups pass RDF
    non-regression.
  - range bin `0` is intentionally full no-op: Right `0`, Down `0`,
    Forward `0`.
  - weakest trial-level Right improvement: trial `5`, `+0.003209966 m`.
  - weakest trial-level Forward improvement: trial `5`, `+19.872045113 m`.
- Bootstrap:
  - overall rows and trial-cluster pass fractions: `1.000`.
  - all populated range-bin and trial bootstrap pass fractions: `1.000`.
  - all RDF 2.5th percentile subgroup improvements are non-negative.

Rejected bbox-axis variants:

- `rg=1`, `dg=1`, `fg=1`, `rmin=1100`, `dmin=800`, `fmin=500` passed the two
  normal reports, but trial 6 bootstrap pass fraction was only `0.902` because
  Down p025 was `-0.002262709`.
- `rg=1`, `dg=0`, `fg=1`, `rmin=1100`, `fmin=500` passed the two normal
  reports, but failed the ultralong report with Right min `-0.031113283`.
- Increasing conservatism to `rg=0.5`, `dg=0`, `fg=1`, `rmin=1100`,
  `fmin=500` removed the ultralong regression while retaining meaningful Right
  and strong Forward improvements.
- Applying `rg=0.5`, `dg=0`, `fg=1`, `rmin=1100`, `fmin=500` across all
  bbox-anchor families failed one report/trial:
  `bbox_anchor_global_fullscreen_right_att_0p1_seed20260625`, trial `5`,
  Right `-0.002268596`.
- Reducing Right gain to `0.1` removed that all-family regression while keeping
  Right p025 positive in the full bootstrap.

Current interpretation:

- This is the best offline-stable non-no-op candidate currently available.
- It is stronger than the ROI branch on evidence count and bootstrap
  robustness.
- It is weaker than the desired final behavior because Down remains no-op and
  the reports are existing validation reports, not the requested five new
  random overlap-safe validation trajectories.

### Bbox Anchor Cell/Range Down Guard LORO

After the all-family axis guard still selected Down gain `0`, an additional
runtime-safe leave-one-report-out check was added for Down:

- Script: `scripts/bbox_anchor_cell_guard_cross_validation.py`
- Contract:
  - Right remains the conservative axis guard: gain `0.1`, raw/no-op below raw
    Forward `1100 m`.
  - Forward remains the conservative axis guard: gain `1.0`, raw/no-op below
    raw Forward `500 m`.
  - Down is enabled only for runtime-known `anchor_cell`, `range_bin`, or
    `anchor_cell + range_bin` cells that improve on the training reports.
  - Each report is held out while the Down policy is selected from the other
    reports, so held-out rows are not used to choose enabled Down cells.

First, a wider global Down gain/range sweep over all 18 bbox-anchor reports was
re-run:

- Output: `reports/bbox_anchor_axis_guard_down_sweep_all_anchor_reports`
- Result: every positive global Down gain regressed at least one validation
  trial. The best pass-all candidate remained `dg=0`.
- Best positive Down-gain example:
  `dg=0.005`, `dmin=1400`, `83/90` trials passing, minimum Down delta
  `-0.000957993 m`, mean Down delta `+0.000011123 m`.

Then the LORO cell guard was tested across all 18 reports:

- Output:
  `reports/bbox_anchor_cell_guard_loro_down_sweep_all_anchor_reports`
- Bootstrap:
  `reports/bbox_anchor_cell_guard_loro_down_sweep_all_anchor_reports_bootstrap/bootstrap_summary.csv`
- Best candidate:
  `feature=anchor_cell`, `dg=0.01`, `min_samples=400`,
  `min_train_down=0.001`.
- Result:
  - reports/trials: `18 / 90`
  - pass_all_nonregression: yes
  - Down-applied rows: `1314`; Down cell-guard no-op rows: `32256`
  - min/mean Down delta: `0.000000000 / 0.000111954 m`
  - min/mean Right delta: `0.000000000 / 0.041759338 m`
  - min/mean Forward delta: `11.032455590 / 40.540801288 m`
  - all-row bootstrap Down p025: `+0.000081958 m`
  - trial-cluster bootstrap Down p025: `+0.000032440 m`
  - trial 6 row-bootstrap pass fraction: `0.993`
- Interpretation:
  - This is safe but almost no-op. It mostly applies in disparity-bin report
    holdouts because different anchor-profile families use incompatible
    `anchor_cell` namespaces.
  - It is not a general all-family Down solution.

Finally, the LORO cell guard was tested only within the disparity-bin profile
family:

- Output:
  `reports/bbox_anchor_cell_guard_loro_down_sweep_disparity_reports`
- Bootstrap:
  `reports/bbox_anchor_cell_guard_loro_down_sweep_disparity_reports_bootstrap/bootstrap_summary.csv`
- Best candidate:
  `feature=anchor_cell`, `dg=0.075`, `min_samples=400`,
  `min_train_down=0.005`.
- Result:
  - reports/trials: `3 / 15`
  - pass_all_nonregression: yes
  - Down-applied rows: `1438`; Down cell-guard no-op rows: `4420`
  - min/mean Down delta: `0.000000000 / 0.005611788 m`
  - min/mean Right delta: `0.000000000 / 0.037088238 m`
  - min/mean Forward delta: `11.652904306 / 37.957681003 m`
  - min/mean Position delta: `11.529034983 / 37.880076309 m`
  - all-row bootstrap Down p025: `+0.004048377 m`
  - trial-cluster bootstrap Down p025: `+0.002303003 m`
  - range-bin Down p025 values:
    - range bin `0`: `+0.002511263 m`
    - range bin `2`: `+0.005641366 m`
    - range bin `3`: `+0.018246858 m`
    - range bin `4`: `+0.008220170 m`
  - trial 6 row-bootstrap pass fraction: `0.984`; Down negative fraction
    `0.016`, although Down p025 remained `+0.000041037 m`.
- Interpretation:
  - This is the best bbox-derived Down-improving branch so far.
  - It still does not meet the desired final standard because only 3
    disparity-bin reports exist, 7 of 15 validation trials apply no Down
    correction, and the result has not been verified on five new
    overlap-safe random trajectories.
  - If continuing this branch, generate new disparity-bin profile/validation
    reports first and re-run this LORO gate. Do not generalize the learned
    cells across other anchor-profile families.

The same family-local LORO sweep was then run over other bbox-anchor profile
families. This found stronger Down evidence than the disparity-bin branch:

| Family | Output | Reports / trials | Best policy | Down applied rows | Zero-Down trials | Min / mean Down delta | Min / mean Forward delta | Verdict |
| --- | --- | ---: | --- | ---: | ---: | ---: | ---: | --- |
| vertical-global | `reports/bbox_anchor_cell_guard_loro_down_sweep_vertical_global_reports` | 3 / 15 | `feature=range_bin`, `dg=0.2`, `min_samples=20`, `min_train_down=0.005` | 2519 | 2 | `0.000000 / 0.008979 m` | `11.938956 / 39.558758 m` | Best bbox-derived Down branch so far |
| global | `reports/bbox_anchor_cell_guard_loro_down_sweep_global_reports` | 4 / 20 | `feature=anchor_cell_range_bin`, `dg=0.2`, `min_samples=20`, `min_train_down=0.005` | 2519 | 7 | `0.000000 / 0.006824 m` | `12.058995 / 40.696798 m` | Second-best; more reports but more Down no-op trials |
| screen5x3 | `reports/bbox_anchor_cell_guard_loro_down_sweep_screen5x3_reports` | 3 / 15 | `feature=anchor_cell`, `dg=0.1`, `min_samples=20`, `min_train_down=0.005` | 1449 | 4 | `0.000000 / 0.002403 m` | `11.032456 / 38.912031 m` | Weaker but nonzero |
| screen3x3 | `reports/bbox_anchor_cell_guard_loro_down_sweep_screen3x3_reports` | 3 / 15 | `feature=range_bin`, `dg=0.05`, `min_samples=400`, `min_train_down=0.005` | 183 | 12 | `0.000000 / 0.000065 m` | `11.595059 / 39.307409 m` | Too close to no-op |

Vertical-global bootstrap:

- Output:
  `reports/bbox_anchor_cell_guard_loro_down_sweep_vertical_global_reports_bootstrap/bootstrap_summary.csv`
- all rows: pass fraction `1.000`, Down p025 `+0.007517806 m`,
  Right p025 `+0.015475855 m`, Forward p025 `+33.529839178 m`.
- trial-cluster resampling: pass fraction `1.000`, Down p025
  `+0.002635419 m`, Right p025 `+0.003290379 m`, Forward p025
  `+25.973918368 m`.
- Trial row-bootstrap Down p025 values:
  - trial 2: `+0.019431892 m`
  - trial 3: `+0.004503170 m`
  - trial 4: `+0.009245690 m`
  - trial 5: `+0.001314844 m`
  - trial 6: `+0.001687143 m`
- Range-bin Down p025 values:
  - bins 0 and 1 are intentional no-op (`0`).
  - bin 2: `+0.019256271 m`
  - bin 3: `+0.019898793 m`
  - bin 4: `+0.015404623 m`

Stricter vertical-global report+trial holdout:

- Tooling change:
  `scripts/bbox_anchor_cell_guard_cross_validation.py` now supports
  `--holdout-mode report_trial`. For each held-out report/trial pair, Down
  policy training excludes both rows from the held-out report and rows from the
  same validation-trial number in the other reports.
- First fixed check:
  `reports/bbox_anchor_cell_guard_loro_down_sweep_vertical_global_report_trial_holdout`
  used the older `dg=0.2`, `min_train_down=0.005` policy. It passed all 15
  report/trial holdouts, but `range_bin=1` had a small Down regression
  (`-0.000078984 m`) and bootstrap pass fraction `0.241`. Do not use this as
  the final vertical-global candidate.
- Threshold sweep:
  `reports/bbox_anchor_cell_guard_loro_down_sweep_vertical_global_report_trial_holdout_threshold_sweep`
  found `dg=0.25`, `min_train_down=0.005` with higher mean Down improvement,
  but `range_bin=1` still had a small bootstrap negative fraction (`0.005`).
  It is better than `dg=0.2` but less conservative than the final fixed pick.
- Current fixed report+trial candidate:
  `reports/bbox_anchor_cell_guard_loro_down_sweep_vertical_global_report_trial_holdout_dg0p25_min0p01`
  with `feature=range_bin`, `dg=0.25`, `min_samples=20`,
  `min_train_down=0.01`.
  - 3 reports / 15 held-out report+trial pairs pass.
  - Trial-summary deltas: Right min/mean `0.000000 / 0.035546 m`,
    Down min/mean `0.000000 / 0.011139 m`, Forward min/mean
    `11.938956 / 39.558758 m`, Position min/mean
    `11.811546 / 39.476514 m`.
  - Sample-weighted group deltas: Right `+0.017152 m`, Down
    `+0.010208 m`, Forward `+34.483498 m`, Position `+34.411635 m`.
  - Range bins 0 and 1 are Down no-op. Range-bin Down improvements are:
    bin 2 `+0.026543 m`, bin 3 `+0.027809 m`, bin 4 `+0.023372 m`.
- Bootstrap:
  `reports/bbox_anchor_cell_guard_loro_down_sweep_vertical_global_report_trial_holdout_dg0p25_min0p01_bootstrap/bootstrap_summary.csv`
  - all rows: pass fraction `1.000`, Down p025 `+0.009328771 m`,
    Right p025 `+0.015475855 m`, Forward p025 `+33.529839178 m`.
  - trial-cluster resampling: pass fraction `1.000`, Down p025
    `+0.003238894 m`, Right p025 `+0.003290379 m`, Forward p025
    `+25.973918368 m`.
  - Trial row-bootstrap Down p025 values:
    trial 2 `+0.024144459 m`, trial 3 `+0.005571230 m`, trial 4
    `+0.011512840 m`, trial 5 `+0.001620313 m`, trial 6
    `+0.002046489 m`.
  - Range-bin row-bootstrap Down p025 values:
    bins 0/1 `0.000000000 m`, bin 2 `+0.024003175 m`, bin 3
    `+0.024626480 m`, bin 4 `+0.018752871 m`.

All-family report+trial replay on 2026-06-11:

- Focused vertical-global policy replay:
  `reports/bbox_anchor_cell_guard_report_trial_all_anchor_dg0p25_min0p01_20260611`
  ran the current vertical-global fixed policy against all 18 existing
  bbox-anchor reports with `--holdout-mode report_trial`.
  - Result: `75/90` held-out report/trial pairs pass; `15/90` fail.
  - Trial-summary deltas: Right min/mean `0.000000 / 0.041759 m`,
    Down min/mean `-0.090941 / 0.009136 m`, Forward min/mean
    `11.032456 / 40.540801 m`.
  - Failing Down regressions are concentrated in cross-family replay, not in
    the vertical-global-only evidence. Worst failures include
    `bbox_anchor_global_right_att_0p1_seed20260623` trial 2
    (`Down=-0.090940841 m`) and trial 4 (`Down=-0.063692938 m`), plus
    `bbox_anchor_global_screen_coverage_right_att_0p1_seed20260624` trials
    2/3/5/6.
  - Interpretation: the vertical-global Down policy is family-local. It should
    not be claimed as an all-family bbox-anchor correction.
- All-family micro Down guard:
  `reports/bbox_anchor_cell_guard_report_trial_all_anchor_micro_down_guard_20260611`
  used `feature=range_bin`, `dg=0.15`, `min_samples=20`,
  `min_train_down=0.02`.
  - Result: deterministic `90/90` held-out report/trial pairs pass.
  - Trial-summary deltas: Right min/mean `0.000000 / 0.041759 m`,
    Down min/mean `0.000000 / 0.000174 m`, Forward min/mean
    `11.032456 / 40.540801 m`.
  - Down correction applied to only `83/33570` samples, all in range bin 3.
    Only `bbox_anchor_global_right_att_0p1_seed20260623` had nonzero
    report-level Down improvement (`+0.002803846 m`); every other report was
    Down no-op.
  - Bootstrap200:
    `reports/bbox_anchor_cell_guard_report_trial_all_anchor_micro_down_guard_20260611_bootstrap200/bootstrap_summary.csv`.
    All-row pass fraction was `0.975`, trial-cluster pass fraction was
    `1.000`, and all-row Down p025 was `+0.000001863 m`. Trial 6 row bootstrap
    had Down p025 `-0.000124118 m` and negative fraction `0.330`.
  - Interpretation: this is useful as an all-family guard sanity check, but it
    is too close to no-op to count as a meaningful Down solution.
- A broader brute-force all-family range-bin sweep was started, but stopped
  because the current implementation is too slow for wide grids. A fast
  read-only parameter scan over the same rows found no pass-all candidate with
  strictly positive minimum Down improvement and nonzero Down application. The
  only pass-all nonzero Down policies had `min_down=0`.

Global-family bootstrap:

- Output:
  `reports/bbox_anchor_cell_guard_loro_down_sweep_global_reports_bootstrap/bootstrap_summary.csv`
- all rows: pass fraction `1.000`, Down p025 `+0.006680221 m`.
- trial-cluster resampling: pass fraction `1.000`, Down p025
  `+0.002292087 m`.
- All trial and populated range-bin row-bootstrap Down p025 values were
  non-negative.

Current interpretation after family-local sweeps:

- Vertical-global remains the strongest bbox-derived Down candidate, but the
  preferred setting is now the stricter report+trial holdout fixed policy:
  `feature=range_bin`, `dg=0.25`, `min_samples=20`,
  `min_train_down=0.01`.
- It is still not a complete success because only 3 reports exist and the
  result has not been repeated on five new random overlap-safe trajectories.
- The result changes the next bbox branch to validate: if new simulation is
  authorized, generate at least five new vertical-global profile/validation
  reports under the same `0.1 deg` attitude-error scenario and re-run the LORO
  gate before considering runtime integration.

## ROI / Silhouette Image Observation Path

This path was added because bbox-center and bbox-size signals are not stable
enough. It is the most plausible next investigation branch, but it is not yet
validated.

Implemented support:

- `scripts/origin_bbox_output_profile_train.py`
  - `CalibrationRecorder` can save left/right ROI crops from ROS image topics.
  - Added CSV fields: `sample_seq`, `left_image_count`, `right_image_count`,
    `left_roi_path`, `right_roi_path`.
  - Default ROI image format is PPM, avoiding the local OpenCV/NumPy ABI issue.
- `scripts/reprojection_profile_random_validation.py`
  - Added ROI capture CLI arguments:
    `--save-roi-images`, `--roi-output-dir`, `--roi-save-stride`,
    `--roi-crop-margin`, `--roi-jpeg-quality`, `--roi-image-format`.
- `scripts/roi_silhouette_offline_validation.py`
  - Offline evaluates ROI silhouette candidates: `centroid`, `bbox_center`,
    `top_center`, `bottom_center`.
  - Missing/invalid/low-contrast ROI or failed triangulation falls back to raw.

Smoke evidence:

- `reports/roi_capture_smoke_20260609_e`
  - `raw_samples.csv` had 34 rows.
  - All 34 rows had ROI paths.
  - 70 ROI PPM files were generated.
- `reports/roi_silhouette_smoke_20260609_e/validation_gate_summary.csv`
  - Pipeline ran, but smoke data was too short for a performance conclusion.
  - Some trial/mode combinations passed, but others regressed sharply.

Representative smoke rows:

| Trial | Mode | Right delta | Down delta | Forward delta | Valid ROI rows | Invalid ROI rows | Verdict |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | `centroid` | 0.002 | 0.291 | 1.223 | 2 | 3 | Pass, too few valid samples |
| 2 | `centroid` | 1.937 | -3.224 | -96.785 | not final | not final | Fail |
| 3 | `top_center` | 3.897 | 2.460 | 127.288 | not final | not final | Pass in smoke only |
| 4 | `centroid` | -0.222 | -3.385 | -142.302 | not final | not final | Fail |
| 5 | `top_center` | 1.058 | 0.611 | 30.656 | not final | not final | Pass in smoke only |
| 6 | `top_center` | 0.361 | 1.230 | 60.482 | not final | not final | Pass in smoke only |

Completed long ROI capture/profile report:

- `reports/roi_reprojection_long_right_att_0p1_seed20260634`
  - Current filesystem state contains `raw_samples.csv`, normalized trial CSVs,
    profile JSON, validation summaries, and ROI images.
  - `raw_samples.csv` rows: 3097.
  - Rows with left/right ROI paths: 514 each.
  - This report is now valid as a completed long-run evidence set.

The completed report's own reprojection gate still failed. It improved Forward
on all five validation trials, but Right/Down had tiny numerical or small-axis
regressions:

| Trial | Samples | Right delta | Down delta | Forward delta | Verdict |
| ---: | ---: | ---: | ---: | ---: | --- |
| 2 | 193 | -0.000000000010 | -0.000000000232 | 0.080683 | Fail |
| 3 | 193 | 0.000000044 | -0.000000043 | 6.973890 | Fail |
| 4 | 193 | -0.000000006 | -0.000000056 | 1.511995 | Fail |
| 5 | 185 | 0.000000040 | 0.000000005 | 3.256851 | Pass |
| 6 | 203 | 0.000000017 | -0.000000100 | 3.924595 | Fail |

Long ROI silhouette evaluation:

- Output: `reports/roi_silhouette_long_right_att_0p1_seed20260634`
- Command:
  `python3 scripts/roi_silhouette_offline_validation.py reports/roi_reprojection_long_right_att_0p1_seed20260634 --output-dir reports/roi_silhouette_long_right_att_0p1_seed20260634`
- Final pass: no.
- Valid silhouette triangulation rows: 159 total across five validation trials.
- Invalid/missing ROI fallback rows: 808 total.

Aggregate by observation point:

| Mode | Passes | Min Right | Min Down | Min Forward | Mean Forward | Verdict |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| `centroid` | 0/5 | -0.043331 | -0.305882 | -2.798088 | 3.455712 | Reject |
| `bbox_center` | 1/5 | -0.043630 | -0.337556 | 0.092354 | 4.032797 | Reject |
| `top_center` | 3/5 | -0.043303 | 0.045742 | 0.471196 | 4.061685 | Best ROI candidate, but Right regresses |
| `bottom_center` | 0/5 | -0.043808 | -1.531533 | 0.117287 | 4.123975 | Reject |

Conclusion for ROI/silhouette so far:

- `top_center` is the only promising ROI observation. It improves Down and
  Forward on every validation trial in the completed long report, but fails the
  all-axis RDF gate due to small Right-axis regressions on trials 2 and 5.
- Increase ROI save density if re-running the simulation. The completed long
  report has only 514 ROI-tagged rows out of 3097 samples, which limits image
  observation coverage.

Axis-gated ROI fusion result:

- Script: `scripts/roi_silhouette_axis_gain_sweep.py`
- Output: `reports/roi_silhouette_axis_gain_long_right_att_0p1_seed20260634`
- Best candidate:
  - mode: `top_center`
  - gains: `Right=0.0`, `Down=0.75`, `Forward=0.75`
  - pass_all_nonregression: yes
  - min RDF improvements: Right `0.000000`, Down `0.055388`,
    Forward `0.514751`
  - mean Position improvement: `3.732222`
  - valid silhouette triangulation rows: 159
  - invalid/missing ROI fallback rows: 808

Best candidate by validation trial:

| Trial | Samples | Right delta | Down delta | Forward delta | Position delta | Valid ROI | Invalid ROI | Verdict |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 2 | 193 | 0.000000 | 0.055388 | 1.035146 | 0.997704 | 32 | 161 | Pass |
| 3 | 193 | 0.000000 | 0.082140 | 13.144780 | 13.136405 | 32 | 161 | Pass |
| 4 | 193 | 0.000000 | 0.075247 | 1.669785 | 1.668974 | 32 | 161 | Pass |
| 5 | 185 | 0.000000 | 0.076085 | 0.514751 | 0.515630 | 30 | 155 | Pass |
| 6 | 203 | 0.000000 | 0.075307 | 2.342271 | 2.342399 | 33 | 170 | Pass |

Current ROI interpretation:

- This is the first non-no-op candidate in this session that passes RDF
  non-regression on all five validation trajectories while improving Down and
  Forward.
- It is still not enough to declare final stability because it has only been
  proven on one completed long ROI report and because most rows are ROI
  fallback rows.
- Next validation should run at least several new random seeds with denser ROI
  capture, then re-run the same axis-gain sweep. Treat a single-seed pass as a
  candidate, not a final solution.

Fixed-candidate coverage diagnostics:

- Script: `scripts/roi_axis_gain_candidate_diagnostics.py`
- Output: `reports/roi_axis_gain_candidate_diagnostics_seed20260634`
- Candidate: `top_center`, gains `Right=0.0`, `Down=0.75`,
  `Forward=0.75`.
- Overall validation rows: 967.
- Valid ROI triangulation rows: 159.
- Overall improvements:
  - Right `0.000000`
  - Down `0.069958`
  - Forward `4.166664`
  - Position `4.164762`
- Trial-level result: all validation trials 2-6 pass RDF non-regression.
- Raw range bin result: all populated range bins pass RDF non-regression.
  The weakest Forward bin is `0-500 m` with `+0.834266 m`; the weakest Down
  bin is `800-1100 m` with `+0.028605 m`.
- Screen-position result:
  - Profile collection covered all 3x3 screen bins
    (`profile_coverage_summary.json` has `screen_bin_coverage_ratio=1.0`).
  - Validation samples only occupied screen bins `u0/v1`, `u1/v1`, and
    `u2/v1`; all valid ROI samples are also in this middle vertical band.
  - Therefore this candidate is not yet proven over top/bottom screen regions
    during validation, even though the profile trajectory covered them.

Bootstrap robustness diagnostics:

- Script: `scripts/roi_axis_gain_bootstrap_diagnostics.py`
- Output: `reports/roi_axis_gain_bootstrap_diagnostics_seed20260634`
- Input: `reports/roi_axis_gain_candidate_diagnostics_seed20260634/candidate_sample_diagnostics.csv`
- Command used 1000 bootstrap iterations with seed `20260609`.
- Overall row bootstrap:
  - RDF pass fraction: `1.000`
  - Down 2.5th percentile: `+0.049612 m`
  - Forward 2.5th percentile: `+2.502812 m`
- Trial-cluster bootstrap:
  - RDF pass fraction: `1.000`
  - Down 2.5th percentile: `+0.062356 m`
  - Forward 2.5th percentile: `+0.631358 m`
- Fragile subgroups:
  - range bin `800-1100 m`: pass fraction `0.946`, Forward 2.5th
    percentile `-0.380184 m`.
  - screen bin `u0/v1`: pass fraction `0.550`, Forward 2.5th percentile
    `-1.023271 m`; only 29 samples and 5 valid ROI rows.
  - screen bin `u2/v1`: pass fraction `0.936`, Down 2.5th percentile
    `-0.016472 m`; only 49 samples and 8 valid ROI rows.
  - trials 2, 5, and 6 have negative Forward 2.5th percentiles under row
    bootstrap despite positive base RMSE improvements.
- Interpretation: the current candidate remains promising, but the single-seed
  pass is not statistically strong enough to claim stable performance. More
  valid ROI observations and new screen-coverage validation seeds are required.

Conservative gain robustness comparison:

- Output: `reports/roi_axis_gain_bootstrap_gain_comparison_seed20260634.csv`
- Compared `top_center` candidates with Right gain fixed at `0.0` and lower
  Down/Forward gains.
- Best subgroup bootstrap pass fractions came from much more conservative
  candidates:
  - `Down=0.0`, `Forward=0.15`: worst subgroup pass fraction `0.822`,
    base Position improvement `+1.177841 m`, but Down is no-op and the worst
    Forward 2.5th percentile is still `-0.151269 m`.
  - `Down=0.0`, `Forward=0.25`: worst subgroup pass fraction `0.816`,
    base Position improvement `+1.869168 m`, but Down is no-op and the worst
    Forward 2.5th percentile is `-0.259297 m`.
  - `Down=0.1`, `Forward=0.1`: worst subgroup pass fraction `0.811`,
    base Position improvement `+0.804073 m`, but both Down and Forward have
    negative subgroup lower tails.
- The original stronger candidate `Down=0.75`, `Forward=0.75` has much larger
  base Position improvement `+4.164762 m`, but a weaker worst subgroup pass
  fraction `0.550`.
- Interpretation: reducing gains trades away useful correction but still does
  not remove subgroup fragility. No tested single-seed ROI gain is strong enough
  to declare stable without additional data.

ROI axis-gain stability gate:

- Script: `scripts/roi_axis_gain_stability_gate.py`
- Output:
  `reports/roi_axis_gain_stability_gate_seed20260634/stability_gate_summary.json`
- Purpose: reject weak evidence before calling an ROI axis-gain candidate
  stable. The gate checks completed report count, skipped/partial inputs,
  aggregate RDF non-regression, validation ROI density, validation screen
  coverage, profile screen coverage, trajectory-generator screen coverage, and
  bootstrap subgroup lower tails.
- Command used:
  `python3 scripts/roi_axis_gain_stability_gate.py --aggregate-summary reports/roi_silhouette_axis_gain_multi_seed_summary_20260609/axis_gain_multi_report_summary.csv --skipped-inputs reports/roi_silhouette_axis_gain_multi_seed_summary_20260609/skipped_inputs.csv --candidate-mode top_center --right-gain 0 --down-gain 0.75 --forward-gain 0.75 --candidate-sample-diagnostics reports/roi_axis_gain_candidate_diagnostics_seed20260634/candidate_sample_diagnostics.csv --bootstrap-summary reports/roi_axis_gain_bootstrap_diagnostics_seed20260634/bootstrap_summary.csv --profile-coverage-summary reports/roi_reprojection_long_right_att_0p1_seed20260634/profile_coverage_summary.json --trajectory-diagnostics reports/trajectory_diagnostics_screen_coverage_seed20260635/trajectory_projection_diagnostic.json --output-dir reports/roi_axis_gain_stability_gate_seed20260634`
- Result: fail, 18 failed checks out of 77.
- Key failures:
  - Completed reports: `1`, required `>=6`.
  - Skipped inputs: `1`, required `0`.
  - Valid ROI fraction: `0.164426`, required `>=0.20`.
  - Candidate validation screen bins: `3/9`, required `9/9`.
  - Minimum valid ROI rows per screen bin: `5`, required `>=10`.
  - Fragile bootstrap groups remain: range bin `2`, screen bins `0:1` and
    `2:1`, and trials `2`, `5`, `6`.
- Passing checks still matter: the selected aggregate row itself has
  `pass_all_reports=yes` for the single completed report, aggregate min RDF
  improvements are non-negative, the profile collection covered all 3x3 screen
  bins, and the new trajectory generator diagnostic covers all 3x3 validation
  bins where configured. The failure is therefore evidence sufficiency and
  subgroup robustness, not a broken command pipeline.

Axis-separated ROI Down + range-only Forward fusion:

- Script: `scripts/roi_down_range_forward_fusion.py`
- Idea: keep Right as raw/no-op, apply ROI silhouette only to Down, and apply
  origin-bbox reprojection/range-only output only to Forward. This avoids using
  full ROI triangulation Forward, which had larger subgroup fragility, while
  preserving the most stable Forward signal already seen in
  `range_only_forward_fusion`.
- Main output:
  `reports/roi_down_range_forward_fusion_seed20260634`
- Best single-seed candidate from the sweep:
  - mode: `top_center`
  - gains: `Right=0.0`, `Down=0.75`, `Forward=1.0`
  - pass_all_nonregression: yes across validation trials 2-6
  - min RDF improvements: Right `0.000000000`, Down `0.055388319`,
    Forward `0.080683350`
  - mean Position improvement: `3.148112044`
- This improves the previous ROI-only candidate by making Forward come from the
  more stable range-only source. The single-seed stability gate still fails:
  `reports/roi_down_range_forward_fusion_stability_gate_seed20260634/stability_gate_summary.json`.
  It has 13 failed checks, fewer than the ROI-only gate's 18 failed checks.

More conservative fusion:

- Output:
  `reports/roi_down_range_forward_fusion_d0p1_f1_seed20260634`
- Candidate:
  - mode: `top_center`
  - gains: `Right=0.0`, `Down=0.1`, `Forward=1.0`
  - pass_all_nonregression: yes across validation trials 2-6
  - min RDF improvements: Right `0.000000000`, Down `0.008607324`,
    Forward `0.080683350`
  - mean Position improvement: `3.145673924`
- Stability gate:
  `reports/roi_down_range_forward_fusion_d0p1_f1_stability_gate_seed20260634/stability_gate_summary.json`
  still fails, but failed checks drop to 10.
- Remaining failures:
  - Completed reports: `1`, required `>=6`.
  - Valid ROI fraction: `0.164426`, required `>=0.20`.
  - Candidate validation screen bins: `3/9`, required `9/9`.
  - Minimum valid ROI rows per screen bin: `5`, required `>=10`.
  - Bootstrap weak groups: range bin `0`, screen bin `0:1`, and trial `2`.
- Lower Forward gains were also checked with `Down=0.1`:
  - `Forward=0.5`: all validation trials pass, but bootstrap range bin `0`
    pass fraction is `0.930` and trial `2` pass fraction is `0.931`.
  - `Forward=0.25`: all validation trials pass, but bootstrap range bin `0`
    pass fraction is `0.936` and trial `2` pass fraction is `0.936`.
  - `Forward=0.1`: all validation trials pass, but bootstrap range bin `0`
    pass fraction is `0.939` and trial `2` pass fraction is `0.941`.
- Interpretation: lowering Forward gain reduces the negative lower-tail
  magnitude but does not clear the 0.95 subgroup bootstrap pass gate unless
  Forward becomes effectively no-op. Lowering Down gain helps, but the weak
  `screen_bin=0:1` ROI subgroup remains limited by only 5 valid ROI rows.
- Current rank: this is the best non-no-op branch after
  `range_only_forward_fusion`, but it is still not stable enough to integrate
  as the main method without multi-seed, denser ROI, and screen-coverage
  validation evidence.

Guarded axis-separated fusion:

- Output:
  `reports/roi_guarded_down_range_forward_fusion_seed20260634`
- Policy:
  - Right remains raw/no-op.
  - Down uses ROI `top_center` with gain `0.1`, but screen bin `0:1` is
    guarded to no-op.
  - Forward uses origin-bbox reprojection with gain `1.0`, but raw range
    `<500 m` is guarded to no-op.
  - Both guards use runtime-available fields only: pre-correction bbox screen
    bin and raw range.
- Candidate summary:
  - pass_all_nonregression: yes across validation trials 2-6
  - min RDF improvements: Right `0.000000000`, Down `0.008607324`,
    Forward `0.146924214`
  - mean improvements: Down `0.014942150`, Forward `3.150931371`,
    Position `3.146453707`
  - valid ROI rows: `159`; invalid/missing ROI rows: `808`
  - range-guarded Forward no-op rows: `411`
  - screen-guarded Down no-op rows: `29`
- Bootstrap:
  `reports/roi_guarded_down_range_forward_fusion_bootstrap_seed20260634/bootstrap_summary.csv`
  - Overall row and trial-cluster bootstrap pass fractions: `1.000`.
  - All populated range bins, screen bins, and trials pass the `0.95`
    subgroup pass fraction gate.
  - All RDF 2.5th percentile subgroup improvements are non-negative.
- Stability gate:
  `reports/roi_guarded_down_range_forward_fusion_stability_gate_seed20260634/stability_gate_summary.json`
  still fails, but only 4 checks fail out of 77:
  - Completed reports: `1`, required `>=6`.
  - Valid ROI fraction: `0.164426`, required `>=0.20`.
  - Candidate validation screen bins: `3/9`, required `9/9`.
  - Minimum valid ROI rows per screen bin: `5`, required `>=10`.
- Interpretation: this is now the strongest non-no-op candidate. The remaining
  failures are evidence/coverage sufficiency, not current single-seed RDF or
  bootstrap subgroup regression. It should be the first candidate evaluated
  when unsandboxed multi-seed ROI validation becomes available.

Guard policy sweep:

- Script: `scripts/roi_guard_policy_sweep.py`
- Input:
  `reports/roi_down_range_forward_fusion_d0p1_f1_seed20260634/candidate_sample_diagnostics.csv`
- Output:
  `reports/roi_guard_policy_sweep_d0p1_f1_seed20260634_tol/guard_policy_candidate_summary.csv`
- Purpose: sweep runtime-available no-op guards over the unguarded
  `Down=0.1`, `Forward=1.0` candidate. Guards use only raw range and screen bin;
  trial/truth-based guards are intentionally not allowed.
- Findings:
  - `forward_min=500`, `disable_down=0:1` remains the best fully gate-checked
    policy.
  - `forward_min=300`, `disable_down=0:1` looked attractive in the quick sweep
    because it preserves more Forward correction, but the 1000-iteration
    stability gate failed on `bootstrap_forward_p025:trial=2 = -0.004600162`.
    Output:
    `reports/roi_guarded_down_range_forward_fusion_fmin300_stability_gate_seed20260634/stability_gate_summary.json`.
  - `forward_min=800`, `disable_down=0:1` is more conservative and reaches a
    perfect quick-sweep pass fraction, but it guards 621 rows and lowers mean
    Forward improvement from about `3.55 m` to `2.33 m`, so it is less useful
    than the current `500 m` guard unless future seeds show `500 m` is unstable.
- Interpretation: current best policy is not arbitrary. It is the strongest
  candidate that cleared the full 1000-iteration single-seed bootstrap subgroup
  gate while retaining materially useful Forward correction.

Tiny Forward correction guard follow-up:

- Script support added:
  `scripts/roi_down_range_forward_fusion.py --forward-min-correction-abs-m`
  and the same runner option in
  `scripts/run_guarded_roi_fusion_multi_seed_validation.py`.
- Purpose: if the Forward source differs from raw Forward by only an epsilon-
  scale amount, use raw/no-op for Forward instead of allowing tiny numerical
  RMSE regressions to fail strict non-regression checks. This guard is
  runtime-available because it uses only the raw estimate and the precomputed
  Forward source, not truth/trial labels.
- Long seed output:
  `reports/roi_guarded_down_range_forward_fusion_tiny0p001_seed20260634`
- Policy:
  - Right remains raw/no-op.
  - Down uses ROI `top_center` with gain `0.1`; screen bin `0:1` is guarded
    to no-op.
  - Forward uses origin-bbox reprojection with gain `1.0`; raw range `<500 m`
    is guarded to no-op; `abs(forward_source - raw_forward) < 0.001 m` is also
    guarded to no-op.
- Long seed candidate summary:
  - pass_all_nonregression: yes across validation trials 2-6.
  - min RDF improvements: Right `0.000000000`, Down `0.008607324`,
    Forward `0.146924214`.
  - mean improvements: Down `0.014942150`, Forward `3.150930218`,
    Position `3.146452555`.
  - valid ROI rows: `159`; invalid/missing ROI rows: `808`.
  - range-guarded Forward no-op rows: `411`.
  - tiny-correction guarded Forward no-op rows: `78`.
  - screen-guarded Down no-op rows: `29`.
- Short smoke comparison:

| Report | Guard | Pass all | Min Right | Min Down | Min Forward | Mean Forward | Note |
| --- | --- | --- | ---: | ---: | ---: | ---: | --- |
| `reports/roi_guarded_down_range_forward_fusion_smoke_c` | range/screen only | no | 0.000000000 | 0.000000000 | -0.000014633 | 0.000011618 | tiny Forward regression |
| `reports/roi_guarded_down_range_forward_fusion_tiny0p001_smoke_c` | range/screen/tiny | yes | 0.000000000 | 0.000000000 | 0.000000000 | 0.000000000 | all corrections guarded/no-op |
| `reports/roi_guarded_down_range_forward_fusion_smoke_d` | range/screen only | no | 0.000000000 | 0.000000000 | -0.000017802 | -0.000012053 | tiny Forward regression |
| `reports/roi_guarded_down_range_forward_fusion_tiny0p001_smoke_d` | range/screen/tiny | yes | 0.000000000 | 0.000000000 | 0.000000000 | 0.000000000 | all corrections guarded/no-op |
| `reports/roi_guarded_down_range_forward_fusion_smoke_e` | range/screen only | no | 0.000000000 | 0.023388008 | -0.000064120 | -0.000019904 | tiny Forward regression |
| `reports/roi_guarded_down_range_forward_fusion_tiny0p001_smoke_e` | range/screen/tiny | yes | 0.000000000 | 0.023388008 | 0.000000000 | 0.000000000 | tiny Forward regression removed |

- Aggregate:
  `reports/roi_guarded_down_range_forward_fusion_tiny0p001_aggregate_seed20260634/axis_gain_multi_report_summary.csv`
  - Loaded reports: `1`.
  - Skipped inputs: `0`.
  - Candidate row `top_center`, gains `Right=0.0`, `Down=0.1`,
    `Forward=1.0` has `pass_all_reports=yes`.
- Bootstrap:
  `reports/roi_guarded_down_range_forward_fusion_tiny0p001_bootstrap_seed20260634/bootstrap_summary.csv`
  - Overall row and trial-cluster bootstrap pass fractions: `1.000`.
  - All populated range bins and trials pass the `0.95` subgroup pass fraction
    gate.
  - The weakest populated screen bin is `u2/v1`, pass fraction `0.997`, still
    above the gate.
  - All RDF 2.5th percentile subgroup improvements are non-negative.
- Stability gate:
  `reports/roi_guarded_down_range_forward_fusion_tiny0p001_stability_gate_seed20260634/stability_gate_summary.json`
  still fails, with the same 4 evidence/coverage failures as the non-tiny
  guarded candidate:
  - Completed reports: `1`, required `>=6`.
  - Valid ROI fraction: `0.164426`, required `>=0.20`.
  - Candidate validation screen bins: `3/9`, required `9/9`.
  - Minimum valid ROI rows per screen bin: `5`, required `>=10`.
- Interpretation: the `0.001 m` tiny guard should be the next multi-seed
  candidate. It removes strict-gate epsilon Forward regressions in smoke C/D/E
  without changing the long-seed minimum RDF improvements or introducing new
  bootstrap subgroup failures. It is still not final because the remaining
  failures are report count, ROI density, and validation screen coverage.

Multi-report aggregate support:

- Script: `scripts/roi_silhouette_axis_gain_aggregate.py`
- Batch runner: `scripts/run_roi_axis_gain_multi_seed_validation.py`
- Guarded fusion batch runner:
  `scripts/run_guarded_roi_fusion_multi_seed_validation.py`
- Candidate sample aggregate:
  `scripts/roi_candidate_sample_aggregate.py`
- Output: `reports/roi_silhouette_axis_gain_multi_seed_summary_20260609`
- Purpose: aggregate completed `axis_gain_candidate_summary.csv` files across
  multiple report directories and reject missing/partial inputs as skipped
  inputs rather than evidence.
- Current aggregate evidence:
  - Loaded completed reports: 1
  - Skipped inputs: 1
  - Skipped partial input:
    `reports/roi_reprojection_long_right_att_0p1_seed20260635_stride3`
    because it has no `axis_gain_candidate_summary.csv`.
  - Best aggregate row remains `top_center`, gains
    `Right=0.0`, `Down=0.75`, `Forward=0.75`, but this is still single-report
    evidence because only seed `20260634` is complete.
- Runner dry-run verification:
  - `python3 scripts/run_roi_axis_gain_multi_seed_validation.py --dry-run --seeds 20260635,20260636 --include-sweep-dir reports/roi_silhouette_axis_gain_long_right_att_0p1_seed20260634 --aggregate-output-dir reports/roi_silhouette_axis_gain_multi_seed_summary_dryrun`
  - Verified command construction for validation, sweep, and aggregate without
    launching simulation.
- Guarded runner dry-run verification:
  - `python3 scripts/run_guarded_roi_fusion_multi_seed_validation.py --dry-run --seeds 20260635,20260636 --include-fusion-dir reports/roi_guarded_down_range_forward_fusion_seed20260634 --aggregate-output-dir reports/roi_guarded_down_range_forward_fusion_multi_seed_dryrun`
  - Verified command construction for validation, guarded fusion, candidate
    aggregate, sample aggregate, bootstrap, and stability gate without launching
    simulation.
- Guarded runner offline verification:
  - `python3 scripts/run_guarded_roi_fusion_multi_seed_validation.py --seeds '' --include-fusion-dir reports/roi_guarded_down_range_forward_fusion_seed20260634 --aggregate-output-dir reports/roi_guarded_down_range_forward_fusion_runner_seed20260634_offline`
  - Candidate aggregate, sample aggregate, and bootstrap completed against the
    existing seed `20260634` guarded fusion output.
  - Stability gate failed as expected with 4 failed checks, matching the
    single-seed evidence insufficiency already recorded.
  - Output:
    `reports/roi_guarded_down_range_forward_fusion_runner_seed20260634_offline`.
- Sample aggregate verification:
  - Output:
    `reports/roi_guarded_down_range_forward_fusion_sample_aggregate_seed20260634_with_partial`
  - Loaded completed guarded fusion diagnostics from seed `20260634`.
  - Skipped partial input
    `reports/roi_reprojection_long_right_att_0p1_seed20260635_stride3`
    because it has no `candidate_sample_diagnostics.csv`.

Validation trajectory coverage fix:

- Script updated: `scripts/reprojection_profile_random_validation.py`
- Added mode: `--validation-trajectory-mode screen_coverage`
- Added diagnostic-only path: `--trajectory-diagnostics-only`
- Runner default updated to use screen-coverage validation with:
  - `--validation-screen-u-fracs 0.31,0.50,0.69`
  - `--validation-screen-v-fracs 0.31,0.50,0.69`
  - `--validation-ranges-m 300,450,650,950,1300`
  - `--validation-altitude-min-m 1.0`
  - `--validation-altitude-max-m 25.0`
  - `--validation-waypoint-repeat 2`
- Dry-run/geometry evidence:
  - Output: `reports/trajectory_diagnostics_screen_coverage_seed20260635`
  - Validation screen coverage: `9/9`, ratio `1.000`.
  - Trial 2 covers all 3x3 bins at 300 m.
  - Trial 3 covers all 3x3 bins at 450 m.
  - Trials 4-5 cover feasible upper/middle bins at 650/950 m under altitude
    limits.
  - Trial 6 covers the middle vertical band at 1300 m because top/bottom screen
    positions are infeasible with the configured altitude limits.
  - This fixes the previous completed seed's validation weakness where all
    validation samples were only in `v_bin=1`.

Next recommended validation step:

1. Treat bbox-anchor all-family axis guard as the current best offline-stable
   candidate to beat:
   `rg=0.1`, `dg=0`, `fg=1`, `rmin=1100`, `fmin=500`.
2. Treat the all-family micro Down guard only as a near-no-op sanity baseline:
   `feature=range_bin`, `dg=0.15`, `min_samples=20`,
   `min_train_down=0.02`. It passes deterministic `90/90` report+trial
   holdouts, but applies Down to only `83/33570` samples and has bootstrap200
   row pass fraction `0.975`, so it does not solve Down.
3. Treat the vertical-global LORO range-bin Down guard as the best
   family-local bbox-derived Down-improving branch, but not as an all-family
   candidate. The stricter vertical-only policy is:
   `feature=range_bin`, `dg=0.25`, `min_samples=20`,
   `min_train_down=0.01`. It failed all-family replay (`15/90` failures), so
   it needs new vertical-global-only profile/validation reports before it can
   be trusted even inside that family.
4. Re-run complete random validation for the bbox-axis candidate, the
   vertical-global family-local Down candidate, and the tiny-guarded ROI
   candidate on the same new seeds. The bbox candidate is simpler and more
   stable but leaves Down no-op; the vertical-global candidate is the strongest
   narrow bbox-derived Down branch; the ROI candidate is weaker on evidence but
   uses an independent image observation.
5. If ROI validation is re-run, use lower `--roi-save-stride` if runtime and
   storage allow it.
6. If multi-seed validation holds, implement runtime application with the same
   protected fusion policy: invalid ROI or failed triangulation falls back to
   raw/no-op, raw-range guards are applied before correction, and any weak axis
   remains raw/no-op rather than being forced through a fragile correction.

Offline command to reproduce the current bbox-axis candidate:

```bash
python3 scripts/bbox_anchor_axis_guard_sweep.py \
  reports/bbox_anchor_disparity_bin_right_att_0p1_seed20260625 \
  reports/bbox_anchor_disparity_bin_right_att_0p1_seed20260626 \
  reports/bbox_anchor_disparity_bin_right_att_0p1_seed20260626_ultralong \
  reports/bbox_anchor_global_fullscreen_right_att_0p1_seed20260625 \
  reports/bbox_anchor_global_right_att_0p1_seed20260623 \
  reports/bbox_anchor_global_right_att_0p1_seed20260625 \
  reports/bbox_anchor_global_right_att_0p1_seed20260626 \
  reports/bbox_anchor_global_right_att_0p1_seed20260626_ultralong \
  reports/bbox_anchor_global_screen_coverage_right_att_0p1_seed20260624 \
  reports/bbox_anchor_screen3x3_min100_right_att_0p1_seed20260625 \
  reports/bbox_anchor_screen3x3_min100_right_att_0p1_seed20260626 \
  reports/bbox_anchor_screen3x3_min100_right_att_0p1_seed20260626_ultralong \
  reports/bbox_anchor_screen5x3_right_att_0p1_seed20260625 \
  reports/bbox_anchor_screen5x3_right_att_0p1_seed20260626 \
  reports/bbox_anchor_screen5x3_right_att_0p1_seed20260626_ultralong \
  reports/bbox_anchor_vertical_global_right_att_0p1_seed20260625 \
  reports/bbox_anchor_vertical_global_right_att_0p1_seed20260626 \
  reports/bbox_anchor_vertical_global_right_att_0p1_seed20260626_ultralong \
  --output-dir reports/bbox_anchor_axis_guard_fixed_rg0p1_down0_all_anchor_reports \
  --right-gains 0.1 --down-gains 0 --forward-gains 1 \
  --right-min-ranges-m 1100 --down-min-ranges-m 0 --forward-min-ranges-m 500 \
  --diagnostic-right-gain 0.1 --diagnostic-down-gain 0 --diagnostic-forward-gain 1 \
  --diagnostic-right-min-range-m 1100 \
  --diagnostic-down-min-range-m 0 \
  --diagnostic-forward-min-range-m 500
```

Offline command to reproduce the all-family micro Down guard:

```bash
python3 scripts/bbox_anchor_cell_guard_cross_validation.py \
  reports/bbox_anchor_disparity_bin_right_att_0p1_seed20260625 \
  reports/bbox_anchor_disparity_bin_right_att_0p1_seed20260626 \
  reports/bbox_anchor_disparity_bin_right_att_0p1_seed20260626_ultralong \
  reports/bbox_anchor_global_fullscreen_right_att_0p1_seed20260625 \
  reports/bbox_anchor_global_right_att_0p1_seed20260623 \
  reports/bbox_anchor_global_right_att_0p1_seed20260625 \
  reports/bbox_anchor_global_right_att_0p1_seed20260626 \
  reports/bbox_anchor_global_right_att_0p1_seed20260626_ultralong \
  reports/bbox_anchor_global_screen_coverage_right_att_0p1_seed20260624 \
  reports/bbox_anchor_screen3x3_min100_right_att_0p1_seed20260625 \
  reports/bbox_anchor_screen3x3_min100_right_att_0p1_seed20260626 \
  reports/bbox_anchor_screen3x3_min100_right_att_0p1_seed20260626_ultralong \
  reports/bbox_anchor_screen5x3_right_att_0p1_seed20260625 \
  reports/bbox_anchor_screen5x3_right_att_0p1_seed20260626 \
  reports/bbox_anchor_screen5x3_right_att_0p1_seed20260626_ultralong \
  reports/bbox_anchor_vertical_global_right_att_0p1_seed20260625 \
  reports/bbox_anchor_vertical_global_right_att_0p1_seed20260626 \
  reports/bbox_anchor_vertical_global_right_att_0p1_seed20260626_ultralong \
  --output-dir reports/bbox_anchor_cell_guard_report_trial_all_anchor_micro_down_guard_20260611 \
  --holdout-mode report_trial \
  --feature-sets range_bin \
  --down-gains 0.15 \
  --train-min-samples 20 \
  --train-min-down-improvements-m 0.02
```

Recommended ROI command once unsandboxed ROS2/Gazebo execution is authorized:

```bash
python3 scripts/run_guarded_roi_fusion_multi_seed_validation.py \
  --seeds 20260635,20260636,20260637,20260638,20260639 \
  --include-fusion-dir reports/roi_guarded_down_range_forward_fusion_seed20260634 \
  --forward-min-correction-abs-m 0.001 \
  --aggregate-output-dir reports/roi_guarded_down_range_forward_fusion_multi_seed_next
```

If future seeds show the tiny guard unexpectedly harms subgroup performance,
remove `--forward-min-correction-abs-m 0.001` and fall back to the previous
guarded candidate from `reports/roi_guarded_down_range_forward_fusion_seed20260634`.

Success condition for declaring the bbox-axis branch stable:

- At least five new random validation reports plus existing completed reports
  are loaded for the bbox-anchor disparity-bin axis-guard candidate.
- The fixed candidate `rg=0.1`, `dg=0`, `fg=1`, `rmin=1100`, `fmin=500`
  passes all validation trials with RDF non-regression.
- Aggregate report, trial, and raw-range-bin groups pass RDF non-regression.
- Bootstrap row, trial-cluster, trial, and range-bin groups keep pass fraction
  `>=0.95` and non-negative RDF p025 values.
- The limitation remains explicit: this branch is stable non-noop for
  Right/Forward, but Down is no-op.

Success condition for declaring the vertical-global LORO Down branch stable:

- At least five new vertical-global profile/validation reports are generated
  under the same `0.1 deg` right-camera roll/pitch/yaw attitude-error
  scenario.
- The LORO policy
  `feature=range_bin`, `dg=0.2`, `min_samples=20`,
  `min_train_down=0.005` is re-evaluated without using each held-out report's
  labels to choose enabled Down cells.
- All validation reports and trials pass RDF non-regression.
- Down is meaningfully applied in most validation trials, not just a small
  subset. The current evidence is not enough because 2 of 15 vertical-global
  trials still use no Down correction and only 3 reports exist.
- Bootstrap row, trial-cluster, trial, and range-bin groups keep pass fraction
  `>=0.95` and non-negative RDF p025 values.
- Any cell-level subgroup that shows Forward or lateral regression is either
  guarded off by runtime-available conditions or explicitly documented as a
  non-blocking subgroup outside the acceptance scope.

Success condition for declaring the ROI Down branch stable:

- At least five new completed validation reports plus the existing seed
  `20260634` are loaded by the aggregate.
- The guarded candidate key `top_center`, gains `Right=0.0`, `Down=0.1`,
  `Forward=1.0`, raw range guard `<500 m`, optional tiny Forward correction
  guard `0.001 m`, and Down screen guard `0:1` has `pass_all_reports=yes`.
- No input directory is skipped because of missing
  `axis_gain_candidate_summary.csv` or `candidate_sample_diagnostics.csv`.
- The aggregate minimum RDF improvements are all non-negative, and Down/Forward
  remain meaningfully positive rather than epsilon-only.
- The aggregated sample diagnostics and bootstrap pass the stability gate:
  valid ROI fraction, validation screen coverage, per-screen valid ROI counts,
  report count, skipped input count, and subgroup lower-tail checks.
- The new reports' `trajectory_projection_diagnostic.json` files must show
  validation screen coverage over all 3x3 bins where physically feasible under
  altitude limits; otherwise a pass should not be treated as full-screen
  stability evidence.

Multi-seed continuation attempt:

- Intended next seed: `20260635`
- Intended output: `reports/roi_reprojection_long_right_att_0p1_seed20260635_stride3`
- Intended change from seed `20260634`: keep the same 0.1 deg right-camera
  attitude error and profile settings, but reduce ROI save stride from `6` to
  `3`.
- Sandbox result:
  - First attempt failed because ROS tried to write logs under
    `/home/moontaehyung/.ros/log`, which is read-only in the sandbox.
  - Second sandbox attempt with `ROS_LOG_DIR=/tmp/...` failed because RTPS/UDP
    socket creation is not permitted in the sandbox.
  - Unsandboxed execution was requested but rejected by the approval reviewer
    because the previous user-visible instruction was to stop and preserve
    current results.
- Current state: `reports/roi_reprojection_long_right_att_0p1_seed20260635_stride3`
  may contain only partial metadata/log files and must not be treated as
  validation evidence.
- Required to continue multi-seed validation: explicit user approval to run the
  ROS2/Gazebo validation command outside the sandbox.
- 2026-06-09 continuation note:
  - The active goal was resumed, but unsandboxed ROS2/Gazebo execution was
    still rejected by the approval reviewer.
  - A safer offline step was completed instead: multi-report aggregate tooling
    was added and verified against the completed seed `20260634` plus the
    partial seed `20260635` directory.
- 2026-06-09 tiny-guard continuation note:
  - Offline tiny-guard checks were completed: smoke E now passes strict RDF
    non-regression, aggregate/bootstrap/stability gate were run for
    `reports/roi_guarded_down_range_forward_fusion_tiny0p001_seed20260634`,
    and `QT_QPA_PLATFORM=offscreen python3 -m pytest -q` passed.
  - A new unsandboxed multi-seed ROS2/Gazebo run was requested with
    `--seeds 20260635,20260636,20260637,20260638,20260639` and
    `--forward-min-correction-abs-m 0.001`, but approval was rejected because
    the last explicit user instruction was to stop and preserve current
    results.
  - Do not treat multi-seed stability as proven. To continue, get explicit
    user approval for the artifact-producing ROS2/Gazebo validation run.

## Test And Verification State

- Earlier full StereoDistance app test run passed:
  `QT_QPA_PLATFORM=offscreen python3 -m pytest -q`
  with `284 passed in 16.19s`.
- After the latest ROI CLI/capture changes, `py_compile` was run on the changed
  scripts and passed.
- `scripts/roi_silhouette_axis_gain_sweep.py` was added and passed
  `python3 -m py_compile`.
- `scripts/roi_silhouette_axis_gain_aggregate.py` was added and passed
  `python3 -m py_compile`.
- `scripts/run_roi_axis_gain_multi_seed_validation.py` was added, passed
  `python3 -m py_compile`, and passed dry-run command construction.
- `scripts/roi_axis_gain_candidate_diagnostics.py` was added, passed
  `python3 -m py_compile`, and produced fixed-candidate coverage diagnostics
  for seed `20260634`.
- `scripts/roi_axis_gain_bootstrap_diagnostics.py` was added, passed
  `python3 -m py_compile`, and showed that the single-seed candidate has
  fragile small-sample subgroups even though the overall bootstrap is positive.
- `scripts/roi_axis_gain_stability_gate.py` was added, passed
  `python3 -m py_compile`, and currently rejects the ROI `top_center`
  `0/0.75/0.75` candidate as insufficiently proven stable:
  `reports/roi_axis_gain_stability_gate_seed20260634/stability_gate_summary.json`.
- `scripts/roi_down_range_forward_fusion.py` was added and used to test
  axis-separated fusion of ROI Down correction with origin-bbox reprojection
  Forward correction. It now supports runtime-available guards for raw range and
  screen bin. The guarded candidate eliminates current bootstrap subgroup
  failures on seed `20260634`, but the stability gate still rejects it as
  insufficiently proven because multi-seed and screen-coverage validation
  evidence is missing.
- `scripts/roi_candidate_sample_aggregate.py` was added, passed
  `python3 -m py_compile`, and verified that partial/missing sample diagnostics
  are skipped rather than counted as evidence.
- `scripts/run_guarded_roi_fusion_multi_seed_validation.py` was added, passed
  `python3 -m py_compile`, passed dry-run command construction for the full
  guarded validation pipeline, and completed the aggregate/bootstrap portions
  offline against the existing seed `20260634` guarded fusion output. The final
  nonzero exit in that offline run is expected because the stability gate still
  fails on single-seed evidence sufficiency.
- `scripts/roi_guard_policy_sweep.py` was added, passed
  `python3 -m py_compile`, and compared runtime-available guard policies over
  the existing unguarded guarded-fusion candidate. It ruled out the tempting
  `forward_min=300` guard after full 1000-iteration stability-gate verification.
- `scripts/validation_gate_aggregate.py` was added and used to aggregate all
  existing validation-gate summaries:
  `reports/validation_gate_global_aggregate_20260609`.
- `scripts/bbox_anchor_axis_guard_sweep.py` was added and used to find the
  current best offline-stable bbox candidate:
  `reports/bbox_anchor_axis_guard_fixed_rg0p1_down0_all_anchor_reports`.
  Bootstrap diagnostics were written to
  `reports/bbox_anchor_axis_guard_fixed_rg0p1_down0_all_anchor_reports_bootstrap`.
- `scripts/bbox_anchor_cell_guard_cross_validation.py` was added to evaluate
  runtime-safe bbox-anchor Down cell/range guards with leave-one-report-out
  policy selection. It produced:
  - `reports/bbox_anchor_axis_guard_down_sweep_all_anchor_reports`, confirming
    that every positive global Down gain still regresses at least one trial.
  - `reports/bbox_anchor_cell_guard_loro_down_sweep_all_anchor_reports`,
    finding only a near-no-op all-family Down candidate.
  - `reports/bbox_anchor_cell_guard_loro_down_sweep_disparity_reports`,
    finding the current best bbox-derived Down-improving branch inside the
    disparity-bin profile family at that point.
  - `reports/bbox_anchor_cell_guard_loro_down_sweep_global_reports`,
    `reports/bbox_anchor_cell_guard_loro_down_sweep_screen3x3_reports`,
    `reports/bbox_anchor_cell_guard_loro_down_sweep_screen5x3_reports`, and
    `reports/bbox_anchor_cell_guard_loro_down_sweep_vertical_global_reports`,
    showing that vertical-global is now the strongest bbox-derived Down branch.
  - corresponding bootstrap summaries under
    `reports/bbox_anchor_cell_guard_loro_down_sweep_all_anchor_reports_bootstrap`
    and
    `reports/bbox_anchor_cell_guard_loro_down_sweep_disparity_reports_bootstrap`.
  - later bootstrap summaries under
    `reports/bbox_anchor_cell_guard_loro_down_sweep_global_reports_bootstrap`
    and
    `reports/bbox_anchor_cell_guard_loro_down_sweep_vertical_global_reports_bootstrap`.
- `scripts/bbox_anchor_cell_guard_cross_validation.py` was later extended with
  `--holdout-mode report_trial`, which excludes both the held-out report and
  the same validation-trial number from Down-policy training. The stricter
  vertical-global fixed candidate is:
  `reports/bbox_anchor_cell_guard_loro_down_sweep_vertical_global_report_trial_holdout_dg0p25_min0p01`
  with bootstrap summary:
  `reports/bbox_anchor_cell_guard_loro_down_sweep_vertical_global_report_trial_holdout_dg0p25_min0p01_bootstrap/bootstrap_summary.csv`.
- `scripts/roi_down_range_forward_fusion.py` and
  `scripts/run_guarded_roi_fusion_multi_seed_validation.py` were extended with
  `--forward-min-correction-abs-m`. The long seed `20260634`, smoke C/D/E,
  aggregate/bootstrap, and stability gate were checked for `0.001 m`.
- Additional conservative gain bootstraps were run and summarized in
  `reports/roi_axis_gain_bootstrap_gain_comparison_seed20260634.csv`; they did
  not produce a robust non-noop candidate.
- `scripts/reprojection_profile_random_validation.py` was updated with
  screen-coverage validation trajectory generation and a diagnostics-only mode;
  it passed `python3 -m py_compile` and produced a 9/9 validation screen
  coverage diagnostic for seed `20260635`.
- `tests/test_roi_validation_tools.py` was added to lock the offline validation
  tooling:
  - screen-coverage trajectory diagnostics must cover all 9 validation screen
    bins without launching ROS/Gazebo.
  - axis-gain aggregate must skip partial inputs instead of treating them as
    validation evidence.
- The same test file now also verifies that the stability gate rejects weak
  evidence, accepts strong synthetic evidence, and that guarded ROI/range fusion
  applies no-op guards correctly. It also verifies sample-diagnostics aggregate
  skipping, guard-policy sweep guard application, bbox-anchor axis range guards,
  and bbox-anchor LORO cell-guard holdout policy application.
- `QT_QPA_PLATFORM=offscreen python3 -m pytest -q tests/test_roi_validation_tools.py -q`
  passed with `10 passed`.
- `QT_QPA_PLATFORM=offscreen python3 -m pytest -q` passed in this Simulation
  workspace with `10 passed in 0.88s` after adding the bbox-anchor LORO
  cell-guard cross-validation test.
- After the tracking/extrinsic rejection update, the same full offscreen pytest
  command passed again with `10 passed in 0.87s`.
- After documenting the multi-source axis LORO fusion result, the same full
  offscreen pytest command passed again with `10 passed in 0.92s`.
- After the family-local bbox-anchor LORO Down sweeps and vertical/global
  bootstrap updates, the same full offscreen pytest command passed again with
  `10 passed in 0.87s`.
- After adding `report_trial` holdout support, the same full offscreen pytest
  command passed again with `10 passed in 0.92s`.
- On 2026-06-11, all-family report+trial replay showed the vertical-global
  Down policy does not generalize across bbox-anchor families (`75/90` pass),
  and the only deterministic all-family pass-all Down policy found was a
  near-no-op micro guard (`83/33570` samples corrected).

Suggested next verification command for app-level tests:

```bash
cd /home/moontaehyung/영상감시시스템/apps/StereoDistance
PYTHONDONTWRITEBYTECODE=1 QT_QPA_PLATFORM=offscreen python3 -m pytest -q -p no:cacheprovider
```

## Stop Point

Stop condition for this session was user-directed. No active simulation process
was found after stopping; only the process-check command itself matched the
simulation search pattern.

Resume from the bbox-anchor all-family axis guard if the goal is stable
Right/Forward correction. Resume from the vertical-global bbox-anchor
`report_trial` range-bin Down branch only as a family-local Down investigation,
not as an all-family candidate. Resume from the ROI/silhouette branch if the
goal is to add a stronger image observation than bbox center. Keep
`range_only_forward_fusion` as the current stable fallback baseline.
