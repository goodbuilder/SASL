# Promoted Methods

No method is currently promoted as hardware-validated complete all-axis
correction.

## Repo-Local Recheck

Machine-readable candidate manifest:
`Simulation/config/promoted_simulation_pass_candidates.json`.

Run the current repo-local software deployment PASS gate with:

```bash
python3 Simulation/scripts/check_promoted_simulation_pass_candidates.py \
  --output-dir /tmp/sasl_promoted_pass_candidates_check
```

Expected result: `status=PASS`, `candidate_count=25`, `pass_count=25`,
`runtime_truth_used=false`, and `coordinate_contracts_explicit=true`.

To also re-run the heavier six-scenario non-alias catalog smoke:

```bash
python3 Simulation/scripts/check_promoted_simulation_pass_candidates.py \
  --output-dir /tmp/sasl_promoted_pass_candidates_check_with_non_alias \
  --include-non-alias-smoke
```

Expected result: `status=PASS`, `candidate_count=26`, `pass_count=26`; the
non-alias candidate must report `deployment_strategy=scenario_profile_catalog`,
`single_profile_pass=false`, selector package `PASS`, operator launch `PASS`,
runtime truth `false`, and direct NED-to-RDF alias forbidden.

## Current Simulation Software Deployment PASS

- `origin-bbox-reprojection-grid-v1 + component_gains + range_only_forward_fusion`
  - Status: `PASS`
  - Report:
    `Simulation/reports/origin_bbox_reprojection_validation_attitude_aware_component_gains_longprofile_right_att_0p1_20260609_seed20260625`
  - Meaning: current saved Simulation-origin reprojection PASS candidate. It is
    an oracle-style Simulation truth validation report, not hardware evidence.
    The policy corrects bbox reprojection with conservative component gains and
    uses the `range_only_forward_fusion` output contract.
  - Evidence: `result.txt` has `final_pass=yes`; `validation_gate_summary.csv`
    has 5/5 validation trials `pass=yes`, 1944 validation samples, and positive
    minimum RMSE improvement on Right, Down, and Forward. Current checker
    result: min Right `0.000065 m`, min Down `0.002199 m`, min Forward
    `3.436333 m`.
  - Promotion status: simulation-origin candidate only; not hardware/WIRIS
    validated.

- `bbox_anchor_global_fullscreen_right_att_0p1`
  - Status: `PASS`
  - Report: `Simulation/reports/bbox_anchor_global_fullscreen_right_att_0p1_seed20260625`
  - Meaning: saved Simulation bbox-anchor PASS candidate. Runtime uses bbox
    `x1/y1/width/height` only, then stereo ray triangulation with right
    extrinsic rotation.
  - Evidence: `result.txt` has `final_pass=yes`; `validation_gate_summary.csv`
    has 5/5 validation trials `pass=yes`, 1654 validation samples, and positive
    minimum RMSE improvement on Right, Down, Forward, and Position. Current
    checker result: min Right `0.235088 m`, min Down `0.000256 m`, min Forward
    `34.786204 m`.
  - Promotion status: simulation saved-report candidate only; not
    hardware/WIRIS validated.

- `origin_bbox_output_validation_8trials_slow`
  - Status: `PASS`
  - Report:
    `Simulation/reports/origin_bbox_output_validation_8trials_slow_20260609_1530_seed20260616`
  - Meaning: legacy origin-bbox output PASS report under
    `stereo_center_frd:forward_right_down`; checker requires corrected MAE to be
    strictly lower for Right, Down, and Forward in committed summary rows.
  - Evidence: `result.txt` has `final_pass=yes`; `summary.csv` has 3/3
    validation trials `pass=yes`, 1127 validation samples, and positive minimum
    MAE improvement on Right, Down, and Forward.
  - Portability note: metadata contains historical machine-local source paths;
    current reproducibility evidence is limited to the committed result and
    summary artifacts.
  - Promotion status: legacy simulation saved-report candidate only; not
    hardware/WIRIS validated.

- `small_drone_multiburst_observation_inner_median_outer_median_b1x3_trim0p2_common0_sim3`
  - Status: `STRICT_PASS`
  - Experiment: `EXP-20260617-213-small-drone-observation-fusion`
  - Meaning: latency-oriented small-drone observation fusion candidate using
    1 burst x 3 observations. It consumes only image `u/v`, measured disparity,
    fixed camera intrinsics, and fixed stereo baseline at runtime.
  - Evidence:
    `Simulation/reports/experiments/EXP-20260617-213-small-drone-observation-fusion/small_drone_observation_method_summary.csv`
    and `small_drone_observation_metrics.json`: close 2m-baseline/4x and mid
    5m-baseline/8x scenarios both passed `50/50`; range RMSE improved
    `0.2775 -> 0.0623 m` and `0.6143 -> 0.0953 m`; position and RDF-axis RMSE
    all improved; target max abs stayed below each scenario limit; runtime truth
    `false`.
  - Promotion status: small-drone synthetic scenario candidate only; not
    hardware/WIRIS validated.

- `small_drone_multiburst_observation_inner_median_outer_median_b3x5_trim0p2_common0_affine_ridge`
  - Status: `STRICT_PASS`
  - Experiment: `EXP-20260617-213-small-drone-observation-fusion`
  - Meaning: quality-oriented small-drone observation fusion candidate using
    3 bursts x 5 observations when 15 observations are affordable at runtime.
  - Evidence:
    `Simulation/reports/experiments/EXP-20260617-213-small-drone-observation-fusion/small_drone_observation_method_summary.csv`
    and `small_drone_observation_metrics.json`: close 2m-baseline/4x and mid
    5m-baseline/8x scenarios both passed `50/50`; range RMSE improved
    `0.2775 -> 0.0337 m` and `0.6143 -> 0.0524 m`; position and RDF-axis RMSE
    all improved; target max abs stayed below each scenario limit; runtime truth
    `false`.
  - Promotion status: small-drone synthetic scenario candidate only; not
    hardware/WIRIS validated.

- BBox-outlier repeated/multiburst observation fusion candidates
  - Status: `STRICT_PASS`
  - Experiments:
    `EXP-20260617-201-bbox-outlier-repeated-observation-fusion`,
    `EXP-20260617-202-bbox-outlier-repeated-observation-fusion-extended`,
    `EXP-20260617-203-bbox-outlier-n15-trim0p1`,
    `EXP-20260617-204-bbox-outlier-n15-trim0p3`,
    `EXP-20260617-205-bbox-outlier-n15-trim0p4`,
    `EXP-20260617-206-bbox-outlier-n21-trim0p2`,
    `EXP-20260617-207-bbox-outlier-n31-trim0p2`,
    `EXP-20260617-208-bbox-outlier-n51-trim0p2`,
    `EXP-20260617-209-bbox-outlier-n75-trim0p2`, and
    `EXP-20260617-210-bbox-outlier-correlated-repeated-observation`,
    `EXP-20260617-211-bbox-outlier-multiburst-common-mode-fusion`, and
    `EXP-20260617-212-bbox-outlier-multiburst-common1-confirmation`
  - Methods:
    `repeated_observation_median_n3_trim0p2`,
    `repeated_observation_median_n7_trim0p2`,
    `repeated_observation_trimmed_mean_n15_trim0p2`,
    `repeated_observation_trimmed_mean_n15_trim0p1`,
    `repeated_observation_trimmed_mean_n15_trim0p3`,
    `repeated_observation_trimmed_mean_n15_trim0p4`,
    `repeated_observation_trimmed_mean_n21_trim0p2`,
    `repeated_observation_trimmed_mean_n31_trim0p2`,
    `repeated_observation_trimmed_mean_n51_trim0p2`,
    `repeated_observation_trimmed_mean_n75_trim0p2`, and
    `correlated_repeated_observation_trimmed_mean_n15_trim0p2_common0`,
    `correlated_repeated_observation_trimmed_mean_n31_trim0p2_common0`,
    `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b5x5_trim0p2_common0p25`, and
    `multiburst_observation_inner_trimmed_mean_outer_median_b3x5_trim0p2_common1`.
  - Evidence: each saved report row is `50/50` `STRICT_PASS`, improves range
    and position RMSE versus raw, keeps target max abs error under the 10m
    simulation gate, declares stereo-center RDF and public FRD+range contracts,
    and records runtime truth `false`.
  - Promotion status: bbox-outlier synthetic scenario candidates only. Repeated
    observation methods require independence and latency validation before real
    deployment; high-count variants are quality/upper-bound evidence.

- `fresh_common_mode_b7x3_aggregate_best + disparity_noise_stress_seed_281000_disparity_residual`
  - Status: `PASS`
  - Experiments:
    `EXP-20260618-215-b7x3-full-followup` and
    `EXP-20260618-281-fresh-stereodistance-disparity-residual-search`
  - Meaning: current repo-reproducible combined observation-fusion plus
    disparity-residual deployment package PASS for the StereoDistance software
    path. The observation-fusion default is 7x3 `MultiBurstObservationFusion`;
    the residual profile is scenario-specific for `disparity_noise_stress`.
  - Evidence:
    `Simulation/config/observation_fusion_fresh_deployment_tiers.json`,
    `Simulation/config/observation_fusion_fresh_deployment_tier_evidence.csv`,
    and
    `Simulation/config/disparity_residual_profile_disparity_noise_stress.json`.
    Current-checkout replay:
    `python3 Simulation/scripts/run_observation_fusion_deployment_package.py --output-dir /tmp/sasl_observation_fusion_with_config_residual_profile_smoke --tier-spec-json Simulation/config/observation_fusion_fresh_deployment_tiers.json --disparity-residual-profile-json Simulation/config/disparity_residual_profile_disparity_noise_stress.json`
    returned `acceptance_status=PASS`, default node smoke
    `fused:MultiBurstObservationFusion:21`, residual node smoke
    `applied:disparity_noise_stress_seed_281000_disparity_residual`,
    operator-view package launch `PASS`, runtime truth `false`, and explicit
    coordinate contracts.
  - Promotion status: simulation/software deployment candidate only; not
    hardware/WIRIS validated. Do not use this residual profile as an
    unconditional all-scenario residual; it is scenario-specific.

- `multiburst_disparity_residual_aggregate_best`
  - Status: `PASS`
  - Experiment: `EXP-20260618-281-fresh-stereodistance-disparity-residual-search`
  - Meaning: aggregate saved-report residual candidate after b7x3 observation
    fusion. Calibration-time truth fits the residual coefficients; runtime
    apply consumes fused disparity/log-disparity plus corrected stereo-center
    RDF only.
  - Evidence:
    `aggregate_summary/multiburst_disparity_residual_summary.json` and
    `multiburst_disparity_residual_scenario_best.csv`: 4/4 stress scenarios
    `STRICT_PASS`, target worst improved `5.0912 -> 2.4378 m`, mean range RMSE
    improved `1.4179 -> 0.4052 m`, mean position RMSE improved
    `2.7947 -> 1.8618 m`, runtime truth `false`, and RDF/public topic contracts
    explicit.
  - Promotion status: simulation aggregate residual candidate only; residual
    catalogs still require explicit deployment labels or truth-free selector
    guards.

- `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_ray_scale_disparity_deg2_ridge0_clip0p25`
  - Status: `STRICT_PASS`
  - Experiment:
    `EXP-20260621-288-b7x3-disparity-residual-all-scenario-fresh50`
  - Meaning: single six-scenario Simulation candidate combining 7x3
    trimmed-mean observation fusion with a degree-2 disparity polynomial
    ray-scale residual. Calibration-time fitting uses train-split truth;
    runtime application consumes fused measured disparity and corrected
    stereo-center RDF only.
  - Evidence: six fresh 50-seed scenario blocks all passed, `300/300` total:
    nominal (`287250..287299`), yaw/pitch (`287300..287349`), bbox-outlier
    (`287350..287399`), combined-sensor (`287400..287449`), disparity-noise
    (`287450..287499`), and intrinsic-scale (`287200..287249`). Worst residual
    target max abs error was `3.382935 m`; intrinsic-scale range RMSE recovered
    from raw `1.851247 m` and observation `2.385011 m` to residual
    `0.223355 m`. An additional intrinsic-scale tail check on seeds
    `287500..287599` also passed `100/100`, with residual target max abs error
    `1.199696 m`. An additional bbox-outlier tail check on seeds
    `287600..287699` passed `100/100`, with residual target max abs error
    `4.067334 m`. Runtime truth was `false`; RDF and public FRD+range
    contracts were explicit.
  - Promotion status: Simulation software PASS candidate; not hardware/WIRIS
    validated and not yet wired into the promoted manifest checker.

- `scenario_labeled_disparity_residual_profile_catalog_exp281`
  - Status: `PASS`
  - Experiment: `EXP-20260618-281-fresh-stereodistance-disparity-residual-search`
  - Meaning: saved runtime-export residual profile catalog for four stress
    scenarios. Runtime selection uses explicit `deployment_label`,
    `scenario_label`, or `scenario_name` only; it must not inspect PX4/RTK
    truth, validation errors, target-window labels, true disparity, or true
    image features.
  - Evidence:
    `runtime_export_scenario_best/scenario_residual_catalog/disparity_residual_profile_label_selector.json`,
    `disparity_residual_profile_catalog.csv`, four `selection_*.json` files,
    four residual profiles, `runtime_export_scenario_best/runtime_export_summary.json`,
    and `runtime_export_scenario_best/operator_view_launch_check_disparity_noise.json`.
    Current checker requires selector `PASS`, 4/4 label selections `PASS`,
    20/20 runtime-export checks `PASS`, runtime truth `false`, stereo-center
    OpenCV optical RDF axis contracts, public FRD+range topic contracts, and
    operator launch residual forwarding `PASS`.
  - Promotion status: simulation saved catalog candidate only; deploy only with
    explicit truth-free operator/deployment labels, not as an unconditional
    single all-scenario residual.

- `scenario_pooled_disparity_residual_catalog_exp289`
  - Status: `PASS`
  - Experiment:
    `EXP-20260621-289-scenario-pooled-disparity-residual-catalog`
  - Meaning: scenario-labeled catalog of runtime-loadable pooled residual
    profiles using the EXP288 b7x3 disparity ray-scale method. Each profile is
    pooled over 50 calibration/evaluation seeds for its own scenario; runtime
    selection must use explicit truth-free deployment/scenario labels.
  - Evidence: six label-specific pooled profiles passed `50/50` each
    (`300/300` total). Fresh label-matched transfer on seeds `288400..288499`
    passed `100/100` for each of six scenarios (`600/600` total). A single
    pooled 6-scenario profile was also tested and failed `100/120` because
    intrinsic-scale failed `0/20`, so this is a catalog method rather than an
    unconditional single profile.
    Runtime truth was `false`; profile application consumes measured disparity
    and stereo-center RDF, with public FRD+range output.
  - Promotion status: simulation scenario-labeled catalog candidate only; not
    hardware/WIRIS validated.

- `fresh_common_mode_b7x3_aggregate_best`
  - Status: `PASS`
  - Experiment: `EXP-20260618-215-b7x3-full-followup`
  - Method:
    `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p2_common0p5`
  - Meaning: current repo-reproducible observation-fusion deployment package
    PASS for the StereoDistance software path. The default tier is an
    all-scenario aggregate `MultiBurstObservationFusion` policy with 7 bursts x
    3 observations = 21 observations.
  - Evidence:
    `Simulation/config/observation_fusion_fresh_deployment_tiers.json` plus
    `Simulation/config/observation_fusion_fresh_deployment_tier_evidence.csv`.
    Current-checkout replay:
    `python3 Simulation/scripts/run_observation_fusion_deployment_package.py --output-dir /tmp/sasl_observation_fusion_repo_tier_package_smoke --tier-spec-json Simulation/config/observation_fusion_fresh_deployment_tiers.json`
    returned `acceptance_status=PASS`, default tier
    `fresh_common_mode_b7x3_aggregate_best`, default node smoke
    `fused:MultiBurstObservationFusion:21`, tier override smoke `2/2`,
    operator-view package launch `PASS`, runtime truth `false`, and explicit
    coordinate contracts.
    Additional fresh bbox-outlier family evidence is recorded in
    `Simulation/reports/experiments/EXP-20260621-286-bbox-outlier-b7x3-observation-fresh50`:
    `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5`
    passed `50/50 STRICT_PASS` on seeds `286500..286549`, with range RMSE
    `2.614845 -> 0.407532 m`, position RMSE `5.084014 -> 0.425134 m`, target
    max abs `3.087384 m`, runtime truth `false`, and explicit image/RDF/FRD
    contracts. This is bbox-outlier family evidence for `trim_fraction=0.05`;
    the deployment tier evidence remains the all-scenario `trim_fraction=0.2`
    aggregate.
  - Promotion status: simulation/software deployment candidate only; not
    hardware/WIRIS validated.

## Current Simulation Catalog PASS Candidate

- `non_alias_scenario_profile_catalog_label_selector`
  - Status: `PASS`
  - Experiment: `EXP-20260618-281-non-alias-robustness-catalog-selector`
  - Meaning: current non-alias robustness route when deployment/scenario labels
    are explicit. A single universal profile is rejected; the valid strategy is
    a scenario profile catalog selected by operator/deployment metadata.
  - Evidence: the extended checker command with `--include-non-alias-smoke`
    runs the current-checkout six-scenario smoke and requires final robustness
    acceptance `PASS`, direct NED-to-RDF alias forbidden 6/6, scenario catalog
    profiles 6/6, StereoDistance promotion 6/6, selector package `PASS`,
    deployment frontier `PASS`, operator launch 19/19 `PASS`, and runtime truth
    `false`.
  - Current smoke strategy: `deployment_strategy=scenario_profile_catalog` and
    `single_profile_pass=false`. The label routes select
    `ridge_global_inverse_depth_linear_lambda1p0` for nominal/intrinsic-scale,
    `ridge_global_inverse_depth_linear_lambda10p0` for disparity-noise/combined
    sensor stress, and
    `target_weighted_disparity_quadratic_then_sim3_target_weighted_quadratic_residual_raw_forward_lut_sigma040`
    for bbox-outlier stress.
  - Promotion status: simulation/software catalog candidate only; not
    hardware/WIRIS validated.

## Current Simulation Runtime PASS Candidate

- `conservative_down_runtime_guard`
  - Status: `PASS`
  - Experiment: `EXP-20260611-013-conservative-down-runtime-guard`
  - Meaning: current live Simulation Runtime PASS under the five-trial
    non-regression MAE gate with right camera roll/pitch/yaw attitude error
    `0.1 deg`.
  - Policy summary: EXP009 runtime policy with Down cell training-improvement
    threshold raised to `0.10 m`, `40` Down cells, stable Right/Forward guard,
    and Forward raw fallback for `anchor_cell=disparity_bin_3`.
  - Promotion status: simulation-runtime candidate only; not hardware/WIRIS
    validated.

## Current Offline PASS Candidate

- `bounded_gain_forward_cell_guard`
  - Status: `PASS`
  - Experiment: `EXP-20260611-009-bounded-gain-forward-cell-guard`
  - Meaning: current offline all-family all-axis PASS under the
    report+trial holdout gate.
  - Policy summary: stable Right/Forward guard, Down selected by
    `profile_family_range_raw_down_center_delta_down_bin` with max selected
    gain `0.25`, and Forward raw fallback for `anchor_cell=disparity_bin_3`.
  - Runtime promotion status: superseded for live Simulation Runtime by
    `EXP-20260611-013-conservative-down-runtime-guard`.

## Stable Baselines

- `range_only_forward_fusion`
  - Status: `STABLE_BASELINE`
  - Meaning: stable Forward improvement with Right/Down protected as no-op.
- `bbox_anchor_axis_guard_rg0p1_dg0_fg1`
  - Status: stable Right/Forward candidate, not a Down solution.
  - Meaning: current best all-family non-noop bbox-anchor branch for
    Right/Forward, with Down no-op.

## Current Audit Exclusions

The current checkout was re-scanned for repo-local PASS evidence after adding
the EXP281 scenario residual catalog. These records are intentionally not
promoted as final Simulation PASS correction methods:

- `Simulation/reports` saved-report audit found no additional all-trial
  `final_pass=yes` report outside the promoted manifest. Many historical
  `validation_gate_summary.csv` files contain one or more `pass=yes` rows, but
  their report-level gates are mixed or incomplete; they remain excluded unless
  `result.txt` reports `final_pass=yes` and every validation trial passes under
  the same coordinate contract. The all-trial local reports found by this scan
  are already covered by promoted candidates:
  `origin_bbox_reprojection_grid_v1_component_gains_range_only_forward_fusion`,
  `bbox_anchor_global_fullscreen_right_att_0p1`, and
  `origin_bbox_output_validation_8trials_slow`.
- `runtime_export_degree1` under
  `EXP-20260618-281-fresh-stereodistance-disparity-residual-search` is not a
  final PASS candidate. It has bbox-outlier runtime residual checks `3/4`
  `PASS`; seed `281003` is `FAIL` because residual range and position RMSE
  regress. The later `runtime_export_scenario_best` catalog supersedes it.
- The four scenario-level EXP281 `multiburst_disparity_residual_metrics.json`
  `STRICT_PASS` methods are covered by
  `scenario_labeled_disparity_residual_profile_catalog_exp281`, not promoted as
  unlabeled universal residuals. Their runtime profiles appear in
  `runtime_export_scenario_best/scenario_residual_catalog/disparity_residual_profile_catalog.csv`
  for `bbox_outlier_stress`, `combined_sensor_stress`,
  `disparity_noise_stress`, and `intrinsic_scale_stress`; the selector may use
  only explicit `deployment_label`, `scenario_label`, or `scenario_name`
  metadata and forbids PX4/RTK truth, validation errors, target-window labels,
  true disparity, and true image features at runtime.
- `EXP-20260617-211-bbox-outlier-multiburst-common-mode-fusion-probe` is not
  promoted separately. It is a 12-trial probe for
  `multiburst_observation_inner_trimmed_mean_outer_median_b5x5_trim0p2_common0p5`;
  the promoted common-mode entry uses the fuller
  `EXP-20260617-211-bbox-outlier-multiburst-common-mode-fusion` report, and
  deployment evidence is superseded by the fresh b7x3 observation-fusion and
  EXP281 residual catalog checks.
- `EXP-20260617-198`, `EXP-20260617-199`, and `EXP-20260617-200` image or
  inverse-depth prefilter reports are not final PASS candidates. Their
  `best_method.status` is `FAIL` and their target-window max errors exceed the
  10 m gate.
- Fresh prefilter rechecks keep inverse-depth filtering as a near-pass lead,
  not a promoted method. Image-space prefiltering on bbox-outlier stress
  (seeds `286200..286204`) stayed `FAIL` with pass count `0/5` and corrected
  range/position RMSE exploding above `960 m`. Inverse-depth prefiltering found
  a small 5/5 `STRICT_PASS` smoke for
  `causal_inverse_depth_prefilter_a1_b0_uv20_id0p001`, but the same branch on a
  50-seed fresh block failed `46/50` with target max abs `12.26 m`. A narrower
  50-seed alpha/clip grid improved to
  `causal_inverse_depth_prefilter_a0p85_b0_uv20_id0p0005`, `49/50`, target max
  abs `10.43 m`; seed `286418` remains outside the 10 m gate. Runtime truth was
  `false` and image/RDF/FRD contracts were explicit, so this is a performance
  tail failure rather than a coordinate-contract failure.
- A finer 50-seed inverse-depth grid around the near-pass point improved the
  best small-scale tail only slightly:
  `causal_inverse_depth_prefilter_a0p86_b0p0001_uv999_id0p0005`, `49/50`,
  target max abs `10.32 m`, still failing seed `286418`. Re-running the same
  family at the default larger sample scale (`1680` samples, target oversample
  `72`) did not validate the near-pass; the best result was
  `causal_inverse_depth_prefilter_a0p84_b0p0005_uv999_id0p0005`, `47/50`,
  target max abs `12.09 m`. Do not promote inverse-depth prefiltering without
  a new tail-safe validation block.
- `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5`
  is not an all-scenario single deployment candidate despite additional fresh
  PASS evidence. The boundary audit
  `EXP-20260621-287-observation-fusion-scenario-boundary-fresh50` found
  `STRICT_PASS` on nominal (`287000..287049`), yaw/pitch stress
  (`287100..287149`), combined-sensor stress (`286600..286649`), and
  disparity-noise stress (`286700..286749`), matching the earlier bbox-outlier
  fresh50 evidence from EXP286. However, intrinsic-scale stress failed `0/50`
  for the direct `trim0p05` run (`286800..286849`) and also failed `0/50`
  across median/trimmed-mean inner/outer grids with trim fractions `0`, `0.1`,
  and `0.2` (`286850..286999`). The failure is the range-improvement gate:
  corrected intrinsic-scale range RMSE stayed around `2.39 m` while raw range
  RMSE stayed around `1.84..1.85 m`. Runtime truth was `false`; image
  `u/v/disparity_px`, stereo-center RDF, and public FRD+range contracts were
  explicit in every saved metrics file.
- Runtime alpha-beta motion fusion over the EXP198 base runtime profile is not
  a promising PASS branch. A fresh bbox-outlier smoke
  (`run_runtime_motion_fusion_sweep.py`, seeds `286000..286002`, RDF axes,
  alpha `0.15/0.3/0.5`, beta `0.02/0.05`, innovation clips `0.5/1.0 m`) found
  best method `causal_alpha_beta_rdf_innovation_clip_a0p5_b0p05_g1` with
  status `FAIL`, pass count `0/3`, range RMSE worsening from about `2.53 m` raw
  to `82.86 m` fused, and position RMSE worsening from about `4.99 m` raw to
  `92.54 m` fused. Runtime truth remained `false`, and the RDF/FRD contracts
  were explicit; the failure is performance, not a coordinate-contract gap.
- Multi-burst observation plus alpha-beta motion fusion is also excluded as a
  motion branch. A matching bbox-outlier smoke
  (`run_runtime_multiburst_motion_fusion_sweep.py`, seeds `286100..286102`,
  b7x3 trimmed-mean observation fusion, RDF alpha-beta grid) found the best
  motion method `FAIL`, pass count `0/3`. The observation stage itself improved
  range RMSE to about `0.39 m` and target max abs error to about `1.14 m`, but
  the motion stage regressed range RMSE to about `83.17 m`, position RMSE to
  about `92.61 m`, and target max abs error to about `74.62 m`. Keep the
  promoted observation-fusion path; do not add this motion layer as a PASS
  correction.
- Registry slug `small_drone_multiburst_median_b1x3_sim3` is already covered
  by promoted candidate
  `small_drone_multiburst_observation_inner_median_outer_median_b1x3_trim0p2_common0_sim3`;
  the same report also contributes the promoted quality candidate
  `small_drone_multiburst_observation_inner_median_outer_median_b3x5_trim0p2_common0_affine_ridge`.
- `experiment_registry.csv` currently has five local PASS/STRICT_PASS rows
  whose `method_slug` is not a promoted manifest id. Two are alias or parent
  records already covered by promoted entries:
  `small_drone_multiburst_median_b1x3_sim3` and
  `multiburst_b7x3_log_disparity_ray_scale_residual`. The remaining local
  PASS rows are recorder/topic/runtime-environment gates
  (`operator_recorder_current_schema_lock`, `external_bbox_topic_runtime_input`,
  and `yolo_runtime_venv_system_site_stability`), not correction methods with
  final measurement PASS evidence.
- The registry also contains 35 PASS/STRICT_PASS rows whose referenced
  artifacts are not present in the current checkout. They remain historical
  leads only until their source artifacts are restored or rerun under the
  current coordinate-contract gate.
- Legacy `PARTIAL_PASS`/`FAMILY_LOCAL_PASS` bbox-anchor reports are kept as
  baselines or family-local evidence only. They are not global final PASS
  corrections under the strict all-axis gate.
