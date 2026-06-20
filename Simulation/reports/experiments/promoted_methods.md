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

Expected result: `status=PASS`, `candidate_count=23`, `pass_count=23`,
`runtime_truth_used=false`, and `coordinate_contracts_explicit=true`.

To also re-run the heavier six-scenario non-alias catalog smoke:

```bash
python3 Simulation/scripts/check_promoted_simulation_pass_candidates.py \
  --output-dir /tmp/sasl_promoted_pass_candidates_check_with_non_alias \
  --include-non-alias-smoke
```

Expected result: `status=PASS`, `candidate_count=24`, `pass_count=24`; the
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
  - Promotion status: simulation/software deployment candidate only; not
    hardware/WIRIS validated.

## Current Simulation Catalog PASS Candidate

- `non_alias_scenario_profile_catalog_label_selector`
  - Status: `PASS`
  - Experiment: `EXP-20260618-281-non-alias-robustness-catalog-selector`
  - Meaning: current non-alias robustness route when deployment/scenario labels
    are explicit. A single universal profile is rejected; the valid strategy is
    a scenario profile catalog selected by operator/deployment metadata.
  - Evidence: current-checkout six-scenario smoke at
    `/tmp/sasl_non_alias_default6_smoke` passed final robustness acceptance,
    direct NED-to-RDF alias forbidden 6/6, scenario catalog profiles 6/6,
    StereoDistance promotion 6/6, selector package PASS, deployment frontier
    PASS, operator launch 19/19 PASS, runtime truth `false`.
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
