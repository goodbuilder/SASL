# Promoted Methods

No method is currently promoted as hardware-validated complete all-axis
correction.

## Repo-Local Recheck

Run the current repo-local software deployment PASS gate with:

```bash
python3 Simulation/scripts/check_promoted_simulation_pass_candidates.py \
  --output-dir /tmp/sasl_promoted_pass_candidates_check
```

Expected result: `status=PASS`, `candidate_count=2`, `pass_count=2`,
`runtime_truth_used=false`, and `coordinate_contracts_explicit=true`.

## Current Simulation Software Deployment PASS

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
