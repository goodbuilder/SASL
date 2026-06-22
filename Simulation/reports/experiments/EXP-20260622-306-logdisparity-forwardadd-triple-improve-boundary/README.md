# EXP-20260622-306 Log-Disparity Forward-Add Triple-Improve Boundary

## Purpose

Continue from EXP305 by changing the correction space from ray-scale only to
forward-add plus ray-scale. The objective is to find a runtime-safe method that
improves observation-relative target max-abs, range RMSE mean, and position
RMSE mean together, even if it does not satisfy the existing strict raw-baseline
gate.

## Method

- Base profile:
  `Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json`
- Fresh20 discovery:
  `EXP-SEARCH-20260622-aggregate-improve-logdisparity-forwardadd-fresh20`
- Fresh100 replay:
  `EXP-SEARCH-20260622-logdisparity-forwardadd-triple-improve-fresh100`
- Fusion shell: b7x3 trimmed-mean / trimmed-mean, trim `0.05`,
  common fraction `0.5`
- Residual feature: `log_disparity`
- Correction: `forward_add`, degree `3`, ridge `100`, clip `0.1`
- Fresh100 seeds: `293300..293399`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Trials: `100` per scenario, `600` total

## Result

Status: `FAIL` under the strict raw-baseline gate, but this is the first
fresh100 result in this search sequence that improves all three tracked
observation-relative aggregate metrics.

Best method:

`multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_forward_add_log_disparity_deg3_ridge100_clip0p1_pooled`

Fresh100 strict pass count was `500/600`. The failures were isolated to
`intrinsic_scale_stress`:

- `bbox_outlier_stress`: `100/100`
- `combined_sensor_stress`: `100/100`
- `disparity_noise_stress`: `100/100`
- `intrinsic_scale_stress`: `0/100`
- `nominal`: `100/100`
- `yaw_pitch_stress`: `100/100`

Observation-relative aggregates improved:

- Target max-abs: `5.322285 m` observation to `5.222288 m` residual
- Range RMSE mean: `1.015607 m` observation to `0.999478 m` residual
- Position RMSE mean: `2.550003 m` observation to `2.544715 m` residual

This should be treated as a new aggregate-improving boundary, not a promoted
strict-pass correction. The next useful direction is an intrinsic-scale guard,
selector, or feature that preserves this aggregate improvement while avoiding
the intrinsic scenario failures.

## Coordinate Contract

- Feature input is `log(measured_disparity_px)`.
- Runtime correction output remains stereo-center RDF forward-add.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is calibration/evaluation-only and is not a runtime input.
