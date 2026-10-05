# EXP-20260621-296 Universal Product Residual Tail4 Fresh100

## Purpose

Validate the EXP292 single product-feature residual on a fourth independent
fresh100 tail seed block.

## Method

- Base profile:
  `Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json`
- Method:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_ray_scale_disparity_forward_product_deg1_ridge10_clip0p1_pooled`
- Feature: `measured_disparity_px * raw_forward_m`
- Train ratio: `0.5`
- Seeds: `289900..289999`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Trials: `100` per scenario, `600` total

## Result

Status: `STRICT_PASS`

All six scenarios passed `100/100`; total `600/600`.

Worst residual target max-abs error was `6.329922 m`, below the 10 m simulation
gate.

## Coordinate Contract

- Feature input is `measured_disparity_px * raw_forward_m`.
- `raw_forward_m` is the forward axis of stereo-center OpenCV optical
  `RDF(right,down,forward)`.
- Runtime correction output frame is stereo-center RDF.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- `runtime_truth_used: false`; truth is calibration/evaluation-only.
