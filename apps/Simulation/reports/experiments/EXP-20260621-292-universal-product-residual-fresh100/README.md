# EXP-20260621-292 Universal Product Residual Fresh100

## Purpose

Validate a single six-scenario pooled runtime residual profile that does not
need scenario/deployment-label routing. The feature is the runtime-observable
product `measured_disparity_px * raw_forward_m`.

## Method

- Base profile:
  `Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Seeds: `289100..289199`
- Trials: `100` per scenario, `600` total
- Train ratio: `0.5`
- Observation fusion: b7x3 trimmed mean, trim `0.05`, common fraction `0.5`
- Residual method:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_ray_scale_disparity_forward_product_deg1_ridge10_clip0p1_pooled`

Runtime application uses only fused measured disparity and the corrected
stereo-center RDF point. Truth is used only for offline fit/evaluation splits.

## Result

Status: `STRICT_PASS`

The fixed product residual profile passed `600/600` fresh trials.

| Scenario | Pass count |
| --- | ---: |
| nominal | 100/100 |
| yaw_pitch_stress | 100/100 |
| bbox_outlier_stress | 100/100 |
| combined_sensor_stress | 100/100 |
| disparity_noise_stress | 100/100 |
| intrinsic_scale_stress | 100/100 |

Worst residual target max-abs error was `6.311829 m`, below the 10 m
simulation gate. This method optimizes the strict simulation PASS criteria
against raw triangulation; it can worsen mean RMSE versus the b7x3 observation
output in some non-intrinsic scenarios. Treat it as a simulation PASS candidate
that still needs runtime package and hardware/WIRIS validation before field use.

Runtime package smoke:

- `runtime_package_smoke/observation_fusion_deployment_package_summary.json`
  reports `acceptance_status: PASS`.
- StereoDistance node smoke reports
  `applied:EXP-20260621-292-universal-product-residual-fresh100_pooled_disparity_residual`.
- Runtime truth remains `false`.

Follow-up tail validation:

- `EXP-20260621-293-universal-product-residual-tail-fresh100` passed an
  independent fresh100 block (`289300..289399`) with `600/600` STRICT_PASS.

## Coordinate Contract

- Image inputs: `left_u_px`, `left_v_px`, and `measured_disparity_px`.
- Runtime residual feature: `measured_disparity_px * raw_forward_m`, where
  `raw_forward_m` is the forward axis of stereo-center OpenCV optical
  `RDF(right,down,forward)`.
- Runtime correction output frame: stereo-center RDF.
- Public StereoDistance output contract remains
  `FRD+range(forward,right,down,range)`.
- `runtime_truth_used: false`; calibration/evaluation truth is offline only.
