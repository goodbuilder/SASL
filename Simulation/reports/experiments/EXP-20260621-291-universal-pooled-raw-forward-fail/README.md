# EXP-20260621-291 Universal Pooled Raw-Forward Residual Fail

## Purpose

Test whether a single six-scenario pooled residual profile can replace the
EXP289/EXP290 scenario-labeled catalog by using runtime-available
`raw_forward_m` or `log_raw_forward` as the polynomial feature.

## Method

- Base profile:
  `Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Seeds: `288800..288819`
- Trials: `20` per scenario, `120` total
- Observation fusion: b7x3 trimmed mean, trim `0.05`, common fraction `0.5`
- Residual feature spaces: `raw_forward`, `log_raw_forward`
- Correction spaces: `ray_scale`, `forward_add`
- Degrees: `1,2,3`; ridge lambdas: `0,0.1,1`; clips: `0.25,0.5,1,2`

Runtime application uses only fused measured disparity and the corrected
stereo-center RDF point. Truth is used only for offline fit/evaluation splits.

## Result

Status: `FAIL`

Best candidate:

`multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_forward_add_raw_forward_deg3_ridge0_clip0p25_pooled`

It reached `100/120` PASS, but intrinsic-scale remained `0/20`.

| Scenario | Pass count | Worst residual target max abs error | Mean residual range RMSE |
| --- | ---: | ---: | ---: |
| nominal | 20/20 | 1.165761 m | 0.280088 m |
| yaw_pitch_stress | 20/20 | 1.166053 m | 0.280003 m |
| bbox_outlier_stress | 20/20 | 2.528102 m | 0.439669 m |
| combined_sensor_stress | 20/20 | 2.444356 m | 0.621442 m |
| disparity_noise_stress | 20/20 | 4.926203 m | 2.068507 m |
| intrinsic_scale_stress | 0/20 | 4.693706 m | 2.337131 m |

Conclusion: raw-forward features do not remove the intrinsic-scale failure for
an unconditional single six-scenario pooled residual. The current strongest
deployable simulation path remains explicit scenario/deployment-label selection
through the EXP289 catalog with EXP290 fresh100 transfer evidence.

## Coordinate Contract

- Image inputs: `left_u_px`, `left_v_px`, and `measured_disparity_px`.
- Runtime residual feature: `raw_forward_m`, the forward axis of stereo-center
  OpenCV optical `RDF(right,down,forward)`.
- Runtime correction output frame: stereo-center RDF.
- Public StereoDistance output contract remains
  `FRD+range(forward,right,down,range)`.
- `runtime_truth_used: false`; calibration/evaluation truth is offline only.
