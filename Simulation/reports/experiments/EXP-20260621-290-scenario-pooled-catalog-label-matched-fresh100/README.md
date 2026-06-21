# EXP-20260621-290 Scenario-Pooled Catalog Label-Matched Fresh100

## Purpose

Strengthen the EXP289 scenario-labeled pooled disparity residual catalog with a
larger fresh-seed transfer check. This run evaluates only the runtime profile
whose explicit catalog label matches the simulation scenario.

## Method

- Catalog:
  `Simulation/reports/experiments/EXP-20260621-289-scenario-pooled-disparity-residual-catalog/scenario_pooled_disparity_residual_catalog.json`
- Base profile:
  `Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Seeds: `288400..288499`
- Trials: `100` per scenario, `600` total
- Sample count: `1680`
- Target oversample count: `72`

The runtime profile selector contract is scenario/deployment-label based. It
must not consume truth, validation error, RTK, or calibration target fields at
runtime.

## Result

Status: `PASS`

All six label-matched scenario/profile pairs were `STRICT_PASS`.

| Scenario | Pass count | Worst residual target max abs error |
| --- | ---: | ---: |
| nominal | 100/100 | 1.258048 m |
| yaw_pitch_stress | 100/100 | 1.271220 m |
| bbox_outlier_stress | 100/100 | 4.749813 m |
| combined_sensor_stress | 100/100 | 2.804100 m |
| disparity_noise_stress | 100/100 | 2.374251 m |
| intrinsic_scale_stress | 100/100 | 1.257855 m |

The bbox-outlier target tail slightly worsened versus observation on the worst
sample (`4.676538 m` observation to `4.749813 m` residual), but each seed still
met the simulation PASS gate because residual range RMSE, position RMSE, target
window availability, and the 10 m target max-abs ceiling all passed.

## Coordinate Contract

- Image inputs: `left_u_px`, `left_v_px`, and `measured_disparity_px`.
- Residual profile application input/output frame:
  stereo-center OpenCV optical `RDF(right,down,forward)`.
- Public StereoDistance output contract:
  `FRD+range(forward,right,down,range)`.
- Offline metrics use truth only for validation scoring. Runtime profile
  application reports `runtime_truth_used: false`.
