# EXP-20260622-353 B15x5 Product Oversample64 Local20 Scout

## Purpose

Scout whether increasing product fallback target oversampling from `48` to `64`
can improve the high-forward intrinsic row or reduce aggregate regression.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.3`
- Target oversample count: `64`
- Feature space: `disparity_forward_product`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambdas: `100`, `300`, `1000`
- Clip: `0.02`

## Result

Status: `STRICT_PASS` on the local20 scout.

All ridge variants passed `120/120`. The script-selected method was ridge
`1000`:

- Target max-abs: `4.610361 m` observation to `5.318136 m` residual
- Range RMSE mean: `0.934600 m` observation to `1.115411 m` residual
- Position RMSE mean: `2.497304 m` observation to `2.568601 m` residual

For seed `298188`, `intrinsic_scale_stress`, ridge `100` has the lowest
residual target max-abs at `2.589285 m`. This is worse than the oversample48
scout, so oversample64 is not prioritized for fresh100 expansion yet.

## Coordinate Contract

- `raw_forward_m` in the product feature is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 common0.3 observation fusion.
- Product feature uses `measured_disparity_px * raw_forward_m`.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
