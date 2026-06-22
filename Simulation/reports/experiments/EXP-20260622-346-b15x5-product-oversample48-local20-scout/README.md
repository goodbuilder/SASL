# EXP-20260622-346 B15x5 Product Oversample48 Local20 Scout

## Purpose

Scout whether reducing calibration target oversampling from `72` to `48` changes
the b15x5 common0.3 product fallback fit enough to recover the known high-forward
intrinsic row.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.3`
- Target oversample count: `48`
- Feature space: `disparity_forward_product`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambdas: `10`, `30`, `100`, `300`
- Clip: `0.02`

## Result

Status: `STRICT_PASS` on the local20 scout.

All ridge variants passed `120/120`. The script-selected method was ridge
`300`:

- Target max-abs: `4.720330 m` observation to `5.453330 m` residual
- Range RMSE mean: `0.934776 m` observation to `1.116079 m` residual
- Position RMSE mean: `2.497594 m` observation to `2.568981 m` residual

For seed `298188`, `intrinsic_scale_stress`, the residual target max-abs was
`2.538524 m` for ridge `300`; all ridge variants passed the scout gate. This is
not promotable by itself because it is local20 only and aggregate metrics
regress, but it is worth a fresh100 expansion.

## Coordinate Contract

- `raw_forward_m` in the product feature is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 common0.3 observation fusion.
- Product feature uses `measured_disparity_px * raw_forward_m`.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
