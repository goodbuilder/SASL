# EXP-20260622-356 B13x5 Product Oversample48 Local20 Scout

## Purpose

Scout whether a smaller b13x5 common0.3 fusion shell improves the oversample48
product guard fallback compared with the b15x5 shell.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b13x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.3`
- Target oversample count: `48`
- Feature space: `disparity_forward_product`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `STRICT_PASS` on the local20 scout.

The method passed `120/120`:

- Target max-abs: `4.600267 m` observation to `5.347959 m` residual
- Range RMSE mean: `0.937850 m` observation to `1.115535 m` residual
- Position RMSE mean: `2.499509 m` observation to `2.569054 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.606585 m`, worse than the b15x5 oversample48 product fallback. The smaller
fusion shell is not prioritized for expansion.

## Coordinate Contract

- `raw_forward_m` in the product feature is stereo-center OpenCV optical RDF
  forward after base profile correction and b13x5 common0.3 observation fusion.
- Product feature uses `measured_disparity_px * raw_forward_m`.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
