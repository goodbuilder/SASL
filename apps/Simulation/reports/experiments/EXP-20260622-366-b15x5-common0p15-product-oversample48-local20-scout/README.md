# EXP-20260622-366 B15x5 Common0.15 Product Oversample48 Local20 Scout

## Purpose

Scout the midpoint between common0.1 and common0.2 for the b15x5 oversample48
product guard fallback.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.15`
- Target oversample count: `48`
- Feature space: `disparity_forward_product`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `STRICT_PASS` on the local20 scout.

The method passed `120/120`:

- Target max-abs: `4.516694 m` observation to `5.264932 m` residual
- Range RMSE mean: `0.924966 m` observation to `1.112537 m` residual
- Position RMSE mean: `2.492889 m` observation to `2.566891 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.578471 m`, worse than common0.2. Common0.15 is not prioritized for fresh100
expansion.

## Coordinate Contract

- `raw_forward_m` in the product feature is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 common0.15 observation fusion.
- Product feature uses `measured_disparity_px * raw_forward_m`.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
