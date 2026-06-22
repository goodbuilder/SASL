# EXP-20260622-367 B15x5 Common0.18 Product Oversample48 Local20 Scout

## Purpose

Scout a narrow common-fraction point between common0.15 and common0.2 for the
b15x5 oversample48 product guard fallback. This is exploration only; a strict
pass is not a terminal success.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.18`
- Target oversample count: `48`
- Feature space: `disparity_forward_product`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `STRICT_PASS` on the local20 scout.

The method passed `120/120`:

- Target max-abs: `4.462603 m` observation to `5.207735 m` residual
- Range RMSE mean: `0.926526 m` observation to `1.111295 m` residual
- Position RMSE mean: `2.493673 m` observation to `2.566358 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.657231 m`, worse than common0.15 and common0.2. Common0.18 is not
prioritized for fresh100 expansion.

## Coordinate Contract

- `raw_forward_m` in the product feature is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 common0.18 observation fusion.
- Product feature uses `measured_disparity_px * raw_forward_m`.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
