# EXP-20260622-359 B15x5 Common0.2 Product Oversample48 Fresh100

## Purpose

Expand the EXP358 common0.2 oversample48 product fallback scout to fresh100.

## Method

- Seeds: `298100..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.2`
- Target oversample count: `48`
- Feature space: `disparity_forward_product`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `STRICT_PASS` on fresh100.

The method passed `600/600`:

- Target max-abs: `4.734651 m` observation to `5.450191 m` residual
- Range RMSE mean: `0.927707 m` observation to `1.112059 m` residual
- Position RMSE mean: `2.493208 m` observation to `2.565751 m` residual

This remains globally regressing, but it is slightly better than common0.3
oversample48 as a guard fallback candidate.

## Coordinate Contract

- `raw_forward_m` in the product feature is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 common0.2 observation fusion.
- Product feature uses `measured_disparity_px * raw_forward_m`.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
