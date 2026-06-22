# EXP-20260622-354 B15x5 Product Oversample48 Clip Local20 Scout

## Purpose

Scout whether reducing the product fallback ray-scale correction clip can reduce
aggregate regression while preserving the oversample48 high-forward intrinsic
recovery.

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
- Ridge lambda: `300`
- Clips: `0.005`, `0.01`, `0.015`, `0.02`

## Result

Status: `STRICT_PASS` on the local20 scout, with no clip sensitivity.

All clip values passed `120/120` and produced identical metrics:

- Target max-abs: `4.720330 m` observation to `5.453330 m` residual
- Range RMSE mean: `0.934776 m` observation to `1.116079 m` residual
- Position RMSE mean: `2.497594 m` observation to `2.568981 m` residual

For seed `298188`, `intrinsic_scale_stress`, all clips produced residual target
max-abs `2.538524 m`. The correction is not clip-limited in this range.

## Coordinate Contract

- `raw_forward_m` in the product feature is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 common0.3 observation fusion.
- Product feature uses `measured_disparity_px * raw_forward_m`.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
