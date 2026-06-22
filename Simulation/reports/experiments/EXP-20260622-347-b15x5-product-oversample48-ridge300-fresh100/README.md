# EXP-20260622-347 B15x5 Product Oversample48 Ridge300 Fresh100

## Purpose

Expand the EXP346 local20 scout to the full fresh100 seed block to verify
whether b15x5 common0.3 product ray-scale with target oversampling reduced to
`48` and ridge `300` recovers the previously failing high-forward intrinsic row.

## Method

- Seeds: `298100..298199`
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
- Clip: `0.02`

## Result

Status: `STRICT_PASS` on fresh100.

The method passed `600/600`, including seed `298188`,
`intrinsic_scale_stress`:

- Seed `298188` target max-abs: `3.745470 m` observation to `2.540842 m`
  residual
- Overall target max-abs: `4.720330 m` observation to `5.451747 m` residual
- Range RMSE mean: `0.933520 m` observation to `1.114182 m` residual
- Position RMSE mean: `2.496093 m` observation to `2.567073 m` residual

This candidate is useful as a strict-pass guard fallback, but it should not be
used globally because aggregate residual metrics regress. Continue search with a
runtime selector that applies this fallback only to high-forward guard rows.

## Coordinate Contract

- `raw_forward_m` in the product feature is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 common0.3 observation fusion.
- Product feature uses `measured_disparity_px * raw_forward_m`.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
