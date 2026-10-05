# EXP-20260622-345 B15x5 Product Common-Fraction Local20 Scout

## Purpose

Scout whether adjusting the b15x5 product fallback common fraction around the
known failing seed can recover the high-forward intrinsic failure before
spending another fresh100 run.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`
- Common fractions: `0.25`, `0.35`, `0.45`, `0.55`, `0.65`
- Feature space: `disparity_forward_product`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `100`
- Clip: `0.02`

## Result

Status: `FAIL`.

Every common fraction passed `119/120`; the failed row remained seed `298188`,
`intrinsic_scale_stress`.

Best by script ordering was common `0.35`:

- Target max-abs: `4.579745 m` observation to `5.318132 m` residual
- Range RMSE mean: `0.936036 m` observation to `1.115049 m` residual
- Position RMSE mean: `2.497757 m` observation to `2.567800 m` residual

For the failed seed, the lowest residual target max-abs in this scout was
common `0.25` at `2.588480 m`, still above the strict gate.

## Coordinate Contract

- `raw_forward_m` in the product feature is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 observation fusion.
- Product feature uses `measured_disparity_px * raw_forward_m`.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
