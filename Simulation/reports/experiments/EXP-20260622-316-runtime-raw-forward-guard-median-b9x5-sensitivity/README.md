# EXP-20260622-316 Runtime Raw-Forward Guard Median B9x5 Sensitivity

## Purpose

Check whether the `raw_forward_mean_m > 1771.0 m` runtime guard remains valid
when the b9x5 fusion shell uses median inner aggregation and trimmed-mean outer
aggregation.

## Method

- Seeds: `294700..294719`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion shell: b9x5 median / trimmed-mean, trim `0.05`,
  common fraction `0.5`
- Runtime feature window: default `520` samples
- Runtime-only guard feature: `raw_forward_mean_m`

## Result

Status: guard sensitivity support.

The median b9x5 shell perfectly separated intrinsic from non-intrinsic
scenarios, and the `1771.0 m` threshold stayed inside the margin:

- Intrinsic: `1773.128253..1773.839001 m`
- Non-intrinsic: `1766.650714..1770.427714 m`

This supports the guard under another aggregation variant, but validation
should continue instead of stopping at this PASS.

## Coordinate Contract

- `raw_forward_mean_m` is stereo-center OpenCV optical RDF forward averaged
  over the default runtime measurement window.
- This experiment changes only the fusion aggregation mode; it does not move
  values between RDF and FRD frames.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is not a runtime input.
