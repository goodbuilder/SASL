# EXP-20260622-318 Runtime Raw-Forward Guard Common0p8 B9x5 Sensitivity

## Purpose

Check whether the `raw_forward_mean_m > 1771.0 m` runtime guard remains valid
when the b9x5 fusion shell uses common fraction `0.8`.

## Method

- Seeds: `295100..295119`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion shell: b9x5 trimmed-mean / trimmed-mean, trim `0.05`,
  common fraction `0.8`
- Runtime feature window: default `520` samples
- Runtime-only guard feature: `raw_forward_mean_m`

## Result

Status: guard sensitivity support.

The common0p8 b9x5 shell perfectly separated intrinsic from non-intrinsic
scenarios, and the `1771.0 m` threshold stayed inside the margin:

- Intrinsic: `1773.346141..1773.921068 m`
- Non-intrinsic: `1766.866449..1770.509624 m`

This supports the guard under the common-fraction variant, but validation
continues.

## Coordinate Contract

- `raw_forward_mean_m` is stereo-center OpenCV optical RDF forward averaged
  over the default runtime measurement window.
- This experiment changes only the fusion common fraction; it does not move
  values between RDF and FRD frames.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is not a runtime input.
