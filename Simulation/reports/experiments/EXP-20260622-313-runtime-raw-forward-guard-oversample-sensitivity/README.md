# EXP-20260622-313 Runtime Raw-Forward Guard Oversample Sensitivity

## Purpose

Check whether the `raw_forward_mean_m > 1771.0 m` guard survives changes to the
target oversampling setting used when generating calibration/evaluation samples.

## Method

- Seeds: `294100..294119`
- Target oversample counts: `36` and `144`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion shell: b7x3 trimmed-mean / trimmed-mean, trim `0.05`,
  common fraction `0.5`
- Runtime feature window: default `520` samples
- Runtime-only guard feature: `raw_forward_mean_m`

## Result

Status: guard sensitivity support.

The guard feature still perfectly separated intrinsic from non-intrinsic
scenarios, and the `1771.0 m` threshold stayed inside the margin:

- Oversample `36`: intrinsic `1772.956297..1773.862491 m`,
  non-intrinsic `1766.462754..1770.451159 m`
- Oversample `144`: intrinsic `1773.031586..1773.989389 m`,
  non-intrinsic `1766.537732..1770.580740 m`

This supports the guard threshold under target-oversampling changes, but it is
still not a reason to stop exploring.

## Coordinate Contract

- `raw_forward_mean_m` is stereo-center OpenCV optical RDF forward averaged
  over the default runtime measurement window.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is not a runtime input.
