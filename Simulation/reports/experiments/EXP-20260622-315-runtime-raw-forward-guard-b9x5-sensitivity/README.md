# EXP-20260622-315 Runtime Raw-Forward Guard B9x5 Sensitivity

## Purpose

Check whether the `raw_forward_mean_m > 1771.0 m` runtime guard remains valid
when the observation fusion shell changes from b7x3 to b9x5.

## Method

- Seeds: `294500..294519`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion shell: b9x5 trimmed-mean / trimmed-mean, trim `0.05`,
  common fraction `0.5`
- Runtime feature window: default `520` samples
- Runtime-only guard feature: `raw_forward_mean_m`

## Result

Status: guard sensitivity support.

The b9x5 shell perfectly separated intrinsic from non-intrinsic scenarios, and
the `1771.0 m` threshold stayed inside the margin:

- Intrinsic: `1773.166086..1774.002352 m`
- Non-intrinsic: `1766.694649..1770.602455 m`

This supports using the guard with the b9x5 fusion shell, but it remains an
ongoing validation result rather than a stopping point.

## Coordinate Contract

- `raw_forward_mean_m` is stereo-center OpenCV optical RDF forward averaged
  over the default runtime measurement window.
- This experiment changes only the fusion shell; it does not move values
  between RDF and FRD frames.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is not a runtime input.
