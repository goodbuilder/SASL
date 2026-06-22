# EXP-20260622-317 Runtime Raw-Forward Guard B11x5 Sensitivity

## Purpose

Check whether the `raw_forward_mean_m > 1771.0 m` runtime guard remains valid
when the observation fusion shell changes to b11x5 with trim `0.1`.

## Method

- Seeds: `294900..294919`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion shell: b11x5 trimmed-mean / trimmed-mean, trim `0.1`,
  common fraction `0.5`
- Runtime feature window: default `520` samples
- Runtime-only guard feature: `raw_forward_mean_m`

## Result

Status: guard sensitivity support.

The b11x5 shell perfectly separated intrinsic from non-intrinsic scenarios,
and the `1771.0 m` threshold stayed inside the margin:

- Intrinsic: `1773.176833..1773.953590 m`
- Non-intrinsic: `1766.702780..1770.548437 m`

This supports the guard under a larger aggregation shell, but validation should
continue rather than treating this PASS as terminal.

## Coordinate Contract

- `raw_forward_mean_m` is stereo-center OpenCV optical RDF forward averaged
  over the default runtime measurement window.
- This experiment changes only the fusion shell; it does not move values
  between RDF and FRD frames.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is not a runtime input.
