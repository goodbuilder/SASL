# EXP-20260622-312 Runtime Raw-Forward Guard Window Sensitivity

## Purpose

Check whether the runtime raw-forward guard depends on the
`RuntimeFeatureWindow` length. EXP308-EXP311 used the default `520` sample
window. This scout recomputes `raw_forward_mean_m` for `120`, `260`, `520`,
and `1680` sample windows on a fresh50 seed block.

## Method

- Seeds: `293900..293949`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion shell: b7x3 trimmed-mean / trimmed-mean, trim `0.05`,
  common fraction `0.5`
- Runtime-only guard feature: `raw_forward_mean_m`
- Window sizes: `120`, `260`, `520`, `1680`

## Result

Status: guard sensitivity support.

Every tested window size perfectly separated `intrinsic_scale_stress` from the
other scenarios, but the absolute threshold changed with window size:

- `120`: intrinsic `1988.024421..1990.068682 m`,
  non-intrinsic `1980.312990..1986.302119 m`
- `260`: intrinsic `1913.090914..1914.817867 m`,
  non-intrinsic `1905.864467..1911.168612 m`
- `520`: intrinsic `1773.027622..1774.159168 m`,
  non-intrinsic `1766.553004..1770.749578 m`
- `1680`: intrinsic `1135.832572..1136.221595 m`,
  non-intrinsic `1132.114094..1134.301002 m`

The recorded `1771.0 m` threshold is therefore tied to the default
`RuntimeFeatureWindow(max_samples=520)` contract. If the runtime window length
changes, the guard threshold must be recalibrated.

## Coordinate Contract

- `raw_forward_mean_m` is stereo-center OpenCV optical RDF forward averaged
  over the selected runtime measurement window.
- This experiment changes only the guard window length; it does not move
  values between RDF and FRD frames.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is not a runtime input.
