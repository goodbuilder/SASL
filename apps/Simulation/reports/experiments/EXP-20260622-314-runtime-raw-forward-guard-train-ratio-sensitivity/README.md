# EXP-20260622-314 Runtime Raw-Forward Guard Train-Ratio Sensitivity

## Purpose

Check whether the `raw_forward_mean_m > 1771.0 m` runtime guard is sensitive to
the calibration train/validation split. The guard is runtime-derived, but this
scout verifies that changing `train_ratio` in the sample builder does not move
the guard margin.

## Method

- Seeds: `294300..294319`
- Train ratios: `0.35` and `0.65`
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

Both train-ratio settings perfectly separated intrinsic from non-intrinsic
scenarios, and the `1771.0 m` threshold stayed inside the margin:

- Train ratio `0.35`: intrinsic `1773.102423..1774.140244 m`,
  non-intrinsic `1766.617613..1770.749837 m`
- Train ratio `0.65`: intrinsic `1773.102423..1774.140244 m`,
  non-intrinsic `1766.617613..1770.749837 m`

The identical margins are expected because the runtime feature window is built
from the fused measurement sequence, not from truth labels.

## Coordinate Contract

- `raw_forward_mean_m` is stereo-center OpenCV optical RDF forward averaged
  over the default runtime measurement window.
- This experiment changes only calibration split selection; it does not move
  values between RDF and FRD frames.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is not a runtime input.
