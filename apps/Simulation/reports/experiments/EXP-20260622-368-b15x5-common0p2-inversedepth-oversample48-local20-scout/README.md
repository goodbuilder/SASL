# EXP-20260622-368 B15x5 Common0.2 InverseDepth Oversample48 Local20 Scout

## Purpose

Test a new feature axis for the b15x5 common0.2 oversample48 guard fallback.
The prior product feature improved the target row but regressed aggregate RMSE;
this scout checks whether `inverse_depth` can reduce aggregate regression while
remaining within strict gates.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.2`
- Target oversample count: `48`
- Feature space: `inverse_depth`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.559166 m` observation to `4.689910 m` residual
- Range RMSE mean: `0.927983 m` observation to `0.959216 m` residual
- Position RMSE mean: `2.494131 m` observation to `2.515773 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.559493 m`, also failing the strict gate. The aggregate regression is smaller
than the product branch, but the intrinsic-scale target failures make this
feature unsuitable as the current guard fallback.

## Coordinate Contract

- `inverse_depth` is derived from the fused stereo measurement, not from
  calibration/evaluation truth.
- `ray_scale` outputs a stereo-center OpenCV optical RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
