# EXP-20260622-369 B15x5 Common0.2 RawForward Oversample48 Local20 Scout

## Purpose

Test `raw_forward` as a feature axis for the b15x5 common0.2 oversample48
guard fallback. This follows the inverse-depth scout: the goal is to see whether
using stereo-center RDF forward directly reduces aggregate regression while
protecting the intrinsic-scale target rows.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.2`
- Target oversample count: `48`
- Feature space: `raw_forward`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.559166 m` observation to `4.416407 m` residual
- Range RMSE mean: `0.927983 m` observation to `0.925776 m` residual
- Position RMSE mean: `2.494131 m` observation to `2.505319 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.857175 m`, failing the strict gate. This feature keeps aggregate RMSE much
closer to observation than the product branch, but it does not protect the
intrinsic-scale target rows.

## Coordinate Contract

- `raw_forward` is stereo-center OpenCV optical RDF forward after base profile
  correction and b15x5 common0.2 observation fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
