# EXP-20260623-415 B15x7 Common0.1 ForwardOverDisparity RayScale Oversample48 Local20 Scout

## Purpose

Test the forward-over-disparity ratio feature under the common0.1 fusion setting.
Common0.1 improved the observation aggregate in product/raw-forward branches, so
this scout checks whether the opposite ratio feature can lower target failures
without losing too much aggregate behavior.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.1`
- Target oversample count: `48`
- Feature space: `forward_over_disparity`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.348758 m` observation to `4.108256 m` residual
- Range RMSE mean: `0.916652 m` observation to `0.919012 m` residual
- Position RMSE mean: `2.488401 m` observation to `2.505376 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.917725 m`. The ratio lowers worst target residual versus the observation, but
it fails every intrinsic-scale row and worsens aggregate RMSE. Reject as a
standalone method.

## Coordinate Contract

- `forward_over_disparity` divides corrected stereo-center OpenCV optical RDF
  `raw_forward_m` by fused measured disparity after base profile correction and
  b15x7 common0.1 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
