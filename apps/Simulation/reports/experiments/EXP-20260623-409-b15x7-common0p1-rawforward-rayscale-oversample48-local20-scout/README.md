# EXP-20260623-409 B15x7 Common0.1 RawForward RayScale Oversample48 Local20 Scout

## Purpose

Switch EXP408's aggregate-oriented common0.1 raw-forward branch from additive
forward correction to ray-scale correction. This tests whether ray scaling keeps
the aggregate benefit while reducing intrinsic-scale target failures.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.1`
- Target oversample count: `48`
- Feature space: `raw_forward`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.348758 m` observation to `4.184628 m` residual
- Range RMSE mean: `0.916652 m` observation to `0.916164 m` residual
- Position RMSE mean: `2.488401 m` observation to `2.500509 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.836726 m`. Ray-scale lowers the worst target residual versus raw-forward
`forward_add`, but it still fails every intrinsic-scale row and worsens position
RMSE. Reject as a standalone method; keep the range-preserving behavior as a
clue for further raw-forward ray-scale tuning.

## Coordinate Contract

- `raw_forward` is the corrected stereo-center OpenCV optical RDF forward
  component after base profile correction and b15x7 common0.1 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
