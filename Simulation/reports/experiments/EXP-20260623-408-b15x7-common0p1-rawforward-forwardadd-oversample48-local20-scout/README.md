# EXP-20260623-408 B15x7 Common0.1 RawForward ForwardAdd Oversample48 Local20 Scout

## Purpose

Combine the lower common0.1 observation aggregate from EXP407 with the
aggregate-preserving raw-forward additive correction from EXP398. The test checks
whether this branch can further improve aggregate RMSE while exposing the same
intrinsic-scale target failure pattern.

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
- Correction space: `forward_add`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.348758 m` observation to `4.328759 m` residual
- Range RMSE mean: `0.916652 m` observation to `0.910094 m` residual
- Position RMSE mean: `2.488401 m` observation to `2.485815 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.699468 m`. This is the strongest aggregate-RMSE improvement in the recent
local20 scouts, but every intrinsic-scale target row still fails. Reject as a
standalone method and retain it as an aggregate anchor for later blended or
gated searches.

## Coordinate Contract

- `raw_forward` is the corrected stereo-center OpenCV optical RDF forward
  component after base profile correction and b15x7 common0.1 fusion.
- `forward_add` adds a residual to stereo-center RDF forward before public
  topic export.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
