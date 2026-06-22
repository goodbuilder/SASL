# EXP-20260623-400 B15x7 Common0.2 DisparityOverForward ForwardAdd Oversample48 Local20 Scout

## Purpose

Test whether the `disparity_over_forward` ratio feature, which had the best
tracked target row among the b15x7 ratio/raw-forward ray-scale family, improves
when paired with additive forward correction.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.2`
- Target oversample count: `48`
- Feature space: `disparity_over_forward`
- Correction space: `forward_add`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.285147 m` observation to `4.305146 m` residual
- Range RMSE mean: `0.924142 m` observation to `0.925304 m` residual
- Position RMSE mean: `2.492225 m` observation to `2.493203 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.651248 m`. The additive ratio branch does not preserve the ray-scale ratio
target-row clue from EXP391 and is close to the product `forward_add` failure
pattern. Reject as a standalone additive fallback.

## Coordinate Contract

- `disparity_over_forward` divides fused measured disparity by corrected
  stereo-center OpenCV optical RDF `raw_forward_m` after base profile correction
  and b15x7 common0.2 fusion.
- `forward_add` adds a residual to stereo-center RDF forward before public
  topic export.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
