# EXP-20260623-401 B15x7 Common0.2 DisparityOverForward Ridge3000 Oversample48 Local20 Scout

## Purpose

Test whether stronger ridge regularization stabilizes the b15x7
`disparity_over_forward` ray-scale branch from EXP391. EXP391 had the best
tracked target row among nearby ratio/raw-forward features but still failed all
intrinsic-scale rows.

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
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `3000`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.285147 m` observation to `4.532519 m` residual
- Range RMSE mean: `0.924142 m` observation to `0.984079 m` residual
- Position RMSE mean: `2.492225 m` observation to `2.527956 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.424058 m`. Ridge `3000` keeps the tracked target row close to EXP391 but
does not reduce the failure count and worsens worst target and aggregate RMSE.
Reject this stronger-ridge ratio ray-scale branch.

## Coordinate Contract

- `disparity_over_forward` divides fused measured disparity by corrected
  stereo-center OpenCV optical RDF `raw_forward_m` after base profile correction
  and b15x7 common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
