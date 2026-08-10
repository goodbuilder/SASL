# EXP-20260623-391 B15x7 Common0.2 DisparityOverForward Oversample48 Local20 Scout

## Purpose

Retest `disparity_over_forward` under b15x7 common0.2 fusion. EXP372 showed
this opposite ratio improved the tracked target row versus other ratio scouts
under b15x5 but regressed aggregate RMSE; this scout checks the same tradeoff
with b15x7 fusion.

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
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.285147 m` observation to `4.528458 m` residual
- Range RMSE mean: `0.924142 m` observation to `0.982989 m` residual
- Position RMSE mean: `2.492225 m` observation to `2.527373 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.428112 m`, better than the b15x7 raw-forward and forward-over-disparity
ratio rows but still failing the strict gate. Aggregate regression remains too
large for a standalone fallback.

## Coordinate Contract

- `disparity_over_forward` uses fused measured disparity divided by
  stereo-center OpenCV optical RDF `raw_forward_m` after base profile correction
  and b15x7 common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
