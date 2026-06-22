# EXP-20260622-342 B15x5 Common0.3 Non-Product Fallback Family Fresh100

## Purpose

Search non-product residual fallback families for the b15x5 common0.3 fused
observation stream after product fallback variants failed to recover the
fresh100 strict failure.

## Method

- Seeds: `298100..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.3`
- Feature spaces:
  `log_disparity`, `inverse_depth`, `raw_forward`, `log_raw_forward`
- Correction spaces: `ray_scale`, `forward_add`
- Degrees: `1`, `2`, `3`
- Ridge lambda: `100`
- Clips: `0.02`, `0.05`, `0.1`

This is a runtime-safe search: truth is used only for calibration/evaluation,
not as a runtime input.

## Result

Status: `FAIL`.

No searched non-product method passed the strict fresh100 gate. The best method
by script ordering was:

`multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b15x5_trim0p1_common0p3_residual_ray_scale_raw_forward_deg3_ridge100_clip0p02_pooled`

- Pass count: `500/600`
- Target max-abs: `4.732347 m` observation to `4.499028 m` residual
- Range RMSE mean: `0.933270 m` observation to `0.930948 m` residual
- Position RMSE mean: `2.496382 m` observation to `2.510136 m` residual

The top aggregate range methods were forward-add variants with `500/600`,
showing that broad non-product fallback families regress all intrinsic stress
rows under this configuration. Product fallback remains better for strict
coverage, but still has the isolated seed `298188` intrinsic failure from
EXP336/EXP341.

The full seed metrics CSV was `41 MB`, so it is not committed. This directory
keeps the method summary, best profile, summary JSON/TXT, and the 100 failing
rows for the best method.

## Coordinate Contract

- `raw_forward` and `log_raw_forward` features are derived from stereo-center
  OpenCV optical RDF forward after base profile correction and b15x5 common0.3
  observation fusion.
- `log_disparity` and `inverse_depth` features are derived from measured
  disparity/depth observables in the same fused runtime stream.
- `forward_add` outputs stereo-center RDF forward-axis corrections.
- `ray_scale` outputs stereo-center RDF ray-scale corrections.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
