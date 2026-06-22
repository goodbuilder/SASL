# EXP-20260622-344 B15x5 Common0.4 Forward-Add on Common0.3 Seeds Fresh100

## Purpose

Test whether a common0.4 log-disparity forward-add branch can serve as a
targeted high-forward exception for the common0.3 product fallback failure on
the same fresh100 seed block.

## Method

- Seeds: `298100..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.4`
- Feature space: `log_disparity`
- Correction space: `forward_add`
- Degree: `3`
- Ridge lambda: `100`
- Clip: `0.1`

## Result

Status: `FAIL`.

The method passed `500/600` and failed all intrinsic stress rows:

- Target max-abs: `4.969521 m` observation to `4.869523 m` residual
- Range RMSE mean: `0.939139 m` observation to `0.921657 m` residual
- Position RMSE mean: `2.499503 m` observation to `2.494331 m` residual

For seed `298188`, `intrinsic_scale_stress`, the residual target max-abs
regressed from `3.948672 m` observation to `4.048669 m`. This branch is not a
viable exception for the isolated product fallback failure.

## Coordinate Contract

- `log_disparity` is derived from measured disparity in the b15x5 common0.4
  fused runtime stream.
- `forward_add` outputs a stereo-center OpenCV optical RDF forward-axis
  correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
