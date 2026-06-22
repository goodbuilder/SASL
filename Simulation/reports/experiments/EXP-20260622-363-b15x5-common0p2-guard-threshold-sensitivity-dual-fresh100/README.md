# EXP-20260622-363 B15x5 Common0.2 Guard Threshold Sensitivity Dual Fresh100

## Purpose

Measure threshold sensitivity for the common0.2 oversample48 runtime guard
selector from EXP360/EXP361.

## Method

- Seed blocks: `298100..298199`, `299000..299099`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Thresholds: `1769.0`, `1770.0`, `1771.0`, `1772.0`, `1773.0`, `1774.0`
- Non-guard branch: b15x5 common0.3 log-disparity forward-add
- Guard branch: b15x5 common0.2 oversample48 ridge300 product ray-scale

## Result

Status: `STRICT_PASS` for thresholds `1769.0..1773.0` on both seed blocks, with
the best aggregate range retained at `1771.0..1773.0`.

- Block `298100`: thresholds `1771.0..1773.0` pass `600/600`, guard count
  `100`, range RMSE mean `0.932889 m` to `0.815233 m`.
- Block `299000`: thresholds `1771.0..1773.0` pass `600/600`, guard count
  `100`, range RMSE mean `0.932459 m` to `0.815042 m`.
- Threshold `1774.0` fails on both blocks because most intrinsic guard rows are
  not selected.

This supports the same `1771.0 m` threshold lower edge for the common0.2 guard
branch. Continue searching rather than stopping at repeated PASS.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 common0.3 observation fusion.
- Thresholding uses only runtime measurement features; scenario labels and
  truth are not runtime inputs.
- Non-guard branch outputs stereo-center RDF forward-add correction.
- Guard branch outputs stereo-center RDF ray-scale correction.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
