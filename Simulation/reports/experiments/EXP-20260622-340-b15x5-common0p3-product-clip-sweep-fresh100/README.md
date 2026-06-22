# EXP-20260622-340 B15x5 Common0.3 Product Clip Sweep Fresh100

## Purpose

Check whether the b15x5 common0.3 fresh100 strict failure can be recovered by
changing the product ray-scale fallback correction clip.

## Method

- Base evidence: EXP-20260622-336 b15x5 common0.3 fresh100
- Seeds: `298100..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Product fallback branch:
  `b15x5`, `trim_fraction=0.1`, `common_fraction=0.3`,
  `feature_space=disparity_forward_product`, `correction_space=ray_scale`,
  `degree=1`, `ridge_lambda=100`
- Correction clips:
  `0.005`, `0.01`, `0.015`, `0.02`, `0.025`, `0.03`

## Result

Status: `FAIL`.

All tested correction clips produced the same outcome: `599/600`. The same
intrinsic seed failed for every clip:

- Seed/scenario: `298188`, `intrinsic_scale_stress`
- Product residual range RMSE: `1.779278 m`
- Product residual position RMSE: `1.789188 m`
- Product residual target max-abs: `2.594554 m`

The product ray-scale fallback is not clip-limited in this region, so changing
`correction_clip` alone cannot recover the common0.3 strict failure. Prefer
common0.4 for strict pass, or try a different fallback family for the
intrinsic guard branch.

This is not a terminal search success. Continue exploring correction variants
until explicitly stopped.

## Coordinate Contract

- Product fallback branch uses `measured_disparity_px * raw_forward_m` and
  outputs stereo-center RDF ray-scale correction.
- `raw_forward_m` is stereo-center OpenCV optical RDF forward after base
  profile correction and b15x5 common0.3 observation fusion.
- Truth is calibration/evaluation-only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
