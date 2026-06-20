# EXP-20260621-286-bbox-outlier-b7x3-observation-fresh50

Purpose: test hierarchical multi-burst fusion under within-burst common-mode image/disparity error.

Coordinate contract: bursted left image u/v and disparity observations are fused in pixel space, re-triangulated into stereo-center RDF right/down/forward, then corrected by the runtime profile. PX4 local NED truth is used only to synthesize/audit samples.

Best method: `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5`
Best status: `STRICT_PASS`

Result summary:

- Trials: `50/50 STRICT_PASS`, seeds `286500..286549`.
- Validation samples: `21000` total (`420` validation samples per seed).
- Runtime inputs: left image `u/v`, measured stereo disparity, fixed camera intrinsics/baseline, and fixed runtime profile parameters.
- Runtime truth: `false`; PX4/RTK/NED truth is calibration/evaluation only.
- Frame contract: input/fused internal points are stereo-center OpenCV optical `RDF(right,down,forward)`; public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Range RMSE: raw mean `2.614845 m` -> corrected mean `0.407532 m`, corrected max `0.493975 m`.
- Position RMSE: raw mean `5.084014 m` -> corrected mean `0.425134 m`, corrected max `0.508755 m`.
- Target-window max absolute error: `3.087384 m`, under the `10 m` gate.

Promotion note: this is fresh bbox-outlier scenario evidence for the b7x3
observation-fusion family with `trim_fraction=0.05`. It supports continued
search and deployment confidence, but it is not a replacement for the existing
all-scenario deployment tier evidence using `trim_fraction=0.2`.
