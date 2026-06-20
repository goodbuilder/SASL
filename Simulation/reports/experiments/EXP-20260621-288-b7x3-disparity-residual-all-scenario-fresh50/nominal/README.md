# EXP-SEARCH-20260621-residual-nominal-fresh50

Purpose: fit train-split residual models from fused measured disparity/inverse-depth to remaining corrected RDF error.

Coordinate contract: observation/profile outputs and residual correction are stereo-center RDF right/down/forward. PX4 local NED truth is used only for calibration-time fitting and offline validation. Public StereoDistance topics remain FRD+range.

Runtime contract: fitted residual application consumes fused measured_disparity_px and corrected RDF only; it does not consume PX4/RTK truth, target-window labels, true image feature, or true disparity.

Best method: `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_ray_scale_disparity_deg2_ridge0_clip0p25`
Best status: `STRICT_PASS`
