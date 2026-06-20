# EXP-SEARCH-20260621-observation-fusion-intrinsic-grid-trim0p1

Purpose: test hierarchical multi-burst fusion under within-burst common-mode image/disparity error.

Coordinate contract: bursted left image u/v and disparity observations are fused in pixel space, re-triangulated into stereo-center RDF right/down/forward, then corrected by the runtime profile. PX4 local NED truth is used only to synthesize/audit samples.

Best method: `multiburst_observation_inner_median_outer_trimmed_mean_b7x3_trim0p1_common0p25`
Best status: `FAIL`
