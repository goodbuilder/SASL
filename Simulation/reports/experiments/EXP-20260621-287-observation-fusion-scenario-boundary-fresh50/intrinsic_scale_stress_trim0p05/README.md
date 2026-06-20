# EXP-SEARCH-20260621-observation-fusion-intrinsic-fresh50

Purpose: test hierarchical multi-burst fusion under within-burst common-mode image/disparity error.

Coordinate contract: bursted left image u/v and disparity observations are fused in pixel space, re-triangulated into stereo-center RDF right/down/forward, then corrected by the runtime profile. PX4 local NED truth is used only to synthesize/audit samples.

Best method: `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5`
Best status: `FAIL`
