# EXP-20260621-287-observation-fusion-scenario-boundary-fresh50

Purpose: map the fresh 50-seed deployment boundary for
`multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5`
after the bbox-outlier PASS evidence in EXP-20260621-286.

Input assumptions: runtime uses fused left image `u/v` pixel centers and
measured stereo disparity only. PX4/RTK truth is used by the simulation harness
to generate and audit samples, not by the runtime correction method.

Output contract: observations are re-triangulated into stereo-center OpenCV
optical RDF `right/down/forward`, corrected by the runtime profile, and
reported through the public FRD+range contract.

Pass/fail criteria: every validation seed must reduce corrected range RMSE and
position RMSE relative to raw, include target-window samples, and keep
target-window max absolute error at or below 10 m.

## Result

The method is not an all-scenario final PASS candidate because every
intrinsic-scale run fails the range-improvement gate. It remains strong
supporting evidence for the nominal, yaw/pitch, bbox-outlier, combined-sensor,
and disparity-noise families.

| Scenario | Seeds | Status | Range RMSE raw -> corrected (m) | Position RMSE raw -> corrected (m) | Target max abs (m) |
| --- | ---: | --- | ---: | ---: | ---: |
| nominal | 287000..287049 | STRICT_PASS 50/50 | 2.439337 -> 0.229507 | 5.010339 -> 0.255297 | 1.209077 |
| yaw_pitch_stress | 287100..287149 | STRICT_PASS 50/50 | 2.432966 -> 0.233767 | 8.590006 -> 3.905737 | 1.244557 |
| combined_sensor_stress | 286600..286649 | STRICT_PASS 50/50 | 3.013284 -> 0.786310 | 10.994250 -> 6.301888 | 2.905239 |
| disparity_noise_stress | 286700..286749 | STRICT_PASS 50/50 | 4.276394 -> 2.038337 | 6.110752 -> 2.041464 | 5.521113 |
| intrinsic_scale_stress trim0p05 | 286800..286849 | FAIL 0/50 | 1.843194 -> 2.385705 | 4.720605 -> 2.392990 | 4.619757 |
| intrinsic_scale_stress grid trim0 | 286850..286899 | FAIL 0/50 | 1.850659 -> 2.391179 | 4.740324 -> 2.398432 | 4.577216 |
| intrinsic_scale_stress grid trim0p1 | 286900..286949 | FAIL 0/50 | 1.844505 -> 2.387012 | 4.724569 -> 2.394240 | 4.488771 |
| intrinsic_scale_stress grid trim0p2 | 286950..286999 | FAIL 0/50 | 1.846728 -> 2.388450 | 4.729572 -> 2.395692 | 4.478720 |

The intrinsic-scale grids tried `median` and `trimmed_mean` inner/outer modes,
common fractions `0.25,0.5,0.75,1`, and trim fractions `0,0.1,0.2`; all
combinations failed `0/50` because corrected range RMSE stayed above raw range
RMSE.
