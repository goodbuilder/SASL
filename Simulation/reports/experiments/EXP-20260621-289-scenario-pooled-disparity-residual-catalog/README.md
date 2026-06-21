# EXP-20260621-289-scenario-pooled-disparity-residual-catalog

Purpose: test whether EXP288's disparity residual method can be exported as
runtime-loadable pooled profiles instead of refitting one residual per seed.

Method:
`multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_ray_scale_disparity_deg2_ridge0_clip0p25_pooled`.

Runtime contract: each profile applies a degree-2 ray-scale residual from fused
measured disparity and corrected stereo-center RDF `right/down/forward`. The
catalog can select a profile only from explicit deployment/scenario labels;
PX4/RTK truth, validation errors, target-window labels, true disparity, and
true image features are forbidden runtime inputs.

Result:

| Scope | Seeds | Status | Note |
| --- | ---: | --- | --- |
| single pooled 6-scenario profile | 287700..287719 | FAIL 100/120 | intrinsic-scale failed 0/20 because residual range stayed above raw |
| nominal pooled profile | 287800..287849 | STRICT_PASS 50/50 | label-specific profile |
| yaw_pitch_stress pooled profile | 287850..287899 | STRICT_PASS 50/50 | label-specific profile |
| bbox_outlier_stress pooled profile | 287900..287949 | STRICT_PASS 50/50 | label-specific profile |
| combined_sensor_stress pooled profile | 287950..287999 | STRICT_PASS 50/50 | label-specific profile |
| disparity_noise_stress pooled profile | 288000..288049 | STRICT_PASS 50/50 | label-specific profile |
| intrinsic_scale_stress pooled profile | 288050..288099 | STRICT_PASS 50/50 | label-specific profile |

Conclusion: a single pooled profile is not a final PASS method, but a
scenario-labeled pooled residual catalog is a viable Simulation PASS candidate
when the label is explicit and truth-free.
