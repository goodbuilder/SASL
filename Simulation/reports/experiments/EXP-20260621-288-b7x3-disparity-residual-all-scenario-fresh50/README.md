# EXP-20260621-288-b7x3-disparity-residual-all-scenario-fresh50

Purpose: validate a single multi-burst observation fusion plus disparity
residual method across six fresh 50-seed Simulation scenarios.

Method:
`multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_ray_scale_disparity_deg2_ridge0_clip0p25`.

Input assumptions: runtime first fuses 7 bursts x 3 image observations using
left image `u/v` pixel centers and measured stereo disparity. A calibration-time
train split fits a degree-2 polynomial ray-scale residual from fused measured
disparity. Runtime residual application consumes fused `measured_disparity_px`
and corrected stereo-center RDF only; PX4/RTK truth is not a runtime input.

Output contract: corrected points remain stereo-center OpenCV optical RDF
`right/down/forward`; public StereoDistance topics remain FRD+range.

Pass/fail criteria: every validation seed must reduce corrected range RMSE and
position RMSE relative to raw, include target-window samples, and keep
target-window max absolute error at or below 10 m.

## Result

All six fresh 50-seed scenario blocks are `STRICT_PASS`, for a total of 300/300
passing validation seeds.

| Scenario | Seeds | Status | Range RMSE raw -> observation -> residual (m) | Position RMSE observation -> residual (m) | Residual target max abs (m) |
| --- | ---: | --- | ---: | ---: | ---: |
| nominal | 287250..287299 | STRICT_PASS 50/50 | 2.414586 -> 0.228801 -> 0.220573 | 0.254248 -> 0.246856 | 1.041098 |
| yaw_pitch_stress | 287300..287349 | STRICT_PASS 50/50 | 2.416583 -> 0.232243 -> 0.223444 | 3.876737 -> 3.876264 | 1.335628 |
| bbox_outlier_stress | 287350..287399 | STRICT_PASS 50/50 | 2.669738 -> 0.402194 -> 0.395821 | 0.420052 -> 0.413959 | 3.382935 |
| combined_sensor_stress | 287400..287449 | STRICT_PASS 50/50 | 3.027143 -> 0.793378 -> 0.508544 | 6.326333 -> 6.296676 | 2.850971 |
| disparity_noise_stress | 287450..287499 | STRICT_PASS 50/50 | 4.289280 -> 2.037793 -> 0.426482 | 2.040902 -> 0.441160 | 2.144146 |
| intrinsic_scale_stress | 287200..287249 | STRICT_PASS 50/50 | 1.851247 -> 2.385011 -> 0.223355 | 2.392302 -> 0.291044 | 1.221048 |

This method fixes the intrinsic-scale boundary found in EXP-20260621-287, where
observation fusion alone failed the range-improvement gate.
