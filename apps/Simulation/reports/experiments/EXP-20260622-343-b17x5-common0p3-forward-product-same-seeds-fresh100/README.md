# EXP-20260622-343 B17x5 Common0.3 Forward/Product Same Seeds Fresh100

## Purpose

Check whether increasing the common0.3 fusion shell from b15x5 to b17x5
recovers the isolated intrinsic failure seen in the b15x5 common0.3 product
fallback selector.

## Method

- Seeds: `298100..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b17x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.3`
- Feature spaces: `log_disparity`, `disparity_forward_product`
- Correction spaces: `forward_add`, `ray_scale`
- Degrees: `1`, `3`
- Ridge lambda: `100`
- Clips: `0.02`, `0.1`

## Result

Status: `FAIL`.

The best method remained product ray-scale:

`multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b17x5_trim0p1_common0p3_residual_ray_scale_disparity_forward_product_deg1_ridge100_clip0p02_pooled`

- Pass count: `599/600`
- Target max-abs: `4.699601 m` observation to `5.443396 m` residual
- Range RMSE mean: `0.929549 m` observation to `1.112710 m` residual
- Position RMSE mean: `2.494406 m` observation to `2.566452 m` residual

The failed row remained seed `298188`, `intrinsic_scale_stress`, with residual
target max-abs `2.713982 m`. The b17x5 shell slightly improves the fused
observation aggregate but does not recover the product fallback strict failure.

## Coordinate Contract

- `raw_forward_m` in `disparity_forward_product` is stereo-center OpenCV
  optical RDF forward after base profile correction and b17x5 common0.3
  observation fusion.
- Product feature uses `measured_disparity_px * raw_forward_m`.
- `forward_add` outputs stereo-center RDF forward-axis corrections.
- `ray_scale` outputs stereo-center RDF ray-scale corrections.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
