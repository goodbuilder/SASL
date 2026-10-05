# EXP-20260622-376 B15x5 Common0.2 Product ForwardAdd Oversample48 Local20 Scout

## Purpose

Test `forward_add` correction with the b15x5 common0.2 product feature. The
ray-scale product branch protects target rows but regresses aggregate RMSE; this
scout checks whether adding a forward residual preserves aggregate behavior.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.2`
- Target oversample count: `48`
- Feature space: `disparity_forward_product`
- Correction space: `forward_add`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.559166 m` observation to `4.579166 m` residual
- Range RMSE mean: `0.927983 m` observation to `0.928348 m` residual
- Position RMSE mean: `2.494131 m` observation to `2.494239 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.670298 m`, failing the strict gate. The additive correction preserves
aggregate RMSE but does not apply enough target-row correction.

## Coordinate Contract

- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center OpenCV optical RDF `raw_forward_m` after base profile correction
  and b15x5 common0.2 fusion.
- `forward_add` adds a residual to stereo-center RDF forward and preserves the
  public `FRD+range` output contract after RDF-to-FRD ordering.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
