# EXP-20260622-331 B15x5 Runtime Guard Threshold Sensitivity

## Purpose

Explore nearby runtime guard thresholds for the b15x5 selector. This is a new
guard-variant experiment after the b15x5 and b17x5 larger-shell sweep showed
that b15x5 fresh100 remained the stronger range-mean candidate.

## Method

- Base branch evidence: EXP-20260622-328 b15x5 fresh100
- Seeds: `296700..296799`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Thresholds:
  `1769.0`, `1770.0`, `1770.5`, `1771.0`, `1771.5`, `1772.0`,
  `1773.0`, `1774.0`
- Selector:
  use b15x5 product ray-scale fallback when
  `raw_forward_mean_m > threshold`; otherwise use b15x5 log-disparity
  forward-add.

## Result

Thresholds `1771.0 m` through `1773.0 m` preserve the same `600/600`
strict-pass result and aggregate metrics as EXP328:

- Range RMSE mean: `0.943552 m` observation to `0.826686 m` selected residual
- Position RMSE mean: `2.499548 m` observation to `2.395193 m` selected residual
- Guard branch count: `100`

Lower thresholds over-select the product fallback branch and degrade aggregate
metrics while still passing. Threshold `1774.0 m` under-selects the fallback
branch and drops to `501/600`.

Keep `1771.0 m` as the conservative recorded guard. The useful plateau extends
through `1773.0 m`, but the upper failure at `1774.0 m` argues against raising
the threshold without more evidence.

This is not a terminal search success. Continue exploring correction methods,
guard features, and packaging/tests until explicitly stopped.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 observation fusion.
- The guard feature is computed over the trailing 520-sample runtime
  measurement window; scenario labels and truth are not runtime selector
  inputs.
- Forward-add branch uses `log(measured_disparity_px)` and outputs
  stereo-center RDF forward-add correction.
- Product fallback branch uses `measured_disparity_px * raw_forward_m` and
  outputs stereo-center RDF ray-scale correction.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
