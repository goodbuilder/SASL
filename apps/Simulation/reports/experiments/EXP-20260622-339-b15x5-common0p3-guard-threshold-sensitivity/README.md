# EXP-20260622-339 B15x5 Common0.3 Guard Threshold Sensitivity

## Purpose

Check whether the b15x5 common0.3 fresh100 strict failure can be recovered by
adjusting the runtime raw-forward guard threshold.

## Method

- Base branch evidence: EXP-20260622-336 b15x5 common0.3 fresh100
- Seeds: `298100..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Thresholds:
  `1769.0`, `1770.0`, `1770.5`, `1771.0`, `1771.5`, `1772.0`,
  `1773.0`, `1774.0`
- Selector:
  use b15x5 common0.3 product ray-scale fallback when
  `raw_forward_mean_m > threshold`; otherwise use b15x5 common0.3
  log-disparity forward-add.

## Result

Status: `FAIL`.

No tested threshold recovers strict pass:

- `1769.0..1773.0 m`: `599/600`
- `1774.0 m`: `507/600`

The best aggregate rows remain around the original `1771.0..1773.0 m` plateau,
but the strict-fail row persists. Do not promote common0.3 by threshold change
alone. Prefer common0.4 when strict pass is required, or inspect the failing
common0.3 row before trying a more targeted fallback.

This is not a terminal search success. Continue exploring correction variants
until explicitly stopped.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 common0.3 observation fusion.
- The guard feature is computed over the trailing 520-sample runtime
  measurement window; scenario labels and truth are not runtime selector
  inputs.
- Forward-add branch uses `log(measured_disparity_px)` and outputs
  stereo-center RDF forward-add correction.
- Product fallback branch uses `measured_disparity_px * raw_forward_m` and
  outputs stereo-center RDF ray-scale correction.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
