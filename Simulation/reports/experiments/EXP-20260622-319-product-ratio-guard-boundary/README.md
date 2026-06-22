# EXP-20260622-319 Product-Ratio Guard Boundary

## Purpose

Evaluate an alternate runtime guard based on
`disparity_forward_product_ratio_to_nominal`. The current primary guard uses
`raw_forward_mean_m > 1771.0 m`; this boundary check asks whether a normalized
product-ratio guard would be equally robust across the recorded sensitivity
experiments.

## Method

Reuse recorded sensitivity CSVs from EXP312-EXP318 and compare intrinsic versus
non-intrinsic ranges for `disparity_forward_product_ratio_to_nominal`.

## Result

Status: boundary evidence.

The product-ratio guard separated intrinsic from non-intrinsic scenarios in
most fixed-window/fusion variants, with midpoint thresholds near `1.005916`.
However, when the EXP312 window-size sensitivity rows are considered together,
product ratio does not perfectly separate the classes:

- EXP312 intrinsic range: `1.002226553..1.006695205`
- EXP312 non-intrinsic range: `1.000287019..1.006719995`

This means `disparity_forward_product_ratio_to_nominal` is less robust than
`raw_forward_mean_m` unless the runtime window contract is held very tightly.
Keep `raw_forward_mean_m` as the primary guard for now.

## Coordinate Contract

- `disparity_forward_product_ratio_to_nominal` is derived from
  `measured_disparity_px * raw_forward_m`.
- `raw_forward_m` is stereo-center OpenCV optical RDF forward.
- This experiment only compares runtime-derived guard features; it does not
  change correction output frames.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is not a runtime input.
