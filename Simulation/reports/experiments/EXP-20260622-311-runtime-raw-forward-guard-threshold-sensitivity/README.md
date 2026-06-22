# EXP-20260622-311 Runtime Raw-Forward Guard Threshold Sensitivity

## Purpose

Check whether the runtime raw-forward guard selector is brittle around the
chosen `1771.0 m` threshold. This reuses the three recorded selector seed
blocks from EXP308, EXP309, and EXP310 and sweeps nearby thresholds.

## Method

- Blocks:
  - EXP308 same-block fresh100: seeds `293300..293399`
  - EXP309 independent fresh50: seeds `293500..293549`
  - EXP310 independent fresh100: seeds `293700..293799`
- Guard feature: `raw_forward_mean_m`
- Thresholds: `1770.6`, `1770.8`, `1771.0`, `1771.2`, `1771.5`, `1772.0`,
  `1772.5`, `1773.0`
- Selector branches:
  - non-guard: log-disparity forward-add
  - guard: product ray-scale fallback

## Result

Status: threshold robustness support.

Every threshold from `1770.8 m` through `1773.0 m` preserved the same strict
pass count and aggregate-improving result on all three blocks:

- EXP308: `600/600`
- EXP309: `300/300`
- EXP310: `600/600`

The slightly lower `1770.6 m` threshold also passed all three blocks, but it
changed aggregate deltas in the fresh100 blocks, so `1771.0 m` remains the
recorded guard threshold for now.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward averaged over the measurement window.
- Selector branches preserve the EXP308-EXP310 contracts:
  log-disparity forward-add or product ray-scale in stereo-center RDF.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is not a runtime input; this sensitivity pass reuses evaluation
  metrics only after branch selection.
