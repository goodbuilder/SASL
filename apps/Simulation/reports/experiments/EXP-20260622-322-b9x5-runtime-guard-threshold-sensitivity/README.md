# EXP-20260622-322 B9x5 Runtime Guard Threshold Sensitivity

## Purpose

Check the threshold sensitivity of the b9x5 runtime-guard selector from
EXP320 and EXP321.

## Method

- Blocks:
  - EXP320 b9x5 fresh50: seeds `295300..295349`
  - EXP321 b9x5 fresh100: seeds `295500..295599`
- Guard feature: `raw_forward_mean_m`
- Thresholds: `1770.6`, `1770.8`, `1771.0`, `1771.2`, `1771.5`, `1772.0`,
  `1772.5`, `1773.0`
- Selector branches:
  - non-guard: b9x5 log-disparity forward-add
  - guard: b9x5 product ray-scale fallback

## Result

Status: threshold robustness support.

The recorded `1771.0 m` threshold preserved strict pass and aggregate
improvement on both b9x5 blocks:

- EXP320 fresh50: `300/300`
- EXP321 fresh100: `600/600`

The fresh100 block remained `600/600` across all tested thresholds. The
fresh50 block dropped to `299/300` at `1773.0 m`, so the upper guard margin is
not unlimited. Keep `1771.0 m` as the recorded threshold.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward averaged over the default runtime measurement window.
- Selector branches preserve the EXP320-EXP321 contracts:
  b9x5 log-disparity forward-add or b9x5 product ray-scale in stereo-center RDF.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is not a runtime input.
