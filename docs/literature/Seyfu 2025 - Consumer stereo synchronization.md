---
type: literature
year: 2025
zotero_item: 4INHI5UA
bibtex_key: seyfu_stereo_2025
doi: 10.3390/s25175535
tags:
  - workstation
  - camera-synchronization
---

# A Stereo Synchronization Method for Consumer-Grade Video Cameras to Measure Multi-Target 3D Displacement Using Image Processing in Shake Table Experiments

**Mearge Kahsay Seyfu, Yuan-Sen Yang.** *Sensors*, 25(17), 5535 (2025). [DOI and publisher full text](https://doi.org/10.3390/s25175535)

**Read status:** Publisher abstract and synchronization/results sections checked; metadata verified against MDPI.

## Content checked

The method estimates video time lag by minimizing triangulation reprojection error and refines the offset below frame resolution with polynomial interpolation. It is tested with motion-capture references across laboratory and structural shake-table setups. The article reports up to 78.97% reprojection RMSE reduction and notes that moving targets and rolling-shutter timing can require point-specific offsets.

**Claim check:** The publisher abstract says “up to 78.79%,” while the Table 1 example and main-text discussion give 78.97%. Confirm the intended figure and table before using the maximum-reduction percentage.

## Relation to this project

This paper links clock timing to stereo reconstruction error. It corrects video alignment from image/reprojection evidence; the Workstation candidate characterizes sender clock-rate drift from RTCP/RTP records. It therefore motivates a follow-up that measures whether the observed clock drift changes exposure or triangulation error.

Related candidate: [[Workstation - Camera clock drift]].

