---
type: literature
year: 2026
zotero_item: QJRBC6AD
bibtex_key: zhuang_revisiting_2026
doi: 10.3390/s26154798
tags:
  - simulation
  - closest-related-work
---

# Revisiting Stereo Triangulation in UAV Distance Estimation

**Jiafan Zhuang, Duan Yuan, Gaofei Han, Zhilang Weng, Rihong Yan, Weixin Huang, Wenji Li, Jie Xu, Zhun Fan.** *Sensors*, 26(15), 4798 (2026). [DOI](https://doi.org/10.3390/s26154798) · [Publisher full text](https://www.mdpi.com/1424-8220/26/15/4798)

**Read status:** Publisher abstract, method and results/ablation sections checked. Zotero metadata verified against the publisher record. The article is the closest comparator in this initial pass.

## Content checked

The paper builds a UAV distance dataset using UWB for distance ground truth and attributes long-range center-based triangulation errors to image-center localization/disparity errors amplified by stereo geometry. Its Position Correction Module predicts horizontal center offsets before triangulation; a Causal Feature Selection module is added to suppress domain-sensitive feature channels. The reported test Abs Rel changes from 0.494 baseline to 0.121 with PCM and 0.086 with PCM+CFS. The paper reports weaker performance when transferring from its 12 mm training lens to a 16 mm test subset.

## Relation to this project

[[Simulation - Multi-burst stereo]] studies a different correction stage and evidence type: 7×3 observation fusion followed by a disparity-dependent ray-scale residual in synthetic runs. Zhuang et al. use learned image-center correction before triangulation and real UAV/UWB data. The overlap is close enough that novelty must be stated against this work, and a real held-out sensor experiment would materially strengthen the Simulation candidate.

