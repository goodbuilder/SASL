---
type: literature
year: 2023
zotero_item: ZS2APR7A
bibtex_key: chen_learning_2023
venue: CVPR
tags:
  - simulation
  - disparity-uncertainty
---

# Learning the Distribution of Errors in Stereo Matching for Joint Disparity and Uncertainty Estimation

**Liyan Chen, Weihan Wang, Philippos Mordohai.** CVPR 2023, 17235–17244. [Official CVF page and PDF](https://openaccess.thecvf.com/content/CVPR2023/html/Chen_Learning_the_Distribution_of_Errors_in_Stereo_Matching_for_Joint_CVPR_2023_paper.html)

**Read status:** Official CVF metadata, abstract, method summary, and conclusion checked. Zotero metadata verified against CVF proceedings.

## Content checked

SEDNet jointly estimates disparity and uncertainty. Its loss matches estimated uncertainty and disparity-error distributions using KL divergence and a differentiable soft histogram. The paper reports improvements on several stereo datasets.

## Relation to this project

This is relevant to the role of disparity error and uncertainty in downstream stereo fusion. The SASL residual candidate uses a small polynomial correction from measured disparity; it does not estimate a probabilistic per-pixel uncertainty and does not alter the stereo matcher.

Related candidate: [[Simulation - Multi-burst stereo]].

