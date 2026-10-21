---
type: literature
year: 2016
zotero_item: FQ93K62Z
bibtex_key: malapelle_data-fusion_2016
doi: 10.1016/j.image.2016.02.003
tags:
  - simulation
  - temporal-fusion
---

# A data-fusion approach to motion-stereo

**Francesco Malapelle, Andrea Fusiello, Beatrice Rossi, Pasqualina Fragneto.** *Signal Processing: Image Communication*, 43, 42–53 (2016). [DOI](https://doi.org/10.1016/j.image.2016.02.003) · [Author/institution full text](https://fusiello.github.io/papers/spic16.pdf)

**Read status:** Full paper checked from the author/institution PDF; metadata verified against the journal record.

## Content checked

The paper studies motion-stereo depth proxies from pairwise stereo matching and uses a Kalman-filter framework to integrate temporal and spatial observations. It is agnostic to the stereo matching algorithm and compares confidence measures on synthetic and natural images.

## Relation to this project

It provides prior work for temporal stereo fusion. The SASL candidate uses repeated observations from a fixed stereo pair and trimmed-mean fusion, followed by a train-split disparity-conditioned ray-scale correction; it is not a recursive Kalman depth-map estimator.

Related candidate: [[Simulation - Multi-burst stereo]].

