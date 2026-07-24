# StereoWIRIS math paper draft

> Archived draft: this document preserves an earlier stereo-center/reprojection-correction and target-telemetry comparison proposal. The product was renamed from StereoDistance to StereoWIRIS; this draft does not describe the current StereoWIRIS implementation, whose Raw/public output is `left_camera_rdf` and which has no measurement-correction or target ULog/telemetry comparison layer. Do not use this draft as the current coordinate or runtime contract; use `../COORDINATE_CONTRACT.md`.

This directory contains a LaTeX draft for a formula-first paper based on
StereoWIRIS.

Main file:

- `stereo_distance_uam_math_paper.tex`

Suggested build command when a Korean LaTeX environment is available:

```bash
xelatex stereo_distance_uam_math_paper.tex
```

The draft intentionally leaves experiment tables, author information, and
figures as `TBD` placeholders. Fill those after ULog/RTK flight validation and
simulation result selection.

Historical draft coordinate contract (not current runtime):

- Camera optical: Right/Down/Forward
- PX4 body: Forward/Right/Down
- PX4 local/world truth: North/East/Down
- Measurement origin: stereo baseline center
