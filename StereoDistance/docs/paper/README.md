# StereoDistance math paper draft

This directory contains a LaTeX draft for a formula-first paper based on
StereoDistance.

Main file:

- `stereo_distance_uam_math_paper.tex`

Suggested build command when a Korean LaTeX environment is available:

```bash
xelatex stereo_distance_uam_math_paper.tex
```

The draft intentionally leaves experiment tables, author information, and
figures as `TBD` placeholders. Fill those after ULog/RTK flight validation and
simulation result selection.

Coordinate contract:

- Camera optical: Right/Down/Forward
- PX4 body: Forward/Right/Down
- PX4 local/world truth: North/East/Down
- Measurement origin: stereo baseline center
