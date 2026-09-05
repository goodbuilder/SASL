# Frontend Contract

## Controls and inputs

No new controls or input fields. Existing `Solve GNSS + Mountain R,t`, candidate table,
and `Apply candidate` controls retain their labels and behavior.

## Payloads and commands

The existing Solve payload and `alignment.pose.solve_gnss_mountain` command are unchanged.
The UI consumes an additive `relative_pose_rdf` mapping in each Mountain candidate row.

## Visible states and transitions

When a Mountain candidate is present and selected, detail text shows:

```text
relative_frame: OpenCV optical RDF
equation: X_right = R_right_from_left_rdf * X_left + t_right_from_left_rdf_m
R_right_from_left_rdf:
  [ ... ]
  [ ... ]
  [ ... ]
t_right_from_left_rdf_m: [ ... ]
right_origin_in_left_rdf_m: [ ... ]
```

No candidate or a legacy candidate without the mapping keeps the existing detail output.

## Errors and recovery actions

Malformed/missing optional display data is omitted without creating a fake zero matrix or
vector. Existing candidate state, warnings, Apply eligibility, and recovery controls are
unchanged.

## Keyboard, focus, accessibility, and layout

The existing read-only detail editor remains keyboard-scrollable/selectable, keeps fixed
numeric formatting, and does not resize the candidate table or move adjacent controls.

## Cross-feature visible behavior and isolation

Only Alignment Candidate review changes. Calibration, Measurement, Pixhawk, GNSS, camera,
recording, and other candidate methods remain independent.
