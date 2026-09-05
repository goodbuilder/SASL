# Change Brief

## Operator outcome

After pressing `Solve GNSS + Mountain R,t`, the operator can select the resulting
candidate and inspect the numeric OpenCV left-to-right relative rotation matrix and
translation vector in the existing Candidate review area.

## Motivation

The current workflow already solves and lists the Mountain candidate, but only shows
the R/t source labels and quality/provenance. The actual numeric result is not visible
without inspecting internal state.

## Included scope

- Add an additive, read-only relative-pose projection to the Mountain candidate snapshot.
- Show `R_right_from_left_rdf` and `t_right_from_left_rdf_m` with stable fixed precision.
- Show the physical `right_origin_in_left_rdf_m` alongside OpenCV `t` to prevent sign/frame ambiguity.
- Add focused owner/UI regression coverage.

## Excluded scope and non-goals

- No change to Mountain, GNSS, intrinsics, or pose-application mathematics.
- No change to Apply/Combine eligibility, persistence schema, commands, workers, or device I/O.
- No new workspace, dialog, styling system, or automatic application of a candidate.

## Acceptance summary

The existing Mountain Solve flow continues to produce the same candidate and active
pose. Selecting that candidate visibly shows the exact numeric relative R/t with the
OpenCV optical RDF frame and metre units; legacy candidates without the new optional
projection continue to display their existing detail text.
