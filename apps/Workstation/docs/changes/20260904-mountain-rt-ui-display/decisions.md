# Decisions

## 2026-09-04 — Display the Mountain candidate's numeric relative R/t

- Decision: add a read-only numeric R/t projection to the existing Candidate detail pane.
- Rationale: the solver already returns/registers the candidate; operators need direct
  evidence before Apply.
- Coordinate decision: display OpenCV `R_right_from_left_rdf` and
  `t_right_from_left_rdf_m`, plus `right_origin_in_left_rdf_m` because OpenCV `t` is not
  the physical baseline vector.
- Preserved: Mountain calculation, GNSS translation source, candidate gates, Apply,
  persistence, and all unrelated workflows.
- Approval: user response `그래` after the proposed implementation scope was presented.
