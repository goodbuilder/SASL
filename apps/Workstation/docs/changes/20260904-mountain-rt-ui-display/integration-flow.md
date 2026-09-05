# Integration Flow

## Action-to-postcondition traces

| Step | Boundary | Expected evidence |
|---|---|---|
| 1 | Operator clicks `Solve GNSS + Mountain R,t` | Existing command/payload is emitted |
| 2 | Controller validates/solves | Existing Mountain candidate is registered with same quality/warnings |
| 3 | Controller returns snapshot | Mountain row contains additive `relative_pose_rdf` |
| 4 | UI applies snapshot | Candidate row appears and selection is preserved |
| 5 | Operator selects row | Detail shows labelled numeric R, OpenCV t, and physical right origin |
| 6 | Operator clicks Apply | Existing candidate Apply path is unchanged |

## Transition tables

| Input state | Result | Display |
|---|---|---|
| Valid Mountain candidate with R/t | Candidate | Numeric readout plus existing evidence |
| Candidate warning/ineligible | WARNING/BLOCKED as before | Numeric readout remains labelled; warning/state remains authoritative |
| No candidate | NO DATA | Existing empty detail |
| Legacy/injected candidate without projection | Existing candidate state | Existing detail only; no fabricated values |

## Failure and recovery flows

Solver validation failure, GNSS failure, poor Mountain geometry, stale K,D, duplicate
commands, cancellation, and restart follow existing paths. The new formatter is read-only
and cannot convert a solver failure into a candidate.

## Cross-feature contracts and dependency direction

Controller/domain -> snapshot mapping -> presentation formatter. No widget-to-controller
private-state access and no dependency from Calibration/Measurement/Pixhawk into this UI.
