# Approval Summary

This document explains the approved small GUI change. Technical detail remains in the
other packet documents.

## What changes and what stays the same

The selected `Mountain + GNSS` candidate will show its numeric OpenCV relative rotation
matrix and translation vector in the existing Candidate review detail area. The solver,
GNSS baseline, candidate gates, Apply/Combine behavior, persistence, and all other
workspaces stay the same.

## Operator workflow before and after

The operator still freezes frames, marks paired Mountain points, and presses the existing
Solve button. Afterward, selecting the candidate now reveals the numeric R/t before the
operator chooses Apply.

## Success, delay, rejection, failure, overload, and recovery

Success is the labelled numeric readout in the selected candidate detail. No new delay,
busy state, rejection, timeout, cancellation, retry, or overload path is introduced.
Existing failures and warnings remain authoritative. Missing optional display data is
shown as the existing detail content, never as zero-valued fake R/t.

## Data and compatibility effects

The candidate snapshot receives an additive JSON-safe projection. Existing profiles,
settings, alignment events, and legacy candidates remain readable; no migration is needed.

## Performance priorities and allowed degradation

No solver or worker performance change. Formatting is a small GUI-thread operation on
candidate snapshot refresh and uses fixed-size text; existing controls remain responsive.

## Visual and interaction changes, including dynamic-layout stability

The existing compact read-only detail editor is reused. Numeric values use stable signed
9-decimal formatting and explicit frame/unit labels. Candidate selection, table columns,
focus, splitter geometry, and scroll behavior remain stable.

## How acceptance will be demonstrated

- Owner integration test confirms Mountain snapshot contains R/t with the expected frame
  and unit fields.
- UI test confirms selecting a Mountain row renders the matrix/vector and warnings.
- Existing candidate selection and optical-pose regression tests pass.

## Cost, risks, assumptions, rollback, and field limits

The main risk is confusing OpenCV `t` with the physical baseline; both vectors will be
labelled. Mountain's existing left-pose anchor and GNSS/lever-arm assumptions remain.
Rollback is a small additive source revert. Real-camera field accuracy is outside this
readout change and remains unverified by automated tests.

## Exact approved scope

Approved by the user's `그래` response on 2026-09-04 after the above outcome and technical
scope were presented: add owner-projected numeric Mountain relative R/t and display it in
the existing Candidate detail pane, with focused tests and no solver/workflow changes.
