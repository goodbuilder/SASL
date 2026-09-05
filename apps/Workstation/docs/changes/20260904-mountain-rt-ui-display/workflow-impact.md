# Workflow Impact

## Affected operators and workflow IDs

- Operator workflow: Alignment / optical-pose candidate review.
- Command: existing `alignment.pose.solve_gnss_mountain`.
- No new permission, device, or timing workflow.

## Current workflow

The operator solves Mountain + GNSS, sees a candidate row and evidence summary, then
must infer or inspect the numeric R/t elsewhere before deciding whether to apply it.

## Proposed workflow

The operator solves the same way, selects the candidate, and reads the numeric relative
R/t directly in the existing read-only detail pane before pressing `Apply candidate`.

## Preserved, added, removed, and reordered steps

- Preserved: capture, annotation, GNSS prerequisite, Solve, candidate selection, Apply,
  Revert, Reset, and all failure gates.
- Added: one read-only inspection step for numeric R/t.
- Removed/reordered: none.

## Decisions, confirmations, handoffs, waits, and artifacts

The Solve result remains an unapplied candidate. Numeric display is not confirmation and
does not mutate active alignment. Apply remains the explicit handoff to active state.
The additive snapshot/event projection preserves existing provenance and audit artifacts.

## Interruption, cancellation, retry, restart, and resume

No behavior changes. Existing Solve worker/command lifecycle and candidate snapshot
refresh remain authoritative. A missing optional R/t projection falls back to existing
detail rendering.

## Degraded, error, and recovery behavior

No new degraded state. Existing BLOCKED/WARNING/ineligible states remain visible. Invalid
or incomplete numeric projection is omitted rather than fabricated; the candidate's
existing warning/state remains the recovery evidence.

## Completion and next actions

Success is visible when the selected candidate detail contains the labelled OpenCV R/t
and frame/unit contract. The operator may then Apply, choose another candidate, or leave
the candidate unapplied.

## Training, operating-procedure, and compatibility impact

Training impact is minimal: select the Mountain candidate to inspect the values. Existing
profiles, settings, session artifacts, and commands remain compatible.

## Workflow-to-screen, command, state, and regression traceability

`Solve GNSS + Mountain R,t` -> controller snapshot -> `OpticalPosePane.apply_snapshot()`
-> candidate table/detail -> explicit `Apply candidate`; focused tests cover owner field
projection and detail rendering.
