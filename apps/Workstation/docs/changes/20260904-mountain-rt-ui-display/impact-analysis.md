# Impact Analysis

## Preserved invariants

- OpenCV contract remains `X_right = R_right_from_left_rdf * X_left + t`.
- Optical frame remains RDF `[Right, Down, Forward]`; translation remains metres.
- Mountain R remains correspondence-derived; metric t remains GNSS-baseline-derived.
- Candidate remains unapplied until explicit Apply; owner remains the source of truth.
- Existing selection, observability, warning, persistence, and reset behavior is unchanged.

## Affected layers and components

- Domain snapshot projection: `optical_pose_controller.py`.
- Presentation formatting: `workspaces.py` / `OpticalPosePane`.
- Focused tests: `tests/test_optical_pose_controller_integration.py` and
  `tests/test_optical_pose_workspace.py`.

## Feature dependency, ownership, and coupling changes

No new dependency or owner is introduced. The controller supplies immutable plain-value
data; the UI only formats it.

## Worker CPU/GPU resource impact

None. No solver, worker, queue, timer, or device path changes.

## Compatibility and migration

The new snapshot key is additive. Older/injected candidates without it continue to use
the existing detail text. No file migration or settings migration is required.

## Rollback and recovery

Rollback is a two-file source revert of the additive mapping/formatting plus its tests.
Runtime recovery is omission of the optional readout; Solve/Apply behavior remains intact.

## Risks and assumptions

- The displayed `t` is OpenCV transform translation, not the physical right-origin
  baseline; both are labelled to avoid a sign error.
- The absolute left pose remains the Mountain anchor, as documented by the existing solver.
- Visual hardware validation is not performed by automated tests.
