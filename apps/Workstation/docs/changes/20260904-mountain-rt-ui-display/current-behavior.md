# Current Behavior

## Version and worktree context

- Repository root: `/home/moontaehyung/바탕화면/SASL-20260821`.
- Workstation source root: `apps/Workstation`.
- Baseline ref: `3288aee` with extensive pre-existing user-owned dirty changes.
- Those changes are preserved; this packet owns only the Mountain R/t read-only projection.

## Reproduction

1. Open Alignment and freeze a left/right pair.
2. Mark equal-order Mountain ridge points on both images and press `Solve GNSS + Mountain R,t`.
3. Select the generated candidate in `R,t candidates · compare`.

## Observed frontend behavior

The candidate table shows method, R source (`paired Mountain u,v`), t source
(`GNSS baseline`), residual, observability, and state. The detail text shows provenance,
quality metrics, and warnings, but not the numeric 3x3 relative rotation or 3-vector
translation.

## Observed backend and persistence behavior

`solve_workstation_optical_pose_gnss_mountain()` solves relative rotation from Mountain
correspondences conditioned on the GNSS baseline direction, obtains metric translation
from the GNSS baseline, derives the right absolute NED pose from the frozen left anchor,
registers an `OpticalPoseCandidate`, and returns `workstation_optical_pose_snapshot()`.
The snapshot candidate already contains `left_pose` and `right_pose` plus source/quality
evidence. The canonical OpenCV relative R/t is derivable from those poses but is not
currently projected as a named numeric field.

## Documentation, code, test, and runtime conflicts

No command or state conflict was found. The product docs already define the Candidate
review and OpenCV optical RDF contracts; this change makes the existing numeric result
visible without changing those contracts.
