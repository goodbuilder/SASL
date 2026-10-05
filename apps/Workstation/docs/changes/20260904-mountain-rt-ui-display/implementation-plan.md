# Implementation Plan

## Approval status

Approved for the exact scope in `approval-summary.md`; implementation is allowed.

## Slices and dependencies

1. Backend projection: add optional `relative_pose_rdf` to the Mountain candidate mapping.
2. UI formatting: render matrix/vector/frame/unit details in the existing candidate detail
   control with fixed precision and safe omission for absent data.
3. Focused tests: verify the controller projection, UI formatting, selection stability, and
   legacy candidate fallback.
4. Regression: run the relevant optical-pose and Workstation UI suites, then the project
   test command available in the canonical environment.

## File and component map

- `optical_pose_controller.py`: candidate snapshot projection.
- `workspaces.py`: read-only detail formatting.
- `tests/test_optical_pose_controller_integration.py`: owner field assertions.
- `tests/test_optical_pose_workspace.py`: rendered detail assertions.
- `docs/changes/20260904-mountain-rt-ui-display/*`: durable change evidence.

## Migration and rollback steps

No migration. Roll back the additive mapping and formatter together; existing candidates
and profiles remain readable.

## Stopping conditions

Stop if the frame/sign contract cannot be represented unambiguously, if candidate Apply
semantics change, or if unrelated dirty-worktree edits make the focused patch unsafe.
