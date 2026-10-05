# Implementation Plan

## Approval status

Approved by the user on 2026-08-29. Implementation is allowed for this packet only.

## Slices and dependencies

1. **Classification / PASS**: hand-built Annex-B positive/negative fixtures.
2. **Exact PTS handoff / PASS**: bounded cache, exact consume, missing/reordered boundaries.
3. **Exact native contract / PASS**: current camera/generation/key/freshness/native checks.
4. **Auto workflow / PASS**: candidate preview, metadata preservation, no-near lookup, existing
   downstream gates.
5. **Evidence/UI / PASS**: bounded status and capture JSON provenance.
6. **Regression/docs / PASS (software)**: focused suites, full pytest, application smoke,
   approved README and product docs.

## File and component map

- `workstation_core/yolo_stream.py`
- `workstation_core/camera_pane.py`
- `standalone_gui.py`
- `calibration_controller.py`
- focused tests under `tests/`
- confirmed `README.md`, affected `docs/ui/`, `WIRIS.md`, and this packet

## Migration and rollback steps

No persisted migration. Optional additive metadata is backward-readable. Rollback is scoped revert.

## Stopping conditions

Return to approval if work requires camera mutation, global keyframe-only decode, new worker/GPU
allocation, setting/schema migration, manual-path removal, pair-timing policy change, or unrelated
cleanup. Stop if concurrent dirty hunks cannot be preserved safely.
