# Implementation Plan

## Approval status

Approved at 2026-08-26T17:45:05+09:00 for Auto GPU-first, visible CPU fallback, existing Camera
status/CAM tooltip/Performance placement, and `./README.md` documentation only.

Software slices 1–5 are implemented and verified. Slice 6 real dual-WIRIS RTSP is NOT_RUN.

## Slices and dependencies

1. Pure decoder presentation mapping for requested/effective/fallback/unavailable.
2. Preserve/extend backend evidence with bounded fallback reason; no worker ownership change.
3. Project per-side readable decoder into Camera workspace and CAM status detail/tooltip.
4. Add driver/element/requested/effective evidence to Diagnostics Performance.
5. Persist Auto setting, restart Workstation, run doctor and synthetic NVDEC decode.
6. Run focused Camera/runtime/UI tests, full suite, layout sequence, and real WIRIS preview when
   devices are available.

## File and component map

Likely: `workstation_core/runtime_camera_status.py`, `standalone_gui.py`, `status_model.py`,
`evidence.py`, `workspaces.py`, `tests/test_workstation_camera.py`, UI docs, confirmed README.
Only change `camera_stream_controller.py` or `yolo_stream.py` if implementation reveals missing
evidence rather than presentation-only work.

## Migration and rollback steps

Settings value `software` → `auto`; rollback is selecting Software. No schema migration.

## Stopping conditions

- No source edits before approval.
- Stop for new worker/concurrency/rate/recording semantics or NVIDIA package/driver replacement.
- Do not claim real dual-WIRIS GPU decode until live preview proves both sides.
