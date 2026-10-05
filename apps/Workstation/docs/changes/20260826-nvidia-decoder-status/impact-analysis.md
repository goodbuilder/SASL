# Impact Analysis

## Preserved invariants

- one stream worker and one decoder per side.
- RTSP packet/frame/recording timestamp and path ownership unchanged.
- Auto fallback stops/finalizes current output before opening a new decoder path.
- explicit Software remains a valid operator choice; explicit NVIDIA remains fail-closed.

## Affected layers and components

- settings persistence/projection: `settings_controller.py`, existing setting JSON.
- selection/probe: `sasl_common.operator_runtime`, `camera_stream_controller.py` (contract tests only
  unless discovery exposes a defect).
- runtime evidence: `standalone_gui.py`, `status_model.py`, `evidence.py`.
- UI: `workspaces.py`, existing Camera/Diagnostics components.
- tests/docs.

## Feature dependency, ownership, and coupling changes

No ownership transfer. Camera stream owner publishes a narrow immutable/read-only decoder projection;
Status/UI consume it. Detection and Recording do not inspect GStreamer internals.

## Worker CPU/GPU resource impact

- Baseline idle: RTX 4050, driver 595.84, 17 MiB, 0% GPU.
- Synthetic 1080p30 stress: NVDEC decode succeeds; decoder engine 78–100% when looped as fast as
  possible. This is a stress diagnostic, not expected realtime utilization.
- Proposed worker count/queues/rates unchanged. Auto shifts H.264 decode from CPU to NVDEC.
- CPU fallback remains available and visibly degraded; no simultaneous NVIDIA+software decoder.

## Compatibility and migration

Existing `software`, `auto`, `nvidia` values remain readable. Only current workspace setting changes
to Auto after approval. No data/schema migration.

## Rollback and recovery

Select Software to return to current behavior. If Auto fails, existing bounded fallback restarts
software. Strict NVIDIA can be selected manually when fail-closed behavior is desired.

## Risks and assumptions

- Driver can become unavailable after suspend/reset; Auto is safer than strict NVIDIA.
- Synthetic H.264 success does not prove both WIRIS RTSP streams until real preview test.
- GPU decode does not imply YOLO inference uses CUDA; those are separate statuses.
