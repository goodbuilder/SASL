# Impact Analysis

## Preserved invariants

- One stream worker/decoder/native ring/CameraPane owner per camera generation.
- Preview, YOLO, Measurement, recording, manual Live Pair, FTP/OZ, and settings keep current paths.
- Native calibration pixels remain lossless, native-sized, unannotated per-side OpenCV pixels.
- Freshness and left/right pair timing remain separate from exact frame identity.

## Affected layers and components

- `workstation_core/yolo_stream.py`: bounded positive NAL1/non-delta AU -> exact decoded PTS metadata.
- `workstation_core/camera_pane.py`: public exact calibration FrameKey resolver.
- `standalone_gui.py`: Auto-active checkerboard preview admission.
- `calibration_controller.py`: exact key validation/resolution and waiting states.
- Focused tests plus README/product/packet docs.

## Feature dependency, ownership, and coupling changes

No owner moves. The stream feature publishes additive immutable provenance through its existing frame
metadata contract; Calibration consumes it without accessing GStreamer or worker-private state.

## Worker CPU/GPU resource impact

No worker/process/GPU allocation change. Only non-delta parser AUs are mapped for NAL headers. The
classification cache is bounded to the existing timing-cache scale and entries are consumed by exact
PTS. The existing bounded child preview queue and GUI pending-preview slot preserve one positive
NAL1 candidate against replacement by an ordinary frame until it is consumed; no queue grows and
decoder/inference/recording admission is unchanged. The visible preview can therefore prefer that
candidate for at most one normal GUI preview interval.

## Compatibility and migration

Metadata is optional and JSON-safe; no file/settings migration. Auto fails closed without positive
evidence; manual capture remains. The timestamp-near API may remain for unaffected callers but is not
an Auto fallback.

## Rollback and recovery

Scoped code/docs revert; no stored-schema rollback. Runtime recovery is the next positive candidate
after eviction, restart, malformed input, or generation change.

## Risks and assumptions

- `h264parse` non-delta plus NAL1 is the approved operational category, not IDR proof.
- Independent camera cadence may reduce stereo yield.
- Exact PTS preservation needs fake-boundary tests and real hardware observation.
- Dirty concurrent work requires hunk-level preservation.
