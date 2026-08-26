# Integration Flow

## Action-to-postcondition traces

Settings Auto → driver+element probe → worker candidate `nvh264dec` → real first decoded frame →
`active_stream_decoders[side]` → Camera status `GPU · nvh264dec` + CAM tooltip + Performance detail.

Auto NVDEC failure → terminal reason → finalize current MP4/stop worker → one software restart →
actual `avdec_h264` frame → `CPU fallback` + reason.

## Transition tables

| Requested | Boundary result | Visible result |
|---|---|---|
| Auto | NVDEC starts/frames | GPU active |
| Auto | NVDEC fails; software starts | CPU fallback WARN |
| NVIDIA | NVDEC unavailable/fails | ERROR, no software restart |
| Software | software starts/frames | CPU selected |
| any | stopped | no active decoder |

## Failure and recovery flows

Driver/plugin unavailable before start, first-frame timeout, frame stall, fallback start failure,
stop during fallback, and app close remain independently visible and bounded.

## Cross-feature contracts and dependency direction

Camera worker → decoder evidence projection → Status/Camera/Performance. No reverse UI dependency.
