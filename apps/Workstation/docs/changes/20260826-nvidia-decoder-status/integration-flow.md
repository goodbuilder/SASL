# Integration Flow

## Action-to-postcondition traces

Settings Auto → driver+element probe → worker candidate `nvh264dec` (`PLAYING` is not active) →
current-generation decoded frame →
`active_stream_decoders[side]` → Camera status `GPU · nvh264dec` + CAM tooltip + Performance detail.

Auto preview NVDEC-health failure → clear failed-side active evidence → per-side override/reason →
same-side software restart → current-generation `avdec_h264` frame → `CPU fallback` + reason.

Auto recording NVDEC-health failure → finalize/stop failed side → terminal child exit → per-side
override for next start. The healthy side and its MP4 remain running; the failed path is not reopened.

RTSP/source/parser or mux/filesink failure → boundary-specific error → no decoder fallback.

## Transition tables

| Requested | Boundary result | Visible result |
|---|---|---|
| Auto | NVDEC candidate + current-generation frame | GPU active |
| Auto | classified NVDEC failure; software current-generation frame | CPU fallback WARN on failed side |
| Auto | RTSP/source or mux/filesink failure | boundary ERROR, no decoder change |
| NVIDIA | NVDEC unavailable/fails | ERROR, no software restart |
| Software | software starts/frames | CPU selected |
| any | stopped | no active decoder |

## Failure and recovery flows

Driver/plugin unavailable before start, first-frame timeout, frame stall, fallback start failure,
stop during fallback, and app close remain independently visible and bounded.

## Cross-feature contracts and dependency direction

Camera worker → decoder evidence projection → Status/Camera/Performance. No reverse UI dependency.
