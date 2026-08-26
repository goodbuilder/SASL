# Workflow Impact

## Affected operators and workflow IDs

- Camera Preview/Measurement operator.
- `WF-CAMERA-PREVIEW`, `WF-MEASUREMENT-START`, decoder failure recovery.

## Current workflow

Settings의 Software mode → Preview → raw decoder name 확인. NVIDIA를 사용하려면 operator가
selector를 직접 바꾸고, fallback 시 log를 찾아야 한다.

## Proposed workflow

Auto(GPU preferred) 저장 → Preview → per-side `GPU · nvh264dec` 확인. GPU가 시작/운용 중
실패하면 active recording을 안전하게 종료하고 기존 절차로 `CPU fallback · avdec_h264`를
표시한다. 기술 상세는 Diagnostics Performance에서 확인한다.

## Preserved, added, removed, and reordered steps

- 보존: RTSP TCP, one worker/side, Auto fallback, explicit NVIDIA fail-closed, MP4 path ownership.
- 추가: readable actual decoder/fallback state와 driver/element evidence.
- 제거: 없음. 새 navigation도 없음.
- 재배치: decoder 확인을 log 탐색보다 Camera status에서 우선한다.

## Decisions, confirmations, handoffs, waits, and artifacts

Pending approval: Auto fallback을 허용할지 strict NVIDIA-only로 할지. Recommended: Auto.

## Interruption, cancellation, retry, restart, and resume

현재 Preview 중 mode 변경은 다음 stream start에 적용한다. Auto runtime fallback은 한 번만
software override를 설정하며 selector 변경 또는 app restart가 override를 clear한다.

## Degraded, error, and recovery behavior

- GPU active: factual OK.
- explicit Software: 정상 selected 상태, 경고 아님.
- Auto CPU fallback: WARN + reason.
- explicit NVIDIA unavailable/failure: ERROR, Preview start/recovery 거부.

## Completion and next actions

Per-side actual decoder와 fallback reason을 확인한 뒤 정상 Preview/Measurement를 계속한다.

## Training, operating-procedure, and compatibility impact

Operator는 GPU availability와 actual use를 구분한다. 기존 Settings와 fallback 조작법은 유지된다.

## Workflow-to-screen, command, state, and regression traceability

Settings decoder combo → `current_decoder_selection` → camera worker status →
`active_stream_decoders`/fallback evidence → Camera workspace + CAM tooltip + Performance diagnostics.
