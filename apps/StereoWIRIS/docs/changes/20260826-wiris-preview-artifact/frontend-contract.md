# Frontend Contract

## Controls and inputs

기존 `카메라 Preview 연결`, `YOLO 측정 시작/중지`, decoder 설정을 보존한다.

## Payloads and commands

승인 전 변경 없음. 향후 계측은 camera_id/frame_seq/PTS/monotonic/size/format/stride/hash를
backend evidence로 전달할 수 있으나 GUI command semantics는 바꾸지 않는다.

## Visible states and transitions

정상 preview, waiting, stale, decoder fallback/error 상태를 현재 위치에서 표시한다.
corrupted frame을 정상 preview로 보이는 것은 허용하지 않는다.

## Errors and recovery actions

오류는 side와 decoder/recovery 이유를 포함해야 하며 수동 Software fallback은 유지한다.

## Keyboard, focus, accessibility, and layout

frame update와 status update가 tab/focus/scroll/geometry를 바꾸지 않는다.

## Cross-feature visible behavior and isolation

Pixhawk/GNSS/recording이 실패하거나 비활성화되어도 camera preview 진단은 독립 동작해야 한다.
