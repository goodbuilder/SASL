# Workflow Impact

## Affected operators and workflow IDs

- `WF-WIRIS-LIVE-PREVIEW`: 현장 운영자가 Left/Right OZ visible 화면을 동시 감시한다.

## Current workflow

운영자가 카메라 preview 또는 YOLO를 시작하면 side별 process가 RTSP를 수신·decode하고
latest QImage를 측정 화면에 표시한다. artifact가 발생해도 stall이 아니면 Auto가 이를
오류로 판정하지 못하며 운영자는 decoder 또는 encoder 설정을 수동 변경해 재연결한다.

## Proposed workflow

기존 버튼과 화면 배치는 유지한다. 내부 경계에서 frame identity/corruption/drop을
증거화하고, 확정된 최소 원인만 수정해 정상 preview를 지속한다. 필요한 경우 기존 status
영역에 decoder/복구 원인을 명확히 표시하되 새 workflow를 만들지 않는다.

live path에서 TCP RTP sequence gap을 감지하면 decoder 종류와 무관하게 preview/detection
frame 방출을 즉시 차단한다. 같은 pipeline의 parsed clean intra PTS부터 재개하며 missing packet,
recovery PTS, 차단 시간을 기록한다. 기존 Auto health fallback과 explicit NVIDIA fail-closed는
유지한다.

## Preserved, added, removed, and reordered steps

- 보존: 연결, YOLO, recording, stop/restart, Software 수동 fallback.
- 추가 후보: 진단 가능한 frame/drop/corruption evidence.
- 추가: RTP gap 즉시 차단과 same-session clean-intra recovery status.
- 추가: 기록용 worker 전환 직후 transient PLAYING 실패의 bounded retry status.
- 제거/재배치: 승인 전 없음.

## Decisions, confirmations, handoffs, waits, and artifacts

baseline readback, packet/AU evidence, decoder A/B, pre-Qt/QImage identity, screenshot, 저장
검증, 15분 검증 로그를 change packet evidence로 남긴다.

## Interruption, cancellation, retry, restart, and resume

카메라 disconnect와 worker stop은 기존 owner를 유지한다. 수정 검증 시 기존 app을 완전히
종료하고 새 PID로 실행한다. encoder A/B는 한 변수만 바꾸고 효과 없으면 원복한다.
기록 시작 재시도 중 operator stop은 대기를 취소하며, bounded 횟수가 끝나면 기존 error를
그대로 보고한다.

## Degraded, error, and recovery behavior

Software decode는 승인된 진단/health fallback이다. 처리율 저하가 있어도 RTP gap부터 clean
intra까지 output gate가 적용된다. Auto fallback은 decoder health 증거와 이유가 visible log에
남으며 silent degradation은 허용하지 않는다.

## Completion and next actions

15분 acceptance와 저장 full decode까지 통과한 뒤에만 완료한다. 물리 조작이나 금지된 장비
변경이 필요할 때만 사용자에게 요청한다.

## Training, operating-procedure, and compatibility impact

현재는 없음. 최종 원인이 operator setting 의미를 바꾸면 `docs/usage.md`를 갱신한다.

## Workflow-to-screen, command, state, and regression traceability

`측정 > 실시간` → CameraPane → YoloStreamWorker child → GStreamer → frame queue → QImage
→ QLabel. 각 화살표를 verification matrix에 연결한다.
