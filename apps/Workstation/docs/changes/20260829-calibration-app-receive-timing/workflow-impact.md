# Workflow Impact

## Affected operators and workflow IDs

- Operator: checkerboard Calibration을 수행하는 필드 운영자/엔지니어
- Workflow: `Calibration > Intrinsics/Extrinsics > Capture > Capture Live Pair/Auto capture`

## Current workflow

Preview 시작 → checkerboard를 촬영 → Live/Auto capture → RTCP/NTP Δt gate →
stereo eligible 또는 K,D-only 저장 → Photo set 선택 → K,D/R,t 계산.

## Proposed workflow

운영자 절차와 버튼은 같다. 단지 Live/Auto capture의 gate가 decoded-frame
app-receive monotonic Δt를 사용하고, UI/capture metadata에 그 기준을 표시한다.
RTCP/NTP는 저장/진단 증거와 사진 wall-clock 표시로 남는다.

## Preserved, added, removed, and reordered steps

- Preserved: Preview, 수동/Auto/FTP capture, checkerboard 검춝, K,D-only 보존, Photo set,
  계산/적용 절차, 임계값 편집
- Added: host receive proxy 증거 표시와 양쪽 app-receive ns 저장
- Removed: Live pair 판정에서 RTCP/NTP 필수 조건
- Reordered: 검증된 exposure 우선순위는 유지, FTP는 기존 dispatch 계약 유지,
  일반 Live fallback만 RTCP/NTP → app receive로 변경

## Decisions, confirmations, handoffs, waits, and artifacts

- 촬영 command acceptance은 성공이 아니며, worker의 checkerboard 검출/저장 결과가 성공 증거다.
- Capture boundary에서 Δt 판정과 시간 기준을 freeze하고 JSON으로 저장한다.
- 저장 artifact: `left/`, `right/`, `capture_metadata/capture_<id>.json`, session manifest.

## Interruption, cancellation, retry, restart, and resume

기존 worker lifecycle를 변경하지 않는다. 중복 capture 거부, worker 완료/실패 처리,
Auto Stop, Photo set 재사용은 그대로다. 앱 재시작 후 이전 증거를 재판정하지 않는다.

## Degraded, error, and recovery behavior

- 한쪽/두쪽 app-receive 시각 누락·비유효: stereo 거부, 유효한 쪽은 K,D-only 보존
- Δt 임계 초과: 기존과 같이 K,D-only 보존, status/detail에 ms 값 표시
- NTP 누락/오프셋: Live gate에 영향 없음, record-only 진단은 그대로
- FTP contract 누락/임계 초과: 기존 fail-closed 동작 유지

## Completion and next actions

성공 시 운영자는 status에서 app-receive 기준/Δt를 확인하고, Photo set을 선택해
기존과 같이 K,D/R,t 계산을 진행한다.

## Training, operating-procedure, and compatibility impact

새 control 학습은 없다. 운영자에게 app receive가 노출시각이 아니며 보드를 멈춘 상태로
촬영해야 함을 알린다. 과거 Photo set은 그 당시 저장된 판정을 유지한다.

## Workflow-to-screen, command, state, and regression traceability

`Capture Live Pair/Auto capture` → `CalibrationWorkflowController` → timing evidence →
`CheckerboardCaptureWorker` → capture JSON/status → session discovery/calculation snapshot.
Focused regression은 manual, Auto, threshold, missing timestamp, worker persistence, FTP, legacy read를 따로 검증한다.
