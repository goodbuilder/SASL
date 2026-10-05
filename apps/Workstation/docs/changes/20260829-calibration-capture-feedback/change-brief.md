# Change Brief

## Operator outcome

Calibration의 기본 `촬영` 탭에서 좌·우 checkerboard 검출 영상, 검출 상태,
이번 capture 검사 누계, 현재 capture set의 저장 후보 수를 동시에 확인한다.

## Motivation

현재 기본 탭은 마지막 L/R 결과만 표시하고 live checkerboard overlay와 누적 후보 수는
접힌 고급 화면 및 hidden owner 상태에 흩어져 있다. 운영자는 보드가 인식됐는지와
계산 가능한 사진이 얼마나 쌓였는지를 촬영 중 한눈에 판단할 수 없다.

## Included scope

- 기존 overlay를 기본 탭에도 표시
- source-backed capture snapshot과 current-root inventory
- 고정형 board/run/set/last-status 표시
- facade, evidence, 문서, 자동화·시각 검증

## Excluded scope and non-goals

- checkerboard, app-receive timing, NAL/FrameKey, FTP 또는 K,D/R,t 수학 변경
- Auto target 도달 시 자동 Stop
- Photo set 자동 선택, capture metadata migration, 새 worker/설정
- unrelated dirty-worktree cleanup

## Acceptance summary

기본 탭에서 양쪽 overlay와 비색상 상태를 보고, 실제 worker 완료 및 디스크 evidence와
일치하는 누계를 확인한다. 1366x768 이상에서 capture control, counter, status가 안정적으로
보이고 기존 고급 경로와 캡처 명령은 그대로 동작해야 한다.
