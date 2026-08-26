# Workflow Impact

## Affected operators and workflow IDs

- 현장 operator의 P1/P2 ETH 연결, RTK correction, measurement readiness workflow.
- `WF-PX4-CONNECT`, `WF-GNSS-READY`, `WF-MEASUREMENT-PREFLIGHT`, 장애 복구 절차.

## Current workflow

앱 시작 → P1/P2 MAVLink listener 열기 → 첫 heartbeat 수신 → stream 요청 → ATTITUDE가
fresh하면 LIVE/READY → stale이면 같은 버튼으로 host reconnect → 실패하면 Pixhawk 수동
전원 재부팅. RTCM route는 connection과 과거 heartbeat 존재만으로 계속 활성화될 수 있다.

## Proposed workflow

앱 시작 → 고정 GCS identity link + 1 Hz heartbeat → 필요한 Stock stream만 요청 →
heartbeat/ATTITUDE/any-packet 별도 관측 → 10분 qualification → READY. 운용 중 ATTITUDE만
늦으면 제한된 stream recovery를 수행하고, 모든 packet이 끊기면 affected side를
`ETH MAVLINK RX STALLED`로 격리하고 RTCM을 중단한다. 이후 승인된 복구 방식으로 해당
Pixhawk만 재부팅하고 qualification을 다시 수행한다.

## Preserved, added, removed, and reordered steps

- 보존: Stock firmware, IP/port, operator 연결 버튼, calibration 결과, per-side 상태.
- 추가: GCS heartbeat, stall 분류, per-side RTCM 차단, 10분 qualification, recovery 확인.
- 제거: PX4-side stall에서 무의미한 host socket 자동 reconnect loop.
- 재배치: READY 판정은 첫 sample 직후가 아니라 qualification 완료 뒤다.

## Decisions, confirmations, handoffs, waits, and artifacts

- 결정: stall 발견 시 heartbeat system ID로 현재 검증된 USB mapping이 있을 때만 operator
  확인 후 해당 side를 재부팅한다. 자동 재부팅은 하지 않는다.
- USB가 없거나 mapping이 불확실하면 command를 보내지 않고 named-side 수동 power-cycle을
  안내한다.
- runtime event log에는 side, endpoint, last heartbeat/ATTITUDE/any-packet age, RTCM 차단,
  recovery choice, soak start/end를 남긴다.

## Interruption, cancellation, retry, restart, and resume

- 앱 종료/재시작은 qualification progress를 이어받지 않는다. 새 process는 새로 10분을 검증한다.
- 한 side stall은 다른 side worker/RTCM route를 취소하지 않는다.
- 측정 기록 또는 finalization 중에는 자동 재부팅하지 않는다.
- qualification 중 stall이면 timer를 reset하고 operator action을 요구한다.

## Degraded, error, and recovery behavior

- heartbeat fresh + ATTITUDE stale: `ATTITUDE STALE`, 제한된 interval 재요청 허용.
- all inbound stale: `ETH MAVLINK RX STALLED`, high-rate outbound 중지, measurement not ready.
- NIC/link down 또는 bind failure: 기존 transport error로 분리; PX4 stall로 표시하지 않는다.
- reboot 후 packet 재개: `RECOVERING`, 10분 연속 evidence 후 `LIVE/READY`.

## Completion and next actions

양쪽 side가 qualification을 통과하고 RTK/기타 기존 gate도 충족해야 측정을 시작할 수 있다.
한 side가 stalled면 장애 side 복구 또는 해당 측정 방법의 명시적 제외가 필요하다.

## Training, operating-procedure, and compatibility impact

현장 SOP에 `STALE`과 `ETH MAVLINK RX STALLED` 차이, RTCM 차단 의미, 재부팅/10분 대기 절차를
추가한다. Stock firmware/기존 endpoint와 호환되며 PX4 parameter 자동 migration은 없다.

## Workflow-to-screen, command, state, and regression traceability

- 상태 카드: `status_model._refresh_pixhawk`.
- 연결/복구 command: `device_runtime_controller`.
- link/heartbeat/message timestamps: `workstation_core/pixhawk_attitude.py`.
- RTCM isolation/readiness: `device_runtime_workers.py`, `gnss_runtime_controller.py`.
- regression: `tests/test_workstation_pixhawk_gnss.py`, `tests/test_rtcm_pipeline_workers.py`,
  `tests/test_status_model_pixhawk.py`, UI contract tests.
