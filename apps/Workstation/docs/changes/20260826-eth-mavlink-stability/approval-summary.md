# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

Workstation이 P1/P2에 자신을 하나의 GCS로 명확히 알리고 1초마다 heartbeat를 보냅니다.
Stock PX4에 없는 custom telemetry를 기본 요청하지 않아 Ethernet MAVLink 부하를 줄입니다.
ATTITUDE만 늦는 경우와 모든 Ethernet MAVLink packet이 멎은 경우를 별도 상태로 표시하고,
멎은 side에는 RTCM과 추가 명령을 보내지 않습니다.

Stock firmware, P1/P2 IP·port·system ID, calibration 결과, RTCM frame 검증, 기존 측정
알고리즘은 그대로입니다. `MAV_2_RATE`, `net.cfg`, `extras.txt`도 자동으로 바꾸지 않습니다.

## Operator workflow before and after

현재는 첫 heartbeat/ATTITUDE 직후 LIVE가 되고, 멎으면 앱 socket을 다시 여는 방식입니다.
수정 후에는 연결 또는 재부팅 뒤 P1/P2가 10분간 heartbeat와 ATTITUDE를 연속 송신해야
READY가 됩니다. 전체 수신이 10초간 멎으면 해당 side는 `ETH MAVLINK RX STALLED`로
격리되고, operator가 선택한 복구 절차 뒤 10분 검증을 다시 시작합니다.

## Success, delay, rejection, failure, overload, and recovery

- 성공: 양쪽이 10분 검증을 통과하고 기존 GNSS/측정 조건도 충족합니다.
- 지연: ATTITUDE만 늦으면 `ATTITUDE STALE`로 표시하고 제한된 stream 재요청만 합니다.
- 실패: 모든 packet이 10초간 없으면 해당 side의 RTCM/명령을 차단합니다.
- 과부하 완화: 고정 GCS identity, 1 Hz heartbeat, Stock용 기본 stream 목록을 사용합니다.
- 복구: host socket 재연결을 Pixhawk 복구로 오인하지 않으며, 아래 정책 중 하나를 사용합니다.

## Data and compatibility effects

저장 데이터나 calibration을 이동·삭제·변환하지 않습니다. PX4 parameter를 자동 저장하지
않으므로 기존 장비 설정과 되돌리기가 단순합니다. Custom firmware 전용 stream은 기본
운용에서 빠지지만, 실제 측정에 필요한 표준 ATTITUDE/GNSS data rate는 유지합니다.

## Performance priorities and allowed degradation

신뢰 가능한 현재성 증거와 PX4 UDP 부하 감소가 우선입니다. 연결 후 READY까지 10분이
추가되지만, 현재 재현된 약 5분 30초 후 failure를 측정 전에 잡기 위한 의도된 지연입니다.
한 side가 실패해도 다른 side와 카메라/NTRIP worker는 계속 동작합니다.

## Visual and interaction changes, including dynamic-layout stability

새 화면을 만들지 않고 기존 P1/P2 status row를 사용합니다. `QUALIFYING`, `LIVE`,
`ATTITUDE STALE`, `ETH MAVLINK RX STALLED`, `RECOVERING`과 RTCM 차단 여부를 표시합니다.
1 Hz timer 변화가 row 폭·높이·focus를 움직이지 않도록 공간을 고정합니다.

## How acceptance will be demonstrated

자동 테스트로 heartbeat 주기, 상태 전이, 한 side만의 RTCM 차단, lifecycle/recovery guard를
검증합니다. 실제 P1/P2에서는 packet capture로 GCS peer와 중복 packet 감소를 확인하고,
RTCM을 활성화한 채 양쪽 모두 최소 10분 연속 수신되는지 검증합니다. 한 side 장애를
주입해 다른 side가 계속 동작하는지도 확인합니다.

10분은 implementation의 최소 hardware acceptance입니다. `Pixhawk.md`에 해결됐다고
기록하려면 추가로 3회의 독립 cold-boot cycle을 각각 10분 이상 통과하고, 정상 Workstation
부하와 RTCM을 켠 P1/P2 dual-link 60분 연속 soak, 한-side 장애 격리, verified-USB recovery를
모두 통과해야 합니다. 실패하면 문서에 해결 완료라고 기록하지 않습니다.

## Cost, risks, assumptions, rollback, and field limits

Workstation 수정은 공식 PX4/NuttX deadlock을 완화하고 빨리 발견할 뿐 firmware bug 자체를
고치지는 못합니다. 3-cycle/60분 통과도 무기한 무고장을 보장하지 않습니다. 현장 soak 후에도
재발하면 PX4 firmware patch 검토가 필요합니다. 변경은 PX4 media/parameter를 건드리지 않아
코드 단위로 되돌릴 수 있습니다.

## Exact approved scope

2026-08-26 16:32 KST에 위의 예방·분류·RTCM 격리·10분 qualification과 아래 복구 정책
1번이 이 packet 범위로 승인되었습니다.

1. **Operator 확인 후 USB reboot (권장)**: Workstation이 현재 heartbeat로 P1/P2 identity를
   검증한 USB port가 있을 때만 확인창 뒤 해당 side를 재부팅합니다. USB가 없거나 애매하면
   named-side 수동 전원 재부팅을 안내합니다.
2. **미승인 — Idle일 때 자동 USB reboot**: 측정/finalization 중이 아닐 때 확인 없이 해당 side를
   재부팅합니다. 무인 복구는 빠르지만 예기치 않은 재부팅 위험이 있습니다.
3. **미승인 — 진단만 제공**: Workstation은 RTCM 차단과 원인 안내만 하고 재부팅은 항상 사람이
   물리적으로 수행합니다. 가장 단순하지만 복구 시간이 길어집니다.

`MAV_2_RATE` 변경과 PX4 firmware patch는 이 승인에 포함되지 않습니다.

승인 조건에는 모든 software/hardware resolution gate 성공 후에만 `Pixhawk.md`를 갱신하는
작업도 포함됩니다. 승인 당시 P1/P2는 ping에는 응답했지만 ETH MAVLink heartbeat가 없고
USB 장치도 없어 실제 firmware를 재확인할 수 없었습니다. 구현은 version-agnostic하게
진행하고, 실제 version/sysid/parameter 재확인은 hardware 시험 전 필수 gate로 남깁니다.

## 2026-08-31 approved P1 IMU delivery amendment

사용자는 P1 `SCALED_IMU/2/3`가 실시간으로 들어와 기존 Pixhawk IMU 표에 표시되는 결과를
직접 요청했다. Workstation은 initial Stock profile command 전송 자체가 아니라 각 stream의
실제 expected-sysid sample 수신을 성공 증거로 사용한다. 누락되거나 오래된 표준 IMU
stream만 기존 side send-lock에서 제한적으로 재요청하고, sample이 도착하면 즉시 재시도를
끝낸다. 전체 ETH RX stall에는 어떤 stream request도 보내지 않는다.

기존 IMU 표는 fresh sample만 `LIVE`로 표시한다. sample이 한 번도 없으면 `WAITING`, 마지막
sample이 오래되면 값을 보존한 채 `STALE`과 실제 age를 표시한다. 새 화면이나 자동 USB
명령은 추가하지 않는다. verified USB runtime recovery/reboot는 계속 별도 operator 확인을
요구하며 PX4 parameter/config/file/firmware는 변경하지 않는다.
