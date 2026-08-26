# Change Brief

## Operator outcome

Stock PX4 firmware를 유지한 P1/P2 Ethernet MAVLink 운용에서 Workstation이 불필요한
UDP 부하를 줄이고, 단순 ATTITUDE 지연과 전체 ETH MAVLink 수신 정지를 구분하며, 정지한
side로 RTCM을 계속 밀어 넣지 않는다. 재부팅 뒤에는 현재 현장 재현 시간보다 긴 연속
수신 검증을 통과해야 측정 준비 상태로 복귀한다.

## Motivation

2026-08-26 현장 감시에서 P1/P2는 재부팅 뒤 heartbeat/ATTITUDE/GPS를 정상 송신했지만
약 5분 30초 뒤 전체 Ethernet MAVLink 수신이 다시 멎었다. 같은 순간 PC의 NIC, IP,
ping, UDP listener는 정상이었고 USB MAVLink는 살아 있었다. 앱 재시작이나 host socket
재연결은 복구시키지 못했고 Pixhawk 재부팅만 복구시켰다. 이 증상은 PX4 공식 이슈
`PX4-Autopilot#25956`의 NuttX UDP `MavlinkReceiver`/IOB 고갈 deadlock과 일치한다.

따라서 `net.cfg`/`extras.txt` 부팅 순서 문제와 런타임 ETH 정지를 분리해야 한다. 전자는
P1 early `config.txt`로 완화되었고, 본 변경은 후자에 집중한다.

## Included scope

- 각 pymavlink link에 고정 GCS identity(`system=255`, `component=190`)를 부여한다.
- 연결이 열린 동안 side별 GCS heartbeat를 1 Hz로 전송한다. PX4가 GCS peer를 명시적으로
  인식하도록 하여 broadcast/unicast 중복 송신을 줄인다.
- Stock firmware가 실제로 제공하는 표준 message만 기본 요청하고, Stock에 없는
  `SASL_CAN_MAG` 등 진단용/custom stream은 기본 재시도 대상에서 제외한다. 측정에 필요한
  ATTITUDE/GNSS rate는 보존하고 최종 목록은 소비자 테스트로 고정한다.
- heartbeat, ATTITUDE, any inbound MAVLink의 마지막 수신 시각을 별도로 추적한다.
- ATTITUDE만 늦는 `STALE`과 모든 inbound packet이 끊긴 `ETH MAVLINK RX STALLED`를
  operator-visible state로 구분한다. 후자는 PX4 내부 deadlock의 확정 진단이 아니라
  관측 가능한 수신 정지 진단이다.
- stalled side는 RTCM injection route에서 즉시 제외하고 stream/parameter 재요청을 멈춘다.
  저율 GCS heartbeat만 recovery probe로 유지한다.
- host socket close/open 자동 반복을 중단하고 원인·side·마지막 packet age·조치 상태를
  기록한다.
- 재부팅/재연결 후 heartbeat와 ATTITUDE를 포함한 연속 10분 soak를 통과해야 READY로
  복귀한다. 현재 재현된 약 5분 30초 실패를 잡기 위한 현장 기준이다.
- operator가 확인한 뒤 현재 heartbeat system ID가 검증된 USB 장치로 해당 side만 재부팅한다.
  USB가 없거나 mapping이 불확실하면 자동 명령 없이 named-side 수동 power-cycle을 안내한다.

## Excluded scope and non-goals

- PX4/NuttX firmware patch 또는 custom firmware 배포는 포함하지 않는다.
- Workstation만으로 PX4 내부 deadlock이 절대 발생하지 않는다고 보장하지 않는다.
- `MAV_2_RATE`, EEPROM/MTD, `net.cfg`, `extras.txt`를 자동 변경하지 않는다.
- Pixhawk의 전원 또는 USB를 무조건 자동 재부팅하지 않는다.
- NTRIP HTTP 401 인증 문제와 앱/session 수명주기 변경은 별도 변경으로 유지한다.
- StereoWIRIS와 Workstation 동시 실행을 정상 운용 조건으로 만들지 않는다.

## Acceptance summary

1. 정상 link에서 P1/P2 각각 1 Hz GCS heartbeat와 고정 source identity가 확인된다.
2. PX4가 GCS peer를 잡은 뒤 같은 sequence의 broadcast/unicast 중복 telemetry가 사라지거나
   명시적으로 계측되어 감소 여부가 증명된다.
3. heartbeat가 살아 있고 ATTITUDE만 늦으면 기존 `STALE`; 모든 inbound가 10초 이상
   없으면 `ETH MAVLINK RX STALLED`로 표시한다.
4. stalled side로 새로운 RTCM frame, stream request, parameter command를 보내지 않는다.
5. 정상 side는 영향을 받지 않고 RTCM/telemetry를 계속 처리한다.
6. 재부팅 뒤 10분 연속 heartbeat+ATTITUDE가 확인되기 전에는 READY가 되지 않는다.
7. unit/integration test와 실제 P1/P2 10분 이상 soak evidence를 모두 남긴다.
