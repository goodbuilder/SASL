# P1/P2 Ethernet MAVLink 안정화 상세 실행 계획

## 0. 문서 목적과 현재 상태

이 문서는 2026-08-26 Workstation 현장에서 재현된 P1/P2 Ethernet MAVLink 수신 정지
문제를 다른 Codex 세션이 이전 대화 없이 이어서 구현·검증할 수 있도록 만든 단일 handoff
계획이다. 문제 상황, 원인 가설, 반증된 가설, 변경 범위, 구현 순서, 시험 절차, 실패 시
분기, 그리고 “해결되었다”고 판단할 조건을 모두 포함한다.

- 작업 디렉터리: `apps/Workstation`
- 조사 당시 Git baseline: `c4397995d864576c966702cfd3c398b6032f8990`
- 변경 packet: `docs/changes/20260826-eth-mavlink-stability/`
- packet 상태: `awaiting_approval`에서 다른 세션으로 handoff 예정
- source 구현: 아직 이 변경 packet으로 승인되거나 시작되지 않음
- 장비 전제: P1/P2 모두 공식 PX4 Stock Stable firmware 사용
- 정상 운용 전제: Workstation과 StereoWIRIS 앱은 동시에 실행하지 않음
- 권장 recovery policy: **operator 확인 후, 현재 heartbeat system ID가 검증된 USB
  장치로 해당 Pixhawk만 재부팅**
- 미승인 범위: PX4 firmware patch, `MAV_2_RATE` 변경, `net.cfg`/`extras.txt`/`config.txt`
  자동 변경, telemetry radio 통합

중요: 사용자는 “실제 장비에서 해결되었음이 검증된 경우에만 `Pixhawk.md`에 원인과
해결 방법을 기록”하도록 요구했다. 구현 또는 unit test만 끝났다는 이유로
`Pixhawk.md`에 해결 완료를 먼저 쓰면 안 된다.

## 1. 새 세션의 시작 순서

새 세션은 다음 순서로 시작한다.

1. 저장소 루트 `AGENTS.md`를 읽고 coordinate contract와 portability gate를 확인한다.
2. 이 파일과 같은 packet의 `state.md`, `approval-summary.md`, `decisions.md`,
   `current-behavior.md`, `backend-contract.md`, `verification.md`를 읽는다.
3. `git status --short`와 아래 owner 파일의 diff를 확인한다. Worktree에는 사용자의 기존
   수정이 매우 많으므로 reset, checkout, broad formatting을 하지 않는다.
4. 실제 연결된 P1/P2 firmware version을 다시 읽는다. 조사 로그는 PX4 1.14.3이지만,
   untracked `firmware/stock/` bundle은 v1.17.0이므로 repo artifact와 실제 장비를 혼동하지
   않는다.
5. 사용자에게 이 packet 전체 범위와 recovery policy 1번의 명시적 승인을 확인한다.
   승인 전에는 source를 수정하지 않는다.
6. 승인 후 `state.md`를 `approved`/`implementation_allowed: true`로 먼저 기록하고,
   구현 중에는 `implementing`, 검증 중에는 `verifying`으로 갱신한다. 매 세션 마지막에
   `state.md`를 마지막으로 수정한다.
7. 아래 구현 slice를 작은 순서로 진행하며 각 slice마다 집중 테스트를 통과시킨다.

다음 세션에서 사용할 시작 요청의 예시는 다음과 같다.

> `docs/changes/20260826-eth-mavlink-stability/PLAN.md`와 `state.md`를 읽고,
> recovery policy 1번을 포함한 변경 범위로 구현을 시작해. 기존 dirty worktree는 보존하고,
> slice별 테스트 후 실제 P1/P2 검증 전에는 Pixhawk.md에 해결 완료를 쓰지 마.

## 2. 목표

### 2.1 운영 목표

Stock PX4를 유지하면서 Workstation이 다음을 수행하도록 한다.

1. PX4가 Workstation을 일관된 GCS peer로 인식하도록 한다.
2. Stock에서 불필요하거나 지원되지 않는 request를 줄여 UDP/IOB 압력을 낮춘다.
3. ATTITUDE stream만 늦는 상태와 Ethernet MAVLink 전체 수신이 멎은 상태를 구분한다.
4. 멎은 side로 RTCM과 추가 command를 계속 보내지 않는다.
5. 효과가 없다고 입증된 host socket 자동 재연결을 remote recovery처럼 실행하지 않는다.
6. 정확한 side가 USB로 검증된 경우에만 operator 확인 후 재부팅한다.
7. 재부팅 또는 link 재생성 후 충분한 연속 수신 검증을 통과해야 READY로 복귀한다.

### 2.2 비목표

- Workstation만 수정해서 PX4/NuttX 내부 defect가 제거되었다고 주장하지 않는다.
- 이번 slice에서 PX4를 custom firmware로 교체하거나 소스 patch하지 않는다.
- `MAV_2_RATE=2000000`을 몰래 낮추거나 PX4 persistent parameter를 자동 변경하지 않는다.
- NTRIP HTTP 401 인증 문제, process/orphan 종료 문제, session/Measurement 구조를 함께
  수정하지 않는다.
- StereoWIRIS 동시 실행을 정상 시나리오로 추가하지 않는다.
- telemetry radio는 독립 serial backup이 될 수 있지만 ETH deadlock 예방책으로 취급하지
  않는다.
- calibration, 측정 수학, 좌표계, 기록 schema를 변경하지 않는다.

## 3. 장비와 네트워크 기준

| 항목 | P1 / Left | P2 / Right |
|---|---:|---:|
| Workstation side key | `left` | `right` |
| PX4 IP | `10.0.0.2/24` | `10.0.0.3/24` |
| Workstation listener | `udpin:0.0.0.0:14550` | `udpin:0.0.0.0:14551` |
| MAVLink system ID | `1` | `2` |
| PC IP | `10.0.0.10/24` | `10.0.0.10/24` |

조사 당시 양쪽 saved parameter readback:

```text
MAV_2_CONFIG=1000
MAV_2_MODE=0
MAV_2_BROADCAST=1
MAV_2_RATE=2000000
P1 MAV_2_REMOTE_PRT=14550, MAV_2_UDP_PRT=14550
P2 MAV_2_REMOTE_PRT=14551, MAV_2_UDP_PRT=14551
```

양쪽 `/fs/microsd/etc/extras.txt`의 의도된 내용:

```sh
ifup eth0
ifconfig eth0 10.0.0.x netmask 255.255.255.0 up
```

`extras.txt`에는 `mavlink start`/`mavlink stop`을 넣지 않는다. Ethernet instance owner는
`MAV_2_CONFIG=1000`을 처리하는 PX4 `rc.serial` 하나여야 한다.

P1에는 operator 승인 후 다음 early `/fs/microsd/etc/config.txt`가 적용되고 readback/hash가
확인되었다.

```sh
# SASL early Ethernet setup: run before rc.serial
ifup eth0
ifconfig eth0 10.0.0.2 netmask 255.255.255.0 up
```

- SHA-256:
  `12d5d5675e8fe7da148dd17172af574ab61e7331844a0f59cc9f686e5e815d9d`
- 변경 전 P1 extras backup:
  `/tmp/P1-extras-before-20260826T140536KST.txt`

이 early config는 **부팅 직후 Ethernet stream이 시작되지 않는 문제**를 완화했지만,
몇 분 정상 동작 뒤 모든 UDP telemetry가 멎는 runtime 문제는 해결하지 못했다. 두 문제를
섞어 `net.cfg`/`extras.txt`만 다시 쓰는 방식으로 runtime 문제를 해결하려고 하면 안 된다.

## 4. 확인된 문제 상황

### 4.1 operator-visible 증상

- 연결 직후 P1/P2가 잠깐 `STALE`로 보였다가 다시 `LIVE`가 되기도 했다.
- 일정 시간 정상 telemetry를 수신한 뒤 P1/P2 Ethernet MAVLink가 주기적으로 끊겼다.
- Pixhawk 전원은 켜진 상태였고, Workstation 앱 재시작 또는 MAVLink 재연결로 복구되지 않았다.
- Pixhawk 전원 재부팅 뒤에만 Ethernet telemetry가 다시 들어왔다.
- 재부팅 후에도 같은 정지가 재현됐다.

### 4.2 정밀 감시 timeline

주요 raw evidence는 다음 machine-local 로그에 남아 있다. 다른 machine으로 옮겨질 수
있도록 필요한 사실은 이 문서에도 모두 전사했다.

- `/tmp/sasl_eth_mav_dynamic_monitor_20260826T143716KST.jsonl`
- `/tmp/sasl_eth_mav_monitor_p1_config_20260826T141016KST.jsonl`
- `/tmp/sasl_eth_mav_monitor_20260826T125213KST.jsonl`

주요 event:

| 시각 (KST) | P1 | P2 | 의미 |
|---|---|---|---|
| 14:39:05.241 | LIVE | - | hub/NIC 복구 후 P1 UDP 수신 시작 |
| 14:39:05.551 | LIVE | LIVE | P2도 수신 시작 |
| 14:42:03.079 | STALE | LIVE | P1 전체 UDP silence, age 2.183 s |
| 14:42:05.749 | STALE | STALE | P2도 2.67 s 뒤 silence |
| 15:13:30.887 | LIVE | STALE | board reboot/power sequence 후 P1 회복 |
| 15:13:31.095 | LIVE | LIVE | P2 회복; 이때 USB 장치는 모두 제거된 상태 |
| 15:19:03.583 | STALE | LIVE | P1이 약 5분 32.7초 후 다시 정지 |
| 15:19:09.165 | STALE | STALE | P2가 약 5분 38.1초 후 다시 정지 |
| 15:19 이후~15:43 | STALE | STALE | 자발 회복 없음, packet 0 유지 |

두 번째 구간은 USB가 없는 상태에서도 같은 runtime failure가 발생했으므로 USB subscriber
경합이 원인이라는 설명과 맞지 않는다. P1/P2가 몇 초 간격으로 함께 정지한 점은 공통
traffic pattern 또는 공통 network 부하가 trigger일 가능성을 높이지만, host NIC 자체가
죽었다는 뜻은 아니다.

### 4.3 정상 구간 telemetry

- HEARTBEAT: P1/P2 각각 약 1 Hz
- ATTITUDE: 첫 구간 P1 약 15 Hz, P2 약 30 Hz
- GPS: 각각 약 5 Hz
- parser errors: 0
- 30초 summary의 raw packet 규모는 P1 약 3,100, P2 약 3,000~3,940 packet이었다.
- 같은 source packet이 LAN unicast와 broadcast로 중복 관측된 구간이 있었다.

### 4.4 정지 순간에도 정상인 것

- host Realtek Ethernet interface, link, `10.0.0.10/24` 주소
- P1/P2 IP ping
- Workstation의 UDP 14550/14551 listener ownership
- 첫 drop 당시 P1/P2 USB MAVLink telemetry
- Pixhawk 전원
- outbound Workstation RTCM send path

즉 `socket exists`나 `ping succeeds`를 MAVLink service가 살아 있다는 증거로 사용할 수 없다.

### 4.5 RTCM 검증 결과

Workstation은 수신이 멎은 뒤에도 양쪽 connection object로 MAVLink
`GPS_RTCM_DATA`를 계속 보냈다. 그러나 캡처한 RTCM 자체는 다음과 같이 유효했다.

- RTCM type: 1074, 1084, 1094, 1124
- 약 4 complete RTCM frame/s/board
- 모든 frame이 180 bytes 미만이어서 MAVLink fragmentation이 필요하지 않음
- 표본 48/48 MAVLink parse 성공
- 표본 48/48 RTCM CRC 성공
- `Rtcm3StreamParser`가 TCP chunk가 아니라 완전한 frame만 injection worker에 전달

따라서 malformed RTCM framing은 현재 failure의 원인에서 배제한다. 다만 유효한 RTCM도
PX4 UDP receive queue의 traffic 양에는 포함되므로 deadlock trigger 부하를 높이는 요소일
수는 있다.

### 4.6 firmware 및 parameter 저장 관련 별도 위험

조사 당시 boot evidence:

- `/tmp/sasl_P1_param_import_fail.txt`
- `/tmp/sasl_P2_param_import_fail.txt`
- PX4 `Release 1.14.3`, git hash
  `1dacb4cdef2d7145754fc788fa8dc482eed74b40`
- NuttX 11.0.0
- 양쪽 모두 `failed to initialize EEPROM driver`,
  `mtd failure: -5 bus 3 address 81 class 4`, parameter import failure 기록

그 뒤 USB readback에서는 필요한 `MAV_2_*` 값이 확인되었으므로 이 MTD error가 이번
5.5분 runtime drop의 직접 원인이라고 입증되지는 않았다. 그러나 parameter persistence와
board health의 별도 위험이므로 새 session은 실제 firmware/version/parameter를 다시 읽고
evidence에 남겨야 한다.

또한 worktree의 `firmware/stock/`은 PX4 v1.17.0 bundle을 설명한다. **이 파일이 존재하는
것은 실제 P1/P2가 v1.17.0으로 flash되었다는 증거가 아니다.** 실제 장비에서
`AUTOPILOT_VERSION`, USB boot output, 또는 QGC firmware metadata를 다시 확인한다.

### 4.7 별도 hub 사건

14:35경 P1 branch USB가 먼저 끊기고, 뒤이어 USB2/USB3 hub와 Realtek NIC 전체가
disconnect된 사건이 있었다. kernel overcurrent message는 없었고 upstream cable/shared
power interruption 가능성이 있다. hub reconnect 후 host와 양쪽 Pixhawk가 복구되었다.

이 사건은 물리적 안정성 문제로 별도 관리해야 하지만, 이후 NIC와 ping이 유지된 채
5.5분 후 재현된 runtime UDP silence의 직접 원인은 아니다.

## 5. 예상 원인과 확신도

### 5.1 1순위: PX4/NuttX UDP `MavlinkReceiver` deadlock — 고신뢰 추정

PX4 공식 open issue:

- <https://github.com/PX4/PX4-Autopilot/issues/25956>
- 제목: `[Bug] MavlinkReceiver over UDP deadlocks on NuttX under high traffic`
- issue 상태 확인일 2026-08-26: open

공식 issue가 설명하는 mechanism:

1. UDP readahead queue가 NuttX IOB pool을 소진한다.
2. 같은 `MavlinkReceiver::run` task가 `recvfrom()`과 `sendto()`를 순차 수행한다.
3. 전송용 IOB가 남지 않은 상태에서 blocking/infinite-timeout `sendto()`가 멈춘다.
4. receiver가 다시 `recvfrom()`으로 돌아가지 못해 readahead queue가 비워지지 않는다.
5. PX4 restart 전까지 UDP MAVLink가 회복되지 않는다.
6. issue 작성자는 receive/send buffer accounting 수정 또는 nonblocking/timeout socket을
   적용한 두 방식을 시험해 해결됐다고 보고했다.

현재 현장 증상과 일치하는 부분:

- high UDP traffic 중 전체 telemetry drop
- host network와 autopilot 자체는 살아 있음
- app/socket restart로 회복되지 않음
- PX4 restart만 회복
- 다시 traffic을 받으면 일정 시간 뒤 재발

단, 현재 Stock board에서 PX4 task stack과 IOB pool 상태를 직접 읽지 못했다. Stock
`SERIAL_CONTROL` shell이 output을 돌려주지 않아 exact stack trace가 없다. 따라서 문서와
UI에는 다음처럼 표현한다.

> `Ethernet MAVLink RX stalled; PX4/NuttX UDP task deadlock suspected`

`PX4 deadlock proven` 또는 `firmware bug fixed`라고 쓰지 않는다.

### 5.2 trigger 후보: 과도한 stream request + RTCM inbound traffic — 중간 이상

현재 Workstation은 첫 heartbeat 뒤 다음 16개 stream을 요청한다.

```text
ATTITUDE 30 Hz
SYS_STATUS 1 Hz
GPS_RAW_INT 5 Hz
GPS2_RAW 5 Hz
GPS_RTK 5 Hz
GPS2_RTK 5 Hz
SCALED_IMU 20 Hz
SCALED_IMU2 20 Hz
SCALED_IMU3 20 Hz
DEBUG_VECT 20 Hz
SASL_CAN_MAG 20 Hz
RAW_IMU 20 Hz
LOCAL_POSITION_NED 10 Hz
GLOBAL_POSITION_INT 10 Hz
VFR_HUD 5 Hz
BATTERY_STATUS 2 Hz
```

Stock firmware에는 private `SASL_CAN_MAG`이 없는데 Workstation은 이를 최대 3회 재요청한다.
또한 RTCM 약 4 frame/s/board를 계속 보낸다. 이 traffic은 공식 issue의 IOB exhaustion을
유발하거나 빨리 드러내는 trigger가 될 수 있다. 다만 어떤 message/rate가 임계점을 넘기는지
A/B capture로 아직 분리하지 않았다.

### 5.3 trigger 후보: GCS peer 확립 부족과 unicast/broadcast 중복 — 중간

운영 connection 생성은 `source_system`/`source_component`를 명시하지 않는다. 현재 host에
설치된 pymavlink signature의 기본값은 `source_system=255`, `source_component=0`이므로
source system이 실제로 0인 것은 아니다. 단순히 `255`를 명시하는 것만으로 장애가 고쳐질
가능성은 낮다.

더 중요한 차이는 Workstation이 운영 link에서 주기적인 `MAV_TYPE_GCS` HEARTBEAT를 보내지
않는다는 점이다. PX4가 명확한 GCS peer를 학습하지 못하면 broadcast와 unicast stream이
중복될 가능성이 있다. 따라서 identity를 `255/190`으로 명시하고 GCS heartbeat를 1 Hz로
보내되, 실제 중복 packet 감소는 반드시 packet capture로 검증한다. 감소하지 않으면 이
변경을 해결 원인으로 주장하지 않는다.

### 5.4 낮거나 반증된 원인

| 가설 | 판단 | 근거 |
|---|---|---|
| `net.cfg`/`extras.txt` 내용만의 문제 | runtime 원인 아님 | 정상 LIVE 뒤 5.5분 후 재현; ping/IP 유지 |
| Workstation UDP port 경합 | 반증 | 14550/14551 owner는 Workstation 하나뿐 |
| QGroundControl 동시 실행 | 반증 | QGC/local competing owner 없음 |
| malformed RTCM/MAVLink | 반증 | 48/48 parse 및 CRC valid |
| USB MAVLink 경합 | 반증 | USB 제거 상태에서도 재현 |
| host NIC RX drop/overload | 낮음 | 약 125.7 Mbps RX, error/drop 0, ping 유지 |
| StereoWIRIS UDP owner | 반증 | StereoWIRIS에는 해당 UDP socket 없음 |
| StereoWIRIS 부하 | 낮음/미확정 | 첫 drop과 시간상 가까웠지만 camera worker 종료 후에도 회복 안 됨 |
| hub disconnect | 별도 물리 사건 | NIC가 계속 살아 있는 두 번째 재현을 설명하지 못함 |
| old custom firmware hardfault | 제외 | 과거 P2 archive이며 현재 Stock runtime과 다른 build |

## 6. 현재 Workstation 동작에서 수정할 지점

### 6.1 MAVLink connection과 heartbeat

Owner: `workstation_core/pixhawk_attitude.py`, `DualPixhawkMavlinkSubscriber`

- `start()`가 `mavutil.mavlink_connection()`을 만들지만 source identity를 계약으로 명시하지
  않는다.
- 운영 link에는 periodic GCS heartbeat가 없다.
- `_heartbeat_logged`는 “한 번이라도 heartbeat를 봄”을 나타낼 뿐 현재 freshness가 아니다.
- `_last_attitude_message_s`만 일부 freshness를 보유하며 모든 packet silence를 추적하지 않는다.

### 6.2 stream request

Owner: `MAVLINK_RM3100_STREAM_REQUESTS`, `_apply_mavlink_stream_config()`,
`_request_mavlink_sensor_streams()`, `_request_stale_attitude_stream()`

- Stock에 없는 private message까지 default request한다.
- stream consumer별 필수 여부가 request profile과 분리돼 있지 않다.
- heartbeat가 살아 있는 ATTITUDE-only stall과 heartbeat를 포함한 전체 receiver stall에서
  같은 request recovery가 실행될 수 있다.

### 6.3 stale reconnect

Owner: `device_runtime_controller.py`

- `_mavlink_side_is_stale()`는 ATTITUDE timestamp age 5초를 사용한다.
- `_reconnect_stale_mavlink_sides()`는 host connection을 close/open한다.
- 현장 evidence에서 remote PX4-side task가 막힌 경우 이 reconnect는 효과가 없었다.

### 6.4 RTCM route

Owners:

- `device_runtime_workers.py::PixhawkRtcmInjectionWorker`
- `gnss_runtime_controller.py::_handle_gnss_base_rtcm_frame()`
- `gnss_runtime_controller.py::_refresh_gnss_operation_readiness()`

현재 route는 connection object와 과거 heartbeat set이 있으면 live로 간주할 수 있다.
RTCM worker는 frame age와 queue bound는 검증하지만 target의 현재 MAVLink RX liveness는
send 직전에 검증하지 않는다.

### 6.5 UI status와 reboot

Owners:

- `status_model.py::_refresh_pixhawk()`
- `device_runtime_controller.py::reboot_pixhawk_from_mavlink()`
- 기존 P1/P2 status/reboot controls in `standalone_gui.py`/workspace projection

현재 UI는 ATTITUDE age 기반 generic `STALE`을 표시한다. 기존 reboot 버튼은 현재 active
MAVLink connection으로 명령을 보내는데, ETH receiver가 deadlock인 바로 그 상황에서는
명령이 도달할 것이라고 믿을 수 없다.

USB discovery도 현재 `(device, baud)`만 반환하고 heartbeat source system ID를 보존하지
않는다. 안전한 side-only USB reboot를 위해 `(device, baud, sysid, component, observed_at)`
같은 검증 evidence가 필요하다.

## 7. 제안하는 최종 동작 계약

### 7.1 per-side health model

subscriber가 최소 다음 immutable snapshot을 제공한다. 정확한 class 이름은 구현 중 기존
style에 맞춰 정하되 private set을 다른 controller가 직접 읽지 않게 한다.

```text
side
connection_generation
endpoint
expected_system_id
observed_system_id
identity_verified
listener_open
last_any_rx_monotonic_s
last_heartbeat_rx_monotonic_s
last_attitude_rx_monotonic_s
last_gcs_heartbeat_tx_monotonic_s
active_command
state
qualification_started_monotonic_s
qualification_elapsed_s
rtcm_eligible
command_eligible
stall_reason
```

관측 규칙:

- expected system ID의 parse 가능한 MAVLink packet만 `last_any_rx`를 갱신한다.
- wrong sysid, `BAD_DATA`, local echo는 liveness 증거로 쓰지 않는다.
- broadcast/unicast duplicate도 실제 transport arrival이므로 `last_any_rx`는 갱신하되,
  downstream sample은 기존 duplicate filter로 한 번만 처리한다.
- blocking PARAM/SERIAL_CONTROL command가 receive path를 예약하는 동안 false stall을 만들지
  않는다. 가능하면 deferred message에 receive timestamp를 함께 저장하거나, 최소한
  `active_command` 동안 stall timer를 정지하고 종료 직후 fresh evidence를 요구한다.
- 모든 clock은 `time.monotonic()`/`monotonic_ns()`를 사용한다.
- old connection callback이 새 link를 살리는 것을 막기 위해 generation을 검증한다.

### 7.2 상태와 threshold

| 상태 | 판정 | 동작 |
|---|---|---|
| `DISCONNECTED` | listener 없음 | 기존 연결 UI |
| `CONNECTING` | listener는 열렸지만 expected remote identity 미확인 | READY 금지 |
| `QUALIFYING` | identity, heartbeat, ATTITUDE fresh; 10분 미완료 | timer 표시, READY 금지 |
| `LIVE` | 10분 연속 qualification 완료, sample fresh | 기존 readiness 평가 허용 |
| `ATTITUDE STALE` | ATTITUDE age > 2 s, heartbeat/other RX fresh | bounded ATTITUDE request |
| `ETH MAVLINK RX STALLED` | 이전에 verified/live였고 모든 expected-sysid RX가 10 s 이상 없음 | RTCM/command 차단 |
| `RECOVERING` | stalled 뒤 packet 재개 또는 approved reboot 완료 | qualification 0부터 재시작 |
| `ERROR` | bind/NIC/worker error | 기존 transport error 처리 |

10초는 “PX4 내부 deadlock 확정” 시간이 아니라 **Workstation이 더는 이 link를 correction
delivery/measurement evidence로 신뢰하지 않을 시간**이다. UI detail은 마지막 HB/ATT/ANY age,
endpoint, `RTCM blocked`, `PX4 UDP task stall suspected`를 표시한다.

`active_command` 중에는 `PARAM SYNC` 또는 기존 busy 상태를 유지한다. command 종료 뒤
fresh packet이 없으면 정상 threshold를 다시 적용한다.

### 7.3 GCS identity와 heartbeat

- operational connection 생성 시 `source_system=255`, `source_component=190`을 명시한다.
- heartbeat payload는 `MAV_TYPE_GCS`, `MAV_AUTOPILOT_INVALID`, base/custom mode 0,
  active/uninitialized 중 기존 temporary probe와 호환되는 상태를 사용한다.
- `udpin` listener는 remote packet 전에는 reply destination을 모를 수 있으므로, expected
  sysid의 첫 packet으로 peer가 확인된 뒤 heartbeat를 시작한다.
- side별 최대 1회/s, missed tick catch-up 없음, send exception은 throttled log 처리한다.
- stop/disconnect 시 timer/state를 정리하고 stale connection에 heartbeat를 누적 전송하지
  않는다.
- `source_system=255`는 현재 pymavlink default와 같으므로, acceptance는 kwargs 존재가 아니라
  실제 1 Hz outbound GCS heartbeat와 PX4 peer/traffic 변화로 판단한다.

### 7.4 Stock-compatible stream profile

무작정 request 전체를 한 번에 줄이면 어느 변경이 효과를 냈는지 알 수 없고 sensor UI를
깨뜨릴 수 있다. 다음 두 단계로 한다.

1. **확실한 제거:** Stock profile에서 `SASL_CAN_MAG` request와 retry를 제거한다. Stock에는
   이 private message가 없다.
2. **consumer audit 후 최소화:** `_handle_mavlink_message()`와 모든 downstream consumer/test를
   message별로 매핑한다. Measurement readiness/evidence, GNSS, battery, attitude, position,
   magnetometer 화면이 실제로 사용하는 표준 message만 남긴다.

초기 보존 후보:

- `ATTITUDE` 30 Hz
- `SYS_STATUS` 1 Hz
- `GPS_RAW_INT`/`GPS2_RAW` 5 Hz
- `GPS_RTK`/`GPS2_RTK` 5 Hz
- `BATTERY_STATUS` 2 Hz

조건부/진단 후보:

- `SCALED_IMU`, `SCALED_IMU2`, `SCALED_IMU3`
- `RAW_IMU`, `DEBUG_VECT`
- `LOCAL_POSITION_NED`, `GLOBAL_POSITION_INT`, `VFR_HUD`

조건부 후보는 consumer가 없거나 Stock에서 의미 있는 sample이 없다는 테스트/evidence 없이
제거하지 않는다. 각 request rate를 변경할 때 UI/measurement requirement와 observed rate를
나란히 기록한다.

`MAV_2_RATE=2000000`은 ceiling이며 실제 rate와 같지 않다. 첫 implementation에서는
persisted value를 유지한다. 완화 후에도 failure가 재현될 때만 별도 승인으로 500000,
250000, 100000 등의 A/B를 수행하며 한 번에 하나의 변수만 바꾼다.

### 7.5 stalled side fail-closed

`ETH MAVLINK RX STALLED` transition 시 해당 side에 대해:

- 새 RTCM target enqueue 금지
- 이미 queue에 있던 frame도 send 직전 liveness 재검사 후 해당 side copy drop
- stream request, parameter command, shell command 금지
- 자동 host socket reconnect loop 중단
- 중복 transition/log spam 금지
- 저율 GCS heartbeat만 recovery probe로 유지 가능
- 다른 side의 queue, connection, RTCM send는 계속 진행

RTCM outcome에는 기존 `fail`과 별도로 다음을 기록한다.

```text
drop_reason=target-mavlink-stalled
dropped_targets=(left|right)
blocked_stall count
last_target_rx_age_s
connection_generation
```

이 drop은 transport send exception이 아니라 정책상 fail-closed이므로 `fail` counter와
구분한다. GNSS aggregate UI가 한 side 성공만으로 양쪽 성공처럼 보이지 않게 한다.

### 7.6 자동 stale reconnect 제거

- ATTITUDE만 stale이고 heartbeat/any RX가 fresh하면 기존 bounded ATTITUDE interval request를
  유지한다.
- all-RX stalled이면 `_reconnect_stale_mavlink_sides()`의 host close/open을 자동 실행하지
  않는다.
- operator가 명시적으로 disconnect/connect할 수는 있지만, UI/log는 이것이 PX4-side
  restart가 아니며 deadlock recovery를 보장하지 않는다고 표시한다.
- fresh expected packet이 자발적으로 들어오면 `RECOVERING`으로 이동하고 10분 qualification을
  새로 시작한다. 바로 `LIVE`로 승격하지 않는다.

### 7.7 권장 복구 정책: 확인 후 verified-USB reboot

정책 1번이 별도로 승인됐을 때만 구현한다.

1. USB 후보를 열어 heartbeat를 받고 `device`, `baud`, source system/component,
   observation time을 반환한다.
2. expected mapping은 P1=sysid 1, P2=sysid 2로만 결정한다. `/dev/ttyACM0` 같은 이름이나
   열거 순서로 side를 추정하지 않는다.
3. 같은 sysid가 둘 이상이거나, sysid가 없거나, observation이 stale이면 reboot action을
   disable한다.
4. confirmation dialog에 `P1/P2`, sysid, USB by-id path, measurement readiness reset을 표시한다.
5. measurement recording, save/finalization, calibration write, 다른 PX4 command가 active이면
   reboot를 거부한다.
6. **stalled Ethernet connection으로 reboot command를 보내지 않는다.** 검증된 USB serial
   connection을 짧게 독점해 `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`을 보내고 닫는다.
7. command send를 실제 reboot 완료로 간주하지 않는다. USB disconnect/re-enumeration 또는
   Ethernet heartbeat 재개와 expected sysid를 다시 관측해야 `RECOVERING`으로 간다.
8. USB가 없거나 mapping이 불확실하면 named-side 수동 전원 재부팅만 안내한다.
9. 자동/무확인 reboot는 하지 않는다.

기존 `reboot_pixhawk_from_mavlink()`은 live ETH 수동 reboot 용도로 남길지, 하나의
side-verified recovery command로 통합할지 impact를 조사한다. 두 버튼이 서로 다른 안전
계약으로 보이지 않게 한다.

### 7.8 qualification과 READY

- connection generation마다 qualification을 0부터 시작한다.
- expected sysid heartbeat와 ATTITUDE가 각각 freshness threshold 안에 연속으로 있어야 한다.
- ATTITUDE/HB gap, stall, disconnect, identity mismatch, connection generation 교체가 있으면
  0으로 reset한다.
- 10분 연속 통과 전에는 Ethernet health가 READY를 허용하지 않는다.
- NTRIP/RTK, camera, measurement method 등 기존 readiness gate는 그대로 추가 적용한다.
- 앱 재시작 후 qualification을 persistent resume하지 않는다.
- 긴 측정 간격 때문에 calibration을 제외한 preflight/connection/RTK/qualification 반복은
  허용된 운영 비용이다.

## 8. 구현 slice와 각 종료 조건

`verify-check` 원칙에 따라 한 slice가 독립적으로 증명된 뒤 다음으로 진행한다.

### Slice 0 — baseline 재확인 및 승인 고정

수정 없음.

- actual P1/P2 firmware/version/sysid/params를 read-only로 기록
- `ss -lunp`로 14550/14551 단일 owner 확인
- QGC/StereoWIRIS/orphan process 부재 확인
- 현재 owner 파일 diff 보존
- recovery policy 1과 전체 scope 승인 기록

종료 조건: packet `state.md`가 승인 scope/time/조건을 기록하고
`implementation_allowed: true`이다.

### Slice 1 — pure liveness/qualification model

가능 owner:

- `workstation_core/pixhawk_attitude.py` 또는 작은 새 core module
- focused unit tests

구현:

- immutable side snapshot
- expected identity validation
- HB/ATT/ANY timestamp
- deterministic state transition/10초 stall/600초 qualification
- connection generation
- active-command suppression contract

종료 조건: fake clock만으로 모든 threshold edge와 reset/recovery transition이 증명된다.

### Slice 2 — explicit GCS heartbeat lifecycle

Owner: `workstation_core/pixhawk_attitude.py`

구현:

- explicit 255/190 identity
- first verified peer 뒤 1 Hz GCS heartbeat
- no catch-up, exception throttle, stop cleanup
- any-RX observation을 duplicate filter 이전의 올바른 boundary에 배치

종료 조건: fake pymavlink connection으로 두 side 독립 주기, wrong sysid, disconnect,
exception, generation replacement가 통과한다.

### Slice 3 — Stock stream profile

Owner: `workstation_core/pixhawk_attitude.py`와 consumer tests

구현:

- Stock profile에서 private `SASL_CAN_MAG` 제거/재시도 제거
- 표준 stream consumer matrix 작성
- 필수/진단 profile 분리 또는 기존 UI를 보존하는 최소 profile 확정
- ATTITUDE-only recovery는 전체 stall에서 비활성

종료 조건: 각 유지 message의 consumer가 test로 추적되고, 제거 message 때문에 기존
measurement/status test가 깨지지 않는다.

### Slice 4 — 상태 projection과 false reconnect 제거

Owners:

- `device_runtime_controller.py`
- `status_model.py`
- 기존 workspace/status components

구현:

- generic `STALE` 분리
- `QUALIFYING mm:ss / 10:00`, `ATTITUDE STALE`, `ETH MAVLINK RX STALLED`,
  `RECOVERING`
- stalled에서 host reconnect loop 금지
- state change 시 focus/layout 유지, log throttle

종료 조건: controller/status/UI contract test와 dynamic label layout check 통과.

### Slice 5 — RTCM per-side fail-close

Owners:

- `device_runtime_workers.py`
- `gnss_runtime_controller.py`
- RTCM pipeline tests

구현:

- route 선정 시 health snapshot 확인
- send 직전 generation/liveness 재확인
- 한 side만 drop하고 다른 side send
- `blocked_stall` evidence/counter
- readiness에서 blocked target 명시

종료 조건: enqueue 후 stall race, partial delivery, recovery, queue bound, shutdown tests 통과.

### Slice 6 — operator-confirmed USB recovery

Owners:

- `probe_usb_mavlink_devices()`와 discovery worker
- `device_runtime_controller.py`
- 기존 reboot controls

구현:

- USB source sysid/component evidence
- expected side mapping
- confirmation/lifecycle guards
- one-shot USB reboot worker
- reboot command 후 observed recovery/qualification

종료 조건: wrong/duplicate/missing sysid, stale discovery, cancel, active measurement,
worker failure, correct side success가 모두 테스트됨.

### Slice 7 — documentation contract와 전체 regression

`update-readme`와 `run-tests` skill을 사용한다.

- `README.md`와 UI contract docs에는 구현된 상태/명령/readiness truth를 갱신한다.
- `Pixhawk.md`의 해결 원인/방법 section은 아직 쓰지 않는다.
- focused tests, full suite, portability check 실행

종료 조건: 아래 software gate 모두 통과. 그 뒤에만 hardware verification으로 이동.

### Slice 8 — P1/P2 hardware verification

아래 10~12절의 절차를 수행한다. 실패하면 구현 또는 가설 단계로 돌아간다.

### Slice 9 — 조건부 `Pixhawk.md` 기록

“해결됨” 조건을 모두 만족한 경우에만 기록한다. 정확한 내용은 13절에 정의한다.

## 9. 자동 테스트 계획

### 9.1 connection/heartbeat

- `mavlink_connection()`에 255/190이 명시됨
- first expected packet 이전에는 잘못된 peer로 send하지 않음
- 두 side 각각 1 Hz 이하, missed tick burst 없음
- wrong sysid packet은 identity/qualification을 시작하지 않음
- stop 후 heartbeat 없음
- send exception이 UI thread를 깨뜨리지 않고 throttled event만 생성

### 9.2 liveness/state

- listener only → CONNECTING
- expected HB+ATT → QUALIFYING
- 599.999 s 연속 → QUALIFYING, 600.000 s → LIVE
- ATT age 2 s 경계 전후
- ANY age 10 s 경계 전후
- HB stale/other packet fresh와 all-packet silence 구분
- active PARAM command 중 false stall 없음
- stall 뒤 한 packet 수신 → RECOVERING, not LIVE
- gap/reconnect/generation 교체 시 qualification reset
- monotonic time만 사용

### 9.3 stream profile

- Stock profile에 `SASL_CAN_MAG` 없음
- unsupported private retry 없음
- expected standard message ID/rate exact test
- ATTITUDE-only stale에서 bounded retry
- all-RX stalled에서 no stream request
- measurement/GNSS/battery/position consumer regression

### 9.4 RTCM

- live P1/P2에는 같은 complete frame 전달
- P1 stalled/P2 live이면 P1 blocked, P2 sent
- enqueue 때 live였다가 send 전에 stalled이면 send-boundary에서 drop
- stale generation queue가 새 connection으로 전달되지 않음
- `blocked_stall`과 transport `fail` counter 구분
- queue/backpressure/2초 source-frame age 기존 계약 보존
- stopped worker가 pending request를 terminal outcome으로 마감

### 9.5 recovery safety

- USB discovery가 sysid/component를 반환
- P1은 sysid1, P2는 sysid2만 허용
- same sysid duplicate, no heartbeat, ambiguous port는 fail-closed
- operator No → command 없음
- measurement/save/finalization/other PX4 command active → reject
- stalled ETH connection으로 reboot send하지 않음
- USB command send만으로 LIVE 처리하지 않음
- expected Ethernet HB+ATT 후 RECOVERING/qualification
- one-side recovery가 다른 side state/RTCM을 변경하지 않음

### 9.6 UI/contract

- 상태 문자열과 severity mapping
- detail에 side/age/RTCM block/cause confidence 표시
- 1 Hz timer가 row width/height/focus/selection을 흔들지 않음
- minimum window/font scaling에서 longest label 표시
- unrelated camera/NTRIP/calibration/measurement result status 유지

### 9.7 실행 명령

정확한 environment는 현재 repo convention을 먼저 확인한다. 기본 명령 후보:

```sh
cd apps/Workstation
python3 -m pytest -q \
  tests/test_workstation_pixhawk_gnss.py \
  tests/test_rtcm_pipeline_workers.py \
  tests/test_rtcm_pipeline_end_to_end.py \
  tests/test_status_model_pixhawk.py \
  tests/test_workstation_ntrip.py \
  tests/test_workstation_lifecycle.py \
  tests/test_workstation_ui.py

python3 -m pytest -q
cd ../..
python3 tools/check_portable_paths.py
```

현재 `.venv`에는 pymavlink가 없고 system `python3`에는 설치되어 있는 것이 확인됐다.
테스트 interpreter를 임의로 바꾸지 말고 project test convention과 dependency check를 먼저
따른다.

## 10. 실제 장비 시험 사전 조건

### 10.1 안전 조건

- bench 상태, vehicle disarmed, actuator/propeller 위험이 없음을 operator가 확인
- QGroundControl 종료
- StereoWIRIS 정상 종료; Workstation만 실행
- 14550/14551 다른 owner 없음
- P1/P2 power/USB/Ethernet cable을 side별로 식별
- USB reboot 시험은 사용자가 승인한 시점에만 실행
- firmware flash, parameter save, config file write는 하지 않음

### 10.2 baseline inventory

각 run 시작 전에 다음을 evidence에 기록한다.

- 날짜/시간/timezone
- Git commit과 dirty file 목록
- Workstation process PID/version
- actual PX4 version/git hash/board GUID
- P1/P2 sysid, IP, port, MAC
- `MAV_2_CONFIG`, `MODE`, `BROADCAST`, `RATE`, `REMOTE_PRT`, `UDP_PRT`
- `netman show`, `ifconfig eth0`, 가능하면 `mavlink status`
- host interface 이름과 `10.0.0.10/24`
- `ss -lunp` 14550/14551 owner
- USB path↔baud↔sysid mapping
- NTRIP/RTCM source, message type/rate/CRC
- 카메라/measurement workload 활성 상태

Stock shell output이 없으면 그 사실을 명시하고 대체 evidence(USB heartbeat,
AUTOPILOT_VERSION, saved param readback)를 사용한다.

### 10.3 packet/log capture

interface 이름을 고정 가정하지 않는다. 현장에서는 MAC `58:86:94:fc:e0:7e`가 Realtek
adapter로 관측됐지만 다시 탐색한다.

예시:

```sh
ip -br link
ip -br address
ss -lunp | rg '14550|14551'
ping -c 3 10.0.0.2
ping -c 3 10.0.0.3
```

tcpdump/monitor는 다음을 구분할 수 있어야 한다.

- P1/P2 inbound packet count and last age
- message type/rate/sequence/parser error
- unicast vs broadcast duplicate
- outbound GCS HEARTBEAT source 255/component 190/rate
- outbound GPS_RTCM_DATA frame/fragment/CRC
- host interface/link/error/drop
- USB device presence와 가능하면 independent MAVLink heartbeat

새 evidence를 필요한 project artifact로 사용할 경우 `/tmp`에만 남기지 말고 secrets와
large pcap을 제외한 요약/재현 script를 repo-relative path에 보존한다. 대형 raw pcap은
machine-local provenance path와 SHA-256만 기록할 수 있다.

## 11. 실제 장비 시험 단계

### Test A — 연결과 peer 확립

1. P1/P2를 cold boot한다.
2. Workstation listener를 연다.
3. expected sysid가 일치하는지 확인한다.
4. P1/P2 각각 outbound GCS heartbeat가 1 Hz인지 capture한다.
5. 첫 heartbeat 전후 unicast/broadcast duplicate count를 비교한다.
6. requested message와 실제 rate를 기록한다.

Pass:

- identity mismatch 없음
- heartbeat rate 1 Hz, burst 없음
- parser error 0
- duplicate가 감소하거나, 감소하지 않았다면 그 사실을 기록하고 heartbeat를 해결 원인으로
  과대 해석하지 않음

### Test B — 최소 qualification gate

정상 Workstation workload와 RTCM injection을 켠 채 P1/P2를 동시에 10분 이상 감시한다.

Pass:

- 양쪽 expected heartbeat/ATTITUDE/any packet이 threshold 안에 유지
- 10분 전 READY 아님, 정확히 통과 후 READY 가능
- RTCM parser/CRC valid, live target injection failure 0
- queue가 bounded이고 지속 증가하지 않음
- UI responsive, state label layout 안정

이 test는 implementation acceptance의 최소 조건이며, 이것 하나만으로
`Pixhawk.md`에 “해결됨”을 기록하지 않는다.

### Test C — 반복 cold-boot 재현 시험

각 cycle:

1. P1/P2 cold boot 또는 승인된 side별 재부팅
2. Workstation connection/qualification 시작
3. RTCM과 정상 measurement-preflight workload 활성
4. 최소 10분 연속 감시
5. event/packet summary 저장

총 3 cycle을 수행한다. 기존 failure는 약 5분 30초에 발생했으므로 매 cycle이 이를 충분히
넘겨야 한다.

Pass: 3/3 cycle에서 all-RX stall 0, spontaneous reconnect 0, parser error 0.

### Test D — 장시간 full-load soak

P1/P2 모두 qualification을 통과한 뒤 **최소 60분 연속** 정상 Workstation 운용 부하로
감시한다.

- Workstation만 실행
- NTRIP 또는 실제 사용할 Base RTCM source 활성
- P1/P2 correction route 활성
- 실제 측정 전 telemetry/GNSS/camera workflow 활성
- calibration은 다시 하지 않아도 됨

Pass:

- P1/P2 all-RX gap 10초 이상 0회
- heartbeat와 ATTITUDE freshness breach 0회(의도된 command reservation 제외)
- live side RTCM blocked/fail 0
- unbounded queue/log/CPU growth 없음
- NIC errors/drops 0 또는 원인이 설명된 nonzero 값
- 앱 UI/worker shutdown 정상

### Test E — 한 side 장애 격리

PX4 deadlock을 일부러 유발하지 않는다. 안전한 side별 cable/power interruption 또는 test
adapter로 P1 한쪽의 inbound만 중단한다.

Pass:

- 10초 전에는 transient/age 표시, 10초 뒤 P1만 `ETH MAVLINK RX STALLED`
- P1 RTCM/commands blocked
- queue에 있던 P1 request도 send 직전 drop
- P2는 LIVE, RTCM/telemetry 계속
- host socket reconnect loop 없음
- aggregate readiness가 양쪽 정상으로 오표시되지 않음

P2에서도 대칭 test를 수행한다.

### Test F — operator-confirmed USB recovery

1. stalled side와 일치하는 USB sysid mapping을 새로 확인한다.
2. recovery action을 누르고 confirmation text를 검토한다.
3. No를 눌러 command가 없음을 먼저 확인한다.
4. 다시 실행해 Yes 후 정확한 side만 reboot되는지 확인한다.
5. 다른 side가 계속 LIVE인지 확인한다.
6. rebooted side가 `RECOVERING` → 10분 qualification → LIVE 순서인지 확인한다.
7. USB 제거/ambiguous mapping 조건에서는 manual guidance만 나오는지 확인한다.

Pass: wrong-side reboot 0, automatic reboot 0, command-send 직후 false LIVE 0.

### Test G — measurement lifecycle isolation

- measurement active와 save/finalization 중 reboot가 차단됨
- stalled side 때문에 calibration result가 삭제/reset되지 않음
- measurement 종료 뒤 안전한 recovery가 가능
- 앱 종료 시 heartbeat timer, RTCM worker, sockets가 남지 않음

## 12. “해결되었다”고 볼 수 있는 조건

다음 네 gate를 **모두** 만족해야 한다.

### Gate 1 — software correctness

- 모든 focused test 통과
- 전체 Workstation test suite 통과
- portability check 통과
- 새 thread/timer/socket leak 없음
- dirty worktree의 기존 사용자 변경을 손상시키지 않음
- coordinate/frame/measurement schema 변화 없음

### Gate 2 — mitigation behavior

- P1/P2 GCS identity/heartbeat가 capture로 확인됨
- Stock-unsupported stream 요청이 제거됨
- ATTITUDE-only stale와 all-RX stall이 정확히 구분됨
- stalled side의 RTCM/command fail-close가 자동/수동 시험으로 확인됨
- host reconnect가 remote recovery로 오작동하지 않음
- verified-USB recovery가 정확한 side와 lifecycle guard를 지킴

### Gate 3 — 현장 재현 조건에서의 안정성

- Test C cold-boot 3 cycle 모두 각 10분 이상 통과
- Test D dual-Pixhawk full-load 60분 연속 soak 통과
- 기존 약 5분 30초 failure window를 모든 run이 충분히 초과
- all-RX stall 0회
- 앱 재시작이나 Pixhawk 추가 reboot 없이 지속
- RTCM active 상태에서도 live injection error 0

### Gate 4 — 실패/복구의 진실성

- 의도적 한-side outage가 10초 내외에 정확히 탐지됨
- 다른 side가 영향을 받지 않음
- blocked RTCM evidence가 남음
- recovery 뒤 10분 qualification 전 READY가 되지 않음
- 어떤 state도 ping/socket 존재만으로 LIVE를 주장하지 않음

네 gate를 통과하면 다음 문구 수준으로만 결론 낸다.

> Stock PX4에서 재현되던 약 5분 30초 후 Ethernet MAVLink 수신 정지는 Workstation의
> GCS heartbeat, Stock-compatible stream profile, stalled-target fail-close 및 안전한 복구
> 적용 후 정의된 반복/60분 현장 시험에서 재발하지 않았다. PX4/NuttX UDP receiver
> deadlock은 증상과 공식 issue에 근거한 고신뢰 원인 추정이며 firmware 자체를 patch한
> 것은 아니다.

다음 표현은 금지한다.

- `PX4 firmware bug 완전 수정`
- `무기한 재발하지 않음`
- `GCS heartbeat 하나가 원인을 제거함` — A/B evidence 없이는 금지
- `net.cfg/extras.txt 수정으로 runtime deadlock 해결`

## 13. `Pixhawk.md` 기록 조건과 내용

12절의 모든 gate가 통과한 뒤에만 `Pixhawk.md`를 수정한다. 기록에는 다음을 포함한다.

1. 날짜와 실제 firmware version/git hash/board target
2. P1/P2 IP, UDP port, sysid, relevant `MAV_2_*` readback
3. 변경 전 failure timeline: reboot-only recovery와 약 5.5분 재발
4. host NIC/ping/socket/USB가 살아 있었다는 분리 evidence
5. RTCM parse/CRC가 정상이라는 반증 evidence
6. 공식 PX4 issue #25956 링크와 `고신뢰 추정` 표시
7. 실제 적용한 Workstation 변경
8. 요청 stream before/after와 packet rate/duplicate evidence
9. 3회 cold boot 및 60분 soak의 정확한 결과
10. 한-side RTCM fail-close와 USB recovery 결과
11. `MAV_2_RATE`/firmware/config를 변경했는지 여부
12. firmware defect가 open이고 Workstation은 mitigation이라는 잔여 한계
13. operator SOP: STALLED 표시, RTCM 차단, verified-USB/manual reboot, 10분 qualification
14. coordinate contract가 변하지 않았다는 명시

hardware gate가 실패하면 `Pixhawk.md`에 해결 방법을 확정 기록하지 않는다. 실패 결과는
이 change packet의 `verification.md`와 evidence artifact에만 남긴다.

## 14. 실패 시 진단 분기

### 14.1 같은 all-RX stall이 재발하면

즉시 다음을 보존한다.

- 정확한 P1/P2 last packet 시각과 순서
- before/after 60초 pcap 또는 packet summary
- host NIC/link/IP/ping/socket owner
- outbound GCS heartbeat와 RTCM count
- USB MAVLink 생존 여부
- stream rate와 queue depth
- actual firmware/version/params
- app worker/thread 상태와 error logs

앱을 먼저 재시작해 evidence를 덮지 않는다. 장비 안전이 허용하면 USB channel에서
`mavlink status`, task/stack/IOB 관련 정보를 시도한다. output이 없으면 그 사실도 기록한다.

### 14.2 한 변수씩 A/B

다음 순서로 한 번에 하나만 바꾼다.

1. RTCM off, 동일 telemetry profile
2. RTCM on, diagnostic stream off/minimal core profile
3. P1만/ P2만 단독
4. `MAV_2_RATE` lower ceiling — 별도 승인과 parameter backup 필요
5. 다른 physical switch/hub/cable/NIC
6. 실제 Stock version 차이(1.14.3 vs pinned 1.17.0) — reflash는 별도 승인

각 run은 기존 failure window보다 긴 최소 10분을 유지한다. 여러 변수를 동시에 바꾸지 않는다.

### 14.3 Workstation 완화로 해결되지 않으면

- packet의 상태를 `blocked`가 아니라 우선 `verifying/failed evidence`로 유지하고 의미 있는
  A/B를 계속할 수 있는지 판단한다.
- Stock-only 요구와 official open defect가 양립하지 않으면 사용자에게 firmware-level
  한계를 명확히 보고한다.
- 다음 선택은 별도 approval packet으로 다룬다.
  - PX4/NuttX receive/send buffer accounting patch
  - UDP socket nonblocking/timeout patch
  - official upstream fixed release로 migration
  - 독립 serial telemetry radio fallback
- telemetry radio는 ETH deadlock을 예방하지 않으며, 별도 link health/identity/routing 계약이
  필요하다.

## 15. 변경 예정 파일과 보존 범위

### 핵심 owner

- `workstation_core/pixhawk_attitude.py`
- `device_runtime_controller.py`
- `device_runtime_workers.py`
- `gnss_runtime_controller.py`
- `status_model.py`
- 기존 status/recovery surface를 소유한 `workspaces.py`/`standalone_gui.py`

### 집중 테스트

- `tests/test_workstation_pixhawk_gnss.py`
- `tests/test_rtcm_pipeline_workers.py`
- `tests/test_rtcm_pipeline_end_to_end.py`
- `tests/test_status_model_pixhawk.py`
- `tests/test_workstation_ntrip.py`
- `tests/test_workstation_lifecycle.py`
- `tests/test_workstation_ui.py`

### 제품 문서

- `README.md`
- `docs/ui/01_USER_WORKFLOW.md`
- `docs/ui/06_STATUS_MODEL.md`
- `docs/ui/07_COMMAND_MODEL.md`
- `docs/ui/09_IMPLEMENTATION_PLAN.md`
- `Pixhawk.md` — hardware resolution gate 뒤에만 수정

### 절대 건드리지 않을 범위

- user calibration files와 calibration 결과
- measurement math/coordinate transform/schema
- 기존 evidence/session data migration 또는 삭제
- StereoWIRIS source
- PX4 firmware/config/parameter persistence, 별도 승인 전까지
- 관련 없는 dirty worktree 변경

## 16. coordinate contract review

이 변경은 MAVLink transport liveness, command eligibility, RTCM routing eligibility, UI status만
수정한다.

- PX4 ATTITUDE의 body-FRD/world-NED 의미를 변경하지 않는다.
- GPS WGS84 latitude/longitude/altitude field 의미를 변경하지 않는다.
- RTCM은 좌표 변환 없이 complete binary correction frame으로 전달한다.
- left/right는 device side identity이며 camera coordinate frame 값을 이동시키지 않는다.
- `x/y/z`, NED/ENU, optical RDF, stereo-center field를 추가·변환하지 않는다.

구현 중 telemetry payload나 기록 schema를 건드릴 필요가 생기면 현재 approval 범위를 벗어난
것이므로 즉시 중단하고 별도 coordinate review와 승인을 받는다.

## 17. compact/resume checkpoint

- Goal: Stock PX4를 유지하면서 P1/P2 ETH MAVLink runtime stall을 완화·탐지·격리하고,
  verified-USB operator recovery와 10분 qualification을 구현·검증한다.
- Latest evidence:
  - P1/P2는 reboot 후 ETH-only 조건에서도 약 5분 30초 뒤 all-RX STALE로 재현됐다.
  - host NIC/IP/ping/socket과 첫 failure 당시 USB MAVLink는 살아 있었다.
  - RTCM 48/48 MAVLink parse/CRC가 valid했고 app/socket restart는 복구하지 못했다.
  - official PX4 issue #25956과 증상이 일치하나 onboard stack trace는 없다.
- In-progress work: source 미수정; change packet과 이 상세 plan만 작성됨.
- Pending next steps:
  1. actual firmware/version과 dirty worktree 재확인
  2. recovery policy 1 포함 scope 명시 승인
  3. Slice 1 pure liveness model부터 구현
  4. 자동/전체 test
  5. P1/P2 3-cycle + 60분 hardware validation
  6. 성공한 경우에만 `Pixhawk.md` 기록
- Blockers/Risks:
  - recovery policy가 아직 명시 승인되지 않음
  - actual board가 1.14.3인지 repo의 1.17.0인지 새로 확인 필요
  - upstream PX4/NuttX issue가 open이라 Workstation만으로 firmware defect 제거 보장 불가
  - worktree가 매우 dirty하므로 기존 변경 보존 필수
  - `/tmp` raw logs는 machine-local; 본 문서가 portable evidence summary임
- Background workflows: ETH monitor는 종료됐고 active background monitor 없음.
- Resume-first action: **이 plan을 읽은 뒤 actual P1/P2 firmware를 read-only로 확인하고,
  recovery policy 1을 포함한 packet-scoped approval을 사용자에게 받아라.**
