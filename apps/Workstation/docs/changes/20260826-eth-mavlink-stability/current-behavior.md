# Current Behavior

## Version and worktree context

- Baseline commit: `c4397995d864576c966702cfd3c398b6032f8990`.
- Worktree에는 본 변경 이전부터 Workstation/StereoWIRIS 관련 수정이 다수 존재한다.
  구현은 그 변경을 보존하며 ETH 관련 owner만 좁게 수정해야 한다.
- Runtime premise: P1/P2는 PX4 Stock Stable 1.14.3, Ethernet은
  `10.0.0.2:14550`/`10.0.0.3:14551`, Workstation만 UDP port를 소유한다.
- 2026-08-26 16:32 KST 재개 점검에서는 P1/P2 ping이 모두 정상이고 host UDP 14550/14551
  owner가 없었지만, passive listener와 direct GCS/`AUTOPILOT_VERSION` request 모두 6초간
  heartbeat를 받지 못했다. USB serial 장치도 없어 actual firmware 재확인은 불가능했다.
  이 결과는 v1.17.0 설치 증거가 아니라 현재 ETH MAVLink silence evidence로 보존한다.

## Reproduction

- P1/P2 reboot 및 hub/NIC reconnect 뒤 양쪽 모두 LIVE가 되었다.
- 첫 정상 구간에서 heartbeat 1 Hz, GPS 약 5 Hz, ATTITUDE P1 약 15 Hz/P2 약 30 Hz였고
  parser error는 0이었다.
- 첫 drop은 P1 `14:42:03.079`, P2 `14:42:05.749`에 발생했다.
- 이후 USB가 제거된 조건에서도 P1 `15:13:30.887`, P2 `15:13:31.095`에 복구된 뒤
  P1 `15:19:03.583`, P2 `15:19:09.165`에 다시 멎었다. 즉 재부팅 후 약 5분 30초다.
- drop 중 host NIC/link/IP/ping/socket owner는 정상이었고, USB가 연결된 첫 drop에서는
  USB MAVLink가 즉시 살아 있었다. 앱 재시작과 host socket 재연결로는 회복되지 않았다.
- 보존된 raw evidence:
  `/tmp/sasl_eth_mav_dynamic_monitor_20260826T143716KST.jsonl`,
  `/tmp/sasl_eth_mav_monitor_p1_config_20260826T141016KST.jsonl`,
  `/tmp/sasl_eth_mav_monitor_20260826T125213KST.jsonl`.

## 2026-08-26 20:05–20:30 KST resumed hardware evidence

- 재인가 전 P1/P2는 각각 ping 2/2에 응답했지만 passive capture와 1 Hz GCS heartbeat plus
  read-only `AUTOPILOT_VERSION` request에 10초간 inbound UDP datagram을 하나도 반환하지
  않았다. Wire에는 host outbound 26 packet만 보였다.
- 첫 operator power reapply 뒤 P1은 `20:11:15`, P2는 `20:11:43`부터 다시 broadcast
  telemetry를 보냈다. 실제 `DualPixhawkMavlinkSubscriber`를 사용한 headless observation은
  `20:14:24.341`–`20:17:52.158` 동안 P1 29,066 packet, P2 28,882 packet을 보존했다.
  이 208초 capture는 양쪽 heartbeat 1 Hz, ATTITUDE 약 30 Hz, GPS1+GPS2 약 10 Hz,
  parser error 0을 확인했지만 5분 acceptance에는 짧다.
- 같은 headless observer가 UDP 14550/14551을 소유한 동안 나중에 실행한 Workstation의
  Connect가 bind되지 않았다. Observer를 종료하자 포트는 해제됐고 실제 Workstation PID가
  두 포트를 소유했으므로, 이 Connect 무반응은 별도 local-owner collision이었다.
- 그 뒤 P1/P2 telemetry가 다시 0이 됐고 실제 앱의 host socket 재생성으로 회복되지 않았다.
  앱 diagnostics에는 잠시 `PARAM SYNC`가 보였지만 지속 inbound packet evidence는 없었다.
  첫 verified heartbeat 직후 자동 QGC calibration PARAM 묶음을 읽는 현재 경로는 새 load
  trigger 후보이며, causal A/B는 아직 `NOT_RUN`이다.
- 두 번째 operator power reapply는 기존 앱이 두 listener를 유지한 상태에서 수행됐지만 새
  identity가 관측되지 않았다. 앱을 `20:26:06`에 새 PID로 다시 실행해 두 포트를 재점유한
  뒤에도 `CONNECTING`, `sysid=None`, `HB/ATT/ANY=--`가 유지됐다.
- 독립 capture `/tmp/sasl_eth_mav_app_soak_20260826T202512KST.pcap`은
  `20:25:12`–`20:29:53` 동안 0 packet인 24-byte pcap header만 남겼다
  (`sha256=704e5e5b3234433c01fcfd1b20a306e77e985038120492dc53965c3edd38a4ea`).
  같은 동안 P1/P2 ping은 계속 정상이다. 이는 runtime mitigation을 평가하기 전 단계의
  boot-time Ethernet MAVLink instance/non-emission failure로 분류하며, USB read-only
  firmware/sysid/`MAV_2_*`/가능한 `mavlink status` 확인이 다음 독립 oracle이다.

## Observed frontend behavior

- UI는 ATTITUDE age만 보고 `LIVE`/`STALE`을 결정한다.
- heartbeat/any packet 전체가 끊긴 상태와 ATTITUDE stream만 끊긴 상태가 같은 `STALE`로
  보일 수 있다.
- stale side를 다시 연결하면 host socket만 close/open하며, PX4-side task가 막힌 경우에도
  복구를 시도한 것처럼 보인다.
- operator에게 RTCM이 해당 side로 계속 전송 중인지, 중단되었는지, 재부팅이 필요한지
  명확한 조치가 없다.

## Observed backend and persistence behavior

- `DualPixhawkMavlinkSubscriber.start()`는 `source_system`/`source_component`를 명시하지
  않고 `mavlink_connection()`을 만든다. 현재 host pymavlink signature의 default는
  `source_system=255`, `source_component=0`이므로 source system 자체가 0인 것은 아니다.
  제안한 255/190 명시는 identity contract를 deterministic하게 만드는 변경이며, 그것만으로
  runtime stall이 해결된다고 보지 않는다.
- 상시 GCS heartbeat 송신이 없다. 임시 USB heartbeat는 probe path에만 존재한다.
- 첫 remote heartbeat 후 16개 stream을 요청하며, Stock에 없는 private
  `SASL_CAN_MAG`도 최대 3회 재요청한다.
- RTCM parser는 완전한 RTCM3 frame만 전달한다. 관측된 48/48 MAVLink packet과 RTCM CRC가
  유효했고 모든 frame이 180 bytes 미만이라 fragmentation 오류는 원인에서 배제된다.
- incoming telemetry가 멎어도 `PixhawkRtcmInjectionWorker`는 기존 connection object로
  `GPS_RTCM_DATA`를 계속 보낸다.
- stale reconnect는 host listener를 재생성하지만 remote PX4 process를 복구할 수 없다.
- P1 early `/fs/microsd/etc/config.txt`는 부팅 시 interface start를 고쳤지만 runtime drop은
  동일하게 재현되었다. `MAV_2_CONFIG=1000`, `MAV_2_MODE=0`, broadcast enabled,
  `MAV_2_RATE=2000000`, ports 14550/14551가 저장돼 있다.
- 조사 boot evidence는 양쪽 실제 장비를 PX4 1.14.3으로 식별하지만, 현재 untracked
  `firmware/stock/` bundle은 v1.17.0이다. bundle 존재를 실제 flash evidence로 간주하면
  안 되며 hardware test 시작 전 actual version을 다시 확인해야 한다.

## Documentation, code, test, and runtime conflicts

- `Pixhawk.md`의 "ping은 되지만 UDP가 없으면 instance가 실행되지 않음" 설명은 runtime
  deadlock 가능성을 포함하지 않아 현재 evidence와 충돌한다.
- `docs/ui/06_STATUS_MODEL.md`는 generic `STALE`만 정의하고 ETH receiver stall을 구분하지 않는다.
- 기존 테스트는 ATTITUDE stale와 reconnect를 검증하지만 any-packet silence, per-side RTCM
  fail-close, 10분 qualification, periodic GCS heartbeat를 검증하지 않는다.
- 공식 PX4 issue `#25956`은 high UDP traffic에서 NuttX readahead queue/IOB 고갈로 blocking
  `sendto()`가 멈추며 restart만 복구하는 동일 계열 증상을 기록한다. 현재 장비의 stack trace는
  Stock SERIAL_CONTROL shell 제약으로 얻지 못했으므로 "고신뢰 추정"으로 표현해야 한다.
