# P1 Ethernet RX 장애 진단 기록

- 기록일: 2026-08-31 (KST)
- 대상: P1 / Left Pixhawk
- 상태: **P1 보드 측 half-only link/external RX 장애 경계 확정, 세부 부품 미분리, ETH 미복구**
- 금지된 결론: `LAN8742A 단일 부품 손상 확정`, `firmware 재설치로 해결`,
  `P1 LIVE이므로 양방향 정상`

## 1. 결론 요약

P1 Ethernet는 현재 비대칭이다.

- **P1 → Workstation 송신:** 안정적으로 동작한다.
- **Workstation → P1 수신:** ARP, GCS heartbeat, MAVLink command, RTCM이 동작하지 않는다.

따라서 Workstation의 `P1 LIVE`는 P1이 보내는 telemetry가 fresh하다는 뜻일 뿐, P1이
Workstation의 명령이나 RTCM을 받고 있다는 뜻이 아니다.

장애는 케이블/스위치 포트, 설치 이미지 손상, Workstation decoder/UI보다 P1 보드 측
Ethernet RX 경계로 좁혀졌다. 추가 진단으로 다음 **직접 고장 메커니즘**은 확인되었다.

1. P1 Ethernet MAVLink receiver가 최초 상대를 Workstation `10.0.0.10`이 아니라 자기 주소
   `10.0.0.2`로 고정한다.
2. 이후 PX4는 같은 MAVLink payload를 자기 주소 unicast와 `10.0.0.255` broadcast로 각각
   한 번씩 송신한다.
3. P1은 두 복사본을 모두 자기 RX로 되받아 거의 모든 MAVLink sequence를 정확히 두 번
   처리한다.
4. 외부에서 들어오는 ARP, GCS heartbeat/command, RTCM은 계속 처리하지 못한다.

즉 `self-partner -> 이중 송신 -> 이중 self-RX` 되먹임은 확인되었다. 2026-08-31
17:56–19:25 KST 추가 실기 A/B에서는 현재 P1 연결 링크가 auto-negotiation 상태에서도
`100 Mbps/half-duplex`로만 형성됐고, host가 `100/full` 또는 `10/full`만 광고하면
carrier가 내려갔다. Ethernet MAVLink instance를 완전히 정지해 self traffic을 제거해도
외부 ARP는 `5 request / 0 reply`였다.

따라서 최초 원인은 self-RX/IOB 부하가 아니라 **P1 보드 측 PHY/RMII/connector의 외부 RX
경로와 half-only PHY mode/advertisement 경계**다. LAN8742A MODE strap, PHY register 상태,
RXP/RXN·magnetics, RMII 신호 중 정확한 불량 부품은 아직 분리하지 않았다.

## 2. 장비와 firmware identity

| 항목 | 값 |
|---|---|
| Side | P1 / Left |
| Hardware | PX4_FMU_V6X / V6X |
| Bootloader board ID | 53, revision 0 |
| PX4 | official Stock v1.17.0 |
| PX4 git hash | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| NuttX | 11.0.0, `fb2fadf6f599c1406f052db013efd00a2518e72c` |
| PX4 GUID | `000600000000363633383433510600210026` |
| MAVLink identity | sysid/component `1/1` |
| Ethernet IP/port | `10.0.0.2/24`, UDP `14550` |
| Ethernet MAC | `1e:1e:b3:a4:ff:b5` |
| Workstation IP/MAC | `10.0.0.10`, `58:86:94:fc:e0:7e` |

## 3. 동작하는 경로

### 3.1 P1 → Workstation ETH telemetry

공식 firmware 재설치 후 619.397초 독립 pcap 결과:

| Message | Count | Rate | Maximum gap |
|---|---:|---:|---:|
| HEARTBEAT | 619 | 1.000 Hz | 1.0037 s |
| ATTITUDE | 9,292 | 15.000 Hz | 0.0715 s |
| GPS_RAW_INT | 3,097 | 5.000 Hz | 0.2044 s |
| GPS2_RAW | 620 | 1.000 Hz | 1.0029 s |

- MAVLink2 raw sequence: 32,034 packets
- duplicate/missing/reordered: `0/0/0`
- tcpdump kernel drop: `0`
- 21개 UI checkpoint: P1/P2 모두 `LIVE`, all-RX stall `0`
- P1 ATTITUDE 표시 age 최대 약 `0.06 s`
- GNSS message 표시 age 최대 약 `0.21 s`

GNSS transport는 정상이나 해는 유효하지 않았다. 실내 시험 조건에서 GPS1/GPS2 모두
`fix_type=0`, satellites `0`, lat/lon `0`이므로 UI의 `NO FIX`는 예상된 상태다.

### 3.2 USB control → P1 ETH telemetry

USB shell에서 다음 runtime-only stream을 활성화하면 데이터는 ETH로 정상 수신된다.

```text
mavlink stream -u 14550 -s SCALED_IMU  -r 20
mavlink stream -u 14550 -s SCALED_IMU2 -r 20
mavlink stream -u 14550 -s SCALED_IMU3 -r 20
```

최종 4초 capture에서 `SCALED_IMU/2/3=62/62/62`, ATTITUDE `46`, HEARTBEAT `3`이었다.
이 명령은 재부팅하면 사라지므로 USB-free 지속 복구가 아니다.

## 4. 실패하는 경로

### 4.1 ARP와 ping

- PC의 P1 ARP request에 P1 reply `0`
- PC → P1 ping: 100% loss
- host neighbor: `10.0.0.2 INCOMPLETE/FAILED`
- P1-originated ping: `sendto failed at seqno 0: errno 114`
- P1은 PC를 위한 ARP request를 송신했고 PC는 P1 MAC으로 reply했지만, P1 ARP table에는
  PC entry가 만들어지지 않았다.

### 4.2 Ethernet MAVLink command

host에 P1 정적 neighbor를 임시 적용해 ARP를 우회했을 때:

- Workstation GCS heartbeat `255/190`: wire 송신 확인
- 15-command Stock profile: wire 송신 확인
- bounded `SCALED_IMU/2/3` retry: wire 송신 확인
- P1 `COMMAND_ACK`: `0`
- P1 automatic `SCALED_IMU/2/3`: `0/0/0`

P1 USB `mavlink status`의 Ethernet instance는 외부 GCS가 아니라 자기
`sysid=1/component=1`만 수신했다고 보고했다.

### 4.3 RTCM

NTRIP input은 `RTCM_ACTIVE`였지만 10초 wire capture 결과는 다음과 같다.

| Target | GPS_RTCM_DATA |
|---|---:|
| P1 (`10.0.0.2:14550`) | 0 |
| P2 (`10.0.0.3:14551`) | 43 |

따라서 현재 P1에는 RTCM correction이 전달되지 않는다. Workstation의 과거 누적 카운터나
`P1 LIVE` 상태는 wire delivery 증거로 사용할 수 없다.

### 4.4 현재 self-partner 및 이중 수신 재확인

2026-08-31 최신 read-only 재확인에서도 상태는 변하지 않았다.

| 항목 | 최신 값 |
|---|---:|
| Ethernet MAVLink TX | `2,176.4 B/s` |
| Ethernet MAVLink RX | `4,352.9 B/s` |
| RX source | 자기 `sysid=1/component=1`만 존재 |
| Received / reported lost | `553,625 / 70,234,621` |
| Reported RX loss | `126.9%` |
| Self ping RTT | `0.89 ms` |
| Host ARP neighbor | `INCOMPLETE` |
| Host ping | `3 sent / 0 received` |

같은 boot의 PX4 로그에는 `partner IP: 10.0.0.2`가 기록되어 있다. 정확한 PX4 v1.17
receiver는 UDP `recvfrom()`의 source IP를 partner로 저장하고, sender는 partner가 설정되면
그 주소로 unicast한다. 외부 GCS가 연결되지 않은 상태에서 broadcast가 활성화되어 있으면
같은 buffer를 broadcast로도 추가 송신한다.

host에서 관측되는 P1 broadcast 120개의 IPv4 ID는 `119/119` 전이가 모두 정확히 `+2`였다.
반면 P2 비교 capture 200개는 `+1`이 197회, `+2`가 2회였다. P1에서 host에 보이는 각
broadcast 사이에 host에는 보이지 않는 자기 unicast IP packet이 하나씩 생성된다는 뜻이다.

PX4 v1.17의 component sequence 통계는 같은 sequence가 연속으로 들어오면 두 번째
복사본을 `254` missed로 계산한다. 최신 누적값은 다음과 같이 정확한 2중 수신 기대값에
가깝다.

```text
70,234,621 / 553,625 = 126.863 lost per received
70,234,621 / 254     = 276,514 estimated second copies
276,514 / 553,625    = 49.946% second-copy fraction
ideal two-copy stream: 50% second copies, 254 / 2 = 127.0
```

따라서 `126.9%`는 실제 wire packet-loss 백분율이 아니라 거의 모든 메시지를 두 번 받은
결과다. 독립적인 IPv4 ID 규칙과 MAVLink sequence 통계가 같은 결론을 지지한다.

```text
최초 P1 broadcast가 P1 RX로 되돌아옴
  -> PX4 partner가 10.0.0.2로 고정됨
  -> 매 payload를 self-unicast + broadcast로 송신
  -> P1이 두 복사본을 모두 수신
  -> duplicate 통계와 네트워크 부하 증가

동시에 외부 ARP/GCS/RTCM frame은 P1 protocol stack까지 도달하지 않음
```

### 4.5 2026-08-31 P1-only link duplex 및 self-load 제거 A/B

17:56–19:25 KST에 P1 USB `/dev/ttyACM0`와 host NIC `enx588694fce07e`를 사용해
한 변수씩 바꾸는 boot-local A/B를 수행했다. P1 identity, IP, parameter, firmware 및 파일은
변경하지 않았다.

기준 readback:

| 항목 | 값 |
|---|---|
| Host link | auto-negotiation `yes`, `100 Mbps`, `half-duplex` |
| P1 interface | `10.0.0.2/24`, router `10.0.0.10`, MAC `1e:1e:b3:a4:ff:b5` |
| P1 Ethernet MAVLink | UDP `14550`, broadcast enabled |
| P1 Ethernet TX/RX | 약 `2.4–2.6 / 4.8–5.2 kB/s` |
| P1 RX source | 자기 `sysid=1/component=1`만 존재 |
| P1 reported loss | 약 `126.8%` |
| P1 ARP table | 자기 `10.0.0.2`만 존재, host `10.0.0.10` 없음 |

Host NetworkManager 원본 profile을 복제한 임시 profile로 full-duplex만 광고하고, 시험 뒤
항상 원본 `pixhawk-dds`를 복원했다.

| A/B | 관측 결과 | 판정 |
|---|---|---|
| 원본 auto-negotiation | carrier `1`, `100/half` | half-duplex link만 형성 |
| host `100/full` only | carrier `0` | 100/full 공통 mode 없음 |
| host `10/full` only | carrier `0` | 10/full 공통 mode 없음 |
| 원본 profile 복원 | carrier `1`, `100/half` | 원상 복구 확인 |
| host carrier `0` 동안 USB `mavlink status` | self RX와 약 2× RX/TX 비율 지속 | 중간 switch/host 반사보다 P1 내부 경계 |
| `mavlink stop -u 14550` 후 host ARP | request `5`, reply `0` | self traffic/IOB 부하는 최초 원인 아님 |
| 원래 Ethernet instance 재시작 | 즉시 `partner IP: 10.0.0.2` | self-partner 재현 |

MAVLink instance 정지 중에는 다음 wire evidence만 존재했다.

```text
58:86:94:fc:e0:7e > ff:ff:ff:ff:ff:ff
ARP Request who-has 10.0.0.2 tell 10.0.0.10
5 requests, 0 replies
```

시험 후 다음 원래 runtime command로 Ethernet instance를 복원했다.

```text
mavlink start -x -u 14550 -o 14550 -r 2000000 -p -n eth0
```

복원 뒤 P1의 self-unicast+broadcast 쌍, Workstation UDP `14550` owner, host
`10.0.0.2 INCOMPLETE`, `100/half` link를 다시 확인했다. 영구 NetworkManager/PX4 변경은
남기지 않았다.

## 5. 반증 또는 분리된 원인

### 5.1 Workstation parser/UI

반증됨. P2는 동일 subscriber/UI에서 `SCALED_IMU/2/3`를 정상 수신하고, P1도 USB로
stream을 활성화한 뒤 동일 ETH parser/UI에서 정상 표시된다.

### 5.2 케이블 또는 스위치 포트

반증됨. P1/P2 Ethernet 케이블/포트를 서로 교환한 180초 capture에서:

- P1 ARP request/reply: `147/0`
- P2 ARP request/reply: `5/5`

장애는 케이블/포트를 따라가지 않고 P1 identity/보드 쪽을 따라갔다.

### 5.3 설치 이미지 손상

반증됨. P1을 repository-pinned official Stock v1.17.0 FMUv6X 이미지로 재설치했다.

- firmware SHA-256: `8f2c42787163b71e00758205ab0bed9df1669fd26d7a9f42d6f995cbabba0d7f`
- bootloader가 board ID 53, STM32H7 rev V, flash size를 확인
- erase/program/verify/reboot 모두 PASS
- force board/force erase 미사용
- sysid, GUID, `MAV_2_*`, netman, extras, parameter storage 보존

재설치 후에도 P1 ARP `90/0`, USB 활성화 전 89.049초 동안 heartbeat `87`, ATTITUDE
`1,299`, `SCALED_IMU/2/3=0/0/0`이었다.

### 5.4 공통 firmware 또는 핵심 MAVLink parameter

반증됨. P2는 같은 FMUv6X, 같은 PX4 git hash, 다음 동일 설정으로 정상 양방향 통신한다.

```text
MAV_2_CONFIG=1000
MAV_2_MODE=0
MAV_2_BROADCAST=1
MAV_2_FLOW_CTRL=2
MAV_2_FORWARD=0
MAV_2_RADIO_CTL=0
MAV_2_RATE=2000000
```

P2 Ethernet MAVLink RX는 외부 GCS `255/190`, 약 38.9 B/s, reported loss 0.1%였다.
P1은 자기 sysid만 약 4.3 KB/s로 되받고 reported loss 126.9%였다.

### 5.5 정상 NuttX software loopback

정상 software loopback만으로는 현재 현상을 설명할 수 없다.

- 정확한 NuttX revision의 `devif_loopback()`은 destination IP가 interface 자신의 IP와
  정확히 같은 packet만 처리하며 broadcast는 대상이 아니다.
- FMUv6X가 사용하는 STM32H7 Ethernet driver는 다른 일부 NuttX driver와 달리
  `devif_loopback()`을 TX 경로에서 호출하지 않는다.
- 따라서 최초 `10.0.0.255` broadcast가 자기 UDP receiver로 돌아온 현상은 정상 generic
  NuttX broadcast loopback이 아니다.
- Workstation capture에는 P1 packet duplicate가 없고 MAVLink sequence도 연속이었다.
  되먹임은 host parser가 만든 것이 아니라 P1 내부/PHY/MAC RX 경계에서 발생한다.

## 6. 확정 경계와 세부 원인 우선순위

| 우선순위 | 예상 원인 | 근거 | 현재 판단 |
|---:|---|---|---|
| 1 | P1 LAN8742A half-only MODE strap/register와 외부 RX 경로 이상 | auto는 `100/half`만 형성되고 `100/full`, `10/full` only에서는 carrier `0`; 외부 ARP/GCS/RTCM은 0 | **보드 측 경계 확정, 세부 부품 미분리** |
| 2 | RXP/RXN·magnetics·connector 또는 RMII RX/CRS_DV 이상 | self frame은 RX로 복귀하지만 self load를 없애도 외부 ARP reply `0` | **높음** |
| 3 | self-RX가 유발한 NuttX IOB/UDP readahead 압박 | 모든 payload가 두 번 RX되지만 instance 정지 중에도 ARP 실패 | **증폭 요인, 최초 원인에서 제외** |
| 4 | 중복 `ifup`/여러 `netinit`이 만든 runtime 상태 | P1/P2 모두 유사 현상이 있고 self partner가 extras `ifup` 완료 전에 기록됨 | **낮음** |
| 5 | cable/switch port, 공통 firmware/parameter, Workstation parser | cable/port swap, 동일 firmware P2, host pcap으로 반증 | **주원인에서 제외** |

LAN8742A는 정상적으로 10/100 full-duplex를 지원하며 auto-negotiation 우선순위도
`100/full -> 100/half -> 10/full -> 10/half`다. MODE strap `111`은 all-capable을
광고하지만 `100`/`101`은 100-half만 광고한다. 현재 host full-only A/B 결과는 P1이
half-only capability를 광고하거나 동일한 PHY register 상태에 있다는 행동 증거다.

정확한 STM32H7 driver는 MAC loopback bit를 초기화 시 clear한다. 대신 `Receive Own Disable`
bit를 0으로 두므로 half-duplex에서 자기 TX가 PHY/RMII RX로 돌아오면 MAC이 이를 받을 수
있다. 이것이 self-partner 되먹임을 설명하지만, Ethernet instance 정지 뒤에도 외부 ARP가
실패했으므로 되먹임 자체는 외부 RX 장애의 최초 원인이 아니다.

LAN8742A의 정식 near-end loopback mode에서는 line transmitter가 꺼진다. Workstation은 P1
broadcast를 계속 수신하므로 단순히 PHY test-loopback bit만 켜진 상태는 현재 증상과 맞지
않는다. 비정상 half-duplex receive-own, RMII/PHY 전기적 echo, 외부 RX analog/connector
장애를 우선 구분해야 한다.

### 6.1 전역 resource 고갈이 주원인일 가능성이 낮아진 이유

최신 read-only runtime에서 다음 상태를 확인했다.

- `arp -s`/`arp -a` ioctl이 `Out of memory`를 반환
- P1 전체 heap은 `497,264 B` free, largest block `420,608 B`로 전역 heap 고갈은 아님
- DMA memory는 1,024/5,120 bytes 사용
- official v1.17 FMUv6X NuttX defconfig: `CONFIG_IOB_NBUFFERS=24`
- CPU는 `55.51% idle`, Ethernet MAVLink receiver는 약 `0.49%` 사용
- MAVLink `send_bytes error`, forwarding error, STATUSTEXT missed는 모두 `0`
- P1과 P2 모두 장시간 후 여러 `netinit` task 및 ARP ioctl ENOMEM을 보일 수 있음

NuttX `arp -a` helper는 조회 전에 새 socket을 열므로 이 `ENOMEM`은 ARP table 자체 고갈의
독립 증거가 아니다. P2에서도 같은 결과가 있었고 P1의 기존 UDP socket은 계속 송수신한다.
또한 ARP는 UDP command handler보다 아래 계층에서 처리되므로 UDP readahead 문제만으로
외부 ARP request/reply가 모두 사라지는 현상을 설명하기 어렵다.

추가로 Ethernet MAVLink instance를 완전히 정지해 P1 self network load를 제거한 상태에서도
host ARP request `5`개에 reply가 하나도 없었다. 이 결과로 IOB/UDP 부하는 최초 원인 후보에서
제외한다.

### 6.2 보드 측 경계는 확정됐지만 세부 부품은 미분리인 이유

현재 증거는 board-side PHY/RMII external RX/half-only 경계를 확정하지만 다음 상태는 직접
읽지 못했다.

1. LAN8742A `reg 4` advertisement와 `reg 31` negotiated mode
2. MODE[2:0] strap이 실제로 `100`/`101`로 잘못 latch됐는지
3. STM32H7 MACCR 및 RX/DMA error/collision counter
4. RXP/RXN, magnetics, connector, RMII RXD/CRS_DV 중 어느 부품/신호가 불량인지

따라서 `P1 board-side external RX/half-only boundary`는 확정하지만 `LAN8742A die 단독
손상`처럼 세부 부품까지 단정하지 않는다. 세부 판별에는 MDIO register readback 또는
보드-level 계측/P2 비교가 필요하다.

## 7. 현재 운용 영향

- ATTITUDE/GNSS telemetry 관찰: 가능
- USB로 활성화한 IMU telemetry를 ETH로 관찰: 가능
- USB 없이 reboot 후 IMU stream 활성화: 불가능
- P1 RTCM 주입: 불가능
- P1 ETH parameter/shell/reboot/mission command: 불가능
- P2 RTCM/command/telemetry: 정상

## 8. 다음 진단 순서

한 번에 한 변수만 바꾸며 다음 순서로 탐색한다.

### 8.1 설정·firmware 변경 없는 판별 결과

1. P1-only host link readback: auto-negotiation `yes`, `100/half`.
2. `100/full` 및 `10/full` only 광고: 모두 carrier `0`.
3. self traffic 제거 후 ARP: `5 request / 0 reply`.
4. 원복 후 self-partner 및 이중 packet: 즉시 재현.
5. 기존 P1/P2 cable/port swap: 장애가 P1을 따라감.

따라서 switch/topology, self-load, host parser보다 P1 board PHY/RMII 경계가 우선이다. 전원
전체를 10초 이상 제거한 cold-boot 첫 packet capture는 세부 strap latch 재현성을 확인하는
추가 시험으로 남는다.

### 8.2 PHY/MAC register 판별

현재 stock NSH에는 MII read 명령이 없으므로 managed switch readback 또는 별도 승인된
diagnostic helper가 필요하다. 다음 값을 P1/P2에서 같은 boot phase에 비교한다.

| Register | 확인할 bit/값 | 판별 목적 |
|---|---|---|
| LAN8742A Basic Control `reg 0` | loopback bit 14, autoneg bit 12 | PHY test-loopback/협상 상태 |
| LAN8742A Advertisement `reg 4` | bit 8/7/6/5 (`100FD/100HD/10FD/10HD`) | half-only advertisement 원인 판별 |
| LAN8742A Special Control/Status `reg 31` | HCDSPEED bits 4:2; `010=100HD`, `110=100FD` | 실제 negotiated mode |
| STM32H7 `MACCR` | duplex bit 13, loopback bit 12, receive-own-disable bit 10 | MAC/PHY mode 불일치 |
| STM32H7 RX/DMA status | CRC/error, RBU/RPS, descriptor 상태 | 외부 frame drop 위치 |

### 8.3 명시적 승인 후에만 수행할 A/B

1. boot-local로 broadcast를 중단하거나 partner를 `10.0.0.10`으로 고정해 self feedback만
   제거한다. 외부 ARP/GCS가 살아나는지 확인하고 즉시 원복한다.
2. 백업된 `extras.txt`의 중복 `ifup`만 임시 비활성화한다. 현재는 self partner가 extras
   `ifup` 완료 전에 생겼으므로 우선순위가 낮다.
3. 마지막으로 IOB accounting, UDP receive limit, nonblocking/timeout을 적용한 diagnostic
   firmware A/B를 수행한다. source patch와 exact build hash를 repository에 보존한다.

### 8.4 권장 해결책

1. **가장 빠르고 신뢰할 수 있는 조치:** P1 FMUv6X 또는 Ethernet PHY/connector carrier를
   정상 P2-equivalent 부품으로 교체한다.
2. 보드 수리가 가능하면 LAN8742A `RXD0/MODE0`, `RXD1/MODE1`, `CRS_DV/MODE2` strap 저항,
   RXP/RXN, transformer/magnetics, connector, RMII 50 MHz/RXD/CRS_DV를 P2와 비교한다.
3. 교체 전 세부 판별이 필요하면 MDIO diagnostic helper로 `reg 0`, `reg 4`, `reg 31`과
   STM32H7 MACCR/RX DMA counter를 읽는다. register write/diagnostic firmware는 별도 승인과
   exact build hash 보존 뒤에만 수행한다.
4. 현재 Ethernet은 P1→Workstation telemetry 관찰용으로만 취급한다. USB shell/stream
   command는 boot-local 진단 우회이며 ETH command/RTCM 복구가 아니다.

공식 firmware reflash, `MAV_2_*`, host static ARP, host full-only 광고, Workstation reconnect는
복구책이 아님이 확인됐다.

### 8.5 복구 합격 조건

다음 항목이 모두 충족되어야 ETH-only 복구로 판정한다.

- P1 ARP reply와 host ping 성공
- P1 `partner IP=10.0.0.10`, external GCS `255/190` 수신
- self `sysid=1/component=1` duplicate와 `126.9%` 통계 소멸
- ETH `COMMAND_ACK`와 reboot 후 `SCALED_IMU/2/3` stream 설정 성공
- P1 `GPS_RTCM_DATA` wire 송신뿐 아니라 board-side correction 수신 증거 확인
- ATTITUDE/GNSS/IMU 장시간 연속 수신에서 sequence gap과 kernel drop 허용치 충족

P1 RTCM board-side delivery가 확인되기 전에는 RTK 운용 가능으로 판정하지 않는다.

## 9. 증거 파일

Repository-local:

- `firmware-reinstall-20260831T1003KST.json`
- `parameters-backup-pre-flash-20260831T1003KST.bson`
- `extras-pre-flash-20260831T1003KST.txt`
- `manifest.json`, `ethernet_params.json`, `board_identity.txt`, `ethernet_runtime.txt`

Machine-local pcap:

| Artifact | Bytes | SHA-256 |
|---|---:|---|
| `/tmp/p1-post-reflash-attitude-gnss-10m.pcap` | 3,186,079 | `980461f489c9b04b0673c6b5f11737d31274edcd08fc266b1cac5e1063a38fbc` |
| `/tmp/p1-post-reflash-eth.pcap` | 3,264,471 | `dc5a32e406747b4d51f94bc552109b81c39ee58fbe975122125cc323b9050bab` |
| `/tmp/p1-p2-cable-swap.pcap` | 4,171,303 | `463101aabe8089ad845e2cf42162ce97b9217bd25052b552158ca5c74a73b235` |
| `/tmp/rtcm-current-routing.pcap` | 178,247 | `4de721e4482cebc539132ca07e28878ca89fb3e9b6a49ae264089e7d4e865773` |
| `/tmp/p1-originated-ping.pcap` | 2,938 | `378cd8f4b51c1486569dafdbd9841f59267b8a2101713225515408cc92b05ebb` |
| `/tmp/p1-originated-ping-second.pcap` | 1,086 | `e554b0ea33ca49ce642d450e8f61ffd9337bfac84c50f59c772ca0f8a3bf8bc7` |

## 10. Source-level 근거

- PX4 v1.17 `MavlinkReceiver::run()` partner 선택:
  <https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/mavlink_receiver.cpp>
- PX4 v1.17 UDP unicast+broadcast 송신:
  <https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/mavlink_main.cpp>
- P1 exact NuttX STM32H7 Ethernet driver:
  <https://github.com/PX4/NuttX/blob/fb2fadf6f599c1406f052db013efd00a2518e72c/arch/arm/src/stm32h7/stm32_ethernet.c>
- STM32H7 MAC `DO/LM/DM` 정의, ST RM0433:
  <https://www.st.com/resource/en/reference_manual/dm00314099.pdf>
- LAN8742A loopback/duplex register, Microchip datasheet:
  <https://ww1.microchip.com/downloads/en/DeviceDoc/DS_LAN8742_00001989A.pdf>
- 별도 IOB/UDP deadlock 가능성 비교 자료:
  <https://github.com/PX4/PX4-Autopilot/issues/25956>

## 11. Coordinate contract

이 진단은 transport와 liveness만 다룬다. PX4 ATTITUDE body-FRD/world-NED, GNSS WGS84,
MAVLink altitude, IMU producer sensor-axis 및 mG 단위를 변환하거나 재해석하지 않았다.
