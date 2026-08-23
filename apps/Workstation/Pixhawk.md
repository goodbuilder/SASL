# Pixhawk P1/P2 Ethernet MAVLink 설정

이 문서는 `apps/Workstation` 현장 운용용 P1/P2 Pixhawk MAVLink 설정값과 복구 절차를 기록한다.
기준일: 2026-08-24 (Asia/Seoul)

## 1. 네트워크 기준값

Ethernet 인터페이스는 카메라와 Pixhawk가 연결된 `enx588694fce07e`를 사용한다.

| 장치 | IPv4 주소 | MAVLink UDP 포트 | MAVLink system ID |
|---|---:|---:|---:|
| Workstation PC | `10.0.0.10/24` | 수신 | - |
| P1 / Left | `10.0.0.2/24` | `14550` | `1` |
| P2 / Right | `10.0.0.3/24` | `14551` | `2` |

Workstation 설정값:

```text
Pixhawk host address: 10.0.0.10
Left Ethernet IP:    10.0.0.2
Right Ethernet IP:   10.0.0.3
Left MAVLink URL:    udpin:0.0.0.0:14550
Right MAVLink URL:   udpin:0.0.0.0:14551
MAVLink baud:        57600
MAVLink stream rate: 2000000
```

PC의 실제 주소를 확인한다.

```bash
ip -brief addr show enx588694fce07e
ip route
```

## 2. PX4 MAVLink 파라미터

각 보드의 Ethernet MAVLink instance는 `MAV_2`를 사용한다.

```text
P1: MAV_SYS_ID=1
P2: MAV_SYS_ID=2
MAV_2_CONFIG=1000
MAV_2_MODE=0
MAV_2_BROADCAST=1
P1 MAV_2_REMOTE_PRT=14550
P1 MAV_2_UDP_PRT=14550
P2 MAV_2_REMOTE_PRT=14551
P2 MAV_2_UDP_PRT=14551
MAV_2_RATE=2000000
```

P1과 P2의 `MAV_SYS_ID`는 반드시 달라야 한다. 두 보드에서 같은 system ID를 사용하면
파라미터, command ACK, telemetry source를 구분할 수 없다.

## 3. PX4 Ethernet 파일

P1은 IP `.2`, P2는 IP `.3`을 사용한다. 각 보드의 `/fs/microsd/net.cfg`는 다음 형식이다.

### P1 `net.cfg`

```text
DEVICE=eth0
BOOTPROTO=static
NETMASK=255.255.255.0
IPADDR=10.0.0.2
ROUTER=10.0.0.10
```

### P2 `net.cfg`

```text
DEVICE=eth0
BOOTPROTO=static
NETMASK=255.255.255.0
IPADDR=10.0.0.3
ROUTER=10.0.0.10
```

각 보드의 `/fs/microsd/etc/extras.txt`에는 해당 보드의 IP를 올린다.

```text
ifup eth0
ifconfig eth0 <PIXHAWK_IP> netmask 255.255.255.0 up
```

`net.cfg`와 `extras.txt`는 보드별로 다른 IP를 가져야 한다. `extras.txt`는 위 두 줄만
포함하며 `mavlink start` 또는 `mavlink stop`을 넣지 않는다. Ethernet MAVLink instance의
유일한 소유자는 `MAV_2_CONFIG=1000`을 처리하는 PX4 `rc.serial`이다.

## 4. Workstation에서 설정하는 순서

### 두 보드 모두 Ethernet MAVLink

1. PC Ethernet 인터페이스에 `10.0.0.10/24`를 설정한다.
2. P1/P2의 Ethernet 케이블과 전원을 연결한다.
3. `Settings > PX4 / MAVLink`에 1절의 값을 입력한다.
4. USB로 P1을 연결하고 `P1 ETH MAV 시작(USB)`를 실행한다.
5. USB로 P2를 연결하고 `P2 ETH MAV 시작(USB)`를 실행한다.
6. 재부팅 후 USB를 제거해도 `P1 MAVLink 연결`, `P2 MAVLink 연결`에서 heartbeat가 들어오는지 확인한다.
7. 활성 MAVLink 연결 후 `P1 ETH 저장`, `P2 ETH 저장`을 눌러 `net.cfg`, `extras.txt`, `MAV_2_CONFIG`를 영구 저장한다.

### P1 USB + P2 Ethernet 임시 연결

이 구성에서는 endpoint를 다음처럼 설정한다.

```text
P1 / Left MAVLink URL:
  /dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00
P1 / Left MAVLink baud: 57600

P2 / Right MAVLink URL:
  udpin:0.0.0.0:14551
P2 / Right MAVLink baud: 57600
```

이것은 혼합 연결이며 P1 Ethernet telemetry 검증은 하지 않는다. P1까지 ETH-MAV로 운용하려면
P1 endpoint도 `udpin:0.0.0.0:14550`이어야 한다.

### Dual-GNSS 표시와 계산 source 선택

Pixhawk workspace의 `P1/P2 GNSS receivers` 표는 다음 네 행을 동시에 표시한다.

```text
P1 GPS1 = GPS_RAW_INT
P1 GPS2 = GPS2_RAW
P2 GPS1 = GPS_RAW_INT
P2 GPS2 = GPS2_RAW
```

각 행에는 fix/RTK 상태, 위성 수, MSL 고도, ellipsoid 고도, HDOP/VDOP, 실제 H/V accuracy,
age가 표시된다. `Pixhawk` workspace 하단의 `P1 antenna`, `P2 antenna` 선택값이 P1→P2
GNSS baseline과 Measurement snapshot에 사용되며 기본값은 `P1 GPS1 → P2 GPS1`이다. 두 값을
평균하거나 자동으로 더 좋은 receiver로 바꾸지 않는다. 선택값은 설정 파일에 저장된다.

PX4 MAVLink 의미상 `GPS_RAW_INT.alt`는 `sensor_gps.altitude_msl_m`이고,
`GPS_RAW_INT.alt_ellipsoid`는 ellipsoid 고도다. `eph/epv`는 PX4 stream에서 HDOP/VDOP ×100이며
metre accuracy가 아니다. 실제 metre accuracy는 `h_acc/v_acc`다. Workstation baseline은
선택된 두 receiver의 lat/lon/alt MSL로 계산하고, `GPS_RTK/GPS2_RTK`의 내부 moving-base
vector를 P1→P2 baseline으로 직접 사용하지 않는다.

USB 장치의 side는 포트 이름만으로 자동 판단하지 않는다. USB heartbeat의 source system ID를
확인한다. `system ID=1`이면 P1, `system ID=2`이면 P2로 적용한다.

## 5. USB 복구 명령

Workstation의 `P1/P2 ETH MAV` 복구는 해당 Pixhawk USB MAVLink endpoint에서 단일-instance
파라미터와 SD 파일을 저장하고 PX4를 재부팅한다. `<PORT>`, `<IP>`, `<SYSID>`를 보드에 맞게
바꾼다. 이미 실행 중인 instance를 stop/start하지 않는다.

```text
param set MAV_SYS_ID <SYSID>
param set MAV_2_CONFIG 1000
param set MAV_2_MODE 0
param set MAV_2_BROADCAST 1
param set MAV_2_REMOTE_PRT <PORT>
param set MAV_2_UDP_PRT <PORT>
param set MAV_2_RATE 2000000
param save
reboot
```

`/fs/microsd/etc/extras.txt`는 다음 두 줄만 저장한다.

```text
ifup eth0
ifconfig eth0 <IP> netmask 255.255.255.0 up
```

값은 다음과 같다.

```text
P1: IP=10.0.0.2, PORT=14550, SYSID=1
P2: IP=10.0.0.3, PORT=14551, SYSID=2
```

영구 운용은 `ETH 저장`으로 파일과 파라미터를 저장한 뒤 재부팅하고 다시 UDP heartbeat를
확인한다. 수동 `mavlink start`로 즉시 복구하면 자동 instance와 경쟁하거나 동일 포트의 두 번째
instance가 생길 수 있으므로 사용하지 않는다.

## 6. 연결 검증

### 네트워크

```bash
ping -I enx588694fce07e -c 3 10.0.0.2
ping -I enx588694fce07e -c 3 10.0.0.3
ip neigh show dev enx588694fce07e
```

### Workstation UDP socket

```bash
ss -lunp | rg '14550|14551'
```

UDP socket이 열렸다는 것만으로 연결 성공으로 판단하지 않는다. 반드시 양쪽 heartbeat와
ATTITUDE/GPS 수신을 확인한다.

### raw Ethernet packet

```bash
sudo tcpdump -ni enx588694fce07e -nn \
  'udp port 14550 or udp port 14551'
```

정상 예:

```text
10.0.0.2.14550 > 10.0.0.255.14550
10.0.0.3.14551 > 10.0.0.255.14551
```

어느 보드든 IP ping은 정상인데 해당 UDP packet이 없으면 Ethernet IP만 올라온 것이며 MAVLink
instance가 실행되지 않은 상태다. `MAV_2_CONFIG=1000`, board별 port/system ID와 `extras.txt`의
ifup/ifconfig 두 줄을 확인한 뒤 PX4를 재부팅한다. `extras.txt`에 수동 start를 추가하지 않는다.
동일 source packet이 `10.0.0.10`과 `10.0.0.255`로 동시에 반복되면 자동 instance와 수동
instance가 중복 실행된 증거다. 단, 이 판정은 부팅 후 host가 아직 MAVLink packet을 하나도
보내지 않은 capture에만 적용한다.

## 7. 현장 주의사항

- `P1/P2 Connect`는 PC의 UDP listener를 열고 heartbeat를 기다린다. Connect 성공 문구나
  socket bind만으로 장치 연결을 판정하지 않는다.
- P1/P2가 같은 `MAV_SYS_ID`를 사용하지 않도록 한다.
- 두 보드가 같은 Ethernet MAC으로 보이면 ARP 충돌 가능성이 있으므로 두 보드의 MAC과 IP를
  별도로 확인한 뒤 운용한다.
- `usb-FTDI_FT231X_USB_UART_DU0E49VU`는 현재 P2 MAVLink heartbeat가 확인되지 않은 포트다.
  P2 직접 USB 제어에는 `usb-Auterion_PX4_FMU_v6X.x_0-if00`처럼 PX4 FMU로 인식되는 포트를
  사용하고, 연결 후 source system ID를 확인한다.
- P1/P2 모두 `10.0.0.10/24`와 같은 Ethernet 대역에 있어야 한다.
- running instance에 `mavlink stop`을 보내지 않는다. PX4 v1.17 signing callback의
  sparse-instance lookup bug가 있는 firmware에서는 receiver task Hard Fault를 유발할 수 있다.
- `MAV_2_CONFIG=1000`과 수동 `mavlink start`를 함께 사용하지 않는다. start가 성공하면 중복
  instance가 되고, 실패 여부도 부팅 순서에 따라 달라져 연결이 비결정적이 된다.

## 8. 현재 확인 기록

- 이전의 USB 제거 45초 검증은 재발 전 warm-state 관찰이므로 최종 완료 근거로 사용하지 않는다.
- 2026-08-24 단일-instance 수정 검증은 USB가 보드 전원을 공급한 상태에서 수행했지만,
  Workstation subscriber 입력은 `udpin:14550/14551` Ethernet endpoint만 사용했다.
- P1 `10.0.0.2:14550`, system ID `1`; P2 `10.0.0.3:14551`, system ID `2`.
- 두 IP가 서로 다른 MAC으로 ARP 응답하고 각 UDP port가 재부팅 후 자동 시작한다.
- 두 보드 모두 `MAV_2_CONFIG=1000`, mode `0`, broadcast `1`, rate `2000000`을 사용한다.
- QGC와 Workstation을 동시에 같은 UDP port에 bind하지 않는다.
- 선택 source를 바꾼 뒤에는 baseline summary가 `P1 GPSx → P2 GPSy`인지 확인한다. 측정 중 source를
  바꾸면 현재 Measurement를 종료하고 새 Measurement에서 사용한다.

## 9. MAVLink sparse-instance Hard Fault 원인과 해결

### 직접 원인

P1에서 반복 생성된 7개 fault log는 모두 task `mavlink_rcv_if2`, PC `0x080efbea`,
`r4=0`으로 동일했다. matching v6X ELF의 ARM 명령과 source를 대조하면 fault instruction은
null `Mavlink *`에서 instance ID를 읽는 다음 동작이었다.

```text
0x080efbea: ldrb.w r1, [r4, #0x908]
r4 = 0x00000000
```

PX4의 `Mavlink::get_instance_for_status()`가 `mavlink_module_instances`의 null slot을 검사하지
않고 역참조했다. Workstation USB recovery가 running port에 `mavlink stop` 후 `start`를 보내면
배열에 일시적인 빈 slot이 생기고, 다른 receiver의 signing callback이 그 slot을 순회하면서
Hard Fault가 발생할 수 있었다. `SASL_CAN_MAG` stream encoder나 CAN1/CAN2 parser가 fault PC는
아니었다.

### 수정

PX4 firmware patch:

```cpp
for (Mavlink *inst : mavlink_module_instances) {
    if ((inst != nullptr) && (status == inst->get_status())) {
        return inst;
    }
}
```

Workstation USB recovery에서는 `mavlink stop`을 제거했다. Persistent ETH 설정은 eth0를 올린
`extras.txt`와 `MAV_2_CONFIG=1000`을 함께 저장하되, instance 생성은 `MAV_2_CONFIG` 한 곳만
담당한다. USB recovery와 일반 stream 설정도 수동 `mavlink start/stop` 없이 저장 후 재부팅한다.
Rebuilt machine code에는 status 주소를 만들기 전 null slot을 건너뛰는 `cbz` guard가 존재한다.
Workstation은 첫 custom stream request가 유실될 때 `SASL_CAN_MAG`만 최대 3회 bounded retry하고,
첫 dedicated sample을 받으면 즉시 재요청을 중단한다.

수정 firmware:

```text
firmware/px4_fmu-v6x_sasl_can_mag/px4_fmu-v6x_default.px4
SHA256 3ef5736aabdaec9047cf97b0cd5eefa2cfc074672c3ae43776c3f0e12d2349ce
FLASH 1,956,228 / 1,966,080 bytes (99.50%)
```

### 추가 재발 원인: competing MAVLink owners

재발 시 P1/P2 모두 ping은 정상인데 10초 raw capture에서 전체 UDP 송신이 0이었고, source port를
맞춘 유효한 GCS heartbeat에도 응답하지 않았다. P1 cold boot 직후에는 같은 source packet이
`10.0.0.10:14550` 유니캐스트와 `10.0.0.255:14550` 브로드캐스트로 동시에 나타났다. USB
MAVFTP readback 결과 `MAV_2_CONFIG=1000`과 `extras.txt`의 수동 `mavlink start`가 함께 저장돼
있었다. P2도 동일한 파일이었지만 해당 부팅에서는 두 번째 bind가 실패해 브로드캐스트 한 벌만
나왔다. 즉 같은 설정이 부팅 timing에 따라 한 개 또는 두 개 instance를 만드는 비결정적 구성이었다.

P1/P2 `extras.txt`에서 수동 start를 제거하고 ifup/ifconfig 두 줄만 readback했다. 이후 각 보드를
soft reboot하여 `MAV_2_CONFIG`가 만든 브로드캐스트 stream 한 벌만 존재함을 raw packet으로
확인했다.

### 2026-08-24 단일-instance 실장비 검증

- P1 patched custom firmware: USB heartbeat, system ID `1`, `MAV_2_CONFIG=1000`, UDP `14550`.
- P1 Ethernet `10.0.0.2:14550`: 지속 packet 및 Workstation heartbeat 수신.
- P2 Ethernet `10.0.0.3:14551`: 지속 packet 및 Workstation heartbeat 수신.
- Workstation dual subscriber: `HEARTBEATS=['left', 'right']`, connection error 없음.
- P1 `SASL_CAN_MAG`: CAN1 `device_id=8944899/node_id=125`,
  CAN2 `device_id=8944395/node_id=123`.
- P2 `SASL_CAN_MAG`: CAN1 `device_id=8944643/node_id=124`,
  CAN2 `device_id=8944395/node_id=123`.
- P1/P2 raw packet은 각각 `10.0.0.255:14550`, `10.0.0.255:14551` 한 목적지만 사용했다.
- 실제 Workstation dual subscriber 30초: heartbeat `['left', 'right']`, connection/status error 없음.
- P1 ATTITUDE `1761`, GPS `294`, `SASL_CAN_MAG` `1174`; CAN1/CAN2 모두 수신.
- P2 ATTITUDE `1738`, GPS `290`, `SASL_CAN_MAG` `1158`; CAN1/CAN2 모두 수신.
- 이번 실행에서는 첫 요청으로 custom stream이 열려 bounded retry는 발생하지 않았다.
- 실제 `./run.sh --start-pixhawk` UI도 수 분 후 `P1 LIVE 2 ms`, `P2 LIVE 22 ms`를 유지했다.
- 별도 raw capture에서 P1 GPS1/P2 GPS1 MSL 고도는 약 `108.30/100.29 m`, P2 GPS1/GPS2는
  약 `100.29/109.11 m`였다. 네 receiver 모두 `fix_type=5 (RTK_FLOAT)`였지만 P2 내부
  solution 차이가 약 `8.83 m`이므로 antenna/correction/receiver role을 검증하기 전에는
  GPS1/GPS1 baseline을 측정에 승인하지 않는다.
- USB는 보드 전원 공급용으로 연결돼 있었다. 별도 전원에서 USB data/power를 모두 제거한 장시간
  soak는 아직 최종 현장 검증 항목이다.
- Workstation 전체 테스트 `1054 passed, 1 skipped`; runtime check 통과.

P2는 기존 custom firmware에서도 정상 동작했지만 동일한 latent null-slot bug를 포함할 수 있다.
Workstation no-stop 변경과 단일 `MAV_2_CONFIG` owner로 알려진 trigger와 competing start를 제거했으며, 정비 가능 시 P2도
위 patched artifact로 업데이트한다.
