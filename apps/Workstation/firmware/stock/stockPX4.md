# Stock PX4 v1.17.0 설정

이 문서는 SASL Workstation의 P1/Left 및 P2/Right Pixhawk에 공식 PX4
`v1.17.0` stock FMUv6X firmware를 적용할 때 필요한 파라미터와
`/fs/microsd/net.cfg` 내용을 정리한다.

- Firmware target: `px4_fmu-v6x_default` (`board_id=53`)
- Firmware file: [`px4_fmu-v6x_default.px4`](px4_fmu-v6x_default.px4)
- P1 QGC overlay: [`p1_workstation.params`](p1_workstation.params)
- P2 QGC overlay: [`p2_workstation.params`](p2_workstation.params)
- 전체 provenance와 검증 절차: [`MANIFEST.md`](MANIFEST.md)

이 firmware는 FMUv6X/Pixhawk 6X 전용이다. FMUv6C/Pixhawk 6C 또는 다른
board target에는 flash하지 않는다. 작업 전에는 각 Pixhawk의 전체 파라미터를
별도 파일로 백업하고 한 번에 한 대만 USB로 연결한다.

## 1. Workstation 네트워크 기준

| 장치 | IP 주소 | MAVLink system ID | UDP port |
|---|---:|---:|---:|
| Workstation PC | `10.0.0.10/24` | - | 수신 |
| P1 / Left | `10.0.0.2/24` | `1` | `14550` |
| P2 / Right | `10.0.0.3/24` | `2` | `14551` |

## 2. PX4 파라미터

| 파라미터 | P1 | P2 | 의미 |
|---|---:|---:|---|
| `MAV_SYS_ID` | `1` | `2` | P1/P2 MAVLink identity 분리 |
| `MAV_PROTO_VER` | `2` | `2` | MAVLink 2 사용 |
| `MAV_2_CONFIG` | `1000` | `1000` | MAVLink instance 2를 Ethernet에 배정 |
| `MAV_2_MODE` | `0` | `0` | Normal/GCS stream profile |
| `MAV_2_BROADCAST` | `1` | `1` | Ethernet heartbeat broadcast |
| `MAV_2_RADIO_CTL` | `0` | `0` | Ethernet에서 SiK-radio throttling 비활성화 |
| `MAV_2_RATE` | `2000000` | `2000000` | 최대 MAVLink 전송 ceiling, bytes/s |
| `MAV_2_REMOTE_PRT` | `14550` | `14551` | Workstation 목적지 UDP port |
| `MAV_2_UDP_PRT` | `14550` | `14551` | PX4 Ethernet MAVLink UDP port |
| `GPS_1_CONFIG` | `201` | `201` | Main F9P를 GPS1 port에 배정 |
| `GPS_1_PROTOCOL` | `1` | `1` | u-blox protocol |
| `GPS_2_CONFIG` | `202` | `202` | Secondary F9P를 GPS2 port에 배정 |
| `GPS_2_PROTOCOL` | `1` | `1` | u-blox protocol |
| `UAVCAN_ENABLE` | `2` | `2` | DroneCAN sensor automatic configuration |
| `UAVCAN_BITRATE` | `1000000` | `1000000` | 현재 RM3100 CAN bus bitrate |
| `UAVCAN_SUB_MAG` | `1` | `1` | DroneCAN magnetometer subscription |

두 번째 F9P가 실제로 연결되지 않은 구성에서는 `GPS_2_CONFIG=0`으로 두고 해당
행을 overlay에서 적용하지 않는다. RM3100이 DroneCAN이 아니거나 CAN bitrate가
다르면 실제 node 설정을 우선한다.

### P1 / Left PX4 shell 값

```sh
param set MAV_SYS_ID 1
param set MAV_PROTO_VER 2
param set MAV_2_CONFIG 1000
param set MAV_2_MODE 0
param set MAV_2_BROADCAST 1
param set MAV_2_RADIO_CTL 0
param set MAV_2_RATE 2000000
param set MAV_2_REMOTE_PRT 14550
param set MAV_2_UDP_PRT 14550
param set GPS_1_CONFIG 201
param set GPS_1_PROTOCOL 1
param set GPS_2_CONFIG 202
param set GPS_2_PROTOCOL 1
param set UAVCAN_ENABLE 2
param set UAVCAN_BITRATE 1000000
param set UAVCAN_SUB_MAG 1
param save
```

### P2 / Right PX4 shell 값

```sh
param set MAV_SYS_ID 2
param set MAV_PROTO_VER 2
param set MAV_2_CONFIG 1000
param set MAV_2_MODE 0
param set MAV_2_BROADCAST 1
param set MAV_2_RADIO_CTL 0
param set MAV_2_RATE 2000000
param set MAV_2_REMOTE_PRT 14551
param set MAV_2_UDP_PRT 14551
param set GPS_1_CONFIG 201
param set GPS_1_PROTOCOL 1
param set GPS_2_CONFIG 202
param set GPS_2_PROTOCOL 1
param set UAVCAN_ENABLE 2
param set UAVCAN_BITRATE 1000000
param set UAVCAN_SUB_MAG 1
param save
```

파라미터는 QGroundControl의 **Vehicle Setup > Parameters > Tools > Load from
file for review**에서 해당 `.params` 파일을 선택해 적용할 수도 있다. P2 stock
board도 첫 부팅은 system ID `1`이므로 P2 한 대만 USB로 연결한 상태에서 P2
overlay를 적용하고 재부팅 후 system ID `2`를 확인한다.

## 3. `/fs/microsd/net.cfg`

PX4 stock firmware의 Ethernet IP는 파라미터가 아니라 SD 카드의
`/fs/microsd/net.cfg` 입력으로 설정한다. `netman update -i eth0`가 이 파일을
비휘발성 설정에 저장한 뒤 파일을 삭제하므로, 재부팅 후 `cat
/fs/microsd/net.cfg`가 실패하는 것은 정상이다. 영구 설정은 `netman show`로
확인한다.

### P1 / Left `/fs/microsd/net.cfg`

```ini
DEVICE=eth0
BOOTPROTO=static
NETMASK=255.255.255.0
IPADDR=10.0.0.2
ROUTER=10.0.0.10
```

원본 파일: [`network/p1/net.cfg`](network/p1/net.cfg)

### P2 / Right `/fs/microsd/net.cfg`

```ini
DEVICE=eth0
BOOTPROTO=static
NETMASK=255.255.255.0
IPADDR=10.0.0.3
ROUTER=10.0.0.10
```

원본 파일: [`network/p2/net.cfg`](network/p2/net.cfg)

## 4. `/fs/microsd/etc/extras.txt`

현재 Workstation 현장 구성은 부팅 시 interface를 명시적으로 올리는 다음 파일도
사용한다.

### P1 / Left

```sh
ifup eth0
ifconfig eth0 10.0.0.2 netmask 255.255.255.0 up
```

### P2 / Right

```sh
ifup eth0
ifconfig eth0 10.0.0.3 netmask 255.255.255.0 up
```

`extras.txt`에는 `mavlink start` 또는 `mavlink stop`을 넣지 않는다.
Ethernet MAVLink instance의 유일한 owner는 `MAV_2_CONFIG=1000`을 처리하는
PX4 `rc.serial`이다. `/fs/microsd/net.cfg`를 복사한 뒤 다음을 실행하면
PX4가 NVM에 저장하고 자동 재부팅한다.

```sh
netman update -i eth0
```

## 5. 적용 후 확인

파라미터와 SD 파일을 저장한 뒤 Pixhawk를 재부팅한다. QGroundControl과
Workstation이 동시에 같은 UDP port를 점유하지 않도록 한 상태에서 다음을 확인한다.

```sh
param show MAV_SYS_ID
param show MAV_2_CONFIG
param show MAV_2_MODE
param show MAV_2_RADIO_CTL
param show MAV_2_REMOTE_PRT
param show MAV_2_UDP_PRT
param show MAV_2_RATE
param show GPS_1_CONFIG
param show GPS_2_CONFIG
param show UAVCAN_ENABLE
param show UAVCAN_SUB_MAG
netman show
ifconfig eth0
mavlink status
```

승인 조건:

- P1/P2가 각각 `10.0.0.2`와 `10.0.0.3`으로 ping에 응답한다.
- `netman show`가 `DEVICE`, `BOOTPROTO`, `IPADDR`, `NETMASK`, `ROUTER`를
  기대값으로 표시한다. `netman update` 후 SD 카드에서 `net.cfg`가 사라지는
  것은 정상이다.
- 두 보드는 서로 다른 MAC 주소와 MAVLink system ID를 가진다.
- 각 보드에는 Ethernet MAVLink instance가 한 개만 존재한다.
- Workstation이 양쪽 `HEARTBEAT`, `ATTITUDE`, `GPS_RAW_INT`, `GPS2_RAW`를 받는다.
- RTCM 주입과 한 번의 Measurement 시작/종료/저장이 정상 완료된다.

Stock PX4에는 private `SASL_CAN_MAG` stream이 없다. RM3100은 정상
`sensor_mag -> EKF2 -> ATTITUDE` 경로에 사용할 수 있지만 Workstation의 CAN1/CAN2
개별 `can_port`, `node_id`, `device_id` 표시는 사용할 수 없다. `CAL_ACC*`,
`CAL_GYRO*`, `CAL_MAG*`, airframe, actuator, battery 및 safety 값은 이 문서와
overlay에서 설정하지 않으며 각 보드에서 QGroundControl 보정을 수행한다.

## Coordinate contract

이 설정은 좌표 계산이나 축 순서를 변경하지 않는다. PX4 `ATTITUDE`의 body-FRD /
world-NED 계약과 GPS WGS84 및 MAVLink altitude 필드의 의미는 기존 Workstation
계약을 그대로 유지한다.
