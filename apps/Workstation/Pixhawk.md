# Pixhawk P1/P2 Ethernet MAVLink 설정

이 문서는 `apps/Workstation` 현장 운용용 P1/P2 Pixhawk MAVLink 설정값과 복구 절차를 기록한다.
기준일: 2026-09-04 (Asia/Seoul)

## 1. 네트워크 기준값

인터페이스 이름은 USB NIC의 MAC에 따라 바뀔 수 있으므로 계약값으로 고정하지 않는다.
현재 field-qualified 배선은 카메라 NIC와 Pixhawk NIC를 분리하지만, Workstation UI는
P1/P2에 같은 NIC 또는 서로 다른 NIC를 operator가 배정할 수 있다.

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

PC의 실제 주소와 route를 확인한다.

```bash
ip -brief addr
ip route get 10.0.0.2
ip route get 10.0.0.3
ip route
```

### Workstation `Find network` / Assign / Connect

1. Pixhawk 화면에서 `Find network`를 누른다.
2. 후보 표에서 원하는 물리 NIC를 선택한다. 표 행은 NIC 후보이며 현재
   `Camera/P1/P2` 역할 집합을 표시한다.
3. `Assign P1` 또는 `Assign P2`를 누르고, 정확한 before/after 주소·route와 자동 재연결 대상을
   확인한 뒤 OS 인증을 승인한다. Camera 화면의 `Assign Camera NIC`도 같은 역할 적용기를 사용한다.
4. 분리된 P1/P2 assignment card에서 각자의 State/NIC/Local IPv4/Endpoint/Route를 확인한다.
5. 이전에 연결돼 있던 side는 자동 재연결되며 expected sysid HEARTBEAT+ATTITUDE를 확인한다.
   이전에 연결되지 않았던 side는 `Connect`를 누른다.

NIC 식별자는 커널 인터페이스 이름이 아니라 MAC이다. USB NIC를 다른 포트에 꽂거나 재부팅해서
이름이 `enxAAAA` -> `enxBBBB`로 바뀌어도, 저장된 MAC과 같은 NIC가 같은 local IPv4를 그대로
들고 있으면 `Find network`가 배정을 그대로 유지하고 새 이름을 기록한다. 이때 assignment card의
Route detail에 `NIC renamed <이전> -> <현재>`가 남고 후보 표의 `Camera/P1/P2` 역할도 새 이름
행에 그대로 표시된다. 같은 MAC은 있는데 저장된 주소가 그 NIC에 없으면 `REASSIGN REQUIRED`로
내려가며, detail이 그 MAC이 현재 어느 NIC/주소에 있는지 알려준다. MAC이 저장되지 않은 옛
설정만 이름 비교로 되돌아간다.

`Find network`와 Connect preflight는 read-only다. 오직 확인된 Assign만 NetworkManager를
변경한다. Workstation은 선택 NIC의 역할에 필요한 주소·host route만 합성하고 기본 route나
다른 사용자 설정은 만들거나 제거하지 않는다. 한 NIC가 Camera+P1+P2를 모두 가질 수도 있고,
여러 NIC에 임의로 나눌 수도 있다. 역할 이동은 대상 NIC를 먼저 적용·readback한 뒤 기존 NIC에서
이동한 역할의 Workstation-owned 항목만 제거하며, 실패하면 기존 배치로 rollback한다.

정상 Camera/Pixhawk가 영향을 받으면 해당 source/target NIC owner만 자동 stop/reconnect한다.
Camera는 새 generation frame, Pixhawk는 expected sysid HEARTBEAT+ATTITUDE가 들어와야 저장한다.
Measurement/기록/save/finalization 중에는 source identity 보호를 위해 변경 전 차단한다. P1/P2
partial connection과 다른 NIC의 정상 장치는 유지한다. 같은 NIC 사용에는 UI 경고/gate가 없지만,
과거 shared-L2 RTSP 부하에서 ETH MAVLink stall이 재현됐으므로 장시간 field 안정성 증거와는 구분한다.

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

### MAG 슬롯 강제 선택별 P1/P2 yaw 오차 진단 (2026-09-04)

P1/P2 모두 disarm 상태에서, 각 MAG 슬롯 하나만 priority `100`으로 두고 나머지 MAG 슬롯은
`0`으로 둔 뒤 `vehicle_magnetometer.device_id`와 5초 ATTITUDE yaw 중앙값을 기록했다. 시험 전
P1/P2의 가속도계·자이로·자력계 보정 correction과 level-horizon offset은 PX4 기본값으로
초기화되어 있었다(`offset/보상=0`, `scale=1`). 따라서 아래 yaw는 최종 운용 캘리브레이션 결과가
아니라 센서 선택별 진단값이다.

`ΔYaw`는 `wrap180(P2 yaw - P1 yaw)`로 계산한 최단각이다. 각 행의 yaw는 순차 시험 중 중앙값이며,
active device가 기대한 MAG ID와 다르면 fallback으로 표시한다.

| 강제 조합 | 센서 | P1 active / yaw | P2 active / yaw | ΔYaw (P2-P1) |
|---|---|---:|---:|---:|
| MAG0 / MAG0 | 내부 BMM150 (`4395041`) | `4395041` / `178.88°` | `4395041` / `147.54°` | `-31.34°` |
| MAG1 / MAG1 | GPS1 IST8310 (`396809`) | `396809` / `147.59°` | `396809` / `162.70°` | `+15.11°` |
| MAG2 / MAG2 | P1 RM3100 `8944899`, P2 RM3100 `8944643` | `8944899` / `-149.43°` | `8944643` / `-152.96°` | `-3.53°` |
| MAG3 / MAG3 | RM3100 backup 기대 ID `8944387` | `396817` fallback / `81.05°` | `396817` fallback / `86.69°` | `+5.64°`* |

`*` MAG3 시험에서는 기대한 `8944387`이 선택되지 않고 GPS2 IST8310 (`396817`)으로
fallback되었으므로 MAG3 조합의 센서 오차로 해석하지 않는다.

진단 중 기본 MAG1 선택 상태의 동시 8초 샘플은 P1 `147.59°`, P2 `162.95°`, `ΔYaw
≈+15.36°`였다. 이후 MAG2를 양쪽에서 선택하도록 priority를 변경했다.

```text
P1: CAL_MAG0_PRIO=0, CAL_MAG1_PRIO=75, CAL_MAG2_PRIO=100, CAL_MAG3_PRIO=0
P2: CAL_MAG0_PRIO=0, CAL_MAG1_PRIO=75, CAL_MAG2_PRIO=100, CAL_MAG3_PRIO=0
P1 active: device_id=8944899
P2 active: device_id=8944643
```

MAG2 active 상태의 동시 8초 샘플은 다음과 같았다.

```text
P1 yaw = -149.229°
P2 yaw = -150.144°
ΔYaw   = -0.915°
```

따라서 현재 장비에서는 GPS1 내장 compass인 MAG1보다 RM3100인 MAG2를 P1/P2 공통 heading
소스로 사용하는 편이 일관성이 높다. 초기 MAG2 선택 실험 당시에는 위 설정을 적용했으며, 이후
MAG2/MAG3 재시험 및 오차순위 적용으로 현재 설정은 아래 후속 섹션에 기록되어 있다. 두
보드의 MAG 보정값은 reset 상태이므로 실제 운용 전 QGroundControl에서 MAG2를 최종 장착
상태로 다시 캘리브레이션해야 한다. `SENS_BOARD_ROT`, `CAL_MAG*_ROT`, 센서 ID는 이 시험에서
변경하지 않았다.

### MAG2/MAG3 재시험 및 오차순위 priority 적용 (2026-09-04)

앞선 표의 MAG2/MAG3 결과를 다시 확인하기 위해 P1/P2를 각각 강제 선택했다. MAG2 시험에서는
P1 RM3100 `8944899`, P2 RM3100 `8944643`가 실제 active device로 확인됐고, 6초 ATTITUDE
중앙값은 다음과 같았다.

```text
P1 MAG2 yaw = -149.339°
P2 MAG2 yaw = -148.721°
MAG2 ΔYaw   = +0.62° (P2-P1)
```

MAG3 시험에서는 양쪽 모두 기대한 RM3100 backup `8944387`이 active device가 되지 않았다.
P1은 GPS2 IST8310 `396817`으로 fallback했고, P2도 `8944387` active readback을 확인하지
못했다. 따라서 MAG3의 yaw는 유효한 센서 조합 오차로 사용하지 않는다.

운영 요청에 따라 MAG2를 `100`, MAG3를 `75`로 두고, 유효하게 측정된 기존 오차 순서에 따라
MAG1을 `50`, MAG0을 `25`로 설정했다. MAG3 `75`는 센서 오차가 검증된 값이 아니라
provisional fallback priority다.

```text
P1: CAL_MAG0_PRIO=25, CAL_MAG1_PRIO=50, CAL_MAG2_PRIO=100, CAL_MAG3_PRIO=75
P2: CAL_MAG0_PRIO=25, CAL_MAG1_PRIO=50, CAL_MAG2_PRIO=100, CAL_MAG3_PRIO=75
P1 active: device_id=8944899
P2 active: device_id=8944643
```

두 보드 모두 disarm 상태에서 priority readback과 active MAG2를 확인했다. 현재 MAG 보정값은
reset 상태이므로 위 priority 설정은 최종 calibration 품질을 의미하지 않는다. QGroundControl에서
P1/P2 MAG2를 재캘리브레이션한 뒤 동일 자세에서 ΔYaw를 다시 측정해야 한다.

### MAG2 1분 안정성 및 자이로 단독 캘리브레이션 전후 관찰 (2026-09-04)

현재 MAG priority는 `MAG0=25, MAG1=50, MAG2=100, MAG3=75`이며, 측정 시작 전에 P1
active device `8944899`, P2 active device `8944643`를 확인했다. 두 보드는 disarm 상태였고,
가속도계·자력계·level 캘리브레이션은 추가로 실행하지 않았다. R/P/Y는 Workstation UI가
아니라 각 보드의 raw MAVLink `ATTITUDE`를 직접 수집했다. Yaw 통계는 `[-180°, 180°)`
경계를 피하도록 첫 샘플 기준으로 unwrap했다.

#### MAG2 baseline과 gyro-only calibration

| 단계 | P1 | P2 |
|---|---:|---:|
| MAG2 baseline wall time / samples | `60.011 s / 5,965` | `60.269 s / 212` |
| 자이로 명령 | `MAV_CMD_PREFLIGHT_CALIBRATION`, `param1=1` | `MAV_CMD_PREFLIGHT_CALIBRATION`, `param1=1` |
| calibration 결과 | ACK `0`, progress `0→100`, `calibration done: gyro` | ACK `0`, progress `30→100`, `calibration done: gyro` |
| calibration 중 경고 | `GPS Horizontal Pos Error too high` | `GPS Speed Accuracy too low` |
| MAG active 유지 | `8944899` | `8944643` |
| post-calibration wall time / samples | `59.996 s / 6,000` | `60.015 s / 237` |

`CAL_GYRO0~2_{X,Y,Z}OFF`는 캘리브레이션 전후 모두 정밀 PARAM readback에서 `0.0`이었고,
gyro ID/priority/rotation도 변하지 않았다. 즉 이번 gyro calibration은 성공적으로 종료됐지만
저장된 gyro offset을 실질적으로 바꾸지는 않았다.

#### R/P/Y 통계

`mean shift`와 `range shift`는 각각 post-calibration 값에서 baseline 값을 뺀 값이다.
`drift`는 각 1분 구간의 시작값에서 끝값으로의 변화다. 단위는 모두 degree다.

| 보드 | 축 | baseline mean | post mean | mean shift | baseline range | post range | baseline drift | post drift |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| P1 | R | `+0.6197` | `+0.6178` | `-0.0020` | `0.0198` | `0.0196` | `-0.0034` | `-0.0052` |
| P1 | P | `+6.4405` | `+6.4387` | `-0.0018` | `0.0165` | `0.0157` | `-0.0057` | `+0.0016` |
| P1 | Y | `-146.5760` | `-146.5607` | `+0.0152` | `0.1019` | `0.0663` | `-0.0349` | `-0.0576` |
| P2 | R | `+0.1714` | `+0.1847` | `+0.0133` | `0.0218` | `0.0162` | `+0.0020` | `+0.0057` |
| P2 | P | `+6.7177` | `+6.7173` | `-0.0004` | `0.0421` | `0.0218` | `+0.0045` | `+0.0008` |
| P2 | Y | `-148.7710` | `-148.7302` | `+0.0407` | `0.0683` | `0.1339` | `-0.0269` | `-0.0576` |

축별 표준편차는 P1에서 R/P/Y 각각 `0.0036/0.0036/0.0255°`에서
`0.0035/0.0028/0.0171°`로 변했고, P2에서는 `0.0044/0.0071/0.0190°`에서
`0.0035/0.0055/0.0369°`로 변했다. P1 yaw 범위는 줄었지만 P2 yaw 범위와 표준편차는
늘어났으므로 gyro calibration이 R/P/Y 안정성을 일관되게 개선했다고 볼 수 없다. 1분 평균
P2-P1 yaw 차이는 `-2.1950°`에서 `-2.1695°`로 약 `+0.0255°`만 변했다.

이번 관찰에서는 gyro 명령 완료 자체는 PASS지만, `CAL_GYRO*` 값이 이미 0이어서 전후
자세 변화의 원인을 gyro calibration 효과로 분리할 수 없다. MAG2 heading과 estimator의
시간·온도·자기장 상태가 함께 영향을 줄 수 있으므로, 최종 운용 판단은 QGroundControl에서
필요한 실제 sensor calibration을 완료한 뒤 반복 측정한다.

#### P2 gyro calibration reset 후 1분 재관찰 (2026-09-04)

P2의 `CAL_GYRO0~2_{X,Y,Z}OFF`만 PX4 기본값으로 다시 reset했다. P2는 disarm이고 MAG2
`device_id=8944643`가 active인 상태였다. reset 전후 9개 offset의 정밀 PARAM readback은
모두 `0.0`이었으므로, 이번 작업은 저장된 gyro 수치를 바꾸지 않은 no-op에 가깝다.

reset 직후 raw MAVLink `ATTITUDE`를 60.093초 동안 253개 수집했다. 비교 기준은 바로 앞의
gyro calibration 이후 P2 1분 관찰이다.

| 축 | 이전 post-cal mean | reset 후 mean | mean shift | 이전 range | reset 후 range | range shift | 이전 drift | reset 후 drift |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| R | `+0.1847°` | `+0.1836°` | `-0.0011°` | `0.0162°` | `0.0212°` | `+0.0050°` | `+0.0057°` | `+0.0047°` |
| P | `+6.7173°` | `+6.7200°` | `+0.0027°` | `0.0218°` | `0.0254°` | `+0.0036°` | `+0.0008°` | `+0.0033°` |
| Y | `-148.7302°` | `-148.8971°` | `-0.1669°` | `0.1339°` | `0.1241°` | `-0.0098°` | `-0.0576°` | `+0.1058°` |

P2의 표준편차는 R/P/Y 각각 `0.0035/0.0055/0.0369°`에서
`0.0040/0.0053/0.0347°`로 변했다. Yaw 범위와 표준편차는 소폭 줄었지만, 1분 drift의
방향이 음수에서 양수로 바뀌었고 평균도 `0.1669°` 이동했다. 그러나 gyro offset 자체가
reset 전후 동일한 `0.0`이었으므로, 이 차이를 gyro reset 효과라고 단정할 수 없다. MAG2
heading, estimator 상태, 시간·온도 변화가 포함된 재관찰 결과로 기록하며, P2는 실제 비행
전에 gyro calibration을 다시 수행할 필요가 있다면 QGroundControl 절차로 확인한다.

#### P2 MAG2 5분 장기 관찰 (2026-09-04)

P2 `device_id=8944643` MAG2 active, `CAL_MAG0~3_PRIO=25/50/100/75`, disarm 상태에서
자이로 offset reset 이후 raw MAVLink `ATTITUDE`를 300.069초 동안 1,193개 수집했다.
Yaw는 첫 샘플 기준으로 unwrap했다.

| 축 | mean | min | max | range | 표준편차 | start→end drift |
|---|---:|---:|---:|---:|---:|---:|
| R | `+0.1824°` | `+0.1704°` | `+0.1942°` | `0.0237°` | `0.0044°` | `-0.0038°` |
| P | `+6.7202°` | `+6.7022°` | `+6.7374°` | `0.0352°` | `0.0062°` | `-0.0021°` |
| Y | `-148.9040°` | `-148.9464°` | `-148.8592°` | `0.0872°` | `0.0187°` | `+0.0014°` |

직전 P2 1분 관찰과 비교한 평균 이동은 R `-0.0012°`, P `+0.0002°`, Y `-0.0069°`였다.
Yaw 표준편차는 `0.0347° → 0.0187°`로 줄었고, 5분 구간의 시작·끝 yaw 차이는 `+0.0014°`로
누적 drift가 거의 없었다. R/P 범위는 관찰 시간이 길어져 각각 `0.0237°/0.0352°`로
기록됐다. 이 결과는 정지 상태에서 P2 MAG2 heading이 5분 동안 크게 누적 표류하지 않았다는
뜻이며, 움직임·온도 변화가 포함된 운용 조건의 정확도를 보증하지 않는다.

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
ip route get 10.0.0.2
ip route get 10.0.0.3
ping -c 3 10.0.0.2
ping -c 3 10.0.0.3
```

### Workstation UDP socket

```bash
ss -lunp | rg '14550|14551'
```

UDP socket이 열렸다는 것만으로 연결 성공으로 판단하지 않는다. 반드시 양쪽 heartbeat와
ATTITUDE/GPS 수신을 확인한다.

### raw Ethernet packet

```bash
sudo tcpdump -ni <ASSIGNED_NIC> -nn \
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
- `usb-FTDI_FT231X_USB_UART_DU0E49VU`는 P2 Telemetry Radio ground 포트다. NETID 25/26 분리
  전에는 이 포트에서 P2 heartbeat가 확인되지 않았으나, 2026-08-31 NETID 26 write/readback과
  공중 라디오 전원 재인가 이후 sysid 2만 수신하는 것이 확인됐다(15초 298 메시지 / heartbeat 15).
  P1은 `D30JNMVH`(NETID 25)를 사용한다. NETID를 바꾼 뒤에는 공중 라디오 전원을 완전히 다시
  인가해야 링크가 살아난다. 근거는 `docs/changes/20260831-pixhawk-radio-fallback/verification.md`.
- P2 직접 USB 제어에는 `usb-Auterion_PX4_FMU_v6X.x_0-if00`처럼 PX4 FMU로 인식되는 포트를
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
## Port-swap baseline before CAN1/CAN2 diagnostic (2026-09-05 16:14 KST)

This is the saved reference state immediately before physically swapping the P1/P2 CAN1 and
CAN2 connections. GPS2 was removed and both Pixhawks had been rebooted. The Workstation was
running over the P1/P2 Radio endpoints.

### Topology and CAL_MAG identity

```text
P1 sensor_mag: instance 0=4395041 (internal), 1=396809 (GPS1),
               2=8944899 (CAN1 RM3100), 3=8944387 (CAN2 RM3100)
P2 sensor_mag: instance 0=4395041 (internal), 1=396809 (GPS1),
               2=8944643 (CAN1 RM3100), 3=8944387 (CAN2 RM3100)

P1 CAL_MAG0_ID=4395041, CAL_MAG1_ID=396809, CAL_MAG2_ID=8944899, CAL_MAG3_ID=8944387
P2 CAL_MAG0_ID=4395041, CAL_MAG1_ID=396809, CAL_MAG2_ID=8944643, CAL_MAG3_ID=8944387
P1/P2 CAL_MAG0~3_PRIO = 1 / 1 / 100 / 1
```

### RAW yaw comparison before port swap

```text
P1: GPS1 +159.271°, CAN1 -158.690°, CAN2 +156.432°, internal +174.359°
P2: GPS1 +177.015°, CAN1 -163.745°, CAN2 +163.087°, internal +153.700°

wrapped P2-P1 RAW yaw difference:
GPS1 +17.744°, CAN1 -5.055°, CAN2 +6.655°, internal -20.660°
```

The smallest cross-Pixhawk RAW yaw difference was CAN1. The subsequent UI priority test set
CAN1 to 100 and CAN2 to 1; direct PX4 readback selected P1 `8944899` and P2 `8944643`, with
fresh ATTITUDE yaw P1 `-159.972°`, P2 `-167.249°`, wrapped delta `-7.277°`. This snapshot is
the pre-swap comparison oracle; do not overwrite it when recording the post-swap result.

### Post-swap result — yaw follows device identity (2026-09-05)

After swapping the physical CAN1/CAN2 connections on both Pixhawks and rebooting:

```text
P1: instance 2 = CAN1 device 8944899, instance 3 = CAN2 device 8944387
P2: instance 2 = CAN2 device 8944387, instance 3 = CAN1 device 8944643

device-identity yaw:
CAN1: P1 -158.7°, P2 -163.7°
CAN2: P1 +156.4°, P2 +162.8°
```

The CAN1/CAN2 yaw characteristics therefore follow the RM3100 device identity rather than the
physical Pixhawk CAN port. This points to sensor-specific axis/frame configuration or physical
magnetic bias/placement, not a Pixhawk CAN-port transform. With the pre-swap priorities retained,
the active devices remained P1 `8944899` and P2 `8944643`; live ATTITUDE was approximately
P1 `-160.01°`, P2 `-168.26°` (wrapped difference about `-8.25°`).

## GPS removal freshness check (2026-09-05)

After the operator removed the GPS magnetometers and rebooted, the raw topic list must be read
with freshness/health, not only by cached `device_id`. Direct P1/P2 checks showed:

```text
P1 sensors_status_mag:
  device_ids = [4395041, 8944899, 8944387, 0]
  healthy    = [True, True, True, False]
  primary   = 8944899

P2 sensors_status_mag:
  device_ids = [4395041, 396809, 8944387, 8944643]
  healthy    = [True, False, True, True]
  MAG1 396809 timestamp age ≈229 s (stale, not live)
  primary   = 8944643
```

Therefore P2 `MAG1=396809` appearing in a Shell/UI topology list does not prove that GPS1 is
physically connected. The UI must surface stale/healthy state and the actual active device
separately; the stale GPS1 record is not a live magnetometer input.
