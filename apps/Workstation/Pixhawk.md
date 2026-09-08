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

## 10. P1-P2 ΔYaw를 0°대로 낮춘 MAG priority 저장/재부팅 절차 (2026-09-06)

### 배경: Telemetry Radio 링크 자체가 죽어 있었음

조사 시작 시점에 P1/P2/Drone 지상 라디오(D30JNMVH/DU0E49VU/DU0E4O0F) 3개 모두 raw
serial 레벨에서 8초간 0바이트 수신이었다. `+++`/`ATI5`로 확인한 지상 라디오 자체
NETID(25/26/27)는 정상이었지만 `RTI5`(원격 조회)가 전부 무응답이었고, 공중 라디오 LED가
P1/P2 모두 **solid red**로 확인됐다. Holybro 공식 문서 기준 solid red는 "firmware
update(bootloader) 모드"이며, 이 상태에서는 응용 펌웨어가 전혀 실행되지 않는다.

- [SiK Telemetry Radio V3 — LED and Connection (Holybro)](https://docs.holybro.com/radio/sik-telemetry-radio-v3/led-and-connection)
- [SIK Telemetry radio V3 solid red led — ArduPilot Discourse](https://discuss.ardupilot.org/t/sik-telemetry-radio-v3-solid-red-led/98980)

즉 **P1/P2 공중 SiK 라디오가 부팅로더에 멈춰 있어 Telemetry Radio로는 현재 아무 것도
연결할 수 없다.** 이 항목은 별도 QGroundControl SiK 펌웨어 복구가 필요하며 이번 작업에서는
해결하지 않았다. 아래 측정/조치는 전부 P1/P2 직접 USB(`/dev/ttyACM0`, `/dev/ttyACM1`,
sysid로 자동 식별, `usb-Auterion_PX4_FMU_v6X.x_0-if00` by-id는 두 보드가 동일 USB
serial ID 0을 보고해 side-stable하지 않음)로 수행했다.

USB 진단 중 다른(이전 날짜) 워크트리에서 실행 중이던 Workstation 프로세스가 같은 두 USB 포트에
동시 접속해 있어 heartbeat sysid가 깨지는(garbled, sysid=0) 현상이 있었다. 그 프로세스를
종료한 뒤에는 정상화됐다 — 같은 Pixhawk를 raw MAVLink로 진단할 때는 그 포트를 쓰는 다른
프로세스가 없는지 `fuser`로 먼저 확인해야 한다.

### P1/P2 전체 파라미터 비교 (읽기 전용, USB)

`PARAM_REQUEST_LIST`로 P1 1173개, P2 1183개 파라미터를 모두 덤프해 `MAG/GYRO/EKF2/ATT_/
SENS_BOARD` 키워드로 필터링한 결과, 실제로 값이 다른 항목은 다음뿐이었다.

```text
CAL_GYRO1_XOFF: P1=-0.001711  P2=0.006096   (정상적인 개별 자이로 바이어스)
CAL_GYRO1_YOFF: P1=0.003596   P2=0.003268   (정상적인 개별 자이로 바이어스)
CAL_GYRO1_ZOFF: P1=-0.009193  P2=-0.009615  (정상적인 개별 자이로 바이어스)
CAL_MAG2_ID:    P1=8944899    P2=8944643    (보드별 고유 RM3100 CAN1 device id, 정상)
EKF2_MAG_DECL:  P1=-8.034926  P2=-8.034929  (사실상 동일, 무시 가능)
IMU_GYRO_RATEMAX: P1=400  P2=800           (비대칭 — 이번엔 건드리지 않음, 후속 과제)
SENS_MAG_RATE:  P1=15.0  P2=200.0          (비대칭 — 이번엔 건드리지 않음, 후속 과제)
EKF2_GPS_CTRL:  P1=7  P2=7                 (2026-09-04 진단 당시 P2=2였던 것이 이미 수정되어 있음)
```

`IMU_GYRO_RATEMAX`/`SENS_MAG_RATE` 비대칭은 실제로 있으나, 아래 조치만으로 목표(ΔYaw
최소화)를 달성해 이번에는 변경하지 않았다. 남겨진 후속 과제로 기록한다.

### 조치: MAG2(CAN1 RM3100) 우선순위 전환 — priority만 바꾸면 반영되지 않는다

두 보드 모두 기존 활성 소스는 `CAL_MAG1_PRIO=100`(GPS1 내장 IST8310)이었고, 이 상태의
ΔYaw(P2-P1) 기준값은 다음과 같았다(USB, 8초, 935 samples).

```text
baseline (MAG1 active): ΔYaw median = +10.844°, range 10.812°~10.901°
```

`CAL_MAG1_PRIO=0`, `CAL_MAG2_PRIO=100`을 PARAM_SET으로 두 보드에 적용하고 readback으로
확인했지만(둘 다 OK), **30초 이상 기다려도 ΔYaw는 거의 그대로였다**(11.5~11.6°대). 즉
`CAL_MAGn_PRIO`를 RAM에서만 바꾸는 것으로는 EKF2가 실제로 새 마그네토미터 인스턴스로
재선택되지 않는다. 과거 섹션들의 MAG2 재현성이 낮았던 것(-0.6°~-8° 사이에서 요동)은 일부
세션이 저장/재부팅 없이 측정했기 때문일 가능성이 있다.

해결책은 `MAV_CMD_PREFLIGHT_STORAGE`(param1=1, flash 저장) 후 `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`
(param1=1)으로 두 보드를 재부팅하는 것이었다. 재부팅 후 재연결해 `CAL_MAG1_PRIO=0`,
`CAL_MAG2_PRIO=100`이 유지된 것과 `SYS_STATUS`의 마그네토미터 `present/enabled/healthy`가
모두 `True`인 것을 확인했다.

```text
after save+reboot, 45s / 4442 paired samples:
ΔYaw(P2-P1) median = -0.006°, mean = -0.007°, range -0.015°~+0.005°
P1 yaw ≈ 0.004°~0.008°, P2 yaw ≈ -0.003°~0.000° (드리프트 없음, 살아있는 노이즈로 확인 — 값이 얼어붙은 것 아님)
```

두 보드 모두 disarm 상태에서 진행했고, arm 중이었다면 스크립트가 거부하도록 gate를 뒀다.
이번 조치로 변경한 파라미터는 `CAL_MAG0~3_PRIO` 네 개뿐이며 flash에 저장했다. 다른
파라미터(`SENS_MAG_RATE`, `IMU_GYRO_RATEMAX` 등)는 변경하지 않았다.

### 결론

같은 마운트에 붙은 P1/P2의 ΔYaw는 활성 마그네토미터 선택(`CAL_MAGn_PRIO`)에 의해
지배되며, GPS1 내장 IST8310(MAG1)은 보드마다 다른 헤딩 바이어스를 갖고 CAN1 RM3100
(MAG2)은 두 보드 간 거의 완벽히 일치한다. 단, **우선순위 변경은 반드시 PARAM_SAVE +
재부팅까지 마쳐야 EKF2 마그네토미터 선택에 반영된다.**

### 라이브 PARAM_SET 이력현상(hysteresis) 주의

재부팅 없이 PARAM_SET만으로 `SENS_MAG_RATE`/`IMU_GYRO_RATEMAX`를 바꿨다가 되돌린 뒤에도
ΔYaw가 원래 값으로 돌아오지 않았다(-0.006° → +1.12° → 되돌린 후 +1.26°). 이후 저장+재부팅으로
초기화하자 -0.017°로 재현됐고, 그 깨끗한 상태에서 같은 값을 다시 라이브로 바꿨을 때는 -0.016°로
영향이 없었다. 따라서 **센서/EKF2 관련 파라미터 비교 실험은 반드시 매 변경마다 저장+재부팅한
깨끗한 상태에서 측정**해야 하며, 라이브 변경 직후의 수치는 비교 근거로 쓰지 않는다.

### SENS_MAG_RATE / IMU_GYRO_RATEMAX 비대칭 해소 (2026-09-06)

P1을 P2와 같은 `SENS_MAG_RATE=200`, `IMU_GYRO_RATEMAX=800`으로 맞추고 저장+재부팅했다.
재부팅 직후 ΔYaw median `0.000°`(range -0.007°~+0.005°, 742 samples)로 비대칭 상태와 차이가
없었다. 두 보드의 현재 확정값은 `CAL_MAG0/1/3_PRIO=0`, `CAL_MAG2_PRIO=100`, `SENS_MAG_RATE=200`,
`IMU_GYRO_RATEMAX=800`, `EKF2_GPS_CTRL=7`, `EKF2_MAG_TYPE=1`, `SENS_BOARD_ROT=0`이며 flash에 저장돼
있다. 장시간 드리프트 관찰 결과는 아래 11절에 기록한다.

## 11. Telemetry Radio(공중 SiK) 복구 조사 — 소프트웨어 경로 없음 (2026-09-06)

### 시도한 경로와 결과

1. **RF 경유 복구**: 불가능. ArduPilot SiK 저장소의 `Firmware/bootloader/bootloader.c`에는
   radio/tdm/freq_hopping 관련 코드가 전혀 없다(순수 UART 부트로더). 부트로더에 멈춘 공중
   라디오는 RF 링크를 만들 수 없으므로 지상 라디오에서 어떤 명령을 보내도 닿지 않는다.
   `AT&UPDATE`(`Firmware/radio/at.c`)는 **로컬** 라디오를 부트로더로 보내는 명령일 뿐이다.
2. **PX4 SERIAL_CONTROL passthrough 경유**(`uploader.py --mavlink --mavport N`): 이 펌웨어에서
   불가능. `uploader.py`는 pymavlink `MavlinkSerialPort`로 SERIAL_CONTROL device 0~5(TEL1/TEL2/
   GPS1/GPS2/TEL3/TEL4)에 바이트를 relay하는데, PX4 쪽 처리(`mavlink_receiver.cpp
   handle_message_serial_control`)는 `CONFIG_DRIVERS_SERIALPASSTHROUGH`가 빌드된 경우에만 device
   0~5를 다루고, 아니면 SHELL(10) 외 모든 device를 조용히 무시한다. P1 NuttShell에서 직접
   확인한 결과 `serialpassthrough status` → `nsh: serialpassthrough: command not found`,
   즉 **현재 실행 중인 stable PX4 v1.17.0 FMUv6X 펌웨어에는 이 모듈이 없다**. (참고: 로컬에
   있는 최신 PX4 main 소스와 `boards/auterion/fmu-v6x/default.px4board`에는
   `CONFIG_DRIVERS_SERIALPASSTHROUGH=y`가 있어 착오를 일으킬 수 있으니 실행 중인 보드의
   `nsh` 응답을 기준으로 판단한다.)
   - 시도 중 `MAV_0_CONFIG=101`(TELEM1=`/dev/ttyS6`) mavlink 인스턴스가 같은 UART를 점유하는
     것이 원인일 가능성을 배제하기 위해 P1에서 `MAV_0_CONFIG=0`으로 끄고 재부팅해 재시도했으나
     동일하게 무응답이었고, 이후 `101`로 원복했다.
   - pymavlink 부작용: `MavlinkSerialPort`는 `MAVLINK20` 없이 v1 파서로 연결하므로 같은 메시지가
     v1/v2로 섞이면 `mavutil.add_message`에서 `'NoneType' object does not support item
     assignment`로 크래시한다. 우회는 `os.environ["MAVLINK20"]="1"` + `add_message` 관용 패치.
3. **NuttShell 경유**: SHELL(10)은 동작하지만 텍스트 콘솔이라 SiK 부트로더의 바이너리 sync/
   program 프로토콜을 UART로 중계할 수단이 없다.

### 결론과 필요한 물리 작업

P1/P2 공중 SiK 라디오(ATI5 기준 `RFD SiK 2.0 on HM-TRP`, 915 MHz, NETID 25/26) 복구는
**라디오 모듈의 TX/RX/GND에 USB-TTL(3.3V) 어댑터를 직접 연결**한 뒤 다음을 실행해야 한다.

```text
python3 SiK_src/Firmware/tools/uploader.py --port /dev/ttyUSBx radio~hm_trp.ihx
```

펌웨어 이미지는 `https://firmware.ardupilot.org/SiK/stable/radio~hm_trp.ihx`(SiK 2.2,
commit 4479658)이며 유효한 Intel HEX(101,122 bytes)임을 확인했다. 라디오가 이미 부트로더에
있으므로(solid red) `+++`/`AT&UPDATE` 단계 없이 부트로더 sync가 바로 잡혀야 한다. 복구 후에는
NETID(P1=25, P2=26), `AIR_SPEED=64`, `MAVLINK=1`, 915~928 MHz 대역을 지상 라디오와 동일하게
다시 설정하고 공중 라디오 전원을 완전히 재인가해야 링크가 살아난다(7절 참조).

이 작업이 끝나기 전까지 P1/P2 운용은 USB(`/dev/ttyACM*`, sysid로 식별) 또는 Ethernet만
가능하다.

2026-09-07 기준으로 P1/P2/Drone 공중 라디오는 다시 정상 동작하며 Telemetry Radio 연결이 확인됐다.
13절을 참조한다.

## 12. 정정: 10절의 "ΔYaw≈0°"는 mag 미융합 상태였음 — 원인과 실제 수정 (2026-09-06)

### 무엇이 잘못됐나

10절에서 `CAL_MAG0/1/3_PRIO=0`, `CAL_MAG2_PRIO=100`으로 바꾸고 재부팅한 뒤 P1/P2 yaw가 모두
`0.00°` 부근에서 ΔYaw≈0°로 나온 것은 **정렬이 된 결과가 아니라 EKF2가 자력계를 전혀 받지 못해
초기 yaw=0에서 자이로 적분만 하고 있던 상태**였다. NuttShell(SERIAL_CONTROL dev 10)로 직접 읽은
근거:

```text
estimator_status_flags: cs_yaw_align=False cs_mag_hdg=False cs_mag=False (P1, P2 동일)
estimator_status:       mag_device_id=0  mag_strength_gs=nan
sensors status (MAG):   validator best: -1, 등록된 mag sensor 0개, "MAG 8944643 EN: 1"만 표시
listener vehicle_magnetometer / sensors_status_mag: never published
listener sensor_mag -i 0..3: 4개 인스턴스 모두 실시간 데이터 있음 (RM3100은 instance 3)
```

이 상태에서 관찰된 "드리프트"(P2만 +0.071°/30 s ≈ +8.5°/h, P1 평탄, IMU 온도 불변, raw mag
방위각 불변)는 자력계나 열이 아니라 **P2 자이로 Z 잔여 바이어스(≈0.04 mrad/s)의 자유 적분**이었다.

### 원인: 다른 CAL_MAG 슬롯을 priority 0으로 두면 상위 인덱스 mag가 voter에 등록되지 않는다

`src/modules/sensors/vehicle_magnetometer/VehicleMagnetometer.cpp` `Run()`:

```text
if (_calibration[uorb_index].enabled()) {          // priority > 0 일 때만
    if (!was_advertised) {
        if (uorb_index > 0) { _voter.add_new_validator(); }   // 인스턴스가 처음 나타날 때만 추가
        ...
    }
    _voter.put(uorb_index, ...);
}
```

validator는 "처음 나타난 시점에 enabled인 인스턴스"에 대해서만 하나씩 추가된다. sensor_mag
instance 0(내장), 1(GPS1), 2(RM3100 backup)를 prio 0으로 끄면 validator가 index 0 하나뿐이고,
instance 3(RM3100 primary)의 `put(3, …)`은 존재하지 않는 validator로 가서 조용히 버려진다. 결과:
mag 선택 불가 → `vehicle_magnetometer` 미발행 → EKF2 mag 없음 → yaw 미정렬.
9/4(`25/50/100/75`), 9/5(`1/1/100/1`)에 정상이었던 이유가 이것이다.

**규칙: 사용하지 않는 CAL_MAGn_PRIO는 0이 아니라 1로 둔다.** (0은 "비활성"이며 위 부작용을 만든다.)

### 조치와 실측

1. P2를 `CAL_MAG0/1/3_PRIO=1`, `CAL_MAG2_PRIO=100`으로 저장+재부팅 → `sensors status` validator
   `best: 3`, EKF `cs_yaw_align=True cs_mag_hdg=True mag_device_id=8944643`,
   `mag_strength_gs=0.474`(WMM ref 0.496, `EKF2_MAG_CHECK=1` 통과). P1도 동일 적용 →
   `mag_device_id=8944899`, 융합 확인.
2. 두 보드 모두 RM3100 융합 상태의 **실제 ΔYaw = -6.54°**(P1 -161.84°, P2 -168.39°, 20 s/2003
   samples, range -6.562°~-6.517°). 9/5 기록(-7.28°/-8.25°)과 같은 수준이며 RM3100 개별 바이어스
   (offset 0 / scale 1, 미보정)가 원인이다.
3. 파라미터만으로 상수 오차 제거: 외부 mag 장착 yaw 오프셋 `CAL_MAG2_YAW`(deg)를 P2에 적용.
   - `+6.54` → ΔYaw `-15.07°` (부호 반대, 이동량 -8.5°: hard-iron 미보정 벡터라 1:1이 아니라 약
     1.3배로 반영됨)
   - `-5.0` → **ΔYaw `+0.057°`** (P1 -161.83°, P2 -161.77°, 20 s/1936 samples,
     range +0.015°~+0.081°), 두 보드 모두 실제 헤딩·mag 융합 상태.
   - 변경마다 PARAM_SAVE + 재부팅 후 측정했다.

주의: `CAL_MAG2_YAW`는 **현재 장착 방향에서의 상수 오차만** 상쇄한다. hard-iron/scale이 미보정이면
헤딩이 바뀔 때 오차가 다시 달라질 수 있으므로, 운용 전 QGroundControl RM3100 재보정이 근본 해결이며
그 뒤에는 `CAL_MAG2_YAW`를 0으로 되돌리고 ΔYaw를 재측정해야 한다. `CAL_MAG2_YAW`≠0을 넣으면 PX4가
`CAL_MAG2_ROT`를 자동으로 `100`(ROTATION_CUSTOM)으로 바꾸는데 이는 정상이다.

### 잔여 흔들림의 원인: EKF2_HEAD_NOISE=0.3 rad 기본값

`1/1/100/1` + `CAL_MAG2_YAW=-5.0` 상태에서 15분 관찰(60 s 간격, USB):

```text
P2-P1: 0.126 0.137 0.153 0.234 0.208 0.213 0.213 0.238 0.255 0.213 0.231 0.220 0.275 0.373 0.511
P2 raw mag 방위각: 15분 내내 152.4~152.6° (±0.06°) — 자력계는 안정
P2 raw zgyro 평균: ±0.07 mrad/s 요동 (≈0.24°/min 상당), P1은 ±0.03
IMU 온도: P1 35.7→36.1°C, P2 34.9→35.5°C (히터 비활성, SENS_EN_THERMAL=-1)
```

즉 자력계 측정은 고정인데 EKF yaw(P2)가 자이로 Z 바이어스 요동을 따라 0.1~0.3° 흔들리고 되돌아오는
데 수 분이 걸린다. 원인은 `EKF2_HEAD_NOISE=0.3 rad`(≈17°) 기본값이 mag heading 보정 게인을 매우 작게
만들어 yaw가 사실상 자이로 적분을 따르기 때문이다. 정지 벤치에서 자력계가 안정하므로
**`EKF2_HEAD_NOISE=0.05`**(양쪽)로 낮춰 yaw가 mag heading을 단단히 따르게 했다.

- `EKF2_HEAD_NOISE=0.05` 적용·재부팅 후 ΔYaw `-0.625°` — 느슨한 추종에서 +0.057°였던 값이 "실제 mag
  heading 차이"로 드러남. 기울기 -1.29°/°로 `CAL_MAG2_YAW`를 재조정:
  `-4.52` → `-1.244°`(방향 오판), **`-5.48` → `-0.004°`**(P1 -161.90°, P2 -161.88°, 20 s/1937 samples,
  range -0.041°~+0.018°).

### 최종 확정 파라미터 (flash 저장, 2026-09-06)

```text
P1: CAL_MAG0/1/3_PRIO=1  CAL_MAG2_PRIO=100 (ID 8944899)  CAL_MAG2_YAW=0
P2: CAL_MAG0/1/3_PRIO=1  CAL_MAG2_PRIO=100 (ID 8944643)  CAL_MAG2_YAW=0 / CAL_MAG2_ROT=0 (트림 제거, 아래 항목 참조)
공통: EKF2_HEAD_NOISE=0.02  EKF2_GYR_B_NOISE=0.005  CAL_GYRO1_PRIO=75  CAL_ACC1_PRIO=75 (IMU #1 2818066 고정)
      SENS_MAG_RATE=200  IMU_GYRO_RATEMAX=800  EKF2_MAG_TYPE=1  EKF2_GPS_CTRL=0(아래 "리그 이동 후" 항목)  MAV_0_CONFIG=101  SENS_BOARD_ROT=0
```

최종 설정 재부팅 후 5분: ΔYaw 0.033~0.048°, P1/P2 각각의 yaw 변화 ±0.012° 이내, 양쪽 선택 자이로
`2818066 (1)` 확인.

### 리그 이동 후 mag heading 융합 중단 — HEAD_NOISE 게이트와 GPS aiding의 상호작용 (2026-09-06)

운영자가 리그를 움직인 뒤 ΔYaw가 0.05° → 1.96° → 2.34°로 계속 벌어졌다. 양쪽 모두
`cs_mag_hdg=False`, `cs_mag_heading_consistent=False`, P2는 `starting_gps_fusion=True`였고 raw mag
방위각은 고정인데 EKF yaw만 자이로 적분으로 흘렀다(12절 초반과 같은 메커니즘).

원인(`src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp`):

```text
493: mag_heading_consistent = (|innov_lpf| < EKF2_HEAD_NOISE) && (|innov| < EKF2_HEAD_NOISE)
181: heading 융합 유지 조건 = mag_heading_consistent || !isNorthEastAidingActive()
```

즉 `EKF2_HEAD_NOISE`는 측정 노이즈이면서 동시에 **heading 일관성 게이트**다. 0.02 rad(1.15°)로 줄인 상태에서
GPS 위치/속도 aiding이 활성화되면(양쪽 GPS 검사 통과 후), 리그 회전 등으로 innovation이 1.15°를 넘는 순간
heading 융합이 중단되고, 융합이 없으니 innovation이 다시 줄지 않아 복구되지 않는다. 벤치 실험 중에는
양쪽 `gps_checks_passed=False`(NE aiding 없음)라 이 게이트가 작동하지 않아 드러나지 않았다.

조치: **`EKF2_GPS_CTRL=0`**(양쪽)으로 NE aiding을 끄면 181행 조건이 항상 참이 되어 heading 융합이 게이트되지
않는다. D 실험에서 GPS 융합 유무는 yaw에 영향이 없었고, Workstation의 `GLOBAL_POSITION_INT`는 optional
evidence이며 RTK baseline은 raw GPS(`GPS_RAW_INT`/`GPS2_RAW`)를 쓴다. 대안은 HEAD_NOISE를 0.1~0.3으로
되돌리는 것(게이트 5.7~17°)이지만 흔들림이 0.1~0.5°로 커진다. 이동하는 기체에 같은 설정을 쓸 때는
반드시 이 트레이드오프를 다시 평가한다. GPS_CTRL=0이면 WMM 기준값이 없어(`mag_strength_ref_gs=nan`)
편각은 저장된 `EKF2_MAG_DECL=-8.03°`, 세기 검사는 0.45 G±0.4 기본값으로 동작한다.

### CAL_MAG2_YAW 트림 제거 (2026-09-06, 운영자 결정)

리그 이동 후 트림값이 자세에 종속적임이 확인되어(-5.48 기준 자세에서 0.0°, 이동 후 -0.41°) 운영자 결정으로
P2 `CAL_MAG2_YAW=0`, `CAL_MAG2_ROT=0`으로 되돌렸다. 저장·재부팅 후 mag 융합 유지(`cs_mag_hdg=True`),
**ΔYaw = -7.29°**(P1 -161.18°, P2 -168.47°, 20 s/1926 samples, range -7.310°~-7.284°). 이것이 현재
자세에서 두 RM3100의 순수 차이이며, 9/5 기록(-7.28°)과 일치한다. 상대/절대 yaw 보정은 이제 파라미터가
아니라 RM3100 보정 또는 RTK baseline 방위각 기반 측정 방식으로 해결한다.

### 절대 방위 오차 (2026-09-06)

운영자가 리그의 실제 방향을 약 **187°**(남에서 서쪽으로 7°)로 확인한 시점에 두 보드의 ATTITUDE.yaw는
`-161.9°`(= 진방위 198.1°, `EKF2_MAG_DECL=-8.03°` 적용값)였다. 즉 P1/P2는 서로 0.05° 안에서 일치하지만
**절대 방위는 약 +11° 치우쳐** 있다. 원인은 RM3100 hard-iron/scale 미보정(측정 자기 경사각 67.2° vs
한국 기준 약 53°)과 장착 회전 미보정(`CAL_MAG2_ROT=0`)이며, `CAL_MAG2_YAW` 트림은 P2를 P1에 맞춘
것이지 절대 방위를 맞춘 것이 아니다. 운영자 결정으로 절대 방위는 파라미터로 보정하지 않는다
(근본 해결은 QGC RM3100 보정 후 `CAL_MAG2_YAW=0` 복귀). 절대 방위가 필요한 계산은 Pixhawk yaw가 아니라
RTK P1→P2 baseline 방위각(`pixhawk_rp_gps_baseline_yaw` 방식)을 써야 한다.

파라미터 백업(2026-09-06T0939KST, USB `PARAM_REQUEST_LIST`, QGC overlay 형식):
`Pixhawk/P1/parameters-full-20260906T0939KST.params`(1173개),
`Pixhawk/P2/parameters-full-20260906T0939KST.params`(1183개), 그리고 이번 튜닝값만 담은
`Pixhawk/P1|P2/yaw-tune-20260906T0939KST.params`. 복원 시 overlay를 PARAM_SET → PARAM_SAVE → 재부팅. 남은 +0.035° 상수는 `CAL_MAG2_YAW`를 약 +0.03° 더 조정하면 없앨 수 있으나 노이즈
수준이라 그대로 두었다.

최종 설정으로 재부팅 후 10분 관찰(60 s 간격):

```text
P2-P1: 0.038 0.024 0.042 0.039 0.066 0.062 0.068 0.071 0.082 0.084   (범위 0.02~0.08°)
P1 yaw Δ: -0.03°, P2 yaw Δ: +0.01~+0.04° (둘 다 mag 방위각 ±0.07° 이내로 고정)
```

이전 설정(HEAD_NOISE 0.3)의 15분 0.13~0.51° 흔들림 대비 안정적이며, 선형 드리프트는 없다.

### 추가 파라미터 실험 (각각 저장+재부팅 후 5분, 60 s 간격)

| 실험 | 변경 | ΔYaw 범위(5분) | 판정 |
|---|---|---|---|
| 기준 | HEAD_NOISE 0.05 | 0.024~0.084° (10분, 서서히 벌어짐) | — |
| A | `EKF2_GYR_B_NOISE` 0.001→**0.005** (양쪽) | -0.011~+0.029° | 채택 — 자이로 바이어스 요동을 EKF가 더 빨리 흡수, 벌어짐 사라짐 |
| B | A + `EKF2_HEAD_NOISE` 0.05→**0.02** (양쪽) | -0.015~+0.002° | 채택 — 가장 조용함 |
| C | B + P2 IMU를 #2(3407882)로 (`CAL_GYRO2/ACC2_PRIO=75`) | -0.075~-0.052° | 기각 — 흔들림 폭은 같고 -0.06° 상수 오프셋만 생김(다른 IMU = 다른 프레임/미보정 offset) |

| D | B + `EKF2_GPS_CTRL` 7→0 (GPS 위치/속도 융합 제거, 양쪽) | 0.039~0.058° | 기각 — B와 동일, EKF 위치 출력만 잃음. mag 융합은 WMM 없이도 유지됨(`mag_strength_ref_gs=nan`, 기본 0.45 G ±0.4 검사). 7로 원복 |

| E | B + `SENS_MAG_RATE` 200→5 (RM3100 25 Hz 발행을 5샘플 평균, 양쪽) | 0.083~0.099° (폭 0.016°) | 기각 — 흔들림 폭 B와 동일. mag 노이즈는 병목이 아님. 200으로 원복 |
| F | B + `EKF2_HEAD_NOISE` 0.02→0.01 (양쪽) | -0.071~-0.023° (폭 0.048°) | 기각 — mag 노이즈가 yaw로 더 전달돼 폭이 3배. 0.02로 원복 |

| G | B + `EKF2_MAG_TYPE` 1→0 (Automatic, 양쪽) | 0.023~0.048° (폭 0.025°) | 동일 — 지상에서는 `cs_mag_hdg=True, cs_mag_3d=False`로 1과 같은 heading-only 동작. 1로 원복 |

`EKF2_MAG_TYPE` 0/1 차이는 **비행 중에만** 생긴다: 0은 이륙 후 3축 벡터 융합으로 전환해 body 고정
hard-iron을 학습하지만(미보정 RM3100엔 이점) 자기 간섭에 민감하고 전환 시 yaw 스텝이 생길 수 있다.
P1/P2가 정지 기준국이면 1을 유지한다.

### P1/P2 EKF2·SENS 파라미터 동일성 확인 (2026-09-06, USB PARAM readback)

| 파라미터 | P1 | P2 | 동일 | 비고 |
|---|---|---|---|---|
| `EKF2_GPS_CTRL` | 7 | 7 | O | bit0 위경도, bit1 고도, bit2 3D 속도 ON; **bit3 dual-antenna heading OFF** — GPS yaw 미사용 |
| `EKF2_MAG_ACCLIM` | 0.5 | 0.5 | O | 기본값 |
| `EKF2_HDG_GATE` | 2.6 | 2.6 | O | 기본값 |
| `EKF2_MAG_CHECK` | 1 | 1 | O | 세기 검사 ON (WMM ref 0.496 G, 실측 P1 0.515 / P2 0.474 G 통과) |
| `EKF2_MAG_CHK_STR` | 0.2 | 0.2 | O | 기본값 |
| `EKF2_MAG_CHK_INC` | 20.0 | 20.0 | O | 기본값 (CHECK bit1 OFF라 미사용) |
| `SENS_MAG_MODE` | 1 | 1 | O | 기본값 |
| `EKF2_MULTI_MAG` | 0 | 0 | O | 단일 EKF, voter가 고른 mag 하나 |
| `SENS_MAG_RATE` | 200 | 200 | O | 캡 (RM3100 실제 25 Hz) |
| `EKF2_MULTI_IMU` | 0 | 0 | O | 단일 EKF, IMU #1(2818066) 고정 |
| `SENS_IMU_TEMP` | 55 | 55 | O | `SENS_EN_THERMAL=-1`이라 히터 비활성, 실측 35~36 °C |
| `EKF2_MAG_GATE` | 3.0 | 3.0 | O | type 1에선 미사용 |
| `EKF2_MAG_TYPE` / `EKF2_HEAD_NOISE` / `EKF2_GYR_B_NOISE` | 1 / 0.02 / 0.005 | 동일 | O | 최종 채택값 |
| `EKF2_DECL_TYPE` | 3 | 3 | O | GPS/WMM 편각 사용·저장 |
| `EKF2_MAG_DECL` | -8.034926 | -8.034929 | △ | WMM 자동 저장값의 부동소수 차이, 실질 동일 |
| `GPS_YAW_OFFSET` / `EKF2_GPS_YAW_OFF` | 0 / 0 | 0 / 0 | O | — |

의도적으로 다른 값은 `CAL_MAG2_ID`(보드별 RM3100), `CAL_MAG2_YAW`(P2 -5.48), `CAL_MAG2_ROT`(P2 100 자동)뿐이다.
yaw 소스는 양쪽 모두 mag heading 하나이며 dual GNSS heading은 꺼져 있다.

재부팅 간 재현성: 같은 설정이라도 재부팅마다 상수항이 약 ±0.05° 범위에서 흩어진다(B +0.03, D +0.04,
E +0.085, F -0.05, 최초 -0.004). raw mag 방위각도 run마다 0.05~0.1° 달라지므로(P1 175.30~175.50°,
P2 152.2~152.6°) 이는 자력계/환경 측 변동이며 현재 파라미터 튜닝의 바닥이다. 따라서 `CAL_MAG2_YAW`를
0.03° 단위로 트림하는 것은 의미가 없다.

"EKF에서 다른 센서를 빼는" 관점의 정리: `EKF2_MAG_TYPE=1`(heading-only)과 GPS yaw 비트 미사용으로
yaw에는 이미 **선택된 mag heading + 선택된 IMU 자이로**만 들어간다. 다른 mag 인스턴스는 voter에서
선택되지 않으면 EKF에 전달되지 않고(prio 1은 failover 후보), baro는 고도만, GPS는 D 실험대로 yaw에
영향이 없다. 남는 결합은 accel(기울기 → mag 수평 투영)뿐이며 이는 IMU 고정으로 상수화된다.

C를 되돌리며 확인한 사실: `CAL_GYRO*_PRIO`가 모두 50이면 **재부팅마다 선택 IMU가 바뀔 수 있다**
(P2가 #1→#0으로 바뀜). IMU가 바뀌면 위처럼 ΔYaw에 수십 m° 오프셋이 생기므로, `CAL_MAG2_YAW`를
튜닝했을 때의 IMU #1(2818066)을 양쪽 모두 `CAL_GYRO1_PRIO=75`, `CAL_ACC1_PRIO=75`로 고정했다.

`EKF2_HEAD_NOISE=0.02`는 정지·저자기간섭 환경 기준이다. 비행/이동 중 자기 간섭이 큰 환경에서는
mag 노이즈가 yaw로 더 직접 전달되므로, 그런 운용에서 흔들림이 보이면 0.1~0.2 사이로 되돌려 재평가한다.

## 13. Telemetry Radio 링크 복귀 확인과 Workstation 라디오 endpoint 저장 (2026-09-07)

### 관찰 (10:36~10:46 KST)

11절 당시 부트로더(solid red)에 멈춰 있던 P1/P2 공중 SiK 라디오는 이 시점에 응용 펌웨어로
동작하고 있었다(복구 작업 자체는 이 세션에서 수행하지 않았고 방법도 확인하지 못했다). 지상
라디오 3개는 모두 `/dev/serial/by-id`에 있었고, Pixhawk 직접 USB(`ttyACM*`)와 Ethernet
(`10.0.0.2/.3` ping 100% loss)은 없었다.

| 지상 라디오 | 커널 포트 | 역할 | passive probe (57600, pymavlink) |
|---|---|---|---|
| `D30JNMVH` | `ttyUSB1` | P1 / NETID 25 | 실행 중 Workstation이 점유 중이라 probe 불가(multiple access) |
| `DU0E49VU` | `ttyUSB0` | P2 / NETID 26 | sysid 2 HEARTBEAT 4/10 s, 5/15 s; `RADIO_STATUS rssi=64 remrssi=64 noise=29 remnoise=50 rxerrors=5231` |
| `DU0E4O0F` | `ttyUSB2` | Drone / NETID 27 | sysid 3 HEARTBEAT 4/15 s, ATTITUDE 12, GPS_RAW_INT 8 |

같은 시간 실행 중이던 Workstation(`run.sh --maximized`, `run.log`)은 P1을 `D30JNMVH @ 57600`,
P2를 `DU0E49VU @ 57600`으로 열어 각각 expected sysid heartbeat를 받았고 두 side 모두
`liveness: LIVE`에 도달했다(P2 1회, P1 2회 기록). P2 RSSI 64는 9/2 기록(154~213)보다 크게 낮으므로
안테나/거리/전원 상태를 확인한다.

### LIVE ↔ QUALIFYING 반복의 의미

`workstation_core/mavlink_liveness.py`는 side에 활성 명령이 있으면(`_active_command`)
`QUALIFYING`(reason `ACTIVE_COMMAND`)으로 보고한다. Connect 직후 자동으로 시작되는
`Stable PX4 sensor_mag Shell polling`(2 s cadence, `listener sensor_mag -i N`)이 57600 라디오에서는
응답이 느려 거의 항상 활성이므로, 링크가 정상이어도 P1/P2가 `LIVE`와
`QUALIFYING; qualification=00:10/00:10`을 오간다. 이것은 링크 단절이나 heartbeat 손실이 아니다.
heartbeat/ATTITUDE가 실제로 끊기면 `ATTITUDE_STALE`, `RX_STALLED`, `INITIAL_RX_TIMEOUT`으로 나타난다.

### 포트 동시 접근 주의 (재현됨)

Workstation이 P2 Connect를 여는 순간 외부 raw probe가 같은 `ttyUSB0`을 열어
`오른쪽 sensor_mag Shell polling 실패: device reports readiness to read but returned no data
(device disconnected or multiple access on port?)`와 PARAM 작업 실패가 1회 발생했다. probe 종료 후
heartbeat가 다시 들어와 LIVE로 회복했다. `fuser`로 미리 확인해도 그 사이에 앱이 포트를 열 수 있으므로,
Workstation이 실행 중이면 라디오 포트에 raw probe를 붙이지 않는다(10절의 규칙을 반대 방향으로도 적용).

### Workstation 설정 저장

이 워크트리의 `.local/workstation_data/stereo_distance_settings.json`은 새로 생성된 파일이어서
`*_pixhawk_radio_device`가 모두 비어 있었고, P2/Drone Connect preflight가
`RADIO DEVICE MISSING: RADIO device is empty`로 실패했다(P1은 UI에서 후보를 골라 연결). Settings
workspace의 in-memory mirror는 Save 전까지 JSON을 바꾸지 않으므로 파일에는 반영되지 않았다.
2026-08-21 작업 복사본의 설정과 2026-09-02 세 링크 검증 기록의 값을 `load_settings → replace →
save_settings`로 옮겨 저장했다. 변경된 키는 아래 여섯 개뿐이다.

```text
left_pixhawk_transport/right/drone = radio (변경 없음), baud 57600 (변경 없음)
left_pixhawk_radio_device  = /dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30JNMVH-if00-port0
right_pixhawk_radio_device = /dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0E49VU-if00-port0
drone_pixhawk_radio_device = /dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DU0E4O0F-if00-port0
left/right/drone_pixhawk_radio_usb_serial = D30JNMVH / DU0E49VU / DU0E4O0F
```

재로드 후 세 device 모두 존재하고 서로 다른 커널 포트로 resolve되어 serial preflight 조건
(`RADIO READY`)을 만족했다. 앱 종료 시 `_shutdown_save_settings`가 메모리 값으로 파일을 다시 쓰므로,
실행 중인 앱에서 Drone 라디오 device를 선택하지 않고 종료하면 Drone 항목만 다시 비워질 수 있다.
P1/P2는 이미 메모리에 같은 값이 있어 유지된다. `settings_controller`가
`pixhawk_transport_schema_version=1`로 쓰고 `load_settings`가 매 기동마다 2로 재저장하는 것은
값에 영향이 없는 기존 동작이다.

### 남은 항목

RTCM/RTK, 장시간 soak, restart/Resume, Workstation-owned 자동 재연결은 여전히 `NOT_RUN`이다.
Ethernet 경로(P1 RX 장애, P2 eth0 DOWN)는 변하지 않았다.
