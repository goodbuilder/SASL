# P2 Ethernet / B-E LED 오류 진단 기록

- 진단 시각: 2026-09-04 02:18 KST
- 대상: P2 / Right Pixhawk
- Workstation: `10.0.0.10`, Ethernet NIC `enx0023a40a2e39`
- P2 목표 주소: `10.0.0.3/24`, MAVLink UDP `14551`, system ID `2`
- P2 USB: `/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00`
- PX4: `Release 1.17.0`, NuttX `11.0.0`

## 결론

P2 부트로더 고장은 확인되지 않았다. USB MAVLink heartbeat가 `sysid=2/compid=1`로
수신되고 `NuttShell`, `mavlink status`, `listener` 명령이 정상 실행되어 PX4 애플리케이션은
부팅되어 있다.

현재 B/E LED는 애플리케이션의 preflight/오류 상태를 표시하는 것으로 판단된다. PX4 공식
LED 문서에서 B/E는 부트로더에서는 점멸하고, 애플리케이션에서는 오류를 표시한다. LED 라벨은
보드에 따라 다를 수 있다.

- [PX4 LED meanings](https://docs.px4.io/main/en/getting_started/led_meanings)

Ethernet 연결 실패는 별도 문제다. P2 `eth0`가 `DOWN`이고 `ifup eth0`가 실패했으며, host에서
`10.0.0.3` ARP/ping/UDP 응답이 없었다.

## PX4에서 확인된 오류

### 1. 자력계 일관성 오류 — 확정

```text
sensor_preflight_mag.mag_inconsistency_angle = 2.31716 rad = 132.8 deg
COM_ARM_MAG_ANG = 60 deg
dmesg: Preflight Fail: Compasses 134° inconsistent
```

P2에는 `sensor_mag` 5개가 모두 게시되고 `error_count=0`이지만, 센서 간 방향/보정 결과가
약 133도 차이난다.

```text
instance 0: device_id=4395041, I2C:4 (0x10)
instance 1: device_id=396809,  I2C:1 (0x0E)
instance 2: device_id=396817,  I2C:2 (0x0E)
instance 3: device_id=8944643, UAVCAN node 0x7C
instance 4: device_id=8944387, UAVCAN node 0x7B
```

가능한 경계는 자력계 보정값, 외부 자력계 방향, 센서 우선순위, 주변 자기 간섭이다. PX4도
이 오류를 보정·방향·자기 간섭 문제로 분류한다.

### 2. GNSS 속도 정확도 오류 — 확정

```text
EKF2_REQ_SACC = 0.500 m/s
EKF2_GPS_CHECK = 2047 (GPS checks enabled)
dmesg: Preflight: GPS Speed Accuracy too low
```

현재 수신값:

| Receiver | fix | satellites | s_variance_m_s | eph | epv |
|---|---:|---:|---:|---:|---:|
| GPS1 | 3D | 17 | 0.725 | 2.780 m | 5.556 m |
| GPS2 | 3D | 15 | 1.036 | 3.398 m | 4.977 m |

두 수신기 모두 `EKF2_REQ_SACC=0.5`보다 속도 정확도 추정값이 크다. 3D Fix와 위성 수가
있다는 것만으로 PX4 GPS preflight가 통과하지는 않는다.

### 3. 위치 추정 비활성/유효하지 않음 — 설정 오류와 결과 상태

```text
EKF2_GPS_CTRL = 2
```

PX4 파라미터 정의상 bit 0은 위도/경도, bit 1은 고도, bit 2는 3D 속도 융합이다. 값 `2`는
고도만 융합하고 위도/경도와 속도 융합을 끈다.

- [PX4 EKF2_GPS_CTRL parameter](https://docs.px4.io/main/en/advanced_config/parameter_reference#EKF2_GPS_CTRL)

실측 결과:

```text
vehicle_local_position.xy_valid = False
vehicle_local_position.v_xy_valid = False
vehicle_local_position.dead_reckoning = True
vehicle_global_position = no current publication
health_report arming error = local_position_estimate,
  global_position_estimate, system, magnetometer
```

일반적인 P2 GNSS 위치·고도·속도 운용이 목표라면 `EKF2_GPS_CTRL=7`을 검토할 수 있으나,
운용 프로파일을 확인하기 전에는 변경하지 않는다.

### 4. Arming/preflight 종합 상태

```text
commander check: Preflight check: FAILED
commander status: Disarmed, failsafe: no
vehicle_status.pre_flight_checks_pass: False
vehicle_status.gcs_connection_lost: True
vehicle_status.manual_control_signal_lost: True
health_report.arming_check_warning_flags: remote_control
health_report.arming_check_error_flags: local_position_estimate,
  system, global_position_estimate, magnetometer
```

`CBRK_USB_CHK=197848`은 USB check circuit-breaker key와 일치하므로, 현재 B/E의 주원인을
USB 연결 자체로 단정하지 않는다. GCS/수동조종 미수신은 현재 Workstation P2 Ethernet이
끊긴 상태에서 발생한 warning/context다.

## Ethernet 상태

P2 USB Shell readback:

```text
ifconfig eth0
eth0 Link encap:Ethernet HWaddr 6e:bf:5c:8d:b8:01 at DOWN
inet addr:10.0.0.3 DRaddr:10.0.0.10 Mask:255.255.255.0

ifup eth0
ifup eth0...Failed
```

P2 `mavlink status`의 Ethernet instance는 존재하지만 실제 통신은 없다.

```text
UDP local/remote port: 14551/14551
Broadcast: YES
tx: 0.0 B/s
rx: 0.0 B/s
```

Workstation 측:

```text
route: 10.0.0.3 dev enx0023a40a2e39 src 10.0.0.10
neighbor 10.0.0.3: FAILED
ping 10.0.0.3: 100% packet loss
UDP 14551: no Workstation listener after failed connect
```

따라서 `14551` 파라미터보다 P2 Ethernet PHY/케이블/스위치 포트/초기화 경계를 먼저
확인해야 한다.

### 부팅 파일 문법 문제

현재 P2 `/fs/microsd/etc/extras.txt`:

```text
ifup eth0
ifconfig eth0 10.0.0.3 netmask 255.255.255.0 up
```

PX4 NuttX `ifconfig` 도움말에는 `up` 인자가 없으며, 동일 명령을 실행했을 때 다음 결과가
확인됐다.

```text
nsh: ifconfig: argument invalid
```

이 줄은 Workstation 생성 코드와 관련 문서/샘플에서도 수정 대상이다. 다만 `ifup eth0` 자체도
현재 `Failed`이므로 trailing `up`만 제거한다고 Ethernet 링크 문제가 자동 해결된다고
판정하지 않는다.

## QGroundControl 포트 충돌

진단 중 QGroundControl이 다음 리소스를 먼저 점유했다.

```text
/dev/ttyACM0
0.0.0.0:14551
```

Workstation P2 Ethernet endpoint도 `udpin:0.0.0.0:14551`이므로 QGC와 동시에 열 수 없다.
QGC 종료 후 `/dev/ttyACM0`와 UDP `14551`은 해제됐고, 이 충돌은 해소됐다. 이후 P2 연결
실패는 QGC 포트 충돌이 아니라 P2 `eth0=DOWN` 상태로 남았다.

## Workstation 현재 상태

진단 시점의 Workstation 상태:

- Camera L/R: 앱 내부 GStreamer에서 LIVE, 약 `17.9 / 11.7 Hz`
- P1: Ethernet MAVLink LIVE, `10.0.0.2:14550`, `sysid=1`
- P2: Ethernet 선택 상태이나 DISCONNECTED, `10.0.0.3:14551`, `sysid=2` 미수신
- P2 radio: Ethernet 전환 후 해제
- 두 카메라 + P1/P2 모두 Ethernet 장시간 안정성: **미검증**

## 미실행 조치와 다음 순서

1. P2를 반드시 disarmed 상태로 유지하고 QGC를 종료한다.
2. P2 Ethernet 케이블과 스위치 포트를 known-good 조합으로 확인하고 링크 LED를 확인한다.
3. 링크가 살아난 뒤 P2 USB에서 `ifconfig eth0`와 `ifup eth0`를 재확인하고 Workstation P2
   Ethernet Connect를 실행한다.
4. P2 `EKF2_GPS_CTRL` 목표값을 운용 프로파일과 대조한다.
5. QGroundControl에서 자력계 방향/보정을 확인하고, 실제 사용하지 않는 자력계만 우선순위
   정책에 따라 정리한다. 무조건 자력계 preflight를 우회하지 않는다.
6. GPS/RTK 안테나·보정·시야를 확인해 `s_variance_m_s <= 0.5` 조건을 다시 검증한다.
7. P2 Ethernet heartbeat/ATTITUDE/GPS가 확인된 뒤 두 카메라와 P1/P2를 모두 Ethernet으로
   연결하고, 전환 동작을 포함한 장시간 raw packet/UI soak을 수행한다.

이 기록에서 실행한 것은 USB readback 및 runtime `ifup/ifconfig` 진단뿐이다. 파라미터 파일
쓰기, 펌웨어 변경, 재부팅은 실행하지 않았다.
