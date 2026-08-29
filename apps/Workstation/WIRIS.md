# WIRIS P1/P2 카메라 설정 및 스트림 튜닝 기록

이 문서는 `apps/Workstation`에서 사용하는 왼쪽/오른쪽 Workswell WIRIS Enterprise의
실장비 설정, Workstation 설정, 성능 기준선, 변경 및 복구 절차를 기록한다.

- 초기 기준 snapshot: `2026-08-24T00:02:11+09:00` (Asia/Seoul)
- 최신 상태 확인: `2026-08-27` 순차 read-only SDK GET
- 최신 변경/복구 확인: `2026-08-29` GOP 15 controlled A/B 후 양쪽 독립 readback
- 실장비 firmware: 양쪽 모두 `2.18.72`
- 목표: 카메라 화질을 최대한 보존하면서 양쪽 decoded rate를 지속적으로 `17 Hz` 이상 유지
- 절대 조건: visible encoder bitrate는 양쪽 모두 `0`으로 유지
- 공식 SDK 문서: <https://my.workswell.eu/link/latest/1653482886wf2Pr9x8SKHQFZ9PN7f9>

## 0. 현재 문제 요약 및 해결 범위 — 2026-08-29

한 줄 판정은 **좌/우 RTSP 연결과 decoded Preview 연속성은 PASS지만, 목표 처리율과 원본
H.264/RTP 무결성은 FAIL/UNVERIFIED**이다. Workstation은 손상 가능성이 있는 decoded frame도
계속 표시하는 영구 tolerant 계약이므로 `LIVE`와 `INPUT UNVERIFIED`가 동시에 나타날 수 있다.

2026-08-29 실장비 Preview의 제한된 관측 표본은 다음과 같다. 수치는 장기 성능 기준선이 아니라
문제를 계층별로 분리하기 위한 한 실행의 snapshot이다.

| 경계 | 관측 | 판정 |
|---|---|---|
| 연결/최신성 | 양쪽 동일 worker와 RTSP/TCP 세션 유지, 좌/우 frame age 약 `105-162/128-135 ms` | PASS |
| decoded rate | 좌/우 `13.5/14.6 Hz`; 목표는 양쪽 지속 `17 Hz` 이상 | FAIL |
| 30초 RTP 연속성 | 좌 `+8 events/+37 missing`, 우 `+2 events/+40 missing` | FAIL |
| NIC/TCP | 1 Gb/s link에서 약 `123.5 Mb/s`, NIC RX error/drop `0`, TCP Recv-Q `0` | PASS |
| decoder | 양쪽 실제 `nvh264dec`, NVIDIA decoder 사용률 약 `13%`, fallback/stall 없음 | PASS |
| 압축 원본 | 양쪽 `INPUT UNVERIFIED`; short raw Annex-B A/B에서 NAL5/IDR `0`, 장기 runtime candidate만 관측 | FAIL |

### 문제와 원인

- H.264 reference chain 중 RTP sequence gap이 생기지만, FW `2.18.72` visible stream에는
  실용적으로 bounded된 random-access 복구점이 확인되지 않았다. Short raw capture에서는 NAL5가
  없었고, 장기 runtime tracker에서만 약 39분 뒤 side별 IDR candidate가 한 번 관측됐다. Decoder
  error concealment로 화면은 계속 `LIVE`일 수 있으나 이후 픽셀의 원본 무결성은 보증할 수 없다.
- `SEIR VISIBLE 0/1/2/3`은 SDK `OK`와 일치하는 `GEIR` readback을 반환했지만, decoder를 제외한
  원시 H.264 artifact는 모두 `NAL5=0`이었다. 설정 readback은 실제 IDR 송출 증거가 아니다.
- RTP sequence probe는 `rtph264depay`의 sink, 즉 Workstation의 H.264 보존 queue, decoder,
  Preview/YOLO보다 앞에 있다. 따라서 관측된 gap은 GUI/YOLO queue가 만든 것이 아니다.
- 현재 live gap 중 카메라 sender omission과 `rtspsrc latency=100 ms`의 late-drop 비중은 완전히
  분리되지 않았다. 그러나 여유 있는 NIC/GPU, zero NIC drop, 독립 raw/MP4의 reference
  discontinuity와 zero-IDR 결과를 합치면 지배적인 결함 경계는 WIRIS encoder/RTSP
  packetizer 또는 firmware로 판단한다.

### NAL5와 IDR의 의미

H.264 NAL unit의 첫 header byte는 다음 8-bit 형식으로 구성된다.

```text
forbidden_zero_bit (1 bit) | nal_ref_idc (2 bits) | nal_unit_type (5 bits)
```

Annex-B에서는 `00 00 01` 또는 `00 00 00 01` start code 직후의 header byte에 대해
`header & 0x1f`를 계산하면 NAL type을 얻는다.
[RFC 6184](https://www.rfc-editor.org/rfc/rfc6184.html)는 `nal_unit_type=5`를 IDR picture에
속하는 coded slice로 정의한다.

| NAL type | 의미 | 이 문제에서의 해석 |
|---:|---|---|
| `1` | non-IDR picture의 coded slice | slice가 intra이고 FFprobe가 `I`로 표시해도 IDR/reset 보장은 없음 |
| `5` | IDR picture의 coded slice | 이전 picture에 대한 inter prediction 의존성을 끊는 표준 random-access 경계 |
| `6` | SEI | `recovery_point` message를 포함할 때만 별도의 점진적 복구 의미를 가질 수 있음 |
| `7` | SPS | 해상도·profile 등 sequence decode parameter; reference picture를 초기화하지 않음 |
| `8` | PPS | picture decode parameter; reference picture를 초기화하지 않음 |
| `9` | Access Unit Delimiter | access-unit 경계 표식이며 random-access 보장이 아님 |

IDR picture를 decode하면 이후 picture들은 IDR 이전 picture를 inter prediction reference로
사용하지 않고 decode할 수 있다. 따라서 RTP loss로 reference chain이 손상된 뒤 검증 가능한
NAL5는 decoder가 과거 손상 상태를 버리고 다시 시작할 수 있는 명확한 경계다. 반면 일반
NAL1 I-picture는 화면 내부 예측만 사용했더라도 non-IDR picture이므로, `pict_type=I`만으로
random access 또는 이후 reference 무결성을 증명할 수 없다. `SPS + PPS + NAL1 I` 조합도
표준상 IDR로 승격되지 않는다.

큰 H.264 NAL unit은 RTP에서 FU-A 등으로 분할될 수 있다. 이때 wire packet의 outer type만 보면
fragment type으로 보이지만, `rtph264depay`가 fragment를 재조립한 Annex-B에서는 원래 type 5
header가 다시 나타나야 한다. 현재 검사는 decoder 이전의 depayloaded Annex-B start code/NAL
header를 직접 세었으므로 RTP fragmentation 때문에 NAL5를 놓친 결과가 아니다. 현재 두
FW `2.18.72` visible artifact에는 NAL1/7/8/9는 존재하지만 NAL5가 없고, 검증 가능한
recovery-point SEI도 없다. 이 판정은 두 실장비의 `/visible` stream에 한정하며 thermal stream
또는 다른 firmware까지 일반화하지 않는다.

2026-08-29 장기 Workstation runtime tracker는 GOP 30에서 좌/우 각각 `blocked=2324.466s`와
`2340.247s` 뒤 `h264_idr` recovery candidate를 한 번 보고했다. 해당 run 종료 뒤 양쪽
`GEIR VISIBLE=0`을 readback했지만, 애플리케이션 재시작으로 이전 run log가 교체됐고 raw NAL5
bytes도 저장되지 않았다. 따라서 이는 “영구 zero-IDR” 주장을 제한하는 observation이지만,
short raw artifact의 `NAL5=0` 판정을 대체하는 독립 증거는 아니다. 현재 요구사항은 여전히
bounded periodic NAL5와 그 경계부터의 독립 decode다.

### 해결 가능 범위

- **앱에서 완전 해결 불가:** 수신 전에 사라진 RTP packet을 복원하거나 카메라가 만들지 않은
  IDR을 합성할 수 없다. 근본 해결은 Workswell의 periodic NAL5/force-IDR 지원 firmware 또는
  정상 random-access stream을 제공하는 대체 송출 경계가 필요하다.
- **앱에서 완화 가능:** `rtsp_latency_ms`와 TCP/UDP를 통제된 A/B로 조정하고, 현재
  `Reference-safe (preserve 8)` queue를 유지하며, gap 후 side-local decoder 재생성 및
  SPS/PPS-backed intra 이후 clean probation을 둘 수 있다. 이는 보이는 손상 시간을 줄일 수
  있지만 non-IDR intra를 IDR로 승격하거나 source integrity를 PASS로 만들 수는 없다.
- **운영 안전성은 앱에서 선택 가능:** `INPUT UNVERIFIED` 구간을 계속 허용하는 현재 정책,
  Measurement를 차단하는 보수적 정책, 또는 원본 무결성이 필요한 경우 WIRIS FTP still로
  우회하는 정책 중 하나를 명시적으로 적용할 수 있다.
- 근본 해결의 합격 기준은 fresh raw RTSP에서 bounded periodic NAL5를 관측하고, 그 IDR부터 만든
  독립 decoder가 오류 없이 시작하며, dual-camera 장시간 시험에서 RTP gap 없이 양쪽 decoded
  rate가 지속 `17 Hz` 이상인 것이다.

### 문제 분리 방법

1. GUI의 per-side `LIVE`, frame age, decoded FPS, 실제 decoder와 `INPUT UNVERIFIED`를 확인했다.
2. worker PID, RTSP socket, RTCP Sender Report와 재시작/종료 로그로 연결 연속성을 확인했다.
3. NIC 처리량/error/drop, TCP queue/RTT, GPU decoder 사용률로 link·host·decoder 포화를 제외했다.
4. 실행 로그의 RTP sequence counter로 gap을 측정하고 source의 probe 위치를 따라 downstream
   Preview/YOLO queue와 분리했다.
5. SDK ACK/readback과 별개로 decoder-free Annex-B NAL header 및 FFprobe 결과를 검사해 해당
   short-capture 구간의 zero-IDR를 독립 확인했다.

## 1. 장치 및 네트워크

| 항목 | 왼쪽 / P1 | 오른쪽 / P2 |
|---|---:|---:|
| 카메라 IP | `10.0.0.231` | `10.0.0.232` |
| SDK TCP | `2240` | `2240` |
| RTSP TCP | `8554` | `8554` |
| FTP TCP | `21` | `21` |
| Visible RTSP | `rtsp://10.0.0.231:8554/visible` | `rtsp://10.0.0.232:8554/visible` |
| Thermal RTSP | `rtsp://10.0.0.231:8554/thermal` | `rtsp://10.0.0.232:8554/thermal` |
| Serial number | `20204-046-2505` | `20207-046-2507` |
| Article number | `WWE-US-640-SNL-256-LRF` | `WWE-US-640-FNL-128-LRF` |

PC 카메라망:

```text
Interface: enx588694fce07e
Camera-network address: 10.0.0.11/32
Host routes: 10.0.0.231/32, 10.0.0.232/32, preferred source 10.0.0.11
Link: 1000 Mb/s, full duplex, carrier up
MTU: 1500
```

초기 snapshot 시점의 `10.0.0.10/24`는 이후 Pixhawk 전용 RTL8156B NIC로 분리됐다. 현재 카메라
route의 source는 `10.0.0.11`이며 이 NIC는 기본 route를 소유하지 않는다.

## 2. 실장비 공통 상태

다음 값은 SDK `GET` 명령의 직접 응답이다.

| SDK 항목 | 왼쪽 | 오른쪽 | 의미 |
|---|---:|---:|---|
| `HIWS` | `OK` | `OK` | SDK 명령 응답 |
| `IACT` | `TRUE` | `TRUE` | Ethernet SDK 활성화 |
| `GETH` | `TRUE` | `TRUE` | Ethernet/RTSP mode |
| `GFWV` | `2.18.72` | `2.18.72` | firmware version |
| `GSPN` | `8554` | `8554` | RTSP server port |
| `GSSS` | `0` | `0` | stream slice-size 제한 없음 |
| `GTMS` | `ERR` | `ERR` | subnet-mask readback 미제공 |
| `GTGW` | `10.0.0.1` | `10.0.0.1` | 저장 gateway |
| `GMGW` | `FALSE` | `FALSE` | manual gateway 비활성 |
| `IRCR` | `FALSE` | `FALSE` | 카메라 자체 녹화 중 아님 |
| `GILC` | `SSD` | `SSD` | image 저장 위치 |

`GTMS=ERR`이므로 장비 subnet mask를 문서에서 임의로 성공 판정하지 않는다. Workstation의
의도값은 `255.255.255.0`이다.

## 3. Encoder 및 영상 설정

### Visible encoder — 2026-08-29 rollback 후 실장비 readback

| 항목 | SDK GET | 왼쪽 | 오른쪽 |
|---|---|---:|---:|
| Bitrate | `GEBR VISIBLE` | `0` | `0` |
| QP | `GEQZ VISIBLE` | `1` | `1` |
| GOP | `GEGS VISIBLE` | `30` | `30` |
| IDR interval | `GEIR VISIBLE` | `0` | `0` |
| Slice-size | `GSSS` | `0` | `0` |

`bitrate=0`은 constant-quality mode이며 QP가 유효한 조건이다. Bitrate에 대해서는 테스트 중
`SEBR` 명령을 보내지 않는다. 모든 변경 전후에 `GEBR VISIBLE=0`을 readback한다.
2026-08-29 GOP A/B 사전·rollback readback은 양쪽 visible `0/1/30/0`, `GSSS=0`,
`GETH=TRUE`, `IRCR=FALSE`였으며 이번 시험은 `SEIR`을 전송하지 않았다. 과거 `GEIR=1`은
설정 readback일 뿐이었고, 당시 bounded raw capture에서는 NAL type 5를 관측하지 못했다.

### Thermal encoder

| 항목 | SDK GET | 왼쪽 | 오른쪽 |
|---|---|---:|---:|
| Bitrate | `GEBR THERMO` | `0` | `0` |
| QP | `GEQZ THERMO` | `17` | `17` |
| GOP | `GEGS THERMO` | `16` | `16` |
| IDR interval | `GEIR THERMO` | `0` | `0` |

`GSSS/SSSS`는 visible/thermal 공통 stream slice-size일 수 있으므로 slice-size 시험은 thermal에도
영향을 줄 수 있는 변경으로 취급한다.

### 해상도, zoom 및 저장 옵션

| 항목 | 왼쪽 | 오른쪽 |
|---|---:|---:|
| Thermal RTSP resolution (`GTRE`) | `640 512` | `640 512` |
| HR camera (`GHRR`) | `4656x3496@10` | `4656x3496@10` |
| Visible zoom (`GZVV`) | index `0`, ratio `1.0` | index `0`, ratio `1.0` |
| Thermal zoom (`GZTV`) | index `0`, ratio `1.0` | index `0`, ratio `1.0` |
| Simultaneous zoom (`GZSM`) | `FALSE` | `FALSE` |
| Main camera (`GMCA`) | `THERMO` | `THERMO` |
| Thermal encoded save (`GVTE`) | `FALSE` | `FALSE` |
| Visible encoded save (`GVVI`) | `FALSE` | `FALSE` |
| Visible HR video save (`GVHR`) | `FALSE` | `FALSE` |
| Radiometric video save (`GVTH`) | `TRUE` | `TRUE` |

### Actual zoom readback UI

Calibration의 `Actual zoom readback`에는 다음 조작이 있다.

| Control | Target | SDK action | Result |
|---|---|---|---|
| `Set both` | 양쪽 | `SZVN <level>` | 완료 후 `GZVV` 자동 readback |
| `Zoom-in` | 양쪽 | `SZIN` | 완료 후 `GZVV` 자동 readback |
| `Zoom-out` | 양쪽 | `SZOT` | 완료 후 `GZVV` 자동 readback |
| `GZVV` | 양쪽 | `GZVV` | actual index/ratio 갱신 |

Zoom-in/out 버튼은 새 SDK 구현을 만들지 않고 기존 `advanced_send_zoom` worker를 사용한다. 명령은
GUI thread 밖에서 실행되며, 완료 뒤 예약된 query가 `actual_visible_zoom`을 갱신한다. 물리 zoom
정지나 좌/우 개별 조작이 필요하면 기존 `Advanced Zoom` 화면을 사용한다.

## 4. 상태/온도 snapshot

온도는 설정이 아니라 `2026-08-24T00:02:11+09:00` 시점의 관측값이다.

| 항목 | 왼쪽 | 오른쪽 |
|---|---:|---:|
| CPU (`GTCU`) | `58.356 °C` | `59.282 °C` |
| Thermal core (`GTIC`) | `41.600 °C` | `44.200 °C` |
| Internal (`GTIN`) | `58.250 °C` | `58.750 °C` |
| Fan power (`GFPW`) | `0.0` | `0.0` |

## 5. Workstation 저장 설정

Canonical settings file:

```text
<repository-root>/.local/workstation_data/stereo_distance_settings.json
SHA256: 4c0a19d8492334c1780e31081f035aeb1c34919747f937c413a8cc61fae173e6
```

WIRIS/RTSP 관련 저장값:

```text
interface_name: enx588694fce07e
camera_host_address: 10.0.0.11/32
left_rtsp_url: rtsp://10.0.0.231:8554/visible
right_rtsp_url: rtsp://10.0.0.232:8554/visible
left_control_host: 10.0.0.231
right_control_host: 10.0.0.232
left_control_port: 2240
right_control_port: 2240
wiris_control_timeout_s: 5.0
wiris_visible_encoder_bitrate: 0
wiris_visible_encoder_qp: 1
wiris_visible_encoder_gop: 90
wiris_visible_encoder_idr: 0
preview_width: 1280
rtsp_latency_ms: 100
rtsp_decoder: auto
stream_backend: gstreamer
rtsp_transport: tcp
yolo_event_queue_size: 128
yolo_frame_queue_size: 2
```

FTP는 양쪽 모두 enabled, port `21`, user `wiris-enterprise`, remote directory `/`로 저장되어 있다.
비밀번호는 이 문서에 기록하지 않는다.

### 장비와 Workstation의 정합성

최신 실장비 readback은 양쪽 `0/1/30/1`이다. 저장된 `0/1/90/0` encoder 값은 수동 작업을 위한 reference이며
장비 상태의 source of truth가 아니다. 실제 상태는 명시적 SDK GET readback으로 확인한다.
Workstation/Legacy startup, Auto-Connect, Enable, Reactivate는 저장값을 장비에 적용하지 않는다.

### Encoder-profile 재전송 방지 코드

현재 자동 RTSP 경로는 다음 계약을 사용한다.

1. `GETH=TRUE`이면 startup/Auto-Connect/Enable/Reactivate 모두 `SETH TRUE`를 보내지 않는다.
2. `GETH=FALSE`일 때만 `SETH TRUE`를 한 번 보낸다.
3. 자동 경로는 `SEBR/SEQZ/SEGS/SEIR`을 전송하지 않는다.
4. encoder 변경은 별도 승인된 수동 WIRIS command에서만 수행하고 전후 readback을 남긴다.

이미 실행 중인 Workstation은 Python code와 settings를 자동 reload하지 않는다. 이 변경 전에 시작된
프로세스는 완전히 종료한 뒤 다시 실행해야 보호 로직이 적용된다.

검증:

```text
Focused RTSP worker tests: 5 passed
Camera regression file: 44 passed
Camera/contract/recovery tests: 71 passed
Full Workstation suite after first guard: 999 passed, 1 skipped
Current full suite: 1466 passed, 1 skipped in 127.86 s
```

## 6. 성능 기준선

### 실제 Workstation camera pipeline 60초 시험

시험 조건:

```text
Left device:  bitrate 0, QP 1, GOP 90, IDR 3
Right device: bitrate 0, QP 1, GOP 90, IDR 2
Diagnostic decoder: nvh264dec
Diagnostic RTSP transport: UDP
Resolution: 1280x720
Warm-up samples excluded: 5
Valid one-second samples per side: 55
```

결과:

| 지표 | 왼쪽 | 오른쪽 |
|---|---:|---:|
| Median decoded Hz | `18.917` | `16.960` |
| Minimum decoded Hz | `13.934` | `13.970` |
| p05 decoded Hz | `14.970` | `14.927` |
| `<17 Hz` 구간 | `14/55` | `35/55` |

이 profile은 지금까지 시험한 값 중 화질과 처리율의 균형이 가장 좋았지만, 엄격한
`매 1초 구간 17 Hz 이상` 조건은 충족하지 못했다.

현재 Workstation 저장값은 `auto` decoder와 RTSP `tcp`이므로 위 UDP/NVIDIA 기준선과 다르다.
모든 non-zero slice-size 후보가 오른쪽 단독 RTP/AU boundary에서 기준보다 악화됐으므로 양쪽
동시/저장 backend 시험으로 승격하지 않았다. 최종 slice-size는 `0`이다.

### Visible GOP별 전체 실장비 측정 결과

Visible GOP로 실제 시험 또는 관측된 값은 `15`, `30`, `60`, `90`, `120`이다. 다음 표는 저장된
증거를 모두 합친 것이며, 조건이 다른 실행의 수치를 같은 모집단처럼 직접 비교하지 않는다.
특히 GOP `15/30`만 2026-08-29 같은 날 동일한 TCP/100 ms/NVIDIA 35초 probe로 직접 비교할 수
있다. GOP `90`의 정량값은 과거 UDP/NVIDIA 60초 기준선이고, GOP `60/120`의 원시 시계열은
보존되지 않아 당시 판정만 남긴다.

| GOP | 증거 조건·수준 | 왼쪽 결과 | 오른쪽 결과 | RTP/NAL 결과 | 판정 |
|---:|---|---|---|---|---|
| `15` | 2026-08-29, side별 35초 TCP/100 ms/NVIDIA, warm-up 5초 제외 | median `14.5`, min/max `11/20`, `<17 Hz` `19/30`, non-delta `33/509` | median `14.0`, min/max `11/20`, `<17 Hz` `23/30`, non-delta `35/472` | Left gap/missing `1/162`, Right `84/245`, 양쪽 IDR AU `0` | **FAIL** — 처리율 미달, Right continuity 급락 |
| `30` | 2026-08-29, GOP15 A/B의 동일 정량 기준선 | median `14.0`, min/max `10/20`, `<17 Hz` `25/30`, non-delta `18/500` | median `13.0`, min/max `10/19`, `<17 Hz` `27/30`, non-delta `13/455` | Left gap/missing `2/114`, Right `9/30`, 양쪽 IDR AU `0` | **FAIL(목표)** / 현재 운용값 — 15보다 continuity 우세 |
| `60` | 2026-08-24 이전 historical single-factor tuning; side·transport·duration·raw series 미보존 | 정량값 미보존 | 정량값 미보존 | 정량값 미보존 | **FAIL** — 엄격한 지속 `17 Hz` 하한 미달로 기록 |
| `90` | 60초 UDP/NVIDIA, warm-up 5초 제외; Left IDR setting 3, Right 2 | median `18.917`, min `13.934`, p05 `14.970`, `<17 Hz` `14/55` | median `16.960`, min `13.970`, p05 `14.927`, `<17 Hz` `35/55` | 이 기준선에는 GOP별 gap/NAL count 미보존 | **FAIL(목표)** — 과거 화질/처리율 균형 최선 |
| `120` | 2026-08-24 이전 historical single-factor tuning; raw series 미보존 | 정량값 미보존 | 정량값 미보존 | RTP marker rate 자체가 기준보다 감소한 것으로 기록 | **FAIL** — sender cadence 악화 |

2026-08-26 Right bitrate-cap 실패 중 QP/GOP가 `17/16`으로 바뀐 적은 있지만, bitrate와 Ethernet
mode까지 동시에 변하고 RTSP timeout/zero-byte가 발생한 **confounded side effect**이므로 GOP
`16` 시험으로 세지 않는다. Visible GOP `1`, `5`, `10`, `20`, `25` 및 `15` 미만의 다른 값은
시험하지 않았다. GOP 15에서 이미 encoded bytes가 약 `5%`, RTP packet이 약 `6-7%` 늘고 Right
gap이 급증했으므로 더 낮은 값으로 진행하지 않았다.

GOP `15/30`의 전체 one-second series와 rollback 증거는
`docs/changes/20260826-integrated-field-stability/evidence/wiris-visible-gop15-20260829.md`에 있다.
GOP `60/120`은 원시 artifact가 없어 이후 숫자를 소급 추정하지 않는다. 재비교가 필요하면 현재
probe 계약으로 한 대씩 fresh A/B를 수행해야 한다.

### 확인된 경계

- NVIDIA decoder는 입력 H.264 access unit을 거의 모두 decode했다. 주 병목은 decoder 앞이다.
- 오른쪽 UDP RTP는 약 `80-90 Mb/s`, 초당 약 `7,000-8,000` packet까지 관측됐다.
- TCP interleaved는 UDP보다 marker rate가 더 낮았다.
- QP `5`, `17`, `30`, `40`, GOP `15`, `30`, `60`, `90`, `120`, IDR `1`, `2`, `3`, `5`를 시험했다.
- QP/GOP/IDR만으로 양쪽의 엄격한 17 Hz 하한은 달성하지 못했다.
- GOP `120`과 QP `40`은 카메라 RTP marker rate 자체를 낮춰 탈락했다.

## 7. Slice-size 시험 계획과 판정

현재 양쪽 `GSSS=0`으로 최대 H.264 slice 크기에 제한이 없다. 대응 설정 명령은 `SSSS`다.
positive 값은 bit 단위로 해석되는 SDK 계약을 사용하며, 첫 후보는 Ethernet MTU 1500보다 작은
H.264 payload를 목표로 `10000 bit` (`1250 byte`)를 사용한다.

시험 순서:

1. Workstation Preview, 진단 RTSP client, 카메라 자체 녹화를 모두 중지한다.
2. 양쪽 `GEBR VISIBLE=0`, QP/GOP/IDR, `GSSS=0`을 다시 읽는다.
3. 한 카메라씩 `SSSS 10000`을 적용하고 최소 30초 기다린다.
4. `GSSS=10000`과 기존 bitrate/QP/GOP/IDR 불변을 readback한다.
5. 오른쪽 단독 RTP marker/AU/decoded rate와 packet rate를 측정한다.
6. 오른쪽 단독 boundary가 PASS일 때만 양쪽 실제 Workstation pipeline을 60초 이상 측정한다.
7. 실패 시 RTSP client가 없는 상태에서 `SSSS 0`으로 rollback하고 readback한다.

합격 기준:

```text
bitrate remains 0
no encoder-profile drift
no SDK/RTSP service loss
both sides: every post-warm-up 1-second decoded sample >= 17.0 Hz
```

## 8. 변경 및 복구 안전 규칙

- Encoder 설정 변경 시 카메라 RTSP가 일시 중단된다.
- 외부 스크립트와 Workstation이 동시에 SDK `2240`에 연결하지 않도록 한다.
- canonical JSON을 변경한 뒤 이미 실행 중인 Workstation은 새 값을 자동 reload하지 않는다. 반드시
  완전히 종료한 뒤 다시 실행해야 한다.
- 자동 RTSP 경로는 `GETH=TRUE`이면 `SETH TRUE`를 생략하고 encoder profile을 적용하지 않는다.
  `GETH=FALSE`일 때만 `SETH TRUE`를 한 번 보낸다. 수정 전에 시작된 프로세스에는 이 보호가
  없으므로 반드시 재시작한다.
- 변경 전 `IRCR=FALSE`와 활성 RTSP client 없음부터 확인한다.
- 한 번에 한 필드, 한 카메라만 변경하고 최소 15-30초 기다린다.
- `REBT`는 FW `2.18.72`에서 FTP만 남기고 SDK `2240`/RTSP `8554`를 재기동하지 못한 사례가
  반복되었으므로 테스트 복구 수단으로 사용하지 않는다.
- software reboot 실패 상태는 FTP `21=open`, SDK `2240=refused`, RTSP `8554=refused`로 나타난다.
- 이 상태는 물리 전원 OFF, 10초 대기, ON으로 복구한다.
- 물리 재부팅 후 QP/GOP는 유지됐지만 IDR은 `0`으로 돌아갔다. IDR은 SDK가 command-ready가 된
  후 마지막에 다시 적용한다.
- `GSSS`가 바뀐 채 서비스가 중단되면 물리 재부팅 후 `GSSS`부터 readback하고 시험을 재개한다.

### Visible GOP 30 현장 적용 — 2026-08-26

사용자 승인에 따라 양쪽 visible encoder의 GOP만 `90 -> 30`으로 변경했다. 목적은 RTP sequence
discontinuity 자체를 없애는 것이 아니라, 무결성 gate가 손상된 reference chain을 차단한 뒤 다음
clean non-delta intra를 기다리는 시간과 그에 따른 `STALE` 체류시간을 줄이는 것이다. 약
15-18 Hz 수신율에서 GOP 90의 명목 intra 주기는 약 5-6초지만 GOP 30은 약 1.7-2초다.

변경 전에는 Workstation camera child, RTSP client 및 encoded recording이 없음을 확인했다. 양쪽
모두 `HIWS=OK`, `IACT=TRUE`, `GETH=TRUE`, `IRCR=FALSE`, visible profile `0/1/90/0`을 readback한
뒤, 한 카메라씩 `SEGS VISIBLE 30 -> OK`와 독립 `GEGS VISIBLE -> 30`을 확인했다. 15초 안정화
후 최종 readback은 양쪽 `0/1/30/0`이며 bitrate, QP, IDR, Ethernet mode와 recording state는
바뀌지 않았다.

변경 후 5초 TCP/100 ms/NVIDIA RTSP probe는 오류 없이 1280x720 decoded frame을 왼쪽 60개,
오른쪽 37개 수신했다. 이 짧은 probe는 송출 복구 확인일 뿐 장시간 성능 합격이나 RTP omission
해결 증거가 아니다. gap 빈도와 복구시간, 화질, bandwidth를 별도로 관찰하고 문제가 커지면
한 카메라씩 `SEGS VISIBLE 90`으로 원복한 뒤 readback한다.

### Visible GOP 15 실장비 A/B — 2026-08-29

양쪽을 한 대씩 `30 -> 15 -> 30`으로 변경하고 각 profile을 TCP/100 ms/NVIDIA의 동일한 35초
probe로 비교했다. 첫 5초는 warm-up에서 제외했다.

| Side | GOP | Median Hz | `<17 Hz` | Non-delta AU | RTP gap/missing | Encoded bytes | IDR AU |
|---|---:|---:|---:|---:|---:|---:|---:|
| Left | 30 | 14.0 | 25/30 | 18/500 | 2/114 | 241,326,409 | 0 |
| Left | 15 | 14.5 | 19/30 | 33/509 | 1/162 | 253,543,247 | 0 |
| Right | 30 | 13.0 | 27/30 | 13/455 | 9/30 | 219,201,664 | 0 |
| Right | 15 | 14.0 | 23/30 | 35/472 | 84/245 | 229,778,432 | 0 |

GOP 15는 non-delta intra 빈도와 median을 조금 높였지만 어떤 side도 지속 17 Hz를 만족하지
못했다. 특히 오른쪽은 RTP gap `9 -> 84`, missing `30 -> 245`로 악화돼 **FIELD FAIL**이다.
15보다 낮은 값은 이 negative boundary 뒤 추가 부하 위험 때문에 시험하지 않았다. 양쪽은
visible `0/1/30/0`, `GSSS=0`, `GETH=TRUE`, `IRCR=FALSE`로 rollback/readback했고, 10초
post-rollback decode와 Workstation 좌/우 `LIVE`를 확인했다. 전체 one-second series와 절차는
`docs/changes/20260826-integrated-field-stability/evidence/wiris-visible-gop15-20260829.md`에 있다.

### Slice-size 실장비 시험 결과

#### 오른쪽 `SSSS 10000` — FAIL, rollback 완료

```text
Before:  GSSS=0, bitrate/QP/GOP/IDR=0/1/90/2
Set:     SSSS 10000 -> OK
Readback after 30 s: GSSS=10000, bitrate/QP/GOP/IDR unchanged
Observed RTP marker: 7-12 Hz
Observed complete AU / decoded: 0 Hz for 11 s
Terminal result: GStreamer Internal data stream error
Rollback: SSSS 0 -> OK, GSSS=0 readback
```

`10000 bit`는 약 `1250 byte`지만 현재 visible frame 크기에서는 프레임당 지나치게 많은
slice를 생성해 카메라 encoder/packetizer를 과부하시킨 것으로 판단한다.

#### 오른쪽 `SSSS 100000` — FAIL, 물리 복구 및 rollback 완료

```text
Before:  GSSS=0, bitrate/QP/GOP/IDR=0/1/90/2
Set:     SSSS 100000 -> OK
30 s readback: SDK connection reset
Service state: FTP 21=open, SDK 2240=refused, RTSP 8554=refused
Automatic recovery: failed
Physical power cycle: SDK recovered, GSSS=100000 persisted, IDR reset to 0
Rollback: SSSS 0 -> OK
IDR recovery: SEIR VISIBLE 2 -> OK
Final readback: GSSS=0, bitrate/QP/GOP/IDR=0/1/90/2, RTSP open
```

rollback 후에도 encoder runtime은 자동으로 기준 상태로 돌아오지 않았다.

```text
Post-rollback 15 s observation:
RTP marker average: 13.599 Hz
AU/decoded average including startup: 9.354 Hz
Steady one-second decoded samples: approximately 12-14 Hz
RTP bandwidth: approximately 61.8 Mb/s
```

따라서 `GSSS` readback 복원과 runtime encoder 복구는 별개다. Slice-size 변경 뒤 성능 비교는
반드시 물리 재부팅으로 동일한 초기 상태를 만든 뒤 수행한다.

#### 오른쪽 `SSSS 1000000` — FAIL, rollback 완료

```text
Before:  GSSS=0, bitrate/QP/GOP/IDR=0/1/90/2
Set:     SSSS 1000000 -> OK
Readback after 30 s: GSSS=1000000, bitrate/QP/GOP/IDR unchanged
Service state: SDK and RTSP remained available
Observed RTP marker: 8-14 Hz
Observed decoded: approximately 9-14 Hz after startup
Observed RTP bandwidth: approximately 49.4 Mb/s average
Rollback: SSSS 0 -> OK
Final readback: GSSS=0, bitrate/QP/GOP/IDR=0/1/90/2
```

`1000000 bit`는 `10000`보다 slice 수를 줄여 서비스 장애는 피했지만 marker/decoded rate를 더
낮췄다. `10000`, `100000`, `1000000` 세 후보 모두 기준보다 나쁘므로 FW `2.18.72`에서는
non-zero slice-size를 사용하지 않는다. 최종 slice-size는 양쪽 `0`을 유지한다.

#### 최종 물리 복구 — PASS

`SSSS 1000000` rollback 뒤 오른쪽 encoder runtime이 저하돼 물리 전원 OFF, 10초 대기, ON을
수행했다.

```text
After physical power cycle:
SDK command-ready: HIWS=OK
Persisted GSSS: 0
Persisted bitrate/QP/GOP: 0/1/90
Boot IDR: 0
IDR recovery: SEIR VISIBLE 2 -> OK
Final right profile: bitrate/QP/GOP/IDR/GSSS=0/1/90/2/0
RTSP port 8554: open
```

Workstation과 같은 GStreamer decode 경로
`rtspsrc(UDP, latency=100) -> rtph264depay -> h264parse(config-interval=-1) -> nvh264dec`에서
다음 boundary를 확인했다.

```text
H264 parsed caps: 1280x720, constrained-baseline, level 3.1
Decoder output caps: NV12, 1280x720, progressive, CUDA memory
Observed RTP session bitrate: approximately 75.5 Mb/s
Observed cumulative packets-lost in sample: 9
```

FFmpeg는 연결 직후 PPS 이전 프레임에서 오류를 보였지만, 실제 Workstation GStreamer 경로는
parsed H.264와 decoder raw output까지 도달했다. 최종 운용 검증은 GStreamer 결과를 사용한다.

양쪽 IDR을 공통 `2`로 맞춘 뒤 `nvh264dec -> videoconvert -> BGR appsink`에서 왼쪽과 오른쪽
모두 `1280x720` decoded sample 수신을 다시 확인했다.

#### 실행 중 Workstation의 stale profile 재적용 사고

canonical JSON을 `0/1/90/2`로 변경하기 전에 시작된 Workstation 프로세스가 계속 실행 중인
상태에서 Preview를 시작했다. 해당 프로세스는 메모리에 남은 과거 profile `0/1/30/1`을 사용했다.

```text
Observed after Preview start:
Left:  FTP open, SDK 2240 refused, RTSP 8554 refused
Right: bitrate/QP/GOP/IDR changed to 0/1/30/1 and RTSP remained connected
```

왼쪽은 물리 재부팅 후 `0/1/90/0`으로 돌아왔고, Workstation을 완전히 종료한 다음 IDR 2를
복구했다. 오른쪽은 GOP 90과 IDR 2를 순차 복원했다. 최종적으로 양쪽 `0/1/90/2`, `GSSS=0`,
FTP/SDK/RTSP open과 `1280x720` BGR decoded sample을 확인했다.

## 9. Readback 명령

```text
HIWS
IACT
GETH
GFWV
GSPN
GSSS
GEBR THERMO
GEQZ THERMO
GEGS THERMO
GEIR THERMO
GEBR VISIBLE
GEQZ VISIBLE
GEGS VISIBLE
GEIR VISIBLE
IRCR
GTRE
GHRR
```

설정 명령 중 이번 slice-size 시험에서 허용된 명령:

```text
SSSS 10000     # tested: FAIL
SSSS 100000    # tested: FAIL, SDK/RTSP service loss
SSSS 1000000   # tested: FAIL
SSSS 0         # final / rollback
```

현재 visible GOP와 rollback 명령:

```text
SEGS VISIBLE 30   # current
SEGS VISIBLE 90   # rollback
```

## 10. Change log

- `2026-08-29`: 양쪽 visible GOP `30 -> 15 -> 30` controlled A/B에서 cadence는 소폭 개선됐지만
  오른쪽 RTP gap/missing이 크게 증가해 GOP 15를 FIELD FAIL로 판정했다. 양쪽을
  `0/1/30/0`, `GSSS=0`으로 rollback하고 SDK/RTSP/Workstation Preview 복구를 확인했다. 같은 날
  장기 runtime tracker의 약 39분 후 side별 IDR candidate도 observation-only로 기록했다.
- `2026-08-29`: 지금까지 시험된 visible GOP `15/30/60/90/120`을 한 표로 통합했다. `15/30/90`은
  보존된 정량값을 기록하고, raw series가 없는 `60/120`은 당시 판정만 남겼다. Bitrate 시험의
  혼입 상태였던 GOP 16과 미시험 GOP를 정식 A/B 결과에서 분리했다.

- `2026-08-24`: 최초 실장비/Workstation snapshot, 기존 encoder 탐색 결과, 서비스 장애 복구 규칙,
  slice-size 시험 계획을 기록했다.
- `2026-08-24`: 오른쪽 `SSSS 10000` 및 `100000` 실패, `SSSS 0` rollback, 물리 재부팅,
  rollback 이후 runtime 저하 결과를 기록했다.
- `2026-08-24`: 오른쪽 `SSSS 1000000` 실패와 `SSSS 0` rollback을 기록하고, FW `2.18.72`의
  최종 slice-size를 `0`으로 확정했다.
- `2026-08-24`: 오른쪽 최종 물리 복구, IDR 2 재적용, GStreamer H.264 parse/NVIDIA decode
  boundary PASS와 RTP packet-loss 관측을 기록했다.
- `2026-08-24`: 왼쪽 IDR을 3에서 2로 변경해 양쪽 장비를 `0/1/90/2`로 통일하고, canonical
  Workstation JSON의 GOP/IDR도 `90/2`로 맞췄다.
- `2026-08-24`: 실행 중 Workstation의 stale `0/1/30/1` profile 재적용으로 발생한 왼쪽
  SDK/RTSP service loss, 오른쪽 profile drift, 물리 복구 및 양쪽 `0/1/90/2` 재복원을 기록했다.
- `2026-08-24`: 일반 RTSP enable의 중복 `SETH TRUE`를 생략하고 encoder read-before-write,
  changed-field-only SET, 필드별 복구/readback을 구현했다. 전체 suite `999 passed, 1 skipped`.
- `2026-08-24`: 실장비 반복에서 연속 GET만으로도 FW service reset이 재현돼 SDK command 간격
  `250 ms`, transient readback 재시도, 중복 final readback 제거를 추가했다. 관련 테스트 71개 PASS.
- `2026-08-24`: Calibration `Actual zoom readback`에 양쪽 `Zoom-in`/`Zoom-out` 버튼과 명령 완료 후
  자동 `GZVV` readback 예약을 추가했다. Calibration regression 40개 PASS.
- `2026-08-26`: RTP gap 뒤 clean-intra 대기와 `STALE` 체류시간을 줄이기 위해 사용자 승인으로
  양쪽 visible GOP를 `90 -> 30`으로 변경했다. 최종 profile `0/1/30/0`과 5초 dual RTSP decode를
  확인했으며, sender omission 자체의 해결로 판정하지 않는다.
- `2026-08-27`: 사용자 승인으로 양쪽 `SEIR VISIBLE 1`을 적용하고 순차 SDK 재부팅, fresh RTSP,
  active-stream 재적용까지 확인했다. 최종 readback은 `GETH=TRUE`, `0/1/30/1`, `GSSS=0`,
  `IRCR=FALSE`지만 모든 elementary stream의 NAL5는 0이었다. 공개 최신 SDK rev 241106EN에는
  별도 visible encoder restart 명령이 없으므로 추가 임의 SET 없이 Workswell firmware/support
  답변을 기다린다.
