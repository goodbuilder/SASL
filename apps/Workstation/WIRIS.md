# WIRIS P1/P2 카메라 설정 및 스트림 튜닝 기록

이 문서는 `apps/Workstation`에서 사용하는 왼쪽/오른쪽 Workswell WIRIS Enterprise의
실장비 설정, Workstation 설정, 성능 기준선, 변경 및 복구 절차를 기록한다.

- 초기 기준 snapshot: `2026-08-24T00:02:11+09:00` (Asia/Seoul)
- 최신 상태 확인: `2026-08-27` 순차 read-only SDK GET
- 실장비 firmware: 양쪽 모두 `2.18.72`
- 목표: 카메라 화질을 최대한 보존하면서 양쪽 decoded rate를 지속적으로 `17 Hz` 이상 유지
- 절대 조건: visible encoder bitrate는 양쪽 모두 `0`으로 유지
- 공식 SDK 문서: <https://my.workswell.eu/link/latest/1653482886wf2Pr9x8SKHQFZ9PN7f9>

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

### Visible encoder — 현재 최선 실측값

| 항목 | SDK GET | 왼쪽 | 오른쪽 |
|---|---|---:|---:|
| Bitrate | `GEBR VISIBLE` | `0` | `0` |
| QP | `GEQZ VISIBLE` | `1` | `1` |
| GOP | `GEGS VISIBLE` | `30` | `30` |
| IDR interval | `GEIR VISIBLE` | `1` | `1` |
| Slice-size | `GSSS` | `0` | `0` |

`bitrate=0`은 constant-quality mode이며 QP가 유효한 조건이다. Bitrate에 대해서는 테스트 중
`SEBR` 명령을 보내지 않는다. 모든 변경 전후에 `GEBR VISIBLE=0`을 readback한다.
`GEIR=1`은 설정 readback일 뿐 이 FW의 실제 random-access 증거가 아니다. 순차 재부팅 전후,
RTSP 비활성/활성 중 적용 후에도 두 visible stream 모두 NAL type 5를 한 번도 내보내지 않았다.

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

현재 Workstation 저장값은 `software` decoder와 RTSP `tcp`이므로 위 UDP/NVIDIA 기준선과 다르다.
모든 non-zero slice-size 후보가 오른쪽 단독 RTP/AU boundary에서 기준보다 악화됐으므로 양쪽
동시/저장 backend 시험으로 승격하지 않았다. 최종 slice-size는 `0`이다.

### 확인된 경계

- NVIDIA decoder는 입력 H.264 access unit을 거의 모두 decode했다. 주 병목은 decoder 앞이다.
- 오른쪽 UDP RTP는 약 `80-90 Mb/s`, 초당 약 `7,000-8,000` packet까지 관측됐다.
- TCP interleaved는 UDP보다 marker rate가 더 낮았다.
- QP `5`, `17`, `30`, `40`, GOP `30`, `60`, `90`, `120`, IDR `1`, `2`, `3`, `5`를 시험했다.
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
