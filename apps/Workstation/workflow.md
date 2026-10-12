# Workstation 표준 작업 순서

이 문서는 2026-10-09 기준 현장 운용 순서다. 가장 중요한 원칙은 저장 완결성보다 **지금 가능한 측정을 실행하는 것**이다.

운용용 Measurement Session은 없다. 새 측정을 위해 폴더를 열거나 닫고, 저장을 확정하거나, 완결성을 복구할 필요가 없다. 과거의 `OPEN`/`RECOVERABLE` 디렉터리와 저장 오류는 `측정 시작`을 비활성화하지 않는다.

## 전체 흐름

```text
QGC에서 P1/P2 센서 보정
  → Workstation 실행 전 검사
  → Data 폴더와 writer 상태 확인
  → WIRIS/Pixhawk/GNSS/선택적 Target 연결
  → K,D와 stereo R,t 보정·적용
  → 필요한 Alignment 계산·적용
  → 측정 시작 → 측정 종료
  → 즉시 다음 측정 또는 장시간 대기
  → 시각을 기준으로 후처리·통합
```

## 1. 실행 전 준비

1. QGroundControl에서 P1/P2의 Accelerometer, Gyroscope, Compass/Magnetometer, Level Horizon과 Board Orientation을 확인한다.
2. WIRIS, P1/P2, Base GNSS, 필요한 경우 Target Drone의 전원과 네트워크를 확인한다.
3. 승인된 YOLO 모델과 Calibration 프로파일을 준비한다. 실행 중 모델은 자동 다운로드되지 않는다.
4. NTRIP Password는 설정/증거 파일에 저장하지 않는다.
5. 데이터 루트를 별도 볼륨에 두려면 실행 전에 지정한다.

```bash
export SASL_WORKSTATION_DATA_ROOT=/absolute/path/to/workstation_data
./apps/Workstation/run.sh --check
./apps/Workstation/run.sh
```

## 2. Data 저장 구조

`Settings > Storage > Data folder`를 비우면 기본 실행은 `<SASL_ROOT>/recordings`에 저장한다. `SASL_WORKSTATION_DATA_ROOT`를 명시적으로 지정한 실행에서는 그 루트의 `recordings/`가 기본이다. 빈 값이 아닌 사용자 경로는 `SASL_WORKSTATION_DATA_ROOT`에 상대적인 하위 경로여야 한다. 절대경로나 루트 밖을 가리키는 값은 기본값으로 대체되고 `DEGRADED`로 표시되지만 측정을 막지 않는다. 기존 `.local/workstation_data/measurement_data/`는 이동·삭제하지 않고 과거 자료로 남긴다.

| 데이터 유형 | 지정 파일 |
| --- | --- |
| Measurement 워크북 (Start→Stop 1건) | `YYYYMMDD_HHMMSS[-NN]_measurement.xlsx` |
| Measurement 영상 | `YYYYMMDD_HHMMSS[-NN]_left.mp4`, `..._right.mp4` (동일 Start prefix) |
| 실행/장치/캡처 이벤트 | `events.jsonl`, `capture_events.jsonl` |
| 측정 결과 | `measurements.jsonl` |
| 준비된 측정 방법 평가 | `method_evaluations.jsonl` |
| 좌·우 카메라 | `camera_left.jsonl`, `camera_right.jsonl` |
| P1/P2/Target PX4 | `px4_left.jsonl`, `px4_right.jsonl`, `px4_drone.jsonl` |
| Base GNSS | `gnss_base.jsonl` |
| 기하 비교 | `geometry_comparisons.jsonl` |
| Alignment | `alignment.jsonl` |
| 아티팩트/로그 | `artifacts.jsonl`, `native_frames.jsonl`, `application_log.jsonl` |
| RTCM 바이너리/인덱스 | `raw/gnss/*.rtcm`, `raw/gnss/*.rtcm.jsonl` |

이 파일은 앱을 재시작해도 같은 파일에 계속 append된다. 자동 rotation, Parquet, `review.xlsx`, Measurement 폴더 확정은 없다.

Measurement 워크북은 사람이 직접 읽는 기본 기록이다. 탭: `00_Summary`, `01/02_*_Detections`(모든 detection 후보),
`03_Calibration`(K,D), `04_Alignment`(적용 R,t), Method 탭 `10_Pixhawk_RTK`…`18_Pixhawk_RP_GPS_Yaw`
(모두 동일 섹션 A 결과/R,t · B 선택 detection · C 실제 u,v/ray · D R,t 근거 · E 진단), PX4 P1/P2/Drone
telemetry 탭(Attitude/GNSS_RTK/IMU_Mag/Position/System, downsampling 없음), `43_Base_GNSS`, `50_Events`,
`90_Flat`, `91_Exact`(float.hex), `98_Provenance`, `99_Definitions`. 값이 없으면 `N/A`. 워크북은 임시 파일을
완성·fsync한 뒤 atomic replace로 갱신되므로 크래시 시에도 마지막 정상 파일이 남는다. 워크북 저장 실패는
`Data > Status`와 `50_Events`에만 나타나며 측정·JSONL·MP4·ULog·RTCM 저장을 막지 않는다.

MP4, 사진, native PNG, ULog의 파일명은 `YYYYMMDD_HHMMSS_nanoseconds+offset_label.ext`다. 충돌하면 `-01`, `-02` 등을 붙이며 기존 파일은 덮어쓰지 않는다.

## 3. Calibration과 Alignment

Calibration은 Measurement와 독립적인 재사용 루트를 사용한다. 기본값 `calibration`은 `<SASL_ROOT>/apps/Workstation/calibration`으로 해석하고, 새 실행은 새 timestamp 하위 폴더를 만든다. 기존 `.local/workstation_data/calibration`과 기존 Workstation calibration 폴더는 자동 이동·병합·삭제하지 않는다. `촬영 → 계산·적용 → 프로파일` 순서로 K,D와 R,t를 확인한다. 승인된 K,D의 재시작 의존 파일은 data root로 해시 검증 복사한다. Calibration 화면의 photo-set 묶음은 Calibration 도메인의 작업 단위이며 Measurement Session 게이트가 아니다.
Live/Auto pair의 `Max Live pair Δt`는 decoded-frame app-receive monotonic 수신시각을 기준으로 하며, NTP/RTCP는 저장·진단용이다. 수신시각은 노출시각 증명이 아니므로 촬영 순간 보드를 멈춘다. FTP는 기존 CPTR dispatch proxy를 유지한다.

Alignment는 앱 실행 수명을 따른다.

- Apply한 Alignment는 다음 Measurement에서도 그대로 사용된다.
- `Revert active pose`는 이전 적용 상태로 돌린다.
- `Reset Alignment`는 후보, frozen frame, annotation, 관측, active pose를 메모리에서 지우고 `alignment_reset` 이벤트를 append한다. 기존 파일은 삭제하지 않는다.
- 앱을 재시작하면 저장된 Alignment를 자동 복구하지 않는다. 재사용이 필요하면 extrinsics 프로파일을 별도로 저장한다.
- Horizon/Mountain `Use suggestion`은 `GOOD`/`LOW_CONFIDENCE` 품질 상태와 무관하게
  사용할 수 있다. Horizon은 현재 frozen identity와 좌·우 각각 정확히 2개의 유한한
  in-frame endpoint 및 image width 25% 이상 간격이 필요하다. Mountain은 exclusion 적용 후
  좌·우 동일 순서의 유한한 native `(u,v)`가 4쌍 이상이고 중복/공선이 아니어야 한다.
- 이 동작은 점을 annotation buffer로 복사할 뿐이며 Solve/Apply를 자동 실행하지 않는다.

## 4. 측정 시작

1. `Measurement`에서 카메라, Detection, Sync, GNSS/RTK, 선택 방법의 상태를 확인한다.
2. `측정 시작`을 누른다. 저장이 `DEGRADED`/`UNAVAILABLE`여도 버튼은 활성이다.
3. 수동 기록 중이면 먼저 중지한 뒤 Measurement를 시작한다. 측정 후 수동 기록을 자동 재개하지 않는다.
4. 다음 카메라 owner가 작업 중이면 Run은 `QUEUED`가 되고 종료 후 자동 시작한다.

   - HR/OZ, Calibration, Alignment still capture/FTP
   - WIRIS command, 시각 동기화, zoom, image format, STIP, LRF
   - 카메라 탐색, RTSP check/enable/reactivation

RTSP URL이 비어 있거나 좌·우가 같은 stream이면 시작을 거부한다. 이것은 저장 무결성 게이트가 아니라 실제 카메라 입력 문제다.

## 5. 측정 중 기록

```text
IDLE → QUEUED(필요 시) → STARTING → RUNNING → STOPPING → IDLE
```

- Run이 `RUNNING`인 동안 `pixhawk_rtk_live` 대표 결과가 `VALID` 또는
  `ESTIMATED`로 도착하면 UI를 갱신하기 전 `measurements.jsonl` priority 큐에
  넣는다. enqueue 시도와 그 결과 확인이 먼저고 현재 테이블·이력·그래프·거리 banner
  갱신은 나중이다. 거부된 enqueue는 degraded evidence로 표시될 수 있다.
- pair마다 준비된 Method만 worker에 제출한다. 준비되지 않은 Method는 워크북에
  `Attempted=NO / Status=N/A`와 사유로 남는다. Method 계산에는 자동 timeout이 없다:
  worker 프로세스가 죽거나 앱이 종료될 때만 `FAILED`가 된다. Method별 대기 한도(20 pair)를
  넘긴 요청만 해당 Method에 한해 `BACKPRESSURE_BLOCKED`가 된다.
- 여러 detection이 있어도, Δt·수직 편차·시차가 커도, depth가 음수여도 결과는 계산·저장된다
  (경고 첨부). 좌·우 detection의 class가 다르면 pair를 만들지 않는다(`correspondence_class_mismatch`).
  수학적으로 정의되지 않는 경우(baseline 0, 평행 ray, 입력 결손/비정상, 비유한 해)만 F/R/D/Range가
  `N/A`이며 입력·R,t·사유는 그대로 남는다.
- primary `BLOCKED`/`FAILED`/`BACKPRESSURE_BLOCKED`는 대표 측정 행을 만들지 않고
  `method_evaluations.jsonl`에만 남는다. 제출된 Method의 결과가 모두 도착하면 같은 파일에
  bundle로 append되고, 개별 결과는 도착 즉시 워크북에 반영된다.
- 여기서 즉시 저장은 비동기 priority enqueue를 뜻한다. 해당 시점에 디스크
  write/flush/`fsync`까지 완료됐다는 뜻은 아니다.
- 제출된 방법의 bundle은 enqueue가 승인된 뒤에만 commit/count한다. 활성 행 enqueue가 거부되면
  같은 owner-epoch/index lease의 행을 재시도하고, 그 사이 Stop되면 `late=true`로 기록한다.
  owner가 clear/rebind/close되어 lease가 무효가 되면 새 행을 덮지 않고 direct writer로 넘긴다.
- RUNNING 중 도착한 primary 대표 행 enqueue가 거부되면 제출된 방법의 bundle 저장 여부와
  무관하게 background 재시도를 유지한다. Stop/다음 run 이후에는 원래 generation의
  `late_primary_measurement_outcome`만 저장하며 새 run의 표·카운터·그래프·거리 UI는 건드리지 않는다.
- Data store가 `CLOSING` 또는 `CLOSED`면 새 enqueue를 거부하고 drop/error 증거를
  남긴다. shutdown 시작 뒤의 행을 저장된 것으로 간주하지 않는다.
- native frame 완료는 다음 run이 시작됐더라도 원래 request의 row owner/lease만
  갱신한다. lease가 무효이거나 원래 행을 갱신할 수 없는 경우를 포함해 `native.frames`에 원래
  runtime generation으로 completion event를 남긴다. row와 event enqueue가 함께 거부되면
  별도 bounded retry가 payload를 유지한다. exact duplicate는 무시하고 conflict는 최초 terminal
  결과를 보존한다. 영구 장애로 retry budget을 소진하면 terminal-drop counter/status/log를 남겨
  종료가 무한 대기하지 않게 한다.
- Camera/PX4/GNSS/Target bulk row는 별도 큐를 사용한다. overload 시 오래된 bulk row가 누락될 수 있으며 `dropped_records`에 누적된다.
- RTCM routing/injection은 증거 저장과 독립적이다. RTCM 파일 오류가 발생해도 Measurement를 저장 재시도 상태로 바꾸지 않는다.
- 정밀 기하 입력이 없으면 결과가 `BLOCKED`/`ESTIMATED`가 될 수 있지만 raw 수집은 계속한다.
- 실시간 거리의 좌·우 frame 선택은 Measurement Settings를 따른다. 기본값은 exact decoded-
  frame `app_receive_monotonic_ns`이다. `RTCP/NTP preferred`는 Clock VERIFIED와 current
  same-epoch frame evidence가 모두 정상일 때 실제 RTCP/NTP로 pair를 고른다.
- RTCP timestamp 누락/오류, Clock 미검증·stale, epoch transition/mismatch 또는 선택 Δt 초과
  시 같은 run은 exact app receive로 자동 fallback한다. 선택 기준의 절대 Δt는 설정값과
  100 ms hard cap을 모두 만족해야 한다. fallback 자체는 유효한 기하를 `ESTIMATED`로 낮추지 않는다.
- 실행 중 Settings 변경은 현재 Measurement에 섞지 않고 다음 Measurement부터 적용한다.
  PX4 attitude/GNSS association은 두 기준 모두 host monotonic을 유지한다.

## 6. 측정 종료와 반복

`측정 종료`를 누르면 Detection, 카메라, 영상 owner를 해제하고 writer에 flush를 요청한다. Stop 전에
RUNNING primary `VALID`/`ESTIMATED`가 이미 priority enqueue됐다면 그 행은 보존된다. Stop이
먼저 실행되고 primary가 나중에 도착하면 `measurements.jsonl` 행을 만들지 않고 거리 UI도
갱신하지 않는다. 단, RUNNING 중 이미 도착했지만 enqueue만 거부된 primary는 background
소유권을 유지하며 Stop 뒤 compact late evidence로 저장을 계속 시도한다. 이 재시도는 다음
측정 시작을 막거나 새 측정 UI를 갱신하지 않는다. 대표 거리 행이 반드시 필요하면 Measurement UI에 Pixhawk attitude +
GNSS/RTK 거리가 나타날 때까지 기다린 뒤 Stop한다. 사용하지 않는 비교 method 종료를 기다릴 필요는
없다. Parquet/엑셀, manifest/hash/completeness, 과거 디렉터리 복구, optional worker 종료를 대기하지
않는다.

`Data > Status`의 store lifecycle가 `OPEN`인지도 확인한다. `CLOSING`/`CLOSED`는 새
enqueue를 받지 않으므로, 필요한 대표 행이 없다면 정상 `OPEN` store로 새 Measurement를
시작하고 거리 표시 후 Stop한다.

Stop 시점에 계산 중이던 Method는 계속 실행되며, 그 결과는 나중에 도착해도 원래 Measurement의
워크북에 반영된다(`Data > Status`의 pending methods 수로 확인). Method worker는 앱 수명 동안 유지되므로
다음 측정 시작이 이전 측정의 계산을 끊지 않으며, 이전 결과가 새 측정에 섞이지 않는다.

실제 카메라 worker가 종료하면 Run은 `IDLE`로 돌아가고 바로 다음 측정을 시작할 수 있다. 나중에 하나의 실험 묶음으로 만들려면 `recorded_utc_ns`, source timestamp, `runtime_generation`과 timestamped binary 파일명을 기준으로 외부에서 모은다.

## 7. 명시적 캡처와 수동 기록

`Data > Capture`의 HR/OZ capture, exact HR import, ULog import과 Alignment capture/apply/reset은 Measurement 밖에서도 사용할 수 있다. 명시적 운영자 이벤트이므로 항상 event를 append한다.

`Advanced > Files > Manual record`는 Measurement 밖의 연속 수집을 위한 보조 기능이다. 수동 기록도 동일한 유형별 파일과 timestamped binary 규칙을 사용한다.

## 8. 저장 오류 처리

`Data > Status`에서 writer 생존 여부, priority/bulk queue 대기 수, stream별 `IDLE/PENDING/ACTIVE/FAILED`, 누락 행 수, 마지막 write 시각과 오류를 확인한다.

`FAILED`인 stream은 `Retry Storage`로 재시도할 수 있다. 재시도는 향후 append를 위한 것이며 누락된 과거 행을 재생성하지 않는다. 어떤 상태도 `측정 시작`을 막지 않는다.

## 계산 작업 상태 확인과 복구

`Data > Status > Method 계산 상태`에서 방법을 선택하고 상태·경과 시간·대기 수를 확인한다.
Run/Pair 식별자는 상태 tooltip에서 확인한다. 경과 시간은 무응답의 증거가 아니며 자동 계산
제한 시간은 없다. 운영자가 중단을 결정한 경우 `선택 계산 중단·재시작`을 누른다.

표시된 worker/요청 식별자가 실제 소유자와 같을 때만 현재 계산을 `FAILED/operator_restart`로
기록하고 해당 worker를 교체한다. 대기 계산·다른 방법·관측은 계속한다. 요청 접수와 교체
완료는 별도 표시이며, 선택이 오래됐으면 거부한다. 교체 실패 후에는 같은 화면에서 다시
요청할 수 있다. 결과와 복구 event는 원래 Run에 귀속한다. 일반 `측정 종료`는 이 작업을
자동 실행하지 않는다.

## 9. 기존 데이터

`recordings/<id>/`, `recordings/sessions/<id>/`의 기존 파일은 삭제·복구·변환하지 않는다. `Data > Status > Open Legacy Recordings Folder`로 file manager에서만 수동 확인한다. 기존 `OPEN`/`RECOVERABLE`은 새 Measurement에 영향을 주지 않는다.

## 10. 좌표계 원칙

| 구분 | 좌표/계약 |
| --- | --- |
| 카메라 픽셀/광학 | OpenCV optical RDF: Right/Down/Forward |
| PX4 body | FRD: Forward/Right/Down |
| 세계/자세 | NED |
| GNSS 위치 | WGS84, MSL/타원체 고도를 별도 표기 |

```text
X_right = R_left_to_right X_left + T_left_to_right_m
C_right_in_left = -R^T T
```

`T_left_to_right_m`와 `C_right_in_left`를 같은 값의 별칭으로 취급하지 않는다. 후처리할 때도 생산자가 기록한 frame/unit/timestamp 계약을 유지한다.

## Target reference 연결

Data → Status → Target reference에서 Drone을 선택한다. Find Network는 PC 수신 포트의 MAVLink를 관찰하고, Find USB는 선택 baud로 직렬 장치의 heartbeat를 기다린다. 기체 측 송신 설정이 없으면 신원 확인이 되지 않을 수 있다. 후보의 출처·SYS/COMP를 확인한 뒤 Use selected로 초안을 채우고 Connect target을 누른다. 후보의30초 유효기간과 실제 연결 시 신원을 다시 검사한다. 같은 SYS/COMP라도 선택한 UDP peer가 다르면 수신하지 않는다.

heartbeat, GPS1/GPS2와 ATTITUDE의 수신 나이·유효성을 구분한다. GPS가 없거나 STALE이면 해당 상태가 그대로 표시된다. Stop 완료 후 연결 대상을 바꿀 수 있다. 탐색 취소 후에는 worker가 종료된 다음 재탐색한다. Save는 연결 선호값만 저장하며 재시작 자동 연결은 없다.

UAM은 외부 사후 RTK를 기다리는 DEFERRED 상태로 적용한다. Target은 MAVLink 수신 전용이다. Telemetry/P1/P2 연결·RTCM 경로와 독립적이며, Target source가 camera RDF 또는 calibration truth로 자동 변환되지 않는다.
