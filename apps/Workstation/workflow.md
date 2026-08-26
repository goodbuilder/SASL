# Workstation 표준 작업 순서

이 문서는 2026-08-26 기준 현장 운용 순서다. 가장 중요한 원칙은 저장 완결성보다 **지금 가능한 측정을 실행하는 것**이다.

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

`Settings > Storage > Data folder`는 `SASL_WORKSTATION_DATA_ROOT`에 상대적인 하위 경로다. 기본값은 `measurement_data`이다. 절대경로나 루트 밖을 가리키는 값은 기본값으로 대체되고 `DEGRADED`로 표시되지만 측정을 막지 않는다.

| 데이터 유형 | 지정 파일 |
| --- | --- |
| 실행/장치/캡처 이벤트 | `events.jsonl`, `capture_events.jsonl` |
| 측정 결과 | `measurements.jsonl` |
| 8개 방법 평가 | `method_evaluations.jsonl` |
| 좌·우 카메라 | `camera_left.jsonl`, `camera_right.jsonl` |
| P1/P2/Target PX4 | `px4_left.jsonl`, `px4_right.jsonl`, `px4_drone.jsonl` |
| Base GNSS | `gnss_base.jsonl` |
| 기하 비교 | `geometry_comparisons.jsonl` |
| Alignment | `alignment.jsonl` |
| 아티팩트/로그 | `artifacts.jsonl`, `native_frames.jsonl`, `application_log.jsonl` |
| RTCM 바이너리/인덱스 | `raw/gnss/*.rtcm`, `raw/gnss/*.rtcm.jsonl` |

이 파일은 앱을 재시작해도 같은 파일에 계속 append된다. 자동 rotation, Parquet, `review.xlsx`, Measurement 폴더 확정은 없다.

MP4, 사진, native PNG, ULog의 파일명은 `YYYYMMDD_HHMMSS_nanoseconds+offset_label.ext`다. 충돌하면 `-01`, `-02` 등을 붙이며 기존 파일은 덮어쓰지 않는다.

## 3. Calibration과 Alignment

Calibration은 Measurement와 독립적인 재사용 루트를 사용한다. `촬영 → 계산·적용 → 프로파일` 순서로 K,D와 R,t를 확인한다. Calibration 화면의 photo-set 묶음은 Calibration 도메인의 작업 단위이며 Measurement Session 게이트가 아니다.

Alignment는 앱 실행 수명을 따른다.

- Apply한 Alignment는 다음 Measurement에서도 그대로 사용된다.
- `Revert active pose`는 이전 적용 상태로 돌린다.
- `Reset Alignment`는 후보, frozen frame, annotation, 관측, active pose를 메모리에서 지우고 `alignment_reset` 이벤트를 append한다. 기존 파일은 삭제하지 않는다.
- 앱을 재시작하면 저장된 Alignment를 자동 복구하지 않는다. 재사용이 필요하면 extrinsics 프로파일을 별도로 저장한다.

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

- 측정 결과와 method 평가는 priority 큐를 통해 append된다.
- Camera/PX4/GNSS/Target bulk row는 별도 큐를 사용한다. overload 시 오래된 bulk row가 누락될 수 있으며 `dropped_records`에 누적된다.
- RTCM routing/injection은 증거 저장과 독립적이다. RTCM 파일 오류가 발생해도 Measurement를 저장 재시도 상태로 바꾸지 않는다.
- 정밀 기하 입력이 없으면 결과가 `BLOCKED`/`ESTIMATED`가 될 수 있지만 raw 수집은 계속한다.

## 6. 측정 종료와 반복

`측정 종료`를 누르면 Detection, 카메라, 영상 owner를 해제하고 writer에 flush를 요청한다. Parquet/엑셀, manifest/hash/completeness, 과거 디렉터리 복구, optional worker 종료를 대기하지 않는다.

실제 카메라 worker가 종료하면 Run은 `IDLE`로 돌아가고 바로 다음 측정을 시작할 수 있다. 나중에 하나의 실험 묶음으로 만들려면 `recorded_utc_ns`, source timestamp, `runtime_generation`과 timestamped binary 파일명을 기준으로 외부에서 모은다.

## 7. 명시적 캡처와 수동 기록

`Data > Capture`의 HR/OZ capture, exact HR import, ULog import과 Alignment capture/apply/reset은 Measurement 밖에서도 사용할 수 있다. 명시적 운영자 이벤트이므로 항상 event를 append한다.

`Advanced > Files > Manual record`는 Measurement 밖의 연속 수집을 위한 보조 기능이다. 수동 기록도 동일한 유형별 파일과 timestamped binary 규칙을 사용한다.

## 8. 저장 오류 처리

`Data > Status`에서 writer 생존 여부, priority/bulk queue 대기 수, stream별 `IDLE/PENDING/ACTIVE/FAILED`, 누락 행 수, 마지막 write 시각과 오류를 확인한다.

`FAILED`인 stream은 `Retry Storage`로 재시도할 수 있다. 재시도는 향후 append를 위한 것이며 누락된 과거 행을 재생성하지 않는다. 어떤 상태도 `측정 시작`을 막지 않는다.

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
