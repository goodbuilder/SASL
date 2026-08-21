# Workstation 작업 순서

이 문서는 SASL StereoWIRIS Workstation을 이용해 장비 상태를 확인하고, 측정을 수행한 뒤,
결과를 저장·검토하는 표준 작업 순서를 설명한다.

## 전체 흐름

```text
사전 점검
  → Workstation 실행
  → 장비 연결 및 상태 확인
  → 카메라 보정·시각 동기화
  → GNSS/RTK 및 물리 정렬 확인
  → 측정 설정 확인
  → 당일 Session 열기
  → 개별 측정 수행
  → 측정 봉인
  → Measurement 반복 또는 Session 종료
  → 저장 결과 검토
```

## 1. 사전 준비

1. QGroundControl(QGC)에서 P1/P2 Pixhawk의 센서 보정을 완료한다.
2. WIRIS 좌·우 카메라, P1/P2 Pixhawk, Base GNSS 및 Target Drone의 전원과 네트워크를 확인한다.
3. 필요한 YOLO 모델과 캘리브레이션 프로파일이 준비되어 있는지 확인한다.
4. NTRIP을 사용할 경우 계정 정보는 설정 파일에 직접 기록하지 않고 지정된 환경 변수로 제공한다.
5. 저장 공간과 Workstation 데이터 경로를 확인한다. 기본 경로는 저장소 루트의
   `.local/workstation_data`이다.

데이터 경로를 변경해야 하면 절대 경로를 지정한다.

```bash
export SASL_WORKSTATION_DATA_ROOT=/absolute/path/to/workstation_data
```

## 2. 실행 전 검사 및 시작

저장소 루트에서 다음 명령을 실행한다.

```bash
./apps/Workstation/run.sh --check
./apps/Workstation/run.sh
```

현재 `apps/Workstation` 디렉터리에 있다면 다음과 같이 실행할 수 있다.

```bash
./run.sh --check
./run.sh
```

`--check`가 실패하면 UI를 실행하기 전에 표시된 의존성, 스트림 백엔드 또는 YOLO 모델 문제를
해결한다. 카메라 자동 탐색 없이 UI만 확인하려면 `./run.sh --no-start`를 사용한다.

## 3. 장비 연결 및 준비 상태 확인

각 장비는 `Connect` 명령 실행 여부가 아니라 실제 수신 상태를 기준으로 판단한다.

1. WIRIS 좌·우 카메라를 연결하고 영상, FPS 및 카메라 ID를 확인한다.
2. P1/P2 Pixhawk를 연결하고 attitude, timestamp, age 및 센서 상태를 확인한다.
3. Base GNSS를 연결한다.
4. 필요하면 `Settings > GNSS`에서 Survey-in 또는 NTRIP 방식을 설정하고 시작한다.
5. 좌·우 Pixhawk로 RTCM이 전달되는지 확인한다.
6. P1/P2의 숫자 `fix_type`, 위성 수, eph/epv, 보정 데이터 age 및 NED baseline을 확인한다.
7. Target Drone을 사용하는 경우 `Devices > Telemetry`에서 연결 상태와 위치 수신을 확인한다.
8. 이상이 있으면 현재 화면의 상태를 유지한 채 `Diagnostics`에서 원인을 확인한다.

## 4. 카메라 보정 및 시각 동기화

내부 카메라 보정은 현장 측정 전에 완료하는 것을 원칙으로 한다.

1. `Calibration > Capture`에서 사용할 zoom을 선택한다.
2. 양쪽 카메라에 동일한 zoom을 명령하고 실제 readback 값이 일치하는지 확인한다.
3. 체커보드를 다양한 위치, 거리, roll, yaw 및 pitch에서 촬영한다.
4. `Calibration > Compute`에서 저장된 좌·우 사진 쌍을 선택한다.
5. 필요한 `K,D` 또는 stereo `R,t`를 계산한다.
6. 품질 결과를 확인한다.
   - `PASS`: 저장·적용 가능
   - `WARNING`: 운영자 확인 후 저장·적용
   - `FAIL`: 저장·적용 금지
7. 측정 zoom에 맞는 정확한 `K,D`와 사용할 `R,t`를 불러와 적용한다.
8. WIRIS 시간 동기화 상태와 좌·우 RTCP/NTP pair 상태를 확인한다.

## 5. 물리 정렬 및 Alignment 확인

1. P1→P2 GNSS baseline을 기준으로 장비의 물리 baseline을 맞춘다.
2. Roll/Pitch/Yaw를 물리적으로 정렬한다.
3. 새로운 샘플을 받아 baseline과 자세를 다시 확인한다.
4. 정렬 후 WIRIS 시간 동기화를 다시 확인하고 필요하면 재동기화한다.
5. 광학 자세 보정이 필요하면 `Alignment`에서 `Capture Live Pair`를 눌러 좌·우 native frame을
   고정한다.
6. Horizon, Mountain + GNSS, Terrain PnP, Drone Rotation 또는 Drone Multi-view 방식으로
   관측값을 입력하고 후보의 품질·관측 가능 DOF를 검토한다.
7. 조건을 만족하는 후보만 적용한다. 자동 선택 결과도 검토 없이 `R,t`에 적용되지 않는다.

## 6. 측정 설정 확인

측정 시작 전 다음 항목을 확인한다.

- 양쪽 카메라 스트림과 시간 동기화
- 적용된 operating zoom용 `K,D`와 선택한 `R,t`
- YOLO 모델, 입력 크기, confidence, IoU 및 pair gate
- Measurement method와 fallback 순서
- 필요한 Pixhawk/GNSS 보완 축과 DOF fill 설정
- GNSS/RTK, baseline 및 물리 정렬 상태
- 녹화 경로와 남은 저장 공간

설정 변경은 새 Detection을 시작하기 전에 완료한다. Measurement별 method/fallback 설정과 DOF
fill 결정은 Detection 시작 시 고정된다.

`Session` 화면의 `Readiness`에서 다음 두 상태를 별도로 확인한다. 표에는 통과 항목을 반복하지
않고 현재 차단 원인만 표시한다.

- `Evidence collection`: Session, 현재 Measurement, 좌·우 카메라, 시각 동기화가 증거 수집에
  준비되었는지 표시한다.
- `Valid measurement`: 위 조건과 함께 유효한 geometry 방식이 하나 이상 준비되었는지 표시한다.

`Measurement method inputs` 표는 Pixhawk, Checkerboard, Horizon, Mountain + GNSS, Drone RTK
Rotation, Drone RTK Multi-view, Terrain PnP, Combined를 항상 표시한다. candidate/profile이
없으면 행을 숨기지 않고 `BLOCKED`와 필요한 입력을 표시한다. 방식을 선택하면 실제 Pixhawk
Roll/Pitch/Yaw, P1→P2 GNSS NED baseline, candidate pose 또는 checkerboard `R,t`, instance,
hash 및 좌표계 계약을 상세 영역에서 확인할 수 있다. 서로 다른 좌표계의 값은 합쳐 표시하지
않는다.

## 7. 세션 및 개별 측정 수행

기본 측정 순서는 다음과 같다.

```text
Start Session
  → New Measurement
  → Pre 증거 수집(선택)
  → Detection 시작
  → 측정 관찰
  → Detection 정지
  → Post 증거 수집(선택)
  → Seal Measurement
```

1. 당일 결과 수집을 시작하기 직전에 `Session` 화면에서 `Open Session`을 누른다. 장비 연결이나
   캘리브레이션 시험만 하는 동안에는 Session을 열 필요가 없다.
2. 같은 화면에서 `New Measurement`를 눌러 첫 번째 측정(`m000001`)을 생성한다.
3. 필요하면 `Pre` 목적으로 HR/OZ still 또는 정렬 증거를 수집한다.
4. `Current Measurement readiness`와 방식별 입력값을 확인한 후 `Measurement` 화면으로 이동해
   `Detection`을 시작한다. 한 Measurement에서는 Detection을 한 번만 실행할 수 있다.
5. 영상, detection, 거리·단위, 유효성, 품질, 동기화 및 RTK 상태를 함께 관찰한다.
6. 필요한 데이터가 수집되면 Detection을 정지한다.
7. 필요하면 `Post` 목적으로 HR/OZ still 또는 추가 증거를 수집한다.
8. `Session` 화면에서 `Seal Measurement`를 눌러 현재 측정을 봉인한다.

Detection을 정지해도 Measurement가 자동으로 봉인되지는 않는다. `Seal Measurement`가 저장,
RTCM drain, 무결성 검사 및 manifest 생성을 완료할 때까지 기다린다.

## 8. 추가 측정 또는 세션 종료

- 같은 현장 세션에서 다시 측정하려면 현재 Measurement 봉인 후 `New Measurement`를 누른다.
- 새 Measurement ID는 `m000002`, `m000003` 순서로 증가한다.
- 봉인된 Measurement에 증거를 추가해야 하면 새 Detection이 아니라 revision을 생성한다.
- 모든 측정이 끝나면 실행 중인 producer와 수정 가능한 child가 없는지 확인한 후
  `Session` 화면에서 `Close Session`을 누른다.
- `Close Session`은 봉인된 모든 Measurement를 포함하는 상위 manifest와 통합 `review.xlsx`를
  생성한다.

## 9. 결과 검토

1. `Open Session Review` 또는 `Recording / Review`에서 저장된 세션을 연다.
2. `(session_id, measurement_id)`를 기준으로 대상 Measurement를 선택한다.
3. manifest 상태, 메타데이터, 측정값, 그래프 및 원본 증거를 확인한다.
4. `review.xlsx`는 운영자용 요약으로 사용하고, 전체 고주기 데이터 분석에는 Parquet 파일을
   사용한다.
5. 필요한 경우 결과를 export하거나 다른 Measurement와 비교한다.

## 10. 실패 시 처리

1. 오류 메시지와 실제 장비 상태를 확인한다. 명령 전송 성공을 장비 동작 성공으로 간주하지 않는다.
2. `Diagnostics`에서 연결, RTSP, FTP, MAVLink, GNSS, RTCM 및 저장 오류의 구체적인 원인을 찾는다.
3. 설정 또는 연결을 수정하고 실제 상태가 회복됐는지 다시 확인한다.
4. `Seal Measurement` 실패 시 결과를 삭제하거나 새 세션을 만들지 말고 동일 Measurement에서
   재시도한다. 실패한 패키지는 `RECOVERABLE` 상태로 보존된다.
5. `SEALED_INCOMPLETE`는 일부 필수 증거가 누락되었다는 뜻이므로 manifest의 누락 항목과 실패
   기록을 확인한다.
6. 강제 종료 후에는 기존 세션의 복구 상태와 spool 무결성을 먼저 확인한 뒤 작업을 재개한다.

## 현장용 핵심 체크리스트

- [ ] QGC 센서 보정 완료
- [ ] WIRIS 좌·우 영상 및 FPS 정상
- [ ] P1/P2 Pixhawk 연결 및 자세 데이터 정상
- [ ] Base GNSS/RTCM 경로 정상
- [ ] 숫자 `fix_type`과 baseline 확인
- [ ] operating zoom용 `K,D` 및 `R,t` 적용
- [ ] WIRIS 시간 동기화 확인
- [ ] 물리 baseline 및 Roll/Pitch/Yaw 정렬 확인
- [ ] YOLO 및 Measurement 설정 확인
- [ ] `Open Session` 후 `New Measurement` 생성
- [ ] Session 화면에서 Evidence collection/Valid measurement 준비 상태 확인
- [ ] Effective method와 실제 입력값·좌표계 확인
- [ ] Pre → Detection → Post 순서로 증거 수집
- [ ] `Seal Measurement` 완료 상태 확인
- [ ] 모든 child 봉인 후 `Close Session`
- [ ] manifest와 `review.xlsx` 검토

## 관련 문서

- 상세 기능 및 운용 규칙: `README.md`
- UI 중심 사용자 흐름: `docs/ui/01_USER_WORKFLOW.md`
- 화면 이동 구조: `docs/ui/05_NAVIGATION.md`
- 상태 표시 규칙: `docs/ui/06_STATUS_MODEL.md`
