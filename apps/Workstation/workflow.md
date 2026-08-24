# Workstation 표준 작업 순서

이 문서는 SASL StereoWIRIS Workstation에서 장비를 준비하고, 카메라를 보정하고, 측정을 수행한
뒤 결과를 봉인·검토하는 현재 표준 작업 순서를 설명한다. 현재 구현의 기준은
`Create Session Folder`로 상위 저장 폴더를 만들고, 각 Measurement의 Detection을 정지하면 저장과
봉인을 자동 요청하는 흐름이다.

## 전체 흐름

```text
QGC에서 P1/P2 센서 보정
  → Workstation 실행 전 검사
  → Session 폴더 생성
  → WIRIS/Pixhawk/GNSS 연결 및 실제 수신 확인
  → WIRIS 시각 동기화
  → 카메라 K,D 및 stereo R,t 보정·적용
  → GNSS baseline과 장비 물리 정렬
  → 필요 시 Alignment 후보 계산·적용
  → Measurement 설정과 readiness 확인
  → Pre 증거 수집(선택, 첫 요청이 Measurement 초안 생성)
  → 측정 시작(child가 없으면 생성 + Detection)
  → 측정 종료
  → 자동 저장·RTCM drain·무결성 검사·봉인
  → Measurement 반복 또는 Close Session
  → manifest, review.xlsx 및 Parquet 검토
```

## 1. 보정값과 좌표계 구분

| 구분 | 결과 | 목적 | 좌표계 |
| --- | --- | --- | --- |
| 카메라 내부 보정 | 좌·우 `K,D` | 픽셀과 카메라 광선의 관계 결정 | OpenCV optical RDF |
| 스테레오 외부 보정 | `R_left_to_right`, `T_left_to_right_m` | 좌·우 카메라의 상대 회전·이동 결정 | OpenCV optical RDF |
| 현장 Alignment | immutable pose candidate와 관측 가능 DOF | 카메라 rig를 PX4/GNSS/세계 좌표에 연결 | virtual-camera FRD / world NED |

OpenCV stereo transform의 계약은 다음과 같다.

```text
X_right = R_left_to_right X_left + T_left_to_right_m
```

OpenCV optical RDF는 Right/Down/Forward이고, PX4 body FRD는 Forward/Right/Down이며, GNSS
baseline은 North/East/Down이다. 이 좌표계의 값을 변환 없이 합치지 않는다. 오른쪽 카메라
원점을 왼쪽 optical 좌표계에서 표현한 값은 `C_right_in_left = -R^T T`이며 OpenCV `T`의
별칭이 아니다.

## 2. 사전 준비

1. QGroundControl(QGC)에서 P1/P2 Pixhawk의 Accelerometer, Gyroscope,
   Compass/Magnetometer 및 Level Horizon 보정을 완료하고 Board Orientation을 확인한다.
2. 좌·우 WIRIS, P1/P2 Pixhawk, Base GNSS 및 사용할 경우 Target Drone의 전원과 네트워크를
   확인한다.
3. 승인된 YOLO 모델과 calibration source/profile을 준비한다. 모델은 실행 중 자동 다운로드되지
   않는다.
4. NTRIP 계정 값은 설정 JSON에 쓰지 않고 지정된 환경 변수로 제공한다.
5. 저장 공간과 데이터 루트를 확인한다. 기본값은 저장소 루트의 `.local/workstation_data`이다.

```bash
export SASL_WORKSTATION_DATA_ROOT=/absolute/path/to/workstation_data
```

NGII RTS2 프로파일을 사용하는 예는 다음과 같다.

```bash
export NGII_RTS2_USERNAME='SASL'
export NGII_RTS2_PASSWORD='<RTS2 password>'
```

## 3. 실행 전 검사 및 시작

저장소 루트에서는 다음을 실행한다.

```bash
./apps/Workstation/run.sh --check
./apps/Workstation/run.sh
```

현재 `apps/Workstation` 디렉터리에서는 다음을 실행한다.

```bash
./run.sh --check
./run.sh
```

`--check`는 dependency, Workstation import, 선택된 GStreamer/PyAV backend 및 기본 YOLO 모델의
크기와 SHA-256을 검사한다. 실패하면 먼저 원인을 해결한다. 카메라 탐색 없이 UI만 확인하려면
`./run.sh --no-start`, 설정된 Pixhawk endpoint도 연결하려면 `./run.sh --start-pixhawk`를 사용한다.

## 4. Sessions 폴더 생성

`Sessions > Current > Create Session Folder`를 눌러 다음 상위 저장 폴더를 만든다.

```text
.local/workstation_data/recordings/YYYYMMDD_HHMMSS_nanoseconds/
```

Session 생성에는 장비, 동기화, Calibration, Measurement method 또는 Alignment readiness gate가
없다. `OPEN` 또는 `Ready for saving`은 이후 출력의 저장 소유자가 정해졌다는 뜻일 뿐 측정 준비
완료를 의미하지 않는다. 생성 후 State, 폴더명, 생성 시각, 절대 경로와 recovery 오류를 확인한다.

Calibration은 Session 생성 전이나 후에 수행할 수 있다. Session 전 보정은 공용 calibration
root를 사용하고, Session 생성 후의 보정 run과 Alignment 증거는 Session-shared asset으로 저장된다.
Session은 현장 작업의 상위 묶음이고 Measurement가 실제 writable·retry·seal 단위다. 승인
Calibration은 `profiles/<family>/<kind>-<sha256>.json`, Alignment 선택은
`alignment/selections/alignment-<sha256>.json`으로 immutable하게 보관한다. 호환 alias와
`active_alignment.json`은 최신 선택 projection이며 과거 Measurement의 identity로 사용하지 않는다.

## 5. 장비 연결 및 실제 상태 확인

공통 순서는 `장비 선택 → 설정 확인 → Connect → 실제 데이터 수신 확인 → 이상 시 Diagnostics`다.
명령 전송 성공을 장비 동작 성공으로 간주하지 않는다.

### 5.1 WIRIS 좌·우 카메라

1. 실제 좌·우 camera ID/serial이 올바른지 확인한다.
2. RTSP 영상, FPS, native 해상도 및 freshness를 확인한다.
3. 요청 zoom과 각 카메라의 `GZVV` 실제 readback을 따로 확인한다.
4. RTCP/NTP timestamp와 좌우 pair delta를 확인한다.
5. visible encoder readback이 bitrate `0`, QP `17`, GOP `30`, IDR interval `1`인지 확인한다.

UI preview는 최신 frame만 표시할 수 있다. preview frame skip을 native Measurement,
Calibration, Detection 또는 녹화 데이터 누락으로 해석하지 않는다.

### 5.2 P1/P2 Pixhawk

MAVLink heartbeat, Roll/Pitch/Yaw, timestamp, age, sensor/calibration 상태, GNSS fix와 RTCM
injection 결과를 확인한다.

### 5.3 Base GNSS, NTRIP 및 Target Drone

1. Base GNSS를 연결하고 필요하면 `Settings > GNSS`에서 Survey-in 또는 NTRIP을 시작한다.
2. NTRIP은 caster host, port, mountpoint, TLS, GGA policy 및 credential 환경 변수 이름을 쓴다.
3. 상태를 구분한다.
   - `CONNECTED_NO_DATA`: 연결됐지만 stream byte가 없음
   - `STREAMING`: byte는 있으나 아직 유효 RTCM frame이 없음
   - `RTCM_ACTIVE`: 신선한 유효 RTCM frame이 있음
   - `RTCM_STALE`/`STALE`: transport 또는 correction age 초과
   - `RECONNECTING`: 재연결 중
   - `ERROR`: 마지막 실패가 존재함
4. P1/P2별 RTCM frame, MAVLink packet, byte, failure와 마지막 성공 전달 age를 확인한다.
5. 숫자 `fix_type`, 위성 수, eph/epv, correction age와 P1→P2 NED baseline을 확인한다.
6. Target Drone을 쓰면 Telemetry position freshness와 RTCM forwarding counter를 확인한다.

TCP 연결이나 MAVLink send 성공만으로 correction 적용을 판단하지 않는다. `RTCM_ACTIVE`, 실제
distribution counter와 rover `fix_type`을 함께 확인한다. 기본 NGII 설정에서 `RTK_FIXED=6`은
중요한 표시 증거지만 별도로 요구하지 않는 한 모든 측정의 공통 차단 조건은 아니다.

## 6. WIRIS 시각 동기화

1. 좌·우 RTC 및 RTCP/NTP timestamp 상태를 확인한다.
2. 좌우 timestamp delta가 pair 허용 범위 안인지 확인한다. Live/Auto frame은 camera RTCP/NTP
   evidence를 사용한다. FTP HR/OZ는 실제 exposure timestamp가 없으면 host CPTR dispatch proxy만
   기록하며, 이 proxy는 pair 저장을 허용할 수 있어도 precision timing을 승인하지 않는다.
3. camera frame time, application receive/capture time과 filesystem mtime을 구분한다.
4. exposure-time 증거가 없으면 다른 timestamp를 빌리지 않고 `SYNC_UNVERIFIED`로 유지한다.
5. 물리 정렬 후 마지막 동기화 checkpoint를 다시 확인하고 필요하면 재동기화한다.

RTCP/NTP Unix time과 application monotonic time은 서로 다른 clock domain이므로 직접 비교하지
않는다.

## 7. 카메라 Calibration

### 7.1 보정 방식

- `Calibration > Intrinsics`: 카메라별 `Capture / Compute / Profiles`로 `K,D`를 독립 관리한다.
- `Calibration > Extrinsics`: stereo `Capture / Compute / Profiles`로 `R,t`를 계산하고
  검증된 `Left K,D + Right K,D + R,t` bundle을 적용한다.
- Capture를 선택하면 Live/FTP 좌·우 영상 바로 아래의 공통 action bar에서 촬영한다.
  Auto 설정과 카메라별 command/response는 필요할 때만 펼쳐서 확인한다.
- `Intrinsics`: 좌·우 카메라의 `K,D`를 독립적으로 계산한다.
- Full checkerboard calibration: 좌·우 `K,D`와 stereo `R,t`를 계산한다.
- `R,t only`: 선택한 `K,D`를 고정하고 OpenCV `CALIB_FIX_INTRINSIC`으로 stereo `R,t`만 계산한다.

### 7.2 Zoom 설정

1. Calibration 공통 zoom 영역에서 calibration zoom을 선택한다.
2. 양쪽 카메라에 같은 zoom을 명령한다.
3. 좌·우 실제 readback이 존재하고 요청값과 일치하는지 확인한다.
4. Intrinsics는 실제 Measurement의 operating zoom과 native raster에서 계산한다.

`K,D`와 `R,t` source zoom은 독립적이다. 예를 들어 operating 8x의 정확한 `K,D`와 1x에서 구한
`R,t`를 함께 쓸 수 있다. 이 cross-zoom provenance는 기록되지만, Workstation은 zoom 변화에도
rig transform이 유지됨을 증명하지 않으므로 실제 사용할 조합을 별도로 검증한다.

### 7.3 Checkerboard 촬영

1. lossless, overlay 없는 좌·우 native frame pair를 촬영한다.
2. board를 중앙, 상하좌우와 가장자리에 배치한다.
3. 가까운/먼 거리, 크기 변화, roll, yaw 및 pitch perspective를 포함한다.
4. motion blur, 과노출, 일부 잘림과 같은 정면 pose 반복을 피한다.
5. Auto capture가 near-duplicate pose를 거부하면 위치·거리·회전을 바꾼다.

각 capture에는 source frame ID, native/preview/saved raster, crop/scale 계약, overlay 상태,
SHA-256, RTCP/NTP timestamp/delta와 detection 결과가 기록된다. resized preview를 calibration
원본으로 쓰지 않는다.

수동 Live, Auto와 FTP 촬영 모두 `Max Live pair Δt`를 적용한다. Live는 각 native frame에 결합된
RTCP/NTP 시각을 사용하고, FTP는 실제 노출 시각이 아닌 synchronized CPTR host-dispatch proxy를
사용한다. 제한을 초과해도 체커보드가 검출된 각 side 이미지는 `K,D`용으로 저장하며 metadata의
`stereo_pair_saved=false`, `intrinsics_only_sides`와 timing 거부 사유를 기록한다. Auto는 같은 새
frame identity를 한 번만 저장하고, 이 이미지를 카메라별 pose novelty 이력에는 포함하지만 유효
stereo pair 수에는 포함하지 않는다.

### 7.4 이미지 선택과 계산

1. Calibration 공통 입력 영역에서 calibration run을 선택한다.
2. 좌·우 사진과 표시된 촬영 시각의 출처를 확인한다.
3. `Intrinsics > Compute`에서 좌·우 이미지 집합을 각각 계산한다.
4. Stereo에는 같은 capture ID이며 양쪽 detection, timing gate와 near-duplicate filter를 통과한
   pair만 사용한다. `INTRINSICS ONLY` 표시는 좌·우 `K,D` 입력에는 남지만 `R,t`에서는 제외됨을
   뜻한다.
5. `K,D`, calibrated raster, `R,t`, reprojection/view error, effective pose, 3×3 coverage,
   scale/perspective 다양성, uncertainty와 subset 안정성을 확인한다.

한쪽 detection이 실패한 사진은 성공한 쪽 intrinsics에는 남을 수 있지만 stereo 교집합에는
들어가지 않는다. Δt 초과 사진도 같은 방식으로 각 카메라 intrinsics에는 남지만, 명시적 timing
거부는 수동 selection으로도 stereo에 다시 포함할 수 없다. Timing 판정 필드가 없는 기존/import
사진은 호환을 위해 `LEGACY_UNVERIFIED` stereo 후보로 유지되며 동시 노출 증거로 해석하지 않는다.
Stereo 계산에는 최소 3 pair가 필요하지만 정밀한 결과에는 충분히 다양한 pose가 필요하다. Random
stereo-pair validation은 seeded subset으로 focal, baseline과 rotation의 안정성을 검사하며 ground
truth가 아니다.

`calibration_pair_selection.json`이 없으면 자동 선택 정책을 사용한다. 파일이 존재하면 지원되는
v1/v2 schema와 경로·pair identity를 모두 검증한다. JSON 손상, unknown schema, 복구 불가능한
경로 또는 의미적으로 잘못된 selection은 명시적 오류로 계산을 차단하며 전체 사진으로 fallback하지
않는다.

Run 시점에는 표시된 Photo set root에서 `CalibrationInputSnapshot`을 한 번 생성한다. Snapshot은
left/right 파일, stereo pair, include/exclude, selection policy, pair limit, zoom metadata 참조와
각 파일 SHA-256을 고정한다. Worker는 이 snapshot에서만 입력 경로를 얻고 OpenCV 실행 직전에
bytes를 재검증하며, 결과 provenance에 동일 snapshot hash를 기록한다.

### 7.5 품질 판정

기본 품질 정책은 다음을 검사한다.

- 서로 다른 유효 pose 최소 12개
- 이미지 x/y 각 축 union extent 60% 이상
- checkerboard center가 3×3 grid 중 최소 5개 cell에 분포
- 수평·수직 perspective 변화와 최소 20% scale variation
- 안정적인 deterministic subset solve
- focal 표준편차 3% 이하
- principal-point 표준편차가 각 raster 축의 2% 이하
- view별 reprojection error 1 px 이하

중앙 raster에서 `cx`/`cy`가 중심에서 5%를 넘게 벗어나면 경고하고, 10%를 넘거나 영상 밖이면
실패한다.

- `PASS`: 승인 profile로 저장·적용 가능
- `WARNING`: 계산 attempt 저장은 허용하고, 명시적으로 승인한 경우만 적용 가능
- `FAIL`: 계산 attempt와 진단 증거는 저장하고, active profile 적용은 금지
- `UNVERIFIED`: 품질 증거가 없는 과거 profile이며 `PASS`로 해석하지 않음

판정 문자열과 별개로 finite positive focal, finite distortion, positive raster와 in-frame
principal point를 다시 검사한다.

### 7.6 저장, 불러오기 및 적용

1. 품질 결과를 검토하고 저장한다. `PASS`는 승인 profile 경로에, `WARNING`/`FAIL`은
   `attempts/`의 비적용 증거 경로에 저장된다.
2. operating zoom용 좌·우 `K,D`와 사용할 stereo `R,t`를 독립적으로 선택한다.
3. bundle을 적용하고 active path, hash, source zoom 및 native raster를 확인한다.

기본 Extrinsics Apply는 2번에서 독립적으로 선택한 K,D/R,t bundle을 적용한다. Advanced
validation의 `Apply result`는 예외적으로 현재 full-checkerboard result 자체의 K,D/R,t를 하나의
exact bundle로 적용하며, 이전에 선택했던 R,t를 새 full result와 암묵적으로 섞지 않는다.

적용된 profile은 먼저 content-addressed immutable 경로로 publish된다. Measurement는 그 exact
profile byte를 `inputs/calibration/active/`에, 적용된 Alignment selection을
`inputs/alignment/active/`에 복사하고 SHA-256을 manifest에 기록한다.

`WARNING`/`FAIL` attempt 저장은 active alias나 persisted settings를 바꾸지 않는다. Profile parse
실패, zoom metadata 누락·불일치, camera serial/stabilization 불일치, calibrated raster와 현재
native raster 불일치 또는 quality `FAIL`이면 Apply는 fail-closed한다. 자동 load는 persisted
`intrinsics_path`를 우선하며, 그 파일이 잘못됐거나 zoom이 다르면 다른 파일로 자동 대체하지
않는다. 기본 `intrinsics.json` alias가 없을 때만 같은 root의
`intrinsics_zoom_XX.json` fallback을 같은 zoom 검사 아래 쓸 수 있다.

## 8. 물리 정렬 및 Alignment

### 8.1 물리 정렬

1. P1→P2 GNSS NED baseline을 기준으로 물리 baseline을 맞춘다.
2. 카메라 좌우 순서, optical axis, Pixhawk Board Orientation과 rig Roll/Pitch/Yaw를 확인한다.
3. 조정 후 새 sample을 받아 baseline과 자세를 다시 확인한다.
4. 구조물 유격, zoom/lens 상태 및 장착 변경 여부를 확인한다.
5. 마지막 WIRIS 시각 동기화 checkpoint를 기록한다.

### 8.2 Alignment

1. `Alignment > Capture Live Pair`로 최신 decoded 좌·우 native frame과 frame ID/hash, exact
   operating `K,D` applicability 및 관련 snapshot을 고정한다.
2. 필요한 방식을 실행한다.
   - Horizon: 실제 지평선으로 Roll/Pitch 관측, Yaw는 관측 불가
   - Mountain + GNSS: 같은 순서의 능선 correspondence와 GNSS baseline 사용
   - Terrain PnP: native image point와 명시된 datum의 NED 3D point 사용
   - Drone Rotation: target pixel과 동기화된 camera/target GNSS line of sight 사용
   - Drone Multi-view: 여러 시점 관측으로 full pose 계산
   - Combined: 같은 prior/calibration을 공유하는 비중첩 DOF 후보 조합
3. 자동 Horizon/Mountain 결과는 proposal이다. `Use suggestion` 후 solver를 명시적으로 실행한다.
4. candidate ID, source hash, residual, uncertainty, warning, 좌우 observability, optimized DOF,
   prior와 좌표계 계약을 검토한다.
5. 실제로 관측·최적화한 DOF만 적용한다. 낮은 residual은 missing applicability나
   observability를 대신하지 않는다.

산 능선·건물·나무 경계를 Horizon으로 쓰지 않는다. Terrain PnP에는 순서가 일치하는 3D point와
vertical datum이 필요하다. Combined는 최신 후보를 자동 선택하거나 pose를 평균하지 않는다.

## 9. Measurement 설정 및 readiness

Detection 전에 다음을 확인한다.

- 좌·우 stream과 시각 동기화
- operating zoom에 정확히 맞는 `K,D`
- 선택한 stereo `R,t` 또는 complete Alignment candidate
- YOLO model, input size, confidence, IoU와 pair gate
- 8개 method의 Measurement-start Frozen Comparison Input과 primary Pixhawk/RTK 상태
- Pixhawk/GNSS DOF fill
- GNSS/RTK, baseline, 물리 정렬, 저장 경로와 공간

실제 `K,D`를 불러오지 못했거나 calibrated K,D와 fresh zoom readback이 다르면,
좌우 zoom readback과 native raster가 유효할 때 zoom table 기반 이상적 pinhole `K,D`를
자동 생성한다. 결과는 항상
`NOMINAL_UNVERIFIED`이며 zero distortion 가정을 사용한다. 정밀 측정 판정이나 Calibration
`PASS`로 취급하지 않는다.

Measurement는 `field_measurement_v1` 단일 정책을 사용한다. 시간 동기화, pair Δt,
vertical Δ, telemetry age, GNSS fix/accuracy/readiness와 Alignment K,D hash 차이는 측정을
차단하지 않고 `ESTIMATED` advisory로 기록한다. 좌우 객체 identity, K,D/R,t,
pair에 동결된 live geometry 입력, finite/positive two-ray 결과와 좌표
계약은 hard 조건으로 유지한다.

`Measurement` 화면에서 다음을 별도로 확인한다.

- `Evidence collection`: 원본 증거 수집 준비 상태
- `Valid measurement`: exact `K,D`와 완성 가능한 geometry를 포함한 유효 측정 준비 상태

원본 증거는 저장할 수 있지만 거리 계산은 차단될 수 있다. Method table은 Pixhawk,
Checkerboard, Horizon, Mountain + GNSS, Drone Rotation, Drone Multi-view, Terrain PnP와 Combined를
표시하고 입력이 없으면 `BLOCKED`와 이유를 표시한다.

Horizon은 Roll/Pitch에 pair-frozen Pixhawk Yaw와 GNSS translation을 채울 수 있다. Single-target
Drone Rotation은 Yaw에 Pixhawk Roll/Pitch를 쓸 수 있다. Mountain, Multi-view, Terrain과 Combined는
complete 6-DOF여야 한다. Method readiness는 독립적이다.

DOF fill은 Pre 증거가 만든 child가 `COLLECTING`이고 Detection 시작 전일 때만 바꿀 수 있다.
Pre 초안이 없으면 새 child는 `측정 시작` 시 기본 fill 정책과 함께 생성된다. Measurement 생성 시
exact K,D와 각 non-Pixhawk 방식의 최신 성공 Draft, fill이 고정된다. 모든 matched pair는 동일한
snapshot으로 8개 방식을 평가하며 대표 결과는 항상 Pixhawk attitude + GNSS/RTK이다. 준비되지 않은
방식은 각각 BLOCKED로 저장하고 다른 방식으로 primary를 대체하거나 가짜 `R,t`를 만들지 않는다.
P1/P2 GNSS는 side별 최신 finite 3D fix를 독립 갱신하고 앱 실행 중 이전 유효값을 `REUSED`로 쓸 수 있다.

자동 Measurement 생성 자체에는 Alignment, exact `K,D`, GNSS/RTK 또는 ready method gate가 없다.
이 입력들이 부족해도 `Pre` 요청 또는 `측정 시작`이 child를 만들어 실패와 evidence를 올바른 ownership 아래 보존한다.
필요한 geometry가 준비되지 않으면 Detection evidence는 수집할 수 있지만 valid distance만
차단된다.

## 10. Measurement 수행

```text
Create Session Folder
  → Pre 증거 수집(선택, Measurement 초안 자동 생성)
  → 측정 시작(필요 시 child 생성 + Detection 시작)
  → 측정 관찰
  → 측정 종료
  → 자동 저장·검증·봉인
  → 필요 시 Retry Measurement Save
```

1. 필요하면 `Pre` HR/OZ still을 수집한다. 첫 Pre 요청은 child `m000001` 초안을 자동 생성한다.
2. readiness와 effective method의 실제 input, hash와 좌표계를 확인한다.
3. `측정 시작`을 누른다. 초안이 없으면 새 child를 만든 뒤 Detection과 자동 기록을 시작하고,
   초안이 있으면 같은 ID를 재사용한다. 한 Measurement에서는 한 번만 실행할 수 있다.
4. 영상, detection, primary 거리·단위, 8개 method 상태, validity, quality, 동기화와 RTK를 함께 관찰한다.
5. 필요한 데이터가 수집되면 `측정 종료`를 누른다.
6. typed Parquet generation, producer/RTCM drain, 무결성 검사와 자동 봉인을 기다린다.

Measurement 시작 시 calibration, redacted settings, method/fill 정책과 software provenance가
snapshot된다. YOLO model 파일은 Workstation data root의 runtime asset으로만 유지하며 Session이나
Measurement에 복사하지 않는다. 각 matched pair는 source pixels, detection, pair 시점
Pixhawk/GNSS, readiness, effective `R,t`와 `measurement_geometry_sha256`을 보존한다.

### 10.1 HR/OZ still

- HR은 `SIHR TRUE`, `CPTR`, `ICPT` polling을 사용한다.
- OZ는 `SIVI TRUE`, `CPTR`, 별도 `ICPT` polling을 사용한다.
- 목적은 `Pre`, `Post`, `Checkpoint`, `Unspecified`로 기록한다.
- HR/OZ는 RTSP/MP4 recording을 시작하거나 정지하지 않는다.
- FTP 사용 시 capture 전후 storage를 비교해 새 파일만 받고 과거 still로 대체하지 않는다.
- FTP CPTR delta는 `HOST_DISPATCH_PROXY`이고 actual exposure verified가 아니다. Pair 저장과
  precision calibration eligibility를 별도로 검토한다.
- HR manual import는 재인코딩 없이 원본 byte와 size, SHA-256, raster, camera identity를 기록한다.
- 봉인된 child에 늦은 증거를 추가할 때는 새 Detection이 아니라 revision을 쓴다.

## 11. 자동 저장과 봉인

```text
producer 정지
  → typed Parquet 생성
  → producer/RTCM drain
  → Pixhawk injection outcome 확인
  → video 안정화와 ffprobe
  → containment, size, SHA-256 검사
  → package manifest 생성
  → revision 봉인
```

- `SEALED_COMPLETE`: 필수 artifact와 무결성 검사 완료
- `SEALED_INCOMPLETE`: 일부 필수 증거 누락이 manifest에 기록됨
- `RECOVERABLE`: save/drain/semantic-integrity 실패로 재시도 가능

`RECOVERABLE`이면 삭제하거나 새 Session을 만들지 말고 `Retry Measurement Save`를 사용한다.
재시도는 저장·봉인을 다시 수행할 뿐 두 번째 Detection을 허용하지 않는다. 늦은 증거는 이전
manifest hash를 잇는 revision으로 추가한다.

## 12. 추가 Measurement와 Session 종료

- 봉인 후 `측정 시작`을 누르면 새 child가 `m000002`, `m000003` 순서로 증가한다.
- 각 child의 spool, video, capture, data와 revision 경로는 분리된다.
- `Sessions > Current > Close Session`을 누르면 빈 current Measurement 또는 Detection이 종료된
  Post-Capture Measurement를 정상 저장·검증·봉인한 뒤 부모 Session을 자동으로 닫는다.
- Detection이 실행 중이면 먼저 Stop한다. Save/drain/integrity 실패 시 Session은 닫히지
  않고 `RECOVERABLE`로 남으며 `Retry Measurement Save`를 사용한다.
- Close Session은 최신 child manifest 목록과 Session-shared Calibration/Alignment를 포함하는
  상위 `measurement_session_manifest.json` 및 통합 `review.xlsx`를 생성한다.

## 13. 결과 검토

1. `Sessions > Review`에서 `(session_id, measurement_id)`로 child를 선택한다.
2. manifest, 최신 revision과 `SEALED_*` 상태를 확인한다.
3. 누락 artifact, calibration path/hash/zoom과 effective method를 확인한다.
4. 동기화, GNSS/RTK, baseline, validity, per-method outcome/timeout과 원본 frame/video/still을 검토한다.
5. `review.xlsx`는 운영자용 요약으로 사용한다.
6. 전체 고주기 분석에는 다음 typed Parquet를 사용한다.

```text
measurement_pairs.parquet
camera_timeline.parquet
pixhawk_timeline.parquet
gnss_base_timeline.parquet
geometry_comparison.parquet
context.json
```

Workbook은 재생성 가능한 derived artifact이므로 생성 실패가 검증된 Parquet를 무효로 만들지
않는다.

## 14. 실패 및 복구

1. 명령 성공과 실제 장비 상태를 구분한다.
2. `Diagnostics`에서 RTSP, FTP, MAVLink, GNSS, RTCM, time sync와 저장 오류를 확인한다.
3. Calibration 실패 시 zoom metadata, serial, raster, quality, `K,D` 수치와 hash를 확인한다.
4. Valid measurement 차단 시 exact `K,D`, complete pose, observability, Pixhawk/GNSS freshness와
   synchronization을 확인한다.
5. `RECOVERABLE`이면 동일 Measurement에서 `Retry Measurement Save`를 실행한다.
6. `SEALED_INCOMPLETE`이면 manifest의 required/missing/failed 항목을 확인한다.
7. 강제 종료 후 Session recovery와 spool/RTCM 무결성을 먼저 확인한다. 자동 recovery는 정확히
   하나의 open Session만 발견했을 때 수행하며 여러 후보 중 임의 선택하지 않는다.

## 현장용 핵심 체크리스트

- [ ] QGC P1/P2 센서 보정 및 Board Orientation 확인
- [ ] `run.sh --check` 통과
- [ ] `Create Session Folder` 경로와 상태 확인
- [ ] WIRIS ID, 영상, FPS, native raster와 zoom readback 정상
- [ ] P1/P2 attitude, timestamp와 age 정상
- [ ] Base GNSS/NTRIP `RTCM_ACTIVE`, distribution과 숫자 `fix_type` 확인
- [ ] P1→P2 NED baseline 확인
- [ ] 다양한 checkerboard pose로 촬영
- [ ] Calibration coverage, error, uncertainty와 품질 판정 확인
- [ ] operating zoom용 exact `K,D` 적용
- [ ] stereo `R,t` path, hash와 좌표계 확인
- [ ] 물리 baseline/Roll/Pitch/Yaw 정렬 후 새 sample 확인
- [ ] 최종 WIRIS 시각 동기화 checkpoint 확인
- [ ] 필요 시 Alignment candidate observability 검토·적용
- [ ] YOLO, 8개 method Frozen Input/primary 상태와 DOF fill 확인
- [ ] Evidence collection / Valid measurement readiness 확인
- [ ] 필요 시 Pre 증거 요청으로 Measurement 초안 생성 확인
- [ ] `측정 시작`/`측정 종료` 1회 후 자동 finalization 확인
- [ ] `SEALED_COMPLETE`/`SEALED_INCOMPLETE` 또는 실패 시 Retry 확인
- [ ] 다음 child 또는 `Close Session`
- [ ] manifest, `review.xlsx`, Parquet와 원본 증거 검토

## 관련 문서

- 현재 구현과 상세 운용 규칙: `README.md`
- UI 중심 사용자 흐름: `docs/ui/01_USER_WORKFLOW.md`
- 화면 이동 구조: `docs/ui/05_NAVIGATION.md`
- 상태 표시 규칙: `docs/ui/06_STATUS_MODEL.md`
- 명령과 상태 전이 규칙: `docs/ui/07_COMMAND_MODEL.md`
