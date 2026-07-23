# StereoDistance 프로젝트

## 목적

StereoDistance는 좌/우 WIRIS RTSP 영상, YOLO 검출, Pixhawk 자세/RTK 데이터를 결합해 현장에서 객체 거리와 3D 위치를 측정하기 위한 PyQt5 데스크톱 운용 도구다.

이 프로젝트는 Pixhawk 자체 보정을 QGC에서 먼저 완료했다는 전제를 둔다. 앱은 그 이후 현장 연결, WIRIS 시간 확인, WIRIS 내부 파라미터 보정, RTK/RM3100/자세 정렬, YOLO 측정, 세션 저장과 분석을 한 흐름으로 묶는다.

## 프로젝트 목적 상세

이 프로젝트의 목적은 단순히 두 카메라 영상에서 거리를 계산하는 것이 아니다. 현장에서 장비 연결, 시간 동기, 카메라 보정, Pixhawk 자세/RTK 상태, YOLO 검출 품질이 모두 얽혀 있을 때 운용자가 어떤 조건에서 측정값을 신뢰할 수 있는지 판단하게 만드는 것이다.

따라서 StereoDistance는 측정 결과만 보여주는 앱이 아니라, 측정 전제 조건을 확인하고 부족한 조건을 드러내며, 가능한 복구 경로를 제공하는 현장 검증 도구를 목표로 한다.

핵심 판단 기준은 다음이다.

- 좌/우 영상이 실제로 서로 다른 stereo source인지 확인한다.
- 카메라 내부 파라미터와 baseline이 현재 측정 조건과 맞는지 확인한다.
- Pixhawk 자세, RTK/GPS, RM3100/MAG 데이터가 실제로 수신되고 충분히 최신인지 확인한다.
- YOLO 검출 쌍이 시간, 클래스, geometry 기준을 만족하는지 확인한다.
- 기록된 거리 샘플이 어떤 입력과 상태에서 만들어졌는지 세션 단위로 추적 가능하게 남긴다.

## 프로젝트 방법론

이 프로젝트는 `측정값 생성`보다 `측정값을 신뢰할 조건을 만들고 검증하는 과정`을 우선한다.

1. 입력 검증 우선
   - RTSP 포트 open, ROS2 topic 이름, MicroXRCEAgent 실행 여부 같은 간접 신호만으로 성공을 판단하지 않는다.
   - 첫 프레임, publisher/sample, MAVLink heartbeat, 실제 MAG/attitude/GPS 수신처럼 계산에 쓰이는 신호를 직접 확인한다.

2. 단계별 운용
   - WIRIS 연결, 시간 확인, 내부 보정, 파라미터 확인, RTK/MAG/자세 정렬, YOLO 설정, 측정, 저장을 분리된 단계로 둔다.
   - 각 단계는 다음 단계의 전제 조건을 확인하는 역할을 가진다.

3. 출처가 명확한 계산
   - 내부 파라미터, baseline, Pixhawk 자세, GPS/RTK, YOLO bbox의 출처와 age를 가능한 한 표시하고 저장한다.
   - ROS2와 MAVLink처럼 같은 종류의 값을 줄 수 있는 경로도 source를 구분해 기록한다.

4. 3D ray triangulation 중심
   - 기본 측정 방법은 카메라 내부 파라미터와 Pixhawk/RTK 외부 파라미터를 사용하는 3D ray triangulation이다.
   - 단순 disparity 값은 진단 지표로만 취급하고, 필요한 입력이 없을 때 임의의 거리값을 기록하지 않는다.

5. 복구 가능한 현장 운용
   - Rover RTK 운용에서는 MAVLink heartbeat를 먼저 확인해 RTCM 주입, PX4 parameter, shell/재부팅, Ethernet MAVLink bootstrap 같은 제어 경로를 확보한다.
   - ROS2/uXRCE는 자세/GPS/MAG/custom field를 구조화해 받는 고속 telemetry 경로로 유지하되, MAVLink 제어 채널이 없으면 Rover RTK 운용 준비 완료로 보지 않는다.

6. 검증 가능한 저장
   - 측정 CSV, 외부 파라미터, MAG 상태, 영상 파일, 설정을 같은 세션 단위로 묶는다.
   - 사후 분석에서 측정값 자체뿐 아니라 당시 입력 조건과 품질 상태를 함께 확인할 수 있게 한다.

7. Raw 측정 경계
   - 최종 표적은 가로폭 15 m급 UAM이지만, StereoDistance는 표적 PX4/ULog 위치나 별도 UAM telemetry를 수신해 Ground Truth 오차를 계산하지 않는다.
   - 15 m급이라는 정보는 표적 규모 설명일 뿐 bbox 폭 기반 거리식이나 정확도 gate로 사용하지 않는다. 실제 YOLO bbox가 기체 전체 폭을 항상 포함한다고 보장할 수 없기 때문이다.
   - 본 실험 카메라 조건은 WIRIS visible 8배줌, 즉 앱의 명목 `zoom index 8 / 8.0x`를 기준으로 잡는다. 측정에는 명목 `fx` 추정값이 아니라 해당 줌에서 체커보드로 만든 `intrinsics_zoom_08.json`을 적용한다.
   - 앱이 생성·표시·저장하는 결과는 왼쪽 optical center 기준 Raw `left_camera_rdf:[Right,Down,Forward,Range]`뿐이다.
   - P1/P2 Pixhawk 자세와 GPS/RTK는 stereo rig의 ray/baseline 기하와 상태 진단에만 사용한다. 표적 궤적이나 정답 위치로 해석하지 않는다.

## 현재 범위

- QGC를 대체하지 않는다. Pixhawk 보정과 최종 파라미터 저장은 QGC 또는 PX4 shell 흐름을 전제로 한다.
- WIRIS 카메라 연결, RTSP 프레임 수신 확인, WIRIS SDK 명령, still image EXIF 시간 비교를 앱 안에서 다룬다.
- Pixhawk는 MAVLink와 ROS2/uXRCE 경로를 모두 지원한다. 현장에서는 Rover MAVLink heartbeat, ROS2 publisher/sample, MAG device_id 수신 여부를 분리해서 확인한다.
- Rover RTK 운용의 우선 경로는 MAVLink다. Base RTCM은 Rover Pixhawk MAVLink `GPS_RTCM_DATA`로 주입하며, ROS2가 살아 있어도 MAVLink heartbeat가 없으면 RTCM 주입과 PX4 제어는 준비되지 않은 상태로 본다.
- ROS2 연결이 막히면 MAVLink를 복구 경로로 사용해 Pixhawk 쪽 MAVLink stream과 `uxrce_dds_client` 설정을 전송한다. 반대로 ROS2는 MAVLink가 제공하지 못하는 namespace 기반 uORB telemetry와 custom field 확인을 보강한다.
- 측정은 내부 파라미터, Pixhawk 자세, RTK/baseline, 좌/우 YOLO 검출 매칭이 준비됐을 때 3D ray triangulation 결과를 기록하는 것을 목표로 한다.
- 하드웨어 연결 성공 여부는 자동 테스트만으로 증명할 수 없으므로, 진단 스크립트와 현장 smoke check를 함께 사용한다.

## Pixhawk 연결 의도

Pixhawk 연결의 우선 의도는 Rover MAVLink 제어 채널을 먼저 확보하는 것이다. MAVLink heartbeat가 있어야 Base RTCM을 Rover Pixhawk에 `GPS_RTCM_DATA`로 주입하고, PX4 parameter read/write, shell/진단, 재부팅, Ethernet MAVLink bootstrap 같은 복구 작업을 앱 안에서 수행할 수 있다. 따라서 Rover RTK 운용에서는 ROS2 topic sample이 들어오더라도 MAVLink heartbeat가 없으면 준비 완료로 보지 않는다.

ROS2/uXRCE는 PX4 uORB 데이터를 namespace와 topic 기준으로 구조화해서 받는 telemetry 경로다. 자세, GPS, MAG, custom field 감시에는 중요하지만 RTCM 주입이나 PX4 제어 채널을 대체하지 않는다.

따라서 MAVLink는 세 역할을 가진다.

- RTCM 주입 경로: GNSS Base serial에서 분리한 RTCM3 frame을 선택한 Rover Pixhawk MAVLink 연결에 `GPS_RTCM_DATA`로 보낸다.
- 복구/제어 경로: Pixhawk 쪽 MAVLink stream 설정, MAVLink `PARAM_SET`/readback 기반 `UXRCE_DDS_*` 설정, PX4 shell, 재부팅, USB를 통한 Ethernet MAVLink bootstrap을 수행한다.
- telemetry 보조 경로: ROS2가 끝내 붙지 않아도 MAVLink `ATTITUDE`, GPS, GNSS relative, `SCALED_IMU*`로 가능한 자세/GPS/MAG raw telemetry를 계속 받아 정렬과 측정 입력을 최대한 유지한다.

MAVLink-first는 ROS2를 제거한다는 뜻이 아니다. ROS2 전용 `vehicle_magnetometer.device_id`, namespace 기반 topic 식별, arbitrary `/fmu/out/...` custom field, 구조화된 parameter service는 MAVLink만으로 보장하지 않는다. 앱은 제어/RTCM은 MAVLink 기준으로, 고속 uORB telemetry와 source identity는 ROS2 기준으로 분리해서 표시해야 한다.

## Raw 측정과 좌표 방법

좌표계와 축 이름의 기준 계약은 `docs/COORDINATE_CONTRACT.md`와
`wiris_stereo_distance/coordinate_contract.py`에 고정한다.

StereoDistance의 Raw/public 출력은 `left_camera_rdf`다. 원점은 왼쪽
optical center이고 배열과 CSV 의미는 `[Right, Down, Forward, Range]`다.
왼쪽 ray와 오른쪽 ray/origin을 모두 이 frame으로 변환한 뒤
`two_ray_closest_midpoint`로 삼각측량한다. baseline 중앙점 이동이나 public
FRD reorder를 하지 않는다.

Pixhawk attitude는 오른쪽 ray와 GNSS baseline을 `left_camera_rdf`로
변환하는 리그 기하 입력으로만 사용한다. 자세 시각은 측정을 차단하지 않고
`FRESH/STALE/VERY_STALE` 상태와 좌·우 sample age, pair Δt를 UI와 CSV에
노출한다. 오래된 자세로 생성된 측정도 보존하되 UI 품질을 `DEGRADED`로
표시해 사후 분석에서 제외 여부를 결정할 수 있게 한다.

표적 UAM의 명목 가로폭은 YOLO 거리값의 자동 검산 입력으로 쓰지 않는다.
bbox가 실제 기체 전체 폭을 잡는다고 보장할 수 없기 때문이다. WIRIS visible
8배줌에서 앱의 명목 `fx` 추정은 초기 설정 참고용이며, 실제 측정에는
`intrinsics_zoom_08.json`처럼 해당 줌에서 얻은 calibration을 사용한다.

StereoDistance에는 target PX4/ULog truth 수집, UAM 실시간 telemetry,
`left_camera_rdf -> px4_local_ned` 좌표 profile, 시간 매칭, truth/error metric
계산이 없다. 필요한 Ground Truth 평가는 별도 분석 도구에서 명시적인 시간·좌표·기준점
계약을 정의한 뒤 수행해야 하며, StereoDistance의 Raw CSV를 NED 값과 직접 빼면 안 된다.

StereoDistance에는 측정값 보정 계층도 없다. 과거 Origin-BBOX reprojection,
affine bbox-center, distance-binned 3-D bias, Kalman 후처리와 이를 만들던
Simulation-origin helper는 코드·UI·신규 기록 schema에서 제거했다.

## 측정 리그 장착 단순화 결정

측정 리그는 알루미늄 가공 케이스로 WIRIS와 측정용 Pixhawk/GNSS의 축 방향을 맞춘다. 따라서 본 프로젝트의 앱 모델에서는 보어사이트와 레버암을 무시하고, 별도 입력/계산/보정을 제공하지 않는다. 모델 이름은 `nominal-shared-rig-v1`로 둔다.

이 프로젝트의 기준 좌표계는 PX4 관례에 맞춰 world는 NED, 기체 body는 FRD로 둔다. 즉 body +X는 전방, +Y는 우측, +Z는 아래 방향이다. 카메라는 OpenCV/ROS optical frame으로 해석한다. 즉 camera +X는 영상 우측, +Y는 영상 아래, +Z는 렌즈가 바라보는 전방 깊이 방향이다.

전방 감시 카메라의 명목 mount profile은 현재 코드와 동일하게 `front_camera_z_to_body_x_x_to_body_y`로 고정한다.

- camera +X right -> body +Y right
- camera +Y down -> body +Z down
- camera +Z forward -> body +X forward

따라서 25 m baseline은 GNSS antenna 간 baseline이자 WIRIS optical-center 간 effective stereo baseline으로 취급한다. 앱은 `Baseline` 입력값 또는 RTK/GNSS baseline을 그대로 stereo translation의 scale로 사용한다.

측정 계산 순서는 `pixel -> camera optical ray -> body FRD ray -> world NED ray`다. 픽셀과 내부 파라미터로 optical ray를 만들고, 고정 mount rotation을 거쳐 body FRD에 올린 뒤, Pixhawk attitude quaternion으로 world NED에 놓는다. 단순히 축 이름을 맞추기 위해 임의로 축을 교환하지 않는다.

본 실험에서 확인해야 할 조건은 다음이다.

- 좌/우 리그의 WIRIS, Pixhawk, GNSS antenna 장착 위치와 방향이 동일하다.
- 좌/우 WIRIS optical axis가 알루미늄 케이스 기준으로 같은 방향을 본다.
- GNSS antenna 간 25 m baseline이 실제 설치 baseline과 일치한다.
- WIRIS visible 8배줌에서 `intrinsics_zoom_08.json`을 만들어 적용한다.
- 남는 장착 제작 오차는 별도 보정하지 않고 최종 오차에 포함한다.

현재 앱 반영 상태는 다음과 같다.

1. nominal mount profile과 Pixhawk attitude 기반 ray 변환은 유지한다.
2. 개별 장착 보정 UI, 설정 파일 저장 필드, CSV 개별 수치 기록은 제거한다.
3. 세션 row에는 `mount_assumption_model=nominal-shared-rig-v1`와 장착 가정만 남긴다.
4. `triangulation.py`는 per-camera 장착 보정을 적용하지 않고, 좌/우 동일 리그 가정으로 baseline을 계산한다.

현재 범위에서 제외하는 것은 명확히 둔다. 좌/우 리그의 상대 배치가 서로 다르거나, 카메라 축 정렬이 케이스 기준에서 크게 틀어졌거나, 짐벌처럼 비행 중 camera-body 관계가 변하거나, zoom에 따라 optical center가 크게 이동하거나, 열화상/가시광 센서의 optical center가 서로 다른 경우는 별도 보정 모델이 필요하다. 이런 조건에서는 `nominal-shared-rig-v1`로 최종 정확도를 보장하지 않는다.

## 핵심 목표

- 좌/우 WIRIS RTSP 스트림을 연결하고, 포트 open이 아니라 실제 첫 프레임 수신까지 확인한다.
- 카메라 preview와 YOLO/depth 측정을 분리해 스트림 상태 확인과 실제 측정을 구분한다.
- WIRIS still image EXIF 시간 차이를 확인해 좌/우 카메라 시간 오차를 파악한다.
- 체커보드 캡처와 줌별 intrinsics 저장/적용으로 WIRIS 내부 파라미터를 관리한다.
- YOLO 검출 결과를 좌/우 카메라에서 시간, 클래스, geometry gate 기준으로 매칭한다.
- 카메라 내부 파라미터, Pixhawk 자세, GPS baseline을 사용해 3D ray triangulation을 수행한다.
- 측정 결과, 외부 파라미터, Pixhawk MAG 상태, 영상 파일, 설정을 세션 단위로 저장한다.
- 현장 운용자가 연결 상태와 측정 품질을 빠르게 판단할 수 있는 UI를 제공한다.

## 시스템 구성

StereoDistance는 PyQt5 GUI가 현장 워크플로를 조율하고, 계산/스트리밍/저장 로직은 `wiris_stereo_distance` 패키지 안의 모듈들이 담당하는 구조다.

현재는 `gui.py`가 많은 상태와 제어 흐름을 가지고 있으므로 완전한 MVC 구조는 아니다. 다만 측정 계산, 삼각측량, 네트워크 탐색, 설정, 저장 같은 핵심 기능은 별도 모듈로 분리되어 있다.

### 진입점과 GUI

- `main.py`: 앱 진입점.
- `wiris_stereo_distance/gui.py`: PyQt5 메인 GUI와 현장 워크플로 오케스트레이션.
- `wiris_stereo_distance/theme.py`: IBM/Carbon 계열 색상 토큰과 공통 스타일 기준.
- `wiris_stereo_distance/camera_pane.py`: 좌/우 카메라 프리뷰, 스트림 워커 연결, bbox 표시.
- `wiris_stereo_distance/measurement_widgets.py`: 거리 기록 그래프와 삼각측량 품질 표시 위젯.

### 스트림과 측정 계산

- `wiris_stereo_distance/yolo_stream.py`: RTSP 수신, 디코딩, YOLO 추론, 프레임/검출 이벤트.
- `wiris_stereo_distance/measurement_attempt.py`: detection pair마다 선택한 자세, GPS, K/D, checkerboard R/T와 geometry 설정을 immutable snapshot으로 고정하고 generation/session-scoped pair identity를 부여한다.
- `wiris_stereo_distance/measurement_controller.py`: 좌/우 검출 매칭과 3D 측정 계산 연결.
- `wiris_stereo_distance/stereo.py`: 검출 observation, stereo match, disparity 기반 진단 계산.
- `wiris_stereo_distance/triangulation.py`: ray triangulation과 좌표계 변환.
- `wiris_stereo_distance/sync.py`: timestamp buffer, 좌/우 sample 매칭, sync 품질 분류.
- `wiris_stereo_distance/attitude_geometry.py`: 좌/우 Pixhawk quaternion의 상대 자세 계산. 측정 bias 캡처나 차감은 수행하지 않는다.

### 보정, 센서, 장비 제어

- `wiris_stereo_distance/calibration.py`: 체커보드 기반 카메라 보정.
- `wiris_stereo_distance/photo_clock.py`: WIRIS still image EXIF/GPS timestamp 읽기와 좌/우 시간 차이 비교.
- `wiris_stereo_distance/pixhawk_attitude.py`: Pixhawk ROS2/MAVLink 자세와 센서 구독.
- `wiris_stereo_distance/px4_mag_manager.py`: RM3100/PX4 magnetometer 파라미터 상태 계산.
- `wiris_stereo_distance/xrce_agent.py`: MicroXRCEAgent 실행 여부 확인과 시작 보조.
- `wiris_stereo_distance/rtk_geometry.py`: GPS/RTK baseline 계산.
- 장착 모델은 `nominal-shared-rig-v1`로 고정하고, 개별 장착 보정 기능은 앱 범위에서 제외한다.
- `wiris_stereo_distance/wiris_zoom.py`: WIRIS SDK TCP 명령, zoom, Ethernet mode, IP 관련 명령 구성.
- `wiris_stereo_distance/network.py`: 카메라 네트워크 탐색, NIC/RTSP/WIRIS TCP 후보 확인.

### 저장과 설정

- `wiris_stereo_distance/recording_session.py`: 세션 폴더와 CSV/JSON 저장.
- `wiris_stereo_distance/settings.py`: 설정 로드/저장.

## 측정 데이터 흐름

1. 좌/우 WIRIS RTSP 스트림을 연결한다.
2. `YoloStreamWorker`가 프레임과 검출 결과를 GUI로 전달한다.
3. `CameraPane`이 최신 프레임과 bbox 상태를 보관한다.
4. `gui.py`가 class mismatch와 동일 class 다중 후보를 fail-closed한 뒤 detection pair의 midpoint를 기준으로 nearest 좌/우 자세를 고르고, 연속해서 복사한 좌/우 GPS pair, 적용 K/D, checkerboard R/T 또는 GPS baseline을 하나의 immutable `MeasurementAttempt`로 고정한다.
5. Pixhawk 경로는 selected GPS pair에서 NED baseline을 다시 계산하고, right ray와 baseline translation 모두 동일 attempt의 동일 selected left `(w,x,y,z)` quaternion과 app-monotonic sample timestamp basis로 `left_camera_rdf`에 놓는다.
6. `measurement_controller.py`가 attempt만 받아 correspondence를 먼저 검증한 후 triangulation 결과를 계산한다. attempt 생성 뒤 GUI latest state를 조회하지 않는다.
7. 결과는 거리 상태와 그래프에 반영되고, valid row는 계산에 사용한 attempt의 selected-input provenance와 legacy full/path-sensitive canonical hash, 정확히 top-level `attempt_id`/`attempt_session_id`/`created_monotonic_s`/`created_wall_s` 및 nested `intrinsics.selected_profile_path`만 제외하는 path-portable semantic content hash(privacy 또는 pair identity가 아님), 그리고 content hash를 full instance에 결합하는 instance hash를 기록한다.
8. 저장 시 `recording_session.py`가 CSV/JSON/영상 경로를 세션 단위로 정리한다.

S3-B1은 하드웨어 신뢰성을 승인하지 않는다. GPS age/skew 및 stale/RTK
acceptance policy는 S3-B2, atomic recorder/session isolation은 S3-B3,
`baseline_center_frd` public migration은 S3-B4 범위다. 현재 public output은
계속 왼쪽 optical-center 원점의 `left_camera_rdf:[Right,Down,Forward,Range]`다.

## 주요 워크플로

1. QGC에서 Pixhawk 보정 완료.
2. `./run.sh`로 앱 실행.
3. `연결` 탭에서 WIRIS, RTSP, P1/P2 Pixhawk를 확인하고, 수동 SDK/필드 감시는 `고급` 탭에서 수행한다.
4. `시간` 탭에서 WIRIS 사진 EXIF 시간 차이 확인.
5. `보정` 탭에서 체커보드 캡처와 내부 파라미터 보정.
6. `파라미터` 탭의 `내부`, `외부/기하` 세부탭에서 현재 내부/외부 파라미터 확인.
7. `정렬` 탭에서 Baseline, MAG, Pixhawk 자세 상태 확인.
8. `YOLO` 탭에서 모델, 장치, 스트림 설정 조정.
9. `측정` 탭에서 `세션 기록 시작`을 누른 뒤 YOLO/depth 측정을 시작한다.
10. `기록 샘플`에서 Raw `left_camera_rdf` 값과 입력·품질 provenance를 확인한다.
11. `세션 저장`에서 거리 기록, 설정과 메타데이터를 저장한다.
12. 필요하면 `고급 > WIRIS SDK` 또는 `고급 > 필드 감시`에서 저수준 명령/ROS2/MAVLink field를 직접 확인한다.

## 연결 확인 기준

- RTSP는 포트 open만으로 충분하지 않다. `영상 수신 확인`에서 첫 프레임 수신까지 확인한다.
- 카메라 preview는 YOLO 없이 스트림 상태를 확인하는 용도다.
- YOLO 측정은 실제 거리 측정이 필요할 때만 시작한다.
- Pixhawk MAVLink와 ROS2는 별도 경로이며 현장 운용에서는 MAVLink heartbeat를 먼저 확보한다. ROS2는 그 뒤 telemetry/source identity를 보강한다.
- ROS2는 topic 이름만으로 연결 성공으로 보지 않는다. publisher와 sample 수신까지 확인한다.
- MAVLink는 heartbeat와 필요한 메시지 수신을 확인한다.

## 저장 기준

- 유효한 3D triangulation 결과가 있을 때 거리 샘플을 기록한다.
- 세션 저장은 측정 CSV, 외부 파라미터 CSV, Pixhawk MAG 상태, 영상 파일을 같은 세션 폴더에 묶는 것을 목표로 한다.

## 설계 방향

기능과 디자인은 점진적으로 분리한다.

- 계산 로직은 `measurement_controller.py`, `triangulation.py`, `calibration.py`처럼 테스트 가능한 모듈에 둔다.
- GUI는 화면 구성, 사용자 입력, 상태 표시, 워크플로 연결에 집중한다.
- 카메라/Pixhawk/저장 같은 장비 제어 흐름은 장기적으로 service/controller 계층으로 분리한다.
- 스타일과 디자인 토큰은 Python 코드 안에 흩어두지 않고 별도 디자인 기준 문서와 테마 코드로 이동한다.
- 측정 경로는 3D ray triangulation을 우선하며, 단순 disparity 값은 진단 지표로만 취급한다.
- hardware I/O와 UI orchestration은 남아 있는 결합 지점이므로 큰 수정 전에 관련 테스트나 현장 smoke check 기준을 먼저 정한다.
- Pixhawk 연결은 Rover RTCM/PX4 제어를 위해 MAVLink를 먼저 확보하고, 이후 ROS2로 namespace 기반 uORB telemetry를 보강한다. 단, ROS2 전용 device_id, namespace, custom uORB field는 MAVLink 대체 대상이 아니다.

## 현재 결합 지점과 분리 계획

- 스타일 토큰 분리
  - 완료: `wiris_stereo_distance/theme.py`에 IBM/Carbon 색상 토큰을 추가했다.
  - 다음: 전역 QSS 생성 함수를 별도 모듈로 분리한다.
- 그래프 위젯 분리
  - 현재: 일부 그래프는 `measurement_widgets.py`, 일부 Pixhawk/RTK 그래프는 `gui.py`에 있다.
  - 다음: Pixhawk/RTK 그래프를 별도 widgets 모듈로 이동한다.
- 카메라 service 분리
  - 현재: `gui.py`가 stream 시작/중지와 UI 상태를 함께 처리한다.
  - 다음: URL 검증, stream kwargs, start/stop orchestration을 작은 service로 분리한다.
- Pixhawk service 분리
  - 현재: ROS2/MAVLink 연결, 복구, fallback, UI 업데이트가 GUI에 강하게 묶여 있다.
  - 다음: 연결 상태, 후보 탐색, 복구 명령, telemetry source 선택, 로그 이벤트를 service/controller 경계로 나눈다.
- 세션 recording controller 분리
  - 현재: 기록 상태와 table 갱신, 저장 호출이 GUI에 있다.
  - 다음: 기록 row 생성과 저장 orchestration을 GUI 밖으로 이동한다.
- 측정 리그 장착 모델
  - 현재: nominal `camera_mount_profile`과 `nominal-shared-rig-v1` 장착 가정을 유지하고, 좌/우 동일 리그 조건에서 baseline을 사용한다.
  - 다음: 좌/우 리그가 동일하지 않은 새 하드웨어 조건이 생길 때만 별도 보정 모델을 재검토한다.

분리 작업은 한 번에 대규모 재작성하지 않는다. 동작을 잠그는 테스트를 먼저 확인하고, 새 의존성은 추가하지 않으며, 각 단계는 `UPDATE.md`에 한글로 기록한다.

## 디자인 방향

현재 디자인 기준은 `DESIGN.md`를 우선 참고한다. `DESIGN-ibm.md`는 IBM/Carbon 계열 원본 레퍼런스로 둔다.

적용 방향은 IBM/Carbon 계열의 구조적인 운용 대시보드다.

- 차분한 흰색/회색/차콜 기반.
- IBM Blue 계열의 단일 강조색.
- 얇은 구분선과 낮은 장식성.
- 상태, 입력, 버튼, 로그, 그래프가 빠르게 스캔되는 정보 밀도.

Apple식 마케팅/갤러리 레이아웃은 이 앱의 목적과 맞지 않으므로 보조 참고로만 사용한다.

## 문서 역할

- `usage.md`: 실행 방법, 상세 운용 방법, 입력/출력, 제한 사항.
- `PROJECT.md`: 프로젝트 목적, 시스템 구성, 핵심 워크플로, 측정 흐름, 설계 방향, 분리 계획.
- `DESIGN.md`: 프로젝트 UI/스타일 기준.
- `DESIGN-ibm.md`: IBM/Carbon 계열 원본 디자인 레퍼런스와 토큰.
- `../UPDATE.md`: 변경 이력과 인수인계 로그.
- `AGENTS.md`: 코딩 에이전트 작업 규칙.
- `README.md`: 문서 목차.

## 세부 문서

- `testing.md`: 테스트와 검증 방법.

## 현재 주의점

- `gui.py`는 아직 화면 구성, 스타일, 장비 제어, 저장 흐름을 많이 함께 가지고 있다.
- 일부 그래프와 Pixhawk/RTK UI 흐름도 아직 `gui.py`에 남아 있다.
- 기존 PyQt stylesheet는 1차로 IBM/Carbon 기준을 적용했지만, 세부 화면 밀도와 레이아웃은 추가 조정이 필요할 수 있다.
- `stereo_distance_settings.json`은 앱 루트 기준 설정 파일이며, 보정/시간 저장 상대경로도 앱 루트 기준으로 해석한다.
- RTSP 포트 open, ROS2 topic 이름 존재, MicroXRCEAgent 실행만으로 실제 측정 준비 완료를 의미하지 않는다. 첫 프레임, publisher/sample, MAVLink heartbeat, MAG/attitude 수신을 별도로 확인해야 한다.
- 하드웨어 연결이 필요한 검증은 자동 테스트만으로 충분히 확인할 수 없다.
