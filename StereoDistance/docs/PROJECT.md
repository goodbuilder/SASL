# StereoDistance 프로젝트

## 목적

두 WIRIS 카메라의 영상에서 같은 객체를 검출하고, 카메라 내부/외부
파라미터와 두 3차원 광선을 이용해 stereo baseline 중심 기준
`Forward/Right/Down/Range`를 측정한다. 카메라·YOLO·Pixhawk·RTK 상태,
측정 품질, 기록을 한 GUI에서 운용한다.

현재 버전은 raw 위치 추정만 제공한다. 결과 후처리 보정, bbox 이동,
결과 융합, 추적 필터는 포함하지 않으며 보정은 후속 단계다.

## 핵심 측정 흐름

```text
left/right frame
  -> YOLO bbox center pixel
  -> calibrated K + distortion removal
  -> camera optical RDF unit ray
  -> checkerboard R/T or Pixhawk attitude + RTK baseline
  -> stereo-center RDF rays
  -> closest-point midpoint
  -> [Right,Down,Forward]
  -> reorder
  -> [Forward,Right,Down,Range] baseline_center_frd
```

공통 최근접-ray 기하는 `sasl_common/stereo_geometry.py`에 있다.
StereoDistance adapter는 `wiris_stereo_distance/triangulation.py`,
pairing과 orchestration은 `stereo.py`와 `measurement_controller.py`에
있다.

## 측정 입력

- 좌/우 YOLO bbox 중심과 검출 timestamp
- 카메라별 K, distortion coefficients, calibration width/height와 일치하는
  bbox pixel grid
- 다음 중 하나의 외부기하
  - checkerboard stereo `R_left_to_right/T_left_to_right`
  - 좌/우 Pixhawk attitude와 RTK/GNSS baseline

Intrinsics가 없으면 metric ray를 만들지 않는다. Pixhawk 경로에서는 attitude와
baseline이 모두 있어야 하고, checkerboard 경로에서는 stereo extrinsic이
있어야 한다.

## Frame 계약

- 개별 카메라 ray: `opencv_optical_rdf:[Right,Down,Forward]`
- 내부 결과: `stereo_center_opencv_optical_rdf:[Right,Down,Forward]`
- 공개 결과: `baseline_center_frd:[Forward,Right,Down,Range]`
- Pixhawk body: `px4_frd:[Forward,Right,Down]`
- PX4 local truth: `px4_local_ned:[North,East,Down]`

자세한 변환과 부호는 `COORDINATE_CONTRACT.md`를 따른다.

## 유효성

객체 pair는 timestamp, vertical center 차이, class 조건으로 선택한다. 수평
픽셀 순서나 수평 간격의 부호는 pair validity 조건이 아니다. 회전된 두
카메라에서는 좌우 수평 픽셀이 같아도 두 3차원 ray가 교차할 수 있다.

geometry는 다음을 거부한다.

- zero baseline
- near-parallel rays
- 한쪽 이상 non-positive ray depth
- non-finite/non-positive range
- bbox pixel grid와 camera intrinsic calibration grid 불일치

유효 결과에는 closest-ray gap, 좌/우 depth, ray angle을 함께 기록한다.
구현 pass 조건은 full-pose projection 왕복, checkerboard transform 역변환,
RDF→FRD 축 순서, invalid reason 회귀와 전체 테스트 통과다. 실제 현장 정확도
pass/fail은 이번 변경의 주장 범위가 아니며 후속 Calibration/Validation에서
거리·자세 조건별 허용 오차를 별도로 고정한다.

## 외부기하 경로

### Checkerboard

OpenCV 계약 `X_right=R*X_left+T`를 사용한다. right camera origin은
left frame에서 `-R^T*T`이고, 두 camera origin 중점을 stereo center로
정한다.

### Pixhawk/RTK

RTK baseline은 world NED vector다. left attitude로 baseline을 body FRD,
fixed camera mount를 거쳐 optical RDF로 변환한다. right attitude는 right
camera ray의 orientation을 left optical frame으로 옮기는 데 사용한다.

## 시스템 구성

- `main.py`: GUI 진입점
- `gui.py`: 운용 화면과 orchestration
- `camera_pane.py`, `yolo_stream.py`: 영상/검출
- `calibration.py`: intrinsic/checkerboard extrinsic
- `triangulation.py`: StereoDistance pose adapter
- `measurement_controller.py`: pair+geometry 결합
- `alignment_managers.py`, `pixhawk_attitude.py`, `rtk_geometry.py`:
  sensor pose/baseline 입력
- `recording_session.py`: raw session 기록

## 저장 계약

현재 측정 row는 의미가 명시된 필드만 사용한다.

- raw bbox와 중심 pixel
- raw stereo-center `right/down/forward`
- public `forward/right/down/range`
- K/distortion과 extrinsic provenance
- closest gap, ray depths, ray angle
- point/output frame, method
- timestamp/sync provenance

ambiguous `x/y/z`나 left-camera 점 alias를 새 schema에 추가하지 않는다.
PX4/GPS/UAM truth를 함께 저장할 수 있지만 이는 offline 비교용이며 raw
estimator 입력이 아니다.

## 운용 연결

Rover 제어와 Base RTCM 주입은 MAVLink heartbeat/transport를 먼저 확인한다.
ROS2 uORB topic은 structured telemetry와 source identity에 사용한다. 두
채널의 연결 상태를 서로 대신하는 것으로 간주하지 않는다.

카메라 timestamp는 가능한 경우 RTCP/NTP를 사용하고, pair timestamp basis를
record에 남긴다.

## 보정 후속 단계

새 보정 연구는 이 raw ray schema와 좌표 계약을 고정한 뒤 별도 formal cycle로
진행한다. Calibration truth와 runtime input을 분리하고, Validation으로 method를
고른 뒤 sealed evaluation/Test를 실행해야 한다. 현재 UI에서 보이는 truth와
RMSE는 sealed/Test 환경이 아니다.

기존 보정 결과와 보고서는 Git history 및 Simulation의 역사 산출물로
보존하지만 현재 runtime 기능이나 현재 성능 근거로 해석하지 않는다.

## 문서

- `usage.md`: 설치와 운용
- `COORDINATE_CONTRACT.md`: frame/축/변환
- `testing.md`: 검증
- `../UPDATE.md`: 변경 이력
- `DESIGN.md`: UI 설계
