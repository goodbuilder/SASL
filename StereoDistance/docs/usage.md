# StereoDistance 사용법

## 현재 기능

StereoDistance는 좌/우 WIRIS 영상의 YOLO bbox 중심을 카메라 K로 3차원
광선으로 바꾸고, 두 ray의 최근접점 중점으로 객체 위치를 측정한다.

```text
internal: [Right,Down,Forward] stereo_center_opencv_optical_rdf
public:   [Forward,Right,Down,Range] baseline_center_frd
```

이번 버전에는 측정 결과 보정, bbox center 이동, 결과 융합, 추적 필터가 없다.

## 설치와 확인

```bash
cd StereoDistance
./run.sh --check
```

기본 runtime 의존성은 `requirements/runtime.txt`, OpenCV calibration
의존성은 `requirements/calibration.txt`에 있다. calibration 패키지가
필요하면 다음 helper를 사용한다.

```bash
bash ./scripts/bootstrap_calibration_deps.sh
python3 scripts/check_calibration_deps.py
```

## 실행

```bash
cd StereoDistance
./run.sh
```

주요 순서는 다음과 같다.

1. 좌/우 카메라 주소와 RTSP를 확인한다.
2. YOLO model/class/confidence를 설정하고 양쪽 검출을 시작한다.
3. 저장하거나 새로 구한 intrinsic/checkerboard extrinsic을 적용한다.
4. checkerboard 또는 Pixhawk/RTK 외부기하 source를 선택한다.
5. 좌/우 pair sync와 ray quality를 확인한다.
6. raw measurement recording을 시작한다.

## 카메라와 YOLO

- 좌/우 영상 해상도는 적용한 intrinsic의 width/height와 같아야 한다.
- bbox 중심은 raw image pixel이다.
- calibration distortion 계수가 0이 아니면 측정기가
  `cv2.undistortPoints`를 적용한 뒤 ray를 만든다.
- 좌/우 class, timestamp, vertical center 차이 조건은 pair selection에
  사용한다.
- 수평 pixel 순서나 간격 부호는 validity 조건이 아니다.

## Intrinsic과 checkerboard extrinsic

각 camera intrinsic은 `fx,fy,cx,cy`, distortion, image size를 포함한다.
checkerboard stereo calibration 결과는 다음 OpenCV 계약을 따른다.

```text
X_right = R_left_to_right * X_left + T_left_to_right_m
```

적용된 file과 zoom provenance를 기록한다. Intrinsics가 없거나 image size가
다르면 metric 위치를 신뢰하지 않는다.

## Pixhawk/RTK 경로

Pixhawk 경로에는 다음이 모두 필요하다.

- left/right PX4 attitude
- attitude timestamp pairing
- right GNSS antenna relative to left GNSS antenna의 world NED baseline
- fixed camera-to-body mount contract

Rover 제어와 Base RTCM 주입은 MAVLink 연결을 사용한다. ROS2는 uORB telemetry와
source identity를 제공한다. MAVLink heartbeat가 없는데 ROS2 topic만 보인다고
RTCM 주입 준비가 끝난 것으로 판단하면 안 된다.

## 측정 화면

측정 품질 패널은 다음 raw 정보만 표시한다.

- stereo-center RDF point
- FRD point와 Euclidean Range
- closest-ray gap
- left/right positive depth
- ray angle
- geometry source와 sync warning

`near_parallel_rays`, `non_positive_depth`, missing intrinsics/extrinsics 상태는
거리 숫자로 대체하지 않고 invalid 이유로 표시한다.

## 저장

session recording은 다음을 저장한다.

- 좌/우 bbox와 center pixel
- raw `right/down/forward`
- public `forward/right/down/range`
- ray gap/depth/angle
- `point_frame=stereo_center_opencv_optical_rdf`
- `output_frame=baseline_center_frd`
- `method=two_ray_closest_midpoint`
- intrinsic/extrinsic/timestamp provenance

새 row에 ambiguous `x/y/z`를 추가하지 않는다. UAM/PX4/GPS 값은 source
frame을 명시해 저장하며 camera-frame 값과 직접 축 alias하지 않는다.

## Simulation과 함께 사용

Simulation은 같은 공통 geometry core를 사용한다.

```bash
./Simulation/run_operator_view.sh --no-px4
```

Simulation의 자세오차 launch 인자는 렌더링 camera pose에만 적용된다. raw
estimator는 nominal `±baseline/2` origin과 CameraInfo K만 사용하므로,
자세오차 truth가 runtime 입력으로 유출되지 않는다.

## 보정 연구

보정은 현재 runtime에서 비활성이다. 새 보정법은 raw recording을 입력으로
Calibration/Validation/sealed evaluation/Test를 분리해 후속 작업으로
진행한다. 기존 Simulation report/result는 역사 자료이며 현재 실행 명령으로
간주하지 않는다.

## 문제 해결

### 측정이 `missing_intrinsics`

현재 zoom과 해상도에 맞는 intrinsic을 load/apply한다.

### `missing_checkerboard_extrinsics`

checkerboard source를 선택했다면 stereo calibration에 R/T가 포함돼 있어야
한다.

### `missing_pixhawk_rotation` 또는 `missing_rtk_translation`

좌/우 attitude stream과 RTK baseline source를 각각 확인한다.

### `near_parallel_rays`

먼 거리에서 ray angle이 너무 작거나 camera pose/baseline/K가 잘못됐을 수
있다. 숫자를 강제로 만들지 말고 geometry provenance를 확인한다.

### `non_positive_depth`

한쪽 ray가 카메라 뒤쪽 해를 만들었다. 좌/우 camera mapping, extrinsic 방향,
bbox pair를 확인한다.

## 검증

`testing.md`의 명령으로 공통 geometry, checkerboard/Pixhawk adapter,
pair selection, 좌표 계약을 검증한다.
