# Ground Stereo Drone Simulation

Gazebo Sim과 ROS 2 Humble을 이용한 지상 고정 스테레오 카메라 시뮬레이션이다.
현재 버전의 위치 추정기는 DKL 방식과 같은 두 3차원 광선의 최근접점 중점을
사용한다. 이번 단계에는 위치 보정, bbox 이동, 결과 융합, 추적 필터가 없다.
보정 연구는 raw 측정 계약을 고정한 뒤 별도 단계에서 다시 시작한다.

> 현재 method/UI/topic/CSV 변경은 이전 보정 검색의 formal cycle과 호환되지
> 않는다. 기존 `results/`, `reports/`, `recordings/`, `captures/`는
> 역사 산출물로만 보존한다. 새 보정 실험은 Calibration부터 새로 시작해야
> 하며, Truth/RMSE가 보이는 기본 operator 화면은 sealed evaluation/Test
> 화면으로 사용할 수 없다.

## 현재 측정 계약

각 카메라의 bbox 중심 픽셀 `(u, v)`와 ROS `CameraInfo.K`에서 광학
RDF 단위 광선을 만든다. bbox를 만든 영상의 width/height는 해당
`CameraInfo.width/height`와 정확히 같아야 하며, Simulation 계약에서는
`CameraInfo.D`가 모두 0이어야 한다. 다르면 측정을 rescale하지 않고
fail-closed 처리한다.

```text
camera optical RDF = [Right, Down, Forward]
ray = normalize([(u-cx)/fx, (v-cy)/fy, 1])
```

시뮬레이션 raw estimator의 명목 카메라 pose는 다음과 같다.

```text
left origin  = [-baseline/2, 0, 0] m
right origin = [+baseline/2, 0, 0] m
frame        = stereo_center_opencv_optical_rdf
rotation     = identity for both nominal cameras
```

두 직선 `O_l + s d_l`, `O_r + t d_r` 사이 거리의 제곱을 최소화해
각 광선의 최근접점 `P_l`, `P_r`를 구하고, raw 3차원 점을
`(P_l + P_r)/2`로 정한다. 단순 `fB/(u_l-u_r)` 거리 투영식은
runtime과 synthetic 입력 생성 모두에서 사용하지 않는다.

내부 점과 공개 출력은 축 순서를 명시적으로 분리한다.

```text
point_center_rdf_m = [Right, Down, Forward]  # stereo_center_opencv_optical_rdf
estimated_position = [Forward, Right, Down, Range]  # baseline_center_frd
Range = norm(point_center_rdf_m)
```

다음 경우 측정은 invalid다.

- `zero_baseline`
- `near_parallel_rays`
- `non_positive_depth`
- `invalid_range`
- `missing_camera_info`
- `invalid_camera_info_grid`
- `invalid_camera_info_intrinsics`
- `pixel_grid_mismatch`
- `unsupported_camera_distortion`

광선 품질은 두 최근접점 간 거리, 좌/우 ray depth, ray 교차각으로 함께
발행한다.

## 자세오차 주입과 raw estimator

`left_*_error_deg`, `right_*_error_deg` launch 인자는 생성되는 SDF의
실제 카메라 pose만 바꾼다. 즉, 렌더링된 영상에는 장착 자세오차가 반영되지만
그 오차값은 raw estimator에 전달되지 않는다. 이는 runtime truth leakage를
막기 위한 의도적인 계약이다.

synthetic detection도 목표의 stereo-center RDF 점을 양쪽 카메라의 전체
pinhole pose와 각 카메라 K로 투영해 만든다. 깊이를 먼저 계산한 뒤 수평
픽셀 간격을 역산하는 우회 경로는 없다.

## 구성

- 지상 고정 카메라 2대
- 기본 baseline: 25 m
- 기본 해상도: 1280x720
- 기본 왜곡: off
- target: 약 15 m급 UAM/드론 모델
- viewer: `rqt_gui`, 선택적으로 RViz2
- 위치 추정 core: `sasl_common/stereo_geometry.py`
- ROS adapter: `stereo_ground_sim/ray_measurement.py`

## 실행

PX4 없이 빠르게 확인:

```bash
./Simulation/run_operator_view.sh --no-px4
```

PX4 SITL 포함:

```bash
./Simulation/run_operator_view.sh
```

PX4 소스는 checkout에 포함하지 않는다. 다른 장비에서는 repo-relative
경로에 다시 생성한다.

```bash
git clone --branch release/1.16 --depth 1 --recurse-submodules \
  https://github.com/PX4/PX4-Autopilot.git Simulation/PX4-Autopilot
PYTHON_EXECUTABLE="$PWD/Simulation/.venv/bin/python" \
  make px4_sitl_default -j"$(nproc)" -C Simulation/PX4-Autopilot
```

자세오차 예:

```bash
./Simulation/run_operator_view.sh --no-px4 \
  left_roll_error_deg:=0.4 right_yaw_error_deg:=-0.3
```

이 인자는 영상 생성 pose에만 적용되고 nominal ray estimator에는 들어가지
않는다.

## ROS topic 계약

| Topic | Type | Data contract |
| --- | --- | --- |
| `/left_camera/camera_info` | `sensor_msgs/CameraInfo` | left K |
| `/right_camera/camera_info` | `sensor_msgs/CameraInfo` | right K |
| `/stereo/detections` | `Float32MultiArray` | 좌/우 raw bbox |
| `/stereo/estimated_position` | `Float32MultiArray` | `[Forward,Right,Down,Range]`, `baseline_center_frd` |
| `/target_distance` | `Float32` | estimated Range |
| `/ground_truth` | `Float32` | TruthRange |
| `/ground_truth_position` | `Float32MultiArray` | `[Forward,Right,Down,TruthRange]` |
| `/stereo/distance_error` | `Float32` | `EstimatedRange-TruthRange` |
| `/stereo/distance_comparison` | `Float32MultiArray` | `[TruthRange,EstimatedRange,RangeError]` |
| `/stereo/triangulation_status` | `String` JSON | ray validity/quality contract |

`/stereo/triangulation_status`의 JSON 필드는 다음과 같다.

```json
{
  "valid": true,
  "reason": "ok",
  "closest_gap_m": 0.0,
  "left_depth_m": 1200.0,
  "right_depth_m": 1200.0,
  "ray_angle_deg": 1.2,
  "point_frame": "stereo_center_opencv_optical_rdf",
  "frame": "baseline_center_frd",
  "method": "two_ray_closest_midpoint"
}
```

invalid 상태의 유한하지 않은 수는 JSON `null`로 발행한다.

## 기록

operator recorder와 calibration CSV recorder는 다음 raw 정보만 측정 결과로
저장한다.

- 좌/우 bbox 및 중심 픽셀
- raw `Forward/Right/Down/Range`
- `valid/reason`
- closest-ray gap
- 좌/우 ray depth
- ray angle
- point/output frame과 method

PX4/Gazebo truth를 함께 저장할 수 있지만 이는 offline calibration 또는
validation용이다. raw estimator의 입력으로 사용하지 않는다. recorder
manifest의 `postprocessing=none`은 저장 중 보정이 적용되지 않았음을 뜻한다.

## 좌표계

- PX4 local position: `px4_local_ned:[North,East,Down]`
- PX4 body: `px4_frd:[Forward,Right,Down]`
- 카메라 광학: `opencv_optical_rdf:[Right,Down,Forward]`
- 내부 측정점: `stereo_center_opencv_optical_rdf:[Right,Down,Forward]`
- 공개 측정 배열: `baseline_center_frd:[Forward,Right,Down,Range]`
- Gazebo world/model pose: world frame이며 카메라 축과 직접 alias하지 않는다.

현재 world 생성 계약에서 target pose의 world `+X/+Y/+Z`는 각각
baseline-center `Right/Forward/Up`으로 변환한다. 따라서 공개 `Forward`는
world Y의 부호를 보존하고, `Down=camera_height-world_Z`이며, scalar range만
제곱합으로 항상 양수다.

source-frame truth와 camera-frame 점을 비교하려면 원점과 회전을 포함한 명시적
`T_source_to_stereo_rdf`가 필요하다. NED의 `x/y/z`를 카메라 RDF 값으로
직접 복사하면 안 된다.

## 검증

```bash
source /opt/ros/humble/setup.bash
PYTHONPYCACHEPREFIX=/tmp/sasl-ray-pycache \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="$PWD:$PWD/StereoDistance:$PWD/Simulation/stereo_ground_sim" \
  Simulation/.venv/bin/python -m pytest -p no:cacheprovider \
  StereoDistance/tests Simulation/tests
```

핵심 회귀는 비대칭 K를 포함한 full projection 왕복, 회전된 두 카메라에서
수평 픽셀이 같아도 유효한 ray 교차, 평행 ray와 뒤쪽 교차 거부,
RDF→FRD 공개 축 순서를 검사한다.

이번 측정법의 구현 gate는 위 회귀, active source의 구식 거리식 부재,
Simulation launch 생성, ROS package build가 모두 통과하는 것이다. 실제 측정
성능의 pass/fail은 아직 선언하지 않는다. 후속 Calibration/Validation에서
거리·자세 조건별 축 RMSE와 ray 품질 허용범위를 먼저 고정한 뒤 판정한다.

## 역사 연구 자료

`Simulation/reports/`와 `Simulation/results/`에는 이전 보정 연구와
formal experiment 기록이 남아 있다. 이 파일은 결과 provenance 보존용이며,
현재 ray-only runtime에서 실행 가능한 기능이나 현재 성능 증거를 뜻하지 않는다.
Git history에서 당시 실행 코드와 문서를 확인할 수 있다.
