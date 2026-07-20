# 테스트와 검증

## Python 문법

```bash
PYTHONPYCACHEPREFIX=/tmp/sasl-ray-pycache \
python3 -m compileall -q sasl_common StereoDistance/wiris_stereo_distance \
  Simulation/stereo_ground_sim/stereo_ground_sim
```

## 핵심 geometry

```bash
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="$PWD:$PWD/StereoDistance:$PWD/Simulation/stereo_ground_sim" \
Simulation/.venv/bin/python -m pytest -p no:cacheprovider \
  StereoDistance/tests/test_stereo_geometry_common.py \
  StereoDistance/tests/test_triangulation.py \
  StereoDistance/tests/test_measurement_controller.py \
  Simulation/tests/test_ray_measurement.py
```

이 테스트는 다음 좌표/기하 계약을 확인한다.

- 비대칭 left/right K와 회전된 pose의 full projection 왕복
- 회전된 camera에서 수평 pixel이 같아도 유효한 두 ray
- near-parallel ray 거부
- camera 뒤쪽 depth 거부
- checkerboard `right_origin=-R^T*T`
- PX4 body FRD ↔ camera optical RDF mount
- stereo-center RDF `[Right,Down,Forward]` → public FRD
  `[Forward,Right,Down]`
- Simulation nominal origin `[-B/2,+B/2]`
- detection-time attitude를 ray와 RTK translation에 함께 사용
- bbox/CameraInfo 또는 bbox/calibration pixel grid 불일치 거부
- Simulation nonzero distortion 거부와 Gazebo world-Y→Forward 부호 보존

## 전체 테스트

```bash
PYTHONPYCACHEPREFIX=/tmp/sasl-ray-pycache \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="$PWD:$PWD/StereoDistance:$PWD/Simulation/stereo_ground_sim:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages" \
Simulation/.venv/bin/python -m pytest -p no:cacheprovider \
  StereoDistance/tests Simulation/tests
```

## 정적 제거 gate

활성 source/test/script/config/launch에서 이전 수평차 기반 estimator 명칭이
남지 않았는지 case-insensitive 검색한다. 역사 `results/reports/recordings`
산출물은 이 gate에서 제외한다.

또한 삭제된 runtime module을 import하는 고아 script가 없는지 확인하고,
shell script에는 `bash -n`, Python에는 AST/compile 검사를 실행한다.

## Runtime smoke

ROS/Gazebo가 준비된 장비에서는 다음을 확인한다.

1. 양쪽 `CameraInfo.K` 수신 전에는 `missing_camera_info`.
2. 영상 grid와 `CameraInfo.width/height`가 같고 `D=0`일 때만
   `/stereo/triangulation_status.valid=true`.
3. `/stereo/estimated_position`은 `[F,R,D,Range]`.
4. synthetic full projection 왕복 오차와 closest gap이 수치 허용범위 안.
5. 자세오차 인자를 바꿔도 estimator parameter에는 그 값이 추가되지 않음.

기본 operator UI는 truth를 표시하므로 sealed evaluation/Test smoke로 사용하지
않는다.
