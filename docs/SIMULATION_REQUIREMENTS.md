# Simulation Requirements

분석일: 2026-07-11  
상태: Phase 0 요구사항 문서. 이 문서는 기능 코드 수정 없이 저장소를 분석한 결과와 향후 구현 요구사항을 고정한다.

## 1. 목적

`Simulation`은 Gazebo와 PX4 SITL을 사용하여 스테레오 카메라 자세 오차가 거리 및 3차원 위치 측정에 미치는 영향을 분석하고, 보정 프로파일의 효과를 검증하기 위한 연구용 애플리케이션이다.

실제 장비용 `StereoDistance`와 분리한다.

- `StereoDistance/`: 실제 WIRIS Enterprise 카메라 2대, Pixhawk, RTSP/RTK/자력계 등 실제 장비 운용 코드.
- `Simulation/`: Gazebo world, PX4 SITL, 가상 좌우 카메라, 자세 오차 sweep, raw/corrected/ground-truth 비교 연구 코드.

## 2. 현재 저장소 근거

| 항목 | 현재 확인 경로 |
| --- | --- |
| Simulation ROS 2 패키지 | `Simulation/stereo_ground_sim/package.xml`, `Simulation/stereo_ground_sim/setup.py` |
| Simulation launch | `Simulation/stereo_ground_sim/launch/stereo_ground_sim.launch.py` |
| Gazebo world/model | `Simulation/stereo_ground_sim/worlds/stereo_ground_drone.sdf`, `Simulation/stereo_ground_sim/models/x500_15m/model.sdf` |
| 현재 거리 노드 | `Simulation/stereo_ground_sim/stereo_ground_sim/stereo_distance_node.py` |
| 현재 operator UI/recorder | `Simulation/stereo_ground_sim/stereo_ground_sim/operator_panel.py`, `operator_recorder.py`, `recorder_panel.py` |
| PX4 비교 CSV recorder | `Simulation/stereo_ground_sim/stereo_ground_sim/calibration_csv_recorder.py` |
| 실제 장비 앱 | `StereoDistance/main.py`, `StereoDistance/wiris_stereo_distance/*` |
| 공통 truth helper | `sasl_common/truth_coordinates.py` |
| PX4 ROS messages | `workspace/main/src/px4_msgs/msg/*.msg` |

## 3. 포함 범위

향후 `Simulation` 구현은 다음을 포함한다.

1. Gazebo world 기반 oracle ground truth.
2. 좌우 가상 카메라.
3. PX4 SITL 드론 모델.
4. 좌우 카메라 roll/pitch/yaw 오차 주입.
5. 공통 자세 오차와 상대 자세 오차 구분.
6. 일반화된 두 광선 기반 삼각측량.
7. Raw 및 Corrected 결과 동시 기록.
8. 고정 출력 frame: `baseline_center_frd`.
9. 자세 오차 sweep.
10. 거리·방위·고도별 오차 분석.
11. 보정 프로파일.
12. 로그 및 통계.
13. PX4 Local, Global, GPS 비교값 기록.
14. `VehicleOdometry`는 선택적 보조 비교값으로만 취급하고 초기 구현 필수 범위에서는 제외한다.

## 4. 제외 범위

현재 연구 단계에서는 다음을 제외한다.

- 실제 RTK Survey-in 절차.
- 실제 RTK Fixed 수렴 시간.
- 실제 STUS 시간 동기화.
- 실제 RTSP 네트워크 지연.
- 실제 RM3100 priority 설정.
- 자기장 간섭 모델.
- 카메라-Pixhawk lever arm.
- 카메라-Pixhawk 방향 불일치.
- 좌우 영상 시간 offset.
- 대기 굴절 및 heat haze.
- rolling shutter.
- H.264 압축 artifact.
- 실제 구조물 진동 모델.
- 전체 센서 오차 예산.

## 5. 기본 카메라 조건

요구사항의 기본 조건은 다음이다.

| 항목 | 요구사항 | 현재 구현/설정에서 확인된 값 | 조치 |
| --- | --- | --- | --- |
| 해상도 | 1280 x 720 | SDF와 launch 기본값 1280 x 720 (`stereo_ground_drone.sdf`, `stereo_ground_sim.launch.py`) | 유지 |
| 프레임률 | 20 Hz | SDF/launch 기본 camera update 60 Hz, `config/camera_model.env` 30 Hz, overlay 20 Hz | Phase 2에서 camera update 기본값을 20 Hz로 통일 |
| 카메라 모델 | WIRIS Enterprise OZ visible RTSP stream 근사 | `config/camera_model.env`에 WIRIS 광학 줌 근사 HFOV table 존재 | profile schema와 연결 |
| 줌 | 한 실험 동안 고정, 줌별 profile | `zoom_index`, `horizontal_fov_rad` launch/config 필드 존재 | Phase 4에서 calibration profile과 연동 |
| distortion | profile로 관리 가능해야 함 | SDF distortion은 모두 0, `StereoDistance/wiris_stereo_distance/calibration.py`는 distortion 계수 지원 | Phase 1/4에서 명시 |

## 6. Ground Truth 정의

### 6.1 절대 Ground Truth

Simulation의 절대 ground truth는 Gazebo world에서 드론 모델의 `target_reference_link` world pose다.

현재 저장소 상태:

- `Simulation/stereo_ground_sim/models/x500_15m/model.sdf`에는 `target_reference_link`가 없다.
- 현재 사용 가능한 대체 link/model pose는 `x500_15m::base_link` 또는 모델 pose다.
- `Simulation/stereo_ground_sim/stereo_ground_sim/stereo_distance_node.py`는 `gz topic -e -t /world/stereo_ground_drone/pose/info --json-output`에서 `target_model_name`과 일치하는 pose를 찾는다.
- 현재 변환은 `x=right`, `y=forward`, `z=up`이라는 하드코딩 성격의 Gazebo-to-stereo RDF 변환을 사용한다.

요구사항:

1. Phase 2에서 `target_reference_link`를 모델에 추가하거나, 추가 전까지 `base_link` 대체 사용 사실을 로그 metadata에 남긴다.
2. Gazebo world pose는 oracle로만 사용한다.
3. Gazebo oracle과 PX4 비교값을 모두 기록하되, PX4 EKF 값은 ground truth로 승격하지 않는다.

### 6.2 PX4 비교값의 역할

| PX4 데이터 | 역할 | Ground Truth 여부 | 근거 |
| --- | --- | --- | --- |
| `vehicle_gps_position` / `SensorGps` | 가상 GPS 비교값 | 아니오 | `workspace/main/src/px4_msgs/msg/SensorGps.msg`는 GPS sensor 측정값 |
| `VehicleGlobalPosition` | PX4 EKF 전역 위치 비교값 | 아니오 | `VehicleGlobalPosition.msg` 주석: estimator fused global position |
| `VehicleLocalPosition` | PX4 EKF Local NED 비교값 | 아니오 | `VehicleLocalPosition.msg` 주석: fused local position in NED |
| `VehicleOdometry` | 선택적 보조 비교값 | 아니오, 초기 구현 제외 | `VehicleOdometry.msg`는 pose_frame/velocity_frame이 runtime enum이므로 별도 frame 검증 필요 |
| `VehicleAttitude` | 드론 body FRD -> NED attitude 비교 또는 디버그 | 아니오 | `VehicleAttitude.msg` q는 Hamilton `(w,x,y,z)`, FRD body to NED earth |

`Local`, `Global`, `Odometry`가 동일 EKF에서 파생되면 서로 독립 ground truth로 취급하지 않는다.

## 7. Raw와 Corrected 정의

### Raw

Raw는 주입된 카메라 자세 오차를 포함한 실제/시뮬레이션 camera pose와 관측값에서 계산한 baseline-center 기준 3D 결과다.

- 출력 frame은 항상 `baseline_center_frd`다.
- 보정 프로파일을 적용하지 않는다.
- invalid 조건은 명시적으로 기록한다.

### Corrected

Corrected는 같은 관측값에 calibration profile 또는 correction profile을 적용한 결과다.

- 출력 frame은 Raw와 같은 `baseline_center_frd`다.
- 보정값이 바뀌어도 출력 frame 자체가 바뀌면 안 된다.
- corrected가 invalid이면 raw와 oracle은 그대로 기록하고 corrected validity/status만 invalid로 둔다.

## 8. 현재 단계의 단순화 가정

1. Gazebo world는 ENU로 유지한다.
2. 카메라 rig 원점은 좌우 optical center 중점으로 둔다.
3. 카메라-Pixhawk lever arm은 무시한다.
4. 카메라-Pixhawk 방향 불일치는 무시한다.
5. 좌우 영상 시간 offset은 무시한다.
6. 대기/압축/rolling shutter/진동 영향은 무시한다.
7. 한 run에서는 zoom을 고정한다.
8. ideal correspondence와 bbox 기반 mode를 분리한다.
9. 자세 오차를 현재 주요 연구 대상으로 제한한다.
10. Odometry는 선택적 보조 비교값으로만 둔다.

## 9. 예상 실행 시퀀스

1. Scenario/config load
   - camera model: resolution, FPS, zoom, intrinsic/profile id
   - rig geometry: baseline, camera optical centers
   - target/drone scenario: spawn/trajectory/sweep
2. Gazebo world launch
   - ENU world
   - left/right camera model or sensor frame
   - PX4 SITL target model
   - `target_reference_link` pose availability check
3. ROS-Gazebo bridge
   - camera image/camera_info
   - future ground-truth pose bridge or `gz topic` reader
4. PX4 SITL attach/start
   - `run_operator_view.sh` currently starts `make px4_sitl gz_x500` with `PX4_GZ_STANDALONE=1`, `PX4_GZ_WORLD=stereo_ground_drone`, `PX4_GZ_MODEL_NAME=x500_15m`
5. Per-frame measurement
   - read/produce left/right observations
   - compute rays in each camera RDF
   - transform rays into common `baseline_center_frd`
   - generalized closest-ray triangulation
   - publish/log Raw
   - apply profile and publish/log Corrected
6. Oracle/comparison logging
   - Gazebo oracle pose transformed to `baseline_center_frd`
   - PX4 local/global/gps snapshots recorded with frame metadata
7. Sweep/statistics
   - attitude error sweep
   - distance/bearing/altitude bins
   - raw/corrected/oracle error summary

## 10. 완료 기준

Simulation 구현은 다음을 만족해야 완료로 볼 수 있다.

1. Gazebo oracle, Raw, Corrected가 모두 같은 `baseline_center_frd` frame으로 비교된다.
2. `target_reference_link` 또는 명시된 대체 link의 world pose가 로그 metadata에 기록된다.
3. PX4 Local/Global/GPS 값은 비교값으로 기록되며 ground truth로 표시되지 않는다.
4. 카메라 RDF, body FRD, PX4 NED, Gazebo ENU 변환이 명명된 함수와 테스트로 고정된다.
5. 일반 두 광선 삼각측량은 ray gap, intersection angle, positive depth, invalid reason을 기록한다.
6. 1280 x 720, 20 Hz 기본 조건이 config/launch/docs에서 일치한다.
7. Raw/Corrected 출력 frame이 보정 적용 여부에 따라 바뀌지 않는다.
8. run metadata, frame measurements, camera observations, PX4 states, triangulation debug, optimization history가 별도 schema로 기록된다.
