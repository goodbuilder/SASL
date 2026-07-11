# Coordinate Conventions

분석일: 2026-07-11  
상태: Phase 0 좌표계 계약 문서. 향후 Simulation 구현과 테스트의 기준이다.

## 1. 핵심 원칙

좌표계는 안전-critical 계약이다. `x/y/z`는 주변 schema 또는 producer code로 axis alias가 확인되기 전까지 모호한 필드로 취급한다.

금지사항:

```text
- Camera RDF == NED로 가정하지 않는다.
- Camera RDF == baseline_center_frd로 가정하지 않는다.
- Raw와 Corrected에서 서로 다른 출력 frame을 사용하지 않는다.
- Euler angle을 내부 회전 표현의 기준으로 사용하지 않는다.
- degree와 radian을 필드명 없이 혼용하지 않는다.
```

## 2. Frame 정의

| Frame | 원점 | 축 | 용도 | 현재 저장소 근거 |
| --- | --- | --- | --- | --- |
| `gazebo_world_enu` | Gazebo world origin | X East, Y North, Z Up | Simulation oracle source | `Simulation/stereo_ground_sim/worlds/stereo_ground_drone.sdf`의 `<world_frame_orientation>ENU</world_frame_orientation>` |
| `px4_local_ned` | EKF2 local origin | X North, Y East, Z Down | PX4 local comparison | `workspace/main/src/px4_msgs/msg/VehicleLocalPosition.msg` |
| `drone_body_frd` | 드론 body reference | X Front, Y Right, Z Down | PX4 body attitude/velocity | `VehicleAttitude.msg`, `VehicleOdometry.msg` |
| `camera_optical_rdf` | 각 카메라 optical center | X Right, Y Down, Z Forward | 픽셀 ray frame | `StereoDistance/wiris_stereo_distance/coordinate_contract.py`, `runtime_deployment_selector.py` |
| `baseline_center_frd` | 좌우 camera optical center 중점 | X Front, Y Right, Z Down | Simulation 최종 출력/비교 frame | 본 요구사항의 고정 출력 frame |

현재 `Simulation` 코드에는 `stereo_center_opencv_optical:RDF(right,down,forward)`와 public `FRD_range(forward,right,down,range)`가 혼재한다. 향후 구현에서는 계산 내부 frame과 public/log frame을 명시적으로 분리해야 한다.

## 3. Gazebo World ENU

요구사항:

- X: East
- Y: North
- Z: Up

현재 world는 ENU로 설정되어 있다. 단, 현재 `stereo_distance_node.py`의 `GAZEBO_ORACLE_AXIS_CONTRACT`는 `x=right,y=forward,z=up`라고 쓰며 stereo rig 배치를 기준으로 world axis를 해석한다. 이는 Gazebo world ENU 자체와 rig-local 해석을 섞어 보이게 하므로 Phase 1/2에서 다음처럼 분리한다.

- `gazebo_world_enu`: world frame 정의.
- `baseline_center_frd`: rig output frame 정의.
- `T_gazebo_world_enu_to_baseline_center_frd`: world point를 baseline center로 옮기는 명시 transform.

## 4. PX4 Local NED

PX4 Local Position은 `VehicleLocalPosition.msg`에 따라 fused local position in NED다.

- `x`: North [m]
- `y`: East [m]
- `z`: Down [m]
- `heading`: radian
- timestamp fields: `timestamp`, `timestamp_sample` in microseconds since system start

`Simulation/stereo_ground_sim/stereo_ground_sim/calibration_csv_recorder.py`의 `fmu_local_position_to_fields()`는 `x/y/z`를 `truth_north_m`, `truth_east_m`, `truth_down_ned_m`로 바꿔 기록한다. 이 패턴은 재사용 가능하다.

## 5. Drone Body FRD

PX4 attitude/odometry conventions:

- `VehicleAttitude.msg`: quaternion `q(w,x,y,z)`는 FRD body frame에서 NED earth frame으로의 Hamilton convention 회전이다.
- `VehicleOdometry.msg`: `pose_frame`과 `velocity_frame` enum이 있으므로 frame id를 runtime에 기록해야 한다.

Simulation 초기 구현은 카메라-Pixhawk lever arm과 방향 불일치를 무시하므로, PX4 body frame은 비교/디버그용이지 카메라 optical frame의 대체가 아니다.

## 6. Camera Optical RDF

각 카메라 optical frame:

- X: Right
- Y: Down
- Z: Forward
- 원점: 해당 카메라 optical center

픽셀 ray 생성 기본식:

```text
x_rdf = (u - cx) / fx
y_rdf = (v - cy) / fy
z_rdf = 1
ray_camera_rdf = normalize([x_rdf, y_rdf, z_rdf])
```

Distortion이 있는 profile은 undistort 후 normalized ray를 만든다. `StereoDistance/wiris_stereo_distance/triangulation.py`의 `normalized_ray_from_pixel()`은 OpenCV distortion 보정 경로가 있어 재사용 후보이다.

## 7. baseline-center FRD

최종 출력 frame:

- 이름: `baseline_center_frd`
- 원점: 좌우 카메라 optical center의 중점
- X: Front
- Y: Right
- Z: Down

Camera RDF와 baseline-center FRD는 축 순서가 다르다.

한 카메라가 rig와 정렬된 이상 상태일 때 direction reorder는 다음과 같다.

```text
camera_rdf = [right, down, forward]
baseline_center_frd = [forward, right, down]
```

그러나 실제 변환은 단순 reorder가 아니라 각 카메라 pose와 attitude error를 포함한 rigid transform이어야 한다.

## 8. Transform 공식

### 8.1 명명 규칙

`R_B_A`는 A frame 좌표 성분을 B frame 좌표 성분으로 바꾸는 rotation matrix다.

```text
v_B = R_B_A @ v_A
```

`o_A_B`는 B frame 원점의 위치를 A frame 성분으로 표현한 것이다.

### 8.2 Position transform

A frame point `p_A`를 B frame point `p_B`로 변환:

```text
p_B = R_B_A @ (p_A - o_A_B)
```

동차 행렬을 쓰면:

```text
T_B_A = [ R_B_A  -R_B_A @ o_A_B ]
        [ 0 0 0           1       ]
[p_B, 1]^T = T_B_A @ [p_A, 1]^T
```

### 8.3 Direction/ray transform

Ray 방향은 원점 이동을 적용하지 않는다.

```text
d_B = R_B_A @ d_A
```

Ray origin은 position transform을 적용한다.

```text
ray_origin_B = R_B_A @ (ray_origin_A - o_A_B)
ray_dir_B = normalize(R_B_A @ ray_dir_A)
```

### 8.4 Ray triangulation output

두 camera ray를 공통 `baseline_center_frd`로 옮긴 뒤 closest points를 계산한다.

```text
left_ray  = (o_L_frd, d_L_frd)
right_ray = (o_R_frd, d_R_frd)
closest_left, closest_right, gap = closest_points(left_ray, right_ray)
point_frd = 0.5 * (closest_left + closest_right)
```

유효성 조건:

- 각 카메라 depth가 positive.
- 두 ray의 intersection angle이 너무 작지 않음.
- ray gap이 threshold 이하.
- point가 FOV/clip 조건에 부합.
- 모든 output은 `baseline_center_frd`다.

## 9. Quaternion convention

내부 저장과 로그는 다음을 권장한다.

- Quaternion field order는 명시적으로 `w,x,y,z` 또는 `x,y,z,w`를 필드명에 포함한다.
- PX4 `VehicleAttitude`와 `VehicleOdometry`는 Hamilton `(w,x,y,z)`다.
- SciPy `Rotation.from_quat()`는 일반적으로 `(x,y,z,w)` 입력을 요구하므로 boundary adapter를 둔다.
- Gazebo pose JSON orientation은 현재 `stereo_distance_node.py`에서 `x,y,z,w` 텍스트를 사용한다. 이를 내부 quaternion으로 읽을 때 순서 변환을 명시한다.

Euler angle은 설정/표시/사용자 sweep 입력에만 사용하고, 내부 회전 조합의 기준 표현으로 사용하지 않는다.

## 10. 현재 코드에서 확인된 혼용/위험

| 위험 | 현재 근거 | 권장 조치 |
| --- | --- | --- |
| Gazebo ENU와 rig-local 해석 혼재 | `stereo_distance_node.py`의 `GAZEBO_ORACLE_AXIS_CONTRACT`와 `_gazebo_world_position_to_stereo_center_rdf()` | `gazebo_world_enu`와 `baseline_center_frd` transform을 분리 |
| `stereo_center_opencv_optical` RDF와 public FRD 혼재 | `runtime_deployment_selector.py`, `operator_recorder.py`, `operator_panel.py` | public/log schema를 `baseline_center_frd`로 고정하고 adapter를 명명 |
| 평행 stereo disparity 공식 중심 | `stereo_distance_node.py::_position_from_centers()`, `RuntimeStereoCameraContract.triangulate_left_observation()` | Phase 1에서 general two-ray triangulation core 추가 |
| `target_reference_link` 없음 | `x500_15m/model.sdf` link 목록: `base_link`, rotors only | Phase 2에서 link 추가 또는 explicit substitute metadata |
| FPS 불일치 | SDF/launch 60 Hz, `config/camera_model.env` 30 Hz, 요구 20 Hz | Phase 2에서 기본값 통일 |
