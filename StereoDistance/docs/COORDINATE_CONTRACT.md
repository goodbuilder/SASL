# Coordinate Contract

StereoDistance와 Simulation의 ray 측정 좌표 계약이다. 기하 상수와 변환의
코드 source of truth는 `sasl_common/stereo_geometry.py`이고,
`wiris_stereo_distance/coordinate_contract.py`는 PX4/UAM 계약과 함께 이를
재사용한다.

## Frame과 축

| Frame id | Axis order | 의미 |
| --- | --- | --- |
| `ned` | `North, East, Down` | PX4 world/local |
| `px4_frd` | `Forward, Right, Down` | PX4 body |
| `opencv_optical_rdf` | `Right, Down, Forward` | 개별 카메라 광학 ray |
| `stereo_center_opencv_optical_rdf` | `Right, Down, Forward` | stereo baseline 중심 내부 측정점 |
| `baseline_center_frd` | `Forward, Right, Down` | 공개 위치 배열 |
| `px4_local_ned` | `North, East, Down` | ULog/PX4 truth |

배열이나 CSV의 `x/y/z`는 주변에 `axis_contract`가 없으면 사용하지 않는다.
현재 raw 측정 schema는 의미가 드러나는 `right/down/forward` 필드만 쓴다.

## 카메라와 body 고정 mount

```text
camera +X Right   -> body +Y Right
camera +Y Down    -> body +Z Down
camera +Z Forward -> body +X Forward
```

mount profile id는
`front_camera_z_to_body_x_x_to_body_y`이다. Pixhawk quaternion은 body FRD의
orientation이고, pixel ray는 camera optical RDF이므로 두 값을 직접 같은
축으로 취급하지 않는다.

## Pixel에서 ray

입력 bbox 중심 `(u,v)`와 K의 `fx,fy,cx,cy` 단위는 pixel이다.
각 observation의 `width/height`가 해당 camera intrinsic의 calibration
`width/height`와 정확히 같을 때만 이 K를 적용한다. 현재 runtime은 pixel/K
grid를 암묵적으로 rescale하지 않는다. StereoDistance는 불일치한 쪽에 따라
`left_pixel_grid_mismatch`, `right_pixel_grid_mismatch` 또는
`stereo_pixel_grid_mismatch`로 거부하고, Simulation은
`pixel_grid_mismatch`로 거부한다.

```text
d_camera = normalize([(u-cx)/fx, (v-cy)/fy, 1])
frame(d_camera) = opencv_optical_rdf
```

StereoDistance calibration에 lens distortion 계수가 있으면
`cv2.undistortPoints`로 raw pixel을 normalized point로 바꾼 뒤 ray를 만든다.
Simulation은 distortion이 꺼진 `CameraInfo.K`를 사용한다.

## 두 ray의 3차원 교차

두 카메라 원점과 ray를 동일한
`stereo_center_opencv_optical_rdf`로 변환한다.

```text
L(s) = O_l + s d_l
R(t) = O_r + t d_r
P_l, P_r = argmin ||L(s)-R(t)||²
point_center_rdf_m = (P_l + P_r)/2
Range = ||point_center_rdf_m||
```

`point_center_rdf_m`은 `[Right,Down,Forward]`이고
`point_frame=stereo_center_opencv_optical_rdf`이다. 공개 배열은 순서만
명시적으로 바꾼다.

여기서 `stereo_center`는 원점 계약이다. StereoDistance에서는 frame의
orientation을 left camera optical RDF와 같게 유지하고 원점만 좌/우 카메라
원점의 중점으로 옮긴다. Simulation에서는 두 명목 카메라가 같은 optical
orientation을 사용한다. 따라서 `baseline_center_frd`도 이 center optical
orientation을 FRD 순서로 바꾼 것이며, world NED 또는 baseline vector 방향을
축으로 삼는 frame이 아니다.

```text
point_center_frd_m = [Forward,Right,Down]
estimated_position = [Forward,Right,Down,Range]
output_frame = baseline_center_frd
```

RDF 점을 FRD field에 복사하거나, center 점을 left/right camera field로
기록하지 않는다.

## 카메라 pose 출처

### Checkerboard 경로

OpenCV extrinsic은 다음 계약이다.

```text
X_right = R_left_to_right * X_left + T_left_to_right_m
right_origin_in_left = -R_left_to_right^T * T_left_to_right_m
```

baseline center는 left/right origin의 중점이다.

### Pixhawk/RTK 경로

- GNSS/RTK baseline vector: world NED
- left/right attitude: PX4 body FRD orientation
- camera ray: optical RDF

baseline은 left attitude와 fixed mount를 통해 left optical RDF로 옮긴다.
right attitude는 right ray의 orientation에만 적용하며 right origin을 다시
회전시키지 않는다. 측정 경로에서는 두 ray에 사용한 detection-time attitude
sample과 동일한 left/right sample로 baseline을 변환한다. UI 표시용 최신
attitude와 측정 시각 attitude를 섞지 않는다.

### Simulation 경로

raw estimator는 명목 pose만 사용한다.

```text
O_l = [-B/2,0,0]
O_r = [+B/2,0,0]
R_center_from_left = I
R_center_from_right = I
```

launch 자세오차는 SDF의 물리 카메라 pose에만 들어간다. 이 값을 estimator에
전달하면 runtime truth leakage가 되므로 금지한다.

## Invalid와 품질

- `zero_baseline`: 원점이 동일함
- `near_parallel_rays`: 해가 수치적으로 불안정함
- `non_positive_depth`: 한쪽 이상 교차 depth가 카메라 뒤쪽임
- `invalid_range`: range가 유한한 양수가 아님
- `*_pixel_grid_mismatch`: bbox pixel grid와 K calibration grid가 다름

유효 결과에는 `closest_gap_m`, `left_depth_m`, `right_depth_m`,
`ray_angle_deg`, `point_frame`, `output_frame`, `method`를 남긴다.

## Truth 비교

PX4 NED, GPS/RTK, Gazebo world와 camera RDF는 서로 다른 frame이다. truth를
camera-frame axis와 비교하려면 origin, rotation, target reference point를
포함한 명시적 transform이 필요하다. scalar range만 비교할 때도 두 producer가
같은 기준점까지의 Euclidean range를 뜻하는지 확인한다.

Truth는 offline calibration/validation에만 사용할 수 있고 raw runtime
estimator의 입력으로 사용할 수 없다.
