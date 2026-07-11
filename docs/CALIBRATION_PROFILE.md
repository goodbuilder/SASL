# Calibration Profile Contract

분석일: 2026-07-11  
상태: Phase 0 calibration/profile 요구사항. Simulation과 StereoDistance가 공유할 수 있는 profile 원칙을 정의한다.

## 1. 목적

Calibration profile은 특정 카메라/줌/해상도/rig 상태에서 다음 정보를 재현 가능하게 저장한다.

1. 좌우 카메라 intrinsics.
2. distortion coefficients.
3. 좌우 optical center와 baseline-center 관계.
4. 좌우 roll/pitch/yaw bias 또는 quaternion correction.
5. profile 유효 범위.
6. validation 결과.
7. Simulation Raw -> Corrected 적용 방향.

## 2. 현재 저장소 근거

| 기능 | 현재 경로 | 재사용 가능성 |
| --- | --- | --- |
| Intrinsic/extrinsic dataclass | `StereoDistance/wiris_stereo_distance/calibration.py` | width/height, camera_matrix, distortion, extrinsics schema 참고 가능 |
| General ray triangulation | `StereoDistance/wiris_stereo_distance/triangulation.py` | normalized ray, distortion, closest-ray core 참고 가능 |
| Runtime residual/fusion profiles | `Simulation/stereo_ground_sim/stereo_ground_sim/runtime_deployment_selector.py`, `runtime_guard_policy.py`, `reprojection_profile.py` | 현재 bbox/disparity profile 계열 참고 가능. 단 자세 오차 profile schema와 분리 필요 |
| Profile loading in node | `Simulation/stereo_ground_sim/stereo_ground_sim/stereo_distance_node.py` | runtime profile loading 패턴 참고 가능 |
| Existing profile docs | `Simulation/docs/experiment-workflow.md`, `Simulation/docs/experiment-gates.md`, `Simulation/docs/method-taxonomy.md` | 실험 운영 문서로 참고, 본 문서는 좌표/보정 profile 계약 고정 |

## 3. Schema version

권장 top-level 형태:

```json
{
  "schema_version": "simulation_calibration_profile.v1",
  "profile_id": "...",
  "created_at_utc": "...",
  "applies_to": {},
  "camera_model": {},
  "rig_extrinsics": {},
  "attitude_bias": {},
  "application_contract": {},
  "validation": {}
}
```

`schema_version`은 필수다. 기존 JSON/CSV profile을 읽더라도 export 시 이 versioned schema로 감싸야 한다.

## 4. 1280 x 720 호환성

기본 영상 조건:

- width: 1280 px
- height: 720 px
- FPS: 20 Hz

Profile은 다음 중 하나를 명시해야 한다.

1. `image_width_px=1280`, `image_height_px=720`에만 적용.
2. 다른 해상도에서 생성됐으나 intrinsics scale rule이 포함됨.
3. 호환 불가.

필수 필드:

| 필드 | 단위 | 설명 |
| --- | --- | --- |
| image_width_px | px | profile calibration width |
| image_height_px | px | profile calibration height |
| nominal_fps_hz | Hz | profile validation FPS |
| compatible_width_px | px/list | 적용 가능 width |
| compatible_height_px | px/list | 적용 가능 height |
| scale_intrinsics_allowed | bool | 해상도 변경 시 scale 허용 여부 |

## 5. Zoom 식별

한 실험 run 동안 zoom은 고정한다. 줌별 intrinsic/profile을 사용한다.

필수/권장 필드:

| 필드 | 설명 |
| --- | --- |
| zoom_id | string/int. 예: `8x`, `zoom_index=8` |
| horizontal_fov_rad | radian |
| focal_length_px | derived 또는 explicit |
| zoom_source | `simulation_config`, `wiris_reported`, `manual` |
| zoom_valid_range | profile validation range |

현재 `Simulation/config/camera_model.env`에는 WIRIS 1x..30x HFOV 근사 table이 문서화되어 있고, launch에는 `zoom_index`, `horizontal_fov_rad`가 있다. Phase 4에서 profile lookup key로 연결한다.

## 6. 좌우 roll/pitch/yaw bias

입력/표시는 degree를 허용하되, 내부 적용은 quaternion 또는 rotation matrix를 사용한다.

권장 필드:

```json
"attitude_bias": {
  "left_camera": {
    "bias_roll_deg": 0.0,
    "bias_pitch_deg": 0.0,
    "bias_yaw_deg": 0.0,
    "rotation_quat_wxyz_camera_raw_to_camera_corrected": [1, 0, 0, 0]
  },
  "right_camera": {
    "bias_roll_deg": 0.0,
    "bias_pitch_deg": 0.0,
    "bias_yaw_deg": 0.0,
    "rotation_quat_wxyz_camera_raw_to_camera_corrected": [1, 0, 0, 0]
  }
}
```

주의:

- `*_deg`는 사람이 읽는 입력/출력용이다.
- 실제 transform 적용 방향은 quaternion/matrix field name에 명시한다.
- degree/radian suffix가 없는 angle field는 금지한다.

## 7. Rotation 적용 방향

Profile은 다음을 반드시 명시한다.

| 필드 | 의미 |
| --- | --- |
| `R_corrected_camera_raw_camera` | raw camera RDF vector를 corrected camera RDF로 회전 |
| `R_baseline_center_frd_camera_corrected_rdf` | corrected camera RDF vector를 output frame으로 회전 |
| `camera_origin_baseline_center_frd_m` | camera origin position in output frame |

예시 계약:

```text
ray_camera_corrected = R_corrected_camera_raw_camera @ ray_camera_raw
ray_baseline_frd = R_baseline_center_frd_camera_corrected_rdf @ ray_camera_corrected
origin_baseline_frd = camera_origin_baseline_center_frd_m
```

Raw와 Corrected 모두 최종 출력은 `baseline_center_frd`다. Corrected가 profile rotation을 적용하더라도 output frame 이름과 축은 바뀌지 않는다.

## 8. Validation 결과

Profile은 사용 가능성을 판단할 수 있도록 validation 결과를 포함한다.

| 필드 | 단위 | 설명 |
| --- | --- | --- |
| validation_dataset_id | text | dataset/sweep id |
| validation_output_frame | text | 반드시 `baseline_center_frd` |
| validation_range_min_m, validation_range_max_m | m | 적용 range |
| validation_bearing_min_deg, validation_bearing_max_deg | deg | 방위 범위 |
| validation_altitude_min_m, validation_altitude_max_m | m | 고도 범위 |
| raw_rmse_m | m | raw error RMSE |
| corrected_rmse_m | m | corrected error RMSE |
| raw_p95_m | m | raw p95 |
| corrected_p95_m | m | corrected p95 |
| ray_gap_p95_m | m | triangulation ray gap p95 |
| invalid_rate | ratio | invalid frame 비율 |
| pass_criteria | JSON/text | pass/fail 기준 |
| validation_pass | bool | 결과 |

Ground truth가 Gazebo oracle인지 PX4 비교값인지 명확히 구분한다. Simulation profile validation의 truth는 Gazebo oracle이어야 한다.

## 9. Simulation과 StereoDistance 공통 사용 원칙

1. 공통 profile 파일은 해상도, zoom, frame, axis contract가 맞을 때만 공유한다.
2. Simulation에서 Gazebo oracle로 학습한 profile을 실제 `StereoDistance`에 적용할 때는 domain gap을 별도 validation으로 표시한다.
3. 실제 장비 RTK/RTSP/time sync 항목은 현재 Simulation profile의 필수 요소가 아니다.
4. `StereoDistance`는 실제 카메라/Pixhawk topic과 device settings를 가진다. Simulation profile은 이를 덮어쓰지 않는다.
5. Profile에는 absolute local path를 필수 의존성으로 넣지 않는다. repo-relative path 또는 artifact manifest를 사용한다.
6. 기존 bbox/disparity residual profile과 자세-bias calibration profile은 목적이 다르므로 schema를 분리하되, metadata/id/validation 방식은 재사용한다.
