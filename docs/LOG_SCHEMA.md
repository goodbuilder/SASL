# Simulation Log Schema

분석일: 2026-07-11  
상태: Phase 0 schema 요구사항. 현재 UI/CSV 구현과 향후 Simulation 전용 schema를 분리한다.

## 1. 분리 원칙

실시간 UI와 파일 로그는 목적이 다르므로 schema를 분리한다.

- UI: 사람이 run 상태와 주요 error를 빠르게 확인하는 최소 상태.
- 파일 로그: 재현 가능한 분석, sweep, profile 검증, Gazebo oracle/PX4 비교를 위한 완전한 기록.

현재 구현은 `Float32MultiArray` 중심이며 header/frame id가 없다. 향후 schema는 모든 topic/CSV row에 frame, axis contract, timestamp source를 명시해야 한다.

## 2. 현재 로그/UI 경로

| 종류 | 현재 경로 | 주요 내용 |
| --- | --- | --- |
| Operator panel | `Simulation/stereo_ground_sim/stereo_ground_sim/operator_panel.py` | `/ground_truth_position`, `/stereo/estimated_position`, `/stereo/corrected_position`를 Forward/Right/Down 그래프로 표시 |
| Recorder panel | `Simulation/stereo_ground_sim/stereo_ground_sim/recorder_panel.py` | recording start/stop, truth source 선택: local/global/gps, sim oracle checkbox |
| Operator recorder | `Simulation/stereo_ground_sim/stereo_ground_sim/operator_recorder.py` | `telemetry.csv`, optional MP4, debug rosbag |
| Calibration CSV recorder | `Simulation/stereo_ground_sim/stereo_ground_sim/calibration_csv_recorder.py` | PX4 local/global/gps snapshots, estimated/detections/disparity, optional sim truth |
| Attitude sweep output | `Simulation/scripts/attitude_error_experiment.py` | per-case distance CSV, CPU CSV, summary CSV, metadata |
| Debug bag topics | `operator_recorder.py`의 `DEBUG_BAG_TOPICS` | image, detection, distance, position, correction profile topics |

## 3. UI 표시 최소 정보

UI는 다음만 최소 표시한다.

| 그룹 | 필드 | 단위 | 설명 |
| --- | --- | --- | --- |
| Run status | scenario_id, run_id, phase | text | 현재 scenario/sweep/case |
| Camera | left/right image preview | image | raw 또는 detection overlay |
| Frame contract | output_frame=`baseline_center_frd` | text | UI 상단 고정 표시 |
| Oracle | gt_forward_m, gt_right_m, gt_down_m, gt_range_m | m | Gazebo oracle in baseline-center FRD |
| Raw | raw_forward_m, raw_right_m, raw_down_m, raw_range_m | m | uncorrected triangulation result |
| Corrected | corrected_forward_m, corrected_right_m, corrected_down_m, corrected_range_m | m | profile-applied result |
| Error | raw_error_range_m, corrected_error_range_m, raw_error_norm_m, corrected_error_norm_m | m | oracle 대비 error |
| Validity | raw_valid, corrected_valid, invalid_reason | bool/text | frame별 valid 상태 |
| Timing | sim_time_s, frame_index, source_age_ms | s/index/ms | freshness 확인 |
| PX4 snapshot | local/global/gps status age | ms/text | 비교 source 수신 여부만 요약 |

UI에서는 PX4 Local/Global/GPS를 ground truth로 표시하지 않는다.

## 4. 파일 로그 구조

권장 output layout:

```text
runs/<run_id>/
  run_metadata.json
  frame_measurements.csv
  camera_observations.csv
  px4_states.csv
  triangulation_debug.csv
  optimization_history.csv
  summary.json
  manifest.txt
```

CSV는 작은 분석/plot 편의를 위해 유지하고, 복합 metadata는 JSON으로 둔다.

## 5. `run_metadata.json`

필수 필드:

| 필드 | 단위/형식 | 설명 |
| --- | --- | --- |
| schema_version | string | 예: `simulation_log.v1` |
| run_id | string | 날짜/UUID 기반 |
| created_at_utc | ISO-8601 | wall-clock 생성 시각 |
| app | string | `Simulation` |
| scenario_name | string | scenario id |
| git_commit | string/null | 가능하면 commit sha |
| dirty_worktree | bool/null | 가능하면 기록 |
| gazebo_world | string | 예: `stereo_ground_drone` |
| gazebo_world_frame | string | `gazebo_world_enu` |
| target_model_name | string | 예: `x500_15m` |
| target_reference_link | string | `target_reference_link` 또는 substitute |
| target_reference_link_status | enum | `present`, `substitute_base_link`, `missing` |
| output_frame | string | `baseline_center_frd` |
| camera_width_px | px | 1280 |
| camera_height_px | px | 720 |
| camera_fps_hz | Hz | 기본 20 |
| zoom_id | string/int | profile lookup key |
| intrinsic_profile_id | string | left/right intrinsic id |
| calibration_profile_id | string/null | corrected 적용 profile |
| sweep_axes | list | attitude sweep axes |
| excluded_effects | list | lever arm/time offset/atmosphere 등 |

## 6. `frame_measurements.csv`

한 row는 하나의 synchronized measurement frame이다.

| 필드 | 단위 | 설명 |
| --- | --- | --- |
| run_id | text | run id |
| frame_index | index | monotonic frame index |
| sim_time_s | s | Gazebo/ROS simulation time |
| ros_time_s | s | ROS node clock time |
| wall_time_utc | ISO-8601 | optional |
| output_frame | text | `baseline_center_frd` |
| gt_source | text | `gazebo_world_pose` |
| gt_link_name | text | `target_reference_link` 또는 substitute |
| gt_forward_m, gt_right_m, gt_down_m, gt_range_m | m | oracle in output frame |
| raw_forward_m, raw_right_m, raw_down_m, raw_range_m | m | raw result |
| corrected_forward_m, corrected_right_m, corrected_down_m, corrected_range_m | m | corrected result |
| raw_error_forward_m, raw_error_right_m, raw_error_down_m, raw_error_range_m | m | raw - GT |
| corrected_error_forward_m, corrected_error_right_m, corrected_error_down_m, corrected_error_range_m | m | corrected - GT |
| raw_valid, corrected_valid | bool | validity |
| raw_invalid_reason, corrected_invalid_reason | text | invalid reason |
| profile_id | text | corrected profile id |
| left_roll_error_deg, left_pitch_error_deg, left_yaw_error_deg | deg | injected left attitude error |
| right_roll_error_deg, right_pitch_error_deg, right_yaw_error_deg | deg | injected right attitude error |

## 7. `camera_observations.csv`

| 필드 | 단위 | 설명 |
| --- | --- | --- |
| run_id, frame_index | text/index | join key |
| camera_id | enum | `left`, `right` |
| image_stamp_s | s | image source timestamp if available |
| frame_id | text | camera optical frame id |
| camera_optical_frame | text | `left_camera_optical_rdf` / `right_camera_optical_rdf` |
| width_px, height_px | px | image size |
| fx_px, fy_px, cx_px, cy_px | px | intrinsics used |
| distortion_model | text | e.g. `none`, `plumb_bob` |
| distortion_coeffs | JSON/text | coefficients |
| observation_mode | enum | `ideal_projection`, `bbox` |
| u_px, v_px | px | selected correspondence center |
| bbox_x1_px, bbox_y1_px, bbox_x2_px, bbox_y2_px | px | bbox mode only |
| confidence | 0-1 | detector confidence |
| ray_origin_forward_m, ray_origin_right_m, ray_origin_down_m | m | ray origin in baseline-center FRD |
| ray_dir_forward, ray_dir_right, ray_dir_down | unitless | ray direction in baseline-center FRD |
| observation_valid | bool | valid flag |
| observation_invalid_reason | text | reason |

## 8. `px4_states.csv`

PX4 states are comparison snapshots, not oracle truth.

| 필드 | 단위 | 설명 |
| --- | --- | --- |
| run_id, frame_index | text/index | nearest/join key |
| source_topic | text | `/fmu/out/...` |
| source_msg_type | text | px4_msgs type |
| px4_timestamp_us | us | `timestamp` |
| px4_timestamp_sample_us | us | `timestamp_sample` if available |
| ros_receive_time_s | s | ROS receive time |
| source_role | enum | `comparison`, not `ground_truth` |
| source_frame | text | `px4_local_ned`, `wgs84_lla`, `vehicle_odometry_pose_frame_*` |
| axis_contract | text | explicit axis order |
| local_north_m, local_east_m, local_down_m | m | VehicleLocalPosition |
| global_lat_deg, global_lon_deg, global_alt_m | deg/deg/m | VehicleGlobalPosition |
| gps_lat_deg, gps_lon_deg, gps_alt_m | deg/deg/m | SensorGps/vehicle_gps_position |
| odom_pose_frame, odom_velocity_frame | enum/text | VehicleOdometry only if enabled |
| q_w, q_x, q_y, q_z | unitless | PX4 quaternion if recorded |
| eph_m, epv_m | m | horizontal/vertical uncertainty if available |
| valid_flags | JSON/text | source validity flags |

초기 구현에서는 `VehicleOdometry`를 기록하지 않아도 된다. 기록하는 경우 pose/velocity frame enum을 반드시 저장한다.

## 9. `triangulation_debug.csv`

| 필드 | 단위 | 설명 |
| --- | --- | --- |
| run_id, frame_index | text/index | join key |
| method | text | `two_ray_closest_point` |
| output_frame | text | `baseline_center_frd` |
| baseline_m | m | optical center distance |
| left_depth_m, right_depth_m | m | positive depth check |
| ray_gap_m | m | closest-point distance |
| intersection_angle_deg | deg | ray angle |
| reprojection_error_left_px, reprojection_error_right_px | px | reprojection debug |
| disparity_px | px | if bbox/disparity mode |
| fov_valid_left, fov_valid_right | bool | FOV check |
| clip_valid_left, clip_valid_right | bool | near/far check |
| invalid_reason | text | no disparity, parallel rays, negative depth 등 |

## 10. `optimization_history.csv`

보정 profile 학습/탐색이 있는 경우에만 기록한다.

| 필드 | 단위 | 설명 |
| --- | --- | --- |
| run_id | text | run id |
| profile_id | text | profile id |
| iteration | index | optimizer iteration |
| parameter_name | text | bias/rotation/etc |
| parameter_value | profile unit | value |
| loss_name | text | objective name |
| loss_value | numeric | objective value |
| train_split | text | train/validation/test/sweep |
| validation_metric | text | metric name |
| validation_value | numeric | metric value |

## 11. Invalid frame 처리

1. 숫자 필드는 가능하면 NaN으로 둔다.
2. CSV writer가 NaN을 빈 문자열로 바꿀 경우 별도 `*_valid=false`와 `*_invalid_reason`은 반드시 남긴다.
3. Raw invalid여도 Gazebo oracle과 PX4 snapshots는 기록한다.
4. Corrected invalid는 Raw validity와 독립적으로 기록한다.
5. 동일 frame에서 좌/우 observation 중 하나가 invalid면 triangulation result는 invalid다.

## 12. Timestamp 정책

필드명 없이 timestamp source를 섞지 않는다.

- `sim_time_s`: Gazebo/ROS simulation clock. Simulation 분석의 기본 정렬 시간.
- `ros_time_s`: ROS node clock read time.
- `wall_time_utc`: 사람이 파일을 추적하기 위한 wall-clock.
- `px4_timestamp_us`: PX4 message timestamp, system-start 기준 microseconds.
- `px4_timestamp_sample_us`: PX4 raw sample timestamp.
- `image_stamp_s`: `sensor_msgs/Image.header.stamp`가 있을 때 사용.
- `time_sec`: monotonic elapsed를 의미한다면 `elapsed_monotonic_s`로 이름을 바꾼다.

현재 `operator_recorder.py`와 `calibration_csv_recorder.py`는 `time_sec`에 `time.monotonic()` 기반 elapsed를, `ros_time_sec`에 ROS clock을 기록한다. 향후 Simulation schema에서는 `elapsed_monotonic_s`와 `ros_time_s`로 명확히 분리한다.
