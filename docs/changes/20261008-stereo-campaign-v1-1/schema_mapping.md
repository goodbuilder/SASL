# v1.1 실제 토픽 → 저장 필드 매핑

2026-10-09 실행 코드 기준. 좌표/시각 계약은 producer의 필드와 함께 기록한다.

| 원본 토픽 또는 생산자 | 저장 파일·필드 | 좌표·시각 계약 |
| --- | --- | --- |
| `/left_camera/image`, `/right_camera/image` (`sensor_msgs/Image`) | `raw_images_or_lossless_bag/{left,right}/` rosbag2와 `image_index.jsonl`; 각 눈의 `observations_*.jsonl` | `Image.header.stamp`가 Gazebo 촬영 시각 ns. bag 수신 시각, YOLO 종료 UTC ns와 별도. `simulation_epoch` 보존. 원본 1280×720 이미지. |
| `/left_camera/camera_info`, `/right_camera/camera_info` | 같은 눈의 rosbag2; 카메라 K는 `effective_config.json`/물리 자세 기록과 대조 | 영상 픽셀은 OpenCV optical RDF: right, down, forward. |
| `/simulation/ground_truth/vehicle_reference_link` (`PoseStamped`) | `ground_truth.jsonl`의 `position_world_enu_m`, `orientation_world_from_reference_xyzw` | Gazebo world ENU `[east,north,up]` m; orientation은 world←reference_link, quaternion XYZW. 참값이며 추정기 런타임 입력 금지. |
| `/clock` | `clock.jsonl`의 `simulation_timestamp_ns`, `simulation_epoch` | Gazebo simulation clock; 역행 시 epoch 증가. |
| `/fmu/out/vehicle_local_position_v1` | `px4_local_position.jsonl`의 `position_local_ned_m`, `velocity_local_ned_mps`, `heading_clockwise_from_north_rad`; 제어기 `flight_state.jsonl` | PX4 local NED `[north,east,down]`; 원점은 실행 중 GT/PX4 표본 20개로 관측하며 Gazebo ENU world 원점과 같다고 가정하지 않음. |
| `/fmu/out/vehicle_attitude` | `px4_attitude.jsonl`의 `quaternion_ned_from_body_wxyz` | 부모 PX4 local NED, 자식 body FRD; PX4 메시지의 WXYZ 순서. |
| `/simulation/camera_pose_diagnostics/{left,right}` | `camera_pose_diagnostics.jsonl`, 마지막 읽기값 `camera_truth.json` | Gazebo world ENU 광심과 optical RDF 축의 물리 자세. `R_WC`는 optical→world. 카메라 RTK/IMU 모의 관측과 구분. |
| `flight_controller.py` | `flight_reference.json`, `flight_state.jsonl`, `events.jsonl`, `controller_result.json` | 참조 world ENU `[east,north,up]` m; 변환한 명령은 PX4 local NED `[north,east,down]`. body heading은 북쪽 기준 180°. |
| `bag_index.py` | `image_index.jsonl`, `pair_index.jsonl`, `image_index_summary.json` | 원본 source frame ID는 눈별로 단조 증가. 명목 pair는 같은 epoch의 촬영시각 차이 ≤1 ms, 프레임 재사용 없음. 비매칭 프레임 유지. |
| `postprocess_detections.py` | `observations_left.jsonl`, `observations_right.jsonl`, `paired_observations.jsonl`, `observation_summary.json` | YOLO 결과를 원본 1280×720 `bbox_xyxy_px` 및 `bbox_center_uv_px`로 역변환. 미검출은 null bbox. 검출기는 고정 모델 해시/임계값 사용. |
| `align_gt.py` | `파생_재처리/<run>/<attempt>/gt-alignment/gt_at_images.jsonl`, 실행 루트의 `gt_alignment_summary.json` | 같은 epoch GT의 world ENU 위치 선형 보간, XYZW shortest-path SLERP; ≥40 ms 간격/외삽 플래그. V0 성능 수치는 동결 전 비공개. |

`x/y/z`라는 이름은 위 프레임 표기가 없는 값의 별칭으로 사용하지 않는다. 카메라 광심 좌표와 PX4 local position, 드론 GT는 서로 다른 계약이다. 기록된 원본은 수정하지 않고 재처리 결과만 `파생_재처리`에 저장한다.
