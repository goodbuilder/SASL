# Simulation Decisions

분석일: 2026-07-11  
상태: Phase 0에서 확정한 단순화 가정과 설계 결정.

## D-001. Simulation과 StereoDistance를 분리한다

- `StereoDistance`는 실제 WIRIS/Pixhawk/RTSP/RTK 운용 앱이다.
- `Simulation`은 Gazebo/PX4 SITL 기반 연구 앱이다.
- Simulation 구현을 위해 실제 장비 설정 파일을 직접 변경하지 않는다.

근거: 실제 장비 앱은 `StereoDistance/`, 연구용 시뮬레이션은 `Simulation/`에 별도 구조가 있다.

## D-002. Simulation Ground Truth는 Gazebo pose다

- 절대 ground truth는 Gazebo world에서 드론 모델의 `target_reference_link` world pose다.
- 현재 `target_reference_link`는 `Simulation/stereo_ground_sim/models/x500_15m/model.sdf`에 없다.
- Phase 2 전까지는 `x500_15m` model pose 또는 `base_link` substitute만 가능하며, substitute 사실을 metadata에 기록한다.

## D-003. PX4 값은 비교값이다

- `VehicleLocalPosition`, `VehicleGlobalPosition`, `vehicle_gps_position`/`SensorGps`는 Gazebo oracle과 비교하기 위한 값이다.
- EKF에서 파생된 Local/Global/Odometry는 독립 ground truth로 취급하지 않는다.

## D-004. Odometry는 선택적 보조 비교값이다

- 초기 구현 필수 범위에서는 `VehicleOdometry`를 쓰지 않는다.
- 나중에 기록할 경우 `pose_frame`, `velocity_frame` enum과 quaternion order를 반드시 함께 저장한다.

## D-005. 카메라-Pixhawk lever arm은 무시한다

- 현재 연구 범위는 스테레오 카메라 자세 오차이다.
- 카메라 rig 원점과 PX4 body origin offset은 모델링하지 않는다.

## D-006. 카메라-Pixhawk 방향 불일치는 무시한다

- 카메라 optical frame과 Pixhawk body frame의 실제 장착 방향 차이는 현재 제외한다.
- Simulation의 camera pose는 Gazebo/model transform으로 직접 정의한다.

## D-007. 시간 offset은 무시한다

- 좌우 영상 시간 offset, RTSP 지연, 실제 STUS 동기화 문제는 현재 제외한다.
- 단 로그에는 timestamp source를 분리해서 나중에 분석 가능하게 한다.

## D-008. 대기 영향은 무시한다

- 대기 굴절, heat haze, rolling shutter, H.264 artifact, 실제 구조물 진동은 현재 제외한다.

## D-009. 기본 영상 조건은 1280 x 720, 20 Hz다

- 향후 구현 기본값은 1280 x 720, 20 Hz로 통일한다.
- 현재 저장소에는 60 Hz(SDF/launch), 30 Hz(`camera_model.env`), 20 Hz(overlay/mission setpoint)가 혼재하므로 Phase 2에서 정리한다.

## D-010. 줌은 한 run 동안 고정하거나 줌별 profile을 사용한다

- profile에는 `zoom_id`, `horizontal_fov_rad`, intrinsic compatibility가 포함되어야 한다.
- `Simulation/config/camera_model.env`의 WIRIS zoom/HFOV 근사 table은 profile lookup의 참고 자료다.

## D-011. Ideal correspondence와 BBox mode를 분리한다

- ideal projection은 Simulation 수학/좌표 검증용이다.
- bbox/YOLO mode는 detector 오차와 runtime behavior를 포함하는 별도 mode다.
- 두 mode의 observation source를 log field로 구분한다.

## D-012. 자세 오차를 현재 주요 연구 대상으로 제한한다

- roll/pitch/yaw 주입, 공통/상대 자세 오차, raw/corrected 차이를 중심으로 실험한다.
- 전체 센서 오차 예산이나 실제 운용 오차는 후속 범위다.

## D-013. Raw와 Corrected는 같은 output frame을 사용한다

- output frame은 항상 `baseline_center_frd`다.
- 보정 profile이 바뀌어도 output frame 이름과 축은 바뀌지 않는다.

## D-014. Euler angle은 설정/표시용이다

- 내부 회전 표현은 quaternion 또는 rotation matrix를 사용한다.
- degree/radian 단위는 필드명 suffix로 구분한다.

## D-015. 기능 코드 수정은 Phase 0 범위 밖이다

- Phase 0은 분석과 문서 작성만 한다.
- 명백한 코드 문제도 수정하지 않고 위치와 위험을 기록한다.

## D-016. PX4 SITL만 Simulation 비행 권한을 가진다

- Simulation의 arm, takeoff, 이동, 자세, approach와 landing 명령은 PX4 SITL이 독점한다.
- `fly_target_in_gazebo`는 항상 `false`, `mission_profile`은 항상 `manual`, `actual_distance_source`는 항상 `gazebo`다.
- Gazebo `target_reference_link` pose는 Ground Truth oracle로만 읽으며 비행 제어 source가 아니다.
- PX4 Local NED는 보조 비교값이며 Ground Truth로 승격하지 않는다.
- 기존 Gazebo-driven attitude-error sweep은 PX4-native 대체 runner가 구현될 때까지 비활성화한다.

근거: 동적 Gazebo 모델에 scripted pose를 주기적으로 적용하면 중력/충돌 물리와 pose reset이 충돌하여 비행 권한과 Ground Truth source가 혼합된다.
