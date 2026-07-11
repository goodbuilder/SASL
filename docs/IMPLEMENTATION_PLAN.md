# Simulation Implementation Plan

분석일: 2026-07-11  
상태: Phase 0 계획 문서. 이번 단계에서는 기능 코드를 수정하지 않았다.

## Phase 0 — 저장소 분석 및 요구사항 문서화

### 목표

- 현재 저장소 구조와 실행 경로를 파악한다.
- 좌표계, 삼각측량, Gazebo/PX4, 로그/UI, 설정의 현재 상태와 gap을 문서화한다.
- 후속 Codex 작업이 참고할 고정 요구사항 문서를 만든다.

### 수정 예상 파일

- 현재 단계에서는 문서만 생성/수정.

### 새로 만든 파일

- `docs/SIMULATION_REQUIREMENTS.md`
- `docs/COORDINATE_CONVENTIONS.md`
- `docs/LOG_SCHEMA.md`
- `docs/CALIBRATION_PROFILE.md`
- `docs/IMPLEMENTATION_PLAN.md`
- `docs/DECISIONS.md`

### 재사용할 기존 코드

- 분석만 수행. 코드 재사용 후보는 아래 Phase 1 이후에 반영한다.

### 테스트

- 문서 생성 확인.
- `git status --short`로 기능 코드가 이 단계에서 추가로 수정되지 않았는지 확인.

### 완료 조건

- 요구사항 문서 6개 존재.
- 현재 구조, 실행 흐름, gap, 위험, Phase 1 수정 후보가 문서와 최종 보고에 포함.

### 의존 관계

- 없음.

### 위험 요소

- 현재 working tree에 이전 작업/사용자 작업의 dirty file이 많으므로 Phase 0 산출물과 기존 변경을 구분해야 한다.

## Phase 1 — 좌표변환 및 삼각측량 수학 코어

### 목표

- `baseline_center_frd`를 Simulation의 고정 output frame으로 구현한다.
- Gazebo ENU, PX4 NED, body FRD, camera RDF, baseline-center FRD 변환을 명명된 함수로 고정한다.
- 평행 stereo disparity식이 아닌 일반화된 두 ray closest-point triangulation을 제공한다.

### 수정 예상 파일

- `Simulation/stereo_ground_sim/stereo_ground_sim/stereo_distance_node.py`
  - 기존 `_position_from_centers()` 직접 사용을 새 math core 호출로 점진 대체.
  - 단, UI/legacy topic compatibility adapter는 유지.
- `Simulation/stereo_ground_sim/stereo_ground_sim/runtime_deployment_selector.py`
  - camera/output frame contract 명칭을 `baseline_center_frd`와 호환되게 adapter 추가.
- `Simulation/stereo_ground_sim/setup.py`
  - 새 module 포함 여부 확인.

### 새로 만들 예상 파일

- `Simulation/stereo_ground_sim/stereo_ground_sim/coordinate_frames.py`
- `Simulation/stereo_ground_sim/stereo_ground_sim/triangulation_core.py`
- `Simulation/tests/test_coordinate_frames.py`
- `Simulation/tests/test_triangulation_core.py`

### 재사용할 기존 코드

| 파일 | 함수/클래스 | 재사용 방향 |
| --- | --- | --- |
| `StereoDistance/wiris_stereo_distance/triangulation.py` | `normalized_ray_from_pixel`, `triangulate_rays`, quaternion helpers | distortion-aware ray 생성과 closest ray 계산 참고. output frame 계약을 Simulation에 맞게 감싼다. |
| `StereoDistance/wiris_stereo_distance/calibration.py` | `CameraIntrinsics`, `StereoCalibrationExtrinsics` | intrinsic/extrinsic profile schema 참고 |
| `sasl_common/truth_coordinates.py` | `local_ned_from_xyz`, `local_ned_from_lla` | PX4 NED/LLA comparison snapshot 변환 |
| `Simulation/scripts/convert_truth_source_to_stereo_rdf.py` | explicit transform validation | direct axis alias 금지 패턴 재사용 |
| `Simulation/stereo_ground_sim/stereo_ground_sim/runtime_deployment_selector.py` | `RuntimeStereoCameraContract`, `RuntimeImageObservation` | 기존 public FRD/RDF adapter와 테스트 참고 |

### 테스트

- ENU -> baseline_center_frd position transform 단위 테스트.
- PX4 NED -> baseline_center_frd는 direct alias가 아니라 explicit transform 없이는 실패하는 테스트.
- Camera RDF ray -> baseline_center_frd ray origin/direction 테스트.
- 두 ray closest-point 삼각측량: ideal intersection, skew rays, parallel rays, negative depth, ray gap threshold.
- quaternion order test: PX4 `(w,x,y,z)`와 SciPy boundary adapter.

### 완료 조건

- 모든 frame transform 함수가 input frame, output frame, unit, axis order를 docstring/test로 갖는다.
- Raw/Corrected가 같은 `baseline_center_frd` type/struct를 사용한다.
- 평행 camera 특별식과 일반 ray 방식이 ideal parallel case에서 수치적으로 일치한다.

### 의존 관계

- Phase 0 문서.

### 위험 요소

- 기존 code와 tests는 `stereo_center_opencv_optical` RDF와 public FRD를 많이 사용한다.
- `StereoDistance`의 `FINAL_ESTIMATE_FRAME`은 `px4_local_ned`라 Simulation 요구와 다르므로 그대로 가져오면 안 된다.

## Phase 2 — Gazebo Ground Truth 및 ideal projection

### 목표

- Gazebo oracle source를 `target_reference_link` world pose로 정의한다.
- `target_reference_link`가 없으면 명시적으로 `base_link`/model pose substitute를 사용하고 metadata에 기록한다.
- ideal projection mode를 구현하여 detection/YOLO 없이 좌우 camera observation을 생성한다.

### 수정 예상 파일

- `Simulation/stereo_ground_sim/worlds/stereo_ground_drone.sdf`
- `Simulation/stereo_ground_sim/models/x500_15m/model.sdf`
- `Simulation/stereo_ground_sim/launch/stereo_ground_sim.launch.py`
- `Simulation/stereo_ground_sim/stereo_ground_sim/stereo_distance_node.py`

### 새로 만들 예상 파일

- `Simulation/stereo_ground_sim/stereo_ground_sim/gazebo_oracle.py`
- `Simulation/stereo_ground_sim/stereo_ground_sim/ideal_projection.py`
- `Simulation/tests/test_gazebo_oracle_contract.py`
- `Simulation/tests/test_ideal_projection.py`

### 재사용할 기존 코드

- `stereo_distance_node.py`의 `/world/stereo_ground_drone/pose/info` reader와 `_find_pose_by_name()`.
- Launch의 SDF generation path.
- SDF camera topics and bridge setup.

### 테스트

- SDF에 `target_reference_link` 존재 여부 검사.
- Gazebo ENU world point를 baseline-center FRD로 변환하는 fixture test.
- Ideal projection round trip: known point -> left/right pixel -> triangulation -> same point.
- Camera update rate 기본값 20 Hz 확인.

### 완료 조건

- Ground truth link name과 source frame이 run metadata/log에 남는다.
- ideal projection mode와 bbox mode가 config에서 분리된다.
- 1280 x 720, 20 Hz 기본값이 launch/config/docs에서 일치한다.

### 의존 관계

- Phase 1 math core.

### 위험 요소

- 현재 SDF camera sensor frame에는 optical frame 이름이 없다.
- 현재 Gazebo pose reader는 model pose 탐색이며 link pose 선택 기능이 없다.

## Phase 3 — 자세 오차 주입 및 Raw 결과

### 목표

- 좌/우 camera roll/pitch/yaw 오차를 명시적으로 주입한다.
- 공통 자세 오차와 상대 자세 오차를 분리 기록한다.
- 주입된 실제 camera pose를 사용해 Raw triangulation을 계산한다.

### 수정 예상 파일

- `Simulation/config/camera_attitude.env`
- `Simulation/config/attitude_error.env`
- `Simulation/run_attitude_error_experiment.sh`
- `Simulation/scripts/attitude_error_experiment.py`
- `Simulation/stereo_ground_sim/launch/stereo_ground_sim.launch.py`
- `Simulation/stereo_ground_sim/stereo_ground_sim/stereo_distance_node.py`

### 새로 만들 예상 파일

- `Simulation/stereo_ground_sim/stereo_ground_sim/attitude_error_model.py`
- `Simulation/tests/test_attitude_error_model.py`

### 재사용할 기존 코드

- Launch의 `left_roll_error_deg`, `left_pitch_error_deg`, `left_yaw_error_deg`, `right_*` argument path.
- `Simulation/scripts/attitude_error_experiment.py`의 sweep case builder.
- `StereoDistance/wiris_stereo_distance/attitude_bias.py`의 quaternion convention 참고.

### 테스트

- deg input -> rad/quaternion conversion suffix/contract test.
- common error와 relative error decomposition test.
- zero-error Raw equals ideal projection baseline.
- one-axis yaw/pitch/roll perturbation sign regression tests.

### 완료 조건

- 자세 오차 설정과 로그 field가 모두 `_deg` suffix를 가진다.
- 내부 rotation은 quaternion/matrix로 적용된다.
- Raw output frame은 항상 `baseline_center_frd`다.

### 의존 관계

- Phase 1, Phase 2.

### 위험 요소

- Gazebo SDF roll/pitch/yaw convention과 camera optical RDF convention의 적용 순서가 혼동될 수 있다.
- 현재 launch는 model pose 문자열 치환 방식이라 확장 시 brittle하다.

## Phase 4 — 보정 프로파일 및 Corrected 결과

### 목표

- 자세-bias correction profile schema를 구현한다.
- Raw와 같은 observation에서 Corrected 결과를 계산한다.
- profile validation envelope와 runtime applicability를 기록한다.

### 수정 예상 파일

- `Simulation/stereo_ground_sim/stereo_ground_sim/stereo_distance_node.py`
- `Simulation/stereo_ground_sim/stereo_ground_sim/runtime_deployment_selector.py`
- `Simulation/stereo_ground_sim/stereo_ground_sim/runtime_guard_policy.py`
- `Simulation/stereo_ground_sim/stereo_ground_sim/reprojection_profile.py`

### 새로 만들 예상 파일

- `Simulation/stereo_ground_sim/stereo_ground_sim/calibration_profile.py`
- `Simulation/tests/test_calibration_profile_contract.py`
- `Simulation/config/calibration_profiles/*.json` 또는 `Simulation/results/...` profile artifact 위치

### 재사용할 기존 코드

- 기존 bbox/reprojection/runtime profile loading pattern.
- `StereoDistance/wiris_stereo_distance/calibration.py` intrinsics/extrinsics dataclasses.
- `Simulation/docs/experiment-gates.md`의 profile 검증 gate 개념.

### 테스트

- profile schema version validation.
- wrong resolution/zoom rejects or no-ops.
- rotation direction test: known bias correction recovers point.
- corrected frame equals raw frame contract test.

### 완료 조건

- Corrected topic/log에는 profile id, validity, fallback/no-op reason이 기록된다.
- profile이 없거나 envelope 밖이면 corrected invalid/no-op 정책이 명확하다.

### 의존 관계

- Phase 1-3.

### 위험 요소

- 기존 bbox correction profile과 자세 correction profile을 섞으면 원인/효과 해석이 불명확해진다.
- Simulation profile을 실제 StereoDistance로 전이할 때 domain gap이 크다.

## Phase 5 — sweep, 로그, 통계, 데이터셋 분리

### 목표

- attitude error sweep을 재현 가능한 dataset으로 저장한다.
- Gazebo oracle, PX4 comparison, Raw, Corrected, triangulation debug를 분리 schema로 기록한다.
- 거리·방위·고도별 통계를 생성한다.

### 수정 예상 파일

- `Simulation/scripts/attitude_error_experiment.py`
- `Simulation/stereo_ground_sim/stereo_ground_sim/operator_recorder.py`
- `Simulation/stereo_ground_sim/stereo_ground_sim/calibration_csv_recorder.py`
- `Simulation/run_attitude_error_experiment.sh`

### 새로 만들 예상 파일

- `Simulation/stereo_ground_sim/stereo_ground_sim/simulation_logger.py`
- `Simulation/scripts/summarize_simulation_sweep.py`
- `Simulation/tests/test_simulation_log_schema.py`
- `Simulation/tests/test_summarize_simulation_sweep.py`

### 재사용할 기존 코드

- `operator_recorder.py` mode/manifest/bag recording pattern.
- `calibration_csv_recorder.py` PX4 local/global/gps subscription pattern.
- `attitude_error_experiment.py` summary writer.

### 테스트

- schema columns exact test.
- invalid row retention test.
- PX4 comparison role never equals ground_truth test.
- summary bin aggregation test.

### 완료 조건

- `run_metadata.json`, `frame_measurements.csv`, `camera_observations.csv`, `px4_states.csv`, `triangulation_debug.csv`, `summary.json` 생성.
- 모든 CSV에 `run_id`, `frame_index`, `output_frame`, timestamp policy가 반영된다.

### 의존 관계

- Phase 1-4.

### 위험 요소

- 현재 `Float32MultiArray` topic에는 header/time/frame_id가 없다.
- 기존 recorder는 NaN을 빈 문자열로 쓰므로 valid/status field가 누락되면 분석 손실이 생긴다.

## Phase 6 — 실시간 UI

### 목표

- Gazebo oracle, Raw, Corrected, error, validity, PX4 source freshness를 실시간으로 표시한다.
- UI는 계산/보정 로직과 분리한다.

### 수정 예상 파일

- `Simulation/stereo_ground_sim/stereo_ground_sim/operator_panel.py`
- `Simulation/stereo_ground_sim/stereo_ground_sim/recorder_panel.py`
- `Simulation/stereo_ground_sim_overlay/src/detection_overlay_node.cpp`
- `Simulation/run_operator_view.sh`

### 새로 만들 예상 파일

- 필요 시 `Simulation/stereo_ground_sim/stereo_ground_sim/ui_models.py`
- UI smoke tests 또는 schema contract tests

### 재사용할 기존 코드

- `operator_panel.py`의 image preview + FRD plots.
- `recorder_panel.py`의 recorder controls and status labels.
- `detection_overlay_node.cpp`의 lightweight overlay publisher.

### 테스트

- UI가 expected topics를 subscribe하는지 static test.
- label에 `baseline_center_frd` contract 표시 test.
- stale data status test.
- headless mode가 UI 없이 logger를 유지하는 smoke test.

### 완료 조건

- UI는 `baseline_center_frd`를 명시한다.
- PX4 data는 comparison source로 표시하고 oracle로 표시하지 않는다.
- 계산 로직은 UI code에 들어가지 않는다.

### 의존 관계

- Phase 5 schema/topic 확정.

### 위험 요소

- 현재 UI는 `Float32MultiArray` index contract에 의존한다.
- UI와 recorder가 같은 topic 명칭을 공유하므로 schema 변경 시 compatibility adapter가 필요하다.

## Requirements 대비 차이 요약

| 요구사항 | 현재 구현 상태 | 관련 파일 | 문제점 | 권장 조치 | Phase |
| --- | --- | --- | --- | --- | --- |
| 최종 output `baseline_center_frd` | public FRD와 internal RDF가 혼재 | `stereo_distance_node.py`, `runtime_deployment_selector.py`, `operator_panel.py` | frame 이름이 `stereo_center_*` 중심 | output frame struct/adapter 추가 | 1 |
| Gazebo world ENU oracle | world는 ENU, 변환은 rig-local 직접 매핑 | `stereo_ground_drone.sdf`, `stereo_distance_node.py` | ENU와 `x=right,y=forward` 해석 혼동 | 명시 transform 분리 | 1-2 |
| `target_reference_link` | 없음 | `models/x500_15m/model.sdf` | model/base pose만 가능 | link 추가 또는 substitute metadata | 2 |
| 20 Hz camera | 현재 60 Hz/30 Hz/20 Hz 혼재 | SDF, launch, `camera_model.env` | 기본 조건 불일치 | 20 Hz 통일 | 2 |
| General two-ray triangulation | 기존 main path는 disparity 공식 | `stereo_distance_node.py::_position_from_centers()` | 자세 오차가 큰 일반 pose에 부족 | core triangulation 추가 | 1 |
| Ray gap/intersection angle/reprojection error | 일부 gap만 기존 StereoDistance에 있음 | `StereoDistance/.../triangulation.py` | Simulation log에는 없음 | debug schema/core 추가 | 1,5 |
| PX4 Local/Global/GPS 비교 기록 | CSV recorder가 local/global/gps 지원 | `calibration_csv_recorder.py` | Gazebo oracle과 unified schema 아님 | `px4_states.csv` 분리 | 5 |
| Odometry 선택 | px4_msgs 정의만 확인, recorder 미구현 | `VehicleOdometry.msg` | frame enum 처리 필요 | 초기 제외, optional adapter later | 5+ |
| Raw/Corrected 같은 frame | 현재 corrected는 bbox/profile 후 public FRD | `stereo_distance_node.py` | frame명 고정 불충분 | same output type 강제 | 1,4 |
| 설정 단일화 | env/launch/script에 분산 | `config/*.env`, launch, run scripts | FPS/height/range 등 불일치 가능 | scenario schema 정리 | 2,5 |
