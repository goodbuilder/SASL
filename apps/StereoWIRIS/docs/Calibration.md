# Calibration Reference Contract

이 문서는 StereoWIRIS와 Simulation에서 보정법을 설계, 탐색, 평가할 때
따르는 기준 계약이다. 목적은 `nominal` 측정값의 오차를 독립 기준으로
측정해 보정 프로파일을 만들고, 별도 비행에서 `corrected nominal`이
참값에 가까워졌는지 검증하는 것이다.

## 용어

- `nominal`: 현재 카메라, 자세, baseline, triangulation 계약만으로 만든
  보정 전 3D 추정값.
- `correction_profile`: 보정 비행에서 학습한 오차 보정 규칙. 런타임 입력은
  nominal 측정값과 카메라/기체 상태처럼 실시간 운용에서 실제로 사용할 수
  있는 값으로 제한한다.
- `corrected_nominal`: `nominal`에 `correction_profile`을 적용한 3D 추정값.
- `gazebo_truth_pose`: 시뮬레이션 검증에서 사용하는 ground-truth pose.
- `px4_gps_reference`: PX4 GPS/RTK/EKF 로그에서 얻은 위치 기준. 실제 비행
  확장성을 검토하기 위한 reference이며, 시뮬레이션의 최상위 truth는 아니다.
- `ideal`: 보고서에서 쓸 수 있는 개념적 표현일 뿐, 저장 필드나 검증
  기준명으로는 `gazebo_truth_pose` 또는 `ground_truth`를 사용한다.

## Truth Source Priority

시뮬레이션에서는 `gazebo_truth_pose`를 비교용 참값으로 사용한다. Gazebo
pose는 센서 모델과 PX4 추정기를 거치기 전의 world state이므로 보정법의
독립 검증 기준으로 가장 명확하다.

PX4 GPS/RTK 정보는 함께 기록하되, 다음 용도로 분리한다.

- GPS/RTK 기준 보정법을 실제 비행으로 확장할 때 필요한 reference 후보.
- PX4/EKF/GPS 자체 오차와 지연을 Gazebo truth 대비 평가하는 보조 지표.
- 1회차 보정 프로파일을 만들 때 사용할 수 있는 현실형 기준값 후보.

PX4 GPS/RTK를 truth로 부를 때는 반드시 어떤 데이터인지 명시한다.

- `vehicle_global_position`
- raw GPS fix
- RTK fixed/float position
- EKF local position
- GPS에서 변환한 local NED 또는 ENU position

## Two-Flight Calibration Flow

### 1회차: 보정 프로파일 생성

1회차 비행은 오차장을 관측하기 위한 calibration flight다.

기록해야 할 값:

- `nominal`
- `gazebo_truth_pose`
- `px4_gps_reference`
- 카메라 FOV 내 위치, depth/range, bbox, disparity, 자세, yaw/pitch/roll,
  baseline, timestamp, sample age/skew

학습 대상:

```text
error = truth_position - nominal_position
```

시뮬레이션 실험에서는 기본 truth를 `gazebo_truth_pose`로 둔다. 실제 비행
확장을 연구할 때만 `px4_gps_reference` 기반 프로파일을 별도 method name으로
분리한다.

### 2회차: 독립 검증

2회차 비행은 validation flight다. 카메라 3D 화각 안에서 랜덤 waypoint를
생성해 1회차와 다른 샘플 분포를 만든다.

비교해야 할 값:

```text
nominal vs gazebo_truth_pose
corrected_nominal vs gazebo_truth_pose
```

성공 조건:

```text
baseline_error = distance(nominal, gazebo_truth_pose)
corrected_error = distance(corrected_nominal, gazebo_truth_pose)

pass if:
  median(corrected_error) < median(baseline_error)
  p90(corrected_error) < p90(baseline_error)
  improvement holds across depth/range/FOV bins
  no major regression appears inside the validated operating envelope
```

`corrected_nominal` 계산 중에는 2회차의 `gazebo_truth_pose`, GPS truth, 또는
ideal 값을 입력으로 사용하면 안 된다. 이 값들은 검증용으로만 사용한다.

## CSV-Based Profile Delivery and Runtime Eligibility

CSV는 후보 보정법을 fitting하고 offline replay로 비교하기에 충분한 입력이지만,
CSV에서 좋은 결과를 낸 모든 후보가 runtime profile로 배포 가능한 것은 아니다.
결과물은 반드시 아래 두 범주로 분리한다.

- `runtime-deployable profile`: 현재 시점에 StereoDistance가 실제로 받을 수
  있는 입력만으로 `corrected_nominal`을 계산하는 JSON profile.
- `offline-only analysis`: Gazebo truth, UAM/PX4 GPS reference, validation error
  또는 전체 CSV를 사용해 후보를 평가한 report. 이 값들은 runtime 입력이 아니다.

### Runtime-deployable profile contract

배포 가능한 profile은 다음을 만족해야 한다.

- 입력은 현재 pair의 nominal stereo 값과, 필요한 경우 현재 pair의 disparity,
  bbox center, 카메라/기체 상태처럼 실시간으로 존재하는 값이어야 한다.
- 출력은 public `baseline_center_frd`의 `[Forward, Right, Down]` metres여야
  하며, profile JSON에 입력·출력 frame, axis order, units, 학습 범위를 기록한다.
- 고정 계수 또는 현재 시점과 과거 sample만 쓰는 causal state만 사용한다.
- StereoDistance runtime loader가 해석할 수 있는 profile schema여야 하며,
  profile 적용 smoke test를 통과해야 한다.

다음 후보는 CSV replay 보고서로는 남길 수 있어도 runtime-deployable profile로
내보내면 안 된다.

- `gazebo_truth_pose`, GT/ideal field, validation error를 입력으로 사용하는 방법.
- 2회차 전체 통계, 미래 sample, 사후 최적 구간을 사용하는 non-causal 방법.
- CSV에만 있는 사후 계산 열 또는 runtime에 보장되지 않는 결측 가능 열에 의존하는
  방법.
- 학습 중인 2회차 데이터로 계수나 profile selector를 다시 fitting하는 방법.
- 검증된 operating envelope 밖에서만 정의된 무제한 외삽 방법.

`uam_raw_gps_reference` 또는 PX4 GPS/RTK를 runtime 입력으로 쓰는 방법은
GPS-assisted correction으로 별도 이름을 붙인다. 해당 GPS stream이 runtime에
안정적으로 존재할 때만 적용 가능하며, nominal-only profile 또는 GPS 없는
운용의 정확도 개선 근거로 혼동해서는 안 된다.

### Two-flight CSV handling

1회차 waypoint CSV는 fitting에만 사용한다. Nominal과 UAM GPS reference를
맞춰 fit할 수 있으나, 그 결과는 우선 `GPS-reference-calibrated` method로
표시한다. Gazebo truth가 함께 있으면 1회차에서는 방법 비교와 학습 품질
진단에 사용하되 runtime 입력에는 포함하지 않는다.

2회차 자유비행 CSV에는 고정된 profile을 적용만 한다. 다음 네 비교를 같은
timestamp/pair와 같은 frame에서 보고한다.

```text
nominal            vs uam_gps_reference
corrected_nominal  vs uam_gps_reference
nominal            vs gazebo_truth_pose
corrected_nominal  vs gazebo_truth_pose
```

GPS-reference 오차만 감소하고 Gazebo truth 오차가 증가하면 GPS reference의
bias/latency를 따라간 과적합 가능성으로 기록하며 PASS로 승격하지 않는다.
반대로 2회차 Gazebo truth 기준에서 corrected error가 nominal보다 감소해야
시뮬레이션 정확도 개선을 주장할 수 있다. 각 비교는 range/FOV/attitude bin과
Forward/Right/Down 축별 RMSE, median, p90을 함께 남긴다.

## Waypoint Generation Contract

Calibration과 validation waypoint는 생성 단계에서 카메라 3D FOV 안에
들어오도록 제한한다. 단순히 world/NED 좌표에서 random waypoint를 만든 뒤
비행 중 우연히 보이기를 기대하지 않는다.

기본 생성 순서:

```text
1. Gazebo/world 또는 local NED에서 후보 waypoint 생성
2. 후보 waypoint를 camera optical RDF 또는 baseline_center_frd로 변환
3. FOV, range, margin, stereo matching 가능 조건 검사
4. 통과한 후보만 calibration/validation waypoint로 사용
```

`baseline_center_frd` 기준 FOV 판정 예:

```text
Forward > min_range
Forward < max_range

abs(atan2(Right, Forward)) < horizontal_fov / 2 - margin
abs(atan2(Down, Forward)) < vertical_fov / 2 - margin
```

Stereo 보정/검증 waypoint는 좌우 카메라 모두에서 관측 가능해야 한다.

```text
valid_waypoint if:
  inside_left_camera_fov
  inside_right_camera_fov
  inside_range_limits
  expected_bbox_not_clipped
  not too close to FOV boundary
```

Calibration waypoint와 validation waypoint는 서로 다른 set이어야 한다.
Validation set은 calibration set과 겹치지 않되, 같은 validated operating
envelope 안에 있어야 한다. 학습 범위 밖, FOV 경계, 과도한 거리, 큰
yaw/pitch/roll 조건은 normal validation이 아니라 별도 stress 또는
extrapolation experiment로 표시한다.

비행 중 waypoint 사이를 이동하다가 잠시 FOV 밖으로 나가는 것은 허용할 수
있다. 그러나 FOV 밖 샘플은 보정/검증 metric에서 제외하고
`out_of_fov` 또는 `invalid_detection` 상태로 분리한다.

## Coordinate Contract

오차 계산 전 모든 위치는 같은 frame, 같은 축 순서, 같은 단위로 변환해야
한다. 서로 다른 frame의 `x/y/z` 값을 직접 빼지 않는다.

기본 계약:

- Gazebo world pose: Simulation world frame. 실험별로 ENU/NED 계약을 명시한다.
- PX4 world/local: PX4 관례상 NED를 기본으로 본다.
- PX4 body: FRD.
- Camera optical: OpenCV/ROS optical RDF.
- Stereo public raw output: `baseline_center_frd`
  `[Forward, Right, Down, Range]`.

허용되는 비교 예:

```text
nominal_baseline_center_frd
  -> local_ned or gazebo_world frame
  -> compare with gazebo_truth_pose in the same frame
```

또는:

```text
gazebo_truth_pose
  -> camera optical RDF or baseline_center_frd
  -> compare with nominal in the same frame
```

금지:

- camera RDF의 `x/y/z`와 PX4 NED의 `north/east/down` 직접 비교.
- center-frame 값을 left/right camera field에 저장.
- Gazebo world frame을 명시하지 않고 `ideal_x/y/z` 같은 이름으로 기록.
- 2회차 truth 값을 보정 입력으로 사용.

## Method Record Requirements

보정 실험을 추가할 때는 README, report, 또는 experiment log에 다음을 남긴다.

- method name
- purpose
- training flight source
- validation flight source
- truth source
- runtime inputs
- output frame and units
- pass/fail criteria
- validated range: depth/range/FOV/yaw/pitch/roll bins
- known regressions and out-of-envelope conditions

런타임 보정법은 ground truth에 의존하면 안 된다. Ground truth를 사용하는
방법은 `offline validation only`로 표시한다.

## Simulation Commands

FOV-constrained calibration/validation waypoint set 생성:

```bash
python3 apps/Simulation/scripts/generate_fov_constrained_waypoints.py \
  --output-dir apps/Simulation/results/calibration_waypoints/run_001 \
  --calibration-count 12 \
  --validation-count 12
```

출력:

- `calibration_waypoints.csv`
- `validation_waypoints.csv`
- `calibration_mission.csv`
- `validation_mission.csv`
- `waypoint_manifest.json`

PX4 waypoint mission runner에는 mission CSV를 전달한다.

```bash
cd Simulation
./run_px4_waypoint_mission.sh --path-file results/calibration_waypoints/run_001/calibration_mission.csv
./run_px4_waypoint_mission.sh --path-file results/calibration_waypoints/run_001/validation_mission.csv
```

두 비행에서 calibration CSV를 각각 기록한 뒤, two-flight profile loop를
실행한다.

```bash
apps/Simulation/run_calibration_profile_loop.sh \
  --calibration-csv apps/Simulation/results/calibration_flight/calibration.csv \
  --validation-csv apps/Simulation/results/calibration_flight/validation.csv \
  --output-dir apps/Simulation/results/calibration_profile_loop/run_001
```

`--truth-transform-json`을 지정하면 calibration/validation CSV의 source-frame
truth를 명시적 `T_source_to_stereo_rdf`로 변환한 뒤 fitting/replay에 사용한다.
이미 `truth_right_m`, `truth_down_m`, `truth_forward_m`이 Gazebo truth에서
기록되어 있으면 transform 없이 실행할 수 있다.

Two-flight loop 산출물:

- `calibration_profile/`: 1회차 CSV에서 만든 runtime profile과 fit report.
- `validation_replay/`: 2회차 CSV에 profile을 적용한 nominal/corrected 비교.
- `two_flight_validation_summary.json`: pass/fail, runtime input contract,
  forbidden truth inputs, validation metrics 요약.
