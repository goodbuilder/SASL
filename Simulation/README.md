# Ground Stereo Drone Simulation

## Observation modes (S2-I1)

In `bbox_yolo`, camera subscription and model loading require a positive
`yolo_target_hz`, a non-empty `yolo_model_path`, and a runtime that has not
been disabled. In `ideal_reference_point`, image stamp/size subscription stays
active without a model or CUDA runtime; no YOLO load or warm-up is attempted.
An ideal-mode queue overflow invalidates the run while preserving the accepted
pair terminal accounting (`processed + discarded == accepted`).

`observation_mode` is immutable for a run and defaults to `bbox_yolo`.
`bbox_yolo` uses image content and YOLO BBox centers; its measurement is
Ground-Truth independent. `ideal_reference_point` is a Simulation-only oracle
mode: the authoritative `vehicle_reference_link` point from Gazebo World ENU
is projected through the actual left/right optical poses to float pixels, then
the same pixels are triangulated with nominal poses (Raw) and injected actual
poses (attitude-corrected). GT is used to create and evaluate the observation,
not as the correction input. Outputs remain `baseline_center_frd` (Forward,
Right, Down, metres). Ideal mode does not measure detector performance and does
not publish fake YOLO detections. Invalid projection is explicitly rejected.

The existing Gazebo target/model domain mismatch for `bbox_yolo` remains a
separate limitation; ideal correspondence is not evidence that YOLO detects
the target.

## Atomic pair-level research recording (S2-R2)

`enable_research_recording` defaults to `false`, so the normal launch creates
no recorder thread or files.  When enabled, the distance node snapshots each
accepted pair exactly once at its terminal state and submits the flat record
to a bounded nonblocking writer queue.  Raw, injected-attitude Corrected,
Gazebo reference-link Ground Truth, four observation pixels, pair/epoch/stamp
identity, attitude inputs, errors, and runtime counters therefore come from
one in-process pair state rather than a latest-value topic join.

The authoritative artifact is UTF-8 `pair_records.jsonl` with schema
`s2_atomic_pair_v1`; deterministic `pair_records.csv` is exported only during
clean finalization.  `run_metadata.json`, `run_summary.json`, SHA-256 hashes,
and `RUN_FINALIZED_VALID` or `RUN_FINALIZED_INVALID` bind provenance and run
validity to the rows.  `RUN_INCOMPLETE` remains after an abnormal termination.
Recorder queue overflow or writer failure invalidates the run and is never a
silent drop.  File I/O, CSV conversion, validation, and hashing execute outside
the measurement condition lock.

`sample_eligible=true` identifies a successfully evaluated, finite,
`baseline_center_frd` pair, but it does not make the dataset valid by itself.
Analysis must use samples only when `run_summary.json` also contains
`run_valid=true`; a later measurement/recorder overrun, reset, drain failure,
accounting mismatch, duplicate pair key, or artifact failure invalidates the
whole run.  This recorder is qualification infrastructure, not a correction
search, train/validation split, or proof of detector performance.

## Deterministic Ideal correction sweep (S2-R3)

`scripts/run_correction_search_sweep.py` plans and executes deterministic,
static-target `ideal_reference_point` qualification grids. It supports all
twelve direct/common/relative optical RDF attitude source parameters, OFAT or
Cartesian attitude cases, Decimal range/bearing/elevation inputs, repetitions,
canonical or seeded execution order, pre-launch actual-camera FOV checks, and
stable case IDs bound to a canonical plan hash. Target cells are authored in
the fixed `baseline_center_frd` frame (Forward, Right, Down) and converted to
Gazebo World ENU through the production coordinate helpers; World axes are not
treated as FRD aliases. R3 explicitly launches the generated research target
as a static Gazebo model while the ordinary launch default remains dynamic and
world gravity remains enabled. The source world and target model are unchanged.

Each case launches from the verified clean repository root and enables the S2
atomic recorder with the exact branch/SHA. The runner stops only its owned
process group, waits for a fixed eligible-sample window, independently checks
R2 schema/status/hash/accounting semantics, and writes case statistics plus a
sweep-level VALID/INVALID marker outside the repository. Resume never deletes
an attempt: only the latest attempt can be skipped, and only after its R2
markers, metadata, hashes, rows, accounting, fixed sample window, stored
statistics, and per-sample GT range/bearing/elevation are revalidated. Invalid,
incomplete, unknown, or spatially moved attempts receive the next numeric
`attempt-NNNN` directory. The requested static physics mode and strict spatial
tolerances are part of the plan and case hashes. Sweep markers form exclusive
PLAN_READY, RUNNING, FINAL_VALID, or FINAL_INVALID states; failed finalization
retains `SWEEP_INCOMPLETE`. Plan-only files contain no time, host, output
path, PID, or attempt data and are byte-reproducible for identical inputs.

This runner does not use the legacy latest-topic recorder, YOLO, dynamic PX4
motion, correction fitting, or train/validation/Test splits. It is a
non-formal static Gazebo qualification tool, not the 72-hour formal correction
search or R4 qualification.

## Sealed Simulation profile qualification (S2-R4-A)

`scripts/run_sealed_correction_qualification.py` prepares the Calibration,
Validation, and sealed-Evaluation plans before any dataset is collected. The
split unit is a complete spatial cell `(range, bearing, elevation)`, so all 25
attitude profiles for one cell stay in one role. R4-A executes only the
Calibration and Validation R3 sweeps. The sealed plan and its ordered case,
profile, and spatial hashes are committed before fitting; no sealed process,
pair record, statistic, or metric may exist until the separate R4-B phase.

The bounded candidate families are nominal Raw, one shared Wahba SO(3)
rotation, and independent left/right Wahba SO(3) rotations. Calibration uses
one median pixel/GT representative per valid static case with equal case
weight. Validation evaluates all 30 analysis records per case and selects one
global family for every attitude profile. Solver inputs are optical RDF pixel
rays and Gazebo GT directions derived through `baseline_center_frd` to World
ENU and nominal camera RDF. Injected attitude values group profiles only;
injected rotations and the R2 oracle Corrected position are not solver inputs.
The fitted matrix direction is
`R_Cnominal_Cactual @ ray_actual_rdf = ray_nominal_gt_rdf`.

The frozen catalog is Simulation method-qualification evidence only. It binds
the exact clean branch/SHA, Calibration and Validation plans, sealed
commitment, candidate code hash, validation score, fitted matrices, access
ledger, and artifact hashes. It records
`runtime_selection_source=not_implemented` and `deployment_ready=false`.
`QUALIFICATION_PROFILE_FROZEN` together with
`SEALED_EVALUATION_NOT_EXECUTED` is the required R4-A terminal state. R4-B may
open that commitment exactly once at the same SHA; R4-A is neither sealed
evaluation nor the formal 72-hour/PX4 correction-search qualification.

## Retired legacy correction tools

As of 2026-07-20, the standalone
`legacy_origin_bbox_roi_tracking_search` tool chain has been removed. That
chain moved the target through Gazebo `set_pose`, fitted Ground-Truth-labelled
Origin-BBOX reprojection profiles, searched ROI/silhouette gains, and evaluated
a constant-velocity Kalman post-filter in the historical stereo-center RDF
contract. Its pass criterion was per-axis and position RMSE improvement on the
saved validation trials.

The retired chain is not part of the PX4-only mission path or the current
Simulation runtime. Historical reports remain under `reports/` as immutable
evidence, but their retired script commands are not runnable from the current
checkout. PX4 missions, Gazebo camera simulation, Raw measurement, and the
current runtime modules remain available.

Gazebo Sim + ROS 2 Humble 기반 지상 고정 스테레오 카메라 시뮬레이션입니다.
PyQt와 브라우저 대시보드는 사용하지 않고, 표준 ROS 2 도구인 `rqt_gui`, RViz2, rosbag을 사용합니다.

## Gazebo 버전

운용 기준은 노트북과 동일한 Gazebo Sim Harmonic 8.x입니다. Ubuntu 22.04에서
설치가 필요하면 `scripts/install_gazebo_harmonic.sh`를 실행합니다. 현재 PC의
ROS 2 Humble apt `ros_gz` 패키지는 Fortress 6에 링크되어 있으므로, Harmonic
설치 전까지는 `GAZEBO_MAJOR_VERSION=6`으로 기존 Fortress 실행을 유지할 수
있습니다. Harmonic 실행은 `GAZEBO_MAJOR_VERSION=8`이 기본값입니다.

## PX4-Autopilot SITL

PX4 SITL 소스는 이 디렉터리 안의 `PX4-Autopilot/`에 `release/1.16` 브랜치로
설치한다. 전체 PX4 checkout과 빌드 산출물은 `.gitignore`에 포함되므로 git에는
커밋하지 않으며, 다른 컴퓨터에서는 다음 명령으로 재생성한다.

```bash
git clone --branch release/1.16 --depth 1 --recurse-submodules \
  https://github.com/PX4/PX4-Autopilot.git Simulation/PX4-Autopilot
PYTHON_EXECUTABLE="$PWD/Simulation/.venv/bin/python" \
  make px4_sitl_default -j"$(nproc)" \
  -C Simulation/PX4-Autopilot
```

현재 `run_operator_view.sh`는 `Simulation/PX4-Autopilot`을 자동으로 찾고,
PX4 빌드에는 `Simulation/.venv/bin/python`을 사용한다. 따라서 아래 실행은
`--no-px4` 없이 PX4 SITL을 함께 시작한다.

```bash
./Simulation/run_operator_view.sh
```

PX4 local position은 `north/east/down` 순서의 NED이고, Gazebo world/model pose와
스테레오 중심 카메라의 `right/down/forward` RDF는 서로 다른 frame이다. SITL
설치 자체는 이 좌표 변환을 변경하지 않는다.

## 구성

- 카메라: 지상 고정 2대
- baseline: 25 m
- camera height: 10 m
- stereo type: parallel stereo
- resolution / FPS: visible 1280x720 @ 20 Hz (50,000,000 ns frame interval)
- horizontal FOV: 0.154997710698 rad, WIRIS Enterprise optical zoom 8x 기준 약 8.88 deg
- distortion: off
- target drone: 약 폭 15 m, 높이 10 m 대형 쿼드콥터 형상
- target motion: QGC/PX4 조종 관찰용. 시작 위치는 카메라 전방 +Y 약 2000 m입니다.
- viewer: 단일 `rqt_gui` 운용 창, 선택적으로 RViz2

## S1 Simulation ground-truth/ray contract

현재 Simulation S1의 절대 Ground Truth는 Gazebo
`/world/stereo_ground_drone/pose/info`의 `gz.msgs.Pose_V`에서 읽은
`base_link` pose이다. 전용 adapter가 원본 `header.stamp.sec/nsec`를 보존해
`/simulation/ground_truth/vehicle_reference_link`
(`geometry_msgs/msg/PoseStamped`, `header.frame_id=gazebo_world_enu`)로
발행한다. 공개 semantic reference 이름은 `vehicle_reference_link`이고 정의는
`project_defined_vehicle_reference_at_base_link_origin`이며, parent는
`base_link`, parent-relative translation은 `[0, 0, 0] m`, rotation은
identity이다. 이 frame은 mass-bearing link가 아니다.

현재 SceneBroadcaster의 `Pose_V` JSON에서는 root `base_link` pose가 모델
relative로 생략될 수 있으므로, adapter는 같은 payload의 `x500_15m` model
world pose와 `base_link` relative pose를 compose해 world ENU로 만든다. 이
parent transform은 world 표현을 위한 것이며 semantic reference를 model
origin으로 대체하는 fallback이 아니다. parent model pose가 payload에
없으면 해당 Ground Truth sample은 unavailable로 남긴다.

nominal zero-error 좌우 optical pose에서 한 번 만든
`baseline_center_frd`가 고정 출력 frame이다. World ENU point는
`p_B = R_B_W @ (p_W - O_W)`로 변환하며, `R_B_W`의 행은 Forward, Right,
Down world 방향이다. World 축을 직접 FRD 별칭으로 사용하지 않는다.

Raw 위치는 좌우 optical RDF pixel을 각각 ray로 만든 뒤 동일한
`sasl_common.stereo_geometry` closest-ray kernel에 넣고 두 최근접점의
midpoint를 저장한다. 결과에는 각 ray 점, midpoint, Forward/Right/Down/Range,
ray gap, 교차각, signed depths, geometry rank, conditioning metric,
`computation_status`가 포함된다. PX4 Local/Global/GPS/Odometry는 이 절대
Ground Truth의 보조 정보일 뿐이다.

Rendered/ideal observation pixel은 actual optical pose에서 투영하고,
canonical Raw는 nominal zero-error optical pose로 같은 pixel을 해석한다.
공통 kernel을 호출할 때는 `common_frame=baseline_center_frd`,
`geometry_policy=diagnostic_signed_depth`, `axis_contract=forward_right_down`를
명시한다. 한 run의 baseline, camera height, FOV, target model identity,
common/relative optical yaw는 geometry를 정의하므로 실행 중 parameter
변경을 거부하고 launch restart로만 바꿀 수 있다. Observation fusion 결과는
legacy/corrected 후보로 보존되며 canonical Raw를 덮어쓰지 않는다.

## S2 simulation-time alignment and injected-attitude oracle

좌우 `sensor_msgs/msg/Image`는 각 `header.stamp.sec/nanosec`를 int64 ns로
변환해 one-to-one pair를 구성한다. Exact timestamp를 우선하고, exact가
없으면 simulation-time watermark 또는 명시적 flush에서 deterministic nearest
pair를 만든다. 한 frame은 한 pair에서만 사용하며 `pair_delta_ns`는
`right-left`, pair midpoint는 `left + (right-left)//2`이다. Simulation reset으로
인한 epoch는 ROS `/clock` 감소만을 authoritative source로 사용해 정확히 한 번
증가시킨다. Image pairer와 GT buffer는 이 epoch를 주입받으며 자체 timestamp
감소로 별도 epoch를 만들지 않는다. Reset 후 현재 `/clock`보다 미래인 이전 epoch
image는 stale pre-reset frame으로 격리해 새 반대쪽 frame과 pair하지 않는다.

Ground Truth history는 pair midpoint를 exact 또는 선형 position interpolation으로
정렬한다. 미래 GT가 아직 없으면 pair를 pending으로 유지하고 bounded overflow,
explicit flush, shutdown에서만 `nearest_before`, `nearest_after`, `unavailable`을
명시한다. Quaternion은 ROS/Gazebo `x,y,z,w` 순서로 보존하며 위치 보간에 Euler
보간을 섞지 않는다.

Measurement와 GT evaluation은 분리된다. Image pair가 생기면 GT availability와
무관하게 measurement queue에서 YOLO/detection, Raw, attitude-corrected를 즉시
계산하고 canonical Raw/Corrected topic을 발행한다. 같은 pair metadata는 별도로
GT aligner에 등록하며, late GT가 준비되면 `pair_id`로 기존 measurement와 한 번만
결합해 GT topic과 error를 계산한다. GT unavailable이면 Raw/Corrected와 해당
diagnostic은 유지하고 GT/error만 unavailable이다. Shutdown은 새 입력을 막은 뒤
image pairer, GT pending alignment, measurement queue, evaluation queue 순서로
drain한다. Image callback admission은 shutdown과 같은 condition lock 안에서
`accepting_input`을 다시 확인하는 지점에서 선형화된다. 따라서 shutdown이 입력을
닫은 뒤 lock을 얻은 callback은 pairer, pair ID, queue 또는 GT aligner를 변경하지
않고 `input_rejected_after_shutdown_count`만 증가시킨다.

자세 연구 입력은 optical RDF local axes 기준 direct left/right roll·pitch·yaw와
각 축의 common/relative 성분이다. 각 축에서 effective left는
`left_direct+common-relative/2`, effective right는
`right_direct+common+relative/2`이다. 단위는 degree이며 내부 회전은
`R_Down(yaw) @ R_Right(pitch) @ R_Forward(roll)` 순서의 full SO(3) 행렬을 사용한다.
Positive roll은 Right를 Down으로, positive pitch는 Forward를 Up(-Down)으로,
positive yaw는 Forward를 Right로 회전시킨다. 기존 yaw-only common/relative 계약은
direct yaw가 0일 때 동일하다. 동일 pair와 동일 bbox pixel에 대해
Raw는 nominal pose, `/stereo/attitude_corrected_position`은 injected actual pose로
ray를 다시 생성해 같은 two-ray midpoint kernel을 호출한다. correction source는
`injected_attitude_oracle`이고 Ground Truth는 correction 입력이 아니다.

Canonical `geometry_msgs/msg/PointStamped` topic은 `/stereo/raw_position`,
`/stereo/attitude_corrected_position`, `/stereo/ground_truth_position`이다. 모두
pair midpoint simulation stamp, `frame_id=baseline_center_frd`, point x/y/z =
Forward/Right/Down metres를 사용한다. 기존 `/stereo/estimated_position`,
`/ground_truth_position`, `/stereo/corrected_position`은 S3 전까지 legacy migration
topic이며 `/stereo/corrected_position`은 injected oracle 의미가 아니다.
Raw와 attitude-corrected는 measurement 완료 직후 발행하고, Ground Truth는 같은
pair midpoint의 exact/interpolated/explicit nearest alignment가 준비된 뒤 발행한다.

YOLO pair 계산은 compute와 commit으로 나뉜다. Inference 완료 뒤 `/clock` epoch를
같은 state lock에서 다시 확인하는 commit이 reset과의 선형화 지점이다. 따라서
reset 전에 시작했지만 reset 뒤 끝난 old-epoch detection/Raw/attitude-corrected는
어떤 canonical topic이나 result map에도 commit하지 않는다. GT는 live callback
순서 probe에서 node 내부 기준 steady state는 최대 20 ms, reset 직후 executor
transient는 최대 108 ms 먼저 도착할 수 있어 64-sample bounded
simulation-time reorder buffer를 사용한다. Clock이 stamp에 도달하면 수용하고,
200 ms보다 먼 future GT와 reset 전 pending GT는 stale
pre-reset data로 폐기한다. Pending GT record에는 callback 수신 당시 epoch를
보존하며 drain 시 current epoch를 다시 부여하지 않는다.

`PoseStamped` 자체에는 epoch ID가 없으므로 reset 후 지연 도착한 old sample을
메시지만으로 완전히 판별할 수 없다. 이 제한에는 conservative simulation-time
quarantine을 적용한다. Reset stamp부터 200 ms까지의 GT는 폐기하며, 이 floor 뒤
단조 증가하는 GT 두 sample이 관측되고 `/clock`이 floor를 통과해야 quarantine을
해제한다. 해제된 sample은 receive-time epoch를 유지한 채 기존 bounded reorder
buffer로 이동하며 position만 선형 보간한다. 따라서 reset마다 초기 200 ms와
continuity 확인 전 sample은 false-stale이 될 수 있지만, wall-clock sleep 없이
old GT 200 ms-window 오염을 차단하고 이후 alignment를 복구한다.

Measurement queue 기본 capacity는 32 pair인 bounded burst buffer이며 20 Hz 지속
처리 보장이 아니다. S2-C2 production-node audit의 CPU 측정은 20 stereo pair
sample에서 image decode, 좌우 순차 YOLO inference와 triangulation을 포함한
`yolo_inference_ms`가 mean 65.53 ms, observed p99/max 74.17 ms였고, 관측된 최대
update rate는 10.76 Hz였다. 이는 project camera input contract 20 Hz보다 낮다.
이전 약 27 ms 수치는 model-only인지 one-image inference인지, decode를 포함하는지,
device와 sample 수가 확인되지 않아 capacity 근거에서 제거했다. Overflow는 silent
drop하지 않고 `measurement_overrun_invalid` run 상태로 전환하며 rejected pair ID와
counter를 남긴다. 따라서 지속 backlog는 strict overrun으로 run-invalid가 된다.

Raw/Corrected dedupe는 단조 pair-id watermark, late GT/evaluation dedupe는 128-entry
bounded history를 사용해 장시간 pair 수에 비례한 set 증가를 막는다. Measurement
terminal은 decode/detection과 detection/Raw/attitude-corrected publication 및 result
저장을 완료한 뒤 `measurement_status=processed`와
`measurement_processed_count`를 확정한다. 이후 GT transform, error 계산과 canonical
GT publication은 별도 evaluation terminal이다. 성공은
`evaluation_status=evaluated`와 `gt_evaluation_success_count`, 실패는
`evaluation_status=evaluation_error`와 `gt_evaluation_error_count` 및 bounded failed
pair history에 한 번만 기록한다. Late GT 또는 measurement-complete trigger의
evaluation 예외는 worker 밖으로 전파되지 않으며 이미 성공한 measurement를
processing error/discard로 재분류하지 않는다.

Shutdown drain은 5초 안에 measurement/GT queue와 in-flight inference가 모두 끝나고
measurement accounting이 일치하며 measurement/evaluation error가 모두 0일 때만
`SUCCESS`다. Timeout, processing error 또는 GT evaluation error는 각각 구분된
진단을 남기고 `INCOMPLETE`다. Pair compute 또는 measurement commit 예외는 pair
단위 failure boundary에서 error/discard를 각 한 번 기록하고 관련 state와 in-flight
flag를 정리하며, 같은 worker는 다음 pair를 계속 처리한다.

ROS publisher 여러 개는 cross-topic transaction이 아니다. Detection, Raw,
attitude-corrected 또는 GT publish 중 예외가 발생하면 일부 topic만 외부에 전달될 수
있다. 현재 보장은 worker 생존, bounded cleanup과 terminal accounting이며 publish
rollback은 제공하지 않는다. 이 제한의 구조적 해결에는 향후 atomic pair recorder
또는 typed aggregate message가 필요하다.

아직 구현하지 않은 범위는 post-hoc 삭제, PX4-only movement cleanup, pair recorder,
typed aggregate message, speed, roll/pitch 연구, BBox observation error correction이다.

## 시뮬레이션 목적

이 시뮬레이션의 목적은 StereoDistance로 실제 본 실험에 가까운 조건의
거리/축별 위치 측정과 보정 방법을 검증하는 것이다. 기준 운용 조건은
25 m stereo baseline, WIRIS visible 8배줌, 약 15 m급 드론/UAM 표적이다.
StereoDistance는 이 조건에서 Stereo WIRIS 영상과 YOLO bbox를 사용해
`estimated_distance`와 카메라 중앙 기준 `Forward/Right/Down` 축 데이터를
산출한다.

측정 수준을 벗어나는 자세각 오차는 단순한 raw stereo 계산만으로 안정적으로
제거하기 어렵다. 따라서 드론/UAM에 Pixhawk와 GPS를 사용해 보정용 비행을
수행하고, 이때 얻은 위치 기준값을 ground truth로 사용한다. 시뮬레이션은
보정용 비행에서 만든 ground truth와 Stereo WIRIS로 측정한 데이터를 비교해
자세각 오차가 거리와 각 축에 만드는 편향을 분석하고, 이를 줄일 보정값과
보정 정책을 찾는 역할을 한다.

그 다음 계측용 실험 조건에서는 보정용 비행에서 얻은 정보를 이용해 고정한
보정 방법을 적용한다. 시뮬레이션에서는 계측용 조건에서도 ground truth를
알 수 있으므로, 보정 후 `ground_truth`, `estimated_distance`, 그리고
`Forward/Right/Down` 축 데이터가 최대한 동일하게 나오고 축별 regression이
발생하지 않는 방법을 찾는다. 실제 계측용 본 실험에서는 ground truth를
런타임 입력으로 사용하지 않고, 시뮬레이션과 보정용 비행에서 검증한 보정
방법을 적용한 뒤 사후 분석으로 정확도를 확인한다.

## RTK 스테레오 보정법 비교 파이프라인

PX4 local position NED를 보정 비행의 ground truth로 두고, 스테레오 카메라의
자세오차와 시차오차를 줄이는 후보 보정법을 단일 Python 실행으로 비교할 수
있다. 실제 Gazebo/PX4/YOLO 토픽 CSV가 아직 없어도 synthetic mode가 기본으로
동작한다.

이 저장소의 `Simulation/.venv`는 현재 별도 패키지 설치용 pip 환경이 아니라
시스템 Python 패키지를 참조하는 lightweight venv로 사용한다. 따라서 README의
`Simulation/.venv/bin/python` 명령은 시스템에 설치된 `numpy`, `scipy`,
`matplotlib`, `opencv-python` 계열 패키지를 볼 수 있어야 한다. 이 환경에서는
`Simulation/.venv/pyvenv.cfg`의 `include-system-site-packages = true` 설정으로
검증됐고, acceptance dashboard wrapper와 deployment package check가 이 경로로
PASS했다.

좌표계 계약은 다음과 같다.

- PX4 local position: `px4_local_position_ned:north,east,down`
- 스테레오 중심 카메라 좌표: `stereo_center_opencv_optical:RDF(right,down,forward)`
- 평가는 항상 스테레오 중심 RDF에서 수행한다:
  `corrected_rdf` vs `truth_rdf`.
- `truth_rdf`는 반드시 명시적인 `T_source_to_stereo_rdf`로 생성된 값이어야 한다.
  시뮬레이션에서도 카메라 optical frame이 local NED에 대해
  `forward=north`, `right=east`, `down=down`으로 정렬되어 있다는 직접 축
  alias 가정은 사용하지 않는다.
- 실제 CSV/비행 데이터에서 PX4 NED만 있고 `truth_right_m`,
  `truth_down_m`, `truth_forward_m` 또는 동등한 명시 변환 산출물이 없으면
  보정 평가 입력으로 보지 않는다.

허용되는 truth source는 PX4 local position NED에 한정되지 않는다. 보정/검증에는
다음 source를 사용할 수 있다.

- PX4 local position NED: source frame을 `px4_local_position_ned`로 둔
  `T_source_to_stereo_rdf`가 필요하다.
- RTK/GNSS world 좌표(LLH/ECEF/ENU/NED): 기준 원점과
  `T_world_to_stereo_rdf`가 필요하다.
- Gazebo world/model pose: Gazebo world frame과 stereo rig frame 사이
  `T_gazebo_world_to_stereo_rdf`가 필요하다.
- 외부 계측 장비(motion capture, total station, LiDAR tracker 등):
  장비 frame에서 stereo-center RDF로 가는 transform이 필요하다.
- 이미 stereo-center RDF로 발행/저장된 truth:
  producer가 RDF `right/down/forward`와 truth point를 명시해야 한다.

어떤 source를 쓰더라도 평가 입력은 동일하다.

```text
truth_source_position --T_source_to_stereo_rdf--> truth_rdf
estimated_camera_rdf  --calibration_profile-----> corrected_rdf
evaluation: corrected_rdf vs truth_rdf
```

source-frame truth CSV를 보정 pipeline에 넣기 전에는 명시 transform JSON으로
`truth_right_m`, `truth_down_m`, `truth_forward_m`을 생성한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/convert_truth_source_to_stereo_rdf.py \
  --input-csv Simulation/results/calibration_flight/telemetry_ned_only.csv \
  --output-csv Simulation/results/calibration_flight/telemetry_with_truth_rdf.csv \
  --transform-json Simulation/config/truth_source_to_stereo_rdf.example.json \
  --strict
```

transform JSON은 source frame, source axis order, source field, origin, rotation, translation을
명시해야 한다. 예시는 다음과 같다.

```json
{
  "source_frame": "px4_local_position_ned",
  "source_axis_order": ["north", "east", "down"],
  "source_fields": ["truth_north_m", "truth_east_m", "truth_down_ned_m"],
  "origin_source_m": [0.0, 0.0, 0.0],
  "rotation_euler_zyx_deg_source_to_stereo_rdf": [12.0, -3.0, 1.5],
  "translation_rdf_m": [0.0, 0.0, 0.0]
}
```

`rotation_matrix_source_to_stereo_rdf`를 3x3 행렬로 제공해도 된다. 이 transform은
카메라 optical frame과 local/world frame의 실제 외부표정 또는 별도 보정 산출물이어야
하며, NED와 RDF 축을 이름만 바꿔 대응시키는 직접 alias는 허용하지 않는다.
변환 도구는 NED source에서 `rdf_right=east`, `rdf_down=down`,
`rdf_forward=north`가 되는 순열 행렬을 입력하면 실패한다.

CSV 없이 transform 파일만 먼저 검증하려면:

```bash
Simulation/.venv/bin/python Simulation/scripts/convert_truth_source_to_stereo_rdf.py \
  --transform-json Simulation/config/truth_source_to_stereo_rdf.example.json \
  --validate-only
```

end-to-end wrapper에서도 같은 변환을 바로 연결할 수 있다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_estimated_to_truth_calibration_exp.py \
  --mode csv \
  --input-csv Simulation/results/calibration_flight/telemetry_ned_only.csv \
  --truth-transform-json Simulation/config/truth_source_to_stereo_rdf.example.json \
  --output-dir Simulation/results/rtk_stereo_calibration_from_csv \
  --strict-preflight
```

이 경우 wrapper는 `truth_rdf_conversion/input_with_truth_rdf.csv`를 먼저 만들고,
그 변환된 CSV를 preflight와 fitting pipeline에 전달한다. 원본 source-frame CSV와
변환 summary는 `metrics.json`의 `calibration_truth_source_input_csv`,
`pipeline_input_csv`, `truth_rdf_conversion`에 함께 기록된다.

보정 후보는 모두 RDF 배열 순서 `[Right, Down, Forward]`에서 학습/평가된다.
legacy `x/y/z` 필드는 쓰지 않고 CSV에는 `*_right_m`, `*_down_m`,
`*_forward_m`, `truth_north_m/east_m/down_m`처럼 축 이름을 명시한다.

기본 synthetic 실행:

```bash
Simulation/.venv/bin/python Simulation/scripts/rtk_stereo_calibration_pipeline.py
```

결과 위치를 지정하려면:

```bash
Simulation/.venv/bin/python Simulation/scripts/rtk_stereo_calibration_pipeline.py \
  --output-dir Simulation/results/rtk_stereo_calibration_smoke
```

정책별 추천이 seed/split에 얼마나 안정적인지 보려면 synthetic multi-seed
집계를 실행한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_policy_multi_seed_validation.py \
  --output-dir Simulation/results/rtk_stereo_policy_multi_seed \
  --seeds 20260613,20260614,20260615
```

CSV 기록 데이터를 여러 split seed로 재검증하려면 `--mode csv --input-csv <file>`를
같이 지정한다. `--output-dir`와 CSV mode의 `--input-csv`는 wrapper 시작 시
절대 경로로 해석되므로, `Simulation/results/...` 같은 상대 경로를 넘겨도
내부 pipeline subprocess와 후속 replay/promotion check가 같은 산출물 위치를
참조한다. 출력은 `seed_runs.csv`, `per_seed_method_comparison.csv`,
`per_seed_policy_recommendations.csv`, `method_aggregate_summary.csv`,
`policy_stability_summary.csv`, `policy_stability_gate.csv`,
`method_stability_gate.csv`, `selection_near_ties.csv/json`,
`method_family_diagnostics.csv/json`,
`multi_seed_recommendation.json`,
`multi_seed_recommended_profiles.csv/json`,
`selection_near_tie_profiles.csv/json`,
`multi_seed_representative_profile.json`, `representative_runtime_profile.json`,
`representative_profile_checks.json`,
`representative_cross_seed_replay_summary.csv/json`,
`multi_seed_summary.json/txt`, `policy_stability_ci.png`,
`method_aggregate_ci.png`이다. `policy_stability_gate.csv`는 모든 seed 실행
성공, raw 대비 평균 validation RMSE 개선, 1852m 허용오차 통과, 정책별
선택 method의 과반 안정성을 분리해 기록한다. raw 대비 평균 validation RMSE
개선은 기본적으로 최소 `0.01m` 이상이어야 하며
`--min-validation-improvement-m`으로 조정할 수 있다. aggregate summary에는
validation RMSE와 1852m 오차 평균의 bootstrap 90% confidence interval도
함께 기록한다. `method_stability_gate.csv`는 정책과 무관하게 각 보정법이
seed 전체에서 raw 대비 개선, 1852m 허용오차, validation CI 기준을 통과하는지
기록한다. `multi_seed_recommendation.json`은 strict pass method를 먼저
고르고, 없으면 performance pass method 중 validation RMSE 평균, validation
worst-seed gap, 1852m 오차 평균과 seed별 최대값, CI 폭, RDF 축별 RMSE
regression, 복잡도 penalty를 함께 본 후보를 기록한다.
`method_stability_gate.csv`에는 raw 대비 RDF right/down/forward RMSE가
후퇴했는지와 최대/합산 regression도 함께 기록된다. 또한 multi-seed 추천과
같은 점수식으로 계산한 `selection_score`, `selection_rank`,
`selection_score_contract`가 기록되어 최종 추천 후보와 차순위 후보의 trade-off를
같은 CSV에서 비교할 수 있다.
`selection_near_ties.csv/json`은 rank 1 대비 score delta가 작거나 상위 rank에
있는 후보들을 따로 모아, 최종 추천이 단독 우세인지 아니면 사실상 동률 후보가
있는지 validation/1852m 차이와 함께 기록한다.
`method_family_diagnostics.csv/json`은 각 method를 `registration_model`,
`residual_model`, `regularization_model`, `runtime_profile_family` 조합으로
묶어 family별 best method, strict pass 수, validation/1852m 성능을 기록한다.
이 파일은 다음 자동탐색에서 어떤 보정 계열을 더 넓힐지 판단하기 위한 offline
진단이며, runtime truth 입력 계약을 바꾸지 않는다.
`multi_seed_recommended_profiles.csv/json`은 multi-seed 추천 method에 대해
각 seed의 `runtime_profiles/<method>.json` 경로, seed별 validation 성능,
축 계약, runtime 입력 계약을 모아 실제 StereoDistance 적용 후보 검토에
사용한다. `multi_seed_representative_profile.json`은 seed별 profile 중
multi-seed 평균 validation/1852m 성능에 가장 가까운 대표 seed profile을
고른 메타데이터이고, `representative_runtime_profile.json`은 그 profile을
최상위로 복사한 바로 적용 가능한 runtime profile이다.
`selection_near_tie_profiles.csv/json`은 최종 추천과 점수 차이가 작은
near-tie 후보들까지 seed별 runtime profile 경로와 validation/1852m 성능을
함께 남긴다. 이 파일은 단일 winner만 고정하지 않고 StereoDistance에 적용
가능한 후보 profile을 계속 탐색하기 위한 offline 후보 manifest이며, runtime
입력은 raw stereo-center RDF와 measured disparity로 제한된다.
`representative_profile_checks.json`은 해당 대표 profile을 validation split에
replay하고 promotion check까지 실행한 결과를 요약한다. replay는 raw 대비
corrected range RMSE 개선과 1852m 오차 통과를 확인하고, promotion check는
StereoDistance loader 호환성과 runtime truth 입력 금지를 확인한다.
`representative_cross_seed_replay_summary.csv/json`은 동일한 대표 profile을
모든 완료 seed의 validation split에 다시 replay해, 특정 seed에만 맞는
profile인지 아니면 seed 전체에서 raw 대비 개선과 1852m 조건을 통과하는지
검증한다. `multi_seed_summary.json/txt`에는 `multi_seed_acceptance_matrix`도
포함된다. 이 matrix는 `seed_runs.csv`, `method_aggregate_summary.csv`,
`policy_stability_gate.csv`, `method_stability_gate.csv`,
`multi_seed_recommendation.json`, `multi_seed_recommended_profiles.csv`,
`representative_profile_checks.json`,
`representative_cross_seed_replay_summary.csv`가 각각 seed coverage, raw 대비
validation 개선, 1852m target error, 추천 후보 존재, runtime profile 존재,
StereoDistance promotion check, cross-seed replay 조건을 통과했는지 PASS/FAIL과
증거 metric으로 기록한다.

오차 주입 조건을 바꿔도 같은 보정 계열이 유지되는지 보려면 robustness
sweep을 실행한다. 이 sweep은 multi-seed 검증을 여러 synthetic error
scenario에서 반복하고, 각 scenario의 `method_stability_gate.csv`를 다시
모아 cross-scenario 기준으로 추천 method를 고른다. 목적은 한 seed나 한
오차 조건에서만 우연히 좋은 후보가 아니라, StereoDistance runtime profile로
적용 가능한 안정적인 estimated-to-truth 보정 후보를 계속 탐색하는 것이다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_robustness_sweep.py \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep \
  --seeds 20260613,20260614,20260615
```

기본 scenario는 `nominal`, `yaw_pitch_stress`, `disparity_noise_stress`,
`intrinsic_scale_stress`, `combined_sensor_stress`, `bbox_outlier_stress`이다.
`combined_sensor_stress`는
자세오차, baseline/focal scale 오차, disparity bias/noise, bbox-center noise를
동시에 키워 단일 오차 조건에만 맞는 후보를 걸러낸다. `sparse_target_window`는
기본에는 포함하지 않는 진단용 scenario이며, 1852m 근방 표본이 부족할 때
target-window gate가 PASS를 막는지 확인할 때만 `--scenarios`에 명시한다.
`bbox_outlier_stress`는 일부 sample의 bbox 중심과 disparity에 큰 튐을
주입해 RANSAC, trimmed, Huber, Tukey 같은 강건 정합 후보가 Gaussian noise
조건뿐 아니라 gross outlier 조건에서도 안정적인지 확인한다. pooled scenario
profile은 기본적으로 여러 ranking 기준에서 뽑은 후보를 transfer replay하며,
`--pooled-candidate-scan-limit`로 replay할 pooled runtime profile 수를
조절한다. `0`으로 지정하면 pooled pipeline이 export한 모든 runtime profile을
replay한다.
추가 synthetic error scenario는 JSON으로 정의해 자동 탐색 범위를 늘릴 수
있다.

```json
{
  "scenarios": [
    {
      "name": "custom_disparity_noise",
      "purpose": "larger disparity and bbox-center noise",
      "args": ["--disparity-noise-px", "0.20", "--bbox-noise-px", "0.40"]
    },
    {
      "name": "custom_bbox_outlier",
      "purpose": "gross bbox-center outlier stress",
      "args": ["--bbox-outlier-rate", "0.10", "--bbox-outlier-px", "3.0"]
    }
  ]
}
```

```bash
Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_robustness_sweep.py \
  --output-dir Simulation/results/rtk_stereo_custom_sweep \
  --scenario-config-json Simulation/config/custom_scenarios.json \
  --scenarios nominal,custom_disparity_noise
```

출력은 `scenario_runs.csv`,
`scenario_recommendations.csv`, `scenario_method_gates.csv`,
`robust_method_summary.csv`, `robust_method_family_summary.csv`,
`robust_policy_recommendations.csv`,
`scenario_near_tie_profiles.csv/json`,
`scenario_near_tie_profile_transfer_replay.csv`,
`scenario_near_tie_profile_transfer_summary.csv/json`,
`scenario_near_tie_method_transfer_summary.csv/json`,
`scenario_near_tie_best_profile.json`,
`scenario_near_tie_best_runtime_profile.json`,
`scenario_near_tie_best_promotion_check.json`,
`robust_policy_profile_candidates.csv`,
`robust_policy_representative_profiles.csv`,
`robust_policy_profile_transfer_replay.csv`,
`robust_policy_profile_transfer_summary.csv`,
`robust_policy_profile_transfer_diagnostics.csv/json`,
`scenario_profile_catalog.csv/json`,
`scenario_profile_catalog_promotion_checks.csv/json`,
`scenario_profile_catalog_transfer_replay.csv`,
`scenario_profile_catalog_transfer_matrix.csv`,
`scenario_profile_catalog_transfer_coverage.csv`,
`scenario_profile_catalog_transfer_summary.json`,
`scenario_profile_catalog_minimal_set.csv/json`,
`pooled_scenario_profile_transfer_replay.csv`,
`pooled_scenario_profile_summary.csv/json`,
`pooled_scenario_profile_transfer_diagnostics.csv/json`,
`deployment_strategy_recommendation.json/txt`,
`robustness_summary.json/txt`이다.
`robust_method_summary.csv`는 method별 scenario 통과 횟수, validation RMSE
평균/최대, 1852m target error 평균/최대, RDF 축별 RMSE regression 합,
복잡도 penalty를 기록한다. `robust_method_family_summary.csv`는
`registration_model`, `residual_model`, `regularization_model`,
`runtime_profile_family`가 같은 후보들을 묶어 계열별 best method와 score를
요약한다. `robustness_summary.json`의
`robust_recommendation`은 모든 scenario strict pass 후보를 먼저 고르고,
없으면 performance pass 전체 통과 후보, 그것도 없으면 partial robustness가
가장 좋은 후보를 기록한다. `robust_policy_recommendations.csv`는 같은 pass
후보를 `balanced`, `validation_best`, `target_guard`, `simple_stable` 기준으로
다시 추천해, validation RMSE 최저 후보와 1852m worst-case 보수 후보가
달라지는지 확인한다. 각 policy method에 대해서는 모든 scenario/seed의
`runtime_profiles/<method>.json` 후보를 모아 policy 평균 validation/1852m
성능에 가장 가까운 대표 profile을
`robust_policy_profiles/<policy>_runtime_profile.json`으로 복사한다.
`scenario_near_tie_profiles.csv/json`은 각 scenario의 multi-seed near-tie
runtime profile 후보를 최상위 sweep 산출물로 모은다. scenario별 winner 하나만
남기지 않고, score delta가 작은 StereoDistance 적용 가능 후보들을 함께 보존해
다음 자동탐색이나 실제 보정 비행 replay에서 비교할 수 있게 한다. 이 manifest도
offline 진단이며 runtime 입력은 raw stereo-center RDF와 measured disparity로
제한된다.
`scenario_near_tie_profile_transfer_replay.csv`와
`scenario_near_tie_profile_transfer_summary.csv/json`은 이 near-tie profile들을
모든 scenario/seed validation split에 다시 replay한다. 이를 통해 특정 scenario의
차순위 후보가 다른 오차 조건에서는 winner보다 더 전이성이 좋은지 확인한다.
PASS 기준은 다른 replay와 동일하게 raw 대비 validation RMSE 개선, target-window
sample 존재, 1852m 근방 error 10m 이하이다.
`scenario_near_tie_method_transfer_summary.csv/json`은 near-tie transfer 결과를
method 단위로 다시 묶어, 특정 seed/profile 하나가 아니라 같은 보정법 계열이
반복적으로 모든 transfer replay를 통과하는지 기록한다. method별 profile 수,
통과 profile 수, best profile, 평균/최악 validation RMSE와 target error를 함께
남기므로 다음 자동탐색에서 어떤 method 계열을 더 넓힐지 판단하는 근거가 된다.
`scenario_near_tie_best_profile.json`은 모든 transfer replay를 통과한 near-tie
후보 중 validation RMSE 평균, 1852m worst-case error, winner 대비 score delta
순으로 가장 좋은 profile을 기록한다. 해당 profile은
`scenario_near_tie_best_runtime_profile.json`으로 복사되어 단일 runtime profile
후보로 바로 검토할 수 있다. `scenario_near_tie_best_promotion_check.json`은 이
복사본이 StereoDistance loader로 읽히고 runtime truth 없이 validation sample에
적용되는지 검증한다. `deployment_strategy_recommendation`도 이 near-tie best
profile이 policy representative profile보다 transfer 성능이 좋고 promotion
check를 통과했을 때만 `single_near_tie_runtime_profile` 전략을 기록한다.
`robust_policy_profile_transfer_replay.csv`는 이 대표 profile 하나를 모든
scenario/seed validation split에 다시 replay해, 특정 synthetic 조건의 profile만
좋은지 아니면 다른 오차 조건에도 전이되는지 검증한다. 이 transfer replay가
실패하더라도 method 자체가 실패했다는 뜻은 아니며, 한 환경에서 fit한
profile을 다른 센서오차 조건에 universal하게 적용하면 위험하다는 뜻이다.
`robust_policy_profile_transfer_diagnostics.csv/json`은 이 실패를
policy/scenario 단위로 요약한다. `failure_reasons`에는
`no_validation_rmse_improvement`, `target_error_over_10m`,
`missing_target_window_validation_samples`, `replay_gate_failed` 같은 원인이
기록되고, 실패 scenario에는 `recommended_action=use_scenario_catalog_profile`이
기록된다.
`robust_method_summary.csv`, `scenario_profile_catalog.csv`,
`robust_policy_recommendations.csv`,
`robust_policy_representative_profiles.csv`는 method 이름만으로 판단하지 않도록
`registration_model`, `residual_model`, `regularization_model`,
`runtime_profile_family`, `selection_note`를 함께 기록한다. 예를 들어
`sim3_smoothed_binned_residual_disparity_bins11`은
`registration_model=sim3_estimated_to_truth`,
`residual_model=forward_disparity_lut`,
`regularization_model=lut_shrinkage_neighbor_smoothing+lut_bin_count_grid`,
`runtime_profile_family=estimated_to_truth_sim3`로 남는다. 이 필드는 outlier나
희소 target-window 조건에서 smoothed/bin-grid 후보가 왜 선택됐는지 해석하기
위한 offline 진단이며, runtime 입력 계약은 바꾸지 않는다.
`scenario_profile_catalog.csv/json`과 `scenario_profile_catalog/`는 각
scenario에서 자체 calibration data로 fit한 대표 profile을 모아 둔다. 이
catalog는 “현재 환경에서 보정 비행을 수행하고 해당 환경 profile을 선택한다”는
운용 방식을 검증하기 위한 산출물이다.
`scenario_profile_catalog_promotion_checks.csv/json`은 catalog의 각 profile이
StereoDistance loader로 읽히고 runtime truth 없이 validation sample에 적용되는지
검사한다. 이 추천 역시 calibration-time truth로 평가만 하며, runtime profile
입력은 raw stereo-center RDF와 measured disparity뿐이다.
`scenario_profile_catalog_transfer_*` 산출물은 catalog의 각 scenario profile을
다른 모든 scenario/seed validation split에 replay해 cross-scenario coverage를
행렬로 기록한다. 이 결과는 catalog를 반드시 scenario별로 나눠야 하는지,
또는 일부 profile이 여러 환경을 커버할 수 있는지 확인하는 진단용이며, 실패한
transfer profile을 runtime에 자동 승격하지 않는다.
`scenario_profile_catalog_minimal_set.csv/json`은 이 transfer matrix에서 모든
validation replay를 통과한 source-target 쌍만 사용해, 전체 scenario를 덮는
최소 후보 profile 집합을 greedy set cover로 산출한다. 이 결과는 catalog
운용을 줄이기 위한 offline 진단이며, runtime에서는 선택된 profile 중 하나를
보정 비행/배포 메타데이터 또는 운용자 설정으로 고른다. live PX4/RTK truth나
validation error를 runtime selection 입력으로 사용하지 않는다.
`pooled_scenario_profile/`과 `pooled_scenario_profile_summary.csv/json`은 여러
scenario의 calibration CSV를 하나로 합쳐 단일 profile을 다시 fitting한 결과를
기록한다. 목적은 scenario catalog 없이도 stress-invariant한 하나의 profile이
가능한지 확인하는 것이다. 이 pooled fitting도 calibration-time PX4/RTK truth를
사용하지만, promotion/replay/runtime 적용은 raw stereo-center RDF와 measured
disparity만 사용한다. pooled pipeline의 최종 추천 하나만 승격하지 않고,
validation RMSE, target-window error, 낮은 복잡도, policy 추천에서 나온 여러
runtime profile 후보를 다시 모든 scenario/seed validation split에 replay한 뒤
선택한다. `pooled_scenario_profile_transfer_replay.csv`에서 선택 후보의 모든
scenario/seed validation replay가 raw 대비 개선되고 1852m target error 10m
이하이며 promotion check를 통과할 때만 `single_pooled_runtime_profile` 후보가
된다. `pooled_scenario_profile_transfer_diagnostics.csv/json`은 선택된 pooled
profile 또는 모든 pooled 후보가 실패한 target scenario, 실패 원인
(`no_validation_rmse_improvement`, `target_error_over_10m` 등), 그리고
catalog/minimal-set profile로 전환해야 하는 scenario를 기록한다. 단순
inverse-depth ray scale 계열에는 identity ray scale로 shrink되는
ridge/Tikhonov 후보도 포함된다. 이 후보는 baseline/focal scale 오차처럼 전역
거리 scale에 가까운 문제에서 과한 Sim(3) 또는 residual LUT 보정을 피하는
안정성 기준 후보이며, output contract는 stereo-center RDF ray scale이다.
`deployment_strategy_recommendation.json/txt`는 sweep 결과를 배포 관점에서
해석한다. 단일 대표 profile이 모든 scenario/seed transfer replay를 통과하면
`single_runtime_profile`을 권장하고, near-tie 후보가 더 잘 전이되면
`single_near_tie_runtime_profile`을 권장한다. 둘 다 실패했지만 pooled profile이
모든 transfer replay와 promotion check를 통과하면
`single_pooled_runtime_profile`을 권장한다. 단일 profile 계열이 모두 깨지지만
scenario별 catalog가 cross-seed replay와 promotion check를 통과하면
`scenario_profile_catalog`를 권장한다.
`robustness_summary.json/txt`에는 `robustness_acceptance_matrix`도 포함된다.
이 matrix는 `scenario_runs.csv`, `scenario_recommendations.csv`,
`robust_method_summary.csv`, `robust_policy_profile_transfer_summary.csv`,
`scenario_profile_catalog.json`,
`scenario_profile_catalog_promotion_checks.json`,
`scenario_profile_catalog_transfer_coverage.csv`,
`deployment_strategy_recommendation.json`이 각각 scenario coverage, scenario별
추천 존재, cross-scenario method pass, policy profile transfer, scenario catalog,
StereoDistance promotion, catalog transfer coverage, 최종 deployment strategy
조건을 통과했는지 PASS/FAIL과 증거 metric으로 기록한다.
cross-scenario robust recommendation은 strict/performance pass 후보 중에서도
RDF right/down/forward 축별 RMSE regression sum이 `0`인 후보를 먼저 고른다.
range RMSE나 1852m error가 낮아도 RDF 축별 non-regression을 깨는 후보는
axis-clean 후보가 하나라도 있으면 robust recommendation에서 제외되고, acceptance
matrix의 `robust_recommendation` evidence에도 `rdf_axis_nonregression_pass`가 기록된다.

StereoDistance는 런타임에 단일 profile JSON을 읽으므로, catalog에서 현재
환경 profile을 명시적으로 선택해 active profile로 복사한 뒤 사용한다.
선택 기준은 보정 비행/배포 메타데이터 또는 운용자 설정이어야 하며, runtime
truth로 고르면 안 된다.

2026-06-13 bbox/disparity gross outlier stress를 포함한 기본 6-scenario
sweep에서는 각 scenario별 catalog profile은 모두 promotion check를 통과했지만,
robust policy representative profile의 transfer replay는 18개 scenario/seed
조합 중 15개만 통과했다. 따라서 현재 추천 배포 전략은 단일 universal profile이
아니라 `scenario_profile_catalog`이다.

```bash
Simulation/.venv/bin/python Simulation/scripts/select_stereodistance_profile_from_catalog.py \
  --deployment-strategy-json Simulation/results/rtk_stereo_robustness_sweep/deployment_strategy_recommendation.json \
  --scenario nominal \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep/active_runtime_profile.json
```

생성된 `active_runtime_profile.json`을 기존 StereoDistance profile 경로
파라미터에 지정한다. selector는 cross-seed replay를 통과하지 못한 catalog
profile을 기본적으로 거부한다. `deployment_strategy_recommendation.json`이
`single_runtime_profile`, `single_near_tie_runtime_profile`,
`single_pooled_runtime_profile`이면 `--scenario` 없이도 검증된 단일 runtime
profile을 active profile로 복사할 수 있다. `scenario_profile_catalog`이면 현재
환경을 나타내는 `--scenario`를 반드시 명시해야 하며, 이 선택은 runtime truth가
아니라 보정 비행/배포 메타데이터 또는 운용자 설정에서 와야 한다.

minimal set 산출물이 있으면 scenario별 catalog 전체를 직접 운용하지 않고,
offline transfer matrix에서 검증된 더 작은 profile set으로 매핑할 수 있다.
예를 들어 6-scenario sweep에서 `nominal` profile이 nominal/yaw/disparity/
combined/bbox outlier 계열을 덮고, `intrinsic_scale_stress` profile이 intrinsic
scale mismatch 계열을 덮는다면 다음처럼 현재 배포 조건을 입력한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/select_stereodistance_profile_from_catalog.py \
  --deployment-strategy-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/deployment_strategy_recommendation.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/scenario_profile_catalog_minimal_set.json \
  --scenario combined_sensor_stress \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep_minimal_set/active_runtime_profile.json
```

이 경우 `--scenario`는 runtime truth가 아니라 배포/보정 조건 label이다.
selector는 minimal set의 `newly_covered_target_scenarios`와
`all_pass_target_scenarios`를 사용해 검증된 source profile을 고르고, 선택
summary에는 `requested_scenario`와 실제 복사된 source `scenario`를 함께 기록한다.

배포 조건 label을 확정하기 어렵고 새 보정 비행 CSV에 truth/raw RDF/disparity가
들어 있다면, runtime이 아니라 calibration-time에서 minimal set 후보들을 replay해
active profile을 고를 수 있다. 이 선택은 보정 비행 truth로만 수행되며, 복사된
runtime profile 자체는 계속 raw RDF/disparity만 입력으로 사용한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/select_stereodistance_profile_by_calibration_replay.py \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/scenario_profile_catalog_minimal_set.json \
  --input-csv Simulation/results/rtk_stereo_robustness_sweep_minimal_set/combined_sensor_stress/seed_20260614/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set/calibration_profile_selection \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep_minimal_set/active_runtime_profile.json
```

`calibration_replay_profile_candidates.csv`에는 후보별 validation RMSE, raw 대비
개선량, 1852m target-window count/error, PASS 여부가 기록된다. 또한
`registration_model`, `residual_model`, `regularization_model`,
`runtime_profile_family`, `selection_note`가 함께 기록되어 calibration replay로
선택된 후보가 Sim(3), affine, disparity polynomial, smoothed/bin-grid LUT 중
어떤 계열인지 추적할 수 있다. PASS 후보는 `corrected RMSE < raw RMSE`,
target-window sample 존재, `target_abs_mean_error_corrected_m <= 10m` 조건을
만족해야 한다. 후보 선택은 corrected RMSE만 보지 않고
`replay_selection_score = corrected RMSE + 0.30 * target_abs_error + 0.10 *
target_max_abs_error + 0.05 * catalog_target_max_abs_error + 0.08 *
model_complexity_estimate + stability_penalty`를 사용해 validation 성능,
1852m 근방 평균/최대 오차, catalog worst-case, 모델 복잡도, split 간
안정성을 함께 반영한다. 기본적으로
CSV에 `train`/`validation` split label이 있으면 두 split 모두에서 raw 대비
range RMSE가 개선되어야 하며, target-window sample이 있는 split에서는
1852m 평균 및 최대 오차 허용치도 통과해야 한다. split label이 없는 실제
CSV에서는 이 추가 gate를 건너뛰고 기존 단일 replay 기준을 유지한다.
CSV에는 `pass_reason`, `range_improvement_margin_m`, `target_error_margin_m`,
`target_max_error_margin_m`,
`stability_splits_evaluated`, `stability_min_range_improvement_m`,
`stability_max_target_abs_mean_error_m`, `stability_penalty_m`,
`model_complexity_estimate`, `replay_selection_score`, `replay_selection_rank`가
같이 기록된다. `--stability-splits ""`를 주면 split-stability gate를 끌 수
있고, 다른 split label을 쓰는 보정 비행 CSV라면 comma-separated 값으로
지정할 수 있다.
기본 실행은 `--minimal-set-json`이 있으면 deployment-ready minimal set 후보만
replay한다. 보정 비행에서 sensor-error regime을 더 적극적으로 식별하려면
`--candidate-source-mode all_catalog`를 추가해 catalog의 모든 validated profile을
후보로 자동 탐색할 수 있다. 이 모드도 선택 과정에서만 truth를 쓰며, 복사되는
runtime profile은 동일하게 raw RDF/disparity 입력만 사용한다.

보정 비행 truth를 아직 사용할 수 없거나, runtime에서 profile을 자동 선택해야
하는 경우에는 raw measurement guard를 별도 실험으로 돌린다. 이 selector의 목적은
truth와 estimated 오차를 직접 보는 것이 아니라, 현재 StereoDistance 입력의 raw
분포만으로 어떤 검증 profile family를 적용할지 고르는 것이다. 입력 계약은
stereo-center OpenCV optical RDF `raw_right_m,raw_down_m,raw_forward_m`,
`raw_range_m`, `measured_disparity_px`, 선택적으로 `left_u_px,left_v_px`이다.
PX4 NED truth, RDF truth, true disparity, residual/error 필드는 selection에서
금지하고 audit에서만 쓴다. 출력 profile은 기존 StereoDistance runtime profile과
같이 raw RDF/disparity만 소비한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/select_stereodistance_profile_by_runtime_measurement_guard.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_audit_all_catalog_transfer_risk \
  --candidate-source-mode all_catalog \
  --selection-policy transfer_risk_fallback \
  --audit-all-sweep-seeds
```

`nearest_centroid`는 raw feature centroid가 가장 가까운 scenario profile을 고른다.
`robust_family_fallback`은 같은 raw feature로 preliminary scenario를 고른 뒤,
nominal/yaw/bbox처럼 raw trajectory만으로 잘 분리되지 않는 저응력/자세/outlier
군을 검증된 `bbox_outlier_stress` profile로 접는다. 이 fallback은 runtime truth를
쓰지 않는다. 더 일반적인 `transfer_risk_fallback`은 hardcoded family 이름 대신
offline `scenario_profile_catalog_transfer_matrix.csv`에서 raw-feature preliminary
scenario와 상호 low-risk인 confusion cluster를 자동으로 만든다. 기본 low-risk
기준은 cross-seed transfer PASS, mean corrected range RMSE `<= 1.3m`,
max target abs mean error `<= 2.5m`이다. 그런 다음 cluster 안 source profile을
`mean RMSE + 0.10 * max target error + negative-improvement penalty + fail penalty`
점수로 비교해 더 안전한 source를 고른다. selection은 raw RDF/disparity와 transfer
matrix metadata만 쓰고, truth는 replay audit에서만 쓴다.

PASS 조건은 replay audit 기준으로 validation corrected range RMSE가 raw보다 낮고,
train/validation stability split이 있는 경우 둘 다 개선되며, 1852m target
window의 평균 및 최대 range error가 `±10m` 이내인 것이다. 2026-06-16
monotone-inverse-depth sweep에서 `all_catalog + transfer_risk_fallback`
leave-one-out audit는 18/18 PASS, validation corrected range RMSE mean/max
`1.089/1.676 m`, 1852m target abs mean/max `1.009/2.087 m`, target max abs
worst-case `9.681 m`를 기록했다. 같은 조건의 `nearest_centroid`는 17/18 PASS,
minimal-set centroid는 12/18 PASS였으므로 현재 자동 runtime profile 선택 후보는
`all_catalog + transfer_risk_fallback`이다.

`transfer_risk_fallback`의 low-risk threshold가 특정 sweep에 과하게 맞춰졌는지
확인하려면 threshold grid를 별도 감사한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/sweep_runtime_measurement_guard_thresholds.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_transfer_threshold_sweep \
  --candidate-source-mode all_catalog \
  --rmse-limits-m 0.9,1.1,1.3,1.5,1.8 \
  --target-limits-m 1.5,2.0,2.5,3.0,4.0
```

산출물은 `runtime_measurement_guard_threshold_sweep.csv`,
`runtime_measurement_guard_threshold_sweep_summary.txt/json`,
`runtime_measurement_guard_threshold_sweep_heatmap.png`이다. 2026-06-16
monotone-inverse-depth sweep에서는 25개 threshold 조합 중 9개가 18/18 PASS였고,
추천 threshold는 RMSE `1.3m`, target `2.5m`였다. 이 값은 18/18 PASS plateau의
가장 보수적인 시작점이며, 그보다 낮은 threshold에서는 nominal/yaw/bbox confusion
cluster가 충분히 묶이지 않아 17/18 PASS, target worst-case `10.055m`로 PASS
한계를 넘었다.

runtime-measurement selector 자체도 같은 조건에서 비교한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_runtime_measurement_guard_policies.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog_minimal_set.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_policy_audit \
  --candidate-source-modes minimal_set,all_catalog \
  --policies nearest_centroid,robust_family_fallback,transfer_risk_fallback,feature_vote_transfer_risk,knn_transfer_risk,centroid_knn_union_transfer_risk,centroid_knn_veto_transfer_risk \
  --transfer-low-risk-rmse-limit-m 1.3 \
  --transfer-low-risk-target-limit-m 2.5
```

산출물은 `runtime_measurement_guard_policy_audit.csv`,
`runtime_measurement_guard_policy_audit_summary.txt/json`,
`runtime_measurement_guard_policy_audit.png`이다. 같은 sweep에서
`all_catalog + transfer_risk_fallback`과 `all_catalog + robust_family_fallback`은
둘 다 18/18 PASS, target worst-case `9.681m`였지만, transfer-risk policy는
fallback count가 4로 hardcoded robust-family fallback의 9보다 작고, fallback
cluster를 transfer matrix에서 유도한다. `all_catalog + nearest_centroid`는
17/18 PASS, target worst-case `10.205m`였고, minimal-set 계열은 12/18 PASS였다.
따라서 현재 자동 runtime selector 추천은
`all_catalog + transfer_risk_fallback`이다.

추가 탐색 후보인 `feature_vote_transfer_risk`는 raw RDF/range/disparity/image-center
feature를 `range_geometry`, `disparity_depth`, `bearing_image` subset으로 나누어
각 subset의 centroid vote를 만든 뒤 transfer-risk cluster를 다시 평가한다. runtime
truth 입력은 여전히 쓰지 않는다. 2026-06-16
`runtime_measurement_guard_policy_audit_feature_vote` 결과에서는 all-catalog
기준 15/18 PASS, validation RMSE max `2.780m`로 기존 `transfer_risk_fallback`보다
나빴다. 실패는 `intrinsic_scale_stress` 3개 seed에서 nominal profile을 선택해
raw 대비 range RMSE 개선이 사라진 경우였다. 따라서 이 후보는 보존하지만 현재
추천에서 제외한다.

centroid 평균 대신 개별 보정 비행 run의 raw feature 이웃을 쓰는
`knn_transfer_risk`도 탐색한다. 이 방법은 raw stereo-center
RDF/disparity/image-center feature 공간에서 가장 가까운 `k`개 run의 source 후보를
만든 뒤 transfer-risk score로 최종 source를 고른다. runtime truth 입력은 쓰지
않는다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/sweep_runtime_measurement_guard_knn.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_knn_sweep \
  --candidate-source-mode all_catalog \
  --neighbor-counts 1,3,5,7,9 \
  --transfer-low-risk-rmse-limit-m 1.3 \
  --transfer-low-risk-target-limit-m 2.5
```

산출물은 `runtime_measurement_guard_knn_sweep.csv`,
`runtime_measurement_guard_knn_sweep_summary.txt/json`,
`runtime_measurement_guard_knn_sweep.png`이다. 2026-06-16 결과에서는 `k=5`가
최선으로 18/18 PASS, 1852m target max abs worst-case `9.681m`를 만족했지만
validation RMSE max가 `2.491m`로 기존 `transfer_risk_fallback`의 `1.676m`보다
컸다. 따라서 kNN 후보도 현재 추천에서 제외하고, 모든 run을 통과한 보조 후보로만
보존한다.

kNN과 centroid transfer-risk를 결합하는 hybrid도 감사한다. `centroid_knn_union`
방식은 centroid low-risk cluster와 kNN 후보를 단순 union으로 묶었고,
`centroid_knn_veto` 방식은 centroid preliminary와 양방향 low-risk인 kNN 후보만
추가했다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_runtime_measurement_guard_policies.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog_minimal_set.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_policy_audit_hybrid_veto \
  --candidate-source-modes all_catalog \
  --policies transfer_risk_fallback,knn_transfer_risk,centroid_knn_union_transfer_risk,centroid_knn_veto_transfer_risk \
  --transfer-low-risk-rmse-limit-m 1.3 \
  --transfer-low-risk-target-limit-m 2.5 \
  --knn-neighbor-count 5
```

2026-06-16 결과에서 `centroid_knn_union_transfer_risk`는 15/18 PASS로 나빠졌다.
`centroid_knn_veto_transfer_risk`는 18/18 PASS, validation RMSE max `1.676m`,
1852m target max abs worst-case `9.681m`로 기존 `transfer_risk_fallback`과
동일한 선택을 했다. 동률이면 복잡도 penalty를 적용해 더 단순한
`transfer_risk_fallback`을 추천한다.

보정 비행 truth로 만든 calibration-replay best source를 학습 라벨로 사용하는
runtime selector도 별도 탐색한다. 이 방식은 training label을 만들 때만 truth를
사용하고, leave-one-out 대상 선택에는 raw stereo-center RDF/disparity/image-center
feature만 사용한다. 따라서 실제 runtime profile 입력 계약은 바뀌지 않는다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_runtime_oracle_label_selector.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --calibration-replay-runs-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/profile_selection_strategy_audit_all_catalog/profile_selection_strategy_runs.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_oracle_label_selector_audit_nearest \
  --selection-policy nearest_label_centroid
```

산출물은 `runtime_oracle_label_selector_audit.csv`,
`runtime_oracle_label_selector_summary.txt/json`,
`runtime_oracle_label_selector_selection_counts.png`이다. 2026-06-16 결과에서
`nearest_label_centroid`는 16/18 PASS, calibration-replay label match 11/18,
validation RMSE max `2.710m`, 1852m target max abs worst-case `10.205m`였다.
`label_centroid_transfer_risk`는 14/18 PASS로 더 나빴다. 실패는
`bbox_outlier_stress` 일부 seed의 1852m target max 초과와 `intrinsic_scale_stress`
일부 seed의 raw 대비 RMSE 개선 실패였다. 즉 truth 기반 best-source label을 그대로
학습해도 현재 raw feature만으로는 label noise와 domain overlap을 충분히 분리하지
못하므로, 이 후보도 현재 추천에서 제외한다.

truth를 사용할 수 있는 offline benchmark에서는 runtime selector와 oracle replay
best의 차이도 감사한다. oracle은 같은 CSV에서 catalog profile 전체를 truth로
replay해 가장 좋은 PASS 후보를 고르는 기준이므로 runtime 입력으로 쓰면 안 된다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_runtime_measurement_guard_oracle_regret.py \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --guard-audit-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_audit_all_catalog_transfer_risk/runtime_measurement_guard_audit.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_oracle_regret_transfer_risk \
  --candidate-source-mode all_catalog
```

산출물은 `runtime_measurement_guard_oracle_regret.csv`,
`runtime_measurement_guard_oracle_candidates.csv`,
`runtime_measurement_guard_oracle_regret_summary.txt/json`,
`runtime_measurement_guard_oracle_regret.png`이다. 2026-06-16 sweep에서
`transfer_risk_fallback`은 18/18 PASS이고 oracle best와 8/18에서 같은 source를
골랐다. 평균 RMSE regret은 `-0.036m`로 selector가 oracle-score 후보보다 RMSE만은
약간 낮은 경우가 있었고, 1852m target abs regret mean/max는 `0.146/0.613m`,
target max regret mean/max는 `0.138/0.612m`였다. 따라서 남은 개선 여지는 주로
nominal/yaw/bbox cluster 안에서 target-window error를 1m 미만 더 줄이는
tie-breaker 탐색이며, 현재 PASS 목표에는 충분한 margin이 있다.

transfer-risk score 안에서 target error 가중치만 바꾸는 단순 tie-breaker도
별도로 감사한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/sweep_runtime_measurement_guard_target_weight.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_target_weight_sweep \
  --candidate-source-mode all_catalog \
  --weights 0.0,0.05,0.1,0.2,0.4,0.8 \
  --transfer-low-risk-rmse-limit-m 1.3 \
  --transfer-low-risk-target-limit-m 2.5
```

산출물은 `runtime_measurement_guard_target_weight_sweep.csv`,
`runtime_measurement_guard_target_weight_sweep_summary.txt/json`,
`runtime_measurement_guard_target_weight_sweep.png`이다. 2026-06-16 sweep에서는
weight `0.0`부터 `0.8`까지 모두 18/18 PASS, target abs regret max `0.613m`,
target max regret max `0.612m`, oracle source match 8/18로 동일했다. 따라서 현재
남은 regret은 transfer-risk score의 target 가중치만으로는 줄지 않으며, 추가 raw
feature 또는 calibration-time replay 결합 selector가 다음 탐색 대상이다.

truth 사용 가능 여부에 따라 selector 운용 모드를 분리한다. runtime truth가 없으면
`transfer_risk_fallback` guard를 쓰고, 보정 비행 CSV에 PX4/local-position truth가
있으면 calibration-time replay selector를 쓴다. calibration replay는 profile을
고를 때만 truth를 사용하며, export된 runtime profile은 여전히 raw RDF/disparity만
소비한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_selector_operating_modes.py \
  --runtime-guard-audit-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_audit_all_catalog_transfer_risk/runtime_measurement_guard_audit.csv \
  --calibration-replay-runs-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/profile_selection_strategy_audit_all_catalog/profile_selection_strategy_runs.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/selector_operating_mode_audit
```

산출물은 `selector_operating_mode_comparison.csv`,
`selector_operating_mode_summary.csv`, `selector_operating_mode_recommendation.json`,
`selector_operating_mode_summary.txt`, `selector_operating_mode_comparison.png`이다.
2026-06-16 sweep에서는 runtime guard와 calibration replay가 모두 18/18 PASS였다.
runtime guard의 validation range RMSE mean은 `1.089m`, calibration replay는
`1.125m`였고, 1852m target abs mean error는 runtime guard `1.009m`,
calibration replay `0.863m`였다. calibration replay는 9/18 run에서 target abs
error를 더 낮췄다. 따라서 운용 기준은 다음과 같다: truth가 있는 보정/검증 단계는
`calibration_replay_all_catalog`, truth가 없는 runtime 자동 선택은
`runtime_transfer_risk_guard`.

위 기준을 active runtime profile export에 바로 적용하려면 operating-mode exporter를
사용한다. `--mode auto`는 CSV에 truth field가 있으면 calibration replay를 쓰고,
truth field가 없으면 runtime measurement guard를 사용한다. runtime guard mode는
truth가 없으므로 replay RMSE/target error를 계산하지 않고, raw feature와 transfer
matrix metadata만으로 profile을 선택해 복사한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/export_stereodistance_profile_by_operating_mode.py \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --input-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/combined_sensor_stress/seed_20260613/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/operating_mode_export_active \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/operating_mode_export_active/active_runtime_profile.json \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --mode auto \
  --calibration-candidate-source-mode all_catalog \
  --runtime-candidate-source-mode all_catalog \
  --runtime-selection-policy transfer_risk_fallback
```

산출물은 `operating_mode_profile_export_summary.txt/json`과
`active_runtime_profile.json`이다. summary의 `selected_operating_mode`가
`calibration_replay`이면 보정 비행 truth로 profile을 선택했다는 뜻이고,
`runtime_guard`이면 truth 없이 raw RDF/disparity와 transfer matrix metadata만으로
profile을 선택했다는 뜻이다. 두 경우 모두 `active_runtime_profile.json`은
StereoDistance runtime에서 raw RDF/disparity만 입력으로 사용한다.

intrinsic-scale gap을 runtime-safe product-ratio로 감지한 뒤 catalog에 있는 같은
family profile을 바로 export하려면 runtime selection policy를
`intrinsic_scale_gap_transfer_risk`로 지정한다. 이 경로는 truth field가 CSV에
있더라도 `--mode runtime_guard`를 지정하면 truth를 선택 입력으로 쓰지 않는다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/export_stereodistance_profile_by_operating_mode.py \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --input-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/intrinsic_scale_stress/seed_20260613/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/operating_mode_export_intrinsic_scale_guard_smoke \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/operating_mode_export_intrinsic_scale_guard_smoke/active_runtime_profile.json \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --mode runtime_guard \
  --runtime-candidate-source-mode all_catalog \
  --runtime-selection-policy intrinsic_scale_gap_transfer_risk
```

2026-06-16 smoke 결과에서는 product-ratio p10 `1.003814`가 threshold `1.0037`을
넘어 `intrinsic_scale_stress` profile을 선택했고,
`intrinsic_scale_product_ratio_guard_selects_scale_profile`로 기록됐다. Export된
profile은 promotion check PASS, StereoDistance node loader PASS,
runtime_truth_used `false`였으며, 같은 validation split replay에서 range RMSE가
raw `1.872m`에서 corrected `0.888m`로 감소했다. 따라서 catalog에 같은
intrinsic-scale family profile이 존재할 때는 runtime truth 없이도 적용 가능한
active profile export 경로가 있다. 반대로 LOSO처럼 해당 source profile이 없으면
감지는 되지만 보정 profile이 없어 calibration-time profile export가 필요하다.

새 intrinsic-scale regime을 보정 비행에서 처음 만났고 CSV에 PX4/RTK truth가
있다면, 같은 CSV로 새 runtime profile을 fitting/export할 수 있다. 이 wrapper는
먼저 runtime-safe product-ratio p10 guard를 확인하고, 감지되면
`rtk_stereo_calibration_pipeline.py --mode csv`를 실행해 validation 기준으로
추천 method를 고른 뒤 active runtime profile로 복사한다. Truth는 이 calibration
fit 단계에서만 쓰이고, export된 profile은 runtime에서 raw RDF/disparity만 쓴다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/export_intrinsic_scale_profile_from_calibration.py \
  --input-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/intrinsic_scale_stress/seed_20260613/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/intrinsic_scale_calibration_export_smoke \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/intrinsic_scale_calibration_export_smoke/active_runtime_profile.json \
  --target-range-m 1852 \
  --target-window-m 30 \
  --seed 20260613
```

2026-06-16 smoke 결과에서는 product-ratio p10 `1.003814`로 intrinsic gap을
감지했고, 새 calibration fit의 추천 method는 `global_inverse_depth_linear`였다.
Promotion check는 PASS, StereoDistance node loader는 PASS, runtime_truth_used는
`false`였다. 같은 validation split replay에서 range RMSE는 raw `1.872m`에서
corrected `0.975m`로 감소했고 1852m target abs mean error는 `0.763m`였다. 이
경로가 현재 LOSO intrinsic-scale 실패에 대한 실제 해결 후보이며, 다음 검증은
여러 intrinsic-scale 보정 비행 seed에서 export profile의 cross-seed 전이를 보는
것이다.

같은 family 안에서 전이되는지 확인하려면 active profile transfer audit를
`--target-scenarios intrinsic_scale_stress`로 제한해 실행한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_active_profile_transfer.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --profile-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/intrinsic_scale_calibration_export_smoke/active_runtime_profile.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/intrinsic_scale_calibration_export_smoke/intrinsic_family_transfer \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10 \
  --target-scenarios intrinsic_scale_stress
```

2026-06-16 결과는 intrinsic-scale family 3/3 PASS였다. Validation RMSE corrected
mean/max는 `0.950/1.024m`, 최소 RMSE 개선은 `0.897m`, 1852m target abs mean
error worst-case는 `1.436m`, target max abs worst-case는 `2.410m`였다. 같은
profile을 전체 sweep에 고정 적용하면 4/18 PASS에 그치므로 universal profile은
아니지만, product-ratio로 감지한 intrinsic-scale family에 대해서는 보정 비행
1개 seed에서 export한 profile이 다른 seed까지 전이됐다.

보정 비행 source seed를 하나로 고정하지 않고 family 내부 모든 seed를 번갈아
calibration source로 삼아 자동 export/replay하려면 family calibration export
audit를 실행한다. 이 audit는 각 source seed의 CSV로 runtime profile을 새로 fit한
뒤 같은 scenario family의 모든 target seed에 replay하고, source seed를 제외한
heldout 전이 성능을 따로 집계한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_family_calibration_profile_exports.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/intrinsic_scale_family_calibration_export_audit \
  --target-scenarios intrinsic_scale_stress \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10 \
  --seed 20260613
```

출력은 `family_calibration_profile_exports.csv`,
`family_calibration_profile_transfer.csv`,
`family_calibration_profile_scenario_summary.csv/json`,
`family_calibration_profile_export_summary.json/txt/png`,
`exports/*/active_runtime_profile.json`, `per_run_predictions/*`이다.
2026-06-16 결과는 export 3개, family replay 9/9 PASS, heldout replay 6/6 PASS였다.
Validation RMSE corrected mean/max는 `0.893/1.024m`, 1852m target max abs
worst-case는 `2.410m`였다. 따라서 intrinsic-scale gap은 “runtime-safe 감지 →
family-local calibration export → heldout family replay” 루프를 통과한 첫 family다.

같은 audit를 특정 family로 제한하지 않고 전체 scenario에 대해 실행하면, 각 scenario
family별로 보정 비행 seed를 runtime profile로 export하고 같은 family의 heldout seed에
전이되는지 확인할 수 있다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_family_calibration_profile_exports.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/all_family_calibration_export_audit \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10 \
  --seed 20260613
```

2026-06-16 전체 family 결과는 export 18개, family replay 54/54 PASS, heldout replay
36/36 PASS였다. Validation RMSE corrected mean/max는 `1.167/1.877m`, 1852m target
max abs worst-case는 `9.882m`였다. Scenario별 heldout은 모두 PASS였지만
`bbox_outlier_stress`가 target max `9.882m`로 pass limit `10m`에 가장 가까웠다.
따라서 다음 탐색 우선순위는 bbox/outlier 계열에서 target-window 최대오차 margin을
늘리는 robust weighting 또는 outlier-aware profile selection이다. 이 audit의 runtime
profile은 StereoDistance 적용 시 raw stereo-center RDF/disparity만 소비하며,
truth는 calibration-time fitting과 offline validation/replay metric에만 사용된다.

family calibration export audit는 `--export-policies`로 calibration fit 내부의 여러
policy profile을 동시에 export/replay할 수 있다. `bbox_outlier_stress`의 pass margin을
확인하려면 다음처럼 실행한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_family_calibration_profile_exports.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/bbox_policy_family_calibration_export_audit \
  --target-scenarios bbox_outlier_stress \
  --export-policies final_recommendation,balanced,target_specialist,target_max_stable,lowest_validation_rmse,axis_balanced,lowest_complexity_pass \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10 \
  --seed 20260613
```

2026-06-16 결과에서 `target_specialist`는 1852m target max worst-case를 `9.438m`까지
낮췄지만 validation RMSE max가 `2.788m`이고 최소 RMSE 개선이 `0.016m`뿐이라 모델
안정성이 약했다. 자동 추천 기준은 heldout 전부 통과와 최소 validation RMSE 개선
`0.5m` 이상을 먼저 요구한 뒤 target max와 validation RMSE max를 비교한다. 이 기준의
추천 policy는 `target_max_stable`였고, heldout 6/6 PASS, validation RMSE corrected
max `1.553m`, 최소 RMSE 개선 `1.305m`, 1852m target max abs worst-case `9.681m`였다.
즉 기존 balanced/final 추천의 `9.882m`와 `lowest_validation_rmse`의 `9.787m`보다
target-window 최대오차 margin이 좋아졌다. 다만 bbox/outlier family는 여전히 가장
빡빡한 조건이다.

같은 policy sweep을 전체 scenario family에 적용하면 scenario별 policy 추천도 함께
저장된다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_family_calibration_profile_exports.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/all_policy_family_calibration_export_audit \
  --export-policies final_recommendation,balanced,target_specialist,target_max_stable,lowest_validation_rmse,axis_balanced,lowest_complexity_pass \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10 \
  --seed 20260613
```

출력은 기존 family export 산출물에 더해
`family_calibration_profile_policy_summary.csv/json`,
`family_calibration_profile_scenario_policy_summary.csv/json`,
`family_calibration_profile_scenario_policy_recommendations.csv/json`을 포함한다. 2026-06-16
결과에서 global policy 추천은 `target_max_stable`였고 heldout 36/36 PASS,
validation RMSE corrected max `2.432m`, 최소 RMSE 개선 `0.514m`, 1852m target max abs
worst-case `9.681m`였다. Scenario별 추천은 `bbox_outlier_stress=target_max_stable`,
`combined_sensor_stress=balanced`, `disparity_noise_stress=lowest_complexity_pass`,
`intrinsic_scale_stress=balanced`, `nominal=balanced`,
`yaw_pitch_stress=lowest_complexity_pass`였다. 모든 scenario 추천은 heldout 6/6 PASS를
만족했다.

scenario별 추천 policy를 실제 StereoDistance runtime selection/export에서 쓸 수
있도록 catalog로 묶으려면 policy profile catalog export를 실행한다. 이 단계는
calibration-time audit 결과를 읽어 각 scenario에서 가장 안정적인 source seed의
`active_runtime_profile.json`을 복사하고, 기존 `scenario_profile_catalog`와 같은
manifest 형식으로 저장한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/export_policy_profile_catalog_from_family_audit.py \
  --base-catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --family-audit-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/all_policy_family_calibration_export_audit \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/policy_family_profile_catalog \
  --output-catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/policy_family_profile_catalog/manifest.json
```

2026-06-16 결과는 profile 6/6 PASS, 선택 source profile 6/6 PASS였다. 선택된 source
profile 기준 worst-case target max abs error는 `8.951m`였고, 가장 약한
`bbox_outlier_stress` profile은 `target_max_stable` policy의
`sim3_local_linear_residual_disparity_sigma020_support7`였다. 기존 catalog selector와
호환되므로 다음처럼 scenario 기반 active profile export에 바로 사용할 수 있다.

```bash
Simulation/.venv/bin/python Simulation/scripts/select_stereodistance_profile_from_catalog.py \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/policy_family_profile_catalog/manifest.json \
  --scenario bbox_outlier_stress \
  --output-profile /tmp/sasl_bbox_policy_active_profile.json \
  --selection-summary-json /tmp/sasl_bbox_policy_selection_summary.json
```

선택된 bbox policy catalog profile을 `bbox_outlier_stress` family에 replay한 결과는
3/3 PASS, validation RMSE corrected max `1.553m`, 최소 RMSE 개선 `1.305m`,
1852m target max abs worst-case `8.951m`였다.

policy catalog를 truth-free runtime measurement guard에 직접 넣어 전체 sweep을 감사할
수도 있다. 이 실행은 profile 선택에 truth를 쓰지 않고 raw RDF/disparity/image-center
통계만 사용한다. 단, replay metric 계산에는 offline 검증용 truth를 사용한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/select_stereodistance_profile_by_runtime_measurement_guard.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/policy_family_profile_catalog/manifest.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_audit_policy_catalog_transfer_risk \
  --candidate-source-mode all_catalog \
  --selection-policy transfer_risk_fallback \
  --audit-all-sweep-seeds \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

2026-06-16 결과는 18/18 PASS였다. Validation RMSE corrected mean/max는
`1.154/1.806m`로 기존 runtime guard catalog의 `1.089/1.676m`보다 약간 커졌지만,
1852m target max abs worst-case는 `9.681m`에서 `8.951m`로 낮아졌다. 즉 policy
catalog runtime guard는 평균 RMSE보다 1852m 최대오차 margin을 우선할 때 더 적합한
truth-free 배포 후보이다.

같은 policy catalog에서 `robust_family_fallback`을 사용하면 transfer matrix 없이도
동일하게 18/18 PASS, validation RMSE corrected mean/max `1.154/1.806m`, 1852m target
max abs worst-case `8.951m`를 얻었다. 선택 분포는 `bbox_outlier_stress`,
`nominal`, `yaw_pitch_stress`를 bbox-robust profile로 접고, 나머지 stress family는
각 자기 scenario profile을 선택한다. 같은 target max 성능이면 matrix 의존이 없는
`policy_catalog+robust_family_fallback`이 현재 더 단순한 truth-free runtime 추천이다.

같은 selector를 strict LOSO 조건으로 감사할 수도 있다. 여기서는 target scenario
family profile을 library에서 제거하므로, 새 stress regime이 기존 catalog로 대체
가능한지 확인한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/select_stereodistance_profile_by_runtime_measurement_guard.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/policy_family_profile_catalog/manifest.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_audit_policy_catalog_robust_family_loso \
  --candidate-source-mode all_catalog \
  --selection-policy robust_family_fallback \
  --audit-all-sweep-seeds \
  --exclude-target-scenario-from-library \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

현재 저장된 strict LOSO 결과는 14/18 PASS, validation RMSE corrected mean/max
`1.554/2.375m`, 1852m target abs mean/max `1.814/4.065m`, target-window max abs
worst-case `7.377m`이다. validation-only 진단 기준으로는 17/18 PASS다.
`bbox_outlier_stress` profile이 제외된 경우에는 `disparity_noise_stress` profile을
fallback으로 사용해 1852m 초과 실패를 제거했다. 남은 엄격 PASS 실패는
`intrinsic_scale_stress` 3개 run의 train-side improvement 부족과
`bbox_outlier_stress` 1개 run의 train-side target guard 실패다. validation-only에서
실제로 남는 실패는 `intrinsic_scale_stress/seed_20260613` 하나다. 즉 runtime 배포
후보는 full catalog 18/18 PASS 결과를 기준으로 유지하되, LOSO 탐색은
intrinsic-scale family coverage와 train-side guard 안정화가 다음 과제다.

intrinsic-scale gap 후보로 stereo depth 식 `Z=fB/d`에 기반한
`disparity_product_ray_scale`과 `disparity_product_ray_scale_sim3`도 추가했다. 이
후보는 train truth에서 calibrated `d_measured * truth_forward` product를 fitting하고,
runtime에서는 measured disparity와 raw stereo-center RDF만 사용해 raw ray forward를
`product / disparity`로 재스케일한다. `disparity_product_ray_scale_sim3`는 그 뒤
Umeyama Sim(3)를 적용하므로 StereoDistance runtime profile로 export/apply 가능하다.
좌표 계약은 입력/출력 모두 stereo-center RDF이며, public topic 변환은 기존
`FRD_range(forward,right,down,range)` 계약을 유지한다.

focused intrinsic-scale smoke sweep
`/tmp/sasl_disparity_product_intrinsic_sweep_smoke`에서는
`disparity_product_ray_scale_sim3`가 raw validation RMSE `1.785m`를 `0.998m`로
줄였고 1852m target max abs error는 `1.862m`였다. 그러나 기존
`target_weighted_global_inverse_depth_linear`는 validation RMSE `0.633m`, target max
`1.260m`, `sim3_local_linear_residual_disparity_sigma020_support5`는 validation RMSE
`0.627m`, target max `0.877m`로 더 좋았다. 따라서 product-ray 후보는 현재 추천을
대체하지 않고, reference-based intrinsic-scale diagnostic baseline으로 유지한다.

product-ray 후보를 전 scenario에서 빠르게 확인하려면 focused sweep과 후보 요약을
별도로 실행한다. 이 sweep은 메인 3-seed robustness 결과를 대체하지 않고, 새 후보의
상대 위치를 확인하는 탐색용 산출물이다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_robustness_sweep.py \
  --output-dir Simulation/results/rtk_stereo_product_ray_focused_sweep \
  --scenarios nominal,yaw_pitch_stress,intrinsic_scale_stress,disparity_noise_stress,bbox_outlier_stress,combined_sensor_stress \
  --seeds 20260613 \
  --sample-count 160 \
  --target-range-m 1852 \
  --target-window-m 30 \
  --synthetic-target-oversample-count 12 \
  --pooled-candidate-scan-limit 20

Simulation/.venv/bin/python Simulation/scripts/summarize_method_candidate_across_sweep.py \
  --sweep-dir Simulation/results/rtk_stereo_product_ray_focused_sweep \
  --output-dir Simulation/results/rtk_stereo_product_ray_focused_sweep/disparity_product_ray_candidate_summary \
  --method-substring disparity_product_ray_scale \
  --target-error-limit-m 10
```

현재 focused sweep 요약에서 product-ray 후보 row는 14개이며 target max 10m 기준은
13/14가 통과했다. 하지만 validation RMSE mean/max는 `1.775/3.237m`, target max
worst-case는 `11.826m`, 최소 RMSE 개선은 `-0.179m`였다. 개별 scenario의
`disparity_product_ray_scale_sim3`는 raw보다 좋아졌지만 rank는 대체로 180위권이고,
best method는 여전히 inverse-depth 또는 residual-disparity 계열이다. 이 결과는
intrinsic-scale gap이 단순 `fB` product 보정 하나로 해결되기보다, regime detection과
scenario-specific profile 선택이 더 중요하다는 증거로 남긴다.

raw RDF/disparity-only 조건에서 bundle-adjustment식 카메라 파라미터 보정에 가장
가까운 후보로 `joint_disparity_ray_rotation_nls` 계열도 포함한다. 이 후보는 보정
비행 truth로 `d_corr = a*d_meas + b`와 stereo-center RDF rotation을 scipy
`least_squares`로 동시에 fitting한다. runtime에서는 `raw RDF` ray를
`d_meas/d_corr`로 scale하고 고정 RDF rotation만 적용하므로 PX4/RTK truth, true
disparity, image truth를 사용하지 않는다. Huber loss와 1852m target-range weighting
variant도 함께 생성된다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_robustness_sweep.py \
  --output-dir Simulation/results/rtk_stereo_joint_disparity_ray_rotation_bbox_probe \
  --scenarios bbox_outlier_stress \
  --seeds 20260613,20260614,20260615 \
  --sample-count 520 \
  --target-range-m 1852 \
  --target-window-m 30 \
  --synthetic-target-oversample-count 12 \
  --pooled-candidate-scan-limit 20
```

현재 bbox-only 3-seed probe에서 `huber_joint_disparity_ray_rotation_nls`는
multi-seed representative/catalog profile로 선택됐고 promotion check도 PASS했다.
cross-seed replay는 3/3 PASS, validation RMSE mean/max `1.230/1.540m`,
1852m target max abs worst-case `9.935m`였다. seed별 method aggregate에서는
`huber_joint_disparity_ray_rotation_nls`가 validation RMSE mean/max
`1.223/1.533m`, target abs mean max `1.990m`를 기록했다. 다만 같은 bbox near-tie
transfer에서 `sim3_binned_residual_disparity` seed 20260615 profile은 target max
`9.214m`였고, 기존 full policy catalog runtime 추천은 `8.951m`이므로 이 후보는
최종 추천을 대체하지 않는다. reference-based camera-geometry NLS baseline으로
보존하고, 향후 full 좌/우 feature correspondence가 저장되면 실제 reprojection
bundle-adjustment 후보로 확장한다.

StereoDistance 런타임에 더 직접 적용 가능한 camera-geometry 후보로
`left_disparity_camera_geometry_sim3` 계열도 추가했다. 이 후보의 목적은 raw RDF를
사후 보정하기 전에 `left_u_px`, `left_v_px`, `measured_disparity_px`에서
stereo-center RDF(`right,down,forward`)를 다시 만들고, 보정 비행 truth로 추정한
고정 파라미터만 런타임에 쓰는 것이다. 입력 가정은 좌측 bbox/feature 중심과 measured
disparity가 런타임에 존재한다는 것이며, 출력 contract는 기존과 동일한 internal
RDF 및 public StereoDistance FRD range topic이다. 학습에서는 `d_corr=a*d+b`,
left image center offset, global Sim(3)을 fit하지만, 런타임에서는 PX4/RTK truth,
true disparity, true image feature location을 사용하지 않는다. PASS 조건은 기존과
같이 validation RMSE 개선, 1852m target-window error ±10m, cross-seed/runtime
promotion check 통과 여부로 판단한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_robustness_sweep.py \
  --output-dir Simulation/results/rtk_stereo_left_disparity_camera_geometry_probe \
  --scenarios bbox_outlier_stress \
  --seeds 20260613,20260614,20260615 \
  --sample-count 520 \
  --target-range-m 1852 \
  --target-window-m 30 \
  --synthetic-target-oversample-count 12 \
  --pooled-candidate-scan-limit 80
```

현재 bbox-only 3-seed aggregate에서 target-weighted Huber left-disparity geometry
후보는 validation RMSE mean `1.219m`로 5위였고, 1852m target abs mean worst-case
`2.069m`로 충분히 PASS 범위에 있었다. 그러나 pooled profile validation에서는
65위(`1.634m`)였고, pooled/cross-seed replay 후보에는 선택되지 않았다. 같은 probe의
pooled transfer best는 `sim3_binned_residual_disparity_bins11`로 replay RMSE
mean/max `1.209/1.517m`, target abs worst-case `2.066m`였다. 따라서 이 후보는
StereoDistance에 적용 가능한 reference-based geometry baseline으로 유지하되, 현재
최종 추천을 대체하지 않는다.

reprojection-error 기반 stereo calibration 연구에서는 2D correspondence에 좋은
parameter set과 3D reconstruction에 좋은 parameter set이 달라질 수 있음을 지적한다.
이를 반영해 `left_reprojection_geometry_sim3` 계열도 추가했다. 이 후보는 보정
비행 truth RDF를 left image center/disparity 관측으로 투영했을 때의 pixel/disparity
residual을 Huber NLS로 줄이고, 그 fitted `d_corr=a*d+b`, left-u/v offset으로
runtime 재삼각측량을 수행한 뒤 Umeyama Sim(3)를 적용한다. runtime 입력은
`left_u_px`, `left_v_px`, `raw_disparity_px`, raw RDF뿐이며 PX4/RTK truth와
true image location은 calibration/replay 단계에서만 사용한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/rtk_stereo_calibration_pipeline.py \
  --output-dir Simulation/results/rtk_stereo_left_reprojection_geometry_smoke \
  --seed 20260613 \
  --sample-count 520 \
  --target-range-m 1852 \
  --target-window-m 30 \
  --synthetic-target-oversample-count 12 \
  --bbox-outlier-rate 0.08 \
  --bbox-outlier-px 2.5 \
  --disparity-outlier-rate 0.06 \
  --disparity-outlier-px 0.45
```

2026-06-16 smoke 결과에서 `huber_left_reprojection_geometry_sim3`는 validation
RMSE `1.563m`, 1852m target max `10.015m`였고,
`target_weighted_huber_left_reprojection_geometry_sim3_sigma160`은 validation RMSE
`1.553m`, 1852m target max `9.964m`였다. target-weighted variant는 PASS 범위지만
기존 `target_weighted_huber_left_disparity_camera_geometry_sim3_sigma160`
(`1.530m`, target max `9.797m`) 및 현재 runtime 추천보다 약하다. 따라서
reprojection 후보는 reference-based 탐색 후보로 보존하되 promotion하지 않는다.

추가 탐색으로 같은 left-disparity geometry 출력에 raw-forward quadratic residual LUT를
붙인 variant도 넣었다. 목적은 left-center 재삼각측량 후 남는 장거리 range residual을
raw_forward_m 기반으로 줄이는 것이다. 입력/출력/금지 입력 contract는 위 후보와 같고,
runtime 입력에 `raw_forward_m` residual key만 추가된다. 단일 bbox stress smoke
(`Simulation/results/rtk_stereo_left_geometry_residual_smoke`)에서는 target-weighted
variant가 31위, validation RMSE `1.529m`, target max `9.807m`로 원 left-disparity
geometry보다 좋아지지 않았다. 이 조합은 PASS 범위에는 있으나 현재 추천 후보가 아니다.

outlier가 섞인 bbox/disparity 조건에는 reference-based robust point-set registration
후보도 포함한다. `truncated_ls_sim3`와 `truncated_ls_range_weighted_sim3`는
Umeyama Sim(3) refit에 truncated least-squares inlier gate를 붙인 후보이며,
TEASER/robust registration 계열의 truncated least-squares 관점과 장거리 stereo에서
disparity error보다 depth/range error를 직접 다루는 관점을 반영한다. 관련 참고는
Yang et al.의 TEASER robust registration 계열, Ling and Shen의 stereo extrinsic
calibration, Chuah et al.의 long-range stereo bias/depth loss 문제정의다.

```bash
Simulation/.venv/bin/python Simulation/scripts/rtk_stereo_calibration_pipeline.py \
  --output-dir Simulation/results/rtk_stereo_tls_candidate_smoke \
  --seed 20260613 \
  --sample-count 160 \
  --target-range-m 1852 \
  --target-window-m 30 \
  --synthetic-target-oversample-count 12 \
  --bbox-outlier-rate 0.08 \
  --bbox-outlier-px 2.5 \
  --disparity-outlier-rate 0.06 \
  --disparity-outlier-px 0.45
```

현재 bbox outlier smoke에서 `truncated_ls_sim3`는 validation range RMSE `1.115m`,
1852m target max abs error `2.907m`, inlier fraction `0.922`를 기록했다.
`truncated_ls_range_weighted_sim3`는 validation range RMSE `1.387m`, target max
`2.620m`이다. 둘 다 Sim(3) 계열 robust baseline으로 유효하지만, 같은 smoke의 최종
추천은 validation RMSE `0.819m`, target abs mean `0.027m`인
`ridge_global_inverse_depth_linear_lambda1p0`였으므로 현재 전체 추천은 바꾸지 않는다.

6개 scenario single-seed focused sweep에서는 `truncated_ls` 후보 row 14개가 모두
1852m target max 10m 조건을 통과했다. validation range RMSE mean/max는
`1.374/2.057m`, target max abs worst-case는 `9.912m`, 최소 RMSE 개선은 `0.231m`였다.
따라서 TLS 계열은 reference-based robust Sim(3) fallback 후보로 남기되, 현재
best/recommended family인 inverse-depth 및 residual-disparity 계열을 대체하지 않는다.
요약 산출물은
`Simulation/results/rtk_stereo_tls_candidate_focused_sweep/truncated_ls_candidate_summary/`에
저장된다.

여러 방법 계열을 한 번에 비교하려면 family-level 요약을 사용한다. 이 스크립트는
기존 `method_comparison.csv`만 읽고 재학습하지 않으며, 각 family의 target-window
PASS 비율, validation RMSE, 1852m target worst-case, runtime profile family를
기록한다. family ranking은 absolute pass count가 아니라 all-pass 여부, pass fraction,
target worst-case, validation RMSE 순서로 정렬한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/summarize_method_families_across_sweep.py \
  --sweep-dir Simulation/results/rtk_stereo_tls_candidate_focused_sweep \
  --output-dir Simulation/results/rtk_stereo_tls_candidate_focused_sweep/method_family_summary \
  --target-error-limit-m 10

Simulation/.venv/bin/python Simulation/scripts/summarize_method_families_across_sweep.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/method_family_summary \
  --target-error-limit-m 10
```

현재 TLS focused sweep family 요약에서는 모든 affine row를 그대로 평가해도
`affine_runtime`이 224/224 all-pass, validation RMSE mean/max `1.124/1.978m`,
1852m target worst-case `9.469m`로 가장 보수적인 family 후보였다. 대표 profile
`affine_binned_residual_raw_forward_bins11`은 StereoDistance promotion check도 PASS했고
runtime truth를 사용하지 않았다. 메인 monotone inverse-depth runtime sweep에서 모든
affine row를 그대로 세면 631/640 pass, target worst-case `11.210m`로 all-pass는
아니다. 그러나 scenario/seed별로 affine family 내부 최선 후보를 offline calibration
selector가 고를 수 있다고 보면 `affine_runtime`은 20/20 all-pass, target worst-case
`9.308m`다. 이는 affine RDF axis-wise gain/bias + residual LUT 계열이 다음 자동 탐색
방향이라는 뜻이다. 반대로 family row 전체 기준에서는 `residual_disparity`가
1107/1120 pass, pass fraction `0.988`, target worst-case `11.957m`로 1위라서
단일 family 전체를 무조건 promote할 수는 없다. 따라서 family-level 탐색은 다음 후보
방향을 찾는 진단이며, 현재 full policy catalog runtime 추천
`policy_catalog+robust_family_fallback`을 대체하지 않는다.

affine family 내부에서 실제 runtime profile을 자동 선택하고 export하려면 다음 audit를
사용한다. 이 단계는 calibration-time validation metric으로 family 내부 best profile을
고르지만, export된 JSON은 일반 StereoDistance runtime profile이며 truth를 입력으로
사용하지 않는다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_method_family_profile_selection.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/affine_family_profile_selection_audit \
  --family-name affine_runtime \
  --method-substring affine \
  --target-error-limit-m 10 \
  --export-selected-profiles
```

현재 결과는 20/20 selected PASS, validation RMSE mean/max `1.307/2.114m`,
1852m target max abs worst-case `9.308m`이다. `selected_profiles/` 아래 export된
20개 affine runtime profile은 모두 `stereodistance_profile_promotion_check.py`를 통과했고,
schema load, 실제 StereoDistance node loader, validation sample truth-free apply가 모두
정상이다. dashboard는 이를 `affine_family_profile_selection` 행으로 기록한다.

이 affine family profile들을 runtime에서 truth 없이 고르려면 raw RDF/disparity feature
selector audit를 실행한다. 이 selector의 입력은 보정/운용 샘플의 raw stereo-center
RDF(`right,down,forward`)와 disparity 통계뿐이며, replay truth는 offline audit metric에만
사용한다. `robust_bbox_fallback` 정책은 raw feature상 nominal/yaw/bbox 저거리 계열에
가까운 경우 bbox-outlier source profile을 보수적으로 우선 검토한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_method_family_runtime_feature_selector.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --family-selection-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/affine_family_profile_selection_audit/method_family_profile_selected.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/affine_family_runtime_feature_selector_audit_robust_bbox \
  --selection-policy robust_bbox_fallback \
  --feature-subset all \
  --target-error-limit-m 10
```

현재 nearest/feature-vote 방식은 19/20 PASS였고 bbox-outlier seed 하나에서
1852m target max error가 10m를 넘었다. `robust_bbox_fallback`은 20/20 PASS,
validation RMSE mean/max `1.366/2.648m`, 1852m target max abs worst-case
`9.423m`였다. 이는 affine family가 truth-free runtime selector로도 유망하다는
증거지만, 아직 full policy catalog의 `8.951m` worst-case보다 크므로 현재 배포 추천은
`policy_catalog+robust_family_fallback`을 유지하고, affine runtime selector는 다음
교체 후보로 추적한다. dashboard는 이를 `affine_family_runtime_feature_selector`
행으로 기록한다.

policy catalog selector와 affine runtime selector를 함께 두고, raw feature threshold로
둘 중 하나를 고르는 portfolio audit도 실행할 수 있다. threshold 규칙은 offline replay
metric으로 학습하지만, leave-one-run-out 평가에서 held-out run 선택 입력은
`feature_*` raw RDF/disparity 통계뿐이다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_runtime_selector_portfolio.py \
  --candidate policy_catalog=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_audit_policy_catalog_robust_family/runtime_measurement_guard_audit.csv \
  --candidate affine_runtime=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/affine_family_runtime_feature_selector_audit_robust_bbox/runtime_feature_selector_audit.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_selector_portfolio_policy_vs_affine \
  --target-error-limit-m 10 \
  --objective target_max_then_rmse \
  --feature-names raw_forward_mean_m,measured_disparity_mean_px,disparity_forward_product_ratio_p10_to_nominal,right_over_forward_std,down_over_forward_std
```

현재 LOO 결과는 18/18 PASS, 1852m target max abs worst-case `8.951m`로 policy
catalog와 동일하고, validation RMSE mean/max는 `1.154/1.806m`에서
`1.150/1.758m`로 소폭 개선된다. 따라서 이는 최종 추천 교체가 아니라 policy catalog를
유지하면서 affine family를 보조 후보로 자동 탐색하는 진단 행이다. 전체 25개 feature를
무조건 쓰면 LOO RMSE가 악화되어 과적합 신호가 나왔으므로, 위 명령처럼 저차원 raw
geometry/product-ratio feature subset을 명시한다. dashboard는 이를
`runtime_selector_portfolio_policy_vs_affine` 행으로 기록한다.

portfolio audit은 후보 selector 사이의 offline oracle gap도 함께 기록한다. 기본
`target_max_then_rmse` 기준에서는 policy/affine 2후보 LOO가 oracle과 8/18 일치했고,
oracle match fraction `0.444`, max RMSE regret `0.064m`, target max regret
`1.688m`였다. 이 objective의 oracle은 target max를 우선하므로 RMSE regret 평균은
음수가 될 수 있다. target-cap 기준의 3후보 진단에서는 oracle과 11/18 일치,
RMSE regret mean/max `0.034/0.337m`로 affine이 일부 row에서 더 낮은 RMSE를 내지만,
LOO에서 안정적으로 잡히지 않았다. dashboard는 이 gap을
`runtime_selector_portfolio_oracle_gap` 행으로 기록한다.

같은 audit은 oracle 후보 label을 raw feature threshold 하나로 분리할 수 있는지도
`runtime_selector_portfolio_oracle_separability.csv/txt`에 남긴다. 이 진단의 목적은
“어떤 regime에서 affine family로 바꾸면 이득인지”를 찾는 것이며, oracle label은
calibration/replay truth로만 계산되고 runtime 입력으로 쓰지 않는다. 2026-06-16
재실행 결과에서 2후보 policy/affine 경로의 최고 분리 feature는
`feature_measured_disparity_mean_px`, accuracy `0.778`, effect size `1.159`였다.
3후보 target-cap 경로에서는 최고 feature가 `feature_down_over_forward_std`,
accuracy `0.667`, effect size `0.369`였다. 따라서 affine override 신호는 존재하지만
현재 raw feature threshold만으로는 충분히 안정적이지 않으므로, 최종 추천은 여전히
`policy_catalog+robust_family_fallback`이다. dashboard는 이 추가 진단을
`runtime_selector_portfolio_oracle_separability` 행으로 기록한다.

추가로 high-precision affine override도 감사한다. 이 모드는 기본 후보
`policy_catalog`를 유지하다가, offline separability 진단에서 false positive가 0인
raw feature threshold를 만족할 때만 `affine_runtime`으로 바꾼다. runtime 입력은
해당 raw feature threshold뿐이고 oracle label은 calibration/replay 진단에만 쓰인다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_runtime_selector_portfolio.py \
  --candidate policy_catalog=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_audit_policy_catalog_robust_family/runtime_measurement_guard_audit.csv \
  --candidate affine_runtime=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/affine_family_runtime_feature_selector_audit_robust_bbox/runtime_feature_selector_audit.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_selector_portfolio_policy_vs_affine \
  --target-error-limit-m 10 \
  --objective target_max_then_rmse \
  --feature-names raw_forward_mean_m,measured_disparity_mean_px,disparity_forward_product_ratio_p10_to_nominal,right_over_forward_std,down_over_forward_std \
  --precision-override-label affine_runtime \
  --precision-override-baseline-label policy_catalog
```

2026-06-16 결과에서 zero-false-positive rule은
`feature_disparity_forward_product_ratio_p10_to_nominal <= 1.0018866404242965`였고
5/18 row만 affine으로 override했다. 18/18 PASS와 1852m target max worst-case
`8.951m`는 유지했지만, validation RMSE mean이 `1.150m`에서 `1.211m`로 악화됐다.
target-cap 기준의 별도 run도 18/18 PASS였지만 RMSE mean `1.154m`로 개선은 없었다.
따라서 conservative override는 안전성 감사 기능으로 보존하되, 현재 추천을 대체하지
않는다. dashboard는 이를 `runtime_selector_portfolio_precision_override` 행으로
기록한다.

추가로 reference 기반 robust regression 후보로 Theil-Sen inverse-depth 계열을
검증한다. Theil-Sen은 pairwise slope의 median으로 1D 선형 관계를 잡는 강건
추정이므로, bbox center outlier 또는 disparity spike가 섞인 보정 비행에서
`1/raw_forward -> 1/truth_forward` 계수가 least-squares보다 안정적인지 확인한다.
보정 truth는 calibration-time fitting과 validation metric에만 사용하며, runtime
profile은 raw stereo-center RDF와 measured disparity, 고정 계수만 사용한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_robustness_sweep.py \
  --output-dir Simulation/results/rtk_stereo_theil_sen_inverse_depth_probe \
  --scenarios bbox_outlier_stress \
  --seeds 20260613,20260614,20260615 \
  --sample-count 520 \
  --target-range-m 1852 \
  --target-window-m 30 \
  --synthetic-target-oversample-count 12 \
  --pooled-candidate-scan-limit 80
```

2026-06-16 bbox outlier focused sweep에서
`target_weighted_theil_sen_global_inverse_depth_linear`는 3/3 seed strict PASS,
validation RMSE mean `1.229m`, 1852m target abs max `1.974m`였다. 기존
`target_weighted_global_inverse_depth_linear`의 validation RMSE mean `1.242m`,
target abs max `2.121m`보다 좋아져, bbox outlier 조건에서는 Theil-Sen robust
inverse-depth fitting이 유효한 후보임을 확인했다. 다만 같은 sweep의 추천은
`huber_joint_disparity_ray_rotation_nls`였고, 그 validation RMSE mean은 `1.223m`,
target abs max `1.990m`였다. pooled replay에서는
`target_weighted_theil_sen_global_inverse_depth_linear`가 3/3 PASS,
validation RMSE mean/max `1.229/1.538m`, target abs max `2.044m`였지만,
pooled validation top인 `sim3_binned_residual_disparity_bins11`은 3/3 PASS,
validation RMSE mean/max `1.209/1.517m`, target abs max `2.066m`였다.

따라서 Theil-Sen 계열은 bbox/disparity outlier가 의심되는 보정 비행에서
저복잡도 inverse-depth fallback 후보로 보존한다. 그러나 validation RMSE와
cross-seed replay 기준에서 현 최종 추천인 `policy_catalog+robust_family_fallback`을
대체하지는 않는다. PASS 기준은 raw 대비 validation RMSE 개선, 1852m target-window
존재, 1852m target abs/max 10m 이하, axis-wise regression 없음, train/validation
gap과 model complexity를 함께 보는 기존 gate와 동일하다.

Theil-Sen 후보를 전체 6개 stress family에 넣은 full-family probe도 실행한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_robustness_sweep.py \
  --output-dir Simulation/results/rtk_stereo_theil_sen_full_family_probe \
  --scenarios nominal,yaw_pitch_stress,disparity_noise_stress,bbox_outlier_stress,intrinsic_scale_stress,combined_sensor_stress \
  --seeds 20260613,20260614,20260615 \
  --sample-count 520 \
  --target-range-m 1852 \
  --target-window-m 30 \
  --synthetic-target-oversample-count 12 \
  --pooled-candidate-scan-limit 100
```

2026-06-16 결과에서 cross-scenario method stability gate의 robust recommendation은
`target_weighted_theil_sen_global_inverse_depth_linear`였다. 6/6 scenario strict
PASS, validation RMSE mean/max `1.071/1.507m`, 1852m target abs mean/max
`0.927/2.126m`, model complexity mean `5`로, 저복잡도 StereoDistance 적용 후보로
가장 안정적이었다. Scenario별 추천은 nominal과 intrinsic-scale에서
`target_weighted_theil_sen_global_inverse_depth_linear`, yaw/pitch에서
`theil_sen_global_inverse_depth_linear`, disparity-noise에서
`target_shrunk_huber_global_inverse_depth_linear_shrink040`, bbox-outlier에서
`huber_joint_disparity_ray_rotation_nls`, combined-stress에서
`huber_global_inverse_depth_linear`였다.

초기 full-family summary에서는 pooled profile이 transfer replay 18/18 PASS였어도
promotion check에서 pooled validation sample 적용 실패로 표시됐다. 원인은
`stereodistance_profile_promotion_check.py`가 left-camera geometry profile에 필요한
runtime 입력 `left_u_px`, `left_v_px`를 sample payload에 전달하지 않은 checker
버그였다. checker 수정 후 같은 pooled profile
`target_weighted_huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_sigma160`
을 재검증한 결과, promotion check는 PASS, schema/node loader OK,
sample apply `50`, invalid `0`이었다. corrected evidence는
`Simulation/results/rtk_stereo_theil_sen_full_family_probe/pooled_scenario_profile/promotion_check_corrected/`
에 저장한다.

따라서 배포 후보는 두 갈래로 둔다. 첫째, scenario-specific catalog profile 6개는
promotion check 6/6 PASS였고, minimal set은 두 source profile만으로 6개 target
scenario를 덮었다:
`bbox_outlier_stress/huber_joint_disparity_ray_rotation_nls`가 bbox, combined,
disparity-noise, nominal, yaw/pitch를 덮고,
`intrinsic_scale_stress/target_weighted_theil_sen_global_inverse_depth_linear`가
intrinsic-scale을 덮는다. 이 선택은 offline 보정/검증 결과이며, runtime에서는
PX4/RTK truth를 쓰지 않고 배포 메타데이터나 운용자 scenario label로 profile을
고른다. 둘째, pooled profile은 단일 universal runtime profile 후보로 다시 유효하다.
평균 RMSE 기준 1등은
`target_weighted_huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_sigma160`이고
transfer replay 18/18 PASS, validation RMSE mean/max `1.433/2.498m`, 1852m
target abs max `3.911m`, corrected promotion check PASS다. 하지만 단일 배포
fallback은 평균 RMSE보다 1852m worst-case와 validation worst-case를 우선해야 하므로,
`target_max_then_rmse_max` 기준으로
`huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut`를 선택한다.
이 후보는 transfer replay 18/18 PASS, validation RMSE mean/max `1.437/2.475m`,
1852m target abs max `3.839m`, corrected promotion check PASS다. 다만 scenario
catalog는 조건별 RMSE와 target-window 여유가 더 좋을 수 있으므로, 최종 배포 선택은
catalog 방식과 pooled 방식의 validation worst-case, 1852m worst-case, operator가
scenario label을 안정적으로 제공할 수 있는지를 함께 본다.

2026-06-16 이후 `run_rtk_stereo_robustness_sweep.py`의 pooled universal profile
자동 선택 기준도 이 목적에 맞춰 수정했다. 후보가 모든 transfer replay를 통과해야
하고, 그 다음에는 1852m target abs worst-case, validation range RMSE worst-case,
validation range RMSE mean, model complexity 순서로 정렬한다. 따라서 앞으로 생성되는
`pooled_scenario_profile` summary는 단순 평균 RMSE 1등이 아니라 단일 StereoDistance
fallback으로 더 안전한 worst-case 중심 후보를 기본 selected profile로 기록한다.
빠른 검증이나 pooled/profile-policy 목적의 반복 실행에서는 `--skip-near-tie-transfer`를
사용할 수 있다. 이 옵션은 scenario near-tie 후보 목록은 보존하되 cross-scenario
near-tie replay와 best near-tie promotion만 건너뛰며, 좌표/런타임 계약과 pooled,
policy, catalog 검증은 그대로 수행한다.

`Simulation/results/rtk_stereo_worstcase_pooled_objective_smoke` smoke EXP에서는
`--skip-near-tie-transfer --pooled-candidate-scan-limit 20`으로 새 기준을 확인했다.
선택된 pooled universal profile은
`target_weighted_huber_left_disparity_camera_geometry_sim3_sigma160`이고, transfer
replay 18/18 PASS, validation RMSE mean/max `1.446/2.527m`, 1852m target abs
max `4.024m`, promotion check PASS, sample invalid `0`, runtime truth 사용
`false`였다. 대표 policy profile들은 intrinsic-scale에서 15/18 이하로 실패했고,
deployment recommendation은 `single_pooled_runtime_profile`이었다.

이후 `Simulation/results/rtk_stereo_left_geom_sigma_grid_smoke`에서는 같은
left-disparity camera-geometry family의 target weighting 폭을 `80/160/250/400m`
grid로 넓혀 자동 탐색했다. `--skip-near-tie-transfer --pooled-candidate-scan-limit 24`
조건에서 새로 선택된 pooled universal profile은
`target_weighted_huber_left_disparity_camera_geometry_sim3_sigma400`이다. 이 후보는
transfer replay 18/18 PASS, validation RMSE mean/max `1.444/2.521m`,
1852m target abs max `4.011m`, promotion check PASS, sample invalid `0`,
runtime truth 사용 `false`였다. 같은 family의 sigma250은 mean/max
`1.444/2.522m`, target abs max `4.013m`, sigma160은 mean/max
`1.446/2.527m`, target abs max `4.024m`였으므로 sigma400이 현 단일 pooled
fallback 후보 중 가장 낮은 worst-case 기준을 기록한다. 입력 계약은
calibration-time PX4/RTK truth로 fit/validation만 수행하고 runtime에서는
left bbox center, measured disparity, raw RDF만 사용하는 StereoDistance 적용
가능 profile이다.

`Simulation/results/rtk_stereo_left_geom_sigma_wide_smoke`에서는 같은 탐색을
sigma600/800까지 확장했다. sigma600과 sigma800도 transfer replay 18/18 PASS였지만
각각 validation RMSE mean/max `1.446/2.528m`, `1.448/2.534m`, 1852m target abs
max `4.025m`, `4.038m`로 sigma400보다 나빴다. 따라서 현재 evidence에서는
target weighting 폭을 sigma400보다 더 넓히는 것은 이득이 없고, sigma400이
단일 pooled fallback의 현 기준점이다.

`Simulation/results/rtk_stereo_left_geom_residual_shrink_smoke`에서는 sigma400
left-disparity geometry profile 위에 raw-forward residual LUT를 약하게 적용하는
shrink 후보를 추가했다. `--pooled-candidate-scan-limit 30` 조건에서 선택된
profile은
`target_weighted_huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_sigma400_shrink050`이다.
이 후보는 transfer replay 18/18 PASS, validation RMSE mean/max `1.436/2.509m`,
1852m target abs max `3.961m`, promotion check PASS, sample invalid `0`,
runtime truth 사용 `false`였다. shrink025도 18/18 PASS였지만 mean/max
`1.439/2.515m`, target abs max `3.986m`로 shrink050보다 약했다. 따라서 현 단일
pooled fallback 3-seed 기준점은 sigma400 단독에서 sigma400 + 50% raw-forward
residual shrink 조합으로 갱신했다. 이 조합도 runtime에서는 left bbox center,
measured disparity, raw RDF forward, calibration-fixed parameters만 사용한다.

`Simulation/results/rtk_stereo_left_geom_residual_shrink_seed5_smoke`에서는 같은
pooling/replay 조건을 seed 5개(`20260611`-`20260615`)로 확장했다. 새 selected
single pooled profile은
`huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut`이며,
transfer replay 30/30 PASS, validation RMSE mean/max `1.403/2.468m`, 1852m
target abs mean/max `1.621/4.888m`, promotion check PASS, sample invalid `0`,
runtime truth 사용 `false`였다. 같은 5-seed run에서 sigma400 기본형은 30/30 PASS,
validation RMSE mean/max `1.409/2.514m`, target abs max `5.050m`였고,
sigma250/160/600/800도 모두 30/30 PASS였지만 target abs max가 약 `5.05m` 이상으로
더 컸다. 반대로 target-weighted inverse-depth, Theil-Sen, joint disparity/ray NLS
계열은 pooled target error가 작거나 combined-sensor 성능이 좋을 수 있었지만 대부분
25/30 transfer replay에 그쳐 단일 fallback 조건을 통과하지 못했다. 따라서 5-seed
기준 단일 StereoDistance fallback은 residual LUT가 포함된 non-target-weighted
left-disparity camera-geometry profile로 갱신한다. 이 결과는 1852m ±10m 조건에는
충분히 들어오지만, 단일 profile만으로 모든 stress family의 target abs max를 full
scenario catalog 수준까지 낮추지는 못한다.

`Simulation/results/rtk_stereo_left_geom_unweighted_residual_shrink_smoke`에서는
5-seed selected family인 non-target-weighted left-disparity camera-geometry residual
LUT에도 shrink025/050/075 후보를 추가해 3-seed smoke로 비교했다. 새 selected single
pooled profile은
`huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_shrink075`이며,
transfer replay 18/18 PASS, validation RMSE mean/max `1.438/2.492m`, 1852m
target abs mean/max `1.583/3.896m`, promotion check PASS, sample invalid `0`,
runtime truth 사용 `false`였다. 같은 run에서 unweighted shrink050은 18/18 PASS,
validation RMSE mean/max `1.440/2.509m`, target abs max `3.952m`였고,
target-weighted sigma400 shrink050은 18/18 PASS, validation RMSE mean/max
`1.436/2.509m`, target abs max `3.961m`였다. 따라서 3-seed 기준에서는
unweighted residual LUT에 75% shrink를 적용한 profile이 1852m worst-case 기준을
가장 낮췄다. 다만 3-seed 결과만으로는 profile 안정성을 확정하지 않고 5-seed 확장
검증 대상으로 둔다.

`Simulation/results/rtk_stereo_left_geom_unweighted_residual_shrink_seed5_smoke`에서는
같은 unweighted residual shrink 후보를 seed 5개(`20260611`-`20260615`)로 확장했다.
선택된 single pooled fallback은 다시
`huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut`이며,
transfer replay 30/30 PASS, validation RMSE mean/max `1.403/2.468m`, 1852m
target abs mean/max `1.621/4.888m`, promotion check PASS, sample invalid `0`,
runtime truth 사용 `false`였다. 3-seed에서 가장 좋았던 shrink075도 30/30 PASS였지만
validation RMSE mean/max `1.404/2.484m`, 1852m target abs mean/max
`1.603/4.940m`로 worst-case validation과 target error가 모두 unshrunk profile보다
약간 나빴다. 따라서 5-seed 기준 단일 StereoDistance fallback은 unweighted residual
LUT를 shrink하지 않는 profile로 유지한다. 이 비교는 calibration-time PX4/RTK NED
truth를 stereo-center RDF truth로 변환해 fit/validation에만 사용하고, runtime에서는
left bbox center, measured disparity, raw RDF forward, calibration-fixed parameters만
사용한다.

`Simulation/results/rtk_stereo_all_pooled_candidates_seed5_probe`에서는 pooled candidate
scan limit을 `0`으로 풀어 66개 pooled 후보 전체를 seed 5개, 6개 stress scenario,
30개 transfer replay 조건에서 다시 비교했다. 12개 후보만 30/30 transfer replay를
통과했고, selected single pooled fallback은 여전히
`huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut`였다.
성능은 validation RMSE mean/max `1.403/2.468m`, 1852m target abs mean/max
`1.621/4.888m`, promotion check PASS, sample invalid `0`, runtime truth 사용
`false`였다. 가장 가까운 target-weighted residual LUT 변형은 30/30 PASS였지만
validation max 또는 1852m target worst-case가 더 컸고, inverse-depth/Sim(3)
residual 계열은 장거리 평균 오차나 combined-sensor 성능이 좋더라도 intrinsic-scale
transfer replay에서 raw보다 악화되어 pass-all 단일 fallback 조건을 통과하지 못했다.
따라서 단일 StereoDistance fallback은 unshrunk left-disparity camera-geometry
residual LUT profile로 유지한다. 동시에 이 결과는 best single profile의 문제가
보정식 형태 자체보다 error-regime 식별과 guard/select policy에 있음을 보여준다.

`Simulation/results/rtk_stereo_runtime_guard_all_catalog_intrinsic_gap_seed5_probe`에서는
같은 5-seed sweep의 full scenario catalog를 대상으로 runtime-measurement guard를
검증했다. 선택 입력은 raw stereo-center RDF(`raw_right/down/forward/range`),
measured disparity, left image center(`left_u/v`) 통계뿐이며, PX4/RTK truth와
validation error는 offline audit에만 사용한다. all-catalog 조건에서
`intrinsic_scale_gap_transfer_risk` policy는 30/30 PASS, validation RMSE mean/max
`1.013/1.670m`, 1852m target abs mean/max `0.936/2.340m`, target max abs max
`9.609m`였다. 같은 조건의 `centroid_knn_veto_transfer_risk`도 동일한 30/30 PASS
요약을 냈다. 단순 nearest-centroid guard
(`Simulation/results/rtk_stereo_runtime_guard_all_catalog_nearest_centroid_seed5_probe`)도
30/30 PASS였지만 validation RMSE mean `1.024m`로 transfer-risk 계열보다 약간 높았다.
이는 단일 pooled fallback보다 validation worst-case와 1852m target worst-case가 모두
낮고, full scenario catalog의 장점을 runtime-like raw feature selection으로 일부
회수한다.

단, `Simulation/results/rtk_stereo_runtime_guard_all_catalog_intrinsic_gap_loso_seed5_probe`
처럼 target scenario family 전체를 feature library에서 제외한 strict LOSO audit에서는
25/30 PASS, validation RMSE mean/max `1.498/2.735m`, 1852m target abs mean/max
`1.778/5.132m`로 떨어졌다. Minimal-set guard
(`Simulation/results/rtk_stereo_runtime_guard_minimal_intrinsic_gap_seed5_probe`)도
24/30 PASS에 그쳤다. 따라서 현재 guard는 알려진 deployment/error regime 사이의
truth-free profile selector로는 유망하지만, 보지 못한 새 regime까지 일반화하는
검출기라고 보기는 어렵다. 실제 배포에서는 operator/deployment metadata 또는 명시적
scenario label이 있으면 full catalog를 우선하고, metadata가 없으면 guard selector를
보조 후보로 두되 unknown regime fallback은 single pooled profile로 유지한다.

Oracle-regret audit도 같은 결론을 지지한다.
`Simulation/results/rtk_stereo_runtime_guard_all_catalog_intrinsic_gap_oracle_regret_seed5_probe`
에서 all-catalog intrinsic-gap guard는 offline oracle과 14/30 일치, selected/oracle
모두 30/30 PASS, target abs regret mean/max `0.040/0.387m`였다.
`Simulation/results/rtk_stereo_runtime_guard_all_catalog_nearest_centroid_oracle_regret_seed5_probe`
에서 nearest-centroid guard는 oracle 일치 20/30, selected/oracle 모두 30/30 PASS,
target abs regret mean/max `0.043/0.387m`였다. 즉 full catalog를 유지할 수 있으면
간단한 raw-feature selector도 offline oracle에 매우 가까우며, 정책 선택은 RMSE 평균을
조금 더 낮출지 또는 oracle replay score 일치를 높일지의 tradeoff로 본다.
`Simulation/results/rtk_stereo_runtime_guard_threshold_sweep_all_catalog_seed5_probe`의
transfer-risk threshold sweep도 기본 low-risk 기준 `RMSE 1.3m / target 2.5m`를 다시
추천했다. 반면
`Simulation/results/rtk_stereo_runtime_guard_threshold_sweep_all_catalog_loso_seed5_probe`
는 best threshold에서도 25/30 PASS에 머물러, strict LOSO 실패가 threshold 튜닝 문제가
아니라 unseen error-regime 일반화 문제임을 확인했다.
`Simulation/results/rtk_stereo_gap_feature_separability_loso_intrinsic_combined_seed5_probe`
에서는 strict LOSO audit row의 raw feature만으로 gap family separability를 점검했다.
combined-sensor는 `down_over_forward_mean <= 0.00641`, intrinsic-scale은
`raw_forward_mean >= 1145.11m` 또는
`disparity_forward_product_ratio_p10_to_nominal >= 1.00374` 같은 단일 feature
threshold로 30-row audit에서 accuracy `1.0`을 보였다. 이는 guard 개선 후보가
있음을 뜻하지만, threshold 자체는 현재 synthetic sweep에서 얻은 offline diagnostic이므로
실제 배포 전에는 별도 비행/환경 데이터로 재검증해야 한다.
`Simulation/results/rtk_stereo_runtime_selector_portfolio_transfer_vs_nearest_limited_seed5_probe`
에서는 transfer-risk guard와 nearest-centroid guard를 4개 핵심 raw feature만으로
depth-2 portfolio rule로 조합했다. 결과는 30/30 PASS, validation RMSE mean/max
`1.013/1.670m`, target max abs max `9.609m`로 transfer-risk guard와 같았고,
추가 이득은 확인되지 않았다. 따라서 현재는 full catalog + simple raw-feature guard를
우선하고, portfolio는 더 다양한 후보가 생긴 뒤 다시 평가한다.

`Simulation/results/rtk_stereo_single_pooled_runtime_candidate_seed5_probe`에서는
all-catalog guard audit의 raw feature 열을 그대로 재사용하고, 단일 pooled fallback
profile을 각 calibration CSV에 offline replay해 runtime-selector 호환 candidate CSV를
만들었다. 이 변환은 selector 입력으로 raw stereo-center RDF, measured disparity,
image-center 통계만 복사하고, PX4/RTK truth는 replay metric 계산에만 사용한다. 생성된
`single_pooled_runtime_candidate.csv`는 30/30 PASS, validation RMSE mean/max
`1.403/2.468m`, target max abs max `8.388m`였다. 실행 예:

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python \
  Simulation/scripts/build_single_profile_runtime_audit_candidate.py \
  --guard-audit-csv Simulation/results/rtk_stereo_runtime_guard_all_catalog_intrinsic_gap_seed5_probe/runtime_measurement_guard_audit.csv \
  --profile-json Simulation/results/rtk_stereo_all_pooled_candidates_seed5_probe/pooled_scenario_profile/pipeline/runtime_profiles/huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut.json \
  --output-csv Simulation/results/rtk_stereo_single_pooled_runtime_candidate_seed5_probe/single_pooled_runtime_candidate.csv \
  --label single_pooled \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

`Simulation/results/rtk_stereo_runtime_selector_portfolio_catalog_vs_single_pooled_seed5_probe`
에서는 full catalog guard와 단일 pooled fallback을 같은 runtime feature portfolio에서
비교했다. 4개 핵심 feature depth-2 rule은 30/30 PASS, validation RMSE mean/max
`1.025/1.670m`, target max abs max `9.609m`로 pass 조건은 만족했지만, 단일 pooled가
더 유리한 일부 seed13 행을 안정적으로 분리하지 못했다. 확장 feature 후보를 넣은
`rtk_stereo_runtime_selector_portfolio_catalog_vs_single_pooled_extended_seed5_probe`도
target max를 낮추지는 못했다. 반면
`rtk_stereo_runtime_selector_portfolio_catalog_vs_single_pooled_single_override_seed5_probe`
의 zero-false-positive override audit은 기본 후보를 full catalog guard로 두고,
`feature_left_v_std_px >= 129.7294559911429`일 때만 단일 pooled fallback을 선택했다.
이 conservative override는 30/30 PASS, validation RMSE mean/max `1.018/1.670m`,
target max abs max `8.388m`로 all-catalog guard의 `9.609m` worst-case를 낮췄다.
다만 override true-positive는 1/30뿐이며, threshold는 synthetic 5-seed replay에서 얻은
offline diagnostic이다. 따라서 현재 추천은 “known scenario catalog가 있으면 catalog
guard를 기본으로 쓰고, raw image-center variance가 zero-FP 조건을 만족하는 경우에만
single pooled fallback으로 보수 전환”이다. 실제 본실험에서는 이 threshold를 고정하기
전에 별도 비행 replay 또는 hardware/WIRIS 데이터로 false-positive 여부를 다시 확인해야
한다.

Strict LOSO 조건에서도 같은 변환을 적용했다.
`Simulation/results/rtk_stereo_single_pooled_runtime_candidate_loso_seed5_probe`의 단일
pooled candidate는 30/30 PASS, target max abs max `8.388m`였다.
`Simulation/results/rtk_stereo_runtime_selector_portfolio_loso_vs_single_pooled_seed5_probe`
의 depth-2 portfolio는 28/30 PASS에 그쳐 pass-all을 회복하지 못했다.
`Simulation/results/rtk_stereo_runtime_selector_portfolio_loso_vs_single_pooled_single_override_seed5_probe`
의 conservative override는 30/30 PASS를 회복했지만 target max abs max가
`9.733m`였다. 따라서 source family가 빠진 unknown-regime 조건에서는 raw-feature
threshold로 LOSO guard를 억지 조합하기보다, 단일 pooled fallback을 그대로 쓰는 편이
현재 synthetic 5-seed replay 기준으로 더 단순하고 worst-case가 낮다.

위 결과를 배포 모드별 추천으로 묶으려면 다음을 실행한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python \
  Simulation/scripts/summarize_runtime_deployment_strategy.py \
  --known-catalog-summary-json Simulation/results/rtk_stereo_runtime_guard_all_catalog_intrinsic_gap_seed5_probe/runtime_measurement_guard_summary.json \
  --known-override-summary-json Simulation/results/rtk_stereo_runtime_selector_portfolio_catalog_vs_single_pooled_single_override_seed5_probe/runtime_selector_portfolio_precision_override_summary.json \
  --single-pooled-candidate-csv Simulation/results/rtk_stereo_single_pooled_runtime_candidate_seed5_probe/single_pooled_runtime_candidate.csv \
  --loso-override-summary-json Simulation/results/rtk_stereo_runtime_selector_portfolio_loso_vs_single_pooled_single_override_seed5_probe/runtime_selector_portfolio_precision_override_summary.json \
  --loso-single-pooled-candidate-csv Simulation/results/rtk_stereo_single_pooled_runtime_candidate_loso_seed5_probe/single_pooled_runtime_candidate.csv \
  --output-dir Simulation/results/rtk_stereo_runtime_deployment_strategy_seed5_probe \
  --target-error-limit-m 10
```

산출물은 `runtime_deployment_strategy_candidates.csv`,
`runtime_deployment_strategy_recommendation.json/txt`이다. 5-seed synthetic replay
기준 자동 추천은 known-catalog 모드에서는
`known_catalog_guard_with_single_pooled_override`이고, runtime rule은
`catalog_guard`를 기본으로 쓰되
`feature_left_v_std_px >= 129.7294559911429`이면 `single_pooled`로 전환하는 것이다.
unknown/unseen-regime 모드에서는 `single_pooled_runtime_profile` 단독을 추천한다. 두
모드 모두 pass-all이고 target max abs max는 `8.388m`로 1852m ±10m 조건을 만족한다.

추천 결과를 실제 배포 패키지 형태로 묶으려면 다음을 실행한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/export_runtime_deployment_package.py \
  --strategy-recommendation-json Simulation/results/rtk_stereo_runtime_deployment_strategy_seed5_probe/runtime_deployment_strategy_recommendation.json \
  --single-pooled-profile-json Simulation/results/rtk_stereo_all_pooled_candidates_seed5_probe/pooled_scenario_profile/pipeline/runtime_profiles/huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut.json \
  --scenario-catalog-json Simulation/results/rtk_stereo_all_pooled_candidates_seed5_probe/scenario_profile_catalog.json \
  --transfer-matrix-csv Simulation/results/rtk_stereo_all_pooled_candidates_seed5_probe/scenario_profile_catalog_transfer_matrix.csv \
  --output-dir Simulation/results/rtk_stereo_runtime_deployment_package_seed5_probe \
  --target-range-m 1852 \
  --target-error-limit-m 10
```

패키지 산출물은 `runtime_deployment_package_manifest.json/txt`,
`known_catalog_runtime_selector.json`, `active_runtime_profile_unknown.json`,
`profiles/single_pooled_runtime_profile.json`, `catalogs/scenario_profile_catalog.json`,
`catalogs/scenario_profile_catalog_transfer_matrix.csv`이다. unknown/unseen-regime에서는
`active_runtime_profile_unknown.json`을 StereoDistance active profile로 바로 사용한다.
known-catalog 배포에서는 `known_catalog_runtime_selector.json`의 baseline catalog guard와
override rule을 사용하되, runtime 입력은 raw RDF/시차/left image-center feature로
제한한다. `active_runtime_profile_unknown.json`은 promotion check에서 schema load,
node loader, sample apply 50/50, forbidden runtime input 선언을 모두 통과했다.

패키지 selector가 특정 raw measurement CSV에서 어떤 profile을 고르는지 확인하려면:

```bash
Simulation/.venv/bin/python Simulation/scripts/apply_runtime_deployment_selector.py \
  --selector-json Simulation/results/rtk_stereo_runtime_deployment_package_seed5_probe/known_catalog_runtime_selector.json \
  --sweep-dir Simulation/results/rtk_stereo_all_pooled_candidates_seed5_probe \
  --input-csv Simulation/results/rtk_stereo_all_pooled_candidates_seed5_probe/bbox_outlier_stress/seed_20260613/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_runtime_deployment_package_seed5_probe/selector_apply_bbox_seed20260613_all_rows
```

이 selector 적용 명령의 `--split`은 replay 평가 split이 아니라 feature window를 줄이는
필터다. 기존 guard audit과 같은 조건을 재현하려면 기본값처럼 split 없이 전체 raw
measurement rows를 사용한다. 예를 들어 bbox outlier seed `20260613` 전체 행에서는
`feature_left_v_std_px=129.739`로 override가 켜져 `single_pooled`가 선택되고, nominal
seed `20260611` 전체 행에서는 `feature_left_v_std_px=127.502`로 catalog guard가 유지된다.

패키지 selector가 30개 seed 전체에서 offline precision-override audit을 재현하는지도
검증할 수 있다.

```bash
Simulation/.venv/bin/python Simulation/scripts/apply_runtime_deployment_selector.py \
  --selector-json Simulation/results/rtk_stereo_runtime_deployment_package_seed5_probe/known_catalog_runtime_selector.json \
  --sweep-dir Simulation/results/rtk_stereo_all_pooled_candidates_seed5_probe \
  --output-dir Simulation/results/rtk_stereo_runtime_deployment_package_seed5_probe/selector_sweep_audit_all_rows \
  --audit-all-sweep-seeds \
  --reference-audit-csv Simulation/results/rtk_stereo_runtime_selector_portfolio_catalog_vs_single_pooled_single_override_seed5_probe/runtime_selector_portfolio_precision_override_audit.csv
```

이 audit은 `runtime_deployment_selector_sweep_audit.csv`와
`runtime_deployment_selector_sweep_summary.json/txt`를 저장한다. 5-seed 결과는
selected candidate match `30/30`, override match `30/30`, override count `1`이며,
유일한 override는 `bbox_outlier_stress/seed_20260613`이다. reference audit은 선택 결과
비교용 offline evidence일 뿐이고, selector 자체는 raw RDF/시차/left image-center feature만
사용한다.

ROS2 런타임에 붙일 때는 `stereo_ground_sim.runtime_deployment_selector`의
`RuntimeMeasurementSample`, `RuntimeFeatureWindow`, `PackagedRuntimeDeploymentSelector`를
사용한다. 토픽 adapter는 `/stereo/estimated_position` 또는 내부 raw RDF 계산 결과를
`raw_right_m/raw_down_m/raw_forward_m/raw_range_m`로, `/stereo/disparity`를
`measured_disparity_px`로, BBox/검출 topic의 left image center를 `left_u_px/left_v_px`로
넣으면 된다. 이 모듈은 feature window와 override rule만 처리하며, PX4/RTK truth,
validation RMSE, target error 같은 replay metric은 runtime input으로 받지 않는다.
기존 StereoDistance 공개 토픽을 바로 연결할 때 `/stereo/estimated_position` 배열은
FRD+range `data=[forward,right,down,range]`로 해석하고,
`RuntimeMeasurementSample.from_public_frd_range(data, disparity, left_u, left_v)`로
내부 selector 입력인 stereo-center RDF `raw_right/raw_down/raw_forward/raw_range`로
변환한다. 이미 내부 계산 단계에서 RDF named fields를 갖고 있다면
`RuntimeMeasurementSample.from_mapping()`에 `raw_right_m/raw_down_m/raw_forward_m`,
`measured_disparity_px`, 선택적 `left_u_px/left_v_px`를 넘긴다. 두 경로 모두 런타임
선택 feature에는 truth, validation error, replay metric을 포함하지 않는다.

Runtime-aligned RDF polynomial residual 계열의 degree/ridge/clamp 탐색은 다음
스크립트로 반복 실행할 수 있다. fitting/evaluation에서만 PX4/local NED truth를
stereo-center RDF로 변환해 사용하고, 후보 runtime contract는 raw RDF와 measured
disparity, 고정 계수만 사용한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_runtime_aligned_rdf_poly_degree_search.py \
  --experiment-id EXP-YYYYMMDD-NNN-runtime-aligned-rdf-poly-search \
  --output-dir Simulation/reports/experiments/EXP-YYYYMMDD-NNN-runtime-aligned-rdf-poly-search \
  --seed-start 20294359 \
  --trial-count 400 \
  --train-sample-count 1260 \
  --validation-sample-count 420 \
  --right-degree 6 \
  --down-degree 7 \
  --forward-degrees 12 \
  --ridge-alphas 0.1 \
  --huber-deltas 8.0 \
  --forward-clamps 1.2,1.6,2.0,2.4
```

2026-06-15 현재 runtime-aligned branch의 synthetic holdout 결론은
`d6/d7/d12 + Huber8 + ridge0.1 + forward clamp 2.0m`가 validation
range/position 기준으로 가장 안정적이라는 것이다. EXP-180은 forward degree
14/16/18 확장이 degree 12 추천을 뒤집지 못함을 보였고, EXP-181은 ridge 0.1을
유지하는 쪽이 aggregate validation RMSE에 유리함을 보였으며, EXP-182는 clamp
2.0m가 1.2/1.6/2.4m보다 validation range/position RMSE 기준으로 낫다는 것을
확인했다. EXP-183/184는 Huber delta 민감도를 확인했으며, delta 6 이상에서는
range/position 결과가 거의 포화되고 delta 1/2처럼 강한 robust weighting은 lateral
RMSE를 조금 낮추는 대신 Forward/range RMSE를 악화한다. EXP-185의 800-seed stability
audit에서도 delta 6/8/10은 사실상 동률이고, delta 6이 aggregate range/position에서
극히 작은 차이로 낮다. 따라서 현재 배포 후보는 기존 Huber8 표기를 유지할 수 있지만,
검증상 핵심은 `delta >= 6` plateau, ridge 0.1, clamp 2.0m, degree d6/d7/d12 조합이다.
1852m target-window error는 모든 후보가 +/-10m 조건을 만족했으므로, 선택 기준은
target만이 아니라 validation RMSE, worst-case, clamp count를 함께 본다.

현재 branch를 StereoDistance runtime JSON으로 내보내려면 다음을 사용한다. 이 export는
z-scored disparity에서 fit한 residual polynomial을 raw-disparity-domain coefficients로
변환해서 `EstimatedToTruthSim3Profile` loader가 그대로 평가할 수 있게 만든다.

```bash
Simulation/.venv/bin/python Simulation/scripts/export_runtime_aligned_rdf_poly_profile.py \
  --experiment-id EXP-YYYYMMDD-NNN-runtime-aligned-d6d7-profile-export-replay-audit \
  --output-dir Simulation/reports/experiments/EXP-YYYYMMDD-NNN-runtime-aligned-d6d7-profile-export-replay-audit/output \
  --seed 20295559 \
  --train-sample-count 1260 \
  --validation-sample-count 420 \
  --right-degree 6 \
  --down-degree 7 \
  --forward-degree 12 \
  --ridge-alpha 0.1 \
  --huber-delta-m 8.0 \
  --forward-clamp-m 2.0
```

EXP-186에서 이 profile은 real StereoDistance node loader와 replay path를 통과했다.
Validation replay는 range RMSE `2.396 -> 0.694m`, position RMSE `4.959 -> 0.704m`,
1852m target-window max absolute error `6.729 -> 1.958m`로 개선됐고, promotion check는
node loader OK, sampled runtime rows `50/50`, invalid `0`, runtime truth unused를 기록했다.
EXP-187은 이 exported profile을 재학습 없이 200개 independent synthetic seed에 고정 적용한
fresh-seed transfer audit이다. 결과는 pass `200/200`, range RMSE mean
`2.419 -> 0.685m`, corrected range RMSE max `0.810m`, 1852m target-window max absolute
error worst `4.607m`, invalid/fallback `0`으로, profile JSON 자체의 synthetic transfer
안정성을 확인했다.
하지만 EXP-188에서 같은 exported profile을 6개 stress scenario에 universal single profile로
고정 적용하면 전체 pass는 `241/300`으로 떨어진다. `nominal`과 `yaw_pitch_stress`는
`50/50` STRICT_PASS였지만, `intrinsic_scale_stress`는 `0/50`, `disparity_noise_stress`는
`49/50`, `bbox_outlier_stress`는 `42/50`이고 bbox/outlier target max abs error는
`19.585m`까지 커졌다. 따라서 이 profile은 nominal/yaw 계열의 강한 runtime profile로
유지하되, 전체 배포에는 scenario/catalog selector 또는 runtime measurement guard가 필요하다.

EXP-190~193은 이 실패를 bbox-outlier 조건에서 더 좁혀 확인한 probe이다. Huber delta,
Forward residual clamp, 기존 inverse-depth/left-disparity geometry profile, 그리고
간단한 temporal/smoothing guard를 비교했지만 fresh bbox-outlier transfer는 최대
`39/50` pass에 머물렀고 1852m target max abs error는 약 `18.8m`로 남았다. 이 조건의
일부 outlier는 measured disparity와 raw Forward를 함께 일관되게 이동시키므로,
단일 프레임의 `raw RDF + measured disparity`만으로는 정상 샘플과 구분되지 않는다.
따라서 이 branch는 더 강한 단일 보정식을 찾는 문제가 아니라, 반복 관측, tracker
association confidence, image-space 품질 신호, 또는 검증된 motion model 기반
multi-frame fusion을 추가해야 하는 관측가능성 문제로 분류한다.

EXP-194~197은 corrected RDF 시계열 위에 runtime-safe causal alpha-beta motion fusion을
추가한 probe이다. 실행은 다음과 같다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_runtime_motion_fusion_sweep.py \
  --experiment-id EXP-YYYYMMDD-NNN-bbox-outlier-causal-alpha-beta-motion-fusion \
  --profile-json Simulation/reports/experiments/EXP-20260615-190-bbox-outlier-runtime-profile-huber-sweep/profiles/huber4/recommended_runtime_profile.json \
  --output-dir Simulation/reports/experiments/EXP-YYYYMMDD-NNN-bbox-outlier-causal-alpha-beta-motion-fusion \
  --scenario bbox_outlier_stress \
  --seed-start 20297659 \
  --trial-count 50 \
  --sample-count 1680 \
  --train-ratio 0.75 \
  --target-oversample-count 72 \
  --axes rdf
```

이 fusion은 timestamped corrected stereo-center RDF만 사용하고 truth, target label,
true disparity를 runtime 입력으로 쓰지 않는다. EXP-194의 best RDF fusion은 `47/50`,
target max abs `13.786m`였고, EXP-196의 forward-only fusion도 `47/50`, target max
`13.788m`였다. EXP-197의 더 강한 smoothing은 pass count와 RMSE가 악화됐다. 따라서
causal alpha-beta fusion은 runtime stabilizer 후보로는 의미가 있지만, 현재
`bbox_outlier_stress`의 final PASS 방법은 아니다.

EXP-198~199는 outlier가 생기는 더 앞단인 image-space `left_u_px`, `left_v_px`,
`measured_disparity_px`를 causal alpha-beta로 prefilter한 뒤 measured-camera RDF로
재삼각측량하고 runtime profile을 적용한 probe이다. 실행 예시는 다음과 같다.

```bash
python3 Simulation/scripts/run_runtime_image_prefilter_sweep.py \
  --experiment-id EXP-YYYYMMDD-NNN-bbox-outlier-causal-image-prefilter \
  --profile-json Simulation/reports/experiments/EXP-YYYYMMDD-NNN-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json \
  --output-dir Simulation/reports/experiments/EXP-YYYYMMDD-NNN-bbox-outlier-causal-image-prefilter \
  --scenario bbox_outlier_stress \
  --seed-start 20297659 \
  --trial-count 50 \
  --sample-count 1680 \
  --train-ratio 0.75 \
  --target-oversample-count 72
```

좌표 계약은 image pixel `u/v/disparity`를 필터링한 뒤 camera intrinsics/baseline으로
stereo-center RDF `right/down/forward`를 재구성하고, calibration/evaluation truth는
PX4 local NED에서 RDF로 변환해 audit에만 사용한다. EXP-198의 작은 disparity clip은
정상 range sweep까지 왜곡해 전체 `0/50` 실패를 만들었다. EXP-199의 wide-clip sanity
check에서 no-op-like 설정은 base profile의 `39/50`, target max `18.836m`를 재현했고,
best guarded image prefilter는 `46/50`, target max `15.397m`에 그쳤다. 따라서 직접
pixel/disparity alpha-beta prefilter도 final PASS 방법은 아니며, 다음 후보는
inverse-depth-domain filtering이나 detector confidence 기반 gate로 분리한다.

EXP-200은 disparity를 직접 clip하지 않고 `disparity/(fx*baseline)` inverse-depth
도메인에서 causal alpha-beta prefilter를 적용한 probe이다. 결과적으로 best setting은
사실상 inverse-depth no-clip(`id999`)과 mild image-center smoothing이었고 `46/50`,
target max abs `15.397m`였다. 실제 inverse-depth clipping은 strict target gate를
개선하지 못했다. 따라서 현재 synthetic bbox-outlier 실패는 disparity/inverse-depth
값의 단순 smoothing보다 detector confidence, association 품질 신호, 반복 독립 관측,
또는 false association을 명시적으로 reject하는 모델이 필요한 문제로 남긴다.

EXP-201은 이 가설 중 반복 독립 관측을 검증한 probe이다. 같은 timestamp/truth에서
여러 독립 `left_u_px`, `left_v_px`, `measured_disparity_px` 관측을 생성하고
median 또는 trimmed mean으로 합성한 뒤 measured-camera RDF로 재삼각측량하고 runtime
profile을 적용했다. 실행 예시는 다음과 같다.

```bash
python3 Simulation/scripts/run_runtime_repeated_observation_fusion_sweep.py \
  --experiment-id EXP-YYYYMMDD-NNN-bbox-outlier-repeated-observation-fusion \
  --profile-json Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json \
  --output-dir Simulation/reports/experiments/EXP-YYYYMMDD-NNN-bbox-outlier-repeated-observation-fusion \
  --scenario bbox_outlier_stress \
  --seed-start 20297659 \
  --trial-count 50 \
  --sample-count 1680 \
  --train-ratio 0.75 \
  --target-oversample-count 72 \
  --repeat-counts 1,3,5,7 \
  --fusion-modes median,trimmed_mean
```

결과는 반복 독립 관측이 bbox-outlier 관측가능성 문제를 실제로 깬다는 것을 보였다.
단일 반복 관측 `n=1`은 `43/50`, target max `16.990m`로 실패했지만, `n=3 median`은
`50/50` STRICT_PASS, range RMSE mean `0.524m`, target max abs `2.650m`였다. 가장
강한 target-worst 결과는 `n=7 median`으로 `50/50`, range RMSE mean `0.347m`,
position RMSE mean `0.365m`, target max abs `2.150m`였다. 이 방법은 runtime truth를
사용하지 않지만, 실제 StereoDistance 적용에는 같은 timestamp 또는 짧은 burst 안에서
독립적인 bbox/disparity 관측을 여러 개 확보해야 한다.

실제 StereoDistance 런타임 연결에는
`stereo_ground_sim.runtime_deployment_selector.RepeatedObservationFusion`을 사용한다.
입력은 같은 timestamp 또는 짧은 burst의 `left_u_px`, `left_v_px`,
`measured_disparity_px` 관측 목록이며, 출력은 기존 runtime profile/selector가 받는
`RuntimeMeasurementSample`이다. 이 변환의 좌표 계약은 left image pixel/disparity에서
camera intrinsics/baseline으로 stereo-center RDF `right/down/forward`를 재구성하는
것이다. public `/stereo/estimated_position`의 FRD+range 배열은 이 단계의 입력이
아니며, PX4/RTK truth와 validation/replay metric도 runtime 입력으로 쓰지 않는다.
EXP-201 기준 최소 실용 설정은 `min_observations=3`, `mode="median"`이고, 더 낮은
target-worst error가 필요하면 `min_observations=7`, `mode="median"`을 후보로 둔다.

EXP-202는 같은 seed block에서 repeat count를 `1/3/5/7/9/11/15`로 넓히고
`median`, `trimmed_mean`, `mean`을 비교했다. best는
`repeated_observation_trimmed_mean_n15_trim0p2`로 `50/50` STRICT_PASS,
range RMSE mean/max `0.221/0.272m`, position RMSE mean/max `0.248/0.293m`,
1852m target max abs worst `1.152m`였다. `n15 median`도 `50/50`,
target max abs worst `1.226m`로 거의 같은 수준이었다. 따라서 반복 관측 branch의
현재 결론은 다음과 같다. 실제 처리량이나 latency 제약이 크면 `n=3 median`을 최소
실용 baseline으로 두고, 짧은 burst 안에서 15개 독립 관측을 확보할 수 있으면
`n=15 trimmed_mean`을 bbox-outlier 대응 best-quality 후보로 둔다. `mean`도 n15에서는
PASS였지만 target worst `2.132m`로 robust fusion보다 약하므로 우선순위는 낮다.

EXP-203~205는 `n=15 trimmed_mean`의 trim fraction 민감도를 확인했다. `trim=0.1`,
`0.3`, `0.4` 모두 `50/50` STRICT_PASS였지만, EXP-202의 `trim=0.2`가 range RMSE
mean `0.221m`와 target max abs worst `1.152m`로 가장 낮았다. 따라서 현재
best-quality runtime observation fusion 설정은 `min_observations=15`,
`mode="trimmed_mean"`, `trim_fraction=0.2`로 유지한다.

EXP-206~207은 같은 `trim=0.2`로 관측 수를 더 늘린 saturation check이다.
`n=21 trimmed_mean`은 `50/50` STRICT_PASS, range RMSE mean/max `0.194/0.229m`,
target max abs worst `0.927m`였고, `n=31 trimmed_mean`은 `50/50` STRICT_PASS,
range RMSE mean/max `0.169/0.199m`, position RMSE mean/max `0.202/0.226m`,
target max abs worst `0.886m`였다. 즉 synthetic independent-observation 조건에서는
관측 수를 늘릴수록 성능이 계속 좋아진다. 다만 실제 StereoDistance에서는 같은 timestamp
또는 짧은 burst 안의 latency, target motion blur, detector correlation 때문에 무작정
큰 n을 기본값으로 둘 수 없다. 현재 runtime tier는 `n=3 median`을 최소 실용 baseline,
`n=15 trimmed_mean`을 balanced high-quality 후보, `n=31 trimmed_mean`을 latency를
허용하는 offline/slow-burst best-quality 후보로 둔다.

EXP-208~209는 더 큰 관측 수 `n=51/75`의 high-count check이다. `n=51 trimmed_mean`은
range RMSE mean/max `0.146/0.162m`, target max abs worst `0.803m`였고,
`n=75 trimmed_mean`은 range RMSE mean/max `0.132/0.154m`, position RMSE mean/max
`0.172/0.189m`, target max abs worst `0.614m`였다. 이 결과는 반복 관측이 현재
bbox-outlier synthetic model에서 거의 averaging-law처럼 동작한다는 것을 보여준다.
하지만 실제 런타임 기본값은 독립성이 검증된 detector/burst pipeline의 frame rate와
target motion 조건으로 결정해야 한다. 따라서 `n=75`는 현재 synthetic 최고 성능
상한으로 기록하고, 실시간 후보는 여전히 `n=3`, `n=15`, `n=31` tier를 우선 검토한다.

EXP-210은 반복 관측의 핵심 가정인 독립성을 깨고, 같은 timestamp/burst 안의
`left_u_px`, `left_v_px`, `measured_disparity_px` 관측들이 공통 noise/outlier 성분을
공유하는 경우를 평가했다. 실행은 다음과 같다.

```bash
python3 Simulation/scripts/run_runtime_correlated_repeated_observation_sweep.py \
  --experiment-id EXP-YYYYMMDD-NNN-bbox-outlier-correlated-repeated-observation \
  --profile-json Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json \
  --output-dir Simulation/reports/experiments/EXP-YYYYMMDD-NNN-bbox-outlier-correlated-repeated-observation \
  --scenario bbox_outlier_stress \
  --seed-start 20297659 \
  --trial-count 50 \
  --sample-count 1680 \
  --train-ratio 0.75 \
  --target-oversample-count 72 \
  --repeat-counts 15,31 \
  --fusion-modes trimmed_mean \
  --trim-fraction 0.2 \
  --common-fractions 0,0.25,0.5,0.75,1
```

결과는 반복 관측 fusion의 적용 조건을 좁혀준다. common fraction `0`에서는 `n=15`,
`n=31` 모두 `50/50` STRICT_PASS였지만, common fraction `0.25`부터는 strict PASS가
깨졌다. `n=31 common0.25`는 pass `46/50`, target max abs worst `16.586m`,
`n=31 common0.75`는 pass `49/50`, target max abs worst `13.024m`였다. 즉 여러
관측을 모아도 같은 false association 또는 같은 detector bias를 공유하면 target max
gate를 보장하지 못한다. 실제 StereoDistance 적용 요구사항은 단순 repeat count가
아니라 “독립 관측”이다. 같은 프레임 crop을 여러 번 후처리하는 반복은 충분하지 않고,
서로 다른 detector augmentation, 시간적으로 분리된 frame, tracker association
confidence, epipolar/size/kinematic consistency gate 등으로 common-mode outlier를
줄여야 한다.

EXP-211~212는 EXP-210의 common-mode 실패를 줄이기 위해 같은 총 관측 수를 하나의
burst에 몰아넣지 않고 여러 burst로 나눈 뒤 계층적으로 fusion했다. burst 내부는
`trimmed_mean`, burst 사이 outer fusion은 `median`을 사용했다. 실행 예시는 다음과
같다.

```bash
python3 Simulation/scripts/run_runtime_multiburst_observation_fusion_sweep.py \
  --experiment-id EXP-YYYYMMDD-NNN-bbox-outlier-multiburst-common1-confirmation \
  --profile-json Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json \
  --output-dir Simulation/reports/experiments/EXP-YYYYMMDD-NNN-bbox-outlier-multiburst-common1-confirmation \
  --scenario bbox_outlier_stress \
  --seed-start 20297659 \
  --trial-count 50 \
  --sample-count 1680 \
  --train-ratio 0.75 \
  --target-oversample-count 72 \
  --burst-specs 3x5,5x5 \
  --inner-modes trimmed_mean \
  --outer-modes median \
  --trim-fraction 0.2 \
  --common-fractions 1
```

EXP-211의 12-seed early probe에서 단일 burst `1x15`는 common0.5/1.0에서 실패했지만,
`3x5`와 `5x5`는 common1.0에서도 `12/12` STRICT_PASS였다. EXP-212의 50-seed 확인도
같은 결론이다. `3x5`는 `50/50` STRICT_PASS, range RMSE mean/max `0.525/0.733m`,
position RMSE mean/max `0.538/0.742m`, target max abs worst `4.193m`였다. `5x5`도
`50/50` STRICT_PASS이고 aggregate RMSE는 더 낮았지만 target max abs worst는
`7.513m`로 `3x5`보다 나빴다. 따라서 common-mode outlier 대응 추천은 target gate
중심이면 `3 bursts x 5 observations` + inner trimmed mean + outer median이고,
aggregate RMSE 중심이면 `5x5`도 후보로 둔다. 실제 StereoDistance에서는 burst 간
시간 간격이 target motion으로 인한 geometry change를 만들지 않을 만큼 짧아야 하고,
동시에 detector/crop common-mode failure는 재샘플링될 만큼 독립적이어야 한다.

실제 StereoDistance 런타임에서 EXP-212 구조를 쓰려면
`stereo_ground_sim.runtime_deployment_selector.MultiBurstObservationFusion`을 사용한다.
입력은 burst별 `left_u_px`, `left_v_px`, `measured_disparity_px` row group이고,
출력은 `RuntimeMeasurementSample`이다. 권장 target-gate 설정은
`min_bursts=3`, `min_observations_per_burst=5`, `inner_mode="trimmed_mean"`,
`outer_mode="median"`, `trim_fraction=0.2`이다. 이 adapter는 public
`/stereo/estimated_position` FRD 배열을 입력으로 받지 않고, pixel/disparity를
camera intrinsics/baseline으로 stereo-center RDF `right/down/forward`로 재구성한다.
생성된 `RuntimeMeasurementSample`은 기존 runtime profile apply payload 또는
`RuntimeFeatureWindow.add()`에 그대로 넘긴다. 단일 burst 안에서도 관측 독립성이
확실하면 `RepeatedObservationFusion`을 쓸 수 있지만, 같은 detector/crop 실패가
반복될 위험이 있으면 multi-burst adapter를 우선한다.

현재 observation-fusion 배포 tier를 JSON/CSV/TXT manifest로 묶으려면 다음을 실행한다.

```bash
python3 Simulation/scripts/export_observation_fusion_runtime_manifest.py \
  --root-dir . \
  --output-dir Simulation/results/observation_fusion_runtime_deployment_manifest
```

생성 산출물은 다음과 같다.

- `observation_fusion_runtime_manifest.json`: runtime adapter class와 설정,
  evidence metric, 좌표 계약을 포함한 machine-readable manifest.
- `observation_fusion_runtime_manifest.csv`: tier별 PASS/metric 비교표.
- `observation_fusion_runtime_manifest_summary.txt`: 사람이 읽는 요약.

EXP-213은 같은 observation-fusion 계열을 small-drone range/camera geometry로 옮긴
검증이다. close scenario는 2m baseline, 4x FOV, 20-120m range, 50m target window를
사용했고, mid scenario는 5m baseline, 8x FOV, 80-350m range, 180m target window를
사용했다. 실행은 다음과 같다.

```bash
python3 Simulation/scripts/run_runtime_small_drone_observation_fusion_sweep.py \
  --experiment-id EXP-YYYYMMDD-NNN-small-drone-observation-fusion \
  --output-dir Simulation/reports/experiments/EXP-YYYYMMDD-NNN-small-drone-observation-fusion \
  --trial-count 50 \
  --sample-count 360 \
  --target-oversample-count 32 \
  --burst-specs 1x3,3x3,3x5 \
  --inner-modes median \
  --outer-modes median \
  --correction-modes fusion_only,sim3,affine_ridge \
  --common-fractions 0,1
```

결론은 두 갈래다. `fusion_only`는 close/mid 모두 실패했으므로 small-drone 보정법으로
단독 사용하지 않는다. 반면 `1x3 median + calibration-time Sim3`는 두 scenario에서
각각 `50/50` STRICT_PASS였고 runtime truth를 쓰지 않았다. close range RMSE는
`0.2775 -> 0.0623m`, mid range RMSE는 `0.6143 -> 0.0953m`였으며 target max abs는
각각 `0.160m`, `0.301m`였다. 더 많은 관측을 쓸 수 있으면 `3x5 median + affine_ridge`
가 품질 우선 후보이며 close/mid range RMSE를 각각 `0.0337m`, `0.0524m`까지 낮췄다.
실제 배포 전에는 small-drone profile export와 camera/body extrinsic 검증이 필요하다.

초기 manifest의 default recommendation은 `common_mode_target_gate`였고, 이후 fresh
deployment package의 default recommendation은 `fresh_common_mode_b5x5_aggregate_best`였다.
EXP-215 full follow-up 이후 최신 fresh deployment package의 default recommendation은
`fresh_common_mode_b7x3_aggregate_best`이다. 이 tier는 bbox, combined sensor,
disparity noise, intrinsic-identity stress 전체의 method aggregate를 근거로 고른
7x3 multi-burst 후보다. 이유는 실제
detector/crop/disparity error가 짧은 burst 안에서 상관될 수 있다고 보고, 단일 scenario
best나 독립 반복 관측보다 all-scenario aggregate common-mode 대응을 우선하기 때문이다.
관측 독립성이 별도 검증되면 `fresh_independent_n7_low_latency` 같은 반복 관측 tier로
전환할 수 있다. manifest도 runtime truth를 사용하지 않으며, 입력 계약은 image
`u/v/disparity`, 출력 계약은 stereo-center RDF이다.

ROS2/StereoDistance runtime node에서 manifest를 직접 읽어 adapter를 만들려면 다음
loader를 사용한다.

```python
from stereo_ground_sim.runtime_deployment_selector import (
    RuntimeFeatureWindow,
    load_observation_fusion_adapter_from_manifest,
)

adapter, tier = load_observation_fusion_adapter_from_manifest(
    "Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_followup/fresh_deployment_package/observation_fusion_runtime_manifest.json"
)

# default tier는 MultiBurstObservationFusion이므로 burst별 row group을 넣는다.
sample = adapter.fuse_row_groups(
    [
        burst0_rows,  # each row has left_u_px, left_v_px, measured_disparity_px
        burst1_rows,
        burst2_rows,
        burst3_rows,
        burst4_rows,
        burst5_rows,
        burst6_rows,
    ]
)
if sample is not None:
    feature_window = RuntimeFeatureWindow()
    feature_window.add(sample)
```

`tier="fresh_independent_n7_low_latency"`처럼 tier 이름을 지정하면 독립 반복 관측용
`RepeatedObservationFusion`을 로드할 수 있다. 이 loader는 adapter 설정만 만들며,
public `/stereo/estimated_position` FRD 배열이나 PX4/RTK truth를 observation-fusion
입력으로 받지 않는다.

manifest의 모든 tier가 실제 runtime adapter로 로드되고 RDF smoke reconstruction을
통과하는지 확인하려면 다음 audit을 실행한다.

```bash
python3 Simulation/scripts/audit_observation_fusion_runtime_manifest.py \
  --manifest-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_followup/fresh_deployment_package/observation_fusion_runtime_manifest.json \
  --output-dir Simulation/results/observation_fusion_runtime_deployment_manifest/audit
```

현재 audit 결과는 `12/12 PASS`이다. `RepeatedObservationFusion` tier들은 synthetic
RDF smoke sample을 수치 오차 수준으로 복원했고, `MultiBurstObservationFusion` tier들은
inner/outer fusion offset 때문에 max axis error가 tolerance `0.2m` 안에
들었다. audit 산출물은 `observation_fusion_runtime_manifest_audit.csv/json` 및
`observation_fusion_runtime_manifest_audit_summary.txt`이다.

CSV observation 로그에 manifest를 적용해 runtime RDF sample CSV를 만들려면 다음
apply bridge를 사용한다.

```bash
python3 Simulation/scripts/apply_observation_fusion_runtime_manifest.py \
  --manifest-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_followup/fresh_deployment_package/observation_fusion_runtime_manifest.json \
  --input-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_followup/fresh_deployment_package/profile_apply_smoke/synthetic_multiburst_observations.csv \
  --output-dir Simulation/results/observation_fusion_runtime_deployment_manifest/apply_smoke
```

입력 CSV 계약은 `group_id`, 선택적 `burst_id`, `left_u_px`, `left_v_px`,
`measured_disparity_px`이다. default `fresh_common_mode_b7x3_aggregate_best` tier는
`MultiBurstObservationFusion`이므로 `burst_id`로 burst를 나눈다. `tier`를
`fresh_independent_n7_low_latency`처럼 반복 관측 tier로 지정하면 `burst_id` 없이
`group_id`별 row 묶음을 `RepeatedObservationFusion`으로 처리한다. 출력
`observation_fusion_runtime_samples.csv`는 `group_id`별 stereo-center RDF
`raw_right_m/raw_down_m/raw_forward_m/raw_range_m`와 fused image/disparity 값을 저장한다.
smoke 결과는 `valid_samples 1/1`, default tier `fresh_common_mode_b7x3_aggregate_best`,
RDF `right=3.981m`, `down=1.500m`, `forward=800.011m`였다.

observation fusion 입력 CSV 자체의 axis/schema 계약을 먼저 검증하려면 다음 validator를
실행한다.

```bash
python3 Simulation/scripts/validate_observation_fusion_runtime_input_schema.py \
  --input-csv Simulation/results/observation_fusion_runtime_deployment_manifest/apply_smoke/synthetic_multiburst_observations.csv \
  --output-dir Simulation/results/observation_fusion_runtime_deployment_manifest/apply_smoke/input_schema_validation \
  --require-burst-id
```

이 gate는 필수 field `group_id`, `left_u_px`, `left_v_px`,
`measured_disparity_px`, multiburst용 `burst_id`, finite numeric value, 양수 disparity,
PX4/RTK truth field 부재를 검사한다. package smoke의 input validation은 `PASS`,
row count `15`, group count `1`, burst count `3`이다. 좌표 계약은 image
`u/v/disparity` 입력이며, 출력은 stereo-center RDF 재구성 준비 상태이다.

ROS2 topic과 CSV 경계의 좌표 계약을 manifest로 저장하려면 다음을 실행한다.

```bash
python3 Simulation/scripts/export_observation_fusion_topic_contract_manifest.py \
  --output-dir Simulation/results/observation_fusion_runtime_deployment_package/topic_contract
```

출력은 `observation_fusion_topic_contract_manifest.json/csv` 및
`observation_fusion_topic_contract_manifest_summary.txt`이다. 기본 계약은
`/fmu/out/vehicle_local_position`과 `/ground_truth_position`을 calibration/validation
truth로만 표시하고, runtime input은 좌/우 image, 좌/우 bbox, positive measured
disparity로 제한한다. 내부 measurement frame은 stereo-center RDF
`right/down/forward`, public StereoDistance topic frame은 FRD+range
`forward/right/down/range`이다.

좌/우 YOLO BBox 또는 synthetic BBox 중심을 observation-fusion 입력 계약으로 바꾸려면
다음 bridge를 사용한다.

```bash
python3 Simulation/scripts/convert_stereo_bbox_observations_to_fusion_input.py \
  --input-csv Simulation/results/observation_fusion_runtime_deployment_package/bbox_conversion_smoke/synthetic_stereo_bbox_observations.csv \
  --output-dir Simulation/results/observation_fusion_runtime_deployment_package/bbox_conversion_smoke
```

입력은 `left_bbox_cx_px/left_bbox_cy_px`, `right_bbox_cx_px/right_bbox_cy_px`
또는 `left_bbox_xmin_px/.../xmax/ymax`, `right_bbox_xmin_px/.../xmax/ymax`를 받는다.
`measured_disparity_px`가 있으면 그대로 쓰고, 없으면 `left_u_px - right_u_px`로
계산한다. 출력 `observation_fusion_bbox_converted_input.csv`는 `group_id`,
선택적 `burst_id`, `left_u_px`, `left_v_px`, `measured_disparity_px`를 저장하므로
그대로 input schema validator와 manifest apply bridge에 넣을 수 있다. package smoke의
bbox conversion은 `PASS`, input row `15`, output row `15`, failure count `0`이고,
변환된 CSV의 input schema validation도 `PASS`이다. 이 bridge도 PX4/RTK truth field를
runtime 입력으로 받지 않는다.

deployment package smoke는 변환된 BBox CSV를 실제 observation fusion adapter와
runtime profile에도 그대로 넣어 본다. `observation_fusion_bbox_runtime_samples.csv`는
변환된 BBox 중심에서 재구성한 raw stereo-center RDF를 저장하고,
`observation_fusion_bbox_corrected_runtime_samples.csv`는 corrected RDF 및 public
FRD+range alias를 저장한다. 현재 bbox-converted end-to-end path는 raw apply valid
sample `1/1`, profile apply valid output `1/1`, output schema validation `PASS`이다.

보정 비행 recorder CSV처럼 truth와 runtime 관측이 한 파일에 함께 들어 있는 경우에는
runtime observation input만 추출하는 bridge를 먼저 실행한다.

```bash
python3 Simulation/scripts/extract_observation_fusion_input_from_calibration_csv.py \
  --input-csv Simulation/results/observation_fusion_runtime_deployment_package/calibration_csv_bridge_smoke/synthetic_calibration_recorder_rows.csv \
  --output-dir Simulation/results/observation_fusion_runtime_deployment_package/calibration_csv_bridge_smoke
```

이 bridge는 `truth_*`, `px4_*`, `rtk_*` 계열 column을 runtime output에서 제거하고,
좌/우 bbox 또는 `left_cx_px/left_cy_px`, `right_cx_px/right_cy_px`,
`measured_disparity_px`만 사용해 `observation_fusion_calibration_runtime_input.csv`를
만든다. package smoke에서는 truth column `5`개를 drop했고, 추출 output row `15/15`,
input schema validation `PASS`, raw apply valid sample `1/1`, profile apply valid
output `1/1`, output schema validation `PASS`였다. 즉 calibration-time truth는
검증용 CSV에 남아 있어도 runtime observation path에는 전달되지 않는다.

deployment package는 세 runtime 입력 경로의 출력 동등성도 검사한다. 기준 CSV는
synthetic `u/v/disparity` path의 `observation_fusion_corrected_runtime_samples.csv`이고,
비교 대상은 BBox-converted path와 calibration-extracted path의 corrected runtime CSV이다.
현재 `observation_fusion_path_equivalence.txt` 결과는 tolerance `1e-9`에서 `PASS`이며,
raw RDF, corrected RDF, public FRD+range, image `u/v`, measured disparity의 max delta가
모두 `0.0`이다.

observation fusion 뒤에 기존 StereoDistance runtime correction profile까지 한 번에
적용하려면 다음 end-to-end bridge를 사용한다.

```bash
python3 Simulation/scripts/apply_observation_fusion_with_runtime_profile.py \
  --manifest-json Simulation/results/observation_fusion_runtime_deployment_manifest/observation_fusion_runtime_manifest.json \
  --profile-json Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json \
  --input-csv Simulation/results/observation_fusion_runtime_deployment_manifest/apply_smoke/synthetic_multiburst_observations.csv \
  --output-dir Simulation/results/observation_fusion_runtime_deployment_manifest/profile_apply_smoke
```

출력 `observation_fusion_corrected_runtime_samples.csv`는 raw stereo-center RDF와
corrected stereo-center RDF를 함께 저장한다. 또한 ROS topic 발행에 바로 쓸 수 있도록
public StereoDistance FRD+range alias도 함께 저장한다. 내부 RDF field는
`raw_right_m/raw_down_m/raw_forward_m` 및
`corrected_right_m/corrected_down_m/corrected_forward_m`이고, public topic alias는
`raw_public_forward_m/raw_public_right_m/raw_public_down_m/raw_public_range_m` 및
`corrected_public_forward_m/corrected_public_right_m/corrected_public_down_m/corrected_public_range_m`이다.
smoke 결과는 `valid_fused_samples 1/1`, `valid_profile_outputs 1/1`, profile fallback
level `0`이었다. 예시 raw RDF는 `right=3.997m`, `down=1.497m`, `forward=800.000m`이고
corrected RDF는 `right=6.656m`, `down=2.624m`, `forward=799.305m`이다. 같은 row의
corrected public alias는 `[forward,right,down,range]=[799.305,6.656,2.624,799.337]`이다.
이 단계도 runtime truth를 사용하지 않는다.

corrected runtime CSV를 실제 public topic payload로 분리하려면 다음 exporter를 사용한다.

```bash
python3 Simulation/scripts/export_observation_fusion_public_topic_payload.py \
  --input-csv Simulation/results/observation_fusion_runtime_deployment_package/profile_apply_smoke/observation_fusion_corrected_runtime_samples.csv \
  --output-dir Simulation/results/observation_fusion_runtime_deployment_package/public_topic_payload
```

출력 `observation_fusion_public_topic_payload.csv`는 `/stereo/corrected_position`에
실을 `std_msgs/msg/Float32MultiArray` payload를 `forward,right,down,range` 순서로 저장한다.
현재 smoke payload는
`[799.3053304217467,6.656288291423835,2.623733437361108,799.3373514302873]`이고,
public topic payload export는 `PASS`, payload row count `1`이다.

publish 직전 payload CSV 계약을 별도로 검증하려면 다음 validator를 실행한다.

```bash
python3 Simulation/scripts/validate_observation_fusion_public_topic_payload.py \
  --input-csv Simulation/results/observation_fusion_runtime_deployment_package/public_topic_payload/observation_fusion_public_topic_payload.csv \
  --source-csv Simulation/results/observation_fusion_runtime_deployment_package/profile_apply_smoke/observation_fusion_corrected_runtime_samples.csv \
  --output-dir Simulation/results/observation_fusion_runtime_deployment_package/public_topic_payload
```

이 validator는 topic `/stereo/corrected_position`, message type
`std_msgs/msg/Float32MultiArray`, axis order `forward,right,down,range`, payload JSON
배열 순서, range 일관성, runtime truth flag를 검사한다. `--source-csv`를 함께 주면
payload 값이 source corrected CSV의 `corrected_public_*` alias와 같은지도 확인한다.
현재 public topic payload validation은 `PASS`, source equivalence도 `PASS`, row count
`1`, failure count `0`이다.

payload CSV를 ROS2 publisher 직전 단계까지 재생하려면 다음 dry-run을 실행한다.

```bash
python3 Simulation/scripts/publish_observation_fusion_public_topic_payload.py \
  --input-csv Simulation/results/observation_fusion_runtime_deployment_package/public_topic_payload/observation_fusion_public_topic_payload.csv \
  --output-dir Simulation/results/observation_fusion_runtime_deployment_package/public_topic_payload \
  --dry-run \
  --once
```

현재 dry-run은 payload count `1`, status `PASS`이고
`[799.3053304217467, 6.656288291423835, 2.623733437361108, 799.3373514302873]`를
`/stereo/corrected_position` payload로 확인했다. 실제 ROS2 환경에서 발행하려면
`--dry-run`을 빼고 같은 명령을 실행한다. 이 publisher도 payload CSV의
`Float32MultiArray[forward,right,down,range]` 계약을 검증한 뒤 발행한다.

end-to-end output CSV의 axis/schema 계약을 검증하려면 다음 validator를 실행한다.

```bash
python3 Simulation/scripts/validate_observation_fusion_runtime_output_schema.py \
  --input-csv Simulation/results/observation_fusion_runtime_deployment_manifest/profile_apply_smoke/observation_fusion_corrected_runtime_samples.csv \
  --output-dir Simulation/results/observation_fusion_runtime_deployment_manifest/profile_apply_smoke/schema_validation
```

현재 smoke output validation은 `PASS`, row count `1`, failure count `0`이다. 이 gate는
내부 RDF `right/down/forward`, public alias `forward/right/down/range`, range 계산,
runtime truth flag, forbidden truth fields를 함께 검사한다.

topic contract export, manifest export, adapter audit, synthetic CSV generation,
bbox-to-fusion conversion, input schema validation, raw observation fusion apply,
runtime profile apply, output schema validation, package acceptance check를 한 번에
재현하려면 deployment package wrapper를 실행한다.

```bash
python3 Simulation/scripts/run_observation_fusion_deployment_package.py \
  --root-dir . \
  --output-dir Simulation/results/observation_fusion_runtime_deployment_package \
  --profile-json Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json
```

현재 package smoke 결과는 default tier `fresh_common_mode_b7x3_aggregate_best`, manifest audit `12/12`,
topic contract manifest export `PASS`, bbox conversion `PASS`, bbox converted input
schema validation `PASS`, bbox-converted raw apply valid sample `1/1`, bbox-converted
profile apply valid output `1/1`, bbox-converted output schema validation `PASS`,
calibration CSV extraction `PASS`, calibration truth columns dropped `5`, calibration
runtime profile apply valid output `1/1`, calibration output schema validation `PASS`,
path equivalence `PASS`, public topic payload export `PASS`, public topic payload
validation `PASS`, public topic payload publish dry-run `PASS`, input schema validation
`PASS`, raw apply valid sample `1/1`, profile apply valid output `1/1`, output schema
validation `PASS`, StereoDistance node smoke `PASS`, all-tier override smoke `PASS`,
operator-view fusion launch check `PASS`, operator-view tier override check `PASS`,
operator-view package-dir launch check `PASS`, package acceptance `PASS`, deployment
inventory `PASS`이다.
패키지 산출물은 manifest `json/csv/txt`, audit `csv/json/txt`, raw apply smoke,
profile apply smoke, topic contract manifest, bbox conversion smoke, input/output schema
validation, acceptance check, 그리고
`active_runtime_profile.json`과 `observation_fusion_deployment_package_summary.json/txt`를
포함한다. package-dir launch check는 wrapper가 package 내부
`active_runtime_profile.json`과 package-local observation-fusion manifest를 자동 해석하는지
검증한다. deployment inventory는
`inventory/observation_fusion_deployment_inventory.json/csv/txt`로 저장되며, 재생성 가능한
검증 로그인 `inventory/`와 `acceptance_check/`를 제외한 package 파일들의 상대 경로,
크기, SHA-256을 기록하고 즉시 재검증한다. 최신 fresh package inventory는 73개 파일,
161744 bytes를 기록한다. 기존 inventory를 다시 검증할 때는 같은
스크립트에 `--inventory-json`을 지정한다. acceptance check는
하위 JSON들을 다시 열어 topic truth offline-only, bbox/disparity runtime input,
default tier와 package summary 일치, default tier의 all-scenario aggregate scope,
truth-free MultiBurst adapter, aggregate STRICT_PASS evidence, manifest audit 전 tier PASS,
bbox conversion PASS, bbox-converted raw/profile apply PASS,
calibration CSV truth-drop bridge PASS, input/output schema PASS, raw/profile valid
output, packaged profile hash/source consistency, 세 runtime 입력 경로의 RDF/public output 동등성, public
`Float32MultiArray[forward,right,down,range]` payload export를 함께 검증한다. 이 wrapper도
runtime truth를 사용하지 않으며 좌표 계약은 image `u/v/disparity` -> raw
stereo-center RDF -> corrected stereo-center RDF -> public FRD+range alias이다.

Corrected deployment comparison은
`Simulation/results/rtk_stereo_theil_sen_full_family_probe/deployment_strategy_corrected_comparison.csv`
에 저장한다. 2026-06-16 기준 비교는 다음과 같다.

- Full scenario catalog: profile 6개, promotion 6/6, validation RMSE mean/max
  `1.090/1.544m`, 1852m target abs max `1.990m`.
- Minimal two-profile catalog: profile 2개, transfer 18/18, validation RMSE
  mean/max `1.293/2.487m`, 1852m target abs max `4.377m`.
- Pooled universal left-disparity geometry profile: profile 1개, corrected
  promotion 1/1, transfer 18/18, validation RMSE mean/max `1.437/2.475m`,
  1852m target abs max `3.839m`.

따라서 scenario/deployment label을 신뢰할 수 있으면 full scenario catalog가 기본
추천이다. scenario label 없이 단일 profile만 배포해야 하면 corrected pooled universal
profile 중 5-seed transfer replay를 통과한
`huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut`를
현재 가장 강한 fallback으로 둔다. Minimal two-profile catalog는 profile 수를 줄일 수
있지만 pooled single profile보다 RMSE worst-case가 거의 같고 target worst-case는 더
나쁘므로, 현재는 우선순위를 낮춘다.

운영 결론은 다음과 같다. 방법론 후보로는
`target_weighted_theil_sen_global_inverse_depth_linear`를 robust low-complexity
baseline으로 승격한다. 실제 StereoDistance 배포 profile은 scenario label을 신뢰할 수
있으면 scenario catalog를 기본으로 사용하고, scenario label 없이 하나의 profile만
배포해야 하면 corrected promotion check를 통과한 pooled left-disparity geometry
profile을 후보로 비교한다. 특히 bbox/outlier 계열 Sim(3) 잔차 profile은 nominal/yaw에는
잘 전이되지만 intrinsic-scale에서는 raw보다 나빠질 수 있으므로, intrinsic-scale
family는 별도 catalog profile로 분리한다. 다음 자동탐색에서는 near-tie replay 폭을
제한해 full-family probe가 부가 replay에 과도한 시간을 쓰지 않도록 한다.

추가로 `audit_runtime_selector_portfolio.py`는 `--objective target_cap_then_rmse`와
`--max-depth 2`를 지원한다. `target_cap_then_rmse`는 1852m target max가
`--target-error-limit-m`을 넘는 선택을 먼저 피하고, 한계 안에서는 validation RMSE를
우선한다. 세 후보(`robust_family`, `transfer_risk`, `affine_bbox`)를 넣은 5-feature
LOO 진단에서는 depth-1 target-cap objective가 기존과 같은 18/18 PASS,
RMSE mean/max `1.150/1.758m`, target max `8.951m`였다. target 포함 oracle
진단은 RMSE mean `1.131m`까지 내려가 잠재 신호는 있었지만, depth-2 LOO는
RMSE mean/max `1.196/2.381m`, target max `9.423m`로 악화됐다. 따라서 현재는
depth-2 tree를 배포 추천으로 쓰지 않고, selector 과적합 진단 기능으로만 유지한다.

strict LOSO에서 intrinsic-scale source profile이 빠진 경우를 별도로 분리하려면
calibration-required gap 요약을 실행한다. 이 요약은 runtime raw RDF/disparity에서
계산한 product-ratio feature만으로 “해당 regime profile이 필요하지만 catalog에 없음”을
표시하고, truth 기반 replay metric은 offline 안전성 감사에만 사용한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/summarize_runtime_calibration_required_gaps.py \
  --runtime-audit-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_audit_policy_catalog_robust_family_loso/runtime_measurement_guard_audit.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_calibration_required_gap_summary_policy_catalog_robust_family_loso \
  --product-ratio-p10-threshold 1.0037 \
  --required-source-scenario intrinsic_scale_stress \
  --target-error-limit-m 10
```

현재 policy catalog robust-family strict LOSO 결과에서는 18개 audit 중 3개가
calibration-required intrinsic-scale regime으로 감지된다. 모두 target max 10m 안에는
있지만 strict PASS는 0/3, validation-only PASS는 2/3이며, 1개 run은 선택된 대체
profile이 validation RMSE를 악화시켰다. 따라서 이 영역은 “대체 profile을 더 고르는
문제”가 아니라 실제 intrinsic-scale 보정 비행 profile을 catalog에 포함하거나,
runtime에서 calibration-required 상태로 보수적으로 처리해야 하는 coverage gap이다.
experiment dashboard는 이를 `runtime_calibration_required_gap_loso` 행으로 기록한다.

필수 intrinsic-scale source profile이 없는 catalog/minimal-set에서 active runtime
profile export를 막는 smoke는 다음처럼 실행한다. 이 경로는 runtime raw
stereo-center RDF/disparity feature만 사용해 차단 여부를 결정하고, calibration-time
truth는 사용하지 않는다.

```bash
Simulation/.venv/bin/python Simulation/scripts/export_stereodistance_profile_by_operating_mode.py \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/policy_family_profile_catalog/manifest.json \
  --input-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/intrinsic_scale_stress/seed_20260613/calibration_samples.csv \
  --output-profile /tmp/sasl_blocked_runtime_profile.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/operating_mode_export_intrinsic_scale_required_block_smoke \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --mode runtime_guard \
  --runtime-candidate-source-mode minimal_set \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_guard_minimal_set_without_intrinsic_scale_for_block_smoke.json \
  --runtime-selection-policy nearest_centroid \
  --intrinsic-scale-gap-source-scenario intrinsic_scale_stress \
  --intrinsic-scale-gap-product-ratio-p10-threshold 1.0037 \
  --block-calibration-required-runtime-export
```

현재 smoke 결과는 `feature_disparity_forward_product_ratio_p10_to_nominal =
1.003814`로 threshold `1.0037`을 넘고, `intrinsic_scale_stress` source가 없으므로
`selected=false`, `calibration_required_blocked_export=true`, `output_profile=""`이다.
즉 안전하지 않은 fallback active profile을 만들지 않는다. dashboard는 이를
`runtime_calibration_required_export_block_smoke` 행으로 기록한다.

여러 audit/export 결과를 한 번에 비교하려면 experiment dashboard를 생성한다. 이
dashboard는 보정 비행 truth를 사용할 수 있는 offline 선택과 truth가 없는 runtime
선택을 분리해서 요약하고, 각 결과의 pass count와 1852m target-window worst-case를
한 곳에 기록한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/summarize_rtk_stereo_experiment_dashboard.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/experiment_dashboard
```

산출물은 `rtk_stereo_experiment_dashboard.csv`,
`rtk_stereo_experiment_dashboard_summary.json`,
`rtk_stereo_experiment_dashboard_summary.txt`,
`rtk_stereo_experiment_dashboard_pass_ratio.png`이다.
2026-06-16 현재 monotone inverse-depth runtime sweep dashboard 기준으로 runtime
truth-free 추천은 `policy_catalog+robust_family_fallback`, truth가 있는 보정/검증 단계
추천은 `calibration_replay_all_catalog`, truth가 없는 runtime 추천은
`runtime_transfer_risk_guard`이다. policy catalog runtime selector는 18/18 PASS,
1852m target max abs worst-case `8.951m`, oracle target abs regret max `0.613m`였다.
dashboard는 `runtime_guard_policy_catalog_robust_family_loso` 행과
`runtime_guard_policy_catalog_robust_family_loso_validation_only`,
`runtime_calibration_required_export_block_smoke`, `method_family_potential`,
`affine_family_profile_selection`, `affine_family_runtime_feature_selector` 행도 기록한다.
strict LOSO는 엄격 기준 14/18 PASS, validation-only 기준 17/18 PASS,
target-window max abs worst-case `7.377m`이며, full catalog runtime 추천을 대체하지
않고 coverage gap 추적에 사용한다. `method_family_potential`은 현재 메인 sweep에서
oracle-style affine family potential 20/20 PASS, target worst-case `9.308m`를
보여준다. `affine_family_runtime_feature_selector`의 robust bbox fallback은 truth-free
runtime feature 선택만으로 20/20 PASS, target worst-case `9.423m`까지 확인했다. 다음
탐색은 full policy catalog보다 낮은 worst-case를 내는 runtime truth-free 선택 정책,
LOSO 조건에서의 coverage gap 축소, affine family 내부 profile 선택 안정화,
policy/affine portfolio의 raw-feature rule 확장이다.
dashboard의 좌표 계약은 stereo-center OpenCV optical
`RDF(right,down,forward)`이고 public topic 계약은 StereoDistance
`FRD_range(forward,right,down,range)`이다.

minimal set의 배포 label 매핑과 calibration-replay 자동 선택 중 어느 쪽이 더
안정적인지는 sweep 전체에 대해 감사할 수 있다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_profile_selection_strategies.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set/profile_selection_strategy_audit
```

`profile_selection_strategy_runs.csv`는 각 scenario/seed에서 label 기반 source
profile, calibration-replay가 선택한 source profile, 두 방법의 validation RMSE와
1852m target error를 나란히 기록한다.
`profile_selection_strategy_scenarios.csv`는 scenario별 pass count, 두 선택이
일치한 횟수, calibration-replay가 label 선택보다 낮은 validation RMSE를 낸
횟수를 요약한다. 이 감사도 calibration-time truth로 선택 전략을 평가하는
offline 절차이며, runtime truth 입력을 허용하지 않는다.
기본 후보 범위는 deployment-ready minimal set이다. 더 넓은 offline 진단에서는
`--candidate-source-mode all_catalog`를 주면 catalog의 모든 validated scenario
profile을 후보로 replay하여, minimal set 밖의 profile이 현재 보정 비행에 더
맞는지 확인할 수 있다.

검증된 catalog profile 둘 이상이 같은 runtime profile family라면, 보정 비행
truth로 profile 사이 alpha를 탐색해 blended runtime profile 후보도 만들 수
있다. 이 방법은 runtime에서 truth를 쓰지 않으며, 선택된 blend는 일반
`estimated_to_truth_sim3` 또는 `estimated_to_truth_disparity_poly` profile JSON으로
저장된다. Sim(3)는 scale/rotation/translation을 보간하고, inverse-depth
ray-scale 계열은 `inverse_depth_coefficients_high_to_low`를 보간한다. 단, blend는
전체 validation RMSE를 조금 희생하고 1852m target error를 크게 줄이는
target-guard 후보가 될 수 있으므로, balanced 추천을 자동 대체하지 말고
`profile_blend_candidates.csv`와 promotion check 결과를 함께 확인한다.
기본 blend 후보도 minimal set의 source profile pair로 제한된다. sensor-error
regime 식별을 더 공격적으로 탐색하려면 `--candidate-source-mode all_catalog`를
추가해 모든 validated catalog profile pair를 blend 후보로 만들 수 있다.
이 경우 조합 수가 늘어나므로 offline audit/보정 단계에서만 사용한다.
`--selection-policy balanced|validation_best|target_guard|simple_stable|replay_best`로
어떤 policy row를 `--output-profile`에 export할지 명시할 수 있다. post-sweep
deployment audit가 `balanced`를 추천했다면 같은 값을 사용해 active profile을
만든다.

```bash
Simulation/.venv/bin/python Simulation/scripts/explore_stereodistance_profile_blends.py \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/scenario_profile_catalog_minimal_set.json \
  --input-csv Simulation/results/rtk_stereo_robustness_sweep_minimal_set/combined_sensor_stress/seed_20260614/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set/profile_blend_exploration \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep_minimal_set/active_runtime_profile_blend.json
```

출력은 `profile_blend_candidates.csv`,
`profile_blend_pareto_frontier.csv`,
`profile_blend_policy_recommendations.csv`,
`profile_blend_selection.json`, `profile_blend_summary.txt`,
`blend_candidate_profiles/*.json`이다. `profile_blend_policy_recommendations.csv`는
같은 alpha 후보를 `balanced`, `validation_best`, `target_guard`,
`simple_stable` 기준으로 다시 고른다. 선택된 blend profile도 실제 적용 전
`stereodistance_profile_promotion_check.py`로 schema/load/runtime input 계약을
검증한다.

blend policy가 특정 보정 비행 하나에만 맞는지, sweep 전체 scenario/seed에서도
안정적으로 동작하는지는 별도 audit로 확인한다. 이 audit는 각 seed의
calibration CSV에 대해 blend 탐색을 반복하고, `balanced`, `validation_best`,
`target_guard`, `simple_stable` 정책별 replay 결과를 집계한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_profile_blend_strategies.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set/profile_blend_strategy_audit
```

출력은 `blend_strategy_policy_runs.csv`,
`blend_strategy_policy_summary.csv`, `blend_strategy_summary.json`,
`blend_strategy_summary.txt`, `per_run_blend_exploration/*`이다.
`blend_strategy_policy_summary.csv`는 정책별 validation RMSE 평균/최대값,
1852m target-window error 평균/최대값, split-stability 최소 개선량,
선택된 alpha 집합을 기록한다. 이 절차도 calibration-time truth로 정책을
평가하는 offline audit이며, 선택된 runtime profile이 truth를 입력으로 쓰는
것을 허용하지 않는다.

monotone inverse-depth runtime sweep에서 all-catalog inverse-depth blend를
추가로 감사하려면 다음처럼 실행한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_profile_blend_strategies.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/profile_blend_strategy_audit_all_catalog \
  --candidate-source-mode all_catalog \
  --alphas 0,0.25,0.5,0.75,1
```

2026-06-16 결과에서 all-catalog `target_guard` blend는 18/18 PASS, 1852m
target abs mean/max `0.175/1.471m`, target max abs worst-case `7.744m`로 기존
runtime selector의 target max abs worst-case `9.681m`보다 보수적이었다. 대신
validation RMSE mean/max는 `1.277/2.186m`로 기존 runtime selector `1.089/1.676m`
보다 높다. 따라서 이 후보는 1852m target-window 정확도를 우선하는
calibration-time offline export 후보로 보존하고, 일반 runtime 자동 추천은 여전히
validation 안정성이 더 좋은 `transfer_risk_fallback`을 기본으로 둔다.

해당 target-guard blend를 실제 active runtime profile 후보로 export하고
StereoDistance loader/promotion check까지 확인하려면 다음을 실행한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/explore_stereodistance_profile_blends.py \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog_minimal_set.json \
  --input-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/combined_sensor_stress/seed_20260613/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_profile_export_target_guard_blend/blend_profile_export \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_profile_export_target_guard_blend/active_runtime_profile_target_guard_blend.json \
  --candidate-source-mode all_catalog \
  --selection-policy target_guard \
  --alphas 0,0.25,0.5,0.75,1

MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/stereodistance_profile_promotion_check.py \
  --profile-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_profile_export_target_guard_blend/active_runtime_profile_target_guard_blend.json \
  --sample-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/combined_sensor_stress/seed_20260613/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_profile_export_target_guard_blend/promotion_check \
  --split validation \
  --max-samples 50
```

2026-06-16 export 결과는 `intrinsic_scale_stress -> combined_sensor_stress`,
alpha `0.5`의 `estimated_to_truth_disparity_poly` profile이었다. Promotion check는
PASS였고, StereoDistance node loader가 profile을 로드했으며 validation sample 50개
적용에서 invalid sample은 0개였다. 해당 active profile의 보정 비행 replay 성능은
validation RMSE `2.186m`, 1852m target abs `0.016m`, target max abs `2.927m`이다.

단일 active profile로 고정한 뒤 다른 scenario/seed에도 전이되는지는 별도 replay로
확인한다. 이 감사는 profile 하나를 모든 calibration CSV validation split에 적용해
raw 대비 RMSE 개선, target-window 존재, 1852m target abs/max 10m 이하를 확인한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_active_profile_transfer.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --profile-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_profile_export_target_guard_blend/active_runtime_profile_target_guard_blend.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_profile_export_target_guard_blend/transfer_replay \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

출력은 `active_profile_transfer_replay.csv`,
`active_profile_transfer_summary.txt/json`,
`active_profile_transfer_summary.png`, `per_run_predictions/*`이다. 2026-06-16 결과는
17/18 PASS, validation RMSE mean/max `2.018/3.804m`, target max abs worst-case
`11.691m`였다. 실패는 `bbox_outlier_stress/seed_20260613`에서 target max error가
10m를 넘은 경우였다. 따라서 target-guard blend는 보정 비행별 offline 선택 후보로는
유효하지만, 이 export profile 하나를 universal runtime profile로 고정하는 것은
아직 권장하지 않는다.

여러 exported active runtime profile을 universal fixed profile 후보로 동시에
비교하려면 portfolio audit를 실행한다. 이 감사는 각 profile을 모든 scenario/seed
validation split에 고정 적용하고, pass count, 1852m target worst-case, validation
RMSE worst-case 순으로 순위를 매긴다. Profile apply는 raw stereo-center RDF와
measured disparity만 사용하며, truth는 offline replay metric 계산에만 사용한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_active_profile_portfolio.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_profile_portfolio_audit \
  --profile runtime_guard_smoke=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/operating_mode_export_smoke_runtime_guard/active_runtime_profile.json \
  --profile calibration_smoke=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/operating_mode_export_smoke_calibration/active_runtime_profile.json \
  --profile deployment_audit=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_runtime_profile_from_deployment_audit.json \
  --profile transfer_risk_single=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_transfer_risk_single_profile_smoke/active_runtime_profile.json \
  --profile target_guard_blend=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_profile_export_target_guard_blend/active_runtime_profile_target_guard_blend.json \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

출력은 `active_profile_portfolio_runs.csv`,
`active_profile_portfolio_summary.csv`,
`active_profile_portfolio_recommendation.json`,
`active_profile_portfolio_summary.txt/png`, `per_profile_transfer_replay/*`이다.
2026-06-16 결과에서는 고정 단일 active profile 후보 중 18/18 PASS가 없었다.
partial best는 `target_guard_blend`로 17/18 PASS, target max abs worst-case
`11.691m`, validation RMSE max `3.804m`였다. `deployment_audit`,
`runtime_guard_smoke`, `transfer_risk_single`은 각각 15/18 PASS였고 target max
abs worst-case는 `9.681m`였지만 일부 run에서 raw 대비 RMSE 개선 조건을
만족하지 못했다. 이 결과는 universal fixed profile 하나보다 raw feature 기반
runtime selector/guard 계층이 현재 문제 설정에 더 적합하다는 근거로 기록한다.

runtime selector의 일반화 범위를 더 엄격히 확인하려면 target scenario 전체를
raw-feature library와 candidate source set에서 제외하는 leave-one-scenario-out
audit를 실행한다. 기존 `include_target_in_centroids=false`는 같은 CSV만 제외하고
같은 scenario의 다른 seed를 학습 library에 남겨 두지만,
`--exclude-target-scenario-from-library`는 target scenario를 완전히 제거한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_runtime_measurement_guard_policies.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog_minimal_set.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_policy_audit_loso \
  --candidate-source-modes all_catalog \
  --policies transfer_risk_fallback,centroid_knn_veto_transfer_risk,knn_transfer_risk,nearest_centroid \
  --exclude-target-scenario-from-library \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

2026-06-16 LOSO 결과에서는 `all_catalog+transfer_risk_fallback`이 여전히 가장
낮은 score였지만 14/18 PASS, validation RMSE mean/max `1.580/2.651m`, target max
abs worst-case `10.055m`로 PASS 기준을 만족하지 못했다. 따라서 현재 18/18 PASS
추천은 catalog에 같은 stress family가 존재하는 보정/검증 sweep 내부 조건에서의
결과로 해석해야 하며, 완전히 새로운 sensor-error regime까지 보장하려면 추가
raw feature, 실제 BBox geometry, 또는 calibration-time replay export가 필요하다.

같은 LOSO 조건에서 transfer-risk threshold 자체도 sweep할 수 있다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/sweep_runtime_measurement_guard_thresholds.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog_minimal_set.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_transfer_threshold_sweep_loso \
  --candidate-source-mode all_catalog \
  --rmse-limits-m 0.9,1.1,1.3,1.5,1.8 \
  --target-limits-m 1.5,2.0,2.5,3.0,4.0 \
  --exclude-target-scenario-from-library \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

이 stricter threshold sweep에서는 `rmse_limit=1.8`, `target_limit=2.5`가
15/18 PASS로 기본 LOSO runtime selector보다 1개 run을 더 통과했다. 1852m target
max abs worst-case도 `9.566m`로 PASS 기준 안에 들어왔지만, 남은 실패 3개는 모두
`intrinsic_scale_stress`이며 실패 이유는 raw 대비 validation/train RMSE 개선이
없다는 것이다. 즉 더 느슨한 transfer cluster는 target-window error를 줄일 수
있지만, intrinsic-scale family 자체가 catalog에서 빠진 조건에서는 보정이 실제로
raw보다 나아졌다고 말할 수 없다.

같은 LOSO 조건에서 calibration-time truth replay selector도 확인한다. 이 감사는
보정 비행 truth로 후보 profile을 replay해 선택하지만, target scenario profile은
후보에서 제거한다. 선택된 runtime profile 자체는 여전히 raw RDF/disparity만
소비한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_profile_selection_strategies.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/profile_selection_strategy_audit_all_catalog_loso \
  --candidate-source-mode all_catalog \
  --exclude-target-scenario-from-candidates \
  --split validation \
  --stability-splits train,validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

2026-06-16 결과는 15/18 PASS였다. `bbox_outlier_stress`는 target scenario 후보
없이도 3/3 PASS로 회복됐지만, `intrinsic_scale_stress`는 0/3 PASS였다. 해당
세 seed의 top candidate는 모두 `nominal`이었고 실패 이유는
`no_positive_range_rmse_improvement`였다. 즉 baseline/focal scale mismatch가
새 regime으로 들어오면 같은 family의 보정 profile 또는 실제 calibration-time
profile export가 필요하며, 다른 regime profile로 대체하는 것은 충분하지 않다.

full-catalog calibration replay와 LOSO calibration replay를 scenario별로 비교해
어떤 stress family가 catalog coverage에 필수인지 요약하려면 coverage-gap
후처리를 실행한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/summarize_stereodistance_loso_coverage_gaps.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/loso_coverage_gap_summary
```

출력은 `loso_coverage_gap_scenarios.csv`,
`loso_coverage_gap_summary.json/txt`, `loso_coverage_gap_pass_counts.png`이다.
2026-06-16 결과에서 full gap scenario는 `intrinsic_scale_stress` 하나였다.
full catalog에서는 3/3 PASS, LOSO에서는 0/3 PASS, runtime LOSO에서도 0/3 PASS다.
나머지 scenario는 target family profile을 제거해도 calibration replay LOSO가
3/3 PASS였다. 따라서 다음 후보 탐색은 arbitrary single profile보다
baseline/focal scale mismatch를 식별하거나, 해당 regime에서 별도
calibration-time profile을 export하는 방향이 우선이다.

policy catalog runtime LOSO 결과를 coverage-gap summary에 직접 포함하려면
`--runtime-loso-dir`에
`runtime_measurement_guard_audit_policy_catalog_robust_family_loso`를 넘긴다. 현재
저장된 policy catalog runtime LOSO summary에서는 `intrinsic_scale_stress`가 여전히
0/3 PASS인 full-gap family이고, `bbox_outlier_stress`는 target-window error는
`disparity_noise_stress` fallback으로 10m 이내에 들어오지만 train-side guard 때문에
2/3 PASS로 남아 있다. 이는 runtime 선택 feature의 좌표 계약은 그대로
stereo-center RDF를 유지하면서, 평가용 truth/replay guard만 더 엄격해진 결과다.

coverage gap family가 runtime raw feature만으로 감지 가능한지 확인하려면
feature separability audit를 실행한다. 이 절차는 gap label은 offline
coverage-gap 결과에서 가져오지만, classifier 후보 feature는 raw RDF/disparity/image
center 집계값만 사용한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_gap_feature_separability.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/gap_feature_separability_audit
```

출력은 `gap_feature_separability.csv`,
`gap_feature_separability_summary.json/txt`,
`gap_feature_separability_top_features.png`이다. 2026-06-16 결과에서
`intrinsic_scale_stress`는 `feature_raw_forward_mean_m >= 1145.212m` 기준으로
현재 synthetic sweep 18개 run을 100% 분리했다. effect size는 `5.301`이다.
`feature_raw_range_mean_m`, `feature_raw_forward_std_m`,
`feature_raw_range_std_m`도 100% 분리했다. 다만 raw forward/range 통계는
trajectory 설계에 민감하므로, 이 결과는 곧바로 일반 runtime detector로 확정하지
않고 “calibration required” guard 후보로 기록한다. 실제 비행에서는 동일 trajectory
coverage 또는 별도 trajectory-normalized scale feature가 필요하다.

runtime selector는 intrinsic-scale regime 후보를 더 물리적으로 보기 위해
`measured_disparity_px * raw_forward_m`를 nominal `f_px * baseline_m`과 비교하는
product-ratio 계열 feature도 기록한다. 이 feature는 stereo depth 식
`Z ~= fB/d`의 scale 일관성 proxy이며 runtime truth를 쓰지 않는다. 단순 평균
ratio는 15/18 수준이었지만, robust 통계인
`feature_disparity_forward_product_ratio_p10_to_nominal >= 1.003744`와
`feature_disparity_forward_product_ratio_median_to_nominal >= 1.003915`는 현재
synthetic sweep의 `intrinsic_scale_stress`를 18/18로 분리했다. raw forward/range
통계보다 effect size는 낮지만 trajectory 해석 의존성이 작아 intrinsic-scale
gap guard 후보로 더 적합하다.

이 신호를 실제 selector에 연결한
`intrinsic_scale_gap_transfer_risk` 정책도 비교한다. 이 정책은 product-ratio p10이
감사에서 정한 threshold를 넘고 `intrinsic_scale_stress` source profile이 catalog에
있으면 해당 profile을 우선 선택하고, 없으면 기존 transfer-risk fallback으로
돌아가며 `intrinsic_scale_gap_detected_but_source_unavailable`을 남긴다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_runtime_measurement_guard_policies.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog_minimal_set.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_policy_audit \
  --candidate-source-modes all_catalog \
  --policies transfer_risk_fallback,intrinsic_scale_gap_transfer_risk,centroid_knn_veto_transfer_risk,knn_transfer_risk,nearest_centroid \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

2026-06-16 재실행 결과에서 일반 catalog 조건은 기존
`transfer_risk_fallback`과 새 product-ratio guard 정책이 모두 18/18 PASS,
validation RMSE mean/max `1.089/1.676m`, target max abs worst-case `9.681m`로
동일했다. 새 정책은 `intrinsic_scale_stress` 3개 run을 명시적으로
`intrinsic_scale_stress` profile로 보냈지만, 성능 이득은 없어 복잡도 penalty
때문에 최종 runtime 추천은 여전히 `transfer_risk_fallback`이다. LOSO 조건에서는
새 정책이 intrinsic gap을 감지하지만 source profile이 없어 14/18 PASS에 머물렀다.
따라서 product-ratio guard는 correction 자체보다 “해당 scale family profile이
catalog/calibration에 필요하다”는 runtime-safe 진단 신호로 해석한다.

참고 방법론과 현재 적용 가능 범위:

- Kabsch/Umeyama/orthogonal Procrustes 계열은 paired estimated RDF point와
  truth RDF point 사이의 전역 rotation/translation/scale을 닫힌 형태로 맞추는
  기준 방법이다. 현재 Sim(3), scale+rotation 후보가 이 계열이며, 보정 비행
  truth를 사용하는 offline calibration 단계에만 해당한다.
- Bundle adjustment 계열은 이미지 reprojection error를 직접 최소화해 camera
  pose/intrinsic/3D point를 동시에 조정하는 방법이다. 현재 synthetic CSV는
  좌/우 이미지 feature track이나 full bbox geometry가 아니라 중심/disparity와
  raw RDF만 보존하므로, 지금 pipeline에서는 residual disparity/RDF correction로
  근사한다. 향후 좌/우 image topic 또는 YOLO BBox topic을 저장하면 reprojection
  residual 기반 후보를 추가할 수 있다.
- Markerless stereo extrinsic calibration 계열은 자연 feature의 epipolar error로
  stereo extrinsic을 online/offline refinement하는 방향이다. 이 방법은 좌/우
  영상 feature correspondence가 필요하므로 현재 RTK synthetic mode의
  raw RDF/disparity-only CSV에는 바로 적용하지 않는다.
- LiDAR-camera targetless calibration, mutual-information registration, ICP/GMM
  point-set registration은 scene geometry 또는 dense point cloud가 있을 때
  유효한 비교군이다. 현재 문제는 target drone의 paired RTK truth와 stereo
  estimate가 있는 supervised correction 문제이므로, 우선순위는 낮지만 실제
  depth/point-cloud 센서가 추가되면 후보군으로 확장할 수 있다.

single profile selection audit와 blend audit를 둘 다 실행한 뒤에는 다음
후처리로 배포 관점의 최종 offline 추천을 만들 수 있다. 추천 strategy 이름에는
후보 범위가 포함된다. 예를 들어 deployment-ready set만 썼으면
`single_profile_calibration_replay_minimal_set`, 전체 catalog를 썼으면
`single_profile_calibration_replay_all_catalog`처럼 기록된다.

전체 후처리 pipeline을 한 번에 실행하려면 다음 wrapper를 사용한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_stereodistance_post_sweep_deployment_audit.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set/post_sweep_deployment_audit \
  --candidate-source-mode all_catalog
```

이 wrapper는 profile selection audit, blend audit, deployment audit strategy
summary를 순서대로 실행하고
`post_sweep_deployment_audit_manifest.json`에 최종 추천 전략/정책과 산출물
경로를 기록한다. 빠른 진단만 필요하면 `--skip-blend-audit`로 blend 탐색을
건너뛸 수 있다.

```bash
Simulation/.venv/bin/python Simulation/scripts/summarize_stereodistance_deployment_audits.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set/deployment_audit_strategy_summary
```

출력은 `deployment_audit_strategy_candidates.csv`,
`deployment_audit_strategy_recommendation.json`,
`deployment_audit_strategy_summary.txt`이다. 이 요약은 validation RMSE
평균/최대, 1852m target-window error 평균/최대를 함께 점수화해 단일
calibration replay profile selection과 blend policy 중 어느 쪽을 offline
배포 후보로 둘지 비교한다. 같은 pass count에서 점수 차이가 기본 0.01m 이내면
과적합과 운용 복잡도를 피하기 위해 더 단순한 single-profile 전략을 우선한다.
여기서도 truth는 audit/selection 단계에서만 사용되며, 실제 runtime profile은
raw RDF/disparity와 고정 보정 파라미터만 사용한다.

post-sweep deployment audit의 추천을 새 보정 비행 CSV에 바로 적용해 active
StereoDistance runtime profile을 export하려면 다음 wrapper를 사용한다. 추천이
single profile이면 calibration replay selector를, blend policy이면 blend
explorer를 호출하고, 마지막에 promotion check로 schema/load/runtime input
계약을 확인한다. 이 단계에서도 CSV truth는 offline 선택/검증에만 쓰이며,
export된 profile은 raw stereo-center RDF와 disparity만 runtime 입력으로 쓴다.

```bash
Simulation/.venv/bin/python Simulation/scripts/export_stereodistance_profile_from_deployment_audit.py \
  --deployment-audit-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/post_sweep_deployment_audit/deployment_audit_strategy_summary/deployment_audit_strategy_recommendation.json \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/scenario_profile_catalog_minimal_set.json \
  --input-csv Simulation/results/rtk_stereo_robustness_sweep_minimal_set/combined_sensor_stress/seed_20260614/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set/active_profile_export_from_deployment_audit \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep_minimal_set/active_runtime_profile_from_deployment_audit.json
```

출력은 `deployment_audit_profile_export_summary.json`,
`deployment_audit_profile_export_summary.txt`, `promotion_check/*`, 그리고
선택 경로별 `single_profile_export/*` 또는 `blend_profile_export/*`이다.
추천과 다른 scope/policy를 강제로 시험하려면 `--candidate-source-mode`와
`--selection-policy`를 명시한다.

pipeline의 train/validation split은 `--target-range-m`과
`--target-window-m`을 기준으로 target-window sample을 stratify한다.
target-window sample이 2개 이상 있으면 train과 validation 양쪽에 최소
1개 이상 남겨 1852m PASS 조건을 실제 validation set에서 평가할 수 있게
한다. target-window sample 자체가 부족한 입력 CSV에서는
`method_stability_gate.csv`가 `*_target_window_missing_in_some_seeds` 사유로
PASS를 막는다.

synthetic mode는 기본적으로 `--synthetic-target-oversample-count 12`를
사용해 1852m target-window 근방 sample을 추가로 배치한다. 이 값은
multi-seed wrapper에도 같은 이름의 옵션으로 전달할 수 있다. 목적은
보정법 점수를 유리하게 만드는 것이 아니라, 1852m PASS 조건이 validation
set에서 충분한 표본으로 평가되도록 하는 것이다. 실제 CSV mode에서는 입력
비행 데이터 자체가 target-window 관측을 충분히 포함해야 한다.

실제 보정 비행 CSV를 후처리하려면, `/fmu/out/vehicle_local_position`,
좌/우 카메라 image 또는 synthetic BBox 중심, YOLO BBox 결과를 먼저 명시적
축 필드가 있는 CSV로 저장한 뒤 실행한다.

실제 topic interface contract는 synthetic mode와 분리한다. 기본
calibration/evaluation truth source는 `/fmu/out/vehicle_local_position`일 수 있지만,
PX4 NED만으로는 평가용 `truth_rdf`가 되지 않는다. PX4 NED, RTK/GNSS world,
Gazebo world/model pose, 외부 tracker, 또는 stereo-frame truth topic 중 어떤 source를
쓰더라도 `T_source_to_stereo_rdf` 또는 이미 stereo-center RDF임을 증명하는 producer
contract가 필요하다. 좌/우 `camera image topic`은 raw stereo image source,
`YOLO BBox topic`은 left/right bbox center와 confidence source, disparity topic은
measured disparity source다. runtime input은 이 topic들에서 생성된 stereo-center
RDF(`right/down/forward`)와 measured disparity뿐이며, PX4/RTK/NED/world truth,
validation RMSE, target-window error는 runtime input으로 넣지 않는다.

라이브 ROS 2 토픽에서 calibration CSV를 직접 만들려면 다음 recorder를
사용한다. 이 recorder는 PX4 `VehicleLocalPosition.x/y/z`를 NED
`north/east/down`으로만 해석하며, PX4 NED를 `truth_right_m`,
`truth_down_m`, `truth_forward_m`으로 자동 변환하지 않는다. StereoDistance
`/stereo/estimated_position`은 public FRD+range
`forward/right/down/range`로 해석한 뒤 CSV에는 named RDF/NED 필드만 쓴다.
Gazebo synthetic truth를 같이 기록해야 하는 경우에는
`--sim-truth-topic /ground_truth_position`을 추가한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/ros2_calibration_csv_recorder.py \
  --output-csv Simulation/results/calibration_flight/telemetry.csv \
  --truth-topic /fmu/out/vehicle_local_position \
  --estimated-topic /stereo/estimated_position \
  --detections-topic /stereo/detections \
  --disparity-topic /stereo/disparity \
  --hz 10
```

Gazebo/PX4/StereoDistance launch와 함께 calibration CSV를 기록하려면 launch
인자로 recorder를 켠다. 실제 PX4 토픽 대신 Gazebo synthetic truth
`/ground_truth_position`을 calibration truth로 기록하려면
`calibration_sim_truth_topic:=/ground_truth_position`을 지정한다.

평소 운용 스크립트를 쓸 때는 다음처럼 실행한다.

```bash
./run_operator_view.sh \
  --record-calibration-csv Simulation/results/calibration_flight/telemetry.csv \
  --calibration-sim-truth
```

기록 전에 필수 ROS 토픽이 떠 있는지 확인하려면 별도 터미널에서 readiness
check를 실행한다. PX4 truth 대신 Gazebo synthetic truth를 쓰는 경우
`--sim-truth-topic /ground_truth_position`을 지정한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/calibration_topic_readiness_check.py \
  --sim-truth-topic /ground_truth_position \
  --output-dir Simulation/results/calibration_flight/topic_readiness
```

```bash
ros2 launch stereo_ground_sim stereo_ground_sim.launch.py \
  record_calibration_csv:=true \
  calibration_csv_path:=results/calibration_flight/telemetry.csv \
  calibration_truth_topic:=/fmu/out/vehicle_local_position \
  calibration_sim_truth_topic:=/ground_truth_position
```

```bash
Simulation/.venv/bin/python Simulation/scripts/rtk_stereo_calibration_pipeline.py \
  --mode csv \
  --input-csv Simulation/results/calibration_flight/telemetry.csv \
  --output-dir Simulation/results/rtk_stereo_calibration_from_csv
```

pipeline 실행과 최종 산출물 audit을 한 번에 끝내려면 다음 단일 Python entrypoint를
사용한다. 실제 CSV가 준비되기 전에는 `--mode synthetic`으로 동일한 산출물 계약을
smoke test할 수 있다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_calibration_with_audit.py \
  --output-dir Simulation/results/rtk_stereo_calibration_from_csv \
  --mode csv \
  --input-csv Simulation/results/calibration_flight/telemetry.csv
```

이 wrapper는 `rtk_stereo_calibration_pipeline.py`에 나머지 인자를 그대로 전달하고,
성공 후 `audit_rtk_stereo_calibration_outputs.py`를 실행한다. audit 실패 시 wrapper도
실패하며, 산출물은 `output_audit/rtk_stereo_calibration_output_audit.json/txt` 및
`output_audit/rtk_stereo_calibration_with_audit_summary.json/txt`이다. wrapper도
runtime truth를 사용하지 않으며, truth는 calibration/evaluation 단계에서만 쓰인다.
단일-run 최종 추천은 validation range RMSE가 raw보다 낮고 1852m target-window
abs mean error가 10m 이하이며, RDF right/down/forward 축별 validation RMSE가 raw
대비 후퇴하지 않는 후보만 대상으로 한다.

CSV 품질을 먼저 확인하려면 preflight를 실행한다. 이 단계는 pipeline loader와
같은 named NED/RDF/FRD 계약으로 usable row 수, skip 사유, 1852m target
window 표본 수를 보고한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/calibration_csv_preflight.py \
  --input-csv Simulation/results/calibration_flight/telemetry.csv \
  --output-dir Simulation/results/calibration_flight/preflight \
  --target-range-m 1852 \
  --target-window-m 30 \
  --min-target-count 3
```

기록 CSV에서 후보 탐색, runtime profile export, offline replay, PASS gate용
`metrics.json`까지 한 번에 만들려면 다음 runner를 사용한다. 실제 CSV가 아직
없으면 `--mode synthetic`으로 같은 경로를 smoke test할 수 있다.
CSV mode에서는 runner가 위 preflight를 자동으로 실행하고 결과를
`preflight/`와 `metrics.json`에 포함한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_estimated_to_truth_calibration_exp.py \
  --mode csv \
  --input-csv Simulation/results/calibration_flight/telemetry.csv \
  --output-dir Simulation/results/estimated_to_truth_calibration_exp
```

운용 스크립트로 기록한 CSV를 바로 후처리하려면 wrapper를 사용할 수 있다.
CSV 경로를 생략하면 `results/calibration_flight` 아래 최신 CSV를 찾는다.

```bash
./run_estimated_to_truth_from_csv.sh \
  results/calibration_flight/telemetry.csv
```

후처리 결과의 runtime profile을 측정 실행에 적용하려면 다음 wrapper를
사용한다. profile 경로를 생략하면 `results/` 아래 최신
`active_runtime_profile.json` 또는 `sim3_runtime_profile.json`을 찾는다.

```bash
./run_operator_view_with_calibration_profile.sh
```

검증된 deployment package를 바로 쓰려면 package directory를 지정한다. 이
wrapper는 package 안의 `active_runtime_profile.json`을 기존
`run_operator_view.sh --bbox-adjustment` 경로로 넘기며, runtime 입력은
stereo-center RDF와 measured disparity뿐이다.

```bash
./run_operator_view_with_calibration_profile.sh \
  --package-dir results/rtk_stereo_acceptance_matrix_summary/deployment_package
```

runtime 관측 외란 조건처럼 profile을 여러 개 보관해야 하는 경우에는 live truth가
아니라 운용자가 명시한 deployment/scenario label로만 선택한다. 예를 들어
`Simulation/results/rtk_stereo_runtime_observation_stress_EXP_20260617/runtime_profile_label_selector.json`
은 `nominal`/`current_default`를 기존 active package로, `runtime_observation_stress`
및 stress scenario label을 observation-stress package로 매핑한다. 이 selector의
입력은 `deployment_label`, `scenario_label`, `scenario_name` 같은 배포 메타데이터
뿐이며, PX4/RTK truth, validation error, replay winner는 runtime 선택 입력으로
금지된다.

```bash
./run_operator_view_with_calibration_profile.sh \
  --selector-json Simulation/results/rtk_stereo_runtime_observation_stress_EXP_20260617/runtime_profile_label_selector.json \
  --deployment-label runtime_observation_stress \
  -- --no-px4 yolo_target_hz:=0.0
```

wrapper는 selector 검사를 먼저 수행한 뒤 선택된 label의 `package_paths` 또는
`profile_paths`를 실제 `run_operator_view.sh --bbox-adjustment <profile>` 명령으로
해석한다. selector 해석 smoke 결과는
`Simulation/results/rtk_stereo_runtime_observation_stress_EXP_20260617/operator_view_launch_check_selector_stress.json|txt`
에 PASS로 기록되어 있다. 이 단계에서도 runtime profile 입력은 stereo-center RDF
`right/down/forward`와 measured disparity이고, public StereoDistance topic은
FRD+range `forward/right/down/range`이다.

selector를 실행하지 않고 선택 결과만 파일로 남기려면:

```bash
Simulation/.venv/bin/python Simulation/scripts/select_stereodistance_runtime_profile_by_label.py \
  --selector-json Simulation/results/rtk_stereo_runtime_observation_stress_EXP_20260617/runtime_profile_label_selector.json \
  --deployment-label runtime_observation_stress \
  --output-json Simulation/results/rtk_stereo_runtime_observation_stress_EXP_20260617/runtime_profile_label_selection_stress.json \
  --output-txt Simulation/results/rtk_stereo_runtime_observation_stress_EXP_20260617/runtime_profile_label_selection_stress.txt
```

배포자가 볼 수 있는 selector frontier를 CSV/JSON/TXT로 요약하려면:

```bash
Simulation/.venv/bin/python Simulation/scripts/export_stereodistance_runtime_deployment_frontier.py \
  --selector-json Simulation/results/rtk_stereo_runtime_observation_stress_EXP_20260617/runtime_profile_label_selector.json \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_EXP_20260617/runtime_deployment_frontier
```

이 산출물은 `runtime_deployment_frontier_labels.csv`에
`nominal -> current_active`, `runtime_observation_stress -> stress_active` 같은
label route를 기록하고, `runtime_deployment_frontier_profiles.csv`에 각 deployable
profile의 method, SHA-256, package path, readiness status를 기록한다. selection
입력은 배포/scenario label뿐이며 runtime truth/PX4/RTK/NED 값은 포함하지 않는다.

명시 profile을 쓰려면:

```bash
./run_operator_view_with_calibration_profile.sh \
  --profile-json results/estimated_to_truth_calibration_exp/sim3_runtime_profile.json
```

Runner 출력:

- pipeline 기본 산출물: `method_comparison.csv`, `validation_predictions.csv`,
  `target_1852m_stats.csv`, PNG 3종, `summary.txt`
- StereoDistance runtime profile: `sim3_runtime_profile.json`
- runtime profile replay: `profile_replay/profile_replay_*.csv|txt|png`
- StereoDistance promotion check:
  `promotion_check/promotion_check.json|txt|csv`
- PASS gate/audit 산출물: `metrics.json`, `promotion_summary.txt`,
  `result_audit.json`, `result_audit.txt`

CSV 입력은 최소한 다음 중 하나의 truth 계약과 raw RDF 또는 BBox/시차 계약을
가져야 한다.

- truth RDF: `truth_right_m`, `truth_down_m`, `truth_forward_m`
- recorded-style truth RDF aliases: `gt_right_m`, `gt_down_m`,
  `gt_forward_m`
- truth NED: `truth_north_m`, `truth_east_m`, `truth_down_ned_m`
- raw RDF: `raw_right_m`, `raw_down_m`, `raw_forward_m`
- recorded-style raw RDF aliases: `estimated_right_m`, `estimated_down_m`,
  `estimated_forward_m`
- 또는 triangulation 입력: `left_u_px`, `left_v_px`, `measured_disparity_px`
- `measured_disparity_px`가 없으면 `left_cx_px`와 `right_cx_px`의 차이를
  measured disparity로 사용할 수 있다.

비교하는 후보 보정법:

- `global_rotation`: `p_corr = R_corr * p_raw`
- `global_scale_rotation`: `p_corr = s * R_corr * p_raw`
- `sim3`: `p_corr = s * R_corr * p_raw + t`
- `diagonal_affine_rdf`, `identity_shrunk_diagonal_affine_rdf_lambda0p01/1`:
  RDF 각 축별 `p_corr_axis = gain_axis * p_raw_axis + bias_axis`만 fitting하는
  6자유도 후보. 목적은 bbox center와 target origin 격차 또는 축별 scale/bias가
  남는지 확인하는 것이며, shrink 후보는 gain을 identity에 묶어 과적합을
  줄인다. 출력은 `estimated_to_truth_affine_rdf` profile이고 PASS 기준은 다른
  후보와 동일하게 validation RMSE 개선, 1852m 오차, stability gate이다.
- `affine_rdf`, `ridge_affine_rdf`: RDF 벡터에서 `p_corr = A * p_raw + t`를
  직접 fitting하는 12자유도 affine 후보. Sim(3)보다 표현력이 크므로
  validation/1852m/stability gate를 통과하는지 확인해야 한다. StereoDistance
  runtime에서는 `estimated_to_truth_affine_rdf` profile로 적용할 수 있지만,
  과적합 위험 때문에 단일 run 성능이 아니라 multi-seed/robustness gate 통과를
  기준으로 승격한다.
- `affine_binned_residual_disparity`: affine RDF 보정 후 train set의
  disparity 구간별 forward residual median LUT를 만들고, runtime에서는
  measured disparity에 대해 선형 보간된 residual을 추가 적용한다.
- `affine_binned_residual_raw_forward`,
  `affine_binned_residual_raw_forward_bins11`: affine RDF 보정 후 train set의
  raw RDF forward 거리 구간별 forward residual median LUT를 만들고,
  runtime에서는 raw RDF forward 값으로 선형 보간된 residual을 추가 적용한다.
  disparity-key LUT가 focal/baseline scale stress에 민감한지 비교하기 위한
  후보이다.
- `affine_monotone_binned_residual_raw_forward`,
  `affine_monotone_binned_residual_raw_forward_bins11`: affine RDF 보정 후
  raw RDF forward 거리 구간별 residual median을 weighted PAVA isotonic
  regression으로 단조화한다. 거리 증가에 따라 보정 residual이 불안정하게
  뒤집히는 과적합을 줄이는 보수적 LUT 후보이다.
- `affine_range_normalized_binned_residual_raw_forward`,
  `affine_range_normalized_binned_residual_raw_forward_bins11`: affine RDF 보정
  후 forward residual 자체가 아니라 `residual / raw_forward` 비율을 구간별로
  fitting하고, runtime profile에는 raw RDF forward 거리별 meter residual LUT로
  저장한다. 거리 비례 scale/bias 성분이 클 때 raw-forward LUT보다 안정적인지
  비교하는 후보이다.
- `affine_inverse_depth_binned_residual_raw_forward`,
  `affine_inverse_depth_binned_residual_raw_forward_bins11`: affine RDF 보정 후
  `1 / truth_forward - 1 / affine_forward` inverse-depth delta를 구간별로
  fitting하고, runtime profile에는 raw RDF forward 거리별 meter residual LUT로
  저장한다. stereo의 `Z = fB / d` 관계에서 disparity scale/bias 오차가
  inverse depth 공간에서 더 단순해지는지 확인하는 후보이다.
- `affine_quadratic_residual_raw_forward_lut`: affine RDF 보정 후
  `residual = a * raw_forward^2 + b * raw_forward + c` 형태의 ridge polynomial을
  fitting하고, runtime profile에는 raw RDF forward 거리별 meter residual LUT로
  저장한다. 시차 bias로 인한 stereo depth 오차가 장거리에서 `Z^2`에 가깝게
  커지는 경우를 겨냥한 물리형 후보이다.
- `affine_quadratic_local_residual_raw_forward_lut`,
  `affine_quadratic_local_residual_raw_forward_lut_bins3`,
  `affine_quadratic_local_residual_raw_forward_lut_bins5`,
  `affine_quadratic_local_residual_raw_forward_lut_shrink025`,
  `affine_quadratic_local_residual_raw_forward_lut_shrink075`: affine RDF 보정 후
  2차 raw-forward residual로 큰 stereo depth bias를 먼저 설명하고, 남은
  residual-of-residual을 shrink된 구간별 median LUT로 더한다. runtime에는
  합쳐진 meter residual LUT 하나만 저장한다. 기본 이름은 local residual
  shrink `0.5`/7-bin이고, suffix 후보는 `0.25`, `0.75` shrink 강도와
  3/5/7-bin local LUT 복잡도를 grid-search한다.
- `affine_binned_residual_disparity_bins5/9/11`: 같은 affine+forward
  residual LUT 구조에서 bin 수를 grid-search하여 과소/과다 smoothing을
  validation 기준으로 비교한다.
- `affine_smoothed_binned_residual_disparity`,
  `affine_smoothed_binned_residual_disparity_bins11`: affine+forward residual
  LUT의 bin median을 global median 쪽으로 shrink하고 인접 bin smoothing을
  적용해 outlier/희소 bin 과적합을 줄이는 후보
- `affine_capped_binned_residual_disparity`,
  `affine_capped_binned_residual_disparity_bins11`: affine+forward residual
  LUT의 bin median을 train residual MAD와 absolute-residual quantile로
  winsorized cap 처리해 calibration stress 조건 전이에서 과도한 residual
  gain이 생기는지 비교하는 후보
- `affine_binned_rdf_residual_disparity`: affine RDF 보정 후 train set의
  disparity 구간별 residual median LUT를 RDF 각 축마다 만들고, runtime에서는
  measured disparity에 대해 축별 선형 보간 residual을 추가 적용한다.
- `trimmed_sim3`: 큰 residual sample을 제외한 뒤 Sim(3)를 다시 fitting하는
  consensus/least-trimmed 계열 강건 정합
- `ransac_sim3`: 여러 최소 subset에서 Sim(3) 가설을 만들고 consensus set으로
  다시 fitting하는 RANSAC 계열 강건 정합
- `huber_sim3`: residual norm에 Huber weight를 적용하는 iteratively
  reweighted Sim(3) 정합
- `tukey_sim3`: 큰 residual sample의 weight를 0에 가깝게 낮추는 Tukey
  biweight redescending M-estimator 기반 iteratively reweighted Sim(3) 정합
- `range_weighted_sim3`: 1852m 목표거리 근방 sample을 더 크게 반영하는
  weighted Sim(3) 정합
- `sim3_piecewise_raw_forward_bins3`,
  `sim3_piecewise_raw_forward_bins5`: raw RDF forward 거리 분위수 구간마다
  Sim(3) node를 따로 fitting하고, runtime에서는 raw_forward_m으로 인접 node의
  transformed RDF point를 선형 보간한다. 목적은 자세/origin/bbox-center 격차가
  거리 구간에 따라 달라질 때 단일 전역 Sim(3)보다 global estimated-to-truth
  오차를 줄일 수 있는지 확인하는 것이다. 입력 가정은 calibration-time PX4/RTK
  truth와 raw stereo-center RDF이고, runtime 출력 계약은 기존
  `estimated_to_truth_sim3` profile과 같은 RDF 보정값이다. PASS 판단은 다른
  후보와 동일하게 validation RMSE, 1852m target-window error, raw 대비 개선,
  모델 복잡도/안정성 gate를 기준으로 한다.
- `sim3_blended_piecewise_raw_forward_bins3_alpha025/050/075`,
  `sim3_blended_piecewise_raw_forward_bins5_alpha025/050/075`: 전역 Sim(3)
  출력과 raw_forward 구간별 Sim(3) 출력 RDF를 calibration-time에 정한 alpha로
  convex blend한다. 목적은 piecewise registration의 낮은 validation RMSE를
  유지하면서 구간별 자세/origin 보정 자유도를 줄여 실제 StereoDistance runtime
  profile로 더 안정적으로 배포 가능한지 확인하는 것이다. runtime 입력은 raw
  stereo-center RDF와 raw_forward_m뿐이며 PX4/RTK truth는 사용하지 않는다.
- `huber_range_weighted_sim3`: Huber 강건 가중과 목표거리 가중을 함께 적용한
  Sim(3) 정합
- `tukey_range_weighted_sim3`: Tukey biweight 강건 가중과 목표거리 가중을
  함께 적용한 Sim(3) 정합
- `soft_l1_sim3`, `cauchy_sim3`: `scipy.optimize.least_squares` robust loss로
  Sim(3) 파라미터를 직접 최적화하는 M-estimator 계열 정합
- `global_disparity_linear`: `d_corr = a * d_meas + b`
- `global_disparity_quadratic`: `d_corr = a2 * d_meas^2 + a1 * d_meas + b`
- `global_inverse_depth_linear`: `1 / forward_corr = a * (1 / raw_forward) + b`
  형태로 raw RDF ray를 재스케일한다. stereo의 장거리 depth error가 range보다
  inverse depth에서 더 선형일 수 있다는 기하 가정을 시험하는 저복잡도 후보이며,
  runtime에서는 truth 없이 raw stereo-center RDF만 사용한다.
- `inverse_depth_linear_then_sim3`: 위 inverse-depth ray 재스케일 뒤 Sim(3)로
  자세/origin/scale 오차를 정합한다. runtime profile에는 inverse-depth
  coefficients와 Sim(3) 파라미터만 저장되며, raw RDF와 고정 파라미터만 사용한다.
- `huber_global_inverse_depth_linear`, `huber_inverse_depth_linear_then_sim3`:
  `1/raw_forward -> 1/truth_forward` 선형 fit에 Huber IRLS를 적용해 일부
  보정 비행 sample의 disparity/bbox outlier 영향이 전체 inverse-depth 계수를
  끌고 가지 못하게 한다. 목적은 저복잡도 inverse-depth 후보의 장거리 안정성을
  유지하면서 calibration-time 이상치 민감도를 낮추는 것이다. 출력은 기존
  inverse-depth coefficients 또는 inverse-depth+Sim(3) profile이고, runtime
  입력은 raw stereo-center RDF와 고정 파라미터뿐이다.
- `target_weighted_global_inverse_depth_linear`,
  `target_weighted_inverse_depth_linear_then_sim3`: inverse-depth ray correction
  fitting에서 1852m 근방 train sample에 더 큰 가중치를 준다. target 가중치는
  calibration-time fitting에만 쓰이고, runtime profile은 raw RDF와 고정
  inverse-depth coefficients 및 선택적으로 Sim(3) 파라미터만 사용한다.
- `theil_sen_global_inverse_depth_linear`,
  `theil_sen_inverse_depth_linear_then_sim3`,
  `target_weighted_theil_sen_global_inverse_depth_linear`,
  `target_weighted_theil_sen_inverse_depth_linear_then_sim3`: inverse-depth
  선형 mapping을 least-squares 대신 Theil-Sen median-slope 추정으로 fitting한다.
  목적은 bbox/disparity outlier가 일부 섞인 보정 비행에서 장거리 inverse-depth
  계수가 한두 샘플에 끌리는지 확인하는 것이다. target-weighted 변형은 1852m
  근방 sample weight를 median-slope와 median residual 계산에 반영한다. 이 계열도
  calibration-time truth로 계수만 추정하고, runtime 입력은 raw stereo-center
  RDF(`right,down,forward`)와 고정 inverse-depth coefficients, 선택적 Sim(3)
  파라미터뿐이다.
- `target_weighted_huber_global_inverse_depth_linear`,
  `target_weighted_huber_inverse_depth_linear_then_sim3`: 1852m 근방 sample
  가중과 Huber IRLS를 함께 적용한다. 이 계열은 target-window 성능을 올리되
  sparse/outlier sample 하나에 profile이 과도하게 맞춰지는지를 validation split,
  1852m 근방 error, train/validation gap, 축별 RMSE regression으로 검증한다.
- `target_weighted_global_inverse_depth_linear_sigma060/250`,
  `target_weighted_inverse_depth_linear_then_sim3_sigma060/250`: target-window
  weighting의 거리 폭을 grid-search한다. 기본 target-weighted 후보는
  `sigma=120m`이고, `060`은 더 좁은 1852m 집중, `250`은 더 넓은 거리 안정성을
  시험한다. 선택은 validation RMSE, 1852m error, train/validation gap, 복잡도
  기준으로만 이루어진다.
- `target_weighted_huber_joint_disparity_ray_rotation_nls_sigma030/045/060/090/120/160/250`:
  corrected disparity 선형항과 RDF rotation을 함께 fitting하는 Huber NLS 후보에
  1852m target-range weight 폭 grid를 적용한다. 기존 robustness 결과에서
  `sigma060`이 active PASS family가 되었으므로, planner의 `exploit_active_family`
  지시에 따라 30/45/90/120m 후보를 추가해 target 집중 강도와 전체 validation
  안정성 사이의 trade-off를 더 촘촘히 탐색한다. runtime profile은 고정된
  disparity correction 계수와 RDF rotation만 사용하며, PX4/RTK/NED truth는
  calibration/evaluation에만 사용된다.
- `huber_joint_disparity_ray_rotation_nls_fscale0p75/1p0/2p0/3p0`,
  `target_weighted_huber_joint_disparity_ray_rotation_nls_sigma060_fscale0p75/1p0/2p0/3p0`:
  Huber robust loss의 residual cutoff(`f_scale_m`)를 grid-search한다. 목적은
  bbox center jitter, disparity bias drift, mixed runtime observation stress에서
  이상치 억제 강도가 너무 약하거나 강할 때 validation RMSE, 1852m 근방 max
  error, RDF axis non-regression이 어떻게 변하는지 자동으로 확인하는 것이다.
  후보 선택과 PASS 판정은 train RMSE가 아니라 validation split, 1852m target
  window, train/validation gap, model complexity, RDF axis regression gate를
  기준으로 한다. runtime 입력 계약은 기존 joint 후보와 동일하게 stereo-center
  RDF(`right,down,forward`)와 measured disparity뿐이며, PX4 local NED truth는
  calibration/evaluation 단계에서만 사용된다.
- `target_shrunk_global_inverse_depth_linear_shrink025/040/050/060/075`,
  `target_shrunk_inverse_depth_linear_then_sim3_shrink025/040/050/060/075`: global
  inverse-depth coefficients와 target-weighted coefficients를 25/40/50/60/75% 비율로
  convex blend한다. 목적은 1852m target-window 개선과 validation 전체 안정성
  사이의 trade-off를 자동 탐색하는 것이다. shrink 값도 calibration-time
  후보 선택에만 쓰이며 runtime 입력 계약은 동일하다.
- `target_shrunk_huber_global_inverse_depth_linear_shrink025/040/050/060/075`,
  `target_shrunk_huber_inverse_depth_linear_then_sim3_shrink025/040/050/060/075`: global
  Huber inverse-depth coefficients와 target-weighted Huber coefficients를
  convex blend한다. 목적은 Huber의 이상치 억제 효과를 유지하면서 1852m
  target-window로 적당히 이동하는 지점을 자동 탐색하는 것이다. target-weighted
  Huber가 validation 전체나 1852m max error에서 과도하게 흔들릴 때, shrink
  후보가 더 안정적인 runtime profile로 선택될 수 있는지 평가한다.
- `disparity_linear_then_sim3`, `disparity_quadratic_then_sim3`: train set에서
  measured disparity를 1차/2차로 보정해 raw RDF ray를 먼저 재스케일하고,
  그 corrected ray를 PX4/RTK truth RDF에 Sim(3)로 정합한다. runtime에서는
  measured disparity와 raw RDF로 ray scale을 적용한 다음 고정 Sim(3)을
  사용하므로 truth를 요구하지 않는다.
- `disparity_linear_then_sim3_quadratic_residual_raw_forward_lut`,
  `disparity_quadratic_then_sim3_quadratic_residual_raw_forward_lut`,
  `disparity_linear_then_sim3_inverse_depth_residual_raw_forward_lut`,
  `disparity_quadratic_then_sim3_inverse_depth_residual_raw_forward_lut`: 위
  disparity-corrected ray + Sim(3) 후보 뒤에 raw RDF forward 기반 2차 residual
  LUT 또는 inverse-depth residual LUT를 추가한다. inverse-depth 변형은 stereo
  depth bias가 range보다 inverse depth에서 더 선형이라는 기하 가정에 맞춰
  `1 / truth_forward - 1 / base_forward` residual을 fitting한다. 목적은 disparity
  보정으로 validation RMSE를 낮추면서, 장거리 residual curve로 1852m
  target-window 안정성도 함께 개선되는지 확인하는 것이다.
- `target_weighted_disparity_linear_then_sim3`,
  `target_weighted_disparity_quadratic_then_sim3`,
  `target_weighted_disparity_linear_then_sim3_quadratic_residual_raw_forward_lut`,
  `target_weighted_disparity_quadratic_then_sim3_quadratic_residual_raw_forward_lut`,
  `target_weighted_disparity_linear_then_sim3_inverse_depth_residual_raw_forward_lut`,
  `target_weighted_disparity_quadratic_then_sim3_inverse_depth_residual_raw_forward_lut`,
  `target_weighted_disparity_linear_then_sim3_target_weighted_quadratic_residual_raw_forward_lut`,
  `target_weighted_disparity_quadratic_then_sim3_target_weighted_quadratic_residual_raw_forward_lut`,
  그리고 `_sigma040/160`, `_shrink025/050/075` 조합:
  disparity-corrected ray + Sim(3) 계열의 fitting에서 1852m 근방 train sample에
  더 큰 가중치를 준다. `_target_weighted_quadratic_residual_raw_forward_lut`
  변형은 Sim(3) 뒤 forward residual curve에도 별도 target-window 가중치를
  적용해 1852m worst-case error를 낮출 수 있는지 검증한다. residual weight
  sigma는 기본 80m이며 `_sigma040/160` 변형은 더 좁거나 넓은 target-window
  강조를 비교한다. `_shrink025/050/075` 변형은 해당 residual을 25%/50%/75%만
  적용해 validation RMSE 손실과 1852m worst-case 개선 사이의 trade-off를
  탐색한다. 이 가중치는 calibration-time fitting에만 쓰이며, runtime profile은
  measured disparity, raw RDF, fixed Sim(3)/LUT 파라미터만 사용한다.
- `sim3_residual_disparity`: Sim(3) 보정 후 train set에서 fitting한
  disparity 기반 forward residual을 추가 적용
- `sim3_residual_disparity_deg0`, `sim3_residual_disparity_deg1`: Sim(3)
  보정 후 평균 bias 또는 1차 disparity residual만 적용하는 보수적 저차 후보
- `sim3_target_weighted_residual_disparity_deg1`,
  `sim3_target_weighted_residual_disparity`: Sim(3) 보정 후 1852m 목표거리
  근방 sample을 더 크게 반영한 forward residual을 추가 적용
- `sim3_ridge_residual_disparity`: Sim(3) 보정 후 ridge-regularized polynomial
  forward residual을 추가 적용
- `sim3_binned_residual_disparity`: Sim(3) 보정 후 train set의 disparity
  구간별 forward residual median LUT를 만들고, runtime에서는 measured
  disparity에 대해 선형 보간된 residual을 추가 적용
- `sim3_binned_residual_raw_forward`,
  `sim3_binned_residual_raw_forward_bins11`: Sim(3) 보정 후 train set의 raw
  RDF forward 거리 구간별 forward residual median LUT를 만들고,
  runtime에서는 raw RDF forward 값으로 선형 보간된 residual을 추가 적용한다.
- `sim3_kernel_smoothed_residual_raw_forward_sigma060/120/250`: Sim(3) 보정 후
  raw RDF forward 거리와 forward residual의 관계를 Gaussian-kernel
  Nadaraya-Watson regression으로 smoothing한 뒤 quantile support LUT로
  저장한다. 목적은 binned median LUT의 bin 경계 민감도를 줄이고, 거리별
  residual curve가 매끄럽게 전이되는지 확인하는 것이다. `sigma`는 meter
  단위 bandwidth이며 calibration-time train set에만 쓰인다. runtime 출력
  계약은 기존 `forward_residual_raw_forward_lut`와 같고, raw RDF forward와
  fixed LUT만 사용한다. PASS 기준은 다른 후보와 동일하게 validation RMSE,
  1852m 오차, replay/promotion check이다.
- `sim3_monotone_binned_residual_raw_forward`,
  `sim3_monotone_binned_residual_raw_forward_bins11`: Sim(3) 보정 후 raw RDF
  forward residual LUT를 weighted PAVA isotonic regression으로 단조화한다.
  stereo depth calibration에서 거리별 residual curve가 완만해야 한다는
  안정성 가정을 둔 후보이며, validation/replay에서만 승격된다.
- `sim3_range_normalized_binned_residual_raw_forward`,
  `sim3_range_normalized_binned_residual_raw_forward_bins11`: Sim(3) 보정 후
  `residual / raw_forward` 비율을 구간별로 fitting하고 meter residual LUT로
  환산해 저장한다. range-proportional residual 가정이 실제 StereoDistance
  오차에 더 맞는지 validation/replay 기준으로 비교한다.
- `sim3_inverse_depth_binned_residual_raw_forward`,
  `sim3_inverse_depth_binned_residual_raw_forward_bins11`: Sim(3) 보정 후
  `1 / truth_forward - 1 / sim3_forward` inverse-depth delta를 구간별로
  fitting하고 meter residual LUT로 환산해 저장한다. runtime에서는 truth 없이
  raw RDF forward 값으로 LUT를 조회한다.
- `sim3_quadratic_residual_raw_forward_lut`: Sim(3) 보정 후 raw RDF forward에
  대한 2차 residual polynomial을 fitting하고 meter residual LUT로 저장한다.
  disparity bias가 거리 오차를 `Z^2` 형태로 키우는 stereo depth 근사에
  기반한 후보이며, runtime에서는 여전히 raw RDF와 fixed LUT만 사용한다.
- `sim3_quadratic_local_residual_raw_forward_lut`,
  `sim3_quadratic_local_residual_raw_forward_lut_bins3`,
  `sim3_quadratic_local_residual_raw_forward_lut_bins5`,
  `sim3_quadratic_local_residual_raw_forward_lut_shrink025`,
  `sim3_quadratic_local_residual_raw_forward_lut_shrink075`: Sim(3) 보정 후 2차
  raw-forward residual과 shrink된 local residual-of-residual LUT를 결합한다.
  목적은 물리형 quadratic 후보가 놓치는 거리대별 국소 bias를 제한적으로
  보정하는 것이다. 기본 이름은 local residual shrink `0.5`이고, suffix
  후보는 더 보수적인 `0.25`, 더 적극적인 `0.75`, 그리고 3/5/7-bin local LUT
  복잡도를 비교한다.
- `sim3_blended_binned_residual_disparity_raw_forward`,
  `sim3_blended_binned_residual_disparity_raw_forward_bins11` 및
  `_alpha025/075` 변형: Sim(3) 보정 후 같은 forward residual을 measured
  disparity LUT와 raw RDF forward LUT에 나눠 적용한다. 기본 후보는 50:50이고
  alpha 변형은 disparity-key 비중을 25%/75%로 바꿔 자동 탐색한다. 단일 key
  LUT가 특정 intrinsic/disparity stress에 과하게 결합되는지 확인하는 dual-key
  보수 후보이다.
- `sim3_agreement_gated_binned_residual_disparity_raw_forward`,
  `sim3_agreement_gated_binned_residual_disparity_raw_forward_bins11`: dual-key
  residual LUT를 쓰되, runtime에서 두 LUT가 예측한 residual 차이가 임계값을
  넘으면 residual 보정을 억제하고 Sim(3) 정합만 남긴다. truth 없이 raw RDF와
  measured disparity의 보정 일치성만으로 single-profile 전이 실패를 줄이는
  후보이다.
- `sim3_binned_residual_disparity_bins5/9/11`: 같은 Sim(3)+forward
  residual LUT 구조에서 bin 수를 grid-search하여 validation/1852m/안정성
  기준으로 선택
- `sim3_kernel_smoothed_binned_residual_disparity_sigma020/040/080` 및
  `_support5/7` 변형:
  measured disparity를 key로 Gaussian-kernel Nadaraya-Watson residual
  regression을 수행하고 runtime에는 같은 piecewise-linear disparity LUT로
  export한다. 목적은 hard bin 경계에서 residual이 튀는 현상을 줄이면서
  `sim3_binned_residual_disparity_bins*` 계열과 같은 StereoDistance runtime
  입력 계약을 유지하는 것이다. `_support5/7`은 support point 수를 줄여
  validation 성능과 모델 복잡도 사이의 trade-off를 탐색한다. PX4/RTK truth는
  fitting/evaluation에만 쓰고, runtime은 raw stereo-center RDF와 measured
  disparity만 사용한다.
- `sim3_local_linear_residual_raw_forward_sigma060/120/250`,
  `sim3_local_linear_residual_disparity_sigma020/040/080` 및 `_support5/7`
  변형: Sim(3) 보정 후 Forward residual을 Gaussian-kernel local-linear
  regression으로 fitting하고 runtime에는 기존 LUT profile로 export한다.
  목적은 Nadaraya-Watson local-constant smoothing이 거리/시차 범위 끝에서
  보일 수 있는 boundary bias를 줄이는 것이다. raw-forward 변형은 raw RDF
  forward meter를 key로 쓰고, disparity 변형은 measured disparity pixel을
  key로 쓴다. pass/fail은 validation range RMSE 개선, 1852m target window
  error, train/validation 안정성, model complexity로 판단한다. PX4/RTK truth는
  fitting/evaluation에만 쓰며 runtime 입력은 raw stereo-center RDF와 measured
  disparity 및 고정 LUT뿐이다.
- `sim3_spline_residual_raw_forward_smooth025/100/400`,
  `sim3_spline_residual_disparity_smooth025/100/400`: Sim(3) 보정 후 Forward
  residual을 regularized cubic smoothing-spline calibration curve로 fitting하고,
  runtime에는 support point 기반 piecewise-linear LUT로 export한다. 목적은
  binned LUT의 hard bin 경계와 sparse-bin overfit을 줄이면서, stereo depth
  residual이 거리/시차에 대해 smooth하게 변한다는 calibration curve 가정을
  검증하는 것이다. `smooth025/100/400`은 smoothing 강도를 grid-search한다.
  pass/fail은 다른 후보와 동일하게 validation range RMSE 개선, 1852m target
  window error, train/validation 안정성, model complexity로 판단한다. PX4/RTK
  truth는 fitting/evaluation에만 쓰며 runtime 입력은 raw stereo-center RDF와
  measured disparity 및 고정 LUT뿐이다.
- `affine_monotone_inverse_depth_binned_residual_raw_forward`,
  `affine_monotone_inverse_depth_binned_residual_raw_forward_bins11`,
  `sim3_monotone_inverse_depth_binned_residual_raw_forward`,
  `sim3_monotone_inverse_depth_binned_residual_raw_forward_bins11`: affine 또는
  Sim(3) estimated-to-truth 정합 뒤 남은 Forward residual을 stereo geometry에
  맞게 `1/truth_forward - 1/base_forward` inverse-depth delta로 fitting한 뒤,
  weighted PAVA isotonic regression으로 단조 제약을 걸고 meter residual LUT로
  다시 export한다. 목적은 일반 inverse-depth residual LUT의 sparse-bin
  과보정과 intrinsic-scale 전이 실패를 줄이는 것이다. pass/fail은 validation
  range RMSE 개선, 1852m target window error, RDF axis-wise RMSE regression,
  cross-seed/transfer 안정성, model complexity로 판단한다. PX4/RTK truth는
  fitting/evaluation에만 쓰며 runtime 입력은 raw stereo-center RDF와 고정
  LUT뿐이다.
- `sim3_smoothed_binned_residual_disparity`,
  `sim3_smoothed_binned_residual_disparity_bins11`: Sim(3)+forward residual
  LUT의 bin median을 global median 쪽으로 shrink하고 인접 bin smoothing을
  적용해 희소 calibration bin과 outlier에 대한 안정성을 비교
- `sim3_capped_binned_residual_disparity`,
  `sim3_capped_binned_residual_disparity_bins11`: Sim(3)+forward residual
  LUT의 bin median을 robust residual cap으로 제한해 single-profile 전이와
  1852m target-window 안정성이 개선되는지 비교
- `sim3_binned_rdf_residual_disparity`: Sim(3) 보정 후 train set의 disparity
  구간별 residual median LUT를 RDF 각 축마다 만들고, runtime에서는 measured
  disparity에 대해 축별 선형 보간 residual을 추가 적용
- `sim3_rdf_residual_disparity`: Sim(3) 보정 후 train set에서 fitting한
  disparity 기반 residual을 RDF 각 축에 추가 적용
- `sim3_target_weighted_rdf_residual_disparity`: Sim(3) 보정 후 1852m 목표거리
  근방 sample을 더 크게 반영한 residual을 RDF 각 축에 추가 적용
- `sim3_ridge_rdf_residual_disparity`: Sim(3) 보정 후 ridge-regularized
  polynomial residual을 RDF 각 축에 추가 적용
- `sim3_ridge_raw_rdf_residual_field_deg1_alpha10`,
  `sim3_ridge_raw_rdf_residual_field_deg1_alpha100_cap4m`,
  `sim3_ridge_raw_rdf_residual_field_deg2_alpha100_cap4m`,
  `sim3_ridge_raw_rdf_residual_field_deg2_alpha1000_cap4m`: Sim(3) 보정 후
  raw stereo-center RDF의 `right/forward`, `down/forward`, normalized
  forward feature로 ridge-regularized vector residual field를 fitting한다.
  목적은 bbox center와 target origin 격차처럼 영상 위치/거리 의존 residual이
  남는지 확인하는 것이다. runtime profile에는 fixed feature normalization과
  RDF residual coefficient matrix, 선택적 residual norm cap만 저장되며,
  runtime에서는 truth 없이 raw RDF만 사용한다.

출력은 `results/rtk_stereo_calibration_*` 아래에 저장된다.

- `method_comparison.csv`: 보정법별 train/validation RMSE, 1852m 근방 통계,
  모델 복잡도, 파라미터 JSON, `registration_model`, `residual_model`,
  `regularization_model`, `runtime_profile_family`, `selection_note`
- `validation_predictions.csv`: validation sample별 truth/prediction/error
- `target_1852m_stats.csv`: 1852m 근방 range error 통계
- `pareto_frontier.csv`, `pareto_frontier.json`: validation range RMSE,
  1852m 근방 절대 평균 오차, 모델 복잡도 기준 Pareto tradeoff
- `policy_recommendations.csv`, `policy_recommendations.json`: balanced,
  axis-balanced, target-specialist, lowest-complexity-pass 등 목적별 추천 후보
- `range_error_before_after.png`: raw와 최종 추천 보정법의 거리 오차 그래프
- `residual_disparity_by_distance.png`: 거리/시차별 residual disparity 그래프
- `rdf_axis_rmse.png`: RDF 축별 validation RMSE 그래프
- `pareto_frontier.png`: 보정 후보들의 validation/1852m/복잡도 tradeoff 그래프
- `summary.txt`: 각 보정법 성능 비교표와 최종 추천 보정법
- `method_parameters.json`, `run_config.json`: 재현용 설정과 보정 파라미터
- `sim3_runtime_profile.json`: legacy 파일명이다. 최종 추천 profile을
  최상위로 복사한 StereoDistance runtime JSON이며, 실제 `mode`는
  `estimated_to_truth_sim3`, `estimated_to_truth_disparity_poly`,
  `estimated_to_truth_affine_rdf` 중 하나일 수 있다.
- `runtime_profiles/`: 최종 추천 여부와 무관하게 StereoDistance runtime에
  적용 가능한 후보 profile 전체와 `manifest.json`
- `policy_profiles/`: 목적별 추천 후보의 StereoDistance runtime profile과
  `manifest.json`

생성된 calibration 결과 폴더가 최종 산출물 계약을 만족하는지 재학습 없이 검사하려면
다음 audit을 실행한다.

```bash
python3 Simulation/scripts/audit_rtk_stereo_calibration_outputs.py \
  --result-dir Simulation/results/rtk_stereo_calibration_<RUN_ID>
```

이 audit은 `method_comparison.csv`, `validation_predictions.csv`,
`target_1852m_stats.csv`, `policy_recommendations.csv/json`, PNG 4종, `summary.txt`,
`run_config.json`, `method_parameters.json`, `runtime_profiles/manifest.json` 존재 여부와
schema를 확인한다. 또한 80:20 train/validation split, 최종 추천이 `raw` 대비
validation range RMSE를 개선하는지, validation set에 1852m target-window sample이
있는지, 추천 후보의 1852m abs mean error가 `10m` 이하인지, validation prediction의
RDF axis/error field가 finite인지, 추천 후보의 RDF right/down/forward validation
RMSE가 raw 대비 후퇴하지 않는지, policy recommendation의 final method가 run config
recommendation과 일치하는지, `runtime_profiles/manifest.json`의 모든 후보 profile이
method table metric과 일치하고 실제 파일로 존재하는지, runtime input에
truth/PX4/RTK/NED field가 들어가지 않는지, forbidden runtime input에 truth/PX4/RTK가
명시됐는지, 각 profile을 `stereo_ground_sim.runtime_guard_policy`의 실제 runtime
class로 로드하고 대표 raw RDF/BBox-center/disparity sample에 apply할 수 있는지,
선택된 `runtime_profiles/<method>.json`이 run config 추천과 일치하는지, top-level
`sim3_runtime_profile.json`이 selected profile 파일과 동일한지,
method table에는 있지만 runtime profile manifest에 없는 method가 `none_raw` 또는
`offline_or_nonpreferred_runtime_candidate` family로 명시된 예외인지,
`policy_profiles/manifest.json`의 목적별 profile들이 실제 파일로 존재하고 metric이
method table과 일치하며 runtime class load/apply smoke를 통과하는지, RDF 계약이
`right/down/forward`로 기록됐는지, public StereoDistance alias가
`forward/right/down/range`인지, PNG 4종이 plot으로 볼 수 있는 최소 크기
`640x360` 이상인지 검사한다. 산출물은
`output_audit/rtk_stereo_calibration_output_audit.json/txt`이다. 이 audit에는
`deliverable_evidence_matrix`도 포함되며, 사용자가 요구한 최종 산출물인
보정법별 성능 비교 CSV, 보정 전/후 거리 오차 그래프, 거리별 residual disparity
그래프, RDF 축별 RMSE 그래프, 1852m 근방 오차 통계, validation prediction CSV,
최종 추천 summary, runtime/policy profile manifest가 각각 어떤 파일과 metric으로
PASS됐는지 기록한다.
`run_rtk_stereo_calibration_with_audit.py`는 audit 통과 후
`inventory/rtk_stereo_calibration_output_inventory.json/csv/txt`도 생성한다.
이 inventory는 재생성 로그인 `output_audit/`와 `inventory/` 자체를 제외한 calibration
결과 파일의 상대 경로, 크기, SHA-256을 기록하고 즉시 재검증한다. 기존 inventory를
다시 검증하려면 다음을 실행한다.

```bash
python3 Simulation/scripts/export_rtk_stereo_calibration_output_inventory.py \
  --result-dir Simulation/results/rtk_stereo_calibration_<RUN_ID> \
  --inventory-json Simulation/results/rtk_stereo_calibration_<RUN_ID>/inventory/rtk_stereo_calibration_output_inventory.json
```

단일 run audit, multi-seed acceptance, robustness acceptance를 한 파일로 묶어
현재 실험 단계별 PASS/FAIL을 보려면 다음 wrapper를 실행한다. 이 wrapper는
단일 run audit의 `deliverable_evidence_matrix`가 method comparison CSV,
거리오차/residual disparity/RDF axis PNG, RDF 축별 non-regression evidence,
1852m 통계, validation prediction, runtime/policy profile manifest를 모두 포함하고
각 항목이 PASS일 때만 single-run stage를 PASS로 인정한다. 또한
deployment strategy에서 `active_runtime_profile.json`을 export하고,
`calibration_samples.csv`의 validation raw RDF/BBox-center/disparity sample로
StereoDistance promotion check를 실행한 뒤, 그 promotion 결과를 최종
acceptance matrix에 다시 포함한다. 또한 promotion check가 검증한
`active_runtime_profile.json`과 최종 dashboard 재생성 후 남은
`active_runtime_profile.json`의 SHA-256이 같은지 확인해, 검증된 profile과
최종 배포 파일이 달라지는 경우를 FAIL 처리한다. 최종 산출물에는
`active_profile_deployment_manifest.json/txt`와
`active_profile_deployment_manifest_validation.json/txt`도 포함된다. manifest는
배포할 profile 경로, SHA-256, method/mode, runtime input 계약, promotion
check 경로와 sample apply 결과, 최종 acceptance 결과를 기록하고, validation은
그 manifest가 실제 profile/promotion/acceptance 파일과 일치하는지 재검증한다.
`deployment_package/`에는 실제 StereoDistance에 넣을
`active_runtime_profile.json`, manifest/validation, promotion evidence, final
acceptance summary, public `/stereo/corrected_position` FRD+range payload
dry-run evidence, `DEPLOYMENT_README.txt`,
`deployment_package_manifest.json/txt` checksum inventory가 함께 복사된다.
wrapper는 마지막에 `deployment_package_check.json/txt`도 생성해
패키지 내부 파일들의 존재, 크기, SHA-256, active profile 계약, 실제 runtime
profile loader/apply smoke, `StereoDistanceNode` loader 경로, promotion evidence
PASS, final acceptance summary PASS, manifest validation PASS를 다시 검사한다.
public payload evidence는 profile을 직접 적용한 corrected RDF와 실제
`StereoDistanceNode`의 `/stereo/corrected_position` 발행 helper가 만든
`Float32MultiArray` 배열이 `0.001m` 허용오차 안에서 같은지도 검사한다.
deployment manifest validation, deployment package check, wrapper summary, readiness summary에는
`final_rdf_axis_nonregression_status: PASS`가 직접 기록되어, RDF 축별
non-regression gate 통과 여부를 package evidence 파일을 다시 열지 않고도 확인할 수
있다.
그 다음 `deployment_package/inventory/stereodistance_deployment_package_inventory.json/csv/txt`를
생성해 `inventory/` 자체를 제외한 패키지 전체 파일의 상대 경로, 크기,
SHA-256을 기록하고 즉시 재검증한다.
`--install-rehearsal-profile`을 지정하면 package check PASS 이후에만 해당
경로로 active profile 설치 리허설을 수행하고, 설치 파일 SHA-256 일치 여부를
wrapper summary에 함께 기록한다. install rehearsal을 수행한 경우 wrapper는
마지막에 `deployment_readiness.json/txt/csv`도 자동 생성해 wrapper summary,
deployment manifest validation, package check, install rehearsal, active profile
SHA-256, public `/stereo/corrected_position` FRD+range payload dry-run,
`StereoDistanceNode` 발행 경로 동등성, operator-view launch command 해석,
runtime truth-free 입력 계약을 한 번 더 재검증한다. operator-view check는
`run_operator_view_with_calibration_profile.sh --package-dir <package> --print-only`
를 실행해 package `active_runtime_profile.json`이 실제
`run_operator_view.sh --bbox-adjustment <profile>` 명령으로 풀리는지 확인한다.
scenario-labeled profile selector를 사용하는 경우에는
`--selector-json <manifest> --deployment-label <label>` 경로도 같은 check로 검증할
수 있으며, 이때도 selector 입력은 배포/scenario label 메타데이터뿐이다.
또한 `stereo_ground_sim.launch.py`가 `enable_bbox_center_adjustment`와
`bbox_center_adjustment_path` launch argument를 선언하고, 두 값을
`stereo_distance_node` params YAML에 전달하는지도 확인한다.

```bash
python3 Simulation/scripts/run_rtk_stereo_acceptance_dashboard_with_promotion.py \
  --single-audit-json Simulation/results/rtk_stereo_calibration_<RUN_ID>/output_audit/rtk_stereo_calibration_output_audit.json \
  --multi-seed-summary-json Simulation/results/rtk_stereo_policy_multi_seed/multi_seed_summary.json \
  --robustness-summary-json Simulation/results/rtk_stereo_robustness_sweep/robustness_summary.json \
  --deployment-strategy-json Simulation/results/rtk_stereo_robustness_sweep/deployment_strategy_recommendation.json \
  --output-active-profile Simulation/results/rtk_stereo_acceptance_matrix_summary/active_runtime_profile.json \
  --calibration-samples-csv Simulation/results/rtk_stereo_calibration_<RUN_ID>/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_acceptance_matrix_summary \
  --split validation \
  --max-samples 50 \
  --runtime-profile-selector-json Simulation/results/rtk_stereo_runtime_observation_stress_EXP_20260617/runtime_profile_label_selector.json \
  --runtime-profile-selector-deployment-label runtime_observation_stress \
  --matrix-active-profile-gate-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_frontier_profile_matrix/active_profile_matrix_gate/active_profile_matrix_gate.json \
  --install-rehearsal-profile Simulation/results/rtk_stereo_acceptance_matrix_summary/install_rehearsal/active_runtime_profile.json \
  --force-install-rehearsal
```

`--runtime-profile-selector-json`를 지정하면 기본 package 기반
`operator_view_launch_check.json|txt`에 더해
`operator_view_launch_check_selector.json|txt`도 생성한다. 이 추가 check는
selector가 배포/scenario label만으로 profile을 고른 뒤, 최종 operator 명령이
선택된 `active_runtime_profile.json`을 `--bbox-adjustment`로 넘기는지 확인한다.
또한 `runtime_deployment_frontier/` 아래에 label route와 profile/package/readiness
상태를 요약한 CSV/JSON/TXT를 자동 생성하고, deployment readiness는 이 frontier
evidence까지 PASS인지 확인한다. selector를 쓰지 않는 일반 dashboard 실행에서는
이 추가 산출물을 만들지 않는다.
`--matrix-active-profile-gate-json`을 함께 지정하면 wrapper가 자동 생성하는
`deployment_readiness.json/csv/txt`에 common-CSV frontier matrix gate 결과도
포함한다. 이 경우 active profile SHA/method가 matrix-selected method와 일치하고,
해당 method가 모든 validation/replay run을 통과하며, runtime 입력이
stereo-center RDF + measured disparity만 사용하는지도 최종 readiness에서 다시
검사한다.

selector를 쓰는 dashboard package에는 `deployment_package/evidence/` 아래에도
`runtime_profile_label_selector.json`, 이동 가능한
`package_runtime_profile_label_selector.json`, 그리고 `runtime_deployment_frontier/*`가
복사된다. `package_runtime_profile_label_selector.json`은 패키지 내부
`../active_runtime_profile.json`을 가리키므로, 패키지만 다른 환경으로 옮겨도 지원
label을 이 패키지 active profile로 해석할 수 있다. 원본 selector/frontier는 어떤
deployment/scenario label이 어떤 profile/package로 해석되는지, 그 profile의 readiness와
SHA-256이 무엇인지 감사하기 위한 evidence로 남긴다.

현재 PASS 후보를 기준선으로 두고 다음 자동탐색 축을 정하려면 method search planner를
실행한다. 이 도구는 새 보정값을 fitting하지 않고, robustness summary의 method별
validation RMSE, 1852m target-window error, RDF axis non-regression, scenario
coverage, model complexity를 읽어 top method frontier와 family별 다음 action을
CSV/JSON/TXT로 기록한다. calibration/evaluation truth는 기존 robustness evidence에만
사용되고, 산출물에는 runtime profile 입력이 stereo-center RDF(`right,down,forward`)와
measured disparity뿐이며 PX4/RTK/NED truth를 runtime 입력으로 쓰지 않는다는 계약을
명시한다.

```bash
python3 Simulation/scripts/plan_stereodistance_method_search.py \
  --robustness-summary-json Simulation/results/rtk_stereo_robustness_sweep/robustness_summary.json \
  --dashboard-summary-json Simulation/results/rtk_stereo_acceptance_matrix_summary/rtk_stereo_acceptance_dashboard_with_promotion_summary.json \
  --active-profile-json Simulation/results/rtk_stereo_acceptance_matrix_summary/active_runtime_profile.json \
  --output-dir Simulation/results/rtk_stereo_acceptance_matrix_summary/method_search_plan \
  --top-n 20
```

주요 산출물은 `method_search_frontier.csv`, `family_search_priorities.csv`,
`method_search_plan.json`, `method_search_plan.txt`이다. `--active-profile-json`을
제공하면 planner의 `active_method`는 실제 export된 StereoDistance runtime profile
method를 뜻하고, `robust_recommendation_method`는 robustness frontier에서 가장 강한
offline 후보를 뜻한다. 두 값이 다를 수 있으므로
`active_profile_matches_robust_recommendation`을 함께 확인한다.
`exploit_active_family`는 현재 active PASS family 주변의 좁은 hyperparameter sweep 대상,
`promote_as_alternate_frontier`는 active profile 대안으로 보존할 후보,
`add_stability_regularization`은 validation 성능은 있으나 strict/stability 조건을 더
강화해야 하는 후보, `diagnose_axis_regression`은 RDF 축별 RMSE 악화를 먼저 해결해야
하는 후보를 뜻한다. 이 planner 결과는 다음 EXP sweep의 method family, sigma/shrink,
Huber/TLS threshold, residual disparity/range term 탐색 우선순위를 정하는 입력으로 쓴다.

새 sweep/dashboard가 PASS했더라도 기존 active profile을 바로 교체하지 않는다. 후보
간 비교는 다음 스크립트로 수행한다. 각 후보는 dashboard directory와 그 후보를 만든
robustness summary를 함께 제공해야 하며, 선택 기준은 readiness PASS,
`final_rdf_axis_nonregression_status: PASS`, runtime truth-free, balanced
validation/1852m target score 순이다. 이 score는 robust validation RMSE mean/max,
1852m target max error를 함께 보므로, 1852m target max가 아주 조금 좋아졌지만
전체 validation RMSE가 크게 나빠진 후보는 교체 후보로 승격하지 않는다. Dashboard
promotion replay sample은 후보마다 다를 수 있으므로, exported active profile
validation RMSE는 tie-break/evidence로만 남기고 primary score에는 넣지 않는다.

```bash
python3 Simulation/scripts/compare_stereodistance_active_profile_candidates.py \
  --candidate current_active:Simulation/results/rtk_stereo_runtime_observation_stress_dashboard_smoke:Simulation/results/rtk_stereo_runtime_observation_stress_EXP_20260617/robustness_summary.json \
  --candidate new_candidate:Simulation/results/rtk_stereo_runtime_observation_stress_sigma_grid_dashboard:Simulation/results/rtk_stereo_runtime_observation_stress_sigma_grid_full/robustness_summary.json \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_sigma_grid_full/profile_candidate_comparison
```

산출물은 `profile_candidate_comparison.csv/json/txt`이다. 이 비교는
calibration/evaluation truth를 새로 사용하지 않고, 이미 생성된 offline evidence만
읽는다. runtime 입력 계약은 stereo-center RDF(`right,down,forward`)와 measured
disparity이며, public output은 FRD+range이다. 또한
`profile_matches_robust_recommendation`과 `method_selection_scope`를 기록해, 실제
export된 active runtime profile method와 robustness frontier recommendation이 서로
다른 경우를 명확히 구분한다. `promotion_sample_fingerprint_sha256`,
`promotion_sample_unique_fingerprint_count`, `promotion_samples_comparable`은 후보
간 promotion replay CSV가 같은 데이터였는지 감사하기 위한 evidence이다.

실제 export된 `active_runtime_profile.json`끼리 같은 calibration CSV에서 직접
비교하려면 common-CSV replay 비교를 실행한다. 이 비교는 robust summary 기반 최종
선택을 대체하지 않고, 동일한 validation rows에서 profile apply 결과를 확인하는
보조 evidence이다. PX4/RTK truth는 replay 평가에만 쓰이며 runtime 입력은 계속
stereo-center RDF(`right,down,forward`)와 measured disparity뿐이다.

```bash
python3 Simulation/scripts/compare_stereodistance_profiles_on_common_csv.py \
  --input-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/mixed_runtime_observation_stress/seed_20260618/calibration_samples.csv \
  --profile current_active:Simulation/results/rtk_stereo_runtime_observation_stress_dashboard_smoke/active_runtime_profile.json \
  --profile fscale_candidate:Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360_dashboard/active_runtime_profile.json \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_profile_comparison \
  --split validation
```

산출물은 `common_csv_profile_comparison.csv/json/txt`,
`common_csv_profile_predictions.csv`,
`common_csv_target_window_stats.csv/json/txt`,
`common_csv_profile_range_errors.png`,
`common_csv_profile_axis_rmse.png`,
`common_csv_residual_disparity_by_range.png`이다.
`common_csv_target_window_stats.*`는 동일 validation rows 중 1852m target-window에
들어간 sample의 raw/corrected range error 통계를 후보별로 따로 기록한다.
`pass_common_replay`는 같은 CSV에서 raw 대비 range RMSE 개선, RDF right/down/forward
축별 RMSE non-regression, 1852m target-window max error `±10m` 이내를 모두 만족할 때
참이다.

같은 방식으로 method-search frontier의 seed runtime profile도 직접 replay할 수 있다.
수동으로 `--profile`을 나열하지 않으려면 frontier CSV와 `runtime_profiles/` 디렉터리를
다음 wrapper에 넘긴다.

```bash
python3 Simulation/scripts/run_common_csv_frontier_profile_comparison.py \
  --frontier-csv Simulation/results/rtk_stereo_runtime_observation_stress_dashboard_smoke/method_search_plan/method_search_frontier.csv \
  --runtime-profiles-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/mixed_runtime_observation_stress/seed_20260618/runtime_profiles \
  --input-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/mixed_runtime_observation_stress/seed_20260618/calibration_samples.csv \
  --active-profile-json Simulation/results/rtk_stereo_runtime_observation_stress_dashboard_smoke/active_runtime_profile.json \
  --active-label current_active_sigma060 \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_frontier_profile_comparison_auto \
  --top-n 12 \
  --split validation
```

이 wrapper는 `method_search_frontier.csv`의 상위 method를
`runtime_profiles/<method>.json`으로 매핑하고, 존재하는 profile만 같은 CSV replay에
넣는다. 추가 산출물은 `frontier_profile_candidate_manifest.csv/json/txt`이며, 어떤
frontier 후보가 실제 runtime profile로 발견됐고 replay에 포함됐는지 기록한다.
2026-06-17 기준
`rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_frontier_profile_comparison_auto`
에서는 current active와 mixed-runtime seed의 top frontier 12개를 같은 validation CSV에
적용했다. 결과는 `current_active_sigma060`이 corrected range RMSE `1.117m`,
1852m target max abs `1.445m`, RDF axis non-regression PASS로 가장 낮은 passing
score를 유지했다. `target_weighted_huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_sigma160`,
`target_weighted_inverse_depth_linear_then_sim3_sigma060`,
`huber_left_disparity_camera_geometry_sim3`, `huber_joint_disparity_ray_rotation_nls`도
PASS했지만 score가 더 컸다.
`disparity_quadratic_then_sim3`와 target-weighted disparity Sim(3)+LUT 계열 일부는
range/1852m error는 통과했지만 Right/Down 축 RMSE가 raw보다 커져
`pass_common_replay: false`로 남긴다. 이 비교도 PX4/RTK truth를 offline 평가에만
사용하며 runtime profile 입력은 stereo-center RDF와 measured disparity뿐이다.

단일 CSV가 아니라 sweep root 아래 모든 `scenario/seed_*` run에 같은 frontier replay를
적용하려면 matrix wrapper를 실행한다.

```bash
python3 Simulation/scripts/run_common_csv_frontier_profile_matrix.py \
  --frontier-csv Simulation/results/rtk_stereo_runtime_observation_stress_dashboard_smoke/method_search_plan/method_search_frontier.csv \
  --sweep-root Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360 \
  --active-profile-json Simulation/results/rtk_stereo_runtime_observation_stress_dashboard_smoke/active_runtime_profile.json \
  --active-label current_active_sigma060 \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_frontier_profile_matrix \
  --top-n 12 \
  --split validation
```

이 wrapper는 각 run의 `calibration_samples.csv`와 `runtime_profiles/`를 찾아 run별
`common_csv_profile_comparison.*`을 생성하고, 전체를
`frontier_profile_matrix_runs.csv`, `frontier_profile_matrix_methods.csv`,
`frontier_profile_matrix_summary.json/txt`로 집계한다. 시각 검토용으로
`frontier_profile_matrix_pass_counts.png`,
`frontier_profile_matrix_rmse.png`,
`frontier_profile_matrix_target_errors.png`도 저장한다. 같은 method가 active profile과
frontier profile 양쪽에 있으면 run별 best replay row 하나만 method aggregate에
반영한다. 이 aggregate는 method family 비교에는 유용하지만 단일 deployable profile
검증으로 해석하면 안 된다. 같은 method 이름 아래에 current active profile과
run별/pooled profile이 함께 있을 수 있기 때문이다. 단일 배포 후보 판단은 반드시
아래 active-profile gate 또는 deployable profile policy selection처럼 실제
`profile_json` 경로를 기준으로 다시 확인한다.

2026-06-17 fscale-grid 3개 scenario x 2개 seed 결과의 초기 top-12/top-20/top-50
matrix aggregate에서는 `target_weighted_huber_joint_disparity_ray_rotation_nls_sigma060`
계열이 6/6 PASS처럼 보였다. 그러나 corrected exact-profile gate로 현재
`active_runtime_profile.json` SHA-256
`19a3a69ee1a8c5c373bbb7000fc087f43310be494473ad0e05955c5b40bf7a63`만 다시 검사하면
actual active profile은 2/6 PASS, RDF axis 2/6 PASS, 1852m target 6/6 PASS다.
target error 자체는 충분히 작지만, 축별 non-regression 조건을 만족하지 못하므로
현재 active profile을 최종 배포 후보로 승격하지 않는다.
같은 frontier CSV의 전체 20개 후보를 `--top-n 20`으로 다시 replay한
`common_csv_frontier_profile_matrix_top20/`에서도 추천은 변하지 않았다.
top-20 결과는 6개 run, 20개 method를 평가했고, selected method는 여전히
`target_weighted_huber_joint_disparity_ray_rotation_nls_sigma060`이다. 이 method는
6/6 PASS, 4/6 run selected, corrected range RMSE mean/max `1.501/2.157m`,
1852m target max `3.955m`, score mean `3.948`이었다. 추가로 들어온
`inverse_depth_linear_then_sim3`, target-shrunk inverse-depth 계열도 6/6 PASS였지만
score mean과 target worst-case가 더 커 active profile을 대체하지 않았다.

matrix 추천이 실제 active runtime profile과 일치하는지 최종 gate로 확인하려면 다음을
실행한다.

```bash
python3 Simulation/scripts/check_common_csv_frontier_matrix_active_profile.py \
  --matrix-summary-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_frontier_profile_matrix/frontier_profile_matrix_summary.json \
  --active-profile-json Simulation/results/rtk_stereo_runtime_observation_stress_dashboard_smoke/active_runtime_profile.json \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_frontier_profile_matrix/active_profile_matrix_gate
```

산출물은 `active_profile_matrix_gate.csv/json/txt`이다. 이 gate는 matrix 자체가 PASS인지,
matrix selected method와 active profile method가 같은지, active method가 모든 run을
통과했는지, 그리고 실제 `profile_json` row가 모든 replay run에서 PASS했는지 확인한다.
active profile runtime input이 truth/PX4/RTK/NED-free인지도 함께 확인한다. corrected
gate의 중요한 차이는 method aggregate PASS만으로 충분하다고 보지 않고, run별
`common_csv_profile_comparison.csv`에서 active profile 경로와 같은 row를 찾아
pass count, RDF axis pass count, 1852m target pass count를 다시 계산한다는 점이다.

frontier CSV 밖의 후보까지 넓히려면 seed별 `method_comparison.csv` 전체를 집계하는
broad ranking을 먼저 만든다.

```bash
python3 Simulation/scripts/rank_sweep_method_comparison_candidates.py \
  --sweep-root Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360 \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/broad_method_candidate_rank \
  --target-error-limit-m 10.0
```

산출물은 `broad_method_candidate_rank.csv/json/txt`와
`broad_method_candidate_run_rows.csv`이다. 이 ranking은 각 seed에서 이미 계산된
offline validation metric을 집계해 raw 대비 range 개선, RDF right/down/forward
non-regression, 1852m target error, runtime profile JSON 존재를 확인한다.
2026-06-17 fscale-grid 결과에서는 6개 run, 248개 method를 집계했고 자기 validation
기준 1위는 `target_weighted_huber_joint_disparity_ray_rotation_nls_sigma030`이었다
(`6/6` PASS, range RMSE mean `1.505m`, 1852m target max `4.011m`). 이 단계는
후보 발굴용이며 최종 선택은 여전히 같은 common-CSV replay matrix로 검증한다.

```bash
python3 Simulation/scripts/run_common_csv_frontier_profile_matrix.py \
  --frontier-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/broad_method_candidate_rank/broad_method_candidate_rank.csv \
  --sweep-root Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360 \
  --active-profile-json Simulation/results/rtk_stereo_runtime_observation_stress_dashboard_smoke/active_runtime_profile.json \
  --active-label current_active_sigma060 \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_broad_profile_matrix_top20 \
  --top-n 20 \
  --split validation
```

broad 후보를 전체 248개로 넓힌 뒤 단일 deployable profile을 검증하려면, 각 run의
local profile이 아니라 하나의 pooled profile directory를 고정해 matrix를 다시 만든다.
이렇게 해야 실제 StereoDistance에 넣을 수 있는 단일 JSON profile의 전이 성능을 볼 수
있다.

```bash
python3 Simulation/scripts/run_common_csv_frontier_profile_matrix.py \
  --frontier-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/broad_method_candidate_rank/broad_method_candidate_rank.csv \
  --sweep-root Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360 \
  --runtime-profiles-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/pooled_scenario_profile/pipeline/runtime_profiles \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active \
  --top-n 248 \
  --split validation
```

이 결과는 6개 replay run, 247개 deployable method를 평가했고, matrix score 기준
selected method는 `affine_monotone_binned_residual_raw_forward_bins11`이다. 이 후보는
6/6 PASS, RDF axis 6/6 PASS, 1852m target 6/6 PASS, validation range RMSE mean/max
`2.064/3.408m`, 1852m target max abs `3.131m`였다. 그러나 사용자의 최종 기준은
train RMSE나 단일 score가 아니라 validation RMSE, 1852m 근방 오차, 모델 안정성이므로
정책별 deployable profile selection을 추가로 실행한다.

```bash
python3 Simulation/scripts/select_deployable_profile_from_common_csv_matrix.py \
  --matrix-summary-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/frontier_profile_matrix_summary.json \
  --runtime-profiles-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/pooled_scenario_profile/pipeline/runtime_profiles \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection \
  --target-error-limit-m 10.0 \
  --recommended-policy range_priority
```

산출물은 `deployable_profile_policy_selection.csv/json/txt`와
`deployable_profile_candidates.csv`이다. hard gate는 단일 profile JSON 존재,
truth/PX4/RTK/NED-free runtime input, 6/6 replay PASS, RDF axis 6/6 PASS,
1852m target 6/6 PASS, 1852m 최대 절대오차 `10m` 이하를 모두 요구한다.
2026-06-18 현재 hard gate 통과 후보는 166개이며, 정책별 winner는 다음과 같다.

- `range_priority`: `sim3_inverse_depth_binned_residual_raw_forward_bins11`,
  validation range RMSE mean/max `1.883/2.981m`, 1852m target max abs `3.535m`
- `target_priority`: `tukey_range_weighted_sim3`,
  validation range RMSE mean/max `2.222/3.387m`, 1852m target max abs `3.005m`
- `score_priority`: `affine_monotone_binned_residual_raw_forward_bins11`,
  validation range RMSE mean/max `2.064/3.408m`, 1852m target max abs `3.131m`
- `balanced`: `huber_joint_disparity_ray_rotation_nls_fscale0p75`,
  validation range RMSE mean/max `1.892/3.103m`, 1852m target max abs `3.501m`

현재 추천은 `range_priority` 기준의
`sim3_inverse_depth_binned_residual_raw_forward_bins11`이다. 1852m 오차는 모든 정책 후보가
`±10m` 조건을 충분히 만족하므로, validation range RMSE mean/max와 all-run RDF axis
안정성을 우선했다. 이 profile의 SHA-256은
`9fcb2e3107be8dd15a7335bab59d09f3d23df03cb74e72172f9a819e18626ceb`이다. 단, 추천은
배포 패키지로 즉시 설치한다는 뜻이 아니라, 이 profile로 acceptance dashboard,
promotion check, deployment readiness를 다시 실행한 뒤 active profile로 승격한다는
뜻이다. 모든 선택 단계는 offline calibration/evaluation replay evidence만 읽으며,
runtime StereoDistance 입력으로 truth/PX4/RTK/NED를 추가하지 않는다.

추천 profile의 StereoDistance loader/apply smoke는 다음처럼 단독으로 확인한다.

```bash
python3 Simulation/scripts/stereodistance_profile_promotion_check.py \
  --profile-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/pooled_scenario_profile/pipeline/runtime_profiles/sim3_inverse_depth_binned_residual_raw_forward_bins11.json \
  --sample-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/pooled_scenario_profile/pipeline/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_promotion_check \
  --split validation \
  --max-samples 80
```

2026-06-18 실행 결과는 `pass_promotion_check: true`, `schema_load_ok: true`,
`node_loader_ok: true`, `sample_apply_count: 80`, `sample_invalid_count: 0`,
`runtime_truth_used: false`였다.

정책 선택 결과를 실제 `active_runtime_profile.json` 후보로 복사하고, 같은 단계에서
promotion check와 public FRD+range payload dry-run까지 묶으려면 다음을 실행한다.

```bash
python3 Simulation/scripts/export_deployable_profile_candidate_from_policy_selection.py \
  --policy-selection-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/deployable_profile_policy_selection.json \
  --output-active-profile Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_active_candidate/active_runtime_profile.json \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_active_candidate \
  --sample-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/pooled_scenario_profile/pipeline/calibration_samples.csv \
  --split validation \
  --max-samples 80
```

산출물은 `deployable_profile_candidate_export.json/txt`,
`active_runtime_profile.json`, `promotion_check/*`,
`public_topic_payload/public_corrected_position_payload_summary.json`,
`public_topic_payload/public_corrected_position_payload.csv`이다. 2026-06-18 실행 결과는
`status: PASS`, `recommendation: use_as_next_acceptance_dashboard_active_profile`,
promotion sample apply `80/80`, public payload sample `80/80`, public payload
node-equivalence `PASS`, runtime truth `false`였다. public payload axis order는
`forward_m,right_m,down_m,range_m`이며, profile runtime 입력은 여전히
stereo-center RDF `right/down/forward`와 measured disparity다.

주의할 점은 기존 `check_common_csv_frontier_matrix_active_profile.py` gate가
“matrix score 기준 selected method와 active profile이 같은가”를 확인하는 gate라는
것이다. 현재 range-priority 추천은 validation range RMSE를 우선하므로 matrix score
selected method인 `affine_monotone_binned_residual_raw_forward_bins11`과 다르다. 따라서
이 후보를 기존 matrix-selected readiness gate에 그대로 넣으면 gate 목적상 FAIL하는
것이 정상이다. range-priority 후보의 근거는
`deployable_profile_policy_selection.json`과
`range_priority_active_candidate/deployable_profile_candidate_export.json`이며, 최종
배포 readiness에는 policy-selection evidence를 별도로 반영해야 한다.

policy-selection active 후보 전용 readiness는 다음으로 확인한다.

```bash
python3 Simulation/scripts/check_deployable_profile_candidate_readiness.py \
  --candidate-export-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_active_candidate/deployable_profile_candidate_export.json \
  --output-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_active_candidate/deployable_profile_candidate_readiness.json \
  --output-txt Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_active_candidate/deployable_profile_candidate_readiness.txt \
  --output-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_active_candidate/deployable_profile_candidate_readiness.csv
```

이 gate는 profile SHA 일치, policy-selection PASS, 6/6 replay PASS, RDF axis 6/6
PASS, 1852m target 6/6 PASS, promotion check PASS, public FRD+range payload PASS,
runtime truth-free 계약을 모두 확인한다. 2026-06-18 실행 결과는 `status: PASS`,
`failure_count: 0`, validation range RMSE mean/max `1.883/2.981m`, 1852m target
max abs `3.535m`이다.

이 후보를 다음 acceptance-dashboard 실행에 넘길 수 있는 작은 evidence package로
묶으려면 다음을 실행한다. 이 패키지는 기존 full deployment package와 달리 최종 설치
승격물이 아니라, policy-selection 후보와 그 검증 근거를 안전하게 옮기기 위한
candidate package다.

```bash
python3 Simulation/scripts/export_deployable_profile_candidate_package.py \
  --candidate-readiness-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_active_candidate/deployable_profile_candidate_readiness.json \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_candidate_package
```

생성 파일은 `active_runtime_profile.json`,
`deployable_profile_candidate_package_manifest.json`,
`DEPLOYABLE_PROFILE_CANDIDATE_README.txt`, 그리고 `evidence/` 아래의 policy selection,
candidate export, readiness, promotion check, public payload CSV/JSON이다. 패키지 자체를
검사하려면 다음을 실행한다.

```bash
python3 Simulation/scripts/check_deployable_profile_candidate_package.py \
  --package-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_candidate_package \
  --output-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_candidate_package/deployable_profile_candidate_package_check.json \
  --output-txt Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_candidate_package/deployable_profile_candidate_package_check.txt
```

2026-06-18 실행 결과는 `status: PASS`, `failure_count: 0`, runtime apply smoke
`PASS`, file count `8`, replay pass `6/6`, RDF axis pass `6/6`, target pass `6/6`,
promotion sample `80`, public payload sample `80`, public payload node-equivalence
`PASS`, runtime truth `false`였다.

candidate package가 operator-view launch wrapper에 실제로 연결되는지 확인하려면
다음 dry-run checker를 실행한다.

```bash
python3 Simulation/scripts/check_stereodistance_operator_view_launch.py \
  --package-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_candidate_package \
  --output-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_candidate_package/operator_view_launch_check.json \
  --output-txt Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_candidate_package/operator_view_launch_check.txt
```

2026-06-18 실행 결과는 `status: PASS`, `package_type: candidate_package`,
`package_check_status: PASS`, `bbox_adjustment_arg_present: true`, launch contract
`PASS`, runtime truth `false`였다. 출력 command는 candidate package 내부
`active_runtime_profile.json`을 `run_operator_view.sh --bbox-adjustment`에 넘긴다.
실제 operator view를 띄울 때는 같은 package-dir을 사용하되 `--print-only` 없이 실행한다.

```bash
./Simulation/run_operator_view_with_calibration_profile.sh \
  --package-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_candidate_package \
  -- --no-px4 yolo_target_hz:=0.0
```

다른 policy winner도 같은 candidate package/operator-view gate를 통과하는지 비교하려면
다음 suite wrapper를 실행한다. 이 wrapper는 각 policy에 대해 policy selection,
active candidate export, readiness, candidate package export/check, operator-view
dry-run, candidate package portability smoke check, 최종 package summary를 생성한다.
`--pipeline-dir`를 함께 지정하면 최종 선택 package에 대해 install rehearsal과
deliverables audit까지 같은 suite summary에 포함한다.

```bash
python3 Simulation/scripts/run_deployable_profile_policy_package_suite.py \
  --matrix-summary-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/frontier_profile_matrix_summary.json \
  --runtime-profiles-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/pooled_scenario_profile/pipeline/runtime_profiles \
  --sample-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/pooled_scenario_profile/pipeline/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite \
  --policies range_priority,target_priority,score_priority,balanced \
  --split validation \
  --max-samples 80 \
  --target-error-limit-m 10.0 \
  --pipeline-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/pooled_scenario_profile/pipeline
```

산출물은 `deployable_profile_policy_package_suite_summary.json/txt`, 각 policy별
`selection/`, `active_candidate/`, `candidate_package/`, 각 candidate package 내부
`deployable_profile_candidate_package_portability.json/txt`, 그리고
`package_summary/deployable_profile_candidate_package_summary.csv/json/txt`이다.
`--pipeline-dir`를 지정한 최종 감사 모드에서는 선택된 policy 아래
`install_rehearsal/install_manifest.json/txt`와
`deliverables_check/deployable_profile_deliverables_check.json/txt`,
`goal_acceptance/rtk_stereo_goal_acceptance.csv/json/txt`도 생성된다.
2026-06-18 suite 실행 결과는 4개 policy 모두 `PASS`이고 최종 선택은
`range_priority`의 `sim3_inverse_depth_binned_residual_raw_forward_bins11`이다.
같은 summary에 `selected_install_status: PASS`,
`selected_deliverables_status: PASS`, `selected_goal_acceptance_status: PASS`,
runtime truth `false`가 기록됐다.

최종 acceptance 증거를 한 디렉터리로 고정하려면 다음 bundle exporter를 실행한다.
이 bundle은 최종 runtime profile, candidate package manifest/check, operator/portability
check, goal acceptance, deliverables check, suite summary, 필수 CSV/PNG/TXT 산출물을
복사하고 각 파일의 SHA256을 기록한다. 내부 보정/검증 좌표계는
`stereo_center_opencv_optical:RDF(right,down,forward)`, public StereoDistance topic
계약은 `FRD_range(forward,right,down,range)`로 유지한다. PX4/RTK/NED truth는
calibration/evaluation evidence로만 포함되고 runtime input에는 포함되지 않는다.

```bash
python3 Simulation/scripts/export_rtk_stereo_goal_acceptance_bundle.py \
  --goal-acceptance-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/goal_acceptance/rtk_stereo_goal_acceptance.json \
  --suite-summary-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/deployable_profile_policy_package_suite_summary.json \
  --deliverables-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/deliverables_check/deployable_profile_deliverables_check.json \
  --candidate-package-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/candidate_package \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/final_acceptance_bundle
```

2026-06-18 bundle 실행 결과는 `status: PASS`, file count `29`,
profile SHA256 `9fcb2e3107be8dd15a7335bab59d09f3d23df03cb74e72172f9a819e18626ceb`,
validation range RMSE `1.852601842130819m`, raw validation range RMSE
`3.7799663436141038m`, 1852m target max abs error `4.524451953154312m`,
goal acceptance `11/11 PASS`, runtime truth `false`였다. 주요 출력은
`final_acceptance_bundle/rtk_stereo_goal_acceptance_bundle_manifest.json`,
`final_acceptance_bundle/rtk_stereo_goal_acceptance_bundle_files.csv`,
`final_acceptance_bundle/rtk_stereo_goal_acceptance_bundle_summary.txt`,
`final_acceptance_bundle/candidate_package/`, `final_acceptance_bundle/evidence/`,
`final_acceptance_bundle/artifacts/`이다.

자동 탐색이 특정 방법군에 갇히지 않았는지 확인하려면 method-search catalog audit를
실행한다. 이 감사는 frontier matrix와 deployable policy selection, 최종 bundle을 함께
읽어서 rotation/pose alignment, scale/intrinsic compensation, Sim(3), disparity 또는
inverse-depth, nonlinear/binned residual, geometry+disparity 조합 계열이 모두 포함됐는지
확인한다. 또한 hard gate를 통과한 후보가 runtime truth-free이고, RDF/FRD 계약을 지키는지
검사한다.

```bash
python3 Simulation/scripts/audit_rtk_stereo_method_search_catalog.py \
  --matrix-summary-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/frontier_profile_matrix_summary.json \
  --policy-selection-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/selection/deployable_profile_policy_selection.json \
  --bundle-manifest-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/final_acceptance_bundle/rtk_stereo_goal_acceptance_bundle_manifest.json \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/method_search_catalog_audit \
  --min-method-count 12 \
  --target-error-limit-m 10.0
```

2026-06-18 catalog audit 결과는 `status: PASS`, method count `247/12`, required
family coverage `PASS`, deployable hard-gate pass count `166`, selected method
`sim3_inverse_depth_binned_residual_raw_forward_bins11`, runtime truth `false`,
coordinate contract `PASS`였다. 다음 자동 탐색 action은 현재 best family 주변의
local sweep을 계속하되 alternate 후보를 보존하는 것이다. 출력은
`method_search_catalog_audit/rtk_stereo_method_search_catalog_audit.json/txt`,
`rtk_stereo_method_search_family_coverage.csv`,
`rtk_stereo_method_search_candidate_status.csv`,
`rtk_stereo_method_search_next_actions.csv`이다.

catalog audit의 next action을 실제 다음 sweep 후보로 내보내려면 local sweep plan exporter를
실행한다. 이 단계는 새 보정법을 fitting하지 않고, 현재 최종 후보
`sim3_inverse_depth_binned_residual_raw_forward_bins11` 주변에서 중복되지 않는 후보 config를
만든다. 탐색 축은 residual basis, bin count, residual prior strength, residual gain,
target weighting, robust loss, monotonic constraint, forward key이며, 이미 catalog에 존재하는
method 이름은 제외한다.

```bash
python3 Simulation/scripts/export_rtk_stereo_local_sweep_plan.py \
  --catalog-audit-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/method_search_catalog_audit/rtk_stereo_method_search_catalog_audit.json \
  --active-profile-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/candidate_package/active_runtime_profile.json \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_plan \
  --max-candidates 64
```

2026-06-18 local sweep plan 결과는 `status: PASS`, base method
`sim3_inverse_depth_binned_residual_raw_forward_bins11`, new candidate count `64`,
excluded existing method count `247`, runtime truth `false`였다. 상위 후보는
`sim3_inverse_depth_monotone_binned_residual_raw_forward_bins11`,
`sim3_inverse_depth_binned_residual_base_forward_bins11`,
`sim3_inverse_depth_monotone_binned_residual_base_forward_bins11`,
`sim3_inverse_depth_cauchy_binned_residual_raw_forward_bins11`,
`sim3_inverse_depth_huber_binned_residual_raw_forward_bins11` 순서다. 출력은
`local_sweep_plan/rtk_stereo_local_sweep_plan.json/txt`,
`local_sweep_plan/rtk_stereo_local_sweep_candidates.csv`,
`local_sweep_plan/rtk_stereo_local_sweep_candidate_configs.csv`,
`local_sweep_plan/candidate_configs/*.json`이다. 이 config들은 offline fitting 시에만 truth를
사용하고, runtime export 후보는 `raw_right_m/raw_down_m/raw_forward_m/raw_disparity_px`만
입력으로 허용해야 한다.

생성된 local sweep config를 실제 calibration pipeline에 연결하려면
`--local-sweep-config-dir`를 지정한다. 현재 pipeline은 이 plan의 target-weight,
Huber/Cauchy robust-weight, range-normalized monotone, blended inverse-depth/disparity
monotone 조합까지 native method search에 추가한다. 각 config의 추가/스킵 상태는
`local_sweep_config_status.csv/json`에 기록된다.

```bash
python3 Simulation/scripts/rtk_stereo_calibration_pipeline.py \
  --mode csv \
  --input-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/pooled_scenario_profile/pipeline/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_full64 \
  --target-range-m 1852.0 \
  --target-window-m 30.0 \
  --local-sweep-config-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_plan/candidate_configs
```

2026-06-18 full local sweep probe 결과는 64개 config 중 native pipeline `ADDED` 64개,
`SKIP` 0개였다. `base_forward` query 후보는 runtime profile에
`forward_residual_raw_forward_query: base_forward`를 기록하고, runtime apply도 Sim(3) 적용 후
forward 값을 LUT query로 사용한다. 추가 후보 중 validation RMSE 상위는
`sim3_meter_binned_residual_raw_forward_bins11` `1.918516m`,
`sim3_meter_binned_residual_base_forward_bins11` `1.918523m`,
`sim3_meter_targetw0p25_binned_residual_raw_forward_bins11` `1.919962m`,
`sim3_meter_monotone_targetw0p25_binned_residual_raw_forward_bins11` `1.919962m`였다.
이번 probe의 전체 최종 추천은 기존 내장 후보
`huber_joint_disparity_ray_rotation_nls`였고 validation RMSE `1.916m`, raw validation RMSE
`4.021m`, 1852m abs mean error `0.003m`였다. 전체 method comparison row 수는 313개다.
따라서 local 후보는 스킵 없이 평가되고 runtime profile까지 생성되지만
현재 최종 acceptance bundle의 `range_priority` 후보를 대체하려면 common-CSV matrix,
package, deliverables, goal acceptance까지 다시 통과해야 한다.

full local sweep 후보를 pooled CSV 결과만으로 판단하지 않으려면 local-only frontier를
내보낸 뒤 기존 common-CSV matrix에 재검증한다. 이 단계는 calibration-time truth를 offline
검증에만 사용하고, replay되는 runtime profile은 `stereo-center RDF(right,down,forward) +
measured disparity`만 입력으로 사용한다. 출력 public topic 계약은
`FRD+range(forward,right,down,range)`다.

```bash
python3 Simulation/scripts/export_rtk_stereo_local_sweep_frontier.py \
  --method-comparison-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_full64/method_comparison.csv \
  --local-sweep-status-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_full64/local_sweep_config_status.csv \
  --runtime-profiles-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_full64/runtime_profiles \
  --output-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_full64/local_sweep_frontier.csv \
  --output-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_full64/local_sweep_frontier.json \
  --output-txt Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_full64/local_sweep_frontier.txt \
  --top-n 64

python3 Simulation/scripts/run_common_csv_frontier_profile_matrix.py \
  --frontier-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_full64/local_sweep_frontier.csv \
  --sweep-root Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360 \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_common_csv_matrix_full64 \
  --runtime-profiles-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_full64/runtime_profiles \
  --top-n 64 \
  --split validation \
  --target-range-m 1852.0 \
  --target-window-m 30.0
```

2026-06-18 local-only common-CSV matrix 결과는 `status: PASS`, run count `6`,
method count `64`, runtime truth `false`였다. matrix selected method는
`sim3_blended_disp_invdepth_monotone_targetw0p25_binned_residual_raw_forward_bins11`이며
6/6 run에서 PASS했다. 성능은 RMSE mean/max `1.981/3.616m`, 1852m target max
`3.734m`이다. target error는 여전히 `±10m` 조건을 만족하지만, 기존 acceptance 후보
`sim3_inverse_depth_binned_residual_raw_forward_bins11`의 common-CSV matrix RMSE mean/max
`1.883/2.981m`, target max `3.535m`보다 낮지 않다. 따라서 이번 local sweep은
alternate 후보 탐색에는 유효하지만 최종 `range_priority` 추천을 교체하지 않는다. 주요 출력은
`local_sweep_pipeline_probe_full64/local_sweep_frontier.csv/json/txt`,
`local_sweep_common_csv_matrix_full64/frontier_profile_matrix_summary.json/txt`,
`frontier_profile_matrix_methods.csv`, `frontier_profile_matrix_runs.csv`,
`frontier_profile_matrix_rmse.png`, `frontier_profile_matrix_target_errors.png`이다.

local-only matrix의 selected 후보를 중심으로 다시 탐색하되 이미 평가한 64개 local 후보를
반복하지 않으려면 plan exporter에 `--exclude-methods-csv`를 지정한다. 이 CSV는
`method` 컬럼을 가진 frontier/status/method comparison 파일이면 된다. 아래 round2는
1차 local matrix selected profile을 active로 삼고 catalog 247개와 1차 local 64개를
제외한 96개 후보를 생성한다.

```bash
python3 Simulation/scripts/export_rtk_stereo_local_sweep_plan.py \
  --catalog-audit-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/method_search_catalog_audit/rtk_stereo_method_search_catalog_audit.json \
  --active-profile-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_full64/runtime_profiles/sim3_blended_disp_invdepth_monotone_targetw0p25_binned_residual_raw_forward_bins11.json \
  --exclude-methods-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_full64/local_sweep_frontier.csv \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_plan_round2 \
  --max-candidates 96

python3 Simulation/scripts/rtk_stereo_calibration_pipeline.py \
  --mode csv \
  --input-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/pooled_scenario_profile/pipeline/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round2_96 \
  --target-range-m 1852.0 \
  --target-window-m 30.0 \
  --local-sweep-config-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_plan_round2/candidate_configs

python3 Simulation/scripts/export_rtk_stereo_local_sweep_frontier.py \
  --method-comparison-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round2_96/method_comparison.csv \
  --local-sweep-status-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round2_96/local_sweep_config_status.csv \
  --runtime-profiles-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round2_96/runtime_profiles \
  --output-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round2_96/local_sweep_frontier.csv \
  --output-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round2_96/local_sweep_frontier.json \
  --output-txt Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round2_96/local_sweep_frontier.txt \
  --top-n 96

python3 Simulation/scripts/run_common_csv_frontier_profile_matrix.py \
  --frontier-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round2_96/local_sweep_frontier.csv \
  --sweep-root Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360 \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_common_csv_matrix_round2_96 \
  --runtime-profiles-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round2_96/runtime_profiles \
  --top-n 96 \
  --split validation \
  --target-range-m 1852.0 \
  --target-window-m 30.0
```

2026-06-18 round2 결과는 plan `PASS`, excluded extra methods `64`, candidate count
`96`, pipeline native `ADDED` 96/96, frontier missing profile `0`, common-CSV matrix
`PASS`였다. round2 matrix selected method는
`sim3_blended_disp_invdepth_monotone_targetw1_binned_residual_raw_forward_bins11`이고
6/6 run에서 PASS했다. 성능은 RMSE mean/max `1.971/3.556m`, 1852m target max
`3.611m`이다. 이는 1차 local matrix selected `1.981/3.616m`, target max `3.734m`보다
개선됐지만, 기존 최종 acceptance 후보 `sim3_inverse_depth_binned_residual_raw_forward_bins11`
의 RMSE mean/max `1.883/2.981m`, target max `3.535m`보다 낮지는 않다. 따라서 round2도
최종 추천 교체 조건은 충족하지 못했고, 다음 탐색은 같은 후보 반복이 아니라 bin count,
prior strength, residual gain 축을 더 멀리 보내거나, scenario별 selector/portfolio
전략으로 분리해야 한다.

round2 winner 주변을 한 번 더 탐색하려면 round1과 round2 frontier를 모두 제외하고
round3 plan을 만든다. 이 round는 residual gain `0.75/1.25`와 target-weight 주변 후보가
주로 남는다.

```bash
python3 Simulation/scripts/export_rtk_stereo_local_sweep_plan.py \
  --catalog-audit-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/method_search_catalog_audit/rtk_stereo_method_search_catalog_audit.json \
  --active-profile-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round2_96/runtime_profiles/sim3_blended_disp_invdepth_monotone_targetw1_binned_residual_raw_forward_bins11.json \
  --exclude-methods-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_full64/local_sweep_frontier.csv \
  --exclude-methods-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round2_96/local_sweep_frontier.csv \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_plan_round3 \
  --max-candidates 128

python3 Simulation/scripts/rtk_stereo_calibration_pipeline.py \
  --mode csv \
  --input-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/pooled_scenario_profile/pipeline/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round3_128 \
  --target-range-m 1852.0 \
  --target-window-m 30.0 \
  --local-sweep-config-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_plan_round3/candidate_configs

python3 Simulation/scripts/export_rtk_stereo_local_sweep_frontier.py \
  --method-comparison-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round3_128/method_comparison.csv \
  --local-sweep-status-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round3_128/local_sweep_config_status.csv \
  --runtime-profiles-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round3_128/runtime_profiles \
  --output-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round3_128/local_sweep_frontier.csv \
  --output-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round3_128/local_sweep_frontier.json \
  --output-txt Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round3_128/local_sweep_frontier.txt \
  --top-n 128

python3 Simulation/scripts/run_common_csv_frontier_profile_matrix.py \
  --frontier-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round3_128/local_sweep_frontier.csv \
  --sweep-root Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360 \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_common_csv_matrix_round3_128 \
  --runtime-profiles-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_pipeline_probe_round3_128/runtime_profiles \
  --top-n 128 \
  --split validation \
  --target-range-m 1852.0 \
  --target-window-m 30.0
```

2026-06-18 round3 결과는 plan `PASS`, excluded extra methods `160`, candidate count
`128`, pipeline native `ADDED` 128/128, frontier missing profile `0`, common-CSV matrix
`PASS`였다. round3 matrix selected method는
`sim3_blended_disp_invdepth_monotone_gain0p75_binned_residual_base_forward_bins11`이고
6/6 run에서 PASS했다. 성능은 RMSE mean/max `1.964/3.518m`, 1852m target max
`3.554m`이다. round3는 round2보다 더 좋아졌고 target max는 기존 최종 후보의
`3.535m`에 가까워졌지만, RMSE mean/max는 여전히 기존 최종 후보
`1.883/2.981m`보다 크다. 따라서 최종 추천은 계속
`sim3_inverse_depth_binned_residual_raw_forward_bins11`로 유지한다. 세 번의 local sweep이
모두 6/6 PASS이지만 global single-profile 교체에는 실패했으므로, 다음 자동 탐색은
단일 전역 winner를 더 미세 조정하기보다 scenario/seed label 또는 runtime 관측 품질에 따른
truth-free selector/portfolio를 비교하는 방향이 더 적합하다.

단일 profile을 더 미세 조정하는 대신 selector/portfolio가 유리한지 확인하려면 기존
all248 matrix와 local round1-3 matrix를 합친 후보 pool을 감사한다. 이 audit은 offline
validation truth로 점수를 계산하지만, deployable selector 후보는 runtime truth를 쓰지 않는
`global_single` 또는 `scenario_loso`만 추천 대상으로 삼는다. `scenario_insample`과
`per_run_oracle`은 상한/진단값이며 배포 추천이 아니다.

```bash
python3 Simulation/scripts/audit_common_csv_matrix_selector_portfolio.py \
  --matrix-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active \
  --matrix-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_common_csv_matrix_full64 \
  --matrix-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_common_csv_matrix_round2_96 \
  --matrix-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/local_sweep_common_csv_matrix_round3_128 \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/selector_portfolio_audit_all248_plus_local_rmse \
  --objective rmse_then_target
```

2026-06-18 selector portfolio audit 결과는 `status: PASS`, candidate method count
`535`, run count `6`, runtime truth `false`였다. 세 objective(`score`,
`target_then_rmse`, `rmse_then_target`) 모두에서 deployable 추천은 `scenario_loso`였다.
RMSE 우선 objective 기준 `scenario_loso`는 RMSE mean/max `1.632/2.617m`, 1852m target
max `3.517m`로 기존 최종 단일 후보 `1.883/2.981m`, target max `3.535m`보다 좋았다.
score objective 기준도 `scenario_loso`는 `1.758/2.617m`, target max `2.696m`였다.
따라서 다음 구현 단계는 단일 global profile 교체가 아니라, calibration-time matrix에서
scenario/deployment label별 profile map을 export하고 operator-view selector package로
검증하는 것이다. selector 입력은 scenario/deployment label 또는 raw runtime feature이며,
PX4/RTK/NED truth, validation error, replay label은 runtime 입력으로 쓰지 않는다.
주요 출력은 `selector_portfolio_summary.json/txt`,
`selector_portfolio_modes.csv`, `selector_portfolio_selected_runs.csv`,
`selector_portfolio_methods.csv`, `selector_portfolio_modes.png`이다.

`scenario_loso` 결과를 실제 operator-view selector 경로로 쓰려면 selector package를
내보낸다. 이 exporter는 scenario label용 profile map과 감사 재현용
`scenario:seed` label을 함께 만든다. 실제 운용 기본 label은 scenario 이름
(`bbox_quantization_stress`, `disparity_bias_drift_stress`,
`mixed_runtime_observation_stress`)이며, seed label은 replay audit 재현용이다.

```bash
python3 Simulation/scripts/export_common_csv_selector_package.py \
  --selector-summary-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/selector_portfolio_audit_all248_plus_local_rmse/selector_portfolio_summary.json \
  --selected-runs-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/selector_portfolio_audit_all248_plus_local_rmse/selector_portfolio_selected_runs.csv \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse \
  --selector-mode scenario_loso \
  --objective rmse_then_target

python3 Simulation/scripts/select_stereodistance_runtime_profile_by_label.py \
  --selector-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/runtime_profile_label_selector.json \
  --deployment-label bbox_quantization_stress \
  --output-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/selection_bbox_quantization_stress.json \
  --output-txt Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/selection_bbox_quantization_stress.txt

python3 Simulation/scripts/export_stereodistance_runtime_deployment_frontier.py \
  --selector-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/runtime_profile_label_selector.json \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/runtime_deployment_frontier

python3 Simulation/scripts/check_stereodistance_operator_view_launch.py \
  --selector-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/runtime_profile_label_selector.json \
  --deployment-label bbox_quantization_stress \
  --output-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/operator_view_launch_check_selector_bbox.json \
  --output-txt Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/operator_view_launch_check_selector_bbox.txt
```

2026-06-18 selector package export 결과는 `status: PASS`, scenario label count `3`,
deployment label count `15`, profile count `10`, runtime truth `false`였다. 세 scenario
label selection은 모두 `PASS`였고, `bbox_quantization_stress` operator-view selector
smoke도 `PASS`였다. 이 check는 selector JSON을 받아 selected label
`scenario_bbox_quantization_stress`를 해석하고, lightweight profile package의
`active_runtime_profile.json`을 `run_operator_view.sh --bbox-adjustment`로 넘기는
print-only command를 검증했다. 생성 출력은
`scenario_loso_selector_package_rmse/runtime_profile_label_selector.json`,
`selector_package_profiles.csv`, `selector_package_labels.csv`,
`runtime_deployment_frontier/runtime_deployment_frontier.json/csv/txt`,
`selection_<scenario>.json/txt`, `operator_view_launch_check_selector_bbox.json/txt`이다.

export된 selector package가 실제 배포 후보로 충분한지 한 번에 확인하려면 readiness
checker를 실행한다. 이 check는 selector export, portfolio summary, runtime deployment
frontier, operator-view selector smoke, scenario label selection 결과를 함께 검사한다.
PASS 조건은 deployable recommendation이 `scenario_loso`이고, `scenario_loso` validation
RMSE mean이 `global_single`보다 낮으며, 1852m target max error가 허용치 안에 있고,
frontier/operator/label selection이 모두 PASS이며, runtime truth 입력이 없는 것이다.

```bash
Simulation/.venv/bin/python Simulation/scripts/check_common_csv_selector_package_readiness.py \
  --selector-package-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse \
  --portfolio-summary-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/selector_portfolio_audit_all248_plus_local_rmse/selector_portfolio_summary.json \
  --frontier-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/runtime_deployment_frontier/runtime_deployment_frontier.json \
  --operator-check-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/operator_view_launch_check_selector_bbox.json \
  --selection-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/selection_bbox_quantization_stress.json \
  --selection-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/selection_disparity_bias_drift_stress.json \
  --selection-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/selection_mixed_runtime_observation_stress.json \
  --output-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/selector_package_readiness.json \
  --output-txt Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/selector_package_readiness.txt \
  --output-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/selector_package_readiness.csv
```

2026-06-18 readiness 결과는 `status: PASS`, `recommended_deployable_selector_mode:
scenario_loso`, RMSE mean/max `1.632/2.617m`, 1852m target max `3.517m`였다. 같은
audit의 `global_single`은 RMSE mean/max `2.023/2.617m`, target max `4.914m`였으므로,
scenario label을 배포 메타데이터나 운용자 설정으로 줄 수 있다면 이 selector package가
현재 단일 profile보다 강한 배포 후보이다. readiness의 axis contract는
`stereo_center_opencv_optical:RDF(right,down,forward)`이고, StereoDistance 공개 topic
contract는 `FRD_range(forward,right,down,range)`로 분리된다. selector package는
deployment/scenario label만 선택 입력으로 사용하며, 선택된 runtime profile은
stereo-center RDF와 measured disparity만 소비한다. PX4/RTK/NED truth는 offline evidence로만
남는다.

명시적인 scenario label 없이 raw runtime-like feature만으로 profile을 고르는 후보도
같은 3-scenario sweep에서 다시 확인한다. 이 guard는 `raw_right_m`, `raw_down_m`,
`raw_forward_m`, `raw_range_m`, `measured_disparity_px`, `left_u_px`, `left_v_px`
통계와 offline transfer matrix만 사용하고, truth/error/replay winner는 selection 입력에서
금지한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/select_stereodistance_profile_by_runtime_measurement_guard.py \
  --sweep-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360 \
  --catalog-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/scenario_profile_catalog_minimal_set.json \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_measurement_guard_audit_common_csv_selector_followup \
  --candidate-source-mode all_catalog \
  --selection-policy transfer_risk_fallback \
  --transfer-low-risk-rmse-limit-m 1.3 \
  --transfer-low-risk-target-limit-m 2.5 \
  --audit-all-sweep-seeds
```

2026-06-18 follow-up 결과는 `pass_count: 6/6`, validation-only `6/6`, validation RMSE
mean/max `1.549/2.264m`, 1852m target mean/max `0.409/1.334m`, target max abs worst
`3.358m`였다. 이 결과는 명시 scenario label selector의 RMSE mean/max
`1.632/2.617m`보다 낮다. 다만 현재 audit은 3개 known scenario와 2개 seed 안에서의
raw-feature 분리 결과이므로, 곧바로 최종 배포 추천을 교체하지 않고 “manual scenario label을
대체할 수 있는 자동 selector 후보”로 둔다. 다음 검증은 leave-one-scenario-out 또는 새
오차 regime에서 raw-feature selector가 틀린 profile을 고르지 않는지 확인하는 것이다.
산출물은
`runtime_measurement_guard_audit_common_csv_selector_followup/runtime_measurement_guard_audit.csv`,
`runtime_measurement_guard_summary.json/txt`,
`runtime_measurement_guard_selection_counts.png`이다.

더 엄격하게 target scenario를 feature library와 candidate source set에서 제외하면 다음처럼
leave-one-scenario-out raw selector를 감사한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_runtime_measurement_guard_policies.py \
  --sweep-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360 \
  --catalog-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/scenario_profile_catalog_minimal_set.json \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_measurement_guard_policy_audit_loso_followup \
  --candidate-source-modes all_catalog \
  --policies nearest_centroid,robust_family_fallback,transfer_risk_fallback,intrinsic_scale_gap_transfer_risk,feature_vote_transfer_risk,knn_transfer_risk,centroid_knn_union_transfer_risk,centroid_knn_veto_transfer_risk \
  --transfer-low-risk-rmse-limit-m 1.3 \
  --transfer-low-risk-target-limit-m 2.5 \
  --exclude-target-scenario-from-library
```

2026-06-18 LOSO policy audit도 모든 policy가 `6/6` PASS였지만, best recommendation은
단순 `nearest_centroid`였고 validation RMSE mean/max `2.612/3.493m`, target max abs worst
`5.595m`로 known-scenario raw selector와 명시 `scenario_loso` selector보다 약했다.
`feature_vote_transfer_risk`와 `centroid_knn_union_transfer_risk`는 RMSE mean과 target
worst를 `2.486m`, `5.255m`까지 낮췄지만, fallback 2회와 복잡도 penalty 때문에 현재
policy score에서는 추천되지 않았다. 이 결과의 의미는 자동 selector가 unknown regime에서도
`±10m` target gate는 유지하지만, 현재 feature만으로는 scenario label을 알고 고르는
selector의 정확도를 대체하기에는 아직 부족하다는 것이다. 따라서 배포 우선순위는
1) 운용자가 scenario/deployment label을 줄 수 있으면 `scenario_loso selector package`,
2) label이 없으면 raw-feature selector를 candidate로 쓰되 새 regime/실제 보정 비행 replay로
한 번 더 검증하는 순서로 둔다.

raw-feature selector를 실제 active profile export 경로로 smoke하려면 truth 열을 제거한
runtime-only CSV를 입력으로 `export_stereodistance_profile_by_operating_mode.py`를 실행한다.
CSV에는 좌표 계약 문자열처럼 쉼표가 포함된 field가 있으므로, runtime-only CSV를 만들 때
단순 field number split이 아니라 CSV parser를 사용해야 한다. 필요한 runtime 열은
`sample_id`, `split`, `raw_right_m`, `raw_down_m`, `raw_forward_m`, `raw_range_m`,
`left_u_px`, `left_v_px`, `measured_disparity_px`이다.

```bash
Simulation/.venv/bin/python Simulation/scripts/export_runtime_guard_input_from_calibration_csv.py \
  --input-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/bbox_quantization_stress/seed_20260617/calibration_samples.csv \
  --output-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_operating_mode_export_followup/bbox_seed20260617_runtime_only.csv \
  --output-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_operating_mode_export_followup/runtime_guard_input_export_summary.json \
  --output-txt Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_operating_mode_export_followup/runtime_guard_input_export_summary.txt

Simulation/.venv/bin/python Simulation/scripts/export_stereodistance_profile_by_operating_mode.py \
  --catalog-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/scenario_profile_catalog/manifest.json \
  --input-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_operating_mode_export_followup/bbox_seed20260617_runtime_only.csv \
  --output-profile Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_operating_mode_export_followup/active_runtime_profile.json \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_operating_mode_export_followup \
  --sweep-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360 \
  --minimal-set-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/scenario_profile_catalog_minimal_set.json \
  --mode auto \
  --runtime-candidate-source-mode all_catalog \
  --runtime-selection-policy transfer_risk_fallback \
  --transfer-low-risk-rmse-limit-m 1.3 \
  --transfer-low-risk-target-limit-m 2.5
```

2026-06-18 smoke 결과는 runtime input export `status: PASS`, output row count `360`,
runtime truth `false`였고, operating-mode export는 `selected_operating_mode:
runtime_guard`, `has_truth_fields: false`, `has_runtime_fields: true`, selected source
`bbox_quantization_stress`, calibration-required block `false`였다. 생성된
`runtime_guard_operating_mode_export_followup/active_runtime_profile.json`은
`stereodistance_profile_promotion_check.py`에서도 `pass_promotion_check: true`,
`sample_apply_count: 50`, `sample_invalid_count: 0`, runtime truth `false`를 기록했다.
즉 scenario label 없이 raw RDF/disparity CSV만 있어도 현재 catalog에서 active profile을
선택해 StereoDistance loader가 읽을 수 있는 형태로 export할 수 있다. 이 smoke는
runtime 적용 경로 확인이고, 성능 평가는 위 runtime guard audit과 별도로 본다.

같은 runtime_guard operating-mode export 경로를 모든 scenario/seed calibration CSV에
대해 반복 확인하려면 batch smoke를 실행한다. 이 wrapper는 각 calibration CSV를
truth-free runtime guard CSV로 변환하고, active profile export와 promotion check를
순서대로 실행한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_runtime_guard_operating_mode_batch_smoke.py \
  --sweep-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360 \
  --catalog-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/scenario_profile_catalog_minimal_set.json \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_operating_mode_batch_smoke_followup \
  --runtime-candidate-source-mode all_catalog \
  --runtime-selection-policy transfer_risk_fallback \
  --transfer-low-risk-rmse-limit-m 1.3 \
  --transfer-low-risk-target-limit-m 2.5 \
  --split validation \
  --max-samples 50
```

출력은 `runtime_guard_operating_mode_batch_smoke.csv`,
`runtime_guard_operating_mode_batch_smoke_summary.json/txt`, 그리고 각
`<scenario>/<seed>/runtime_guard_input.csv`,
`<scenario>/<seed>/active_runtime_profile.json`,
`<scenario>/<seed>/promotion_check/promotion_check.json/txt`이다. 이 batch smoke도
selection/evaluation metric을 재계산하지 않고, runtime 적용 경로가 truth-free CSV와
StereoDistance loader 기준으로 깨지지 않는지 확인한다.

2026-06-18 batch smoke 결과는 `status: PASS`, run/pass `6/6`, runtime truth `false`였다.
각 scenario/seed의 promotion check는 `sample_apply_count: 50`, `sample_invalid_count: 0`을
기록했다. 선택 source는 bbox quantization 2개 seed가 `bbox_quantization_stress`,
disparity-bias drift 2개 seed가 `disparity_bias_drift_stress`, mixed runtime observation
2개 seed가 `mixed_runtime_observation_stress`였고, 모두 `runtime_guard` operating mode와
StereoDistance loader promotion을 통과했다.

batch smoke가 만든 lightweight profile package들을 operator-view wrapper까지 통과시키려면
다음 readiness check를 실행한다. 이 check는 각 `<scenario>/<seed>/active_runtime_profile.json`
의 부모 디렉터리를 lightweight package로 보고,
`run_operator_view_with_calibration_profile.sh --print-only --package-dir ...` 경로가
`--bbox-adjustment`에 같은 profile을 넘기는지 검사한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/check_runtime_guard_batch_operator_view.py \
  --batch-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_operating_mode_batch_smoke_followup/runtime_guard_operating_mode_batch_smoke.csv \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_batch_operator_view_followup
```

출력은 `runtime_guard_batch_operator_view_readiness.csv`,
`runtime_guard_batch_operator_view_readiness_summary.json/txt`, 그리고 각
`<scenario>/<seed>/operator_view_launch_check.json/txt`이다. 이 단계도 실제 ROS2/Gazebo를
띄우지 않고 print-only command contract를 확인한다.

2026-06-18 operator-view batch readiness 결과는 `status: PASS`, run/pass `6/6`,
runtime truth `false`였다. 6개 모두 `package_type: lightweight_profile_package`,
package check `PASS`, wrapper return code `0`, launch contract `PASS`였고, wrapper가
해석한 `resolved_runtime_profile`은 각 run의 `active_runtime_profile.json`과 일치했다.

자동 `runtime_guard` 경로를 하나의 배포 후보로 볼 수 있는지 최종 점검하려면 다음
readiness checker를 실행한다. 이 check는 known-scenario raw-feature audit, stricter
leave-one-scenario-out policy audit, runtime_guard operating-mode batch smoke,
operator-view batch readiness를 함께 묶는다.

```bash
Simulation/.venv/bin/python Simulation/scripts/check_runtime_guard_deployment_readiness.py \
  --known-guard-summary-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_measurement_guard_audit_common_csv_selector_followup/runtime_measurement_guard_summary.json \
  --loso-policy-summary-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_measurement_guard_policy_audit_loso_followup/runtime_measurement_guard_policy_audit_summary.json \
  --batch-smoke-summary-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_operating_mode_batch_smoke_followup/runtime_guard_operating_mode_batch_smoke_summary.json \
  --operator-readiness-summary-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_batch_operator_view_followup/runtime_guard_batch_operator_view_readiness_summary.json \
  --output-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_deployment_readiness_followup/runtime_guard_deployment_readiness.json \
  --output-txt Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_deployment_readiness_followup/runtime_guard_deployment_readiness.txt \
  --output-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_deployment_readiness_followup/runtime_guard_deployment_readiness.csv
```

이 readiness의 PASS 기준은 known-scenario raw selector와 LOSO policy audit가 모두
`±10m` target gate를 통과하고, runtime_guard batch smoke와 operator-view batch readiness가
같은 run count로 전부 PASS하며, 모든 단계가 runtime truth 없이 stereo-center RDF와 measured
disparity 기반 입력 계약을 유지하는 것이다. 이 결과가 PASS이면 `runtime_guard`는
“label이 없을 때 쓸 자동 배포 후보”가 된다. 단, manual scenario/deployment label을 줄 수
있으면 위 `scenario_loso selector package`가 여전히 우선 후보이고, runtime_guard는 실제
보정 비행 CSV나 새 오차 regime replay로 한 번 더 검증한다.

2026-06-18 runtime_guard deployment readiness 결과는 `status: PASS`,
recommendation `runtime_guard_candidate_ready_for_real_calibration_csv_replay`였다.
known-scenario raw selector는 `6/6` PASS, RMSE mean/max `1.549/2.264m`, target max
`3.358m`였고, LOSO policy audit도 `6/6` PASS, best policy `nearest_centroid`, RMSE
mean/max `2.612/3.493m`, target max `5.595m`였다. batch smoke와 operator-view readiness는
각각 `6/6` PASS였다. 따라서 자동 runtime_guard는 “새 실제 보정 비행 CSV replay로 넘어갈
수 있는 후보”로 승격한다.

deployment readiness는 기존 3개 runtime observation stress scenario에서의 검증이다. 기존
stress set에 과적합되지 않았는지 보려면 exported runtime profile을 독립 synthetic seed와
다른 perturbation family(`nominal`, `yaw_pitch_stress`, `disparity_noise_stress`,
`intrinsic_scale_stress`, `combined_sensor_stress`, `bbox_outlier_stress`)에 replay한다.
이 fresh replay도 runtime profile 입력은 stereo-center RDF와 measured disparity만 사용하고,
PX4/NED truth는 offline 평가에만 사용한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_runtime_profile_fresh_seed_replay.py \
  --profile-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_operating_mode_batch_smoke_followup/bbox_quantization_stress/seed_20260617/active_runtime_profile.json \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_fresh_seed_replay_followup/bbox_profile \
  --experiment-id EXP-20260618-runtime-guard-fresh-replay-bbox \
  --seed-start 20310000 \
  --trial-count 20 \
  --sample-count 840 \
  --target-oversample-count 48

Simulation/.venv/bin/python Simulation/scripts/summarize_fresh_seed_profile_replays.py \
  --replay-root Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/runtime_guard_fresh_seed_replay_followup \
  --replay-root Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/selector_package_fresh_seed_replay_followup \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/fresh_seed_profile_replay_combined_followup
```

2026-06-18 fresh-seed replay에서는 runtime_guard batch에서 선택된 대표 profile 3개와
scenario selector package의 전체 profile 10개를 비교했다. 합산 결과는 `status: FAIL`,
strict pass `0/13`, runtime truth `false`였다. 가장 좋은 ranked candidate는 selector package의
run-level bbox profile
`rtk_stereo_calibration-sim3_blended_disp_invdepth_monotone_huber_gain1p25_binned_residual_base_forward_bins11`
였고, pass rate `0.817`, range RMSE mean `1.567m`, position RMSE mean `3.076m`였지만
1852m 근방 target max abs error가 `14.219m`로 `±10m` gate를 넘었다. runtime_guard batch
대표 profile 중 best는 `rtk_stereo_calibration-theil_sen_global_inverse_depth_linear`였고
pass rate `0.808`, range RMSE mean `1.541m`, target max `14.927m`였다.

따라서 현재 결론은 둘로 나눈다. 기존 stress scenario와 operator-view 경로 기준으로는
`scenario_loso selector package`와 `runtime_guard`가 모두 배포 후보지만, 새 synthetic
perturbation family까지 포함한 최종 보정법으로는 아직 PASS가 아니다. 다음 탐색은
fresh replay의 tail failure를 줄이는 방향, 특히 intrinsic scale stress와 bbox/disparity
outlier stress에서 1852m target error를 `±10m` 이내로 제한하는 runtime guard 또는 더 보수적인
profile selection rule을 찾아야 한다.

후속 탐색으로 기존 best Sim(3) profile에 runtime-safe guard를 덧붙여 fresh replay를
반복했다. `export_guarded_runtime_profile.py`는 임의의 Sim(3) runtime profile에
`target_disparity_range_delta_cap_guard`, `target_disparity_blend_guard`,
`target_range_away_blend_guard`를 주입한다. 이 guard들은 measured disparity, raw range,
corrected range만 사용하고 truth는 사용하지 않는다.

```bash
Simulation/.venv/bin/python Simulation/scripts/export_guarded_runtime_profile.py \
  --input-profile Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/packages/run_bbox_quantization_stress_seed_20260618/active_runtime_profile.json \
  --output-profile Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/fresh_guard_profile_sweep_followup/cap0p5/active_runtime_profile.json \
  --guard-mode target_disparity_range_delta_cap \
  --half-width-px 0.35 \
  --range-delta-cap-m 0.5
```

2026-06-18 guard sweep 결과는 아직 `FAIL`이다. 12개 guard variant 중 best는
`away_w70_a0p75`였고 pass rate `0.825`, range RMSE mean `1.604m`, position RMSE mean
`3.089m`, target max `12.899m`였다. range-delta cap 계열의 best는 `cap0p5`로 target max를
기존 `14.219m`에서 `12.216m`까지 줄였지만 `±10m` gate는 통과하지 못했다. 즉 단일 프레임
profile에 target-neighborhood guard만 추가하는 방식은 tail error를 줄이지만 충분하지 않다.

worst row 분석에서는 bbox/disparity outlier가 섞인 validation target-window sample에서
measured disparity가 true disparity보다 약 `+2.2px` 크게 튀어 raw range가 truth보다 약
`37m` 짧아지는 사례가 확인됐다. 이 정보는 offline 분석에서만 알 수 있으며, 실제 runtime에서
true disparity는 사용할 수 없다. 따라서 이 tail은 global pose/origin 보정만의 문제가 아니라
measurement outlier rejection 문제로 분리해 다뤄야 한다.

같은 이유로 `audit_runtime_profile_fresh_seed_replay.py`에 corrected RDF range history만 쓰는
trailing temporal filter 옵션도 추가했다. 초기 trailing median과 낮은 threshold linear filter는
접근 비행의 정상 range 증가까지 outlier로 처리해 성능이 크게 악화됐고, cascade를 제거한
trailing linear prediction도 bbox-outlier replay에서 PASS하지 못했다. 현재 결론은 단순 temporal
range smoothing이 아니라, 영상/BBox/disparity 측정 단계에서 outlier score를 산출하거나,
시간 연속성과 stereo geometry를 함께 쓰는 robust tracker/filter가 필요하다는 것이다.

그 다음에는 measured disparity history만 쓰는 runtime-safe prefilter도 시험했다. 이 prefilter는
이전 measured disparity로 현재 disparity를 예측하고, threshold를 넘는 경우 raw RDF bearing은
유지한 채 range를 measured/predicted disparity 비율로 rescale한다. raw range target-window
제한 옵션도 추가해 1852m 주변에서만 동작하게 할 수 있다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_runtime_profile_fresh_seed_replay.py \
  --profile-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/fresh_bbox_outlier_direct_calibration_followup/policy_profiles/target_max_stable.json \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/fresh_disparity_prefilter_replay_followup/w5_th3p5 \
  --experiment-id EXP-20260618-fresh-disparity-prefilter-w5-th3p5 \
  --scenarios bbox_outlier_stress \
  --disparity-prefilter-window 5 \
  --disparity-prefilter-threshold-px 3.5
```

2026-06-18 disparity prefilter sweep도 `FAIL`이었다. 12개 variant 중 target max만 보면
`w5_th3p5`가 `9.106m`로 `±10m` gate 안에 들어왔지만, prefilter count가 `944/4200` 수준으로
너무 많아 range RMSE mean이 `8.479m`, position RMSE mean이 `9.543m`까지 악화됐고 pass rate도
`5/20`에 그쳤다. raw-range window를 걸면 false positive는 줄었지만 prefilter가 거의 작동하지
않거나 target max가 다시 `14m` 이상으로 남았다. 따라서 단순 disparity trend thresholding은
tail만 줄이고 전체 정확도를 해치는 tradeoff가 커서 최종 후보가 아니다.

이미 각 policy package가 만들어져 있을 때는 다음 요약만 단독 실행할 수 있다.

```bash
python3 Simulation/scripts/summarize_deployable_profile_candidate_packages.py \
  --package-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection/range_priority_candidate_package \
  --package-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection_target_priority/candidate_package \
  --package-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection_score_priority/candidate_package \
  --package-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_selection_balanced/candidate_package \
  --output-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_summary
```

2026-06-18 결과는 4개 policy package 모두 package/operator-ready `PASS`였다. package와
operator-view까지 통과한 후보 중 validation range RMSE 기준 최종 선택은 계속
`range_priority`의 `sim3_inverse_depth_binned_residual_raw_forward_bins11`이다.

- `range_priority`: RMSE mean/max `1.883/2.981m`, target max `3.535m`
- `balanced`: RMSE mean/max `1.892/3.103m`, target max `3.501m`
- `score_priority`: RMSE mean/max `2.064/3.408m`, target max `3.131m`
- `target_priority`: RMSE mean/max `2.222/3.387m`, target max `3.005m`

모든 후보가 1852m `±10m` 조건과 RDF axis 6/6 조건을 만족하므로, 현재 기본 추천은
validation range RMSE가 가장 낮은 `range_priority` 후보로 유지한다. target worst-case를
최우선으로 두는 별도 운용 모드에서는 `target_priority` 후보를 alternate package로
보존한다.

candidate package를 다른 디렉터리로 옮겨도 package-local `active_runtime_profile.json`만
참조하는지 확인하려면 다음 portability smoke check를 실행한다. 이 check는 패키지를
임시 디렉터리로 복사한 뒤 package checker와 operator-view dry-run을 다시 실행한다.

```bash
python3 Simulation/scripts/check_deployable_profile_candidate_package_portability.py \
  --package-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/candidate_package \
  --output-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/candidate_package/deployable_profile_candidate_package_portability.json \
  --output-txt Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/candidate_package/deployable_profile_candidate_package_portability.txt
```

2026-06-18에는 suite의 4개 policy candidate package 모두 portability `PASS`였다.
각 복사본에서 package check `PASS`, operator-view dry-run `PASS`,
`package_type: candidate_package`, runtime truth `false`, 그리고 복사된 패키지 내부
`active_runtime_profile.json` 해석을 확인했다. 이 검증의 입력/출력 계약은 runtime
profile 입력 `stereo-center RDF(right,down,forward) + measured disparity`, public topic
출력 `FRD+range(forward,right,down,range)`이다.

배포 패키지를 다른 디렉터리로 옮겨도 package-local selector가 원본 실험 결과
경로가 아니라 패키지 내부 `active_runtime_profile.json`만 참조하는지 smoke
검증하려면 다음을 실행한다. selector를 쓰는 acceptance dashboard wrapper는 이
검증을 자동 실행하고, 결과를
`deployment_package/evidence/package_portability_check.json|txt`와
`deployment_readiness.json/csv/txt`에 포함한다.

```bash
python3 Simulation/scripts/check_stereodistance_deployment_package_portability.py \
  --package-dir Simulation/results/rtk_stereo_acceptance_matrix_summary/deployment_package \
  --deployment-label runtime_observation_stress \
  --output-json Simulation/results/rtk_stereo_acceptance_matrix_summary/deployment_package/evidence/package_portability_check.json \
  --output-txt Simulation/results/rtk_stereo_acceptance_matrix_summary/deployment_package/evidence/package_portability_check.txt
```

이 check는 패키지를 임시 디렉터리로 복사한 뒤
`evidence/package_runtime_profile_label_selector.json`을 실행하고, 선택 결과가
복사된 패키지 내부 `active_runtime_profile.json`과 복사된 package dir을 가리키는지
확인한다. 입력 label은 운용자가 정한 배포/scenario label뿐이며, PX4/RTK/NED truth나
validation/replay 오차값은 selector 입력으로 사용하지 않는다.

이미 생성된 manifest만 단독으로 재검증하려면 다음을 실행한다.

```bash
python3 Simulation/scripts/validate_stereodistance_active_profile_deployment_manifest.py \
  --manifest-json Simulation/results/rtk_stereo_acceptance_matrix_summary/active_profile_deployment_manifest.json \
  --output-json Simulation/results/rtk_stereo_acceptance_matrix_summary/active_profile_deployment_manifest_validation.json \
  --output-txt Simulation/results/rtk_stereo_acceptance_matrix_summary/active_profile_deployment_manifest_validation.txt
```

검증된 manifest에서 배포 패키지만 다시 만들려면 다음을 실행한다.

```bash
python3 Simulation/scripts/export_stereodistance_active_profile_deployment_package.py \
  --manifest-json Simulation/results/rtk_stereo_acceptance_matrix_summary/active_profile_deployment_manifest.json \
  --manifest-validation-json Simulation/results/rtk_stereo_acceptance_matrix_summary/active_profile_deployment_manifest_validation.json \
  --output-dir Simulation/results/rtk_stereo_acceptance_matrix_summary/deployment_package
```

이미 만들어진 배포 패키지만 단독 검사하려면 다음을 실행한다.

```bash
python3 Simulation/scripts/check_stereodistance_active_profile_deployment_package.py \
  --package-dir Simulation/results/rtk_stereo_acceptance_matrix_summary/deployment_package \
  --output-json Simulation/results/rtk_stereo_acceptance_matrix_summary/deployment_package/deployment_package_check.json \
  --output-txt Simulation/results/rtk_stereo_acceptance_matrix_summary/deployment_package/deployment_package_check.txt
```

이미 생성된 배포 패키지 inventory만 단독 재검증하려면 다음을 실행한다. 이
검증은 `deployment_package/inventory/` 자체를 제외한 패키지 파일들의 상대
경로, 크기, SHA-256이 기록 당시와 같은지만 확인하며, PX4 NED truth,
stereo-center RDF, public FRD+range 좌표값을 변환하거나 다시 계산하지 않는다.

```bash
python3 Simulation/scripts/export_stereodistance_deployment_package_inventory.py \
  --package-dir Simulation/results/rtk_stereo_acceptance_matrix_summary/deployment_package \
  --inventory-json Simulation/results/rtk_stereo_acceptance_matrix_summary/deployment_package/inventory/stereodistance_deployment_package_inventory.json \
  --output-dir Simulation/results/rtk_stereo_acceptance_matrix_summary/deployment_package/inventory
```

패키지 검사까지 통과한 active profile을 실제 StereoDistance 런타임 profile
경로로 설치하려면 다음을 실행한다. 이 명령은 full deployment package와
lightweight candidate package를 자동 감지한다. 먼저 해당 package checker를 다시
실행하고, candidate package에 portability evidence가 있으면 그것도 `PASS`인지 확인한
뒤 `active_runtime_profile.json`만 지정 경로로 복사한다. 설치된 파일의 SHA-256은
package manifest의 `active_profile_sha256`과 같아야 한다. 기본값은 기존 파일을
덮어쓰지 않으며, 의도적으로 교체할 때만 `--force`를 지정한다.
런타임 입력 계약은 설치 후에도 stereo-center RDF(`right_m`, `down_m`,
`forward_m`)와 measured disparity이며, PX4/RTK/NED truth는 입력하지 않는다.

```bash
python3 Simulation/scripts/install_stereodistance_active_profile_package.py \
  --package-dir Simulation/results/rtk_stereo_acceptance_matrix_summary/deployment_package \
  --output-profile <StereoDistance-runtime-profile-path>/active_runtime_profile.json \
  --output-json Simulation/results/rtk_stereo_acceptance_matrix_summary/profile_install/install_manifest.json \
  --output-txt Simulation/results/rtk_stereo_acceptance_matrix_summary/profile_install/install_manifest.txt
```

현재 policy suite의 최종 추천 candidate package를 설치 리허설하려면 다음처럼
결과 폴더 안의 rehearsal 경로를 출력 대상으로 둔다.

```bash
python3 Simulation/scripts/install_stereodistance_active_profile_package.py \
  --package-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/candidate_package \
  --output-profile Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/install_rehearsal/active_runtime_profile.json \
  --output-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/install_rehearsal/install_manifest.json \
  --output-txt Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/install_rehearsal/install_manifest.txt \
  --force
```

2026-06-18 실행 결과는 `status: PASS`, `package_type: candidate_package`,
`package_check_status: PASS`, `portability_check_status: PASS`, 설치 SHA-256
`9fcb2e3107be8dd15a7335bab59d09f3d23df03cb74e72172f9a819e18626ceb` 일치,
runtime truth `false`였다. 실제 StereoDistance 운용 경로에 쓸 때는 같은 명령의
`--output-profile`만 운용 profile 경로로 바꾼다.

최종 추천 package가 원래 요구한 CSV/PNG/TXT 산출물과 연결되어 있는지 한 번에
감사하려면 다음을 실행한다. 이 check는 보정값을 다시 계산하지 않고, pooled pipeline의
`method_comparison.csv`, `target_1852m_stats.csv`, `validation_predictions.csv`,
`range_error_before_after.png`, `residual_disparity_by_distance.png`,
`rdf_axis_rmse.png`, `summary.txt`와 package/operator/portability/install evidence를
검사한다. 또한 selected method가 raw보다 validation range RMSE를 줄였는지,
1852m target max error가 `±10m` 안인지, RDF/NED/FRD 계약이 명시되어 있는지 확인한다.

```bash
python3 Simulation/scripts/check_deployable_profile_deliverables.py \
  --pipeline-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/pooled_scenario_profile/pipeline \
  --package-summary-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/package_summary/deployable_profile_candidate_package_summary.json \
  --candidate-package-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/candidate_package \
  --install-manifest-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/install_rehearsal/install_manifest.json \
  --output-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/deliverables_check/deployable_profile_deliverables_check.json \
  --output-txt Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/deliverables_check/deployable_profile_deliverables_check.txt
```

2026-06-18 실행 결과는 `status: PASS`, selected method
`sim3_inverse_depth_binned_residual_raw_forward_bins11`, pooled pipeline validation
range RMSE `1.853m` 대 raw `3.780m`, pooled pipeline 1852m max abs error
`4.524m`, package/operator/portability/install `PASS`, runtime truth `false`였다.
같은 package의 common-CSV matrix worst target max error는 위 package summary 기준
`3.535m`이며, 두 검증 모두 `±10m` 조건을 만족한다.

원래 실험 목표의 numbered requirements와 최종 산출물 요구를 행 단위로 감사하려면
다음 checklist를 실행한다. 이 check는 suite summary, deliverables check, candidate
package manifest, README interface contract를 함께 읽고, PX4 NED truth offline-only,
stereo-center RDF/BBox/disparity 입력, synthetic/error injection, method family coverage,
train/validation split, validation/1852m PASS, CSV/PNG/TXT 산출물, runtime package readiness,
실제 topic 교체 interface, validation 중심 최종 선택 정책을 각각 PASS/FAIL로 기록한다.

```bash
python3 Simulation/scripts/check_rtk_stereo_goal_acceptance.py \
  --suite-summary-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/deployable_profile_policy_package_suite_summary.json \
  --deliverables-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/deliverables_check/deployable_profile_deliverables_check.json \
  --candidate-package-dir Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/candidate_package \
  --readme Simulation/README.md \
  --output-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/goal_acceptance/rtk_stereo_goal_acceptance.json \
  --output-csv Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/goal_acceptance/rtk_stereo_goal_acceptance.csv \
  --output-txt Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/goal_acceptance/rtk_stereo_goal_acceptance.txt
```

2026-06-18 실행 결과는 `status: PASS`, `pass_count: 11/11`, selected policy
`range_priority`, selected method `sim3_inverse_depth_binned_residual_raw_forward_bins11`,
runtime truth `false`였다.

ROS2/Gazebo operator view에서 검증된 배포 패키지를 바로 적용하려면 다음을
사용한다. `--print-only`를 붙이면 실제 ROS2/Gazebo를 띄우지 않고
`run_operator_view.sh --bbox-adjustment <package>/active_runtime_profile.json`
명령만 확인한다.

```bash
./run_operator_view_with_calibration_profile.sh \
  --package-dir Simulation/results/rtk_stereo_acceptance_matrix_summary/deployment_package \
  --print-only \
  -- --no-px4 yolo_target_hz:=0.0
```

같은 조건을 JSON/TXT evidence로 남기려면 다음 checker를 실행한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/check_stereodistance_operator_view_launch.py \
  --package-dir Simulation/results/rtk_stereo_acceptance_matrix_summary/deployment_package \
  --output-json Simulation/results/rtk_stereo_acceptance_matrix_summary/operator_view_launch_check.json \
  --output-txt Simulation/results/rtk_stereo_acceptance_matrix_summary/operator_view_launch_check.txt
```

launch 파일의 profile parameter contract만 단독 확인하려면 다음을 실행한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/check_stereodistance_launch_profile_contract.py \
  --output-json Simulation/results/rtk_stereo_acceptance_matrix_summary/launch_profile_contract.json \
  --output-txt Simulation/results/rtk_stereo_acceptance_matrix_summary/launch_profile_contract.txt
```

wrapper summary, deployment manifest, package check, install rehearsal을 모두
수동으로 다시 최종 점검하려면 readiness checker를 실행한다. 이 단계는 보정법을
다시 학습하지 않고, 이미 생성된 evidence가 서로 같은 active profile SHA-256을
가리키는지, promotion/package/install/public topic payload 상태가 PASS인지,
operator-view launch check가 PASS인지, runtime 입력이 truth/PX4/RTK/NED를
요구하지 않는지 확인한다.
`--install-rehearsal-profile`을 넣어 wrapper를 실행했다면 같은 파일들이
wrapper에서 이미 자동 생성된다.

```bash
Simulation/.venv/bin/python Simulation/scripts/check_rtk_stereo_deployment_readiness.py \
  --wrapper-summary-json Simulation/results/rtk_stereo_acceptance_matrix_summary/rtk_stereo_acceptance_dashboard_with_promotion_summary.json \
  --matrix-active-profile-gate-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_frontier_profile_matrix/active_profile_matrix_gate/active_profile_matrix_gate.json \
  --output-json Simulation/results/rtk_stereo_acceptance_matrix_summary/deployment_readiness.json \
  --output-txt Simulation/results/rtk_stereo_acceptance_matrix_summary/deployment_readiness.txt \
  --output-csv Simulation/results/rtk_stereo_acceptance_matrix_summary/deployment_readiness.csv
```

같은 과정을 수동으로 나눠 디버그하려면 다음 summary를 실행한다. 단독 promotion
check까지 matrix에 넣으려면 아래 `stereodistance_profile_promotion_check.py`를 먼저
실행해 `promotion_check.json`을 만든 뒤 `--promotion-check-json`을 함께 지정한다.

```bash
python3 Simulation/scripts/summarize_rtk_stereo_acceptance_matrices.py \
  --single-audit-json Simulation/results/rtk_stereo_calibration_<RUN_ID>/output_audit/rtk_stereo_calibration_output_audit.json \
  --multi-seed-summary-json Simulation/results/rtk_stereo_policy_multi_seed/multi_seed_summary.json \
  --robustness-summary-json Simulation/results/rtk_stereo_robustness_sweep/robustness_summary.json \
  --deployment-strategy-json Simulation/results/rtk_stereo_robustness_sweep/deployment_strategy_recommendation.json \
  --output-active-profile Simulation/results/rtk_stereo_acceptance_matrix_summary/active_runtime_profile.json \
  --promotion-check-json Simulation/results/rtk_stereo_acceptance_matrix_summary/active_profile_promotion_check/promotion_check.json \
  --output-dir Simulation/results/rtk_stereo_acceptance_matrix_summary
```

출력은 `rtk_stereo_acceptance_matrix_summary.json/csv/txt`이며, 단일 산출물
계약, seed/split 안정성, cross-scenario robustness/deployment strategy를
동일한 CSV row schema로 기록한다. 이 summary도 좌표값을 변환하지 않고,
PX4/RTK NED truth는 calibration/evaluation evidence로만, runtime profile은
stereo-center RDF와 measured disparity 입력으로만 사용됐는지 확인한다.
`--deployment-strategy-json`과 `--output-active-profile`을 함께 지정하면
검증된 deployment strategy에 따라 실제 StereoDistance에 넣을
`active_runtime_profile.json`도 복사하고, `active_profile_export` PASS row를
summary matrix에 추가한다. 이 row는 최종 export 파일 자체를 다시 읽어
runtime profile class로 로드하고 대표 raw RDF/BBox-center/disparity 입력에
apply할 수 있는지, runtime input이 truth/PX4/RTK/NED를 포함하지 않는지,
forbidden runtime input에 truth/PX4/RTK가 명시됐는지도 직접 검사한다.
deployment strategy가 `scenario_profile_catalog`이면
runtime truth가 아니라 보정/배포 메타데이터 또는 운용자 설정에서 온
`--scenario <label>`을 추가로 지정해야 한다.
export된 active profile을 StereoDistance promotion gate로 단독 재검증하려면
다음을 실행한다. `--sample-csv`는 `validation_predictions.csv`가 아니라 raw RDF,
left image center, measured disparity를 가진 `calibration_samples.csv`여야 한다.

```bash
python3 Simulation/scripts/stereodistance_profile_promotion_check.py \
  --profile-json Simulation/results/rtk_stereo_acceptance_matrix_summary/active_runtime_profile.json \
  --sample-csv Simulation/results/rtk_stereo_calibration_<RUN_ID>/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_acceptance_matrix_summary/active_profile_promotion_check \
  --split validation \
  --max-samples 50
```

이 promotion check는 runtime profile schema load, 실제 `StereoDistanceNode`
profile loader, axis contract 선언, forbidden runtime input 선언,
truth/PX4/RTK/NED-free runtime input, validation raw sample apply를 검사한다.
`--promotion-check-json`을 summary 명령에 함께 넣으면 이 독립 promotion 결과도
`standalone_profile_promotion` row로 acceptance matrix에 포함되어, sample apply
0건 또는 invalid sample이 있으면 전체 `overall_pass`가 실패한다.

강건/가중 Sim(3) 후보의 목적은 bbox center, origin 차이, 자세각 오차,
baseline/focal/disparity 오차가 섞여 만든 raw stereo RDF와 PX4/RTK truth
RDF 사이의 전역 estimated-to-truth 오차를 보정하는 것이다. 입력 가정은
train set에서만 PX4/RTK truth RDF와 raw RDF correspondence를 사용한다는
것이며, 출력 계약은 런타임에서 truth 없이 적용 가능한
`estimated_to_truth_sim3` profile(`scale`, `rotation_matrix_rdf`,
`translation_rdf_m`)이며, residual 후보는 named RDF axis별 disparity
polynomial 계수를 추가로 저장한다. disparity-then-Sim(3) 후보는 measured
disparity polynomial로 raw RDF ray를 먼저 scale한 뒤 같은 Sim(3) profile
계약으로 저장된다. disparity-only 후보는
`estimated_to_truth_disparity_poly` profile로 저장되며, runtime에서 measured
disparity를 보정한 뒤 raw RDF ray를 corrected forward/raw forward 비율로
스케일한다. RANSAC/trimmed 후보는 outlier consensus,
Huber/Tukey/robust-loss 후보는 robust M-estimation, range-weighted 후보는
weighted least-squares/Procrustes 목적식, target-weighted residual 후보는
목표거리 근방 가중 polynomial regression, ridge residual 후보는 정규화된
polynomial ridge regression, binned residual 후보는 quantile-binned median
calibration curve와 piecewise-linear interpolation을 기반으로 한다.
diagonal affine 후보는 RDF 축간 shear를 금지한 축별 gain+bias estimated-to-truth
보정으로, runtime에서는 truth 없이 raw RDF와 고정된 diagonal affine profile만
사용한다. identity-shrunk diagonal affine 후보는 gain을 1에 가깝게 묶어
validation/replay에서 안정성이 좋아지는지 확인한다.
raw-RDF residual field 후보는 thin residual vector field/regularized
regression 방식으로 Sim(3) 뒤에 남는 시야 위치 의존 bias를 설명할 수 있는지
비교하되, residual norm cap과 validation/replay 기준으로 과적합을 걸러낸다.
smoothed binned 후보는 empirical-Bayes shrinkage와 인접 bin smoothing,
capped binned 후보는 MAD/quantile 기반 winsorization으로 residual LUT
보정량을 제한한다. raw-forward binned 후보는 measured disparity 대신 raw
RDF forward 축을 LUT key로 사용해 disparity scale stress와의 결합을
비교한다. kernel-smoothed raw-forward 후보는 Gaussian kernel regression으로
거리별 residual curve를 smoothing해 bin 경계와 희소 sample에 대한 민감도를
줄이는지 비교한다. range-normalized raw-forward 후보는 residual 자체가 아니라
`residual / raw_forward` 비율을 먼저 fitting해 거리 비례 오차 성분이 더
안정적으로 전이되는지 확인한다. quadratic raw-forward 후보는 시차 bias의
1차 근사에서 거리 오차가 `Z^2`에 비례하는 성질을 반영해 장거리 bias를
더 적은 자유도로 설명할 수 있는지 확인한다. blended binned 후보는
disparity-key residual과 raw-forward-key residual을 grid-searched 비율로
함께 적용해 single-key 전이 실패가 줄어드는지 본다.
agreement-gated 후보는 두 key의 residual 예측이 크게 다를 때 residual 항을
끄거나 낮추는 runtime guard를 추가한다.
이 계열들은 모두 runtime에서는 truth 없이 measured disparity와 raw RDF만
사용한다.
Tukey biweight 후보는 bbox center 튐이나 correspondence outlier처럼 큰
잔차 sample이 일부 섞였을 때 해당 sample의 영향이 Huber보다 더 빨리
사라지는지 확인하기 위한 redescending M-estimator 후보이다. forward-only
LUT는 거리축 오차만 줄이는 보수적 후보이고, RDF-axis LUT는 bbox
center-origin 격차가 right/down 축에도 남는 경우를 탐색하는 확장 후보이다.
Pass 기준은 raw보다 validation range/position RMSE가 줄고, RDF 축별 RMSE가
후퇴하지 않으며, 1852m 근방 절대 평균 거리 오차가 허용치 안에 있고,
profile replay와 promotion check가 truth 입력 없이 통과하는 것이다.

최종 추천은 train RMSE가 아니라 validation range RMSE, 1852m 근방 절대 평균
오차, train/validation gap, 모델 복잡도를 함께 고려한다. 1852m 근방 절대
평균 거리 오차가 ±10m 안에 들어오고 raw보다 validation RMSE가 줄어드는
방법을 우선한다.

`estimated_to_truth_sim3` runtime profile은 내부 보정 벡터를 RDF
`[right, down, forward]`로 저장한다. 기존 StereoDistance position topic은
호환성을 위해 계속 FRD+range `[forward, right, down, range]` 순서로
발행된다. profile 적용 시 measurement runtime 입력은 raw stereo RDF와
measured disparity뿐이며, PX4/RTK truth나 true disparity는 사용하지 않는다.

보정 profile을 StereoDistance runtime에 적용하려면 pipeline 결과의
`sim3_runtime_profile.json`을 기존 `--bbox-adjustment` 경로에 넘긴다.

```bash
./run_operator_view.sh \
  --bbox-adjustment reports/experiments/EXP-20260613-010-sim3-disparity-global-calibration/output/sim3_runtime_profile.json
```

또는 launch 인자로 직접 지정할 수 있다.

```bash
ros2 launch stereo_ground_sim stereo_ground_sim.launch.py \
  enable_bbox_center_adjustment:=true \
  bbox_center_adjustment_path:=reports/experiments/EXP-20260613-010-sim3-disparity-global-calibration/output/sim3_runtime_profile.json
```

ROS/Gazebo를 띄우기 전, 기록 CSV에 대해 runtime profile과 같은 로직으로
offline replay를 할 수 있다. 이 검증은 CSV의 named RDF/NED/FRD 필드만 읽고,
runtime 입력으로 raw stereo RDF와 measured disparity만 사용한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/stereodistance_profile_replay.py \
  --input-csv Simulation/reports/experiments/EXP-20260613-010-sim3-disparity-global-calibration/output/calibration_samples.csv \
  --profile-json Simulation/reports/experiments/EXP-20260613-010-sim3-disparity-global-calibration/output/sim3_runtime_profile.json \
  --output-dir Simulation/reports/experiments/EXP-20260613-010-sim3-disparity-global-calibration/profile_replay \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30
```

Replay 출력:

- `profile_replay_summary.csv`: raw/corrected range, position, RDF axis RMSE
- `profile_replay_predictions.csv`: row별 truth/raw/corrected RDF와 error
- `profile_replay_range_error.png`: truth range별 raw/corrected 거리 오차
- `profile_replay_summary.txt`: 사람이 읽는 요약

## 실행

```bash
./run_operator_view.sh
```

이 단일 명령이 build, Gazebo Sim 서버, ROS-Gazebo bridge, 거리 계산 노드, PX4 SITL, 단일 `rqt_gui` 운용 창을 모두 실행합니다. 커스텀 앱을 만들지 않고 표준 ROS 도구만 실행합니다.
기본 실행은 자동 이동을 켜지 않고, PX4가 실험 타겟 `x500_15m`에 attach되므로 QGroundControl이 MAVLink UDP 14550으로 같은 드론을 조종할 수 있습니다. 실행만으로 이륙-접근-착륙까지 시작하려면 `--auto-mission`을 붙입니다.
`x500_15m`는 카메라 전방 +Y 약 2000 m 지점에서 yaw -90도로 스폰되어, QGC/PX4에서 전진 명령을 주면 카메라 방향으로 접근합니다. 스테레오 베이스라인은 X축 방향입니다.
Gazebo 3D GUI에서는 카메라 뒤에서 전방 +Y를 바라보는 운용자 기준으로 왼쪽 카메라는 world X 음수 위치의 파란색 본체, 오른쪽 카메라는 world X 양수 위치의 빨간색 본체로 표시됩니다. 카메라 모델 원점은 실제 sensor optical center이며, 렌즈 visual의 앞면이 이 원점에 맞춰집니다.
`rqt_gui` 안에는 상단 실험 HUD, 좌/우 표준 Image View 2개, 가운데 기록 패널, 하단에 상하 3단으로 쌓인 Plot 3개가 로드됩니다. HUD는 사용 중인 PyTorch 모델 파일명, mission 거리, profile 상태, YOLO Hz/ms, truth/raw/corrected FRD, Forward/Down `estimate-truth` 오차와 최근 시간창 RMSE, detection valid ratio, 좌표 계약을 표시합니다. 기본 화면은 카메라와 그래프가 최대한 넓게 보이도록 파라미터 패널을 열지 않습니다.
Image View는 기본적으로 YOLO bbox overlay 토픽인 `/left_camera/detection_image`, `/right_camera/detection_image`를 봅니다. 추론 부하를 낮추고 원본 화면을 보려면 `--raw-view`를 붙여 `/left_camera/image`, `/right_camera/image`를 표시합니다. Plot 3개는 각각 카메라 중앙 기준 FRD축 실제값(`/ground_truth_position`), raw YOLO 추정값(`/stereo/estimated_position`), bbox 중심 보정 추정값(`/stereo/corrected_position`)을 비교합니다. 축 계약은 StereoDistance와 같은 비교 계약으로 `data[0]=Forward`, `data[1]=Right`, `data[2]=Down`입니다.
Gazebo visible 카메라 센서는 기본 20 Hz(`CAMERA_UPDATE_RATE_HZ=20.0`), 1280x720이며 frame interval은 50,000,000 ns입니다. Ground Truth는 별도 adapter가 raw `Pose_V`의 simulation stamp를 보존해 발행하고, S1의 비교 frame은 고정 `baseline_center_frd`입니다. 이전 model-origin/Down offset 설명은 폐기된 historical 기록이며 active 경로에서 사용하지 않습니다. 거리/오차 그래프 및 overlay timer는 이 Phase의 camera-rate contract와 별개입니다. 기본 실행은 Gazebo 3D GUI도 함께 엽니다(`gz_gui:=true`). 카메라/그래프 Hz를 우선해 headless로 돌리려면 `gz_gui:=false`를 붙입니다.
각 Plot의 시간축은 최근 30초 창으로 autoscroll되며, 값 축은 현재 보이는 데이터 범위에 맞춰 자동으로 다시 잡히도록 실행 시 rqt_plot 패치를 적용합니다. 시간 창 길이는 `--plot-window 30`처럼 바꿀 수 있습니다.
기본 YOLO 설정은 `config/yolo.env`에서 읽습니다. 기본 모델은 `yolov8n_uav.pt`, class는 `UAV`, confidence는 `0.10`입니다. `.pt` 파일은 git에 넣지 않는 machine-local 입력이므로, 다른 PC에서는 `Simulation/yolov8n_uav.pt`를 같은 위치에 두거나 `YOLO_MODEL_PATH`/`--yolo-model`로 다른 파일을 명시합니다. 이 설정은 left/right image pixel bbox detector만 바꾸며, `/ground_truth_position`과 거리 토픽의 stereo-center FRD 축 계약은 바꾸지 않습니다.
기본 실행은 YOLO 거리 계산을 `yolo_target_hz:=40.0`, `yolo_device:=cuda:0`, `yolo_imgsz:=1280`으로 요청합니다. 2 km 부근의 작은 드론 bbox를 양쪽 카메라에서 유지하고 YOLO stride 32 경고를 피하기 위해 카메라 가로 해상도와 같은 1280 입력 크기를 기본으로 사용합니다. 거리 노드는 `config/yolo.env`의 `YOLO_PYTHON` 또는 로컬 `.venv`, 없으면 `python3`로 실행합니다. CUDA가 실제로 잡히면 CUDA/FP16으로 좌/우 이미지를 batch 추론하고, CUDA가 잡히지 않으면 CPU로 fallback하더라도 YOLO 이미지 구독과 거리 계산을 계속 시도합니다. CPU fallback은 느릴 수 있으므로 필요하면 `--yolo-hz`나 `yolo_imgsz`를 낮춥니다.
YOLO runtime bootstrap은 기본적으로 CUDA PyTorch wheel을 요구합니다(`YOLO_TORCH_BACKEND=cuda`, `YOLO_TORCH_CUDA_INDEX_URL=https://download.pytorch.org/whl/cu126`). CPU-only 환경을 의도적으로 쓰려면 `YOLO_TORCH_BACKEND=cpu ./run_operator_view.sh ...`처럼 실행합니다. CUDA wheel이 설치되어도 OS에서 NVIDIA driver와 `/dev/nvidia0`, `/dev/nvidiactl`, `/dev/nvidia-uvm` 장치 노드가 정상이어야 `torch.cuda.is_available()`이 true가 됩니다.
YOLO bbox overlay 화면은 기본으로 켜집니다(`overlay_publish_hz:=20.0`). Overlay 이미지는 전용 `detection_overlay_node`가 raw HD 카메라와 `/stereo/detections` bbox 결과를 결합해 발행하므로, YOLO 거리 계산 루프와 이미지 그리기/rqt 표시 부하를 분리합니다. Overlay 노드는 최신 카메라 프레임을 캐시하고 좌/우 이미지를 교차 발행해 부하 피크를 줄입니다. 이 값은 목표 발행률이며, Gazebo/rqt/PX4 부하에 따라 실측 Hz는 낮아질 수 있습니다. YOLO 거리 계산에 자원을 우선 배정하려면 `--raw-view`를 붙여 overlay 발행을 끕니다.
YOLO 실제 추론 Hz는 `/stereo/yolo_update_hz`를 기준으로 확인합니다. `/target_distance`는 그래프 표시를 위해 마지막 거리값도 `plot_publish_hz`로 재발행하므로, 이 토픽 Hz가 곧 YOLO 추론 Hz를 의미하지는 않습니다.
실시간 기록은 앱 가운데의 Recorder 패널에서 `Start Lite`, `Start Full`, `Start Debug`, `Stop` 버튼으로 제어합니다. `Lite`는 Excel/MATLAB용 `telemetry.csv`, `Full`은 CSV와 좌/우 overlay MP4, `Debug`는 CSV/MP4와 재현용 rosbag을 함께 저장합니다. 새 `telemetry.csv`에는 `axis_contract=stereo_center_frd:forward_right_down`이 들어가며, 카메라 기준 실제 위치는 `drone_forward_m`, `drone_right_m`, `drone_down_m`, 카메라 기준 실제 3D 직선거리는 `ground_truth_m`으로 저장됩니다. 스테레오 3D 추정값은 `estimated_forward_m`, `estimated_right_m`, `estimated_down_m`, `estimated_range_m`과 raw YOLO alias `raw_forward_m/right_m/down_m`, bbox 보정 alias `corrected_forward_m/right_m/down_m`으로 저장합니다. 새 CSV는 공개 축 필드로 legacy `x/y/z`를 내보내지 않습니다. 명령 실행과 동시에 기록을 시작해야 하는 경우에는 `--record-lite`, `--record-full`, `--record-debug` 옵션을 사용할 수 있습니다.
저장된 debug rosbag을 같은 `rqt_gui` 창 안의 Bag 플러그인으로 확인하려면 Plot 대신 Bag 구성을 선택합니다.

```bash
./run_operator_view.sh --bag-view
```

이 환경의 ROS 2 Humble `rqt_bag`과 `rqt_plot`은 같은 perspective로 자동 로드하면 시작 중 충돌할 수 있어, 기본 관찰 구성은 카메라+그래프이고 `--bag-view`는 카메라+Bag 확인 구성입니다.

관찰과 저장을 한 번에 시작하려면:

```bash
./run_operator_view.sh --record-lite
```

YOLO `.pt`를 바로 지정하려면:

```bash
./run_operator_view.sh --yolo-model yolov8n_uav.pt --yolo-class UAV --yolo-hz 20
```

파일 선택 창으로 `.pt`를 고르려면:

```bash
./run_operator_view.sh --select-yolo-model --yolo-class UAV --yolo-hz 20
```

YOLO bbox 보정 profile을 적용하려면:

```bash
./run_operator_view.sh --bbox-adjustment ../ref/Stereo_Camera_3D_Position_DKL/Cent_Bbox_Adj_Mtrx.csv
```

`--bbox-adjustment`는 기존 6x4 CSV bbox-center affine profile과 JSON profile을 모두 받을 수 있습니다.
JSON `origin_bbox_output_ridge` profile은 원본 bbox 좌표와 raw stereo FRD 추정값을 feature로 사용해 `/stereo/corrected_position`을 직접 보정합니다.
JSON `bounded_gain_forward_cell_guard` profile은 EXP009 계열 runtime policy로, 보정용 데이터에서 고정한 guard/gain/cell 정책을 사용해 ground truth 없이 `/stereo/corrected_position`을 발행합니다. 이 profile은 bbox anchor profile, right-camera rotation, `profile_family`, Down cell 정책을 함께 포함해야 합니다.
JSON `estimated_to_truth_sim3` profile은 보정 비행에서 학습한 Sim(3)
RDF 정합 파라미터를 raw stereo RDF에 적용해 ground truth 없이
`/stereo/corrected_position`을 발행합니다. 내부 계산은 RDF
`right/down/forward`이고, topic 출력은 기존 FRD+range
`forward/right/down/range` 순서를 유지합니다.
JSON `estimated_to_truth_affine_rdf` profile은 같은 RDF 계약에서
`p_corr = A * p_raw + t`를 적용합니다. 이 profile은 optional
`forward_residual_disparity_lut`, `forward_residual_raw_forward_lut`, 또는
`rdf_residual_disparity_lut`를 포함할 수 있습니다. disparity residual LUT는
보정 비행 truth로 fitting한 고정 LUT를 runtime measured disparity에
적용하고, raw-forward residual LUT는 runtime raw RDF forward 값에 적용합니다.
JSON
`estimated_to_truth_disparity_poly` profile은 measured disparity polynomial로
raw RDF ray scale을 보정합니다. 세 profile 모두 보정 비행 truth로 파라미터를
학습하지만 runtime 입력은 raw RDF와 measured disparity뿐입니다.
예시:

```bash
./run_operator_view.sh --bbox-adjustment reports/origin_bbox_output_profile_20260609_1415/origin_bbox_output_profile.json
```

실행 중 같은 `rqt_gui` 창에서 Dynamic Reconfigure로 `yolo_model_path`, `yolo_class_name`, `yolo_confidence`, `yolo_device`를 바꾸려면 `--params`를 붙여 실행합니다. `yolo_model_path`를 바꾸면 노드가 `.pt`를 다시 로드합니다.

```bash
./run_operator_view.sh --params
```

거리 계산 주기를 높이고 싶으면 `--yolo-hz` 또는 launch 인자 `yolo_target_hz`를 조정합니다. 이 값은 화면 표시 Hz가 아니라 YOLO 추론 Hz입니다. 그래프 토픽 발행 Hz는 `--plot-hz` 또는 `plot_publish_hz`로 조정합니다. Ground Truth 원본 시각은 Gazebo `Pose_V` adapter가 보존합니다.

```bash
./run_operator_view.sh yolo_target_hz:=40.0
./run_operator_view.sh --plot-hz 30
./run_operator_view.sh --plot-window 30
```

CUDA/CPU 장치를 명시하거나, 부하가 부족하면 추론 Hz 또는 입력 크기를 더 줄일 수 있습니다.

```bash
./run_operator_view.sh --yolo-device cuda:0
./run_operator_view.sh --yolo-device cpu
./run_operator_view.sh --yolo-python python3
./run_operator_view.sh yolo_target_hz:=0.0
./run_operator_view.sh yolo_imgsz:=256
```

거리 노드의 Python 자체를 바꾸려면 `--yolo-python` 또는 launch 인자 `yolo_python:=/path/to/python`을 사용합니다. 기본 실행은 로컬 `.venv`를 우선 찾고, 없으면 `python3`를 사용합니다. 별도 site-packages를 써야 하면 `YOLO_PYTHONPATH=/path/to/site-packages ./run_operator_view.sh ...`처럼 지정합니다.

자동 접근은 QGC/PX4 제어 경로로 수행합니다. 기존 Gazebo `cmd_vel` 미션 옵션은 QGC 조종과 충돌하므로 사용하지 않습니다.
드론 스폰 위치와 PX4 자동 미션 기본값은 `config/drone_mission.env`에서 수정할 수 있습니다.

```bash
DRONE_SPAWN_X_M=0.0
DRONE_SPAWN_Y_M=2000.0
DRONE_SPAWN_Z_M=5.56
DRONE_SPAWN_YAW_DEG=-90.0

DRONE_TAKEOFF_ALTITUDE_M=10.0
DRONE_FORWARD_SPEED_MPS=15.0
DRONE_MOVE_DISTANCE_M=1500.0
DRONE_TARGET_DISTANCE_M=500.0
DRONE_APPROACH_TIMEOUT_S=600.0
DRONE_LAND_DURATION_S=10.0
DRONE_LAND_TIMEOUT_S=90.0
```

`DRONE_SPAWN_*` 값은 `./run_operator_view.sh`와 자세각 실험이 Gazebo 월드를 만들 때 적용합니다. 기본값은 카메라 전방 2000 m에서 스폰한 뒤 1500 m 접근해 계획 임무를 수행하도록 맞춥니다. Ground Truth를 읽지 못하면 `vehicle_reference_link` 상태를 unavailable로 기록하며 model-origin, PX4 pose, 계획 거리값으로 대체하지 않습니다. `DRONE_TAKEOFF_ALTITUDE_M`, `DRONE_FORWARD_SPEED_MPS`, `DRONE_MOVE_DISTANCE_M`, `DRONE_TARGET_DISTANCE_M` 값은 자동 미션, waypoint 미션, 자세각 실험에서 공통 기본값으로 사용합니다. 명령줄 옵션과 launch 인자를 같이 주면 명령줄 값이 우선합니다.

Gazebo, rqt, PX4 실행 후 자동 이륙-접근-착륙까지 한 번에 시작하려면:

```bash
./run_operator_view.sh --auto-mission
```

기본 자동 임무는 PX4 SITL이 붙을 시간을 기다린 뒤 `./run_auto_mission.sh`를 실행합니다. 빠른 확인용 설정은:

```bash
./run_operator_view.sh --auto-mission --quick-mission
```

이미 `./run_operator_view.sh`로 Gazebo, PX4 SITL, QGC 연결이 떠 있는 상태에서 자동 이륙-접근-착륙만 실행하려면 별도 터미널에서 아래 명령을 실행합니다.

```bash
./run_auto_mission.sh
```

`run_auto_mission.sh`는 기본적으로 `Simulation/.venv/bin/python`을 사용하므로
venv를 별도로 activate하지 않아도 `pymavlink`를 찾는다. 다른 Python을 써야
할 때만 `MISSION_PYTHON=/path/to/python ./run_auto_mission.sh`처럼 지정한다.

기본 동작은 `config/drone_mission.env` 기준으로 이륙, 호버, PX4 local `north` 음수 방향 접근, Gazebo Ground Truth가 목표 거리 이하가 되면 호버, 착륙입니다. 현재 월드에서는 이 축이 Gazebo Y 거리를 줄이는 카메라 방향 접근입니다. Ground Truth가 unavailable인 경우에도 임무 명령은 `DRONE_MOVE_DISTANCE_M` local displacement를 상한으로 계속할 수 있지만, 그 계획값은 `/ground_truth`, actual distance, distance error에 절대 사용하지 않습니다. PX4 기본 `MPC_XY_CRUISE`/`MPC_XY_VEL_MAX` 제한 때문에 명령 속도보다 실제 접근 속도가 낮을 수 있습니다. 한 번만 목표 거리나 속도를 바꾸려면:

```bash
./run_auto_mission.sh --target-distance-m 200 --altitude-m 10 --speed-mps 15
```

이 명령은 Gazebo나 rqt 앱을 다시 띄우지 않고, 실행 중인 PX4에 MAVLink offboard 명령만 병렬로 보냅니다. 기본 연결 포트는 PX4 SITL offboard 포트인 `udpin:0.0.0.0:14540`입니다.
임무를 시작하지 않고 연결만 확인하려면:

```bash
./run_auto_mission.sh --check-connection
```

`./run_operator_view.sh`의 기본값은 `gz_gui:=true`, `launch_rviz:=false`입니다. 기본 실행에서 Gazebo 3D GUI와 rqt 카메라/그래프 창이 함께 뜹니다. RViz Marker/3D도 같이 보고 싶으면 아래처럼 켭니다.

```bash
./run_operator_view.sh launch_rviz:=true
```

인자를 넘길 수도 있습니다.

```bash
./run_operator_view.sh right_yaw_error_deg:=0.5
./run_operator_view.sh approach_duration_s:=20.0
```

`./run_x500_mission.sh`는 이전 Gazebo `cmd_vel` 방식이라 현재 QGC/PX4 모드에서는 비활성화되어 있습니다.

카메라/그래프 Hz를 우선해서 Gazebo 3D GUI를 끄려면:

```bash
./run_operator_view.sh gz_gui:=false
```

ROS/Gazebo 로그 디렉터리 권한 문제가 있으면:

```bash
export ROS_LOG_DIR=/tmp/ros2-launch-log
export GZ_CONFIG_HOME=/tmp/gz-config
```

운용 화면 가운데의 Recorder 패널에서 보통은 `Start Lite`를 눌러 CSV만 저장하고, 화면 확인용 영상까지 필요할 때는 `Start Full`을 사용합니다. 원본 ROS 토픽 재생/재처리까지 필요할 때만 `Start Debug`를 사용합니다.
명령 실행과 동시에 기록을 시작해야 하면 아래처럼 실행할 수도 있습니다.

```bash
./run_operator_view.sh --record-lite
```

명령형 기록의 저장 위치를 지정할 수도 있습니다.

```bash
./run_operator_view.sh --record-lite bags/yaw_0p25deg
```

## 주요 토픽

```text
/left_camera/image
/right_camera/image
/left_camera/detection_image
/right_camera/detection_image
/left_camera/camera_info
/right_camera/camera_info
/stereo/disparity
/ground_truth
/ground_truth_position
/stereo/estimated_position
/stereo/corrected_position
/stereo/bbox_adjustment
/stereo/distance_error
/stereo/distance_comparison
/stereo/distance_marker
/target_distance
```

거리 값 확인:

```bash
ros2 topic echo /target_distance
```

실제 거리와 측정 오차 확인:

```bash
ros2 topic echo /ground_truth
ros2 topic echo /ground_truth_position
ros2 topic echo /stereo/estimated_position
ros2 topic echo /stereo/corrected_position
ros2 topic echo /stereo/distance_error
```

한 토픽에서 같이 보려면:

```bash
ros2 topic echo /stereo/distance_comparison
```

`/stereo/distance_comparison` 배열 순서는 다음과 같습니다.

```text
[actual_m, measured_m, error_m, disparity_px]
```

`/ground_truth_position` 배열 순서는 다음과 같습니다.

```text
[forward_m, right_m, down_m, ground_truth_m]
```

여기서 배열은 StereoDistance와 같은 FRD 비교 계약입니다. `data[0]`은 전방, `data[1]`은 오른쪽, `data[2]`는 아래쪽입니다. `ground_truth_m`은 카메라 위치 기준 실제 3D 직선거리이며, `estimated_range_m` 비교용입니다. `/ground_truth` 토픽과 `/stereo/distance_comparison`의 `actual_m`은 카메라 전방축 실제 거리 `forward_m`을 발행합니다.
새 CSV에서는 `drone_forward_m/right_m/down_m`과 `estimated_forward_m/right_m/down_m`을 표준으로 사용하고 legacy `x/y/z` 공개 필드는 쓰지 않습니다. 과거 CSV에는 `drone_x_m/y_m/z_m`과 `estimated_x_m/y_m/z_m`가 남아 있을 수 있으므로, reader는 `axis_contract`를 확인해 `stereo_center_frd:forward_right_down`이면 legacy `x/y/z=forward/right/down`, 누락/구버전이면 변경 전 RDF인지 구분해야 합니다.

`/stereo/estimated_position` 배열 순서는 다음과 같습니다.

```text
[estimated_forward_m, estimated_right_m, estimated_down_m, estimated_range_m]
```

`/stereo/corrected_position` 배열 순서도 같습니다.

```text
[corrected_forward_m, corrected_right_m, corrected_down_m, corrected_range_m]
```

추정값도 같은 FRD 축 계약을 사용합니다. `/target_distance`는 raw `estimated_forward_m`을 발행합니다. CSV bbox-center affine 보정에서는 `/stereo/bbox_adjustment`가 `[left_corrected_cx, left_corrected_cy, right_corrected_cx, right_corrected_cy, left_du_px, left_dv_px, right_du_px, right_dv_px]`를 발행합니다. JSON output profile은 bbox 중심을 옮기지 않고 FRD 결과를 직접 보정하므로 `/stereo/corrected_position`만 비교하면 됩니다.

## 파라미터 확인/변경

```bash
ros2 param get /stereo_distance_node baseline_m
ros2 param get /stereo_distance_node horizontal_fov_rad
ros2 param get /stereo_distance_node approach_start_distance_m
ros2 param get /stereo_distance_node approach_end_distance_m
ros2 param get /stereo_distance_node approach_duration_s
ros2 param get /stereo_distance_node mission_profile
ros2 param get /stereo_distance_node takeoff_altitude_m
ros2 param get /stereo_distance_node fly_target_in_gazebo
ros2 param get /stereo_distance_node yolo_model_path
ros2 param get /stereo_distance_node yolo_class_name
ros2 param get /stereo_distance_node yolo_confidence

ros2 param set /stereo_distance_node baseline_m 25.0
ros2 param set /stereo_distance_node horizontal_fov_rad 0.154997710698
ros2 param set /stereo_distance_node approach_duration_s 120.0
ros2 param set /stereo_distance_node yolo_model_path /path/to/drone.pt
ros2 param set /stereo_distance_node yolo_class_name drone
```

카메라 위치, 높이, 타겟 크기, 해상도, FPS는
`stereo_ground_sim/worlds/stereo_ground_drone.sdf`에서 수정합니다.
기본 ground는 `4000 m x 4000 m`이고 월드 기준 x=0..4000, y=-2000..2000 범위를 덮습니다. 실행 시 `stereo_ground_sim.launch.py`가 같은 범위에 100 m 간격 grid line visual을 생성합니다. 카메라 모델 원점은 sensor optical center이고, Gazebo GUI 확인용 본체와 렌즈 visual은 그 기준점 뒤쪽에 배치됩니다.

카메라 자세각은 자주 바꾸는 값이므로 `config/camera_attitude.env`에서 수정할 수 있습니다.

```bash
RIGHT_YAW_ERROR_DEG=0.25
```

파일을 저장한 뒤 `./run_operator_view.sh`를 다시 실행하면 적용됩니다. 명령줄 인자를 함께 주면 명령줄 인자가 우선합니다.

PX4 waypoint 경로도 같은 방식으로 `config/waypoints.env`에서 밖으로 뺄 수 있습니다.

```bash
WAYPOINTS_XY_ALT_M="-300,0,10;-600,0,10;-900,0,10"
WAYPOINT_SPEED_MPS=12.0
WAYPOINT_ACCEPTANCE_RADIUS_M=5.0
WAYPOINT_LAND_AFTER=true
WAYPOINT_RECORD_LITE=true
```

`WAYPOINTS_XY_ALT_M`의 각 항목은 `north,east,altitude_agl_m` 순서이며, 항목은 세미콜론으로 구분합니다. PX4 내부 `LOCAL_NED` 좌표계에서 north/east는 x/y이고, 고도는 사람이 보기 쉽게 양수 AGL 값으로 적습니다. 기본 waypoint는 `config/drone_mission.env`의 `DRONE_MOVE_DISTANCE_M`와 `DRONE_TAKEOFF_ALTITUDE_M`를 참조합니다. 파일을 수정한 뒤 `./run_operator_view.sh`로 Gazebo/PX4를 먼저 띄우고 별도 터미널에서 실행합니다.
현재 시뮬레이션 축 기준에서는 카메라 방향 접근이 `north` 음수 방향이고, `east`는 카메라 베이스라인과 같은 횡방향입니다. 예를 들어 시작 거리 2000 m에서 카메라까지 415 m 지점으로 가려면 local north는 `-(2000 - 415) = -1585`입니다.

```bash
./run_px4_waypoint_mission.sh
```

이 명령은 기본적으로 lite CSV도 함께 저장합니다. 저장 위치는 `bags/waypoint_YYYYmmdd_HHMMSS_lite`이고, `telemetry.csv`에는 실제 Forward/Right/Down/GroundTruth, 스테레오 3D 추정값, 거리/오차/YOLO bbox/confidence 같은 수치 로그가 기록됩니다. 저장 위치를 바꾸거나 끄려면:

```bash
./run_px4_waypoint_mission.sh --record-dir bags/waypoint_test_01
./run_px4_waypoint_mission.sh --no-record
```

한 번만 다른 경로를 쓰려면 명령줄 인자가 설정 파일을 대체합니다.

```bash
./run_px4_waypoint_mission.sh \
  --waypoint -300,0,10 \
  --waypoint -600,0,10 \
  --speed-mps 8
```

## 카메라 자세각 오차 실험

launch 인자로 좌/우 카메라 roll, pitch, yaw 오차를 도 단위로 줄 수 있습니다.
거리 계산 노드는 이상적인 평행 스테레오 조건에서 bbox 중심을 3D 위치로 복원하므로, RViz2와 토픽에서 자세각 오차가 3D 추정값과 거리 측정값에 주는 영향을 관찰할 수 있습니다.
단일 `rqt_gui` 운용 창에는 좌/우 원본 카메라 영상과 하단에 상하 3단으로 쌓인 Forward/Right/Down축 실제값-raw 추정값-corrected 추정값 비교 그래프가 표시됩니다.
그래프는 표준 `rqt_plot` 플러그인 3개로 나뉩니다. Forward Axis Plot은 `/ground_truth_position/data[0]`, `/stereo/estimated_position/data[0]`, `/stereo/corrected_position/data[0]`, Right Axis Plot은 각 topic의 `data[1]`, Down Axis Plot은 각 topic의 `data[2]`를 표시합니다. 런타임 패치는 각 plot 제목과 Y축 라벨에 `Forward`, `Right`, `Down` 이름을 명시합니다.
거리 측정은 왼쪽/오른쪽 영상에서 같은 YOLO class의 bbox 중심을 찾고, disparity와 영상 중심 오프셋으로 내부 RDF를 계산한 뒤 FRD topic 순서인 `estimated_forward_m`, `estimated_right_m`, `estimated_down_m`, `estimated_range_m`으로 발행합니다. bbox 중심 보정이 켜져 있으면 같은 순서로 `corrected_forward_m`, `corrected_right_m`, `corrected_down_m`, `corrected_range_m`도 발행합니다. 기존 거리 그래프와 `/target_distance`는 이 3D 복원 결과 중 raw 카메라 전방 거리인 `estimated_forward_m`을 사용합니다. `/ground_truth_position`의 `ground_truth_m`은 카메라 기준 FRD로 계산한 3D 직선거리이므로 `estimated_range_m` 및 `corrected_range_m`과 직접 비교합니다.
자동 접근 실험은 QGC mission 또는 PX4용 MAVLink/offboard 명령으로 수행합니다. 이전 `./run_x500_mission.sh`의 Gazebo `cmd_vel` 방식은 현재 QGC/PX4 제어와 충돌하므로 비활성화되어 있습니다.

자세각 오차 정량 실험은 아래 명령을 사용합니다. 기본 실행은 Gazebo 이미지와 YOLO 거리 계산을 포함한 반복 실험입니다.
실행 전에 이전 `attitude_error` Gazebo/ROS 프로세스가 남아 있으면 자동으로 종료한 뒤 새 실험을 시작합니다. 이 자동 정리를 막고 기존처럼 즉시 실패하게 하려면 `--no-stop-existing-sim`을 사용하고, 의도적으로 기존 시뮬레이션과 함께 실행하려면 `--allow-existing-sim`을 사용합니다.

```bash
./run_attitude_error_experiment.sh
```

기본 Gazebo/YOLO 실험은 오른쪽 카메라 roll/pitch/yaw 축을 대상으로 `+0.25도` 오차를 테스트합니다. 음수 오차도 포함하려면:

```bash
./run_attitude_error_experiment.sh --include-negative
```

각도 오차 범위는 `config/attitude_error.env`에서 조절합니다. 예를 들어 `-1.0도`부터 `+1.0도`까지 `0.25도` 간격으로 테스트하려면:

```bash
ATTITUDE_EXPERIMENT_ANGLE_MIN_DEG=-1.0
ATTITUDE_EXPERIMENT_ANGLE_MAX_DEG=1.0
ATTITUDE_EXPERIMENT_ANGLE_STEP_DEG=0.25
```

`0도` 케이스는 별도 `baseline`과 중복되므로 자동으로 제외됩니다.

축마다 다른 범위와 단계를 쓰려면 `AXIS:MIN:MAX:STEP` 형식의 `--axis-range`를 반복해서 줍니다.

```bash
./run_attitude_error_experiment.sh \
  --axis-range right_roll_error_deg:-5:5:0.5 \
  --axis-range right_pitch_error_deg:-2:2:0.25 \
  --axis-range right_yaw_error_deg:-10:10:1
```

설정 파일에서는 공백으로 구분합니다.

```bash
ATTITUDE_EXPERIMENT_AXIS_RANGES="right_roll_error_deg:-5:5:0.5 right_pitch_error_deg:-2:2:0.25 right_yaw_error_deg:-10:10:1"
```

축별 범위가 지정된 축은 전역 `ATTITUDE_EXPERIMENT_ANGLE_MIN_DEG`, `ATTITUDE_EXPERIMENT_ANGLE_MAX_DEG`, `ATTITUDE_EXPERIMENT_ANGLE_STEP_DEG` 대신 해당 축별 값을 사용합니다.

Gazebo/YOLO 실험 결과는 `reports/attitude_error_*/summary.csv`와 케이스별 CSV/log로 저장됩니다. `summary.csv`에는 케이스별 좌/우 roll/pitch/yaw, 마지막 실제 Forward/Right/Down/GroundTruth, 마지막 3D 추정값, baseline, 카메라 높이, FOV, 해상도 같은 카메라 모델 정보도 함께 들어가며, 공통 실행 조건은 `metadata.txt`에도 저장됩니다.

카메라 모델 기본값은 `config/camera_model.env`에서 공통으로 바꿉니다. YOLO 기본값은 `config/yolo.env`에서 공통으로 바꿉니다. 두 파일 값은 operator view와 자세각 실험에 같이 적용됩니다.

```bash
CAMERA_BASELINE_M=25.0
CAMERA_HEIGHT_M=5.0
CAMERA_HORIZONTAL_FOV_RAD=0.154997710698
CAMERA_IMAGE_WIDTH_PX=1280
CAMERA_IMAGE_HEIGHT_PX=720
CAMERA_UPDATE_RATE_HZ=20.0
CAMERA_CLIP_NEAR_M=0.1
CAMERA_CLIP_FAR_M=2500.0
```

```bash
YOLO_MODEL_PATH="yolov8n_uav.pt"
YOLO_CLASS_NAME=UAV
YOLO_CONFIDENCE=0.10
YOLO_DEVICE=cuda:0
YOLO_TARGET_HZ=40.0
YOLO_IMGSZ=1280
YOLO_PYTHON=python3
```

자세각 실험 범위 기본값은 `config/attitude_error.env`에서 바꿀 수 있습니다. 각도 단위는 도(deg)입니다.

```bash
ATTITUDE_EXPERIMENT_AXES="right_roll_error_deg right_pitch_error_deg right_yaw_error_deg"
ATTITUDE_EXPERIMENT_AXIS_RANGES=""
ATTITUDE_EXPERIMENT_ANGLE_MIN_DEG=0.25
ATTITUDE_EXPERIMENT_ANGLE_MAX_DEG=0.25
ATTITUDE_EXPERIMENT_ANGLE_STEP_DEG=0.25
ATTITUDE_EXPERIMENT_INCLUDE_NEGATIVE=false
ATTITUDE_EXPERIMENT_QUICK=false
```

다른 설정 파일을 임시로 쓰려면 `CAMERA_MODEL_CONFIG=config/my_camera.env ./run_operator_view.sh`, `YOLO_CONFIG=config/my_yolo.env ./run_operator_view.sh`, 또는 `ATTITUDE_EXPERIMENT_CONFIG=config/my_attitude.env ./run_attitude_error_experiment.sh`처럼 실행합니다.
드론 스폰/미션 설정도 같은 방식으로 `DRONE_MISSION_CONFIG=config/my_drone_mission.env ./run_operator_view.sh`, `DRONE_MISSION_CONFIG=config/my_drone_mission.env ./run_auto_mission.sh`, `DRONE_MISSION_CONFIG=config/my_drone_mission.env ./run_px4_waypoint_mission.sh`, `DRONE_MISSION_CONFIG=config/my_drone_mission.env ./run_attitude_error_experiment.sh`처럼 바꿀 수 있습니다.

예: 오른쪽 카메라 yaw를 1도 틀기

```bash
./run_operator_view.sh right_yaw_error_deg:=1.0
```

예: 양쪽 카메라 pitch 오차를 다르게 주기

```bash
./run_operator_view.sh \
  left_pitch_error_deg:=0.5 \
  right_pitch_error_deg:=-0.5
```

사용 가능한 인자:

```text
baseline_m
camera_height_m
approach_start_distance_m
approach_end_distance_m
approach_duration_s
target_spawn_x_m
target_spawn_y_m
target_spawn_z_m
target_spawn_yaw_deg
mission_profile
fly_target_in_gazebo
gz_gui
launch_rviz
takeoff_altitude_m
takeoff_duration_s
first_hover_duration_s
final_hover_duration_s
land_duration_s
left_roll_error_deg
left_pitch_error_deg
left_yaw_error_deg
right_roll_error_deg
right_pitch_error_deg
right_yaw_error_deg
yolo_model_path
yolo_class_name
yolo_confidence
yolo_device
yolo_python
plot_publish_hz
yolo_imgsz
```

## 드론 모델 선택

이 시뮬레이션의 주 목적은 카메라 자세각 오차가 거리 측정에 미치는 영향을 보는 것입니다.
그래서 기본 타겟은 2000 m 장거리에서도 영상 검출이 가능한 15 m 폭의 대형 쿼드콥터 형상으로 둡니다.

X500 또는 IRIS 모델로 시각 형상을 바꾸는 것은 가능하지만, 실제 모델 텍스처가 초록색 검출 조건과 맞지 않으면 거리 노드가 타겟을 놓칠 수 있습니다. 특히 2000 m 장거리에서는 실제 크기가 작으면 픽셀 수가 부족합니다. 이 경우에는 폭을 실험 조건에 맞게 스케일링하거나, 색상 검출 대신 YOLO/형상 기반 검출로 바꾸는 것이 좋습니다.
현재 거리 노드는 YOLO `.pt` 기반 검출을 기본으로 하므로, `.pt` 안에 `drone` 클래스가 있거나 `--yolo-class` / `yolo_class_name`을 실제 클래스명으로 맞춰야 합니다. 이 Python 환경에 `ultralytics`와 `torch`가 설치되어 있지 않으면 노드는 상태에 import 실패를 표시하고 거리를 계산하지 않습니다.

## 드론 깜빡임 완화

2000 m 장거리 카메라 영상에서는 얇은 텍스처 면이나 큰 배율의 얇은 프로펠러 메시가 픽셀 경계에서 깜빡이는 것처럼 보일 수 있습니다.
`x500_15m` 모델은 이 영향을 줄이기 위해 본체 위 로고 텍스처 plane을 제거하고, 네 개의 얇은 프로펠러 STL 시각 요소를 어두운 원통형 로터 디스크로 바꿨습니다.
본체, 암, 모터 형상은 유지하므로 타겟은 여전히 X500 기반 드론 형상입니다.
Z축 비교의 Ground Truth 기준점은 `vehicle_reference_link`이며, `base_link` 원점에 project-defined zero offset(`[0, 0, 0]` m, identity RPY)으로 정의합니다. 이 semantic reference는 mesh centroid, mass center, visual/collision AABB center가 아니며 이전 estimator-derived Z offset을 사용하지 않습니다. 출력 비교 frame은 nominal zero-error rig에서 한 번 구성한 `baseline_center_frd`입니다. 기본 `DRONE_SPAWN_Z_M=5.56`은 PX4/Gazebo mission spawn 설정이며 reference offset을 의미하지 않습니다.

## QGroundControl

`./run_operator_view.sh` 기본 실행은 PX4 SITL을 standalone Gazebo attach 모드로 시작합니다.
PX4는 이미 world 안에 있는 15 m급 `x500_15m` 타겟에 붙고, QGC는 PX4의 MAVLink UDP 14550 heartbeat를 받아 자동 연결됩니다. PX4 콘솔 출력은 기본적으로 버립니다. 디버깅 로그가 필요하면 `PX4_CONSOLE_LOG=/tmp/px4_qgc.log ./run_operator_view.sh`처럼 실행합니다.

PX4를 끄고 카메라/거리 계산만 실행하려면:

```bash
./run_operator_view.sh --no-px4
```

PX4/QGC 제어가 필요하면 `./run_operator_view.sh`를 기본값으로 다시 실행합니다.
Ground Truth adapter는 Gazebo `gz.msgs.Pose_V` 토픽(`/world/stereo_ground_drone/pose/info`)에서 `name == "base_link"`인 entity의 simulation stamp와 world ENU pose를 읽어 `/simulation/ground_truth/vehicle_reference_link` (`geometry_msgs/msg/PoseStamped`, `frame_id=gazebo_world_enu`)로 발행합니다. `base_link` pose에 zero transform을 적용한 semantic reference가 `vehicle_reference_link`이며, distance node가 이를 고정 `baseline_center_frd`로 변환합니다. pose 토픽을 읽지 못하면 GT unavailable로 표시하고 model-origin, PX4, 계획 거리값으로 fallback하지 않습니다.
반복 가능한 접근 실험이 필요하면 QGC mission 또는 PX4용 MAVLink/offboard 명령을 사용합니다.

## 기록 프리셋

```text
Lite  = telemetry.csv
Full  = telemetry.csv + left_overlay.mp4 + right_overlay.mp4
Debug = telemetry.csv + left_overlay.mp4 + right_overlay.mp4 + rosbag/
```

운용 중에는 직접 `ros2 bag record`를 실행하지 않아도 됩니다. 앱 가운데의 Recorder 패널에서 `Start Lite`, `Start Full`, `Start Debug`, `Stop` 버튼을 사용합니다. 실행과 동시에 기록을 시작하려면 `./run_operator_view.sh --record-lite`, `./run_operator_view.sh --record-full`, `./run_operator_view.sh --record-debug`을 사용할 수 있습니다. Debug rosbag은 MCAP 플러그인이 있으면 MCAP을 쓰고, 없으면 sqlite3로 자동 대체합니다. 저장된 bag을 단일 `rqt_gui` 창 안에서 열려면 `./run_operator_view.sh --bag-view`를 사용합니다.

## 변경 이력

- 2026-06-18: fresh-seed replay에서 단일 frame 보정 profile과 단순 raw/disparity/alpha-beta prefilter는 1852 m worst-case tail을 안정적으로 통과하지 못했습니다. 실패 원인은 전역 자세/원점 보정 부족보다 순간 disparity/BBox 측정 이상치가 장거리에서 증폭되는 문제로 좁혀졌습니다. 후속으로 `repeated_observation_trimmed_mean_n7_trim0p2`를 적용해 left image `u/v`와 measured disparity를 7회 독립 관측 후 trimmed mean으로 fuse하고 stereo-center RDF로 재삼각측량한 뒤 runtime profile을 적용했습니다. 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/repeated_observation_fusion_all_scenarios_followup/aggregate_summary/`에 CSV/TXT로 저장되어 있습니다. nominal, yaw/pitch, disparity noise, combined sensor, bbox outlier stress는 기존 bbox-runtime profile과 함께 STRICT_PASS이고, intrinsic-only stress는 calibrated profile이 range RMSE를 악화시켜 identity passthrough profile이 STRICT_PASS입니다. runtime 입력은 fused pixel/disparity와 raw RDF뿐이며 PX4/NED truth는 synthetic generation/audit 전용입니다. 내부 보정 좌표는 stereo-center RDF(right, down, forward), public StereoDistance topic 계약은 FRD+range(forward, right, down, range)입니다.
- 2026-06-18: repeated observation의 독립 오차 가정이 너무 낙관적인지 확인하기 위해 common-mode 오차를 포함한 multi-burst fusion도 검증했습니다. `multiburst_observation_*_b5x5_trim0p2_common0p5` 계열은 bbox outlier, combined sensor, disparity noise, intrinsic identity stress에서 모두 20/20 STRICT_PASS입니다. 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_followup/aggregate_summary/`에 저장되어 있습니다. 이 결과는 실제 운용에서 단일 BBox 중심 하나를 바로 보정하지 말고, 짧은 burst 단위로 pixel/disparity를 robust fusion한 뒤 보정 profile 또는 identity passthrough를 적용하는 구조가 더 적합하다는 증거입니다.
- 2026-06-18: 최신 multi-burst 결과를 StereoDistance runtime에서 읽을 수 있는 observation-fusion manifest로 export했습니다. manifest는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_followup/fresh_runtime_manifest/observation_fusion_runtime_manifest.json`이며 초기 fresh 기본 tier는 `fresh_common_mode_b5x5_aggregate_best`였습니다. audit, raw apply smoke, runtime profile apply smoke, input/output schema validation을 통과했습니다. 이 manifest는 image `left_u_px/left_v_px/measured_disparity_px` burst를 stereo-center RDF로 fuse하는 adapter 설정만 포함하며, PX4/NED truth나 validation metric은 runtime 입력으로 사용하지 않습니다.
- 2026-06-18: fresh multi-burst manifest를 전체 observation-fusion deployment package로 묶었습니다. 패키지는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_followup/fresh_deployment_package/`이며 acceptance check `PASS`입니다. 포함 검증은 manifest audit, bbox conversion/apply/profile apply, calibration CSV truth-column drop/extract/apply/profile apply, path equivalence, public topic payload validation/source equivalence/dry-run publish, input/output schema validation, StereoDistance node smoke, all-tier override smoke, operator-view fusion launch check입니다. 초기 node smoke는 package manifest default 5x5 tier로 `fused:MultiBurstObservationFusion:25`와 finite corrected range를 확인했습니다.
  EXP-215 full follow-up 이후 package default를 `fresh_common_mode_b7x3_aggregate_best`로 승격했습니다. EXP-226 이후 최신 all-tier override smoke는 12개 tier 모두 실제 `StereoDistanceNode` helper path에서 fused 상태를 확인했고, `fresh_independent_n7_low_latency`는 `fused:RepeatedObservationFusion:7`, default는 `fused:MultiBurstObservationFusion:21`, `fresh_common_mode_b9x3_range_quality`와 `fresh_common_mode_b9x3_target_tail_high_common`은 `fused:MultiBurstObservationFusion:27`, `fresh_common_mode_b12x3_target_tail_slow`는 `fused:MultiBurstObservationFusion:36`, `fresh_common_mode_b15x3_target_tail_mid_slow`는 `fused:MultiBurstObservationFusion:45`, `fresh_common_mode_b16x3_target_tail_upper_bound`는 `fused:MultiBurstObservationFusion:48`, 5x5 fallback tier들은 `fused:MultiBurstObservationFusion:25`로 PASS했습니다. operator-view tier override dry-run도 `fresh_independent_n7_low_latency`를 `--observation-fusion-tier`로 전달해 PASS했습니다. 패키지는 검증된 `active_runtime_profile.json`을 내부에 복사하고, package-dir operator-view dry-run이 package-local profile/manifest를 자동 해석하는지 확인해 PASS했습니다. inventory/checksum gate도 재생성했으며 `inventory/`와 `acceptance_check/`를 제외한 73개 파일, 161744 bytes를 기록합니다.
- 2026-06-18: multi-burst 결과 전체 CSV에서 latency frontier를 다시 산출했습니다. 새 산출물은 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_followup/aggregate_summary/multiburst_observation_latency_frontier.csv`, `multiburst_observation_method_aggregate.csv`, `multiburst_observation_latency_frontier.txt`입니다. 15관측 3x5 후보도 bbox/combined/disparity/intrinsic-identity stress 모두 STRICT_PASS이지만, 당시 all-scenario worst target error와 mean validation range RMSE 기준 best aggregate는 25관측 `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b5x5_trim0p2_common0p5`였습니다. 이후 EXP-215에서 21관측 7x3 후보가 이 기준을 갱신했습니다.
- 2026-06-18: EXP-214 expanded multi-burst probe로 `3x3/3x5/3x7/5x3/5x5/7x3/7x5` burst spec과 median/trimmed-mean inner/outer 조합을 추가 탐색했습니다. 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_expanded_probe/aggregate_summary_selector/`에 저장했습니다. selector-compatible 평가에서는 bbox/combined/disparity stress는 active runtime profile, intrinsic-only stress는 identity passthrough profile을 사용했으며 runtime truth는 사용하지 않았습니다. best aggregate probe는 `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p2_common0p5`로 4/4 STRICT_PASS, 21 observations, target worst `6.009134m`, mean range RMSE `1.402767m`입니다. 이 단계에서는 full-size 재검증 전이라 후보로만 기록했고, 이후 EXP-215 full follow-up에서 default 승격 조건을 통과했습니다. 좌표 계약은 image `left_u_px/left_v_px/measured_disparity_px` -> stereo-center RDF(right,down,forward) -> public FRD+range이며, PX4/NED truth는 synthetic generation/audit 전용입니다.
- 2026-06-18: EXP-215에서 EXP-214의 `7x3 trimmed_mean/trimmed_mean common0.5` 후보를 기존 full follow-up seed/sample 조건으로 재검증했습니다. 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_b7x3_full_followup/aggregate_summary/`에 저장했습니다. 4개 stress 모두 STRICT_PASS, target worst `5.063616m`, mean range RMSE `1.308669m`, mean position RMSE `3.624118m`로 기존 b5x5 default의 target worst `6.046387m`, mean range RMSE `1.326220m`, mean position RMSE `3.630421m`보다 모두 낮고 관측 수도 25에서 21로 줄었습니다. 이에 fresh deployment default를 `fresh_common_mode_b7x3_aggregate_best`로 승격하고 기존 b5x5 aggregate는 fallback tier로 유지했습니다.
- 2026-06-18: EXP-216에서 high-burst `9x3/11x3` 후보를 같은 full follow-up 조건으로 추가 탐색했습니다. 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_highburst3_followup/aggregate_summary/`에 저장했습니다. `9x3`는 4/4 STRICT_PASS, target worst `5.079371m`, mean range RMSE `1.296722m`, mean position RMSE `3.621114m`였고, `11x3`는 mean range RMSE `1.287791m`로 더 낮지만 target worst가 `5.392157m`로 악화되었습니다. default selection rule은 1852m target worst를 먼저 보므로 `7x3` default를 유지하고, `9x3`는 aggregate range RMSE를 더 중시할 때 쓰는 `fresh_common_mode_b9x3_range_quality` override tier로 추가했습니다.
- 2026-06-18: EXP-217에서 중간 후보 `8x3`과 burst 내부 관측을 늘린 `7x4`도 같은 full follow-up 조건으로 확인했습니다. 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_midburst_followup/aggregate_summary/`에 저장했습니다. `8x3`는 4/4 STRICT_PASS, target worst `5.553816m`, mean range RMSE `1.298221m`, 관측 24개였고, `7x4`는 4/4 STRICT_PASS, target worst `5.285188m`, mean range RMSE `1.307341m`, 관측 28개였습니다. 둘 다 기존 `7x3` default보다 target worst가 나쁘고, `7x4`는 관측수도 더 많으므로 manifest tier로 승격하지 않았습니다. 이 결과는 현재 target-worst 우선 선택에서는 burst 수/관측 수를 단순히 늘리는 것보다 `7x3` 구성이 더 안정적이라는 negative evidence로 기록합니다.
- 2026-06-18: EXP-218에서 active `7x3` 구조의 trim fraction을 `0.1/0.2/0.25/0.3/0.4`로 바꿔 같은 full follow-up 조건에서 재검증했습니다. 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_b7x3_trim_followup/aggregate_summary_with_baseline/`에 baseline `trim0.2` 포함 CSV/TXT로 저장했습니다. 목적은 StereoDistance runtime 입력 계약을 바꾸지 않고 hierarchical trimming만 조정해 target-window tail error와 validation RMSE가 더 안정적인지 확인하는 것입니다. 입력은 image `left_u_px/left_v_px/measured_disparity_px`를 7 bursts x 3 observations로 묶은 runtime 관측이며, PX4/NED truth는 synthetic evaluation/audit에만 사용합니다. 출력은 stereo-center RDF(right,down,forward)이고 public topic은 FRD+range(forward,right,down,range)입니다. 모든 trim 후보가 4/4 STRICT_PASS였지만, `trim0.1`은 target worst를 `5.018522m`로 약간 낮추는 대신 mean range RMSE `1.327366m`와 mean position RMSE `3.627202m`로 default보다 후퇴했습니다. `trim0.25`는 이 7x3 discrete geometry에서 `trim0.2`와 동일했고, `trim0.3/0.4`는 target worst가 각각 `5.222556m`/`5.442956m`로 악화되었습니다. 따라서 active default는 validation RMSE와 outlier robustness가 더 안정적인 `trim0.2`로 유지하고, `trim0.1`은 target-tail specialist 후보로만 기록합니다.
- 2026-06-18: EXP-219에서 within-burst common-mode image/disparity error를 더 강하게 둔 `common0.75/1.0` 조건으로 `3x5/5x5/7x3/9x3` 후보를 재검증했습니다. 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_common_mode_followup/aggregate_summary/`에 저장했습니다. 목적은 active `7x3` default가 높은 관측 상관에서도 유지되는지, 아니면 이미 추가한 `9x3` range-quality override가 더 적합한지 확인하는 것입니다. 입력/출력 계약은 EXP-218과 동일하게 image `left_u_px/left_v_px/measured_disparity_px` -> stereo-center RDF(right,down,forward) -> public FRD+range이며, PX4/NED truth와 common-mode fraction은 synthetic audit에만 사용합니다. 4개 stress 모두에서 best aggregate는 `9x3 trim0.2 common0.75`였고 4/4 STRICT_PASS, target worst `6.150405m`, mean range RMSE `1.389720m`, mean position RMSE `3.648680m`입니다. 같은 조건의 `7x3 common0.75`는 target worst `6.407873m`, mean range RMSE `1.406747m`였습니다. 따라서 기본 운용 default는 기존 `7x3 trim0.2 common0.5`로 유지하지만, 관측 상관이나 range RMSE 우선 운용에서는 `fresh_common_mode_b9x3_range_quality` override를 우선 후보로 본다는 근거를 추가합니다.
- 2026-06-18: EXP-220에서 high-common-mode `9x3` override의 trim fraction을 `0.1/0.2/0.25/0.3/0.4`로 다시 훑었습니다. 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_b9x3_trim_common_followup/aggregate_summary_with_baseline/`에 baseline `trim0.2` 포함 CSV/TXT로 저장했습니다. `trim0.1 common0.75`는 4/4 STRICT_PASS, target worst `5.986332m`로 high-common-mode 조건에서 가장 낮은 1852m tail error를 냈지만 mean range RMSE `1.403496m`, mean position RMSE `3.651919m`로 `trim0.2 common0.75`의 `1.389720m`/`3.648680m`보다 후퇴했습니다. 따라서 기존 `fresh_common_mode_b9x3_range_quality`는 range-quality override로 유지하고, target-tail 우선 운용을 위해 `fresh_common_mode_b9x3_target_tail_high_common` tier를 추가했습니다. package 재생성 후 manifest audit `9/9`, all-tier override smoke `9/9`, acceptance check `PASS`, runtime truth `false`입니다.
- 2026-06-18: EXP-221에서 high-common-mode 조건의 `b9x3` 주변 burst shape `8x3/8x4/10x3/10x4/11x3/12x3`와 trim `0.1/0.2`를 비교했습니다. 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_adjacent_shape_followup/aggregate_summary/`에 저장했습니다. best aggregate는 `12x3 trim0.1 common0.75`로 4/4 STRICT_PASS, 36 observations, target worst `5.839273m`, mean range RMSE `1.380696m`, mean position RMSE `3.646339m`였습니다. 이는 EXP-220의 `9x3 trim0.1 common0.75`보다 target-tail과 평균 RMSE가 모두 낮지만 27관측에서 36관측으로 늘어납니다. 따라서 default와 b9x3 lower-latency override는 유지하고, latency/motion budget이 허용될 때 쓰는 `fresh_common_mode_b12x3_target_tail_slow` tier를 추가했습니다. package 재생성 후 manifest audit `10/10`, all-tier override smoke `10/10`, acceptance check `PASS`, runtime truth `false`입니다.
- 2026-06-18: EXP-222에서 `12x3` 이후 포화 여부를 보기 위해 `12x3/14x3/16x3/12x4/14x4`, trim `0.1`, common `0.75`를 비교했습니다. 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_saturation_followup/aggregate_summary/`에 저장했습니다. best target-tail aggregate는 `16x3 trim0.1 common0.75`로 4/4 STRICT_PASS, 48 observations, target worst `5.691110m`, mean range RMSE `1.381157m`, mean position RMSE `3.647368m`였습니다. `12x3`보다 target worst는 낮지만 평균 range/position RMSE가 근소하게 후퇴하고 관측 수가 36에서 48로 늘어납니다. 따라서 `fresh_common_mode_b16x3_target_tail_upper_bound`를 latency-tolerant target-tail upper-bound tier로 추가하고, balanced slow target-tail은 `12x3`로 유지합니다. package 재생성 후 manifest audit `11/11`, all-tier override smoke `11/11`, acceptance check `PASS`, runtime truth `false`입니다.
- 2026-06-18: EXP-223에서 `b16x3` upper-bound tier의 trim/common 민감도를 `trim=0.05/0.10/0.15/0.20`, `common=0.5/0.75/1.0`로 다시 확인했습니다. 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_b16x3_sensitivity_followup/aggregate_summary/`에 저장했습니다. best aggregate는 `16x3 trim0.2 common0.5`로 4/4 STRICT_PASS, 48 observations, target worst `5.490163m`, mean range RMSE `1.360747m`, mean position RMSE `3.614085m`였습니다. 이는 EXP-222의 `16x3 trim0.1 common0.75`보다 target-tail, 평균 range RMSE, 평균 position RMSE를 모두 낮추므로 `fresh_common_mode_b16x3_target_tail_upper_bound` evidence를 EXP-223 결과로 교체했습니다. default는 여전히 21관측 `b7x3`, balanced slow target-tail은 36관측 `b12x3`이고, `b16x3`는 48관측 latency를 허용할 때만 쓰는 upper-bound override입니다. package 재생성 후 manifest audit `11/11`, all-tier override smoke `11/11`, acceptance check `PASS`, inventory `73` files / `156421` bytes, runtime truth `false`입니다.
- 2026-06-18: EXP-224에서 EXP-223의 best 조건인 `trim0.2 common0.5`를 고정하고 burst count `13x3/14x3/15x3/16x3/17x3/18x3/20x3`를 비교했습니다. 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_burst_count_common0p5_followup/aggregate_summary/`에 저장했습니다. all-scenario best는 여전히 `16x3 trim0.2 common0.5`로 4/4 STRICT_PASS, 48 observations, target worst `5.490163m`, mean range RMSE `1.360747m`, mean position RMSE `3.614085m`였습니다. `15x3`는 45 observations로 낮은 latency 후보지만 target worst가 `5.646591m`로 후퇴했고, `17x3/18x3/20x3`는 관측 수 증가에도 disparity stress target-tail이 악화되었습니다. 따라서 EXP-224는 `16x3`가 현재 48관측 upper-bound 포화점이라는 negative evidence로 기록하고, runtime manifest tier는 변경하지 않습니다.
- 2026-06-18: EXP-225에서 `16x3 trim0.2 common0.5` 구조를 유지한 채 inner/outer fusion mode를 `median/trimmed_mean/mean` 조합으로 비교했습니다. 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_b16x3_mode_followup/aggregate_summary/`에 저장했습니다. all-scenario best는 기존 `inner trimmed_mean / outer trimmed_mean`이며 4/4 STRICT_PASS, target worst `5.490163m`, mean range RMSE `1.360747m`, mean position RMSE `3.614085m`를 유지했습니다. `inner mean / outer trimmed_mean`은 aggregate 수치상 동률이지만 burst 내부 outlier에 대한 의미가 약하고, `median/median`은 bbox stress에서는 target-tail이 좋았으나 disparity stress worst가 `5.819561m`로 후퇴했습니다. 따라서 EXP-225도 runtime manifest tier 변경 없이 기존 `trimmed_mean/trimmed_mean` upper-bound를 유지하는 negative evidence로 기록합니다.
- 2026-06-18: EXP-226에서 45관측 `b15x3` lower-latency target-tail 후보의 trim fraction을 `0.10/0.15/0.25/0.30`로 비교했습니다. 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_b15x3_trim_followup/aggregate_summary/`에 저장했습니다. best aggregate는 `15x3 trim0.15 common0.5`로 4/4 STRICT_PASS, 45 observations, target worst `5.633070m`, mean range RMSE `1.367329m`, mean position RMSE `3.617117m`였습니다. 이는 EXP-224의 `15x3 trim0.2` target worst `5.646591m`보다 약간 좋고, 36관측 `b12x3`보다 target-tail이 낮지만 48관측 `b16x3` upper-bound보다는 후퇴합니다. 따라서 `fresh_common_mode_b15x3_target_tail_mid_slow` tier를 추가해 b12x3와 b16x3 사이의 latency/target-tail trade-off로 제공했습니다. package 재생성 후 manifest audit `12/12`, all-tier override smoke `12/12`, b15x3 override `fused:MultiBurstObservationFusion:45`, acceptance check `PASS`, inventory `73` files / `161744` bytes, runtime truth `false`입니다.
- 2026-06-18: EXP-227에서 36관측 `b12x3` slow target-tail tier도 EXP-223/226에서 강했던 `common0.5` 조건으로 재평가했습니다. trim fraction은 `0.10/0.15/0.20/0.25`를 비교했고 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_b12x3_common0p5_trim_followup/aggregate_summary/`에 저장했습니다. best aggregate는 `12x3 trim0.1 common0.5`와 discrete-equivalent `trim0.15`로 4/4 STRICT_PASS, 36 observations, target worst `6.206171m`, mean range RMSE `1.372707m`, mean position RMSE `3.617990m`였습니다. 이는 기존 `fresh_common_mode_b12x3_target_tail_slow` evidence인 `12x3 trim0.1 common0.75`의 target worst `5.839273m`보다 나쁘므로 b12x3 tier는 기존 high-common evidence를 유지합니다. EXP-227은 b12x3에서는 moderate common-mode가 target-tail을 개선하지 못한다는 negative evidence입니다.
- 2026-06-18: observation-fusion deployment acceptance checker에 default-tier policy gate, all-tier override smoke gate, operator-view tier override gate를 추가했습니다. 이제 package check는 manifest default와 package summary default가 일치하는지, default tier가 `selection_scope=all_scenario_aggregate`인지, adapter가 truth-free `MultiBurstObservationFusion`인지, aggregate evidence가 `STRICT_PASS`이며 pass/trial이 전부 통과인지 확인한다. 또한 manifest의 모든 tier override가 실제 node helper path에서 fused 상태로 통과하는지, operator wrapper가 저지연 tier override를 실제 `--observation-fusion-tier` 인자로 전달하는지 확인합니다. EXP-222 upper-bound target-tail tier 추가 후 fresh package acceptance는 `default_selection_scope=all_scenario_aggregate`, `default_evidence_pass=4/4`, `tier_override_smoke_pass=11/11`, `operator_view_fusion_tier_override=fresh_independent_n7_low_latency`, `failure_count=0`으로 다시 PASS했습니다.
- 2026-06-18: observation-fusion manifest를 실제 `stereo_distance_node` 실행 경로에 연결했습니다. 새 launch 파라미터는 `enable_observation_fusion`, `observation_fusion_manifest_path`, `observation_fusion_tier`입니다. 노드는 YOLO left/right bbox center에서 만든 runtime image observation(`left_u_px`, `left_v_px`, measured disparity)을 짧은 buffer에 쌓고, manifest adapter가 요구하는 개수에 도달하면 fused observation을 stereo-center RDF(`right,down,forward`)로 재삼각측량한 뒤 기존 bbox/runtime profile 보정 입력으로 사용합니다. warm-up 또는 fusion 실패 시 기존 단일-frame raw RDF를 그대로 사용합니다. 실행 예시는 다음과 같습니다.

```bash
./run_operator_view_with_calibration_profile.sh \
  --profile-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/fresh_guard_profile_sweep_followup/away_w100_a0p75/active_runtime_profile.json \
  --observation-fusion-manifest Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_followup/fresh_deployment_package/observation_fusion_runtime_manifest.json \
  -- --no-px4
```

runtime 입력은 영상 기반 `left_u_px/left_v_px/measured_disparity_px`와 그로부터 계산된 raw stereo-center RDF뿐입니다. PX4/RTK/NED truth는 calibration/evaluation evidence로만 사용하고 runtime path에는 들어가지 않습니다. `/stereo/estimated_position` 및 `/stereo/corrected_position` 공개 topic은 기존 FRD+range(`forward,right,down,range`) 순서를 유지합니다.
이 경로의 operator dry-run evidence는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_followup/fresh_deployment_package/operator_view_fusion_launch_check.json` 및 `.txt`에 저장되어 있으며, `observation_fusion_manifest_status=PASS`, `launch_contract_status=PASS`, `runtime_truth_used=false`를 기록합니다.
운용 중에는 `/stereo/profile_status` JSON에서 `observation_fusion_status`, `observation_fusion_adapter`, `observation_fusion_buffer_count`, `observation_fusion_required_count`를 확인할 수 있습니다. 예를 들어 default multi-burst tier는 required count가 burst 수와 burst당 observation 수의 곱으로 표시되며, warm-up 중에는 기존 단일-frame raw RDF가 계속 발행됩니다.
- 2026-06-18: EXP-228에서 현재 fresh observation-fusion deployment tier들의 offline portfolio upper bound를 진단했습니다. 도구는 `Simulation/scripts/summarize_observation_fusion_tier_portfolio.py`이며, 기본 비교는 `selection_scope=all_scenario_aggregate` tier만 포함합니다. single-source fallback tier는 scenario label이 aggregate tier와 직접 비교되지 않도록 기본 oracle에서 제외하고, 필요할 때만 `--include-single-source`로 포함합니다. 재실행 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_tier_portfolio_followup/EXP-20260618-228-tier-portfolio/`에 저장했습니다. aggregate-only oracle은 4개 scenario에서 `status=PASS`, `runtime_truth_used=false`, default tier `fresh_common_mode_b7x3_aggregate_best`, default worst target error `5.063616m`, oracle worst target error `5.063616m`로 동일했습니다. scenario별 oracle은 bbox outlier에서 27관측 `b9x3 range_quality`, combined/intrinsic stress에서 48관측 `b16x3 target_tail_upper_bound`, disparity noise에서 21관측 default `b7x3`를 골랐습니다. 따라서 현재 배포된 all-scenario aggregate tier들만 runtime selector로 고르는 방식은 worst-case target-tail을 더 줄이지 못하며, 다음 탐색은 disparity-noise stress 자체를 낮추는 새 관측/잔차 모델 또는 truth-free runtime feature 기반 selector로 진행해야 합니다. 이 진단은 offline validation only이고 runtime 입력은 계속 image `left_u_px/left_v_px/measured_disparity_px` 및 stereo-center RDF이며, PX4/NED truth는 사용하지 않습니다.
- 2026-06-18: EXP-229에서 EXP-228의 병목인 `disparity_noise_stress`만 대상으로 `b7x3` low-trim/mode probe를 수행했습니다. 공정 비교를 위해 EXP-218과 같은 scenario-loso profile 및 seed range(`21220000..21220019`)를 사용했고, 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_disparity_b7x3_lowtrim_mode_probe_matched/aggregate_summary/`에 저장했습니다. `trim0.05/0.10/0.15`와 inner/outer `median/trimmed_mean/mean` 조합은 모두 STRICT_PASS였지만 best target max는 기존 `b7x3 trim0.1`과 같은 `5.018522m`, range mean `1.997943m`에 머물렀습니다. median outer 조합은 target-tail이 더 나빴고, discrete 7x3 구조에서는 여러 low-trim/mean 조합이 같은 fused value로 수렴했습니다. 따라서 b7x3 mode/low-trim만으로는 disparity-noise 병목을 더 줄이지 못하며 manifest tier는 변경하지 않습니다.
- 2026-06-18: EXP-230/231에서 disparity-noise 병목에 causal alpha-beta motion innovation gate가 도움이 되는지 확인했습니다. EXP-230은 corrected RDF stream 단독 motion fusion을 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/motion_fusion_disparity_probe/`에 저장했고, best는 STRICT_PASS였지만 target max `8.555299m`, range mean `2.298741m`로 b7x3 observation fusion보다 나빴습니다. EXP-231은 새 도구 `Simulation/scripts/run_runtime_multiburst_motion_fusion_sweep.py`로 image `left_u_px/left_v_px/measured_disparity_px` multi-burst fusion 뒤 corrected stereo-center RDF에 motion gate를 추가했습니다. 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_motion_fusion_disparity_probe/forward/`에 저장했습니다. best는 `alpha=1.0`, `innovation_clip=40m`로 observation-only와 완전히 같은 target max `5.018522m`, range mean `1.997943m`였고, 더 강한 temporal gate는 target max를 20m 이상으로 악화시켰습니다. 따라서 현재 disparity-noise stress에서는 temporal motion smoothing/gating이 추가 개선을 만들지 못하며, 다음 탐색은 disparity/inverse-depth residual 자체 또는 runtime observable selector로 진행합니다. 좌표 계약은 두 단계 모두 stereo-center RDF이고 public topic은 계속 FRD+range입니다.
- 2026-06-18: EXP-232/233에서 multi-burst observation fusion 이후 남는 disparity/range residual을 calibration-time train split으로 fit하는 새 후보를 추가했습니다. 실행 도구는 `Simulation/scripts/run_runtime_multiburst_disparity_residual_sweep.py`이고, 요약 도구는 `Simulation/scripts/summarize_multiburst_disparity_residual_results.py`입니다. 이 방법은 train truth로만 residual polynomial을 학습하고, runtime 적용에는 fused `measured_disparity_px` 또는 `log_disparity`와 corrected stereo-center RDF만 사용합니다. PX4/NED truth, target label, true disparity는 runtime 입력이 아닙니다. `disparity_noise_stress` 단독 probe(EXP-232)는 기존 b7x3 observation-only target max `5.018522m`, range mean `1.997943m`를 `log_disparity` degree-2 ray-scale residual로 target max `1.645409m`, range mean `0.459445m`까지 낮췄습니다. 이어서 bbox/combined/intrinsic까지 같은 family를 확인한 EXP-233 aggregate 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_disparity_residual_probe/aggregate_summary/`에 저장했습니다. 4개 scenario 공통 best는 `b7x3 trim0.05 common0.5 + ray_scale log_disparity degree2 ridge1 clip0.5`로 4/4 STRICT_PASS, 21 observations, residual target worst `3.288941m`, residual range RMSE mean across scenarios `0.442031m`, residual position RMSE mean `2.946359m`입니다. scenario별로 disparity, combined, intrinsic은 target-tail과 range가 크게 개선됐고, bbox는 target max가 `3.220680m`에서 `3.288941m`로 약간 나빠졌지만 여전히 1852m ±10m gate 안입니다. 이 후보는 현재 default b7x3의 worst `5.063616m`와 EXP-228 aggregate-tier oracle worst `5.063616m`를 모두 낮추는 첫 residual-family 증거입니다. 단, 아직 deployment manifest/node runtime export에는 넣지 않았으므로 후속 작업은 residual coefficients export, loader/apply smoke, package check입니다.
- 2026-06-18: EXP-232/233 residual family를 runtime-loadable JSON까지 연결했습니다. `stereo_ground_sim.runtime_deployment_selector.DisparityResidualRuntimeProfile`은 JSON의 `coefficients_low_to_high`, `feature_mean`, `feature_scale`, `feature_space`, `correction_space`, `correction_clip`을 읽고 `RuntimeMeasurementSample`에 적용합니다. runtime apply 입력은 corrected stereo-center RDF(`right,down,forward`)와 fused `measured_disparity_px`뿐이며, public FRD topic이나 PX4/RTK truth는 입력이 아닙니다. Export 도구는 `Simulation/scripts/export_runtime_disparity_residual_profile.py`, loader/apply smoke 도구는 `Simulation/scripts/check_runtime_disparity_residual_profile.py`입니다. 대표 export는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_disparity_residual_probe/runtime_profile_export/disparity_noise_seed21220000_residual_profile.json`에 저장했고, validation smoke는 `.../runtime_profile_export/disparity_noise_seed21220000_check/`에 저장했습니다. smoke 결과는 `status=PASS`, validation rows `420`, observation range RMSE `1.965520m -> 0.438736m`, target max abs error `4.447238m -> 1.098600m`, `runtime_truth_used=false`입니다.
- 2026-06-18: EXP-235에서 EXP-234 residual profile을 fresh observation-fusion deployment package에 포함했습니다. `Simulation/scripts/run_observation_fusion_deployment_package.py`는 이제 선택 인자 `--disparity-residual-profile-json`을 받아 package-local `disparity_residual_profile.json`으로 복사하고, `StereoDistanceNode` smoke에서 observation fusion -> base runtime profile -> residual profile 적용 순서를 확인합니다. `Simulation/scripts/check_observation_fusion_deployment_package.py`는 residual profile이 포함된 경우 `DisparityResidualRuntimeProfile` type, truth-free runtime input, coordinate contract, source hash, node smoke `applied:*` 상태를 acceptance gate로 검증합니다. 재생성한 fresh package는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_followup/fresh_deployment_package/`이며 acceptance `PASS`, manifest audit `12/12`, tier override smoke `12/12`, default node smoke `fused:MultiBurstObservationFusion:21`, residual smoke `applied:disparity_noise_stress_seed_21220000_disparity_residual`, inventory `74` files / `165416` bytes입니다. 런타임 입력은 여전히 image `left_u_px/left_v_px`, `measured_disparity_px`, stereo-center RDF이며, PX4/NED/RTK truth는 calibration/evaluation evidence에만 남습니다.
- 2026-06-18: EXP-236/237에서 residual profile 일반화 범위를 확인했습니다. 새 도구 `Simulation/scripts/run_runtime_pooled_disparity_residual_profile.py`는 여러 scenario/seed의 train split을 합쳐 하나의 `DisparityResidualRuntimeProfile`을 fit하고 validation split으로 평가합니다. EXP-236의 4-scenario 단일 pooled profile은 `60/80` PASS로 실패했습니다. best는 target worst를 `5.018522m -> 4.691842m`로 줄였지만, `intrinsic_scale_stress`에서 raw 대비 range 개선 조건을 모두 실패했고 `disparity_noise_stress` 평균 range도 `1.997943m -> 2.097912m`로 악화되었습니다. 따라서 하나의 global residual polynomial을 모든 stress에 무조건 적용하는 방식은 rejected evidence로 둡니다. 반면 EXP-237 scenario별 pooled profile catalog는 4개 scenario 각각 `20/20 STRICT_PASS`입니다. 대표 성능은 bbox range mean `0.563071m -> 0.524207m`, combined target max `2.476891m -> 1.865128m`, disparity target max `5.018522m -> 1.621288m`, intrinsic target max `4.309140m -> 0.953251m`입니다. scenario-pooled disparity profile은 실제 `StereoDistanceNode` smoke도 `applied:EXP-20260618-237-disparity-scenario-residual-profile_pooled_disparity_residual`로 PASS했습니다. 이 branch의 결론은 “unconditional single residual”이 아니라, 배포 label/운용 mode/관측 stress classifier가 있을 때 선택 가능한 residual profile catalog입니다. runtime selection은 future work이며 PX4/NED/RTK truth 또는 validation metric을 runtime selector 입력으로 쓰면 안 됩니다.
- 2026-06-18: EXP-238에서 EXP-237 scenario residual profile들을 label-selectable catalog로 묶고 operator 실행 경로에 연결했습니다. 새 도구는 `Simulation/scripts/export_disparity_residual_profile_catalog.py`와 `Simulation/scripts/select_disparity_residual_profile_by_label.py`입니다. catalog는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_disparity_residual_probe/scenario_residual_catalog/`에 저장했으며, `bbox_outlier_stress`, `combined_sensor_stress`, `disparity_noise_stress`, `intrinsic_scale_stress` label이 각각 package-local residual JSON으로 resolve됩니다. selector 입력은 `deployment_label`, `scenario_label`, `scenario_name`뿐이고, forbidden runtime fields에는 PX4/RTK truth, validation error, target-window label, true disparity, true image feature가 명시됩니다. `run_operator_view.sh`와 `run_operator_view_with_calibration_profile.sh`는 이제 `--disparity-residual-profile`/`--disparity-residual-profile-json` 및 `--disparity-residual-selector-json`을 통해 `enable_disparity_residual_profile`과 `disparity_residual_profile_path`를 launch/node parameter로 전달합니다. `check_stereodistance_launch_profile_contract.py`와 `check_stereodistance_operator_view_launch.py`도 이 경로를 검증합니다. 저장된 operator evidence `.../scenario_residual_catalog/operator_view_launch_check_residual_selector.json`은 `PASS`이고 command에 observation fusion manifest, base runtime profile, resolved disparity residual profile이 모두 포함됐습니다. 예시는 다음과 같습니다.

```bash
./run_operator_view_with_calibration_profile.sh \
  --profile-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/common_csv_pooled_profile_matrix_all248_no_active/deployable_profile_policy_package_suite/range_priority/scenario_loso_selector_package_rmse/packages/run_bbox_quantization_stress_seed_20260618/active_runtime_profile.json \
  --observation-fusion-manifest Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_observation_fusion_followup/fresh_deployment_package/observation_fusion_runtime_manifest.json \
  --disparity-residual-selector-json Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_disparity_residual_probe/scenario_residual_catalog/disparity_residual_profile_label_selector.json \
  --deployment-label disparity_noise_stress \
  -- --no-px4
```

이 명령의 residual selector는 명시 label만 사용합니다. 실제 자동 selector를 만들기 전까지는 운영자가 deployment label을 고르거나, 별도 truth-free 관측 stress classifier가 label을 제공해야 합니다.
- 2026-06-18: EXP-239에서 scenario residual catalog의 cross-scenario transfer matrix를 만들었습니다. 새 도구는 `Simulation/scripts/evaluate_disparity_residual_catalog_transfer.py`이며, 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_disparity_residual_probe/scenario_residual_catalog/transfer_matrix/`에 저장했습니다. label-matched profile은 4/4 scenario에서 `STRICT_PASS`입니다. 하지만 wrong-profile transfer는 위험합니다. bbox profile은 combined/disparity에서도 gate는 통과하지만 range 개선이 약하고, intrinsic scenario에서는 `0/20` 실패입니다. disparity profile은 bbox/combined에서 gate는 통과해도 range를 크게 악화시키고, intrinsic에서는 `0/20` 실패입니다. intrinsic profile은 bbox에서 `17/20`, disparity에서 `5/20`만 통과합니다. 결론은 residual catalog를 자동 적용하려면 stress classifier 또는 명시 deployment label의 신뢰도가 필요하다는 것입니다. selector가 불확실할 때는 residual profile을 끄거나 conservative default를 쓰는 guard가 필요합니다. 이 transfer 평가에서도 truth는 offline metric 전용이고, residual apply는 measured disparity와 corrected stereo-center RDF만 사용했습니다.
- 2026-06-18: EXP-240에서 residual catalog 자동 적용을 위한 truth-free feature-domain guard를 시험했습니다. 새 도구는 `Simulation/scripts/evaluate_disparity_residual_feature_guard.py`이며, 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_disparity_residual_probe/scenario_residual_catalog/feature_guard/`에 저장했습니다. guard decision은 measured disparity feature z-score p95와 residual correction magnitude p95만 사용하고, PX4/RTK/NED truth는 offline metric에만 사용했습니다. 결과는 `FAIL`입니다. viable guard는 `0/120`이고, 가장 보수적인 bbox row는 wrong-profile 적용을 `0/60`으로 막았지만 residual 적용도 `0`회라서 단순 disable fallback입니다. combined/disparity profile의 best row는 label-matched `20/20`을 유지했지만 wrong-profile에도 `33/60`회 적용됐고, intrinsic profile은 wrong-profile `60/60`회 적용됐습니다. 따라서 현재 두 feature만으로는 EXP-238 explicit label selector를 대체할 수 없습니다. 다음 자동 탐색은 image-quality, detector/tracker confidence, epipolar consistency, temporal innovation, calibration metadata처럼 truth-free이면서 scenario를 더 잘 분리하는 feature를 추가해야 합니다.
- 2026-06-18: EXP-241에서 단일 threshold guard 대신 truth-free multi-feature nearest-centroid selector를 시험했습니다. 새 도구는 `Simulation/scripts/evaluate_disparity_residual_feature_selector.py`이며, 결과는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_disparity_residual_probe/scenario_residual_catalog/feature_selector/`에 저장했습니다. selector feature는 image-space `left_u_px/left_v_px`, measured/fused disparity 통계, raw/observation stereo-center RDF 통계뿐이며, PX4/RTK/NED truth, validation error, target-window label, true disparity, true image feature는 사용하지 않습니다. 4개 stress scenario에서 seed 앞 10개/scenario로 centroid를 만들고 뒤 10개/scenario를 selector validation으로 평가한 결과 `STRICT_PASS`입니다. scenario label match `40/40`, selected residual pass `40/40`, selected range RMSE mean `0.439528m`, 1852m target max abs worst `3.186540m`입니다. Per-scenario heldout target worst는 bbox `3.186540m`, combined `1.713508m`, disparity `1.489892m`, intrinsic `0.819831m`입니다. 이 결과는 explicit label selector를 자동화할 가능성이 있지만 아직 synthetic feature-window 증거입니다. 다음 단계는 selector model export, operator/node runtime wiring, 그리고 실제 CSV/log에서 같은 truth-free feature separation이 유지되는지 검증하는 것입니다.
- 2026-06-18: EXP-242에서 EXP-241 selector model을 실제 residual catalog profile path로 resolve하는 smoke를 추가했습니다. 새 도구는 `Simulation/scripts/select_disparity_residual_profile_by_features.py`입니다. 입력은 saved selector model JSON, residual catalog JSON, feature JSON 또는 CSV row이며, forbidden selector/runtime fields에 truth/PX4/RTK/NED/validation/replay token이 있으면 FAIL합니다. Smoke 출력은 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_disparity_residual_probe/scenario_residual_catalog/feature_selector/feature_selector_profile_resolution_smoke.json|txt`입니다. held-out feature CSV row `10`에서 `bbox_outlier_stress`를 선택했고, `scenario_residual_catalog/profiles/bbox_outlier_stress.json`으로 resolve했습니다. 이 smoke는 자동 selector 배포를 위한 software-path evidence이며, 아직 live `StereoDistanceNode` 내부 feature extractor/selector wiring은 아닙니다.
- 2026-06-18: EXP-243에서 feature selector를 operator wrapper print-only 경로에 연결했습니다. `run_operator_view_with_calibration_profile.sh`는 이제 `--disparity-residual-feature-selector-json`, `--disparity-residual-feature-catalog-json`, `--disparity-residual-feature-csv`/`--disparity-residual-feature-json`, `--disparity-residual-feature-row-index`를 받을 수 있습니다. wrapper는 feature selector로 residual profile을 resolve한 뒤 기존 `run_operator_view.sh --disparity-residual-profile` 인자로 넘깁니다. `Simulation/scripts/check_stereodistance_operator_view_launch.py`도 같은 경로를 검증합니다. 저장된 evidence는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_disparity_residual_probe/scenario_residual_catalog/feature_selector/operator_view_launch_check_feature_selector.json|txt`이고 `PASS`입니다. 이 smoke는 held-out feature CSV row `10`에서 `bbox_outlier_stress` residual을 선택하고, command에 base runtime profile, observation-fusion manifest, resolved residual profile, `--no-px4`, `yolo_target_hz:=0.0`이 모두 들어가는 것을 확인했습니다. 아직 live node 내부에서 feature summary를 직접 생성하는 단계는 아니며, 실제 로그/CSV에서 만든 truth-free feature row를 operator 실행에 넣는 중간 배포 경로입니다.
- 2026-06-18: EXP-244에서 runtime observation/RDF CSV에서 feature selector 입력 row를 만드는 파일 기반 경로를 추가했습니다. 새 도구는 `Simulation/scripts/export_disparity_residual_feature_row_from_observations.py`입니다. 필수 입력 컬럼은 `left_u_px`, `left_v_px`, `measured_disparity_px`, `raw_right_m`, `raw_down_m`, `raw_forward_m`이고, 선택 컬럼 `fused_disparity_px`, `observation_right_m`, `observation_down_m`, `observation_forward_m`이 있으면 함께 사용합니다. truth/PX4/RTK/NED/validation/replay 컬럼은 거부합니다. Smoke 입력은 `.../feature_selector/runtime_observation_window_smoke_input.csv`이고, exporter 출력은 `.../feature_selector/runtime_observation_window_feature_export/disparity_residual_selector_feature_row.json|csv`입니다. 이 feature JSON을 EXP-241 selector에 넣으면 `disparity_noise_stress` residual profile로 resolve됐고, operator-view check `.../runtime_observation_window_feature_export/operator_view_launch_check_feature_json.json|txt`도 `PASS`입니다. 즉 현재 파일 기반 경로는 `runtime observation CSV -> feature row JSON/CSV -> residual profile selection -> operator command`까지 이어집니다. live `StereoDistanceNode` 내부에서 이 feature row를 직접 계산하는 것은 아직 다음 단계입니다.
- 2026-06-18: EXP-245에서 feature selector summary 계산을 공용 패키지 모듈로 분리했습니다. 새 모듈은 `Simulation/stereo_ground_sim/stereo_ground_sim/disparity_residual_feature_summary.py`이고, `feature_summary_from_arrays`와 `feature_summary_from_runtime_rows`를 제공합니다. `export_disparity_residual_feature_row_from_observations.py`와 `evaluate_disparity_residual_feature_selector.py`는 이제 같은 함수로 image/disparity/RDF 통계를 계산합니다. 새 테스트는 RDF axis order `right,down,forward`, raw RDF fallback, truth-free metadata를 확인합니다. 리팩터링 뒤 EXP-244 runtime observation CSV export와 operator-view feature-selector smoke를 다시 돌렸고 둘 다 `PASS`입니다. 이 단계의 목적은 live node 내부 feature extractor를 만들 때 동일 계산식을 재사용할 수 있게 하는 것입니다.
- 2026-06-18: EXP-246에서 `StereoDistanceNode` 내부에서 같은 feature summary를 생성하도록 최소 통합했습니다. `_calculate_metrics`는 이제 `disparity_residual_feature_summary_status`와 `disparity_residual_feature_summary`를 metrics/profile status에 넣습니다. base correction이 있으면 corrected stereo-center RDF를 observation RDF로 쓰고, base correction이 없으면 raw RDF fallback을 사용합니다. `Simulation/scripts/check_stereodistance_observation_fusion_node_smoke.py`는 feature summary가 `ready:current_sample`인지, `raw_forward_m_mean`이 있는지, profile status에도 같은 상태가 들어가는지 검사합니다. 저장된 smoke는 `Simulation/results/rtk_stereo_runtime_observation_stress_fscale_grid_matched360/multiburst_disparity_residual_probe/scenario_residual_catalog/feature_selector/node_smoke_feature_summary_residual.json|txt`이고 `PASS`입니다. 이 단계는 아직 node 내부 자동 residual selection이 아니라, live node가 selector와 동일한 truth-free feature schema를 만들 수 있음을 검증한 것입니다.
- 2026-06-18: EXP-247에서 `StereoDistanceNode` 내부 자동 residual selection까지 연결했습니다. 새 모듈 `stereo_ground_sim.disparity_residual_feature_selector`는 EXP-241 nearest-centroid selector model과 EXP-238 residual catalog를 읽고, 현재 sample의 image/disparity/RDF feature summary만으로 residual profile을 선택합니다. launch argument는 `enable_disparity_residual_feature_selector`, `disparity_residual_feature_selector_model_path`, `disparity_residual_feature_selector_catalog_path`이며, `run_operator_view.sh`에서는 `--disparity-residual-feature-selector`와 `--disparity-residual-feature-catalog`로 전달합니다. `run_operator_view_with_calibration_profile.sh`에는 파일 기반 pre-resolve selector와 구분되는 `--node-disparity-residual-feature-selector-json`, `--node-disparity-residual-feature-catalog-json` 옵션을 추가했습니다. explicit residual profile이 있으면 explicit profile이 우선이고, node-side selector는 explicit residual source와 함께 쓰지 않도록 wrapper에서 막습니다. Node smoke `.../feature_selector/node_smoke_feature_selector_auto_residual.json|txt`는 `selected:disparity_noise_stress`, residual `applied:EXP-20260618-237-disparity-scenario-residual-profile_pooled_disparity_residual`, observation fusion `fused:MultiBurstObservationFusion:21`로 `PASS`입니다. Operator dry-run evidence `.../feature_selector/operator_view_launch_check_node_feature_selector.json|txt`도 base runtime profile, observation-fusion manifest, node selector model/catalog를 command에 포함해 `PASS`입니다. 목적은 runtime truth 없이 StereoDistance가 관측 feature로 residual catalog를 자동 선택하는 것이며, 입력은 image `left_u_px/left_v_px`, measured/fused disparity, raw/corrected stereo-center RDF summary뿐입니다. PX4/NED/RTK truth, validation error, target-window label, true disparity/image feature는 selector runtime 입력이 아닙니다. 출력 계약은 내부 stereo-center RDF(right,down,forward), public topic FRD+range입니다.
- 2026-06-18: EXP-248에서 EXP-247의 current-sample selector를 rolling runtime window selector로 확장했습니다. `StereoDistanceNode`는 이제 `disparity_residual_feature_selector_window_size`와 `disparity_residual_feature_selector_min_samples` launch parameter를 받아 최근 image/disparity/raw RDF/base-corrected RDF 관측 row를 누적하고, window summary가 준비된 뒤 residual profile을 선택합니다. 기본값은 둘 다 `1`이라 기존 current-sample 동작과 호환됩니다. `run_operator_view.sh`는 `--disparity-residual-feature-window-size`, `--disparity-residual-feature-min-samples`를 지원하고, calibration-profile wrapper는 `--node-disparity-residual-feature-window-size`, `--node-disparity-residual-feature-min-samples`를 지원합니다. 새 node smoke `.../feature_selector/node_smoke_feature_selector_windowed_auto_residual.json|txt`는 window size `21`, min samples `7`에서 selector summary `ready:runtime_window:21`, selector `selected:disparity_noise_stress`, residual `applied:EXP-20260618-237-disparity-scenario-residual-profile_pooled_disparity_residual`, observation fusion `fused:MultiBurstObservationFusion:21`로 `PASS`입니다. Operator dry-run `.../feature_selector/operator_view_launch_check_node_feature_selector_windowed.json|txt`도 window args가 command에 들어가는지 확인해 `PASS`입니다. 이 단계는 EXP-241의 window-stat selector 가정에 더 가깝게 node runtime 경로를 맞춘 것이며, 아직 recorded calibration CSV/live camera 로그에서 selector separation을 검증한 것은 아닙니다.
- 2026-06-18: EXP-249에서 EXP-241에 저장된 feature-window validation rows를 `StereoDistanceNode`가 사용하는 동일 selector class로 replay했습니다. 새 도구는 `Simulation/scripts/check_disparity_residual_feature_selector_replay.py`이며, selector model JSON, residual catalog JSON, feature CSV를 받아 지정 split의 feature rows를 `stereo_ground_sim.disparity_residual_feature_selector.DisparityResidualFeatureSelector`로 다시 선택합니다. forbidden runtime column은 truth/PX4/RTK/NED/validation/replay 계열을 거부하되 `runtime_truth_used=false` 메타데이터만 허용합니다. 저장된 evidence `.../feature_selector/node_selector_feature_window_replay.json|csv|txt`는 EXP-241 validation split `40` rows에 대해 `PASS`, label match `40/40`, selected profile exists `40/40`, runtime truth count `0`, distance margin min `0.576134`입니다. 이 결과는 offline evaluator와 node runtime selector class가 같은 saved feature-window evidence에서 같은 residual label을 고른다는 software-path 검증입니다. 여전히 다음 단계는 recorded/live observation windows에서 feature 분리가 유지되는지 확인하는 것입니다.
- 2026-06-18: EXP-250에서 runtime observation CSV를 rolling window로 잘라 selector replay까지 수행하는 recorded/live 로그용 검증 경로를 추가했습니다. 새 도구는 `Simulation/scripts/check_disparity_residual_selector_on_observation_windows.py`입니다. 입력 CSV 필수 컬럼은 image `left_u_px`, `left_v_px`, `measured_disparity_px`와 raw stereo-center RDF `raw_right_m`, `raw_down_m`, `raw_forward_m`이며, 선택 컬럼 `fused_disparity_px`, `observation_right_m`, `observation_down_m`, `observation_forward_m`이 있으면 사용합니다. truth/PX4/RTK/NED/validation/replay 컬럼은 거부합니다. 도구는 `--window-size`, `--stride`, `--min-samples`로 rolling windows를 만들고, 각 window feature summary를 EXP-241 selector model과 EXP-238 residual catalog에 replay합니다. Smoke evidence `.../feature_selector/runtime_observation_window_selector_replay/observation_window_selector_replay_summary.json|txt`는 기존 `runtime_observation_window_smoke_input.csv` 5 rows를 window size `5`로 처리해 `PASS`, selected label `disparity_noise_stress`, label match `1/1`, profile exists `1/1`, runtime truth `false`, distance margin `0.920563`입니다. 이 단계는 실제 recorded/live observation CSV가 들어왔을 때 selector 안정성을 같은 계약으로 바로 검증할 수 있는 경로입니다.
- 2026-06-18: EXP-251에서 operator recorder의 `telemetry.csv`를 selector observation CSV로 바꾸는 bridge를 추가했습니다. 새 도구는 `Simulation/scripts/export_selector_observation_csv_from_operator_telemetry.py`입니다. operator telemetry source는 public FRD(`forward,right,down,range`) 필드와 ground-truth 비교 컬럼을 함께 담기 때문에 selector에 직접 넣지 않습니다. bridge는 `left_cx/left_cy`, `raw_disparity_px`, public FRD raw/corrected fields를 읽어 selector용 명시 RDF 컬럼 `raw_right_m/raw_down_m/raw_forward_m`, `observation_right_m/observation_down_m/observation_forward_m`으로 내보내고, `drone_*`, `ground_truth`, `actual`, PX4/RTK/NED 계열 컬럼은 drop합니다. Smoke evidence `.../feature_selector/operator_telemetry_selector_bridge_smoke/operator_telemetry_to_selector_observation_summary.json|txt`는 input rows `5`, output rows `5`, dropped truth columns `4`, runtime truth `false`로 `PASS`입니다. 그 출력 CSV를 EXP-250 checker에 넣은 `.../operator_telemetry_selector_bridge_smoke/window_selector_replay/observation_window_selector_replay_summary.json|txt`도 selected label `disparity_noise_stress`, label match `1/1`, profile exists `1/1`, distance margin `0.925397`로 `PASS`입니다. 이로써 `run_operator_view.sh --record-lite`로 남긴 telemetry를 truth-free selector replay로 넘기는 소프트웨어 경로가 생겼습니다.
- 2026-06-18: EXP-252에서 EXP-251 bridge와 EXP-250 rolling-window replay를 한 명령으로 묶는 operator telemetry selector replay wrapper를 추가했습니다. 새 도구는 `Simulation/scripts/run_operator_telemetry_selector_replay.py`입니다. 입력은 `run_operator_view.sh --record-lite`류의 `telemetry.csv`, EXP-241 selector model JSON, EXP-238 residual catalog JSON이며, 출력은 `selector_observations.csv`, bridge summary, window replay CSV/JSON/TXT, 통합 summary입니다. 이 wrapper도 runtime truth를 쓰지 않고, 입력 telemetry의 public FRD(`forward,right,down,range`) 필드를 selector용 stereo-center RDF(`right,down,forward`) 컬럼으로 명시 변환한 뒤 truth/PX4/RTK/NED 컬럼을 drop합니다. Smoke evidence `.../feature_selector/operator_telemetry_selector_replay_smoke/operator_telemetry_selector_replay_summary.json|txt`는 input rows `5`, dropped truth columns `4`, window count `1`, selected label `disparity_noise_stress`, label match `1/1`, profile exists `1/1`, distance margin min `0.925397`, runtime truth `false`로 `PASS`입니다. 실제 녹화 telemetry가 생기면 같은 명령에 `--input-telemetry-csv`만 교체해서 selector가 어떤 residual profile을 고르는지 즉시 검사할 수 있습니다.
- 2026-06-18: EXP-253에서 operator/observation rolling-window selector replay summary에 label 안정성 지표를 추가했습니다. `Simulation/scripts/check_disparity_residual_selector_on_observation_windows.py`와 `Simulation/scripts/run_operator_telemetry_selector_replay.py`는 이제 `selected_label_counts`, `dominant_selected_label`, `dominant_selected_label_count`, `selector_label_stable`을 JSON/TXT summary에 기록합니다. 긴 telemetry에서 window마다 residual profile 선택이 흔들리는지 확인하기 위한 지표이며, selector 입력 계약은 변하지 않습니다. EXP-252 smoke를 재실행한 결과 `selected_label_counts={"disparity_noise_stress": 1}`, dominant label `disparity_noise_stress`, `selector_label_stable=true`, runtime truth `false`로 `PASS`입니다.
- 2026-06-18: EXP-254에서 operator/observation rolling-window selector replay에 선택형 runtime-safe gate를 추가했습니다. `Simulation/scripts/check_disparity_residual_selector_on_observation_windows.py`와 `Simulation/scripts/run_operator_telemetry_selector_replay.py`는 이제 `--require-stable-label`과 `--min-distance-margin`을 지원합니다. 이 gate는 PX4/RTK/NED truth나 validation error를 보지 않고, selector가 window들에서 같은 residual label을 고르는지와 nearest-centroid distance margin이 지정 threshold 이상인지로만 PASS/FAIL을 정합니다. Gated smoke evidence `.../feature_selector/operator_telemetry_selector_replay_gated_smoke/operator_telemetry_selector_replay_summary.json|txt`는 `--require-stable-label --min-distance-margin 0.5` 조건에서 selected label `disparity_noise_stress`, label match `1/1`, profile exists `1/1`, selector label stable `true`, distance margin min `0.925397`, runtime truth `false`로 `PASS`입니다. 실제 긴 telemetry에서는 이 gate가 residual profile 적용 전 선택 churn이나 낮은 confidence를 fail로 드러내는 배포 전 검문 역할을 합니다.
- 2026-06-18: EXP-255에서 gated selector replay 결과를 실제 operator 적용 결정 artifact로 내보내도록 확장했습니다. `Simulation/scripts/run_operator_telemetry_selector_replay.py`는 이제 통합 summary와 별도로 `operator_telemetry_selector_decision.json|txt`를 씁니다. 이 artifact는 replay/export/gate가 모두 PASS이고 runtime truth가 쓰이지 않았으며 선택 profile path가 존재할 때만 `apply_residual_allowed=true`를 기록하고, 그때만 `recommended_residual_profile`을 채웁니다. Gate 실패나 export 실패에서는 recommended profile을 빈 문자열로 남겨 downstream이 residual을 적용하지 못하게 합니다. Gated smoke evidence `.../feature_selector/operator_telemetry_selector_replay_gated_smoke/operator_telemetry_selector_decision.json|txt`는 `apply_residual_allowed=true`, recommended label `disparity_noise_stress`, recommended profile `scenario_residual_catalog/profiles/disparity_noise_stress.json`, selector label stable `true`, distance margin min `0.925397`, runtime truth `false`로 `PASS`입니다.
- 2026-06-18: EXP-256에서 EXP-255 decision artifact를 operator launch dry-run 경로에 연결했습니다. `Simulation/run_operator_view_with_calibration_profile.sh`와 `Simulation/scripts/check_stereodistance_operator_view_launch.py`는 이제 `--disparity-residual-decision-json`을 지원합니다. Wrapper는 decision JSON의 `apply_residual_allowed=true`를 확인한 뒤에만 `recommended_residual_profile`을 `run_operator_view.sh --disparity-residual-profile`로 넘깁니다. Decision이 FAIL이거나 apply가 허용되지 않으면 wrapper는 에러로 종료하고 residual profile을 적용하지 않습니다. Smoke evidence `.../feature_selector/operator_telemetry_selector_replay_gated_smoke/operator_view_launch_check_residual_decision.json|txt`는 EXP-255 decision artifact를 입력으로 받아 residual decision label `disparity_noise_stress`, resolved residual profile `scenario_residual_catalog/profiles/disparity_noise_stress.json`, observation-fusion manifest, base runtime profile이 command에 모두 들어가는 것을 `PASS`로 확인했습니다. 이 경로도 runtime truth를 쓰지 않고, public output 계약은 FRD+range로 유지됩니다.
- 2026-06-18: EXP-257에서 실제 녹화 telemetry 적용 전용 end-to-end preflight를 추가했습니다. 새 도구는 `Simulation/scripts/run_operator_telemetry_residual_preflight.py`입니다. 이 스크립트는 `telemetry.csv -> truth-drop selector replay -> gated decision artifact -> operator launch dry-run`을 한 번에 수행합니다. Selector decision이 PASS이고 `apply_residual_allowed=true`일 때만 `check_stereodistance_operator_view_launch.py --disparity-residual-decision-json`을 호출하며, gate가 실패하면 operator launch check를 만들지 않고 preflight를 FAIL로 끝냅니다. Smoke evidence `.../feature_selector/operator_telemetry_residual_preflight_smoke/operator_telemetry_residual_preflight_summary.json|txt`는 selected label `disparity_noise_stress`, apply residual allowed `true`, resolved residual profile `scenario_residual_catalog/profiles/disparity_noise_stress.json`, operator launch status `PASS`, runtime truth `false`로 `PASS`입니다. 이 경로가 실제 `run_operator_view.sh --record-lite` telemetry를 residual 적용 전에 검사하는 표준 preflight가 됩니다.
- 2026-06-18: EXP-258에서 EXP-257 preflight 산출물용 readiness checker를 추가했습니다. 새 도구는 `Simulation/scripts/check_operator_telemetry_residual_preflight_readiness.py`입니다. 이 checker는 preflight summary, selector decision JSON, operator launch check JSON이 모두 존재하는지, preflight/decision/launch status가 모두 `PASS`인지, `apply_residual_allowed=true`인지, recommended residual profile과 launch에서 resolved residual profile이 같은지, `--disparity-residual-profile` 인자가 실제 command에 들어갔는지, runtime truth가 쓰이지 않았는지, public FRD -> stereo-center RDF -> public FRD+range 계약이 기록됐는지를 검사합니다. Smoke evidence `.../feature_selector/operator_telemetry_residual_preflight_smoke/operator_telemetry_residual_preflight_readiness.json|txt`는 selected label `disparity_noise_stress`, residual arg present `true`, decision apply allowed `true`, runtime truth `false`로 `PASS`입니다. 이로써 preflight output 자체도 배포 전 artifact gate를 통과해야 live residual enablement 후보가 됩니다.
- 2026-06-18: EXP-259에서 `Simulation/scripts/run_operator_telemetry_residual_preflight.py`에 최신 telemetry 자동 선택 옵션을 추가했습니다. `--latest-telemetry-root DIR`와 `--latest-telemetry-pattern '**/telemetry.csv'`를 사용하면 지정 root 아래에서 mtime이 가장 최신인 `telemetry.csv`를 골라 preflight에 넣습니다. Probe evidence `.../feature_selector/operator_latest_telemetry_residual_preflight_probe/operator_telemetry_residual_preflight_summary.json|txt`는 최신 파일 `Simulation/bags/operator_20260529_221251_lite/telemetry.csv`를 선택했지만 `FAIL`입니다. 이유는 이 과거 recorder CSV가 현재 residual preflight가 요구하는 public FRD raw/corrected fields(`raw_forward_m/raw_right_m/raw_down_m`, `corrected_forward_m/corrected_right_m/corrected_down_m`)를 갖지 않아 selector observation export가 valid row를 만들 수 없었기 때문입니다. 따라서 residual apply는 허용되지 않았고 operator launch check도 생성되지 않았습니다. 이는 보정 실패가 아니라 schema gate가 live residual enablement를 막은 정상 동작이며, 다음 실제 녹화는 EXP-251 이후 telemetry schema로 생성해야 합니다.
- 2026-06-18: EXP-260에서 실제 telemetry residual preflight 앞단에 schema audit gate를 추가했습니다. 새 도구는 `Simulation/scripts/audit_operator_telemetry_residual_preflight_schema.py`입니다. 이 audit는 operator telemetry가 public FRD raw fields(`raw_forward_m/raw_right_m/raw_down_m` 또는 `estimated_*` alias), corrected FRD fields(`corrected_forward_m/corrected_right_m/corrected_down_m`), image center, measured disparity를 갖는지 먼저 검사하고, valid row가 `--min-samples`보다 적으면 selector replay를 시작하지 않습니다. `run_operator_telemetry_residual_preflight.py`는 이제 `operator_telemetry_schema_audit.json|txt`를 쓰고, audit 실패 시 `operator_telemetry_selector_decision.json|txt`를 `SKIPPED`, `apply_residual_allowed=false`로 덮어 stale decision이 downstream에 남지 않게 합니다. 같은 latest telemetry probe를 재실행한 evidence는 `.../feature_selector/operator_latest_telemetry_residual_preflight_probe/operator_telemetry_schema_audit.json|txt` 및 preflight summary입니다. 결과는 여전히 `FAIL`이지만 실패 지점이 selector/export row 실패가 아니라 schema audit로 앞당겨졌습니다: input rows `9384`, valid rows `0`, missing groups `raw_forward_m/raw_right_m/raw_down_m` 및 `observation_forward_m/observation_right_m/observation_down_m`, runtime truth `false`, residual apply `false`, launch `SKIPPED`. 좌표 계약은 source public FRD -> selector stereo-center RDF -> public FRD+range입니다.
- 2026-06-18: EXP-261에서 다음 실제 operator telemetry가 EXP-260 schema audit를 통과할 수 있도록 recorder schema를 고정했습니다. `Simulation/stereo_ground_sim/stereo_ground_sim/operator_recorder.py`는 이제 CSV `axis_contract`를 `stereodistance_public_frd:forward,right,down,range`로 기록하고, manifest에 raw/estimated/corrected public FRD fields를 명시합니다. `json` import 누락도 보완해 `/stereo/profile_status` JSON metadata(`profile_id`, `profile_mode`, `fallback_level`)가 CSV에 정상 기록됩니다. 새 테스트 `Simulation/tests/test_operator_recorder_schema.py`는 recorder callback output이 `raw_forward_m/raw_right_m/raw_down_m`, `estimated_forward_m/estimated_right_m/estimated_down_m`, `corrected_forward_m/corrected_right_m/corrected_down_m`, `raw_disparity_px`, image center를 포함하고, 그 telemetry row가 `audit_operator_telemetry_residual_preflight_schema.py`에서 `PASS`하는지 확인합니다. 즉 기존 bags의 구 schema는 여전히 FAIL이지만, 다음 `run_operator_view.sh --record-lite` 녹화는 residual preflight가 요구하는 public FRD raw/corrected 계약을 만족해야 합니다. selector 내부 변환은 여전히 stereo-center RDF(`right,down,forward`)이고, runtime truth는 사용하지 않습니다.
- 2026-06-18: EXP-262에서 current recorder schema telemetry가 full residual preflight와 readiness를 통과하는지 단일 Python smoke로 검증했습니다. 새 도구는 `Simulation/scripts/run_operator_current_schema_preflight_smoke.py`입니다. 이 스크립트는 `stereodistance_public_frd:forward,right,down,range` telemetry CSV를 생성하고, `run_operator_telemetry_residual_preflight.py`와 `check_operator_telemetry_residual_preflight_readiness.py`를 연속 실행합니다. Evidence는 `.../feature_selector/operator_current_schema_preflight_smoke/`에 저장했습니다. 결과는 `PASS`: schema audit valid rows `5/5`, selected label `disparity_noise_stress`, distance margin min `0.9253968578`, `apply_residual_allowed=true`, operator launch dry-run `PASS`, readiness `PASS`, runtime truth `false`입니다. 생성된 source CSV는 public FRD+range이고, selector observation CSV는 explicit stereo-center RDF(`right,down,forward`)입니다. 따라서 실제 old bags는 아직 FAIL이지만, EXP-261 이후 형식으로 새로 녹화한 telemetry가 들어오면 residual preflight가 끝까지 통과할 수 있는 software path가 검증됐습니다.
- 2026-06-18: EXP-263에서 실제 최신 `Simulation/bags` telemetry를 위한 live gate wrapper를 추가했습니다. 새 도구는 `Simulation/scripts/run_operator_latest_telemetry_residual_gate.py`입니다. 이 wrapper는 최신 `telemetry.csv`를 선택해 residual preflight를 실행하고, preflight가 `PASS`이고 `apply_residual_allowed=true`일 때만 readiness checker를 실행합니다. 같은 old latest telemetry로 실행한 evidence는 `.../feature_selector/operator_latest_telemetry_residual_gate_probe/operator_latest_telemetry_residual_gate_summary.json|txt`입니다. 현재 결과는 `FAIL`: 선택된 파일은 `Simulation/bags/operator_20260529_221251_lite/telemetry.csv`, preflight `FAIL`, readiness `SKIPPED`, residual apply `false`, operator launch `SKIPPED`, runtime truth `false`입니다. 실패 원인은 EXP-260과 동일하게 old schema가 public FRD raw/corrected groups를 갖지 않는 것입니다. 새 로그를 `run_operator_view.sh --record-lite`로 녹화한 뒤 이 wrapper를 다시 실행하는 것이 실제 live residual enablement gate가 됩니다.
- 2026-06-18: EXP-264에서 실제 녹화 전 `run_operator_view.sh --record-lite` 경로가 current recorder schema를 쓸 준비가 되었는지 확인하는 pre-record readiness checker를 추가했습니다. 새 도구는 `Simulation/scripts/check_operator_recording_preflight_readiness.py`입니다. 이 checker는 `run_operator_view.sh`가 `--record-lite/full/debug` 옵션과 `stereo_ground_sim.operator_recorder` 실행 경로를 갖는지, `setup.py`에 `operator_recorder = stereo_ground_sim.operator_recorder:main` entry point가 있는지, recorder schema가 `stereodistance_public_frd:forward,right,down,range`와 raw/estimated/corrected public FRD fields 및 image/disparity fields를 포함하는지 검사합니다. Evidence `.../feature_selector/operator_recording_preflight_readiness/operator_recording_preflight_readiness.json|txt`는 `PASS`이며 ambiguous x/y/z recorder fields `0`, runtime truth `false`입니다. 추천 녹화 명령 템플릿은 `cd Simulation && ./run_operator_view.sh --record-lite bags/operator_$(date +%Y%m%d_%H%M%S)_lite --bbox-adjustment <active_runtime_profile.json>`입니다.
- 2026-06-18: EXP-265에서 residual live enablement 전 과정을 한 명령으로 묶는 suite를 추가했습니다. 새 도구는 `Simulation/scripts/run_operator_residual_enablement_suite.py`입니다. suite는 (1) EXP-264 source recording readiness, (2) installed ROS overlay recorder readiness, (3) installed recorder CSV row smoke, (4) EXP-262 current-schema software smoke, (5) EXP-263 latest real-telemetry live gate를 순서대로 실행합니다. Evidence `.../feature_selector/operator_residual_enablement_suite/operator_residual_enablement_suite_summary.json|txt`의 현재 결과는 overall `FAIL`입니다. 하지만 세부 결과는 recording readiness `PASS`, install overlay readiness `PASS`, installed CSV smoke `PASS`, current-schema smoke `PASS`, latest gate `FAIL`입니다. 즉 소스와 실제 ROS install overlay의 recorder schema 및 CSV row output, software path는 public FRD+range -> stereo-center RDF selector -> residual decision -> operator launch dry-run까지 준비됐고, 남은 실패는 현재 최신 실제 bags telemetry가 old schema인 점으로 좁혀졌습니다. 선택된 latest 파일은 `Simulation/bags/operator_20260529_221251_lite/telemetry.csv`, residual apply `false`, operator launch `SKIPPED`, runtime truth `false`입니다.
- 2026-06-18: EXP-266에서 fresh `record-lite` 녹화와 EXP-265 suite 실행을 묶는 operational wrapper를 추가했습니다. 새 도구는 `Simulation/scripts/run_operator_record_lite_and_residual_suite.py`입니다. 이 wrapper는 `run_operator_view_with_calibration_profile.sh -- --record-lite DIR` 명령을 구성하고, 선택적으로 지정 시간 녹화를 실행한 뒤 최신 telemetry 기준으로 residual enablement suite를 실행합니다. `--print-only`는 실제 ROS/Gazebo/PX4를 실행하지 않고 record command와 suite args만 저장합니다. Evidence `.../feature_selector/operator_record_lite_and_residual_suite_print_only/operator_record_lite_and_residual_suite_summary.json|txt`는 `PASS`, `record_status=PRINT_ONLY`, `suite_status=PRINT_ONLY`, runtime truth `false`입니다. 실제 비행/시뮬레이션 녹화에서는 `--operator-arg=--no-px4` 같은 dry-run 인자를 제거하거나 상황에 맞게 조정한 뒤 실행해야 합니다.
- 2026-06-18: EXP-267에서 EXP-266 wrapper에 fresh telemetry guard를 추가했습니다. `run_operator_record_lite_and_residual_suite.py`는 이제 실행 후 `record_dir/telemetry.csv` 존재 여부와 EXP-265 suite가 선택한 `latest_input_telemetry_csv`가 그 파일인지 확인합니다. `--skip-record`는 기존 latest telemetry를 검사하는 명시 모드라 이 guard를 강제하지 않지만, 실제 record 모드에서 녹화가 실행되지 않았거나 `record_dir/telemetry.csv`가 없거나 suite가 다른 telemetry를 잡으면 FAIL입니다. Print-only evidence는 같은 `.../operator_record_lite_and_residual_suite_print_only/`에 갱신됐고 `record_telemetry_csv=Simulation/bags/operator_next_current_schema_lite/telemetry.csv`, `record_telemetry_exists=false`, `suite_used_record_telemetry=false`를 명시합니다. 이 값들은 print-only라 정보 필드이며, 실제 실행에서는 live residual enablement가 과거 old log를 재사용하는 것을 막는 gate가 됩니다.
- 2026-06-18: EXP-268에서 EXP-267 wrapper에 `--synthetic-record` 모드를 추가해 ROS/Gazebo/PX4 없이도 non-print record-lite residual enablement 경로를 검증했습니다. 이 모드는 `record_dir/telemetry.csv`에 current public FRD+range schema telemetry를 새로 쓰고, EXP-265 suite가 그 파일을 latest telemetry로 선택해야만 PASS합니다. Evidence `.../feature_selector/operator_record_lite_synthetic_enablement_suite/operator_record_lite_and_residual_suite_summary.json|txt`는 `PASS`: `record_status=SYNTHETIC`, `record_telemetry_exists=true`, `record_telemetry_fresh=true`, `suite_used_record_telemetry=true`, latest gate `PASS`, selected residual label `disparity_noise_stress`, runtime truth `false`입니다. 이것은 실제 비행/시뮬레이션 녹화 증거가 아니라 current-schema synthetic telemetry가 운영 wrapper의 freshness/schema/selector/readiness gate를 끝까지 통과한다는 software-path evidence입니다. 실제 Gazebo/PX4 실행에서는 `--synthetic-record`를 빼고 fresh `--record-lite` telemetry를 만들어 같은 wrapper를 다시 통과시켜야 합니다.
- 2026-06-18: EXP-269에서 실제 `run_operator_view.sh`가 사용하는 installed ROS overlay의 recorder schema를 검사하는 gate를 추가하고 overlay를 갱신했습니다. 새 도구 `Simulation/scripts/check_operator_recording_install_overlay_readiness.py`는 `/opt/ros/humble/setup.bash`와 `Simulation/install/setup.bash`를 source한 뒤 `stereo_ground_sim.operator_recorder`를 import하여 installed `AXIS_CONTRACT`와 raw/corrected public FRD fields, image/disparity fields를 확인합니다. Build 전 evidence `.../operator_recording_install_overlay_readiness_before_build/`는 `FAIL`이었고 installed axis contract가 `stereo_center_frd:forward_right_down`으로 stale였습니다. `colcon build --packages-select stereo_ground_sim` 후 evidence `.../operator_recording_install_overlay_readiness_after_build/`는 `PASS`, installed axis contract `stereodistance_public_frd:forward,right,down,range`입니다. EXP-265 suite에도 이 installed overlay gate를 포함했습니다. 이 단계는 source 파일만 고치고 실제 ROS 실행 경로가 stale로 남는 문제를 막기 위한 배포 전 gate입니다.
- 2026-06-18: EXP-270에서 installed ROS overlay의 `operator_recorder`가 실제 CSV row를 current schema로 쓰는지 확인하는 smoke를 추가했습니다. 새 도구 `Simulation/scripts/run_installed_operator_recorder_csv_smoke.py`는 `/opt/ros/humble/setup.bash`와 `Simulation/install/setup.bash`를 source한 subprocess에서 installed `stereo_ground_sim.operator_recorder`를 import하고, callback 경로로 `telemetry.csv` 한 row를 쓴 뒤 기존 schema audit에 통과시키는 방식입니다. Evidence `.../feature_selector/installed_operator_recorder_csv_smoke/installed_operator_recorder_csv_smoke_summary.json|txt`는 `PASS`: installed axis contract `stereodistance_public_frd:forward,right,down,range`, schema audit valid rows `1/1`, first row `raw_forward_m=1000`, `corrected_forward_m=999`, `raw_disparity_px=200`, runtime truth `false`입니다. EXP-265 suite에도 이 installed CSV smoke를 포함했습니다. 이는 실제 Gazebo/PX4 녹화 전, 설치된 recorder가 current public FRD+range telemetry를 residual preflight가 읽을 수 있는 형태로 쓰는지 검증하는 gate입니다.
- 2026-06-18: EXP-271에서 headless operator record-lite가 GUI/YOLO 없이도 current telemetry row를 만들 수 있도록 `StereoDistanceNode`에 `enable_synthetic_detections:=true` launch mode를 추가했습니다. 이 모드는 stereo-center RDF(`right,down,forward`) 위치를 synthetic left/right BBox center와 disparity로 역투영한 뒤 기존 `/stereo/detections`, `/stereo/estimated_position`, `/stereo/corrected_position` publish path를 그대로 사용합니다. 실제 측정 성능 증거가 아니라 headless/operator recording and residual gate 검증용입니다. Evidence `.../feature_selector/operator_record_lite_headless_synthetic_detection_intrinsic_split_label_probe/operator_record_lite_and_residual_suite_summary.json|txt`는 `PASS`: `record_telemetry_exists=true`, schema audit valid rows `234/234`, latest gate `PASS`, selected residual label `intrinsic_scale_stress` stable `214/214`, `distance_margin_min=1.043713`, operator launch `PASS`, runtime truth `false`입니다. Suite에는 `--current-schema-expected-label`을 추가해 내부 current-schema smoke의 고정 기대 라벨(`disparity_noise_stress`)과 live/latest telemetry 기대 라벨을 분리했습니다.
- 2026-06-18: EXP-272에서 실제 YOLO/BBox publisher를 붙이기 위한 external BBox runtime input을 추가했습니다. `StereoDistanceNode`는 `enable_external_bbox_detections:=true`일 때 `external_bbox_detections_topic`의 `std_msgs/msg/Float32MultiArray`를 구독합니다. 기본 topic은 `/stereo/external_detections`이고 메시지 계약은 `[left_valid,left_x1,left_y1,left_x2,left_y2,left_confidence,left_cx,left_cy,right_valid,right_x1,right_y1,right_x2,right_y2,right_confidence,right_cx,right_cy]`입니다. 입력 `cx/cy/x1/y1/x2/y2`는 image pixel coordinate이고, node 내부 계산은 stereo-center RDF(`right,down,forward`)로 수행되며 public output topic은 계속 `/stereo/estimated_position`, `/stereo/corrected_position`의 `[forward,right,down,range]`입니다. 이 runtime path는 calibration truth, PX4/RTK/NED truth를 입력으로 쓰지 않습니다. Unit evidence는 external callback이 estimated public FRD를 발행하는 것을 확인했고, print-only wrapper evidence는 `enable_external_bbox_detections:=true external_bbox_detections_topic:=/yolo/stereo_bbox`가 operator command에 전달되는 것을 확인했습니다.
- 2026-06-18: EXP-273에서 external BBox sidecar publisher를 실제 headless record-lite wrapper에 붙여 synthetic detection 없이 current telemetry와 residual gate를 통과시켰습니다. 새 sidecar는 `Simulation/stereo_ground_sim/stereo_ground_sim/external_bbox_publisher.py`이며, real YOLO/BBox publisher가 준비되기 전 `/stereo/external_detections`에 같은 16-field `Float32MultiArray` 계약을 발행하는 integration substitute입니다. Evidence `.../feature_selector/operator_record_lite_external_bbox_sidecar_clean2_probe/operator_record_lite_and_residual_suite_summary.json|txt`는 overall `PASS`: `record_status=TIMEOUT_TERMINATED`, `sidecar_returncode=0`, `record_telemetry_exists=true`, `record_telemetry_fresh=true`, `suite_used_record_telemetry=true`, schema audit valid rows `463/463`, selector window count `443`, selected residual label `disparity_noise_stress`, label stable `true`, `distance_margin_min=1.211552`, `apply_residual_allowed=true`, operator launch `PASS`, runtime truth `false`입니다. 이 결과는 측정 정확도 PASS가 아니라 external BBox topic -> StereoDistance estimated/corrected public FRD telemetry -> truth-free selector/preflight -> residual launch gate까지의 software-path PASS입니다.
- 2026-06-18: EXP-274에서 EXP-273 telemetry를 재검토하며 recorder detection schema bug를 수정했습니다. `/stereo/detections` message는 16개 값(`left/right valid,x1,y1,x2,y2,confidence,cx,cy`)인데 recorder CSV `DETECTION_FIELDS`에는 width/height/raw-disparity 파생 필드가 함께 있어, 기존 `_detections_cb`가 단순 zip을 사용하면 right-side fields가 밀렸습니다. `Simulation/stereo_ground_sim/stereo_ground_sim/operator_recorder.py`는 이제 `DETECTION_MESSAGE_FIELDS`를 별도로 사용해 16개 message field를 먼저 매핑하고, `left/right_width_px`, `left/right_height_px`, `raw_disparity_px=left_cx-right_cx`를 명시 계산합니다. 또한 `export_selector_observation_csv_from_operator_telemetry.py`는 detection 수신 전 warm-up row를 skip하고 valid rows가 있으면 PASS할 수 있게 했습니다. 새 validator `Simulation/scripts/validate_external_bbox_topic_contract.py`는 real YOLO/BBox publisher 교체 전 16-field `Float32MultiArray`의 길이, finite 값, bbox bounds, center-in-box, confidence range, positive left-minus-right disparity, forbidden truth columns를 검사합니다. Evidence `.../feature_selector/operator_record_lite_external_bbox_sidecar_schemafix2_probe/operator_record_lite_and_residual_suite_summary.json|txt`는 full record-lite `PASS`: `sidecar_returncode=0`, schema audit valid rows `473/474`, selector windows `453`, selected label `disparity_noise_stress` stable `453/453`, `distance_margin_min=1.211552`, `runtime_truth_used=false`입니다. External BBox contract validator smoke `.../feature_selector/external_bbox_topic_contract_validation_schemafix2_smoke/external_bbox_topic_contract_validation.json|txt`도 `PASS`: valid pair `473/473`, disparity min/max `206.041138px`, forbidden truth input 없음입니다.
- 2026-06-18: EXP-275에서 record-lite 반복 실행 후 `Simulation/.venv/pyvenv.cfg`가 `include-system-site-packages = false`로 되돌아가 numpy/ROS Python import가 깨지는 환경 문제를 수정했습니다. 원인은 `run_operator_view.sh`가 YOLO runtime 준비를 위해 호출하는 `Simulation/scripts/ensure_yolo_runtime.sh`가 `.venv`를 `python -m venv`로 재생성하면서 system-site 설정을 보존하지 않은 것이었습니다. 이제 이 스크립트는 기존 venv를 검사하기 전 `ensure_system_site_packages`로 `pyvenv.cfg`를 `true`로 정규화하고, 새 venv 생성도 `python -m venv --system-site-packages`로 수행합니다. 테스트 `Simulation/tests/test_ensure_yolo_runtime.py`는 이 동작을 회귀 방지하고, fake venv 행동 smoke도 false -> true 정규화를 확인했습니다. 이 변경은 측정 알고리즘이 아니라 반복 가능한 EXP/runtime 검증 환경을 안정화하기 위한 software-path gate입니다.
- 2026-06-18: EXP-276에서 real YOLO/BBox publisher를 residual pipeline에 연결하기 전 topic 자체를 독립적으로 sampling/검증하는 preflight wrapper를 추가했습니다. 새 도구 `Simulation/scripts/run_external_bbox_topic_preflight.py`는 live ROS2 `std_msgs/msg/Float32MultiArray` topic을 `--topic`으로 일정 시간 샘플링하거나, 이미 저장된 sample CSV를 `--input-csv`로 받아 EXP-274 validator를 실행합니다. 산출물은 `external_bbox_topic_samples.csv`, `external_bbox_topic_contract_validation.json|txt`, `external_bbox_topic_preflight_summary.json|txt`이고, gate는 16-field 길이, finite 값, bbox geometry/bounds, confidence range, center-in-box, positive `left_cx-right_cx`, forbidden truth columns, runtime truth 미사용을 확인합니다. Offline smoke evidence `.../feature_selector/external_bbox_topic_preflight_schemafix2_smoke/external_bbox_topic_preflight_summary.json|txt`는 EXP-274 sidecar samples 기준 `PASS`: sample count `473`, valid pair `473/473`, disparity min/max `206.041138px`, runtime truth `false`입니다. 이것도 측정 정확도 PASS가 아니라 real BBox publisher 투입 전 topic contract gate입니다.
- 2026-06-18: EXP-277에서 EXP-276 preflight를 `run_operator_record_lite_and_residual_suite.py`의 선행 gate로 통합했습니다. 새 옵션 `--external-bbox-preflight`를 켜면 wrapper는 record-lite 또는 synthetic record를 시작하기 전에 `run_external_bbox_topic_preflight.py`를 실행합니다. Preflight가 `PASS`가 아니면 `record_status=SKIPPED_BY_EXTERNAL_BBOX_PREFLIGHT`, `suite_status=SKIPPED_BY_EXTERNAL_BBOX_PREFLIGHT`로 종료하여 잘못된 YOLO/BBox topic이 residual enablement까지 넘어가지 못하게 합니다. Smoke evidence `.../feature_selector/operator_record_lite_external_bbox_preflight_synthetic_gate/operator_record_lite_and_residual_suite_summary.json|txt`는 `PASS`: external BBox preflight `PASS`, sample count `473`, valid pair `473/473`, synthetic current-schema record fresh/selected `true`, latest gate `PASS`, selected residual label `disparity_noise_stress`, operator launch `PASS`, runtime truth `false`입니다. Unit test는 정상 preflight 후 suite 진행과 음수 disparity preflight failure가 record/suite를 skip하는 경로를 모두 고정합니다.
- 2026-06-18: EXP-278에서 EXP-277 gate의 negative safety behavior를 artifact로 남겼습니다. 입력 CSV `.../feature_selector/external_bbox_topic_preflight_negative_guard_input/external_bbox_bad_disparity_samples.csv`는 bbox bounds와 center-in-box는 정상으로 유지하되 `left_cx=700`, `right_cx=730`이라 `left_cx-right_cx=-30px`가 되도록 만들었습니다. Evidence `.../feature_selector/operator_record_lite_external_bbox_preflight_negative_guard/operator_record_lite_and_residual_suite_summary.json|txt`는 wrapper status `FAIL`이지만 gate behavior는 기대대로입니다: external BBox preflight `FAIL`, failure `row0:left_minus_right_disparity_below_min:-30`, `record_status=SKIPPED_BY_EXTERNAL_BBOX_PREFLIGHT`, `suite_status=SKIPPED_BY_EXTERNAL_BBOX_PREFLIGHT`, record telemetry missing, suite not run, runtime truth `false`. 이 실험은 bad external BBox topic을 residual enablement로 통과시키지 않는 negative guard PASS입니다.
- 2026-06-18: EXP-279에서 external BBox preflight에 최소 valid pair gate를 추가했습니다. `run_external_bbox_topic_preflight.py`는 이제 `--min-valid-pairs`를 받고, validator는 통과했더라도 valid pair 수가 부족하면 `valid_pair_count_below_min`으로 FAIL합니다. 또한 `--min-disparity-px <= 0` 설정은 positive `left_cx-right_cx` 계약을 우회할 수 있으므로 `config:min_disparity_px_not_positive`로 FAIL합니다. `run_operator_record_lite_and_residual_suite.py`도 `--external-bbox-preflight-min-valid-pairs`를 받아 record/suite 전 preflight에 전달합니다. Evidence `.../feature_selector/external_bbox_topic_preflight_min_valid_pairs_pass/external_bbox_topic_preflight_summary.json|txt`는 EXP-274 samples에 `min_valid_pairs=5`를 걸어 `473/473`으로 PASS했고, `.../feature_selector/operator_record_lite_external_bbox_preflight_min_pairs_negative_guard/operator_record_lite_and_residual_suite_summary.json|txt`는 positive disparity one-row sample에서 `valid_pair_count=1<2`로 record/suite를 skip했습니다. `.../feature_selector/external_bbox_topic_preflight_min_disparity_config_guard/`는 `min_disparity_px=0` 설정 자체가 FAIL되는 것을 남깁니다.
- 2026-06-18: EXP-280에서 external BBox preflight의 live ROS2 topic sampling path를 검증했습니다. `stereo_ground_sim.external_bbox_publisher` sidecar를 `/stereo/external_detections_live_preflight`에 30Hz로 띄우고, `run_external_bbox_topic_preflight.py --topic /stereo/external_detections_live_preflight --sample-seconds 3 --max-samples 60 --min-valid-pairs 5`를 실행했습니다. Evidence `.../feature_selector/external_bbox_topic_live_preflight_sidecar_smoke/external_bbox_topic_preflight_summary.json|txt`는 `source_mode=ros_topic`, sample count `60`, valid pair `60/60`, disparity min/max `206.041138px`, runtime truth `false`로 `PASS`입니다. 참고로 sandbox 내부에서는 ROS2 DDS transport socket 생성이 제한되어 `topic:no_samples`가 발생했으며, live topic smoke는 socket 권한이 있는 환경에서 실행해야 합니다. CSV smoke와 달리 이 결과는 sampler가 실제 `Float32MultiArray` topic에서 메시지를 수신했음을 보입니다.

Headless synthetic-detection record-lite residual gate 예:

```bash
Simulation/.venv/bin/python Simulation/scripts/run_operator_record_lite_and_residual_suite.py \
  --profile-json <active_runtime_profile.json> \
  --latest-telemetry-root Simulation/bags \
  --record-dir Simulation/bags/operator_headless_synthetic_detection_current_schema_lite \
  --record-seconds 20 \
  --selector-model-json <disparity_residual_feature_selector_model.json> \
  --catalog-json <disparity_residual_profile_label_selector.json> \
  --output-dir <results_dir> \
  --window-size 21 \
  --min-samples 5 \
  --expected-label intrinsic_scale_stress \
  --current-schema-expected-label disparity_noise_stress \
  --require-stable-label \
  --min-distance-margin 0.5 \
  --operator-arg=--headless \
  --operator-arg=--no-px4 \
  --operator-arg=yolo_target_hz:=0.0 \
  --operator-arg=gz_gui:=false \
  --operator-arg=enable_synthetic_detections:=true
```

실제 YOLO/BBox topic 교체 예:

```bash
./Simulation/run_operator_view_with_calibration_profile.sh \
  --profile-json <active_runtime_profile.json> \
  -- \
  --headless \
  --no-px4 \
  yolo_target_hz:=0.0 \
  enable_external_bbox_detections:=true \
  external_bbox_detections_topic:=/yolo/stereo_bbox \
  gz_gui:=false
```

이 경우 `/yolo/stereo_bbox` publisher가 위 16개 값의 `Float32MultiArray`를 발행해야 합니다. Recorder와 residual preflight는 public FRD+range telemetry를 읽고 selector 내부에서만 stereo-center RDF로 변환합니다.

YOLO publisher 없이 external BBox topic path를 integration smoke로 재현하려면 sidecar publisher를 사용한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_operator_record_lite_and_residual_suite.py \
  --profile-json <active_runtime_profile.json> \
  --latest-telemetry-root Simulation/bags \
  --record-dir Simulation/bags/operator_external_bbox_sidecar_current_schema_lite \
  --record-seconds 22 \
  --selector-model-json <disparity_residual_feature_selector_model.json> \
  --catalog-json <disparity_residual_profile_label_selector.json> \
  --output-dir <results_dir> \
  --window-size 21 \
  --min-samples 5 \
  --current-schema-expected-label disparity_noise_stress \
  --require-stable-label \
  --min-distance-margin 0.5 \
  --operator-arg=--headless \
  --operator-arg=--no-px4 \
  --operator-arg=yolo_target_hz:=0.0 \
  --operator-arg=gz_gui:=false \
  --operator-arg=enable_external_bbox_detections:=true \
  --operator-arg=external_bbox_detections_topic:=/stereo/external_detections \
  --external-bbox-publisher \
  --external-bbox-topic /stereo/external_detections \
  --external-bbox-forward-m 1000 \
  --external-bbox-right-m 7 \
  --external-bbox-down-m 4
```

Real YOLO/BBox topic을 residual suite에 넣을 때는 record 전에 preflight gate를 켠다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_operator_record_lite_and_residual_suite.py \
  --profile-json <active_runtime_profile.json> \
  --latest-telemetry-root Simulation/bags \
  --record-dir Simulation/bags/operator_external_bbox_live_lite \
  --record-seconds 22 \
  --selector-model-json <disparity_residual_feature_selector_model.json> \
  --catalog-json <disparity_residual_profile_label_selector.json> \
  --output-dir <results_dir> \
  --window-size 21 \
  --min-samples 5 \
  --expected-label disparity_noise_stress \
  --current-schema-expected-label disparity_noise_stress \
  --require-stable-label \
  --min-distance-margin 0.5 \
  --external-bbox-preflight \
  --external-bbox-preflight-topic /yolo/stereo_bbox \
  --external-bbox-preflight-sample-seconds 5 \
  --external-bbox-preflight-min-valid-pairs 5 \
  --operator-arg=--headless \
  --operator-arg=--no-px4 \
  --operator-arg=yolo_target_hz:=0.0 \
  --operator-arg=gz_gui:=false \
  --operator-arg=enable_external_bbox_detections:=true \
  --operator-arg=external_bbox_detections_topic:=/yolo/stereo_bbox
```

`--external-bbox-preflight`가 실패하면 이 wrapper는 record telemetry를 쓰지 않고 residual suite도 실행하지 않는다. 예를 들어 left-minus-right disparity가 음수인 sample은 `record_status=SKIPPED_BY_EXTERNAL_BBOX_PREFLIGHT`와 `suite_status=SKIPPED_BY_EXTERNAL_BBOX_PREFLIGHT`로 종료되어야 한다.

실제 YOLO/BBox publisher가 낼 sample CSV 또는 `ros2 topic echo`에서 추출한 16개 값을 먼저 검증하려면 다음 contract validator를 사용한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/validate_external_bbox_topic_contract.py \
  --input-csv <external_bbox_samples.csv> \
  --output-dir <results_dir>/external_bbox_topic_contract_validation \
  --width-px 1280 \
  --height-px 720 \
  --min-disparity-px 0.5
```

CSV는 16개 컬럼(`left_valid,left_x1,left_y1,left_x2,left_y2,left_confidence,left_cx,left_cy,right_valid,right_x1,right_y1,right_x2,right_y2,right_confidence,right_cx,right_cy`)을 갖거나, 단일 `data` 컬럼에 16개 숫자를 공백/쉼표/세미콜론으로 저장할 수 있다. 이 validator도 calibration truth, PX4/RTK/NED truth 컬럼을 runtime topic 입력으로 허용하지 않는다.

실제 ROS2 topic을 바로 샘플링해 같은 검증을 실행하려면 preflight wrapper를 사용한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_external_bbox_topic_preflight.py \
  --topic /yolo/stereo_bbox \
  --sample-seconds 5 \
  --max-samples 200 \
  --min-valid-pairs 5 \
  --output-dir <results_dir>/external_bbox_topic_preflight \
  --width-px 1280 \
  --height-px 720 \
  --min-disparity-px 0.5
```

이미 저장된 CSV로 동일한 wrapper를 재현하려면 `--topic` 대신 `--input-csv <external_bbox_samples.csv>`를 사용한다. 이 wrapper가 `PASS`한 뒤에만 `enable_external_bbox_detections:=true external_bbox_detections_topic:=/yolo/stereo_bbox`를 record-lite/residual gate에 연결한다.

Live ROS2 topic 검증은 DDS transport socket을 열 수 있는 환경에서 실행해야 한다. 제한된 sandbox에서 `topic:no_samples`가 발생하면 같은 명령을 실제 ROS2 실행 권한이 있는 셸에서 다시 실행한다.
- 2026-06-18: non-alias synthetic robustness 결과를 StereoDistance에서 바로 선택할 수 있는 scenario catalog label selector package로 export했다. 입력 결과는 `Simulation/results/non_alias_robustness_default6_seed2_probe`이며, direct NED-to-RDF alias는 6개 scenario/12개 seed config 모두에서 금지 상태로 확인됐다. 생성 명령은 다음과 같다.

```bash
python3 Simulation/scripts/export_robustness_scenario_catalog_selector.py \
  --robustness-dir Simulation/results/non_alias_robustness_default6_seed2_probe \
  --output-dir Simulation/results/non_alias_robustness_default6_seed2_probe/runtime_label_selector_package_operator_gate \
  --baseline-scenario nominal \
  --with-deployment-frontier \
  --with-operator-launch-checks
```

생성물은 `runtime_label_selector_package_operator_gate/runtime_profile_label_selector.json`이며, `deployment_label`, `scenario_label`, `scenario_name` 같은 명시적 operator/deployment metadata만 사용한다. PX4/RTK/NED truth, validation error, target-window label, replay metric은 runtime selection input이 아니다. 같은 명령이 `runtime_deployment_frontier/`와 `operator_view_launch_checks/`도 함께 생성한다. 6-scenario evidence에서 selector export `PASS`, frontier `PASS`, operator launch `19/19 PASS`, runtime truth `false`를 확인했다.

6-scenario selector deployment readiness를 한 번에 재검증하려면 다음을 실행한다.

```bash
python3 Simulation/scripts/check_non_alias_selector_deployment_readiness.py \
  --robustness-dir Simulation/results/non_alias_robustness_default6_seed2_probe \
  --selector-package-dir Simulation/results/non_alias_robustness_default6_seed2_probe/runtime_label_selector_package_operator_gate \
  --readme Simulation/README.md \
  --output-json Simulation/results/non_alias_robustness_default6_seed2_probe/runtime_label_selector_package_operator_gate/non_alias_selector_deployment_readiness.json \
  --output-csv Simulation/results/non_alias_robustness_default6_seed2_probe/runtime_label_selector_package_operator_gate/non_alias_selector_deployment_readiness.csv \
  --output-txt Simulation/results/non_alias_robustness_default6_seed2_probe/runtime_label_selector_package_operator_gate/non_alias_selector_deployment_readiness.txt
```

저장된 readiness 결과는 `PASS`, requirement `7/7`, scenario/profile `6/6`, operator launch `19/19`, runtime truth `false`다.

2026-06-18 이후 `run_rtk_stereo_robustness_sweep.py`는 scenario catalog가 PASS이면 위 selector package와 deployment frontier를 자동으로 생성한다. 별도 export 명령 없이 단일 robustness sweep 결과의 `robustness_summary.json|txt`에 `runtime_label_selector_package` 섹션이 기록된다. 이 산출물은 robustness acceptance matrix의 `runtime_label_selector_package` 항목에도 포함되며, selector export `PASS`, frontier `PASS`, operator-view launch dry-run `PASS`, profile/label count 양수, runtime truth `false`를 모두 만족해야 전체 robustness acceptance가 PASS다. Smoke evidence는 `Simulation/results/non_alias_robustness_auto_selector_operator_gate_smoke`이며, 2개 non-alias synthetic scenario에서 direct NED-to-RDF alias 금지 `2/2`, selector export `PASS`, frontier `PASS`, operator launch `7/7 PASS`, runtime truth `false`, final robustness acceptance `PASS`를 확인했다.

중요: synthetic mode에서도 카메라 optical frame이 local NED에 대해 `forward=north`, `right=east`, `down=down`으로 정렬되어 있다고 가정하면 안 된다. `truth_rdf`는 explicit `T_source_to_stereo_rdf` 또는 이미 stereo-frame truth를 내는 producer에서만 와야 하며, 직접 alias `rdf_right=east`, `rdf_down=down`, `rdf_forward=north`는 현재 gate에서 금지된다. 평가는 계속 stereo-center RDF(right,down,forward)에서 수행하고, StereoDistance public topic은 별도 FRD+range(forward,right,down,range) 계약으로 변환된다.
- 2026-06-11: EXP009 `bounded_gain_forward_cell_guard` JSON profile을 StereoDistance runtime loader에서 지원하도록 추가했습니다. 보수 Down guard를 적용한 EXP013은 5-trial Simulation Runtime 검증 `PASS`이며, 아직 hardware/WIRIS 검증은 아닙니다.
- 2026-06-09: YOLO inference 기본 입력 크기를 `1280`으로 통일했습니다.
