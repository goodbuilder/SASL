# RQT operator view

Run the StereoCAM operator entry point with `./run_operator_view.sh`. The
default `operator_rqt.perspective` loads one tabbed `OperatorPanel` in the
same rqt window. It keeps setup, flight observation, analysis, and raw
diagnostics separate while retaining the three recording presets. Pair
diagnostics are operator visualization only, not qualification evidence, and use completed evaluated pairs in
`baseline_center_frd` (metres; angles in degrees). Missing values are omitted
rather than replaced by zero.

`/stereo/uam_raw_gps_reference_position` is a distinct GPS-based target
reference in the same public FRD frame. It converts simultaneous UAM and
left-camera raw WGS-84 fixes through ECEF and left-local ENU, then anchors the
result at the nominal left optical center before applying the fixed rig FRD
transform. It is shown as `UAM GPS` beside Stereo measurements in rqt and as
`UAM GPS Ray` from the fixed baseline-center origin in RViz. The same source is
recorded in CSV, but is not simulator ground truth.

Simulation World uses ENU; camera optical vectors use RDF; public measurement
vectors use fixed `baseline_center_frd` (Forward, Right, Down), with origin at
the nominal optical-center midpoint and quaternion metadata `xyzw`.

## Preflight versus flight

Use the `Setup` tab before takeoff. It shows the model, mission and coordinate
contract, and the live parameters used by the current run.
It also contains the recording controls: `Save CSV`, `Save MP4`, `Save Debug`,
`Stop`, and the output directory. `Save CSV` writes telemetry; `Save MP4` adds
left/right overlay video; `Save Debug` also adds a rosbag. While recording,
all preset buttons and the output directory are locked. Auto itself does not
start a recorder: these buttons remain available during Auto, and the selected
recording starts at the button click. `Flight` contains balanced left/right camera images and direct telemetry values. `Analysis` gives the
Forward, Right, Down, and Range plots the whole tab. The Raw GPS baseline
publishes F/R/D; its Range plot value is the explicit 3D vector magnitude.
Global Baseline is displayed from the same Ray snapshot as the other diagnostic Rays.
`Debug` provides raw runtime
status without turning it into an automated warning or qualification result.
Calibration recording remains a CLI workflow.

When a mission was started from the Flight toolbar, `Stop` stops the owned
Mission/Waypoint/Auto process group. For Auto, the mission requests PX4 `LAND`
on interruption, and the recorder receives the same stop signal before the UI
waits for CSV flush, MP4 release, and XLSX/compact-session finalization. The
saved artifacts therefore contain only samples accepted from the Save button
click through the Stop action. If no Save button was pressed, Auto produces no
CSV or MP4 artifact.
An Auto process started separately from a terminal or from `--auto-mission` is
not owned by this rqt control and must be stopped by its launcher/QGC path.

`Flight` also has a `Ray Diagnostics` group below the direct
telemetry line. It reads the existing `/stereo/ray_metrics` snapshot only; it
adds no triangulation, GT, or GPS-anchor logic of its own. `Geometry` lists
seven rays (`Nominal Pose`, `Optical Pose`, `Ideal`, `GPS Baseline`,
`Global Baseline`, `Attitude`, `Operational`) with a state chip, Forward/Right/Down/Range/Gap/Angle in
`baseline_center_frd`, and a tooltip carrying the underlying
`computation_status`/`failure_reason`. `Optical Pose Δ` shows how much
`Attitude` and `Operational` move relative to `Optical Pose`. The former
Interaction panel was removed because its `Full − Attitude − Baseline + Truth`
decomposition depended on the removed Baseline Ray and was not a directly
triangulated result. `GT Error` shows absolute error against the Gazebo
`target_reference_link` Ground Truth, which is different from Optical Pose
(an oracle triangulation of observed pixels, not the target GT point itself).
`Sensors` shows PX4 attitude and GPS baseline input health (age, pair skew,
reset counters, quaternion order/semantics, baseline length and anchor
policy) read from the same snapshot, with quaternion components and baseline
ENU vectors in hover tooltips to keep the default view compact. Every numeric
cell shows `—` for a missing or invalid value instead of `0.000`, and a new
`/stereo/ray_metrics` snapshot fully replaces the previous one so an invalid
pair cannot leave a prior pair's numbers on screen.

`Record` shows the path, mode, persisted row count, elapsed time, and factual
coverage counts from the active session's atomic `progress.json`. It refreshes
the exact persisted CSV tail twice per second while writing and shows only
complete newline-terminated rows. Its purpose views split the same latest rows
into `Summary` (range and Raw GPS overview), `Raw GPS` (full Raw GPS FRD view),
`BBox` (both source bbox corners and disparity), `PX4 / Camera` (left/right
Raw GPS, PX4 local-NED state, and estimated world-ENU camera exterior pose),
and `Raw CSV` (copyable exact tail). GlobalPosition remains in the underlying
CSV/XLSX for offline comparison. After
`Stop`, use `Open session` for the artifact folder, `Open CSV` for the immutable
raw `telemetry.csv`, `Open XLSX` for the finalized workbook, or `Export analysis CSV` to create the compact
`analysis.csv` without modifying the raw evidence.

At `Stop`, the recorder first finalizes the authoritative `telemetry.csv`, then
creates `session.xlsx` from it. The workbook contains `Summary`, `Baseline`,
`BBox`, `Camera Pose`, and `Manifest` sheets. The Manifest includes the CSV row
count, FRD contract, SHA-256, and each fixed base-to-optical camera mount 6-DOF
snapshot. The `Camera Pose` sheet retains the time-varying PX4-estimated
world-ENU camera exterior 6-DOF, derived from local-NED odometry bootstrapped
to the Gazebo camera diagnostic. This keeps the human-readable workbook tied to its
single raw CSV source. If XLSX creation fails, the finalized CSV is preserved
and the factual export status is shown in `progress.json`.

The Setup tab also reads the live ROS parameter list from
`/stereo_distance_node` and displays the values actually used by the running
simulation. Use `Refresh run parameters` after changing an allowed runtime
parameter. Apart from PX4 baseline source and the three explicit YOLO controls,
this display is read-only: scenario, geometry, model path, and other
restart-sensitive settings must be changed before launching a new run.

Use `Open ROS graph` in the `Debug` tab when a connection diagnosis is needed.
It opens native `rqt_graph` in a separate window, preserving the full tabbed
operator workspace for setup, flight, and analysis. It is an inspection view
only: it does not change flight, sensor, or recording settings.

The two `/stereo/gps_baseline_position` and `/stereo/global_baseline_position`
topics remain the canonical stamped FRD results
for recorders. The rqt plots use the matching `*_baseline_plot` arrays
(`data[0..2] = Forward, Right, Down`) so simulation header time cannot move a
curve outside rqt's wall-clock time window.

Do not use interactive operator parameters as deterministic sweep, freeze, or
sealed-evaluation inputs. Atomic sealed execution and the sealed runner remain
R4-B work.

## GUI를 유지한 baseline CSV 기록

`Setup` 탭의 `Save CSV`는 Lite 기록을, `Save MP4`는 Full 기록을, `Save Debug`는
Debug 기록을 시작한다. `Save CSV`는 `telemetry.csv`만, `Save MP4`는 CSV와 좌/우
overlay MP4를, `Save Debug`는 그 결과물과 rosbag까지 남긴다. 바로 옆 `Stop`으로
종료한다. `Auto` 실행 중에도 Save 버튼을 누를 수 있으며, 버튼 클릭 이후부터
수집된 데이터만 저장한다. 별도 `operator_recorder` 프로세스가 현재 ROS 토픽을 구독하므로
카메라·그래프·baseline source 선택 상태는 유지된다.

각 `telemetry.csv` 행은 public FRD (Forward, Right, Down) 미터 계약으로
`gps_baseline_*`와 `global_baseline_*`를 동시에 남긴다. 각
`*_position_error_m`은 `/ground_truth_position` 대비 3D FRD 오차이며,
`gps_baseline_minus_global_baseline_{forward,right,down}_m` 및
`gps_baseline_minus_global_baseline_delta_m`은 raw GPS minus PX4 GlobalPosition의
축별/3D 차이다. 값이 없는 스트림은 0으로 채우지 않고 빈 셀로 남긴다.

This phase provides only a pure terminal-record adapter. Runtime publishers,
DiagnosticArray, panel integration, launcher flags, and parameter safety are
not connected yet. Runtime pair IDs are positive integers. The adapter
compares the `StereoImagePair` identity with the identity copied into the
assembled terminal record; `GroundTruthAlignment` itself does not carry that
identity. Only evaluated/processed/evaluated records are accepted. Missing or
non-finite individual metrics are omitted, but a snapshot with no finite
metrics is not allowed. Values are immutable tuple-of-tuples. Operator
diagnostics are not qualification evidence. The prior force-with-lease Git
procedure violation remains recorded separately.
