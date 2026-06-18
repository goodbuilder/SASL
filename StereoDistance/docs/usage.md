# StereoDistance

## 먼저 읽기

StereoDistance는 좌/우 WIRIS RTSP 카메라, YOLO 검출, Pixhawk 자세/RTK 데이터를 함께 사용해 현장에서 객체 거리와 3D 위치를 확인하는 PyQt5 운용 도구입니다.

빠른 실행:

```bash
cd <STEREO_DISTANCE_APP>
./run.sh
```

자주 쓰는 실행 옵션:

```bash
./run.sh --maximized      # 최대화 실행
./run.sh --start          # YOLO/Depth 스트림 자동 시작
./run.sh --wiris-only     # WIRIS 스테레오 창만 실행
```

문서 역할:

- `docs/usage.md`: 실행 방법, 운용 순서, 상세 사용법.
- `PROJECT.md`: 프로젝트 목적, 시스템 구성, 핵심 워크플로, 개발 방향.
- `COORDINATE_CONTRACT.md`: 좌표계, 축 이름, CSV 필드, UAM 보정 profile 계약.
- `DESIGN.md`: 프로젝트 UI/스타일 기준.
- `DESIGN-ibm.md`: IBM/Carbon 계열 원본 디자인 레퍼런스.
- `../UPDATE.md`: 의미 있는 변경 이력과 인수인계 로그.
- `AGENTS.md`: 코딩 에이전트 작업 규칙.
- `docs/`: 프로젝트 문서 디렉터리.

세부 문서:

- `testing.md`: 테스트와 검증 방법.

기본 현장 순서:

1. QGC에서 Pixhawk 보정을 먼저 완료한다.
2. WIRIS 카메라와 Pixhawk 연결을 확인한다.
3. WIRIS 사진 시간 차이를 확인한다.
4. 체커보드로 WIRIS 내부 파라미터를 보정한다.
5. GPS baseline, RM3100, 자세 정렬을 확인한다.
6. YOLO 설정 후 `세션 기록 시작`과 측정을 시작한다.
7. 보정 세션에서는 ULog로 좌표 보정 profile을 만들고 저장한다.
8. 본 실험에서는 저장된 profile과 새 ULog 또는 telemetry fallback으로 UAM 비교 row를 만든다.
9. 거리 기록, 설정, UAM 분석 CSV, 로그를 저장한다.

아래 내용은 기존 상세 운용 매뉴얼입니다.

## Purpose

StereoDistance is a PyQt5 field GUI for measuring object distance from two WIRIS RTSP camera streams while monitoring two already-calibrated Pixhawk systems. QGC is the expected tool for Pixhawk calibration before this app is used; this app separates field connection, WIRIS time verification, WIRIS intrinsic calibration, RTK/RM3100/attitude alignment, YOLO/depth measurement, and saving/analysis into a step-by-step workflow.

The app is designed for this operating sequence:

1. Calibrate Pixhawks in QGC before launching this app.
2. Connect WIRIS cameras, Pixhawk MAVLink/ROS2, and confirm stream reception.
3. Verify WIRIS photo timestamp offset from externally triggered still images.
4. Capture checkerboard images for either camera and compute WIRIS intrinsic parameters.
5. Align GPS baseline, compare RM3100 CAN1/CAN2 data, and align camera attitude.
6. Configure YOLO.
7. Start session recording, then start YOLO/depth measurement.
8. For a calibration session, load PX4 ULog, estimate/review time offset, estimate the coordinate-alignment profile, and save it.
9. For the main experiment, load the saved coordinate-alignment profile, load the new PX4 ULog or use telemetry fallback, run UAM comparison, and calculate accuracy metrics from matched rows while reporting each truth source separately.
10. Save distance history, settings, analysis CSVs, and review logs.

## Current Run Flow

Launch through `run.sh` so ROS2 and the local PX4 message workspace are sourced before the GUI starts.
The launcher also tries to install `arp-scan` when it can do so without an interactive sudo password; if that is not possible, the GUI still starts and Auto Detect falls back to TCP scans.

```bash
cd <STEREO_DISTANCE_APP>
./run.sh
```

Useful options:

```bash
./run.sh --maximized
./run.sh --start        # auto-start YOLO/depth streams
./run.sh --wiris-only   # open the standalone WIRIS stereo window
```

The current top-level tab order follows the field workflow:

1. `연결`
2. `시간`
3. `보정`
4. `파라미터`
5. `정렬`
6. `YOLO`
7. `측정`
8. `저장·분석`
9. `고급`

The `연결` tab contains `WIRIS`, `Pixhawk`, and `UAM`. It keeps only the primary connection workflow.
The `시간` tab contains WIRIS still-image EXIF timestamp comparison.
The `보정` tab contains a `체커보드` subtab for checkerboard capture/calibration/zoom control and a `줌 내부 파라미터` subtab for per-zoom intrinsics with save/apply controls.
The `파라미터` tab contains `내부` and `외부/기하` subtabs so intrinsics and live geometry state can be inspected separately.
The `정렬` tab contains `Baseline`, `MAG`, and `자세`.
The `고급` tab contains `WIRIS SDK`, `필드 감시`, and `Bias 보정`.
The `측정` tab contains `실시간`, `계산 설정`, and `삼각측량 품질`.
The `저장·분석` tab contains `세션 저장`, `UAM 비교`, and `기록 샘플`.

At startup the GUI only reports the selected network interface and leaves camera/Pixhawk checks in a waiting state. Connect the hardware first, then use `영상 수신 확인`, `카메라 Preview 연결`, `ROS2 연결`, or `MAVLink 연결`; `--start` is the opt-in path for immediately starting YOLO/depth streams. Camera network fields follow the WIRIS OSD names where practical: `IP Address`, `IP Mask`, `RTSP port`, and `Telnet port`.

### 연결

Use this tab for WIRIS/Pixhawk connection controls and field readiness checks.

- `Telnet port 탐색` scans candidate WIRIS `IP Address` values for TCP ports `1-65535`, verifies open ports with `HIWS`, and applies the discovered `IP Address`/`Telnet port` only to the side whose button was pressed. `WIRIS 자동 연결` is the path that can fill both sides at once.
- `WIRIS 자동 연결` tries a monitorless setup flow: full-scan WIRIS SDK control ports, verify each candidate with `HIWS`, fill left/right control hosts and their per-camera ports, sync RTSP hosts, send Ethernet Mode commands, check RTSP frames, and connect camera preview when both streams are valid.
- `영상 주소 탐색` scans camera network candidates for RTSP ports and fills the PC camera-network IP plus left/right camera RTSP URL fields.
- When `영상 주소 탐색` finds only one RTSP candidate, the GUI keeps the other side unchanged and updates only the side whose current control host or RTSP host matches that candidate, so left/right URLs do not collapse to the same IP.
- `탐색 후보` shows the latest NIC, RTSP, and WIRIS TCP/HIWS candidates found by camera discovery. Select a row and use `선택 NIC`, `Left에 적용`, or `Right에 적용` to manually apply that candidate without retyping interface names, IPs, URLs, or ports.
- `PC IP 탐색` reads the active NetworkManager profile on the selected interface and fills the PC camera-network IP field when it finds a camera-subnet address. If the saved interface is missing after a USB-C LAN adapter swap, it selects an available wired interface and fills the camera-network IP candidate.
- `PC IP 적용` applies the compact top-row `PC IP` value to the selected `인터페이스` so TCP control and RTSP checks route through the camera NIC. The automatic camera path now first replaces a missing saved interface with an available wired adapter, reads the active NetworkManager connection on that interface, reuses any camera-subnet address stored there, and then applies the resulting address before camera-network discovery, RTSP frame checks, WIRIS control checks, RTSP enable, and camera preview start.
- When `arp-scan` is installed and permitted, `Telnet port 탐색` and `영상 주소 탐색` use ARP scan results as host candidates before falling back to TCP subnet scans.
- `영상 주소 탐색` scans the full local camera subnet when fewer than two usable defaults are found, instead of stopping after the first two port candidates.
- `제어 확인` checks whether each WIRIS `IP Address` accepts TCP commands on its own configured `Telnet port`. It does not change RTSP URLs; if an OSD `IP Address` and RTSP URL host differ, the app logs a warning and leaves the stream settings untouched.
- `IP Address / IP Mask 변경` contains STIP static-IP controls. `STIP 적용` writes a new OSD `IP Address` and `IP Mask` to the selected WIRIS through its current `IP Address`/`Telnet port`; use it only when the target camera is uniquely identified, preferably with one WIRIS connected.
- The `RTSP port / 영상` group shows the full `RTSP URL`; its URL port is the OSD `RTSP port`. Its `IP Address` + `RTSP 적용` controls update only the RTSP URL host, while Telnet control settings remain separate.
- `영상 송출 활성화` sends only the WIRIS Ethernet Mode command to the configured left/right control hosts with each side's control port and LINE termination, waits briefly for the RTSP server to open, then re-runs RTSP frame checks. It does not change GOP; use the WIRIS SDK command controls in `고급 > WIRIS SDK` when GOP must be checked or changed manually.
- If a saved `Telnet port` is open but returns `ERR` for `GETH`, the app now treats it as the wrong SDK command port, tries the default WIRIS SDK port `2240`, saves the corrected port when `GETH` returns `TRUE/FALSE`, and waits until the RTSP URL port is actually open before checking frames.
- `영상 수신 확인` checks both TCP port reachability and actual first-frame reception using the selected stream `백엔드`. A camera can report port-open while still failing stream/frame reception.
- `카메라 Preview 연결` starts left/right camera preview streams without loading YOLO; while connected, the same button changes to `카메라 Preview 해제` and stops camera preview plus any active YOLO stream without disconnecting Pixhawk.
- If left/right RTSP URLs resolve to the same stream identity, preview/YOLO start and distance calculation are blocked because the pair is not a valid stereo source.

### YOLO

Use this top-level tab for model, inference-device, and stream processing settings before measurement.

- `입력 크기(px)`: YOLO input image size. Larger values can help accuracy, but they cost more latency and throughput. `416` is a practical throughput-oriented default.
- `신뢰도 임계값`: YOLO confidence threshold.
- `IoU 임계값`: YOLO non-max suppression IoU threshold.
- `가중치`: YOLO model weights path. A changed value is persisted by `전체 설정 저장` or when the window closes.
- `미리보기 폭(px)`: maximum preview image width sent to Qt. Keep this at `1920` for the current 1080p WIRIS stream so Qt receives the encoded frame without an extra resize. YOLO detection still uses the original decoded frame.
- `미리보기 Hz`: maximum preview display rate in Qt. It cannot exceed the camera RTP/RTSP source FPS. `15 Hz` is the current tuned default so preview load stays aligned with the YOLO target.
- `RTP 지연(ms)`: GStreamer RTP jitter-buffer latency in milliseconds.
- `디코더`: RTSP H.264 decoder mode. `NVIDIA` is the default and forces `nvh264dec`; `Auto` tries NVIDIA first and falls back to software; `Software` forces `avdec_h264`.
- `수신 백엔드`: RTSP receive backend. `GStreamer` gives the best latency/drop/decoder control; `OpenCV/FFmpeg` is a simpler fallback path; `OpenCV Auto` lets OpenCV choose the available capture backend; `PyAV` uses the optional `av` package and reports `pyav-pts` when frame PTS is available.
- `RTSP 전송`: RTSP media transport. `UDP` is the default for live streams because it avoids TCP receive-queue buildup when decoding falls behind; `TCP` remains available for networks where UDP is blocked or too lossy.
- `추론 장치`: toggle for Ultralytics/PyTorch inference device. `CPU 추론` passes `cpu`; `CUDA 추론` passes `cuda:0`.

### 연결 > Pixhawk

This connection subtab assumes Pixhawk calibration was already completed in QGC and focuses on ROS2/MAVLink transport setup.

- `Pixhawk 자동 탐색` scans ROS2 PX4 topics using the current `ROS_DOMAIN_ID` and fills Pixhawk namespaces.
- The Pixhawk subtab is ordered as `0 공통 준비`, `1 탐색 후보 / USB 할당`, `2 Pixhawk 1/2 사전 설정`, and `3 연결 실행` so USB bootstrap, persistent Ethernet/uXRCE setup, MAVLink, and ROS2 checks stay in the field order.
- `1 탐색 후보 / USB 할당` lists ROS2 namespace and USB MAVLink candidates. Select a row and use `P1에 적용` or `P2에 적용` to apply that namespace or USB MAVLink device to a specific Pixhawk side.
- `2 Pixhawk 1/2 사전 설정` keeps Pixhawk 1 and Pixhawk 2 in separate panels with each side's MAVLink URL, baudrate, Ethernet IP, namespace, PX4 shell, and `ETH 저장` action.
- `MAVLink 연결` opens direct MAVLink links; while connected, the same button changes to `MAVLink 해제`.
- `ROS2 연결` starts or reuses `MicroXRCEAgent`, then subscribes to available PX4 ROS2 topics; while connected, the same button changes to `ROS2 해제`.
- `픽스호크 Host IP` should be the PC address reachable from the Pixhawk Ethernet/uXRCE network. The field default is `10.41.10.1`.
- If `ROS2 연결` fails during session startup, the GUI automatically starts the configured MAVLink side(s), sends the selected MAVLink stream setup and Pixhawk-side `uxrce_dds_client` command when those toggles are enabled, keeps MAVLink telemetry as fallback, and retries ROS2 after the ROS2 client command is sent.
- After each MAVLink heartbeat, the app requests `ATTITUDE`, `GPS_RAW_INT`, `GPS2_RAW`, `SCALED_IMU`, `SCALED_IMU2`, and `SCALED_IMU3` with `MAV_CMD_SET_MESSAGE_INTERVAL`. This lets the alignment tab fill RM3100 CAN1/CAN2 readings from MAVLink when PX4 ROS2 magnetometer topics are not published.
- When `ROS2 client 시작` is enabled, the MAVLink path sends `uxrce_dds_client stop` and restarts the Pixhawk-side client with the selected namespace, for example `-n uav_0` or `-n uav_1`. If the initial UDP shell command is sent before a MAVLink peer is known, the app retries the pending setup after the first heartbeat.
- Launching with `--start-pixhawk` or `--connect-pixhawk` automatically opens MAVLink and switches the first view to `정렬 > MAG`.
- `Pixhawk 1 Shell` and `Pixhawk 2 Shell` open PX4 shell dialogs over an active MAVLink connection.

### 연결 > UAM

Use this subtab for UAM/drone ground truth telemetry. It is separate from `연결 > Pixhawk` because the same Pixhawk transport can serve two roles: vehicle telemetry for measurement geometry and UAM/drone center telemetry for accuracy analysis. The experiment assumes the UAM/drone GPS antenna and Pixhawk are mounted at the vehicle center, so the received PX4 local position is treated as the body-origin ground truth N/E/D.

- `소스` selects `ROS2 topic`, `MAVLink LOCAL_POSITION_NED`, `MAVLink GPS Position`, `MAVLink Global Position`, or `사용 안 함`.
- ROS2 mode registers `uam_x`, `uam_y`, and `uam_z` watches against the selected topic and fields. The default is `/uav_0/fmu/out/vehicle_local_position` with `x/y/z`.
- MAVLink `LOCAL_POSITION_NED` mode registers the same `uam_x/y/z` watches against a selected Pixhawk side and message. The default message is `LOCAL_POSITION_NED` with `x/y/z`.
- MAVLink GPS/global modes do not write raw `lat/lon/alt` into `uam_x/y/z`.
  They receive `GPS_RAW_INT`/`GPS2_RAW` or `GLOBAL_POSITION_INT`, preserve the
  raw latitude/longitude/altitude, convert it to local N/E/D using the first
  valid sample as an explicit fallback origin, and mark the live sample source
  as GPS/global-derived. Treat this live conversion as an operational monitor;
  final calibration and PASS/FAIL evidence should be regenerated offline from
  ULog/CSV with the chosen origin, time offset, and alignment profile.
- In data-collection sessions, keep local, global, and GPS truth sources
  whenever they are available. In profile-generation or UAM comparison steps,
  choose exactly one truth source contract (`local_ned`, `global_position`, or
  `gps_position`) for that profile/evaluation run. Do not mix the three truth
  sources inside one calibration profile.
- `UAM 텔레메트리 연결` starts the selected watch path. ROS2 remains the preferred source, but MAVLink keeps a live fallback buffer when ROS2 is blocked.
- `UAM 텔레메트리 해제` removes only the UAM `uam_x/y/z` watches. It does not disconnect Pixhawk ROS2/MAVLink, and it keeps the live buffer for fallback analysis.
- `실시간 버퍼 초기화` clears only the live UAM ground truth buffer and N/E/D graph; it does not delete saved session rows or selected ULog files.
- The latest UAM/drone center coordinate is also reused by `고급 > Bias 보정`. `연결 > UAM` is the single ground-truth telemetry entry point; legacy `drone_x/y/z` values are accepted only when they already exist in old recorded rows.

### 정렬

Use this top-level tab to align the field geometry and sensor sources used by measurement.

- `Baseline` aligns the physical RTK antenna baseline with the camera baseline needed by stereo measurement.
- `MAG` compares Pixhawk 1/2 health, RM3100 CAN1/CAN2 readings, F9P GPS1/GPS2 data, and dual-GNSS heading readiness.
- `MAG > 요약` includes a `PX4 Magnetometer / RM3100 관리` panel split into `Pixhawk 1` and `Pixhawk 2` tabs. Each tab lists that Pixhawk's `CAL_MAG0` through `CAL_MAG4` device IDs, priorities, expected selected MAG, and actual selected MAG when `/fmu/out/vehicle_magnetometer` provides `device_id`.
- The current Pixhawk 6X Pro custom DDS firmware build exports `/fmu/out/sensor_mag` so RM3100 raw samples can be graphed. It intentionally does not export `/fmu/out/vehicle_magnetometer` because the v6x FLASH image is full; actual selected MAG remains unavailable over ROS2 in that build.
- `RM3100을 Yaw 기준으로` runs against the selected Pixhawk tab: it raises registered RM3100 priorities to `100` and `90` and sets non-RM3100 `CAL_MAG*_PRIO` values to `0`; `파라미터 저장` and reboot still require QGC or PX4 shell unless a MAVROS2 command path is wired for the field setup.
- In each RM3100 panel, `Yaw 기준` can query the current PX4 `CAL_MAG_PRIME` setting or apply a CAN1/CAN2 RM3100 sample with a known `device_id` as the primary magnetometer via MAVLink PX4 shell.
- `자세` compares Pixhawk attitude streams when two namespaces are configured.
- `요약` shows Pixhawk 1 and Pixhawk 2 health side by side.
- `픽스호크 1` and `픽스호크 2` each show `RM3100 · CAN1/CAN2`, `F9P · GPS1/GPS2`, magnitude delta, vector-angle delta, GNSS quality comparison, sample skew, and dual-GNSS heading readiness.
- Roll, Pitch, and Yaw each have a separate graph.
- Each graph plots two series: Pixhawk 1 and Pixhawk 2.
- Y-axis range defaults to `Auto` and can be changed per axis.

ROS2 topic subscription is tolerant of missing PX4 message types. Missing GPS or magnetometer topic types should not make the whole Pixhawk connection fail.
When ROS2 magnetometer topics have no publisher, `MAVLink 연결` can still populate RM3100 CAN fields from `SCALED_IMU*` magnetometer values after the stream request succeeds.

- `측정 > 계산 설정`의 `Baseline` is the shared baseline reference for the Pixhawk 3D path's translation magnitude.
- The `Baseline > GPS 계산` subtab can calculate a baseline from left/right latitude, longitude, and altitude, then apply that result into the manual `Baseline` field.
- `Baseline > 실시간 GPS` keeps the live Pixhawk GPS baseline as an alignment reference for N/E/D, quality, and camera-frame component checks; the Pixhawk 3D path still uses the measurement-tab `Baseline` as its scale reference.
- `RTK Fixed 품질 확인` marks non-fixed GPS as degraded/bad quality for the live reference, but GPS-only 3D fixes can still be visualized.
- The baseline display shows length, N/E/D vector, sample age, quality, and warnings.
- The alignment visualization separates the antenna vector from the camera-frame components. The left plot shows the GPS antenna baseline in the N/E plane, and the right plot shows camera-frame X/Z once Pixhawk attitude is available.
- If only GPS is available, the right plot still draws a GPS-only E/U projection instead of waiting for MAG or attitude data.
- The same visualization overlays camera attitude arrows inside the camera X/Z plot: the optical-axis arrow reflects yaw/pitch, the roll arc shows roll, and the summary text reports relative `ΔR/P/Y`.
- The camera-frame X component shows the left/right optical-baseline component. The full RTK antenna length is shown for alignment but is not automatically equivalent to the camera optical baseline.

### 보정

Use this tab after camera connection and WIRIS timestamp verification, before parameter review and RTK/RM3100/attitude alignment.

- The `체커보드` subtab contains WIRIS zoom controls, checkerboard capture, intrinsic calibration, and calibration preview.
- WIRIS zoom controls can target left, right, or both cameras.
- Calibration preview uses current camera frames.
- Checkerboard overlay is only refreshed while `보정` is visible, and live checkerboard detection runs in a background worker so tab entry and resizing do not block the main GUI.
- When `YOLO 측정 시작` has switched into active measurement mode, Camera Setup preview is intentionally hidden and calibration preview frame copying is skipped. Camera Setup and Depth Measurement are not intended to be used at the same time.
- `캡처` saves only the current left/right frame(s) where the checkerboard is detected. One visible camera is enough for that camera's intrinsic dataset.
- `보정 실행` runs OpenCV checkerboard calibration. The default checkerboard is 13 inner columns, 8 inner rows, and 40.0 mm square size.
- The `줌 내부 파라미터` subtab shows the currently visible WIRIS zoom's left/right intrinsics like the Parameter tab and puts `저장` / `적용` on the right.
- `저장` writes the calibration for the current visible WIRIS zoom level under `<STEREO_DISTANCE_APP>/calibration_captures/` as `intrinsics_zoom_##.json` and refreshes the latest `intrinsics.json` alias.
- `적용` loads the current visible WIRIS zoom's intrinsics and applies them to distance calculation.
- The `저장 파일 비교` list lets you add multiple saved intrinsics JSON files, compare their left/right fx/fy/cx/cy/error values in a scrollable list, remove entries from the comparison list without deleting files, drag entries to reorder them, and apply the selected comparison file to distance calculation.

### 파라미터

Use this tab after `보정` to confirm the geometry that will be used by alignment and measurement.

- `내부` shows the currently loaded left/right intrinsics, distortion coefficients, image count, reprojection error, and whether the result is active or only the latest calibration output. Zoom-specific saves are keyed off the visible WIRIS zoom level used when saving or loading calibration.
- `외부/기하` shows live Pixhawk-derived pose, the source of the effective focal length and baseline, the fixed `nominal-shared-rig-v1` mount assumption, loaded UAM ground truth source/sample counts, and whether a 2D bbox-center adjustment matrix is loaded. The `계산 설정` view still splits `내부 파라미터` and `외부/기하` side by side around the center line. The default measurement mode is 3D ray triangulation with `Pixhawk 자세 + RTK 위치`; if required intrinsics, attitude, or RTK translation are unavailable, the app waits instead of recording a simple disparity fallback.
- `수동 초점거리 사용` allows manual `fx` entry when a checkerboard is unavailable. The focal-length and WIRIS zoom-level tooltips include nominal WIRIS OZ zoom estimates from index `0` to `14`, using the 1280x720 HFOV 63.7 degree estimate where index `0` is about `1,030 px` and index `14` is about `30,906 px`.

### 고급 > WIRIS SDK

`WIRIS SDK` can send SDK commands selected from grouped presets or typed manually. Use the `Stream / Set GOP size` and `Stream / Get GOP size` presets there when you want to verify or change visible-stream GOP by hand. Presets default to LINE termination, commands with placeholders such as `<IPv4>` must be edited before sending, and potentially disruptive commands prompt for confirmation.

### 고급 > 필드 감시

`필드 감시` can either accept manual ROS2 topic/field input, run `ROS2 토픽 찾기`, or run `MAVLink 필드 찾기`. ROS2 selections use the Ground_Analysis_Console-compatible `ros2:/topic:field` naming style; MAVLink selections use `mavlink:MESSAGE.field` and are graphed from the active MAVLink Pixhawk connection with left/right prefixes. `감시 제거` uses the compact selector above the graph instead of a separate Watch/Latest Value/Action table, and the graph itself stays label-free.

### 고급 > Bias 보정

`Bias 보정` keeps three correction surfaces separate. Origin-BBOX Reprojection Profile JSON is the main bbox correction path: it moves left/right bbox center pixels by learned reprojection residual components, then reruns stereo ray triangulation and records corrected `Right/Down/Forward/Range`, corrected bbox center, `profile_id`, and fallback fields. The default Reprojection Profile application is conservative `component_gains`: `disparity_gain=0.125`, `screen_u_gain=0.0`, and `screen_v_gain=0.025`. This keeps the unstable average screen-position shift from dominating Right/Down while still opening the left/right horizontal separation enough to reduce Forward/Range error. Profile JSON can also set `output_policy=range_only_forward_fusion`; in that mode the app still records corrected bbox centers and rerun triangulation diagnostics, but the final corrected point preserves raw `Right/Down` and uses only corrected `Forward`. This is a lateral non-regression policy, not a claim that Right/Down improved. Reference-style `Cent_Bbox_Adj_Mtrx.csv` remains a global affine baseline candidate and is also triangulated from its corrected bbox centers when loaded. The distance-binned 3D bias JSON is legacy/operator display support; it applies m-offsets after triangulation and is not the final UAM accuracy path. `저장·분석 > UAM 비교` prefers raw stereo-center `Right/Down/Forward` fields when they exist, so a live 3D Bias profile does not silently become part of the final ULog-based comparison. UAM/drone center telemetry is connected only through `연결 > UAM`; the Bias tab shows the latest center-position status and still accepts legacy `drone_x/y/z` values already present in old recorded rows.

### 측정

Use this tab to start and monitor object distance.

- `실시간` contains live WIRIS views, stream receive/YOLO Hz status, distance status, recording controls, and a split Right/Down/Forward/Range graph. Press `세션 기록 시작` before collecting rows that must later be compared with ULog. The Range pane always plots `YOLO Range`, the stereo baseline-center distance estimated from YOLO/stereo. If a UAM alignment profile is loaded and live UAM N/E/D telemetry is available, it also plots `Telemetry Range`, the baseline-center reference distance computed from live telemetry through the profile origin, and stores `telemetry_range_m`/`telemetry_range_error_m` fields in the sample row. All displayed Range/Distance values are baseline-center distances, so UI and new CSV labels omit `baseline`. This is an operational monitor; final accuracy reporting should use `저장·분석 > UAM 비교` with ULog-based matching.
- `계산 설정` contains focal length, baseline, sync gates, class matching, and attitude-gating settings for `Pixhawk 자세 + RTK 위치` 3D ray triangulation. The graph records only successful ray-triangulated samples. Matching keeps recent candidates first, then chooses the smaller timestamp delta for accuracy; `disparity_px` remains a bbox horizontal-delta diagnostic in saved rows, not a distance mode. The `내부 파라미터` block uses a collapsible group, while the external-parameter block stays visible and the two blocks are split left/right around the center. Runtime settings are saved from `저장·분석 > 전체 설정 저장`.
- If an Origin-BBOX Reprojection Profile JSON is loaded in `고급 > Bias 보정`, the measurement path applies it before display/recording: raw bbox center and raw triangulation are kept, corrected bbox center is triangulated again, and the CSV stores `left/right_cx`, `left/right_width_px`, `left/right_height_px`, `left/right_confidence`, `raw_disparity_px`, `raw_range_m`, `left/right_corrected_cx/cy`, `left/right_bbox_du/dv_px`, corrected `Right/Down/Forward/Range`, `profile_output_policy`, and profile fallback fields. Missing cells, out-of-range bins, incomplete bbox pairs, and bad disparity are no-op fallbacks. Profiles built without an explicit correction mode now default to conservative `component_gains`; use `full` only as a diagnostic baseline when validation proves it does not regress Right/Down. If `profile_output_policy=range_only_forward_fusion`, the displayed/saved corrected `Right/Down` remain raw values while corrected `Forward` comes from the reprojection-triangulated point.
- `삼각측량 품질` is a display-only panel for the current 3D ray result. It shows ray gap, distance, baseline, method, raw/corrected 3D point, and a simple left/right ray midpoint sketch; it does not add warning thresholds, automatic decisions, or save-analysis fields.
- `YOLO 측정 시작` restarts camera streams in YOLO/depth mode; while measuring, the same button changes to `YOLO 측정 중지` and returns to camera preview mode while keeping Pixhawk connections active.
- The tab shows left/right camera views, current distance status, sync quality, why a sample is not added to the graph when applicable, and Right/Down/Forward/Distance history over time.
- The live camera status chips show `수신` Hz and YOLO Hz in measurement mode. `수신` Hz counts frames decoded by the stream reader, while YOLO Hz counts frames whose inference completed successfully.
- Recording controls store valid distance samples in memory before saving.

`카메라 Preview 연결` and `YOLO 측정 시작` are intentionally separate. Preview connection verifies stream availability and frame rate without loading YOLO. YOLO/depth mode is started only when depth measurement is needed.

### 저장·분석

Use this tab to review the current recording summary, compare UAM ground truth, inspect recorded samples, and persist data/settings. It is split into `세션 저장`, `UAM 비교`, and `기록 샘플`.

- `세션 저장` contains `세션 파일 저장` and `전체 설정 저장`. `세션 파일 저장` writes the active session under `<STEREO_DISTANCE_APP>/calibration_captures/sessions/<session_id>/`. The session folder contains `session_metadata.json`, optional `distance_history.csv`, optional `external_parameters.csv`, `pixhawk_mag_status.csv`, and the raw left/right MP4 files when video recording was active.
- Session storage follows a fixed metadata/time-series split. `session_metadata.json` stores session-level constants and contracts: scenario name, target class/size, stereo baseline, camera height, zoom/FOV or intrinsics ID, image size, selected correction/profile IDs, validated profile envelope, truth source, time basis, and coordinate/axis contracts. `distance_history.csv` stores timestamped measurement rows: bbox centers/sizes/confidence, raw disparity/range, raw and corrected stereo-center Right/Down/Forward, frame timestamps, active intrinsics/profile IDs, fallback status, and per-row latency/age diagnostics. `external_parameters.csv`, `pixhawk_mag_status.csv`, and `uam_analysis.csv` are lower-rate or post-analysis time-series files and must keep their own timestamps. If a setting can change during a run, record its initial contract in metadata and the active setting/profile/intrinsics identifier on every affected row.
- `기록 샘플` contains `거리 샘플 실시간 보기`. It appends each newly recorded distance sample to a table with the main CSV columns. Selecting a row shows the full CSV field set for that sample, including raw/corrected stereo-center camera Right/Down/Forward, diagnostic left-camera raw point, drone/UAM North/East/Down telemetry point, provisional mixed-frame residual error, applied bias, left/right bbox center fields, bbox UTC fields, and drone sample age when available.
- `pixhawk_mag_status.csv` stores a Pixhawk 1/2 snapshot of `CAL_MAG0` through `CAL_MAG4`, selected MAG, RM3100 status, priorities, arm state, warning text, and session elapsed time so MAG state can be checked against distance and video data.
- `UAM 비교` contains `UAM Ground Truth 비교`, loads a PX4 `.ulg` file, and time-matches recorded YOLO 3D rows with `vehicle_local_position`, `vehicle_odometry`, or `vehicle_visual_odometry` N/E/D samples. PX4 topics still expose these fields as `x/y/z`; the app names them `truth_north_m`, `truth_east_m`, and `truth_down_m`. If local topics are unavailable, the loader can convert `vehicle_global_position`, `vehicle_gps_position`, or `sensor_gps` latitude/longitude/altitude to local N/E/D using the local-position reference, `home_position`, or first valid GPS/global sample fallback origin. When the ULog also contains `vehicle_gps_position.time_utc_usec` or `sensor_gps.time_utc_usec`, the loader maps PX4 local timestamps to UTC and uses `timestamp_sample` before `timestamp` when that field is valid.
- Profile generation should expose the selected truth source in the output
  profile and comparison CSV. A `local_ned` profile, a `global_position` profile,
  and a `gps_position` profile are separate artifacts even if they come from the
  same flight/session rows.
- `ULog 선택/로드` opens a file picker and immediately loads the selected `.ulg`. `현재 경로 다시 로드` reuses the path already typed in the ULog field, which is useful after manually editing the path or replacing the file.
- Recorded YOLO values use the stereo-rig center optical frame and are displayed as `Right`, `Down`, and `Forward`. Legacy CSV fields `x_m/y_m/z_m` remain as aliases for compatibility. The left-camera triangulation midpoint remains available as `raw_left_x/y/z_m` for diagnostics. A target on the camera centerline should therefore have `Right` close to `0`.
- If a ULog is loaded, it is the preferred analysis source. If no ULog is loaded, `UAM 비교 실행` uses the live telemetry buffer from `연결 > UAM`; if that is unavailable, it uses `drone_x/y/z` values already embedded in recorded rows. A single comparison run does not mix ULog/live samples with recorded-row fallback. Rows outside the active truth time range are excluded and reported through the matched/unmatched count.
- `최대 Δt` gates the time match between YOLO row time and ground truth time. `time offset` is added to the YOLO timestamp for post-flight correction before matching. Matching is UTC-first when `bbox_mid_utc_unix_s` and ULog UTC are both available, then falls back to `bbox_mid_timestamp_s`/`timestamp_s`, then to recorded `drone_utc_unix_s` or `drone_timestamp_s` row values.
- `시간 offset 추정` scans candidate offsets against the loaded ULog or live buffer, using UTC basis first when available and timestamp basis otherwise. It applies the best candidate to `time offset` and immediately reruns comparison. Treat the result as a reviewable candidate: confirm the matched count, time basis, timestamp deltas, and error plot before report generation.
- `좌표 보정 자동 추정` estimates a rigid camera-to-PX4-local transform from the currently matched YOLO raw stereo-center Right/Down/Forward rows and UAM North/East/Down truth rows. It requires at least three non-collinear matched samples; move the UAM/drone across several screen positions before using it. The profile stores the rotation matrix and origin N/E/D offset.
- `보정 저장` writes the coordinate-alignment profile as JSON, and `보정 불러오기` applies a saved profile to the current UAM comparison. Use a saved profile from a calibration run for the main experiment when the camera/UAM/Pixhawk geometry has not changed.
- The comparison table shows YOLO Forward/Right/Down, ground truth transformed back to camera-center FRD when a coordinate profile is active, estimated N/E/D, ground truth North/East/Down, frame metadata, source, time basis, and timestamp delta for each matched row. Without a coordinate profile, axis differences remain provisional `mixed-frame` values. With a profile, `오차 그래프` prioritizes camera-frame `Err Forward`, `Err Right`, and `Err Down`; `Err Dist` remains Euclidean position error regardless of whether the truth source is ULog, live telemetry, or recorded-row fallback. `Raw NED` shows the loaded ULog or live UAM ground truth North/East/Down sequence with a compact N/E/D color legend; hovering near a plotted point shows the series, time `t`, and value.
- When analysis rows exist, `세션 파일 저장` also writes `uam_analysis.csv` and records ULog/live sample counts, matched/unmatched counts, and comparison settings in `session_metadata.json`.
- `전체 설정 저장` persists current runtime settings to `<STEREO_DISTANCE_APP>/stereo_distance_settings.json`, including setup camera, WIRIS control, YOLO stream, ROS2, MAVLink, and time-tab FTP/download fields.
- Closing the GUI also saves the current settings automatically.

### Simulation Origin Profile workflow

The upper `Simulation` workspace publishes ROS2 topics directly:

- `/ground_truth_position`: Simulation origin truth in StereoDistance FRD comparison names, `Forward/Right/Down/Range`.
- `/stereo/estimated_position`: YOLO/stereo estimate in the same `Forward/Right/Down/Range` contract.

Use these helper scripts from this app while `<SASL_ROOT>/Simulation/run_operator_view.sh` is already running. The scripts find a sibling `Simulation/` directory by walking upward from the app root; set `SASL_ROOT=/path/to/SASL` or `SIM_DIR=/path/to/Simulation` to override it. The main correction target is now Origin-BBOX Reprojection Profile; the old `origin-bbox` 3D residual model remains a legacy/baseline comparison.

- `scripts/simulation_origin_profile_path.sh` generates a non-collinear waypoint path inside the 8x stereo overlap and flies it through `Simulation/run_px4_waypoint_mission.sh`.
- `scripts/simulation_origin_tools.py build-profile --profile-kind origin-bbox-reprojection` reads telemetry rows, reprojects Simulation truth or camera-frame ULog truth to left/right pixels, learns `duL/dvL/duR/dvR`, and writes a JSON profile plus comparison CSV. This Simulation-origin flow may use Gazebo/Simulation truth as an oracle-style training baseline. For local/global/GPS truth-source comparison, train three separate profiles from the same measurement run using only the selected PX4-derived truth source for each profile; hold Gazebo true pose out as the independent evaluation oracle. Defaults are `screen grid + raw_disparity_px bin + robust median + exact-cell lookup + component_gains(disparity=0.125, screen_u=0.0, screen_v=0.025)`; `--reprojection-correction-mode full`, `--reprojection-correction-mode disparity_only`, `--reprojection-bin-axis range`, `--reprojection-lookup-policy nearest`, and `--profile-output-policy range_only_forward_fusion` are explicit options.
- `scripts/simulation_origin_tools.py evaluate --profile-kind origin-bbox-reprojection` applies the profile without external distance truth by using each row's raw bbox centers and `raw_disparity_px` or `raw_range_m`, then writes corrected bbox centers, rerun stereo position, fallback reason, output policy, and corrected axis/range/position errors.
- `scripts/simulation_origin_evaluate.sh --repeats 5 --random-origin-path --fail-fast-axis-regression --skip-auto-mission` applies the profile to five different random overlap-safe paths. Each generated path records `randomized=true` and its seed. If corrected RMSE is larger than raw RMSE on any Forward/Right/Down axis, the run fails immediately and the profile method must be revised before continuing. For attitude-error validation, generate the profile trajectory from the attitude-aware common FOV and dwell long enough to stabilize each screen/depth cell median; the right-camera `+0.1deg` Roll/Pitch/Yaw validation passed with `profile_waypoint_repeat=5`, `profile_trial_duration_s=180`, and `min_cell_samples=6`. The aggregate report still includes scalar range error and position/range metrics, but scalar range error is not the axis-regression gate.
- `<SASL_ROOT>/Simulation/scripts/bbox_non_center_method_sweep.py` tests non-center bbox point candidates such as top/bottom/edge/corner/cross-edge and learned-origin anchors against the same five-trial gate. `<SASL_ROOT>/Simulation/scripts/bbox_feature_residual_offline_validation.py` tests bbox size/shape/disparity residual candidates. Current 0.1 degree CSV-only results did not produce an all-report RDF strict-improvement candidate; image/silhouette/template/keypoint candidates require recording actual image or ROI data first.

The generated waypoint CSV uses PX4 `north/east/altitude` because that is the mission API, but every recorded comparison/profile/evaluation field uses StereoDistance FRD names: `measurement_frd_forward_m`, `measurement_frd_right_m`, `measurement_frd_down_m`, `truth_frd_forward_m`, `truth_frd_right_m`, `truth_frd_down_m`, `estimated_range_m`, `yolo_range_m`, and `origin_range_m`. Existing Simulation CSVs that only have legacy `estimated_x/y/z` and `drone_x/y/z` are accepted as aliases; `axis_contract=stereo_center_frd:forward_right_down` means legacy `x/y/z` is `forward/right/down`, while missing/older axis contracts are treated as the old RDF order. Reprojection profile training additionally requires left/right raw bbox center fields and benefits from saved bbox width/height/confidence for filtering.

The profile path is constrained by the configured `.env` values: `DRONE_SPAWN_Y_M`, `CAMERA_BASELINE_M`, `CAMERA_HORIZONTAL_FOV_RAD`, and the 15 m target width. Near 320 m the lateral waypoint is kept close to center because the full stereo overlap is narrow; farther points include larger lateral and altitude variation to avoid degenerate one-line calibration. Random validation paths use the same overlap formula, sample range and lateral position with a recorded seed, and keep the target inside the stereo overlap envelope. If `build-profile` reports degenerate samples, the run did not cover enough non-collinear screen positions.

### 시간

Use this tab to capture WIRIS still images over the Ethernet SDK path, download the latest FTP images, and inspect EXIF clock offset.
The default FTP download directory is `<STEREO_DISTANCE_APP>/clock_captures/`; relative paths in this tab are resolved from the `StereoDistance` app root, not from the shell's current working directory.

- `CPTR 촬영 / FTP 다운로드` sends `CPTR` to both WIRIS cameras, checks `ICPT`, downloads the newest JPG/TIFF from each camera's FTP folder, and can run the pair EXIF analysis in one sequence.
- FTP settings use the current left/right WIRIS control hosts for the FTP host addresses; set the FTP user, password, port, remote image directory, and local download folder before running the sequence.
- `파일 쌍 비교` reads one left image and one right image and reports `right-left` capture timestamp offset.
- `폴더 비교` scans left/right folders recursively, pairs images by matching filenames when possible, otherwise by sorted order, and reports per-pair offsets plus median/mean offset.
- The parser reads JPEG/TIFF EXIF `GPSDateStamp`/`GPSTimeStamp` first, then falls back to `DateTimeOriginal` or `DateTime`.
- The table shows capture timestamp source, subsecond availability, GPS coordinates when present, and warnings for second-only timestamps or mixed timestamp sources.
- Manufacturer guidance says WIRIS `camera_captured` timestamps and RTP/RTCP stream timing use the same camera RTC. The `시간` tab therefore checks the left/right WIRIS RTC offset that also applies to stream-derived frame timing.

## Inputs/Outputs and Interfaces

### Camera Inputs

- Left and right WIRIS RTSP URLs.
- Local Host IP and interface for the camera network.
- WIRIS OSD `IP Address`, discovered `Telnet port`, and command templates.

Default RTSP URLs are provided by the app, but field deployment should use `Telnet port 탐색`, `영상 송출 활성화`, `영상 주소 탐색`, and `영상 수신 확인` before connecting when camera network state is unknown.

### YOLO Inputs

- Model weights path.
- `추론 장치` toggle for CPU/CUDA inference.
- Input size, confidence threshold, IoU threshold.
- Processing and preview rate controls.

YOLO runs only after `YOLO 측정 시작`.

### Pixhawk Interfaces

ROS2 path:

- `ROS_DOMAIN_ID`
- Pixhawk 1 namespace
- Pixhawk 2 namespace
- Pixhawk Host IP / uXRCE Agent IP, normally `10.41.10.1`
- `MicroXRCEAgent` UDP port, commonly `8888`
- `/fmu/out/sensor_mag` for RM3100 raw magnetometer samples when the PX4 DDS export includes it
- `/fmu/out/vehicle_magnetometer` for actual selected MAG `device_id` when px4_ros_com publishes it
- Optional `px4_mag_manager_node` status topic `/px4_mag_manager/status` as JSON `std_msgs/String`
- Optional services `/px4_mag_manager/refresh`, `/px4_mag_manager/use_rm3100_for_yaw`, `/px4_mag_manager/save_params`, and `/px4_mag_manager/reboot_px4` as `std_srvs/Trigger`

PX4 parameter path:

- The GUI panel uses the same mockable manager logic as the ROS2 node and starts with the documented Pixhawk 6X Pro/RM3100 field values when no parameter service is attached.
- Run `python3 -m wiris_stereo_distance.px4_mag_manager --ros-args -p backend:=mavros` when MAVROS2 is available and `/mavros/param/get` plus `/mavros/param/set` should back the manager node.
- With px4_ros_com alone, PX4 parameter read/write is not provided by this app; add MAVROS2 parameter services or use QGC/PX4 shell for parameter persistence.

MAVLink path:

- Pixhawk 1 MAVLink URL, commonly `udpin:0.0.0.0:14550`
- Pixhawk 2 MAVLink URL, commonly `udpin:0.0.0.0:14551`
- Optional stream configuration through PX4 shell commands. Pending setup is retried after the first heartbeat so UDP `udpin` links can learn their peer before shell commands are sent.
- Optional `uxrce_dds_client` start command through MAVLink shell. The command includes the selected PX4 namespace.

MAVLink and ROS2 connection paths are separate but can be used together during recovery. MAVLink can reach PX4 shell, start PX4-side clients, configure MAVLink streams, and provide fallback `ATTITUDE`, GPS, GNSS relative, and `SCALED_IMU*` samples while ROS2 is unavailable. ROS2 still requires PX4 topics to be published through XRCE-DDS. Treat ROS2 as actually connected only when `ros2 topic info -v` shows PX4 publishers and `ros2 topic hz` receives samples; a topic name alone can come from this GUI's subscriptions.

MAVLink fallback does not fully replace ROS2/uORB. Actual selected MAG `device_id`, ROS2 namespace identity, arbitrary `/fmu/out/...` custom fields, and structured parameter services still require ROS2/px4_ros_com or MAVROS2/QGC/PX4 shell support.

Use the bundled diagnostic when Pixhawk transport state is unclear:

```bash
./scripts/diagnose_pixhawk_links.py --timeout 3
```

The diagnostic checks host IPs, UDP `8888` / `14550` / `14551`, USB MAVLink candidates, MAVLink heartbeat, ROS2 publisher/sample state, `MicroXRCEAgent` logs, and ARP scan results. `result=OK` requires at least one MAVLink heartbeat and at least one ROS2 PX4 topic with publishers and samples. `result=NOT_CONNECTED` with `topic_names_seen > 0` but `publishers=0` means ROS2 graph topic names exist but no Pixhawk is publishing.

### WIRIS FTP EXIF Clock Inputs

- Left and right WIRIS still image files downloaded over FTP.
- Left and right WIRIS FTP download folders for batch comparison.
- JPEG/JPG and TIFF/TIF files with EXIF timestamp metadata.
- GPS EXIF timestamp tags when GPS time is enabled on the WIRIS, or camera `DateTimeOriginal` as a fallback.

### Outputs

- Live camera preview frames.
- Left/right `수신` Hz and YOLO Hz in the `측정` tab.
- Left/right camera status lines in the same `카메라 상태` group as stream Hz and YOLO Hz in the `측정` tab, showing UTC only when the timestamp basis is marked GNSS-UTC and camera RTC time derived from RTCP Sender Reports otherwise, with source timestamps as fallback.
- Optional recorded CSV fields include GStreamer RTP timestamp, SSRC, RTCP Sender Report based NTP estimates, `camera_rtc_frame_unix_s` aliases, and the frame timestamp reference for later timing analysis.
- YOLO detections and best bounding boxes.
- Current distance estimate.
- Distance history graph with left-click inspection and right-click sample deletion.
- Optional recorded distance CSV.
- Pixhawk attitude comparison graphs.
- RM3100 and GPS sensor graphs.
- Runtime log messages in the `YOLO` and `연결 > Pixhawk` tabs.
- WIRIS SDK command responses in `고급 > WIRIS SDK`.
- WIRIS photo clock-offset table and median/mean folder offset summary.
- Persistent settings JSON.
- Intrinsic calibration JSON.

## Operational Rules

- Start with each side's `Telnet port 탐색` when camera network state is unknown, RTSP may be disabled, or the WIRIS Telnet/SDK ports may not be `2240`. The left button only changes left `IP Address`/`Telnet port`, and the right button only changes right `IP Address`/`Telnet port`.
- Use `WIRIS 자동 연결` first when a monitor is unavailable and the app should try the full WIRIS setup path automatically. It cannot guarantee discovery across unknown IP subnets, but it full-scans candidate hosts and verifies SDK candidates with `HIWS` before using them.
- Use `PC IP 적용` when the selected interface does not already have the displayed PC camera-network address. The camera actions above first replace a missing saved interface after USB-C LAN adapter swaps, consult the active NetworkManager profile on the selected interface, then apply the matching camera-subnet address automatically before discovery, RTSP checks, and preview start. If the profile has no usable camera-subnet address, the GUI falls back to the saved `PC IP` field and logs the exact `sudo` fallback when elevation is unavailable.
- Use `PC IP 탐색` when you want the GUI to inspect the active NetworkManager profile on the selected interface and recover the current camera-subnet address into the `PC IP` field without applying it yet. If the saved interface no longer exists, this also selects the first usable wired adapter as the new camera NIC candidate.
- Use `영상 송출 활성화` after control hosts are found and before `영상 주소 탐색` if the camera RTSP server may be disabled. The app gives the WIRIS a few seconds to switch Ethernet Mode on before checking frames.
- Use `영상 주소 탐색` to fill RTSP URLs after the cameras are reachable on the network.
- Use the `탐색 후보` table when automatic assignment is not the desired left/right mapping. `선택 NIC` applies the selected candidate interface and PC IP; `Left에 적용` / `Right에 적용` applies selected RTSP or WIRIS candidates to that side and copies the discovered SDK port into that side's control-port field when present.
- Keep STIP changes in `IP Address / IP Mask 변경` as a deliberate last step. Normal manual connection should use `카메라망 / NIC`, `탐색 후보`, left/right `IP Address / Telnet port`, `RTSP port / 영상`, then `검증 / 활성화`.
- Start in `연결 > Pixhawk` from `0 공통 준비`, then use `1 탐색 후보 / USB 할당` to discover USB/ROS2 candidates and assign them with `P1에 적용` or `P2에 적용`.
- Use `2 Pixhawk 1/2 사전 설정` to confirm each side's MAVLink URL, baudrate, Ethernet IP, and namespace before `ETH 저장`, then use `3 연결 실행` for `MAVLink 연결` followed by `ROS2 연결`.
- Use `정렬 > MAG > 요약 > 새로고침` before changing magnetometer priority, then confirm RM3100 IDs `8944899` and, when present, `8944387` are shown as RM3100 devices.
- Do not change MAG parameters while PX4 is armed. The MAG manager disables or rejects `RM3100을 Yaw 기준으로`, save, and reboot actions when armed state is known.
- Use `영상 수신 확인` before `카메라 Preview 연결`; require `frame-ok`, not only port-open.
- Keep `GStreamer` as the default stream backend and `NVIDIA` as the default decoder for low-latency operation. Try `OpenCV Auto` when the explicit GStreamer or OpenCV/FFmpeg paths fail on a PC. Use `PyAV` when the `av` package is installed in the active Python environment.
- Use `카메라 Preview 연결` to verify both camera streams and Hz before starting YOLO.
- Use `YOLO 측정 시작` only when depth measurement is needed.
- YOLO runs on the latest decoded frame at the configured/adaptive inference rate, while the stream reader continues draining decoded frames independently.
- During YOLO mode, detection results are produced every inference frame, but preview annotation is drawn only on frames that are actually sent to the GUI and only for the best detection box.
- The camera status reports stream `수신` Hz separately from display Hz in preview mode and shows `수신` Hz plus YOLO Hz in measurement mode. Measurement-mode `수신` Hz is counted in the reader thread before YOLO inference, so slow inference should lower YOLO Hz without forcing receive Hz to the same value.
- Each camera stream runs decode/YOLO work in a child process so one overloaded stream is less likely to block the other stream or the Qt UI process.
- Stereo bbox matching is separate from the old `focal * baseline / disparity` distance formula. The GUI distance path now keeps only the shared left/right detection-pair validation used by 3D ray triangulation.
- Keep `Preview W` at the stream's native width when you want the GUI to see the encoded frame without downscaling. For the current WIRIS visible stream, `1920` preserves the 1080p frame. Detection still uses the original frame, not the preview image.
- Use `CUDA 추론` when a CUDA GPU is available.
- Use `CPU 추론` only when GPU inference is unavailable or intentionally disabled. If CUDA prediction fails at runtime, the worker retries on CPU so preview and YOLO can continue when possible.
- Use the live GPS baseline only when GPS quality and sample age are acceptable.
- Use calibrated intrinsics for real measurements. Default focal and baseline values are placeholders for setup/testing.
- If checkerboard calibration is unavailable, enable `수동 초점거리 사용` and enter the estimated `fx px` for the current WIRIS visible zoom ratio from the tooltip. Treat this as a temporary estimate, not a replacement for per-zoom calibration.
- Use the connected-state `카메라 Preview 해제`, `ROS2 해제`, and `MAVLink 해제` button states independently when isolating connection issues.
- Use `전체 설정 저장` or close the GUI normally after changing setup or Pixhawk connection values that should be restored on the next launch.
- Use the SDK command controls in `고급 > WIRIS SDK` for camera SDK commands that are not represented by first-class buttons. Confirm the target camera and command text before sending IP, port, reboot, activation, or camera-side recording commands.
- Use `시간` after downloading still images over FTP; do not use FTP transfer time or PC file modification time as the capture timestamp.
- For clock-offset checks, prefer WIRIS images that include GPS EXIF time from the same GPS time base on both cameras.
- For folder comparisons, use matching filenames when possible. If names differ, review the sorted-order pairing before trusting median/mean offset.

## Known Limitations and Workarounds

- Camera RTSP decoding defaults to `NVIDIA` and can also use `Auto` or `Software`. NVIDIA mode requires a working NVIDIA driver and GStreamer `nvh264dec`; if `nvidia-smi` cannot communicate with the driver, use `Auto` or `Software`.
- Software H.264 decoding skips B-frames to reduce CPU pressure in live mode.
- `영상 수신 확인` distinguishes `port-closed`, `open-no-frame`, and `frame-ok`; `open-no-frame` usually means the RTSP server is listening but the media path or encoder is not delivering frames.
- `Telnet port 탐색` and `영상 주소 탐색` can only TCP-scan IP ranges that are currently routed through a local interface; add the camera subnet to the selected interface first when cameras use a different static IP range.
- The full WIRIS control-port scan runs on current RTSP/control host hints, default WIRIS hosts, ARP-scan hosts, and IP-neighbor hosts. If both the camera IP and SDK port are unknown and the camera is not visible through those host sources, configure the PC camera-network IP or confirm the address from HDMI before scanning again.
- The GUI keeps separate left/right WIRIS SDK control ports. If automatic discovery cannot prove the intended left/right identity, apply candidates manually or operate one camera at a time before sending camera-state-changing commands.
- `PC IP 적용` may require system authorization because Linux needs elevated privileges to add an address to a network interface.
- `PC IP 탐색` does not change the live interface address or the saved NetworkManager connection. It may update the GUI `인터페이스` field when the previously saved USB-C LAN adapter name no longer exists.
- The active NetworkManager connection on the selected interface is the source of truth for the automatic camera-network IP recovery path. If that profile changes, the GUI follows it; if it does not contain a camera-subnet address, the GUI falls back to the displayed `PC IP` value on the selected wired adapter.
- `Preview W` reduces GUI copy/scale cost, but it does not eliminate all decode cost.
- YOLO throughput is intentionally uncapped, so a weak CPU/GPU can still become the bottleneck during depth mode.
- During measurement, `검출 없음` frames do not clear the previous valid left/right detections; sparse detections can still form a timestamp-matched distance pair within the configured `최대 Δt`.
- Distance rows are recorded only after 3D ray triangulation succeeds. Use `Pixhawk 자세 + RTK 위치` with calibrated intrinsics and RTK translation.
- Stream processing is process-isolated per camera, but total CPU/GPU, memory bandwidth, and RTSP network capacity can still bottleneck the system.
- `Auto` decoder mode falls back from NVIDIA to software when the NVIDIA decoder cannot be created. It cannot fix missing frames from the RTSP source itself.
- The RTSP pipeline drops stale buffered frames when it falls behind. This favors current frames over complete historical frame delivery, which is better for live distance measurement.
- The `PyAV` stream backend depends on the `av` package. If it is missing from a Python environment, the app reports `PyAV 사용 불가` and other backends continue to work.
- If YOLO dependencies or model loading fail, the measurement stream falls back to camera preview only and reports the failure in the YOLO/camera log. Distance calculation resumes only after YOLO inference produces detections.
- The app treats Pixhawk attitude as synchronization/geometry input, not as a replacement for camera extrinsic calibration.
- Actual selected MAG can only be shown when px4_ros_com publishes `vehicle_magnetometer.device_id`; MAVROS2 `/mavros/imu/mag` usually does not expose that ID, so verify with `listener vehicle_magnetometer` when the UI reports it unavailable.
- The MAG manager node can use MAVROS2 parameter get/set services, but parameter save and PX4 reboot remain documented operator actions unless the deployment wires a confirmed command service.
- Pixhawk-based 3D triangulation uses a fixed camera mount contract, not a UI-selectable preset: world frame `NED`, Pixhawk body frame `PX4_FRD`, camera frame `OpenCV optical`, with camera `+Z -> body +X`, camera `+X -> body +Y`, and camera `+Y -> body +Z`.
- GPS/RTK antenna baseline assumes the Pixhawk/GPS baseline is a usable proxy for camera baseline. Mount offsets still matter.
- Live triangulation uses loaded camera matrices and RTK/Pixhawk geometry; full field accuracy still depends on calibration, sync, baseline quality, and mounting repeatability.
- EXIF clock-offset checks are limited by the timestamp fields present in the saved image. If WIRIS stores only whole seconds and no subsecond tag, subsecond camera skew cannot be proven from metadata alone.
- EXIF `DateTimeOriginal` is timezone-less on many cameras. GPS EXIF time is preferred because it is UTC; mixed GPS and camera-local timestamps are reported as a warning.
- If OpenCV cannot import in the active Python environment, image capture can still work, but intrinsic calibration must wait until the OpenCV/NumPy stack is repaired.

### Calibration Dependency Bootstrap

If OpenCV calibration dependencies need repair:

```bash
cd <STEREO_DISTANCE_APP>
PYTHON_BIN=/home/moontaehyung/Capstone/.venv/bin/python ./scripts/bootstrap_calibration_deps.sh
```

Or run the bootstrap before opening the GUI:

```bash
STEREO_BOOTSTRAP_CALIBRATION_DEPS=1 ./run.sh
```

The bootstrap installs `requirements/calibration.txt` and verifies `import cv2`.

## Change Log

### 2026-06-09

- Moved low-level/manual tools to `고급`: `WIRIS SDK`, `필드 감시`, and `Bias 보정`.
- Kept `연결` to the primary `WIRIS`, `Pixhawk`, and `UAM` connection workflow, and kept `측정` to live measurement, calculation settings, and triangulation quality.
- Marked `연결 > UAM` as the standard ground-truth telemetry entry point for the central UAM/drone GPS antenna and Pixhawk body-origin N/E/D assumption.
- Removed duplicate local settings-save buttons from the YOLO and measurement tabs; use `저장·분석 > 전체 설정 저장`.
- Removed legacy UAM center watch registration from `고급 > Bias 보정`; use `연결 > UAM` for live ground-truth telemetry.
- Split ULog actions into `ULog 선택/로드` and `현재 경로 다시 로드`.
- Added UAM coordinate-alignment profile estimation, JSON save/load, and application in `저장·분석 > UAM 비교` so final analysis can report estimated N/E/D and position error after origin correction.
- Added a compact legend and hover tooltip to telemetry graphs, including `Raw NED`, so operators can identify series color and inspect point time/value.

### 2026-06-02

- Added `연결 > UAM` and `저장·분석 > UAM Ground Truth 비교` so ULog-first post-flight analysis can fall back to live ROS2/MAVLink UAM telemetry or embedded recorded drone rows.
- Added a PX4 Magnetometer / RM3100 manager panel under `정렬 > MAG > 요약`, plus a mockable `px4_mag_manager_node` that publishes `/px4_mag_manager/status` and exposes refresh/use/save/reboot Trigger services.
- Added RM3100 priority logic and tests for the current Pixhawk 6X Pro values and the two-RM3100 backup case.
- Kept measurement-tab camera preview alive when YOLO model loading or per-frame inference fails, including a CUDA prediction failure path that retries on CPU.
- Preserved recent valid detections across intermittent `검출 없음` frames so sparse left/right detections can still match and produce distance estimates.
- Removed the GUI's simple Disparity distance mode; bbox horizontal delta remains available as the `disparity_px` diagnostic field.
- Decoupled stream readers from YOLO inference so measurement-mode `수신` Hz counts decoded reader frames while YOLO Hz counts successful inference frames.
- Reordered tabs for the field workflow: `연결`, `시간`, `보정`, `파라미터`, `정렬`, `YOLO`, `측정`, `저장·분석`, `고급`; moved `WIRIS SDK` after `Pixhawk` inside `연결`.
- Split stereo detection-pair validation from the old simple disparity distance formula so 3D triangulation does not depend on that calculation path.
- Applied the fixed front-facing camera mount frame contract to Pixhawk-based RTK translation and ray rotation, and record `world_frame`, `body_frame`, `camera_frame`, and `camera_mount_profile` in distance CSV rows.
- Split WIRIS SDK controls into the separate `연결 > WIRIS SDK` subtab.

### 2026-05-17

- Changed WIRIS control discovery from a single configured-port probe to candidate-host TCP `1-65535` scanning with `HIWS` verification.
- `Telnet port 탐색` and `WIRIS 자동 연결` now update each side's discovered SDK/Telnet port before running control checks or RTSP enable commands.
- Changed WIRIS SDK command preset descriptions to Korean and enabled wrapped description text/tooltips.
- Removed the global operation status chip/header and next-action button from the top of the window.
- Unified start/stop style button pairs for camera preview, YOLO measurement, ROS2, and MAVLink into single state-changing buttons.
- Compressed the `인터페이스`, `PC IP`, `포트`, `타임아웃`, `IP Mask`, and `STIP 후 재부팅` camera network settings into one compact top row.
- Replaced the free-form YOLO `추론 장치` input with a CPU/CUDA toggle that stores `cpu` or `cuda:0`.
- Moved the `R/T 삼각측량 사용` toggle from `보정` into `측정 > 계산 설정`.
- Reorganized tabs for the QGC-first field workflow: `연결`, `시간`, `보정`, `파라미터`, `정렬`, `YOLO`, `측정`, `저장·분석`, and `고급`.
- Added WIRIS visible zoom ratio estimated `fx px` tooltips and widened manual focal-length input so high zoom ratios can be entered without checkerboard calibration.
- Added `OpenCV Auto` and optional `PyAV` choices to the YOLO `수신 백엔드` selector.

### 2026-05-19

- Changed `영상 주소 탐색` so a single open RTSP candidate updates only its matching camera side instead of copying that URL to both sides.
- Restored the per-camera `Apply STIP` controls so the IP update action stays separated from the RTSP URL display.
- Moved the measurement-tab frame timestamps into the same `카메라 상태` group as receive Hz and YOLO Hz so the live camera status reads as one line per camera.
- Added left-click inspection and right-click deletion to the distance history graph so individual samples can be reviewed or removed directly from the measurement tab.
- Added automatic camera-network IP recovery from the active NetworkManager connection on the selected interface so the GUI can reapply the current camera-subnet address after reboot or link reactivation even when the field value was changed outside the app.
- Added a `PC IP 탐색` button that reads the active NetworkManager profile and fills the camera-network IP field without applying it.
- Added automatic camera NIC replacement for USB-C LAN adapter swaps: when the saved interface no longer exists, `PC IP 탐색`, `영상 주소 탐색`, `Telnet port 탐색`, `WIRIS 자동 연결`, and RTSP enable choose an available wired interface and apply the displayed camera-network IP before scanning.
- Added the `탐색 후보` table so automatic NIC, RTSP, and WIRIS control discovery results can also be manually applied to NIC, Left, or Right fields.
- Added the `Pixhawk 탐색 후보` table so ROS2 namespace and USB MAVLink discovery results can also be manually applied to Pixhawk 1 or Pixhawk 2.
- Split WIRIS SDK/Telnet ports into left/right fields so cameras can use different ports, and changed per-side `Telnet port 탐색` to update only the selected side.
- Changed the WIRIS visible zoom tooltip basis from 1920x1080 to 1280x720 so the nominal focal-length estimates match the HD stream configuration.
- Refined the measurement-tab camera status display to keep receive Hz, YOLO Hz, and frame time on one line per camera, showing UTC only for GNSS-UTC-synced frames and RTCP/NTP otherwise while keeping source timestamps as fallback and exposing the frame timestamp reference in recordings.
- Compacted the real-time header so the action buttons and camera status share one horizontal strip on wide windows, and fold vertically on compact windows to avoid overlap while leaving more vertical space for the camera views.
- Added a `계산 설정 저장` button inside the measurement `계산 설정` group so the active stereo calculation settings can be saved without leaving the measurement tab.

### 2026-05-18

- Requested MAVLink `SCALED_IMU*` sensor streams automatically on heartbeat so the `정렬 > MAG` panels can show CAN1/CAN2 magnetometer values even when ROS2 magnetometer topics are unavailable.
- Added `--start-pixhawk` / `--connect-pixhawk` startup flags to open MAVLink and show the RM3100 alignment view immediately during field checks.
- Added GStreamer RTP timestamp and RTCP Sender Report based NTP estimate CSV fields for left/right stream timing diagnostics.
- Removed the measurement-tab live `t`, `age`, `wall`, `src`, display-delta, `srcPTS`, and RTP/RTCP timing readouts so routine monitoring focuses on receive Hz and YOLO Hz.
- Added YOLO Hz next to receive Hz in the measurement-tab live camera status chips.
- Made NVIDIA the default RTSP decoder and reduced YOLO preview CPU cost by drawing only the best detection box on frames that are actually displayed.
- Changed WIRIS SDK command defaults to LINE termination and delayed RTSP frame checks after Ethernet Mode activation so cameras have time to open port 8554.

### 2026-05-16

- Added the `시계 동기화` tab for comparing WIRIS FTP-downloaded still image EXIF timestamps.
- Added JPEG/TIFF EXIF parsing for GPS timestamp, camera timestamp, subsecond fields, and GPS coordinates without adding a runtime dependency.
- Added single-pair and folder-batch clock offset summaries using `right-left` seconds, median, and mean values.
- Added latest left/right displayed frame timestamp readouts, `프레임 Δ`, and optional GStreamer `srcPTS` readouts to the `측정` tab.
- Added displayed/source frame timestamp fields to distance CSV recording rows.

### 2026-05-11

- Reorganized the GUI around field scenarios: `준비`, `보정`, `측정`, `결과`, `시계 동기화`, and `진단`.
- Moved detailed Pixhawk, RTK, camera parameter, and stereo configuration views under the `진단` tab.
- Added a session summary to the `결과` tab.
- Collapsed advanced stream/YOLO settings and camera diagnostics by default to keep routine field operation focused.
- Added a grouped WIRIS Ethernet Stream SDK command selector with editable command text, delimiter selection, left/right/both target selection, response logging, and confirmation for disruptive commands.
- Added `WIRIS 자동 연결`, a monitorless setup button that discovers SDK control candidates, verifies `HIWS`, enables Ethernet/RTSP mode, checks RTSP frames, and starts camera preview when both streams are valid.

### 2026-04-28

- Reorganized the app README around the current tab workflow.
- Documented separate camera preview and YOLO/depth measurement behavior.
- Documented ROS2 and MAVLink connection separation.
- Documented Pixhawk attitude comparison, GPS baseline observation, RM3100/GPS views, and PX4 shell access.
- Removed the `Preview Hz` cap so preview and YOLO both run as fast as the pipeline can sustain; `Preview W` remains as the GUI load control.
- Hid Camera Setup previews during YOLO depth mode because calibration and depth measurement are separate workflows.
- Strengthened `영상 수신 확인` so it validates first-frame reception instead of only checking whether port 8554 is open.
- Split Auto Detect into separate camera and Pixhawk discovery actions.
- Added RTSP decoder selection with NVIDIA H.264 decode support and automatic software fallback.
- Reduced camera preview latency by dropping stale RTSP/GStreamer buffers and decoupling receive/YOLO Hz from Qt preview rendering Hz.
- Made Camera Auto Detect continue scanning the camera subnet for additional RTSP candidates when default right-camera discovery fails.
- Split camera discovery into `Telnet port 탐색` for WIRIS control TCP hosts and `영상 주소 탐색` for RTSP URL discovery.
- Persisted setup camera, WIRIS control, YOLO stream, ROS2, and MAVLink fields across GUI restarts.
- Added `PC IP 적용` to apply the displayed camera-network host address to the selected local interface before TCP discovery.
- Removed outdated documentation for the old nested WIRIS/Pixhawk tab layout and removed references to the deleted Pixhawk bias workflow.
