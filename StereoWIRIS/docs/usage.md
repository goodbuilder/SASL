# StereoWIRIS

## 먼저 읽기

StereoWIRIS는 좌/우 WIRIS RTSP 카메라, YOLO 검출, Pixhawk 자세/RTK 데이터를 함께 사용해 현장에서 객체 거리와 3D 위치를 확인하는 PyQt5 운용 도구입니다.

빠른 실행:

```bash
cd <STEREO_WIRIS_APP>
./run.sh
```

자주 쓰는 실행 옵션:

```bash
./run.sh --maximized      # 최대화 실행
./run.sh --start          # YOLO/Depth 스트림 자동 시작
./run.sh --wiris-only     # WIRIS 스테레오 창만 실행
./run.sh --ui workstation  # Workstation shell 실행; 생략하면 legacy UI
```

문서 역할:

- `docs/usage.md`: 실행 방법, 운용 순서, 상세 사용법.
- `PROJECT.md`: 프로젝트 목적, 시스템 구성, 핵심 워크플로, 개발 방향.
- `COORDINATE_CONTRACT.md`: 좌표계, 축 이름과 Raw CSV 필드 계약.
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
7. Raw `baseline_center_frd` 기록, 설정과 로그를 저장한다.

아래 내용은 기존 상세 운용 매뉴얼입니다.

## Purpose

StereoWIRIS is a PyQt5 field GUI for measuring object distance from two WIRIS RTSP camera streams while monitoring two already-calibrated Pixhawk systems. QGC is the expected tool for Pixhawk calibration before this app is used; this app separates field connection, WIRIS time verification, WIRIS intrinsic calibration, RTK/RM3100/attitude alignment, YOLO/depth measurement, and saving/analysis into a step-by-step workflow.

The app is designed for this operating sequence:

1. Calibrate Pixhawks in QGC before launching this app.
2. Connect WIRIS cameras, Pixhawk MAVLink/ROS2, and confirm stream reception.
3. Verify WIRIS photo timestamp offset from externally triggered still images.
4. Capture checkerboard images for either camera and compute WIRIS intrinsic parameters.
5. Align GPS baseline, compare RM3100 CAN1/CAN2 data, and align camera attitude.
6. Configure YOLO.
7. Start session recording, then start YOLO/depth measurement.
8. Save the Raw `baseline_center_frd` distance history, settings, and review logs.

## Current Run Flow

Launch through `run.sh` so ROS2 and the local PX4 message workspace are sourced before the GUI starts.
The launcher also tries to install `arp-scan` when it can do so without an interactive sudo password; if that is not possible, the GUI still starts and Auto Detect falls back to TCP scans.

```bash
cd <STEREO_WIRIS_APP>
./run.sh
```

Useful options:

```bash
./run.sh --maximized
./run.sh --start        # auto-start YOLO/depth streams
./run.sh --wiris-only   # open the standalone WIRIS stereo window
./run.sh --ui workstation  # launch the Workstation shell; default is legacy
```

`--ui legacy`와 `--ui workstation`은 한 프로세스에서 하나의 operational UI만
선택한다. Legacy UI 구현은 `legacy_gui.py`에 있고 기존 `gui.py`는 호환 alias로 남아 있으며,
Workstation은
저장소 루트의 `workstation/standalone_gui.py`/`StandalonePanelBackend` 경로를 사용한다. lifecycle은
UI-neutral `sasl_common/lifecycle.py`를 공유하지만 panel/resource owner는 분리되어 있다. 두 UI는 서로의
panel owner를 import하거나 instantiate하지 않는다. Workstation 복사본은 현재 공유
measurement/camera/telemetry/calibration/recording 모듈과 저장·좌표 계약을 그대로 사용하며,
Overview·Camera·Time/Sync·Measurement·Alignment·Calibration·Recording/Review와
Inspector·Diagnostics를 새 shell에서 제공한다. Camera IP/RTSP/control, Pixhawk discovery/MAVLink,
GNSS Base serial/Survey-in/NTRIP은 Workstation의 1단계 command facade에서 기존 owner를
호출한다. Calibration capture/run/save/load/apply/validation과 calibration setup fields,
measurement/synchronization settings, alignment source scan/baseline sampling/profile
save/load도 owner-backed facade로 호출한다. 세션 목록·manifest 오류·CSV preview·세션/산출물
경로 열기도 Workstation에서 제공한다. 고급 owner tools 화면에서 실시간 checkerboard
preview/zoom, raw WIRIS SDK 명령, PX4 MAG·attitude·Shell, ROS2/MAVLink field-watch와
baseline field browser, 세션 CSV preview·XLSX/Parquet/video artifact 열기를 직접 제공한다.
video는 별도 Workstation decoder를 만들지 않고 시스템 연결 player로 열며, workbook 생성은
기존 session save writer가 담당한다. 두 UI를 동시에 실행해도 서로 다른 panel/backend
owner가 생성되며, 같은 프로세스 안에서 같은 camera queue나 Pixhawk subscriber를 이중
소비하지 않는다. 원본과 복사본의 UI 코드는 자동 동기화되지 않으므로 기능 변경 시 두
경로를 의도적으로 비교하고 테스트한다.
Workstation의 Measurement 화면은 기존 `CameraPane.frame_changed` 신호를 프록시한 좌/우
영상과 함께 측정 결과/품질, detection, RTCP/NTP 동기화, GNSS/RTK 및 NED baseline,
recording, decoded FPS를 한 화면에서 보여준다. 화면 상단 Run control에서 Preview →
Detection → Record → Save session 순서로 실행하며, 녹화 중 Preview 중지·카메라 매핑
교체·세션 저장은 비활성화된다. `Current measurement quality`는 기존 panel의 ray gap,
midpoint Range, method, attitude timing, Raw point를 복사해 표시한다. 공개 측정값은 계속
`baseline_center_frd`의 `forward/right/down` 미터 계약을 따른다.
`Tasks > All settings` 화면에서는 Legacy에만 남아 있던 Camera/WIRIS, YOLO/RTSP,
PX4/ROS2, runtime flag, Clock/FTP와 전체 GNSS/RTK nested operation 설정을 편집할 수
있다. `Apply all settings`는 기존 panel owner의 위젯/운용 설정에 반영하고,
`Save all settings`는 Workstation 전용 `workstation_data/stereo_distance_settings.json`에,
Legacy는 `StereoWIRIS/stereo_distance_settings.json`에 기존 schema로 저장한다. 두 JSON은
서로 읽지 않는다. GNSS credential은 기존처럼 비밀번호 값이 아니라 환경변수 이름만
설정하며, 좌표 입력은 WGS84이고 측정 출력/기존 저장 schema의 NED·camera RDF·FRD
계약은 변경하지 않는다. 메인 Workstation 카드에는 반복적인 `source=`를 표시하지
않고, 값의 provenance는 Inspector와 Diagnostics의 Source/Evidence 영역에서 확인한다.
Inspector의 provenance는 `source=` 접두어 없이 표시하며, 내부 source 필드와 저장 계약은 유지한다.
Alignment 화면은 PX4 roll/pitch/yaw와 PX4/GPS timestamp/age, GPS fix/satellites/eph/epv,
명시적인 NED baseline을 표시한다. Calibration 화면은 active profile/zoom, camera matrix
pixel metrics, stereo R/T frame contract를 표시한다. Recording/Review/Export 화면은
session ID, elapsed/finalization/retry, sample counts, video paths와 저장 artifact를 기존
session owner에서 읽어 표시하고, CSV는 저장된 열·`recording_time_ns`·좌표 필드를 변환 없이
미리 본다. 저장 포맷과 AppSettings schema는 유지하되, Workstation은 `workstation_data/`, Legacy는
`StereoWIRIS/`를 각각의 runtime data root로 사용한다.
Detection 설정과 target bbox는 기존 AppSettings/DetectionObservation을 읽고, bbox는
이미지 pixel `u/v`로만 표시한다.
1366×768 화면에서는 Workstation workspace와 Inspector가 세로 스크롤을 사용해
좌표 계약·진단·고급 handoff 내용을 잘라내지 않고 탐색한다.

Workstation field sequence:

1. `Devices > Cameras`: IP 탐색 → WIRIS control 확인 → RTSP 활성화 → RTSP frame 확인.
2. `Devices > Time / Sync`: PC UTC 설정 → RTCP/NTP 확인 → still-image clock workflow.
3. `Devices > Pixhawk`: ROS2 namespace/MAVLink 상태와 P1/P2 mapping을 확인한다.
4. `Devices > GNSS / RTK`: Base serial, Survey-in/NTRIP, GPS fix와 NED baseline을 확인한다.
5. `Tasks > Calibration`: setup 적용 → preview/촬영 → intrinsic → validation 순서로 수행한다.
6. `Processing > Measurement`: Preflight evidence와 Current measurement quality를 확인한 뒤
   Run control에서 Preview, Detection, Record를 실행하고 정지 후 Save session을 누른다.

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

The `연결` tab contains `WIRIS` and `Pixhawk`. It keeps only the primary connection workflow.
The `시간` tab contains WIRIS RTC setting, live RTCP/NTP stream synchronization checks, and still-image EXIF timestamp comparison.
The `보정` tab contains a `체커보드` subtab for checkerboard capture/preview/zoom control, a `체커보드 검증/계산` subtab for file-set based calibration and random stability validation, and a `줌 내부 파라미터` subtab for per-zoom intrinsics with save/apply controls.
The `파라미터` tab contains `내부`, `외부/기하`, and `계산 설정` subtabs so intrinsics, live geometry state, and calculation controls can be inspected separately.
The `정렬` tab contains `센서 인식`, `소스 / 품질`, `GNSS Base`, `MAG 관리`, `Baseline 계산`, `Baseline / 자세 정렬`, and `Optical 정렬`.
The `고급` tab contains `WIRIS SDK`, `필드 감시`, `Baseline 필드`, and `PX4 MAG` advanced diagnostics.
The `측정` tab contains `실시간` and `측정 품질`; calculation controls live under `파라미터 > 계산 설정`.
The `저장·분석` tab contains `세션 저장` and `기록 샘플`.

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
- `RTP 지연(ms)`: GStreamer RTP jitter-buffer latency in milliseconds.
- `디코더`: RTSP H.264 decoder mode. `NVIDIA` is the default and forces `nvh264dec`; `Auto` tries NVIDIA first and falls back to software; `Software` forces `avdec_h264`.
- `수신 백엔드`: RTSP receive backend. `GStreamer` gives the best latency/drop/decoder control; `OpenCV/FFmpeg` is a simpler fallback path; `OpenCV Auto` lets OpenCV choose the available capture backend; `PyAV` uses the optional `av` package and reports `pyav-pts` when frame PTS is available.
- `RTSP 전송`: RTSP media transport. `UDP` is the default for live streams because it avoids TCP receive-queue buildup when decoding falls behind; `TCP` remains available for networks where UDP is blocked or too lossy.
- `추론 장치`: toggle for Ultralytics/PyTorch inference device. `CPU 추론` passes `cpu`; `CUDA 추론` passes `cuda:0`.

### 연결 > Pixhawk

This connection subtab assumes Pixhawk calibration was already completed in QGC and focuses on MAVLink-first Rover control plus ROS2/uXRCE telemetry setup.

- `Pixhawk 자동 탐색` scans ROS2 PX4 topics using the current `ROS_DOMAIN_ID` and fills Pixhawk namespaces.
- The Pixhawk subtab is ordered as `0 공통 네트워크`, `1 연결 후보`, `2 Rover MAVLink / RTCM`, `3 ROS2 Telemetry`, and `4 고급 / 진단` so the shared host IP/P1-P2 mapping, MAVLink URL/USB control, RTCM-capable heartbeat, ROS2 telemetry, and shell diagnostics stay in the field order.
- `0 공통 네트워크` contains the shared `Pixhawk Host IP` and `P1/P2 매핑 교체`. The mapping swap is kept here because it exchanges both MAVLink and ROS2 side assignments.
- `1 연결 후보` stacks three candidate areas vertically: `PC / Host IP`, `MAVLink / USB 제어`, and `ROS2 Namespace`. Each area has its own double-height full-width table and only exposes the apply buttons that match that candidate type.
- Pixhawk `PC IP 탐색` appends all local IPv4 candidates to the `PC / Host IP` table as `PC_IP` rows and does not change `Pixhawk Host IP` by itself. Select the intended wired Pixhawk/uXRCE Agent IP and press `Host에 적용` explicitly when both wired Ethernet and Wi-Fi are active.
- `MAVLink / USB 제어` contains Ethernet MAVLink URL candidates, USB control-port candidates, and USB serial direct MAVLink candidates. Apply these with `P1에 적용` or `P2에 적용`.
- `ROS2 Namespace` contains ROS2 namespace candidates from `Pixhawk 자동 탐색`. Apply these with `P1 적용` or `P2 적용`.
- `2 Rover MAVLink / RTCM` keeps MAVLink-only settings together: MAVLink stream rate, MAVLink stream setup toggle, each side's Ethernet MAVLink URL, MAVLink baudrate, USB control/bootstrap port, and Ethernet IP. Its action row contains P1/P2 MAVLink connection and P1/P2 Ethernet MAVLink start over USB.
- `3 ROS2 Telemetry` keeps ROS2/uXRCE settings together: `ROS_DOMAIN_ID`, uXRCE port, PX4 uXRCE Agent setup toggle, and P1/P2 ROS2 namespaces. It then provides P1/P2 ROS2 connection and P1/P2 uXRCE parameter setup. The uXRCE setup buttons are placed here because they serve ROS2 recovery, but they still require an active MAVLink heartbeat.
- `4 고급 / 진단` contains PX4 Shell, `ETH 저장`, and MAVLink reboot actions. Shell is diagnostic only; use `P1/P2 재부팅` for reboot when Shell/serial-control does not respond.
- `MAVLink URL 후보` adds common Ethernet UDP listen candidates `udpin:0.0.0.0:14550` and `udpin:0.0.0.0:14551`, plus P1/P2 Ethernet `tcp:<Pixhawk ETH IP>:5760` and `udpout:<Pixhawk ETH IP>:<port>` candidates when Pixhawk ETH IP fields are filled. Apply one to P1/P2, then press `MAVLink 연결`.
- `USB 연결 탐색` scans `/dev/serial/by-id/*`, `/dev/ttyACM*`, and `/dev/ttyUSB*` using common MAVLink baudrates including `57600`, `115200`, and `921600`. A USB row applies to `USB 제어 포트` for Ethernet bootstrap; a `MAVLink` row from the same device applies the serial device directly to the side's MAVLink URL and baudrate.
- `MAVLink 연결` opens direct MAVLink links; while connected, the same button changes to `MAVLink 해제`. Rover RTCM injection and PX4 control require MAVLink heartbeat, not only a UDP socket.
- `P1/P2 ETH MAV 시작(USB)` uses the configured USB control/bootstrap port to send PX4 shell commands that bring up `eth0` and start the Ethernet MAVLink stream toward the current Pixhawk Host IP, then reconnects the app's UDP MAVLink listener. If the saved USB baud is wrong, it tries the fast MAVLink baud candidates `57600`, `115200`, and `921600`; if PX4 rejects `mavlink start -m normal`, it retries the bootstrap stream with `-m onboard -n eth0`.
- `ROS2 연결` starts or reuses `MicroXRCEAgent`, then subscribes to available PX4 ROS2 topics; while connected, the same button changes to `ROS2 해제`. ROS2 is the structured telemetry path and does not replace MAVLink RTCM/control.
- If ROS2 is connected but `sensor_gps` or `sensor_mag` data is not published, the GUI starts the configured MAVLink URL(s) as an auxiliary sensor receiver after startup. This keeps ROS2 attitude/GPS1 data active while using MAVLink `GPS2_RAW` for the GPS2 slot and `SCALED_IMU*` for RM3100 CAN1/CAN2 slots.
- `픽스호크 Host IP` should be the PC address reachable from the Pixhawk Ethernet/uXRCE network. The field default is `10.0.0.10`; Pixhawk 1/2 Ethernet defaults are `10.0.0.2` and `10.0.0.3`.
- If `ROS2 연결` fails during session startup, the GUI can use the already configured MAVLink side(s) to send MAVLink stream setup and PX4 `UXRCE_DDS_*` parameters with MAVLink `PARAM_SET` when `PX4 uXRCE Agent 설정` is enabled, verifies them by readback, keeps MAVLink telemetry active, and retries ROS2.
- After each MAVLink heartbeat, the app requests `ATTITUDE`, `GPS_RAW_INT`, `GPS2_RAW`, `SCALED_IMU`, `SCALED_IMU2`, and `SCALED_IMU3` with `MAV_CMD_SET_MESSAGE_INTERVAL`. `GPS_RAW_INT` fills the GPS1 position path; `GPS2_RAW` fills the GPS2 sensor slot and a `left_GPS2`/`right_GPS2` raw LLA position path for Baseline GPS2 selection; `SCALED_IMU` and `SCALED_IMU2` fill RM3100 CAN1/CAN2 magnetometer slots. MAVLink fields can also be inspected in the field browser.
- When `PX4 uXRCE Agent 설정` is enabled, the MAVLink path sets `UXRCE_DDS_CFG`, `UXRCE_DDS_KEY`, `UXRCE_DDS_DOM_ID`, `UXRCE_DDS_AG_IP`, and `UXRCE_DDS_PRT`. Here `AG_IP` means the PC Micro XRCE-DDS Agent IP that the PX4 uXRCE-DDS Client connects to. If ROS2 topics still have no PX4 publishers after readback succeeds, reboot PX4 or restart `uxrce_dds_client`; PX4 namespace still depends on the vehicle-side client configuration.
- Launching with `--start-pixhawk` or `--connect-pixhawk` automatically opens MAVLink first and switches the first view to `정렬 > Baseline`.
- `Pixhawk 1 Shell` and `Pixhawk 2 Shell` open PX4 shell dialogs over an active MAVLink connection.
- `P1 재부팅` and `P2 재부팅` send MAVLink `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` over the active MAVLink link. Use these instead of typing `reboot` in Shell when PX4 Shell does not respond.

### 정렬

Use this top-level tab to define and validate the stereo-camera extrinsic parameters used by measurement. Most alignment subtabs use PX4/GNSS telemetry; `Optical 정렬` additionally uses the current YOLO matched target as a display-only mechanical alignment preview.

- `정렬 > 센서 인식` summarizes Pixhawk, GPS1/GPS2, CAN1/CAN2, baseline, attitude delta, and overall GOOD/WARNING/BAD alignment state.
- `정렬 > GNSS Base` supports three explicit modes: `현장 Base Survey-in`, `NTRIP 보조 Fixed Base`, and `Network RTK 직접`. The first uses u-blox `CFG-TMODE3`/`NAV-SVIN`; the second feeds NTRIP RTCM into a Base serial receiver, validates repeated RTK Fixed `NAV-PVT` WGS84 ellipsoid coordinates, then requires `CFG-TMODE3` ACK, exact ECEF readback, and fresh post-configuration Base RTCM before it becomes active; the third parses NTRIP RTCM3 frames in the app and routes them directly to selected Rover Pixhawk MAVLink links via `GPS_RTCM_DATA`. Mode, GGA policy, datum, caster/serial identity, and rover targets are latched until stop/restart. It keeps RTCM receive/message-type/injection statistics, Rover RTK status, Survey-in progress, convergence trend/ETA, NAV-SAT signal quality, NTRIP state, and operation readiness in the same scrollable workflow. NTRIP credentials are environment-variable based and are not saved in settings. A correction stream must also satisfy the selected rover RTK Fixed/freshness gate before measurement is allowed; missing enabled quality values fail closed. Disabling the readiness gate is explicit and leaves a `gate_disabled` metadata warning rather than asserting GNSS readiness.
- `정렬 > 소스 / 품질` shows PX4/GNSS/MAG source readiness and quality summaries.
- `정렬 > MAG 관리` keeps five Device Type rows for (`BOARD`, `CAN1`, `CAN2`, `GPS1`, `GPS2`) with X/Y/Z, vector norm (`벡터 크기`), selection, health, rate, and source. Priority editing, save, and reboot remain here for normal operation. The CAL_MAG slot/priority table and raw PX4 parameter mapping are moved to `고급 > PX4 MAG`; when only selected telemetry arrives, the role rows explicitly report that their raw samples are still pending.
- `정렬 > Baseline 계산` computes WGS84 GPS positions into ENU `[E,N,U]` and NED `[N,E,D]` using the selected left camera GPS as origin.
- `정렬 > Baseline / 자세 정렬` shows Pixhawk attitude and the GPS baseline converted into camera RDF components so forward leakage and effective right/down baseline can be inspected.
- `정렬 > Optical 정렬` is for physical camera-R adjustment while viewing a shared target, typically at 30x. It shows L/R bbox centers, signed `dx`/`dy`, expected `dy` (`Rectified 0 px`, raw intrinsics `left_cy - right_cy`, or manual), vertical residual in px/mrad, expected disparity from `fx * baseline / target_distance`, yaw residual in px/mrad, YOLO Range, ray gap, baseline source, triangulation source, and attitude status. It is preview-only and does not write or apply any calculation parameters.

ROS2 topic subscription is tolerant of missing PX4 message types. Missing GPS or magnetometer topic types should not make the whole Pixhawk connection fail.
When ROS2 magnetometer topics have no publisher, `MAVLink 연결` can still populate RM3100 CAN fields from `SCALED_IMU*` magnetometer values after the stream request succeeds.

- The Pixhawk 3D path freezes the consecutively selected left/right GPS samples for each detection pair and recomputes that pair's NED baseline. The GUI's cached latest baseline is diagnostic state, not the authoritative geometry input.
- GPS-only operation remains allowed when both selected samples have at least a 3D GPS fix (`fix_type >= 3`). Non-fixed RTK quality is preserved as selected provenance/warnings; this phase does not introduce an RTK Fixed measurement gate.
- The baseline display shows length, ENU/NED vectors, sample age, quality, and warnings.
- The camera-frame X component shows the left/right optical-baseline component. The full RTK antenna length is shown for alignment but is not automatically equivalent to the camera optical baseline.

### 보정

Use this tab after camera connection and WIRIS timestamp verification, before parameter review and RTK/RM3100/attitude alignment.

- The `체커보드` subtab contains WIRIS zoom controls, checkerboard capture, intrinsic calibration, and calibration preview.
- WIRIS zoom controls can target left, right, or both cameras.
- The `체커보드` subtab is capture-only. Calibration preview uses current camera frames.
- Checkerboard overlay is only refreshed while `보정` is visible, and live checkerboard detection runs in a background worker so tab entry and resizing do not block the main GUI.
- When `YOLO 측정 시작` has switched into active measurement mode, Camera Setup preview is intentionally hidden and calibration preview frame copying is skipped. Camera Setup and Depth Measurement are not intended to be used at the same time.
- `캡처` saves only the current left/right frame(s) where the checkerboard is detected. One visible camera is enough for that camera's intrinsic dataset. When both cameras have frames and a stereo pair would be saved, both frames must have RTCP/NTP-derived camera timestamps and the left/right timestamp delta must be within 50 ms; raw RTP timestamps and app receive times are not accepted for checkerboard stereo pairing.
- Each saved calibration capture writes `capture_metadata/capture_##.json` with the saved sides, RTCP/NTP timing fields, pair delta, and the timing contract used for the stereo pair gate.
- `체커보드 검증/계산` selects a saved calibration root, scans left/right images and matching stereo pairs, and keeps calculation separate from capture. `전체 사진으로 계산` has two modes: `Pixhawk 자세 + GPS/RTK baseline` computes intrinsics only from left/right images and does not write `extrinsics`; `체커보드 Stereo 외부 파라미터` uses only matching stereo pairs and computes intrinsics plus stereo `R/T`. `기존 intrinsics JSON 불러오기` loads an already saved intrinsics file when Pixhawk mode only needs camera intrinsics.
- `랜덤 검증` always uses random matching stereo-pair subsets. It repeats the full intrinsics calculation, and in checkerboard-stereo mode also repeats stereo `R/T`, then reports Full-vs-random baseline, rotation, reprojection, and intrinsics deltas. This is a stability check for the calibration image set, not an automatic truth source.
- `계산 결과 저장` writes the pending full result for the current WIRIS zoom to `intrinsics_zoom_##.json` and refreshes `intrinsics.json`. `계산에 적용` applies the pending full result to measurement and switches the external-parameter calculation mode to Pixhawk or checkerboard according to the selected calculation mode. Full/random validation results do not affect live distance calculation until `계산에 적용` is pressed.
- Checkerboard stereo extrinsics are stored as `extrinsics.rotation_left_to_right` and `extrinsics.translation_left_to_right_m` in the OpenCV optical camera frame contract `RDF(right, down, forward)`, where `X_right = R_left_to_right * X_left + T_left_to_right_m`. In `파라미터 > 계산 설정`, choose `체커보드 Stereo 외부 파라미터` to use the inverted right-camera origin and ray rotation for live stereo triangulation without Pixhawk/GPS runtime inputs. The default remains `Pixhawk 자세 + GPS/RTK baseline`.
- The `줌 내부 파라미터` subtab shows the currently visible WIRIS zoom's left/right intrinsics like the Parameter tab and puts `저장` / `적용` on the right.
- `저장` writes the calibration for the current visible WIRIS zoom level under `<STEREO_WIRIS_APP>/calibration_captures/` as `intrinsics_zoom_##.json` and refreshes the latest `intrinsics.json` alias. If checkerboard stereo extrinsics were computed, they are saved in the same JSON under `extrinsics`.
- `적용` loads the current visible WIRIS zoom's intrinsics and applies them to distance calculation.
- The `저장 파일 비교` list lets you add multiple saved intrinsics JSON files, compare their left/right fx/fy/cx/cy/error values in a scrollable list, remove entries from the comparison list without deleting files, drag entries to reorder them, and apply the selected comparison file to distance calculation.

### 파라미터

Use this tab after `보정` to confirm the geometry that will be used by alignment and measurement.

- `내부` shows the currently loaded left/right intrinsics, distortion coefficients, image count, reprojection error, and whether the result is active or only the latest calibration output. Zoom-specific saves are keyed off the visible WIRIS zoom level used when saving or loading calibration.
- `외부/기하` shows the effective extrinsic source selected in `계산 설정`, checkerboard stereo extrinsics when available, the source of the effective focal length and baseline, and the fixed `nominal-shared-rig-v1` mount assumption. In `Pixhawk 자세 + GPS/RTK baseline` mode, the left/right panels show live P1/P2 Pixhawk-derived pose. In `체커보드 Stereo 외부 파라미터` mode, they switch to the applied checkerboard stereo `R/T`: left is the reference optical frame, right is the inverted right-camera origin/rotation in left RDF, and live Pixhawk telemetry is marked diagnostic-only. The `계산 설정` view under `파라미터` still splits `내부 파라미터` and `외부/기하` side by side around the center line. The default measurement mode is 3D ray triangulation with `Pixhawk 자세 + GPS/RTK baseline`; selecting `체커보드 Stereo 외부 파라미터` uses the saved OpenCV optical RDF stereo transform and only waits for applied intrinsics plus checkerboard stereo extrinsics.
- `수동 초점거리 사용` allows manual `fx` entry when a checkerboard is unavailable. The focal-length and WIRIS zoom-level tooltips include nominal WIRIS OZ zoom estimates from index `0` to `14`, using the 1280x720 HFOV 63.7 degree estimate where index `0` is about `1,030 px` and index `14` is about `30,906 px`.

### 고급 > WIRIS SDK

`WIRIS SDK` can send SDK commands selected from grouped presets or typed manually. Use the `Stream / Set GOP size` and `Stream / Get GOP size` presets there when you want to verify or change visible-stream GOP by hand. Presets default to LINE termination, commands with placeholders such as `<IPv4>` must be edited before sending, and potentially disruptive commands prompt for confirmation.

### 고급 > 필드 감시

`필드 감시` can either accept manual ROS2 topic/field input, run `ROS2 토픽 찾기`, or run `MAVLink 필드 찾기`. ROS2 selections use the Ground_Analysis_Console-compatible `ros2:/topic:field` naming style; MAVLink selections use `mavlink:MESSAGE.field` and are graphed from the active MAVLink Pixhawk connection with left/right prefixes. `감시 제거` uses the compact selector above the graph instead of a separate Watch/Latest Value/Action table, and the graph itself stays label-free.

### 측정

Use this tab to start and monitor object distance.

- `실시간` contains live WIRIS views, stream receive/YOLO Hz status, distance status, recording controls, and a split baseline-center Forward/Right/Down/Range graph. Press `세션 기록 시작` before collecting rows that must be persisted. `YOLO Range` is the Euclidean range from the selected baseline-frame midpoint; no target-telemetry range or truth/error series is produced.
- `파라미터 > 계산 설정` contains focal length, external-parameter source, camera-pair sync gates, class matching, and the attitude timing policy. The `카메라 RTCP 동기화 Gate` group exposes frame history, receive skew, RTCP/NTP pair Δt, Sender Report stale age, Sender Report-to-PC offset, clock-rate tolerance, minimum Sender Report count, and warmup timeout; these values are saved with `저장·분석 > 전체 설정 저장`. `Pixhawk 자세 + GPS/RTK baseline` uses live Pixhawk attitude and GPS/RTK translation; `체커보드 Stereo 외부 파라미터` uses the applied checkerboard `R/T` in the OpenCV optical RDF contract and does not require live Pixhawk/GPS inputs. Pixhawk attitude timing is a display-only, non-blocking diagnostic: sample age above 2 s or pair Δt above 1 s is `STALE`, and sample age above 10 s or pair Δt above 5 s is `VERY_STALE`. The graph records successful ray-triangulated samples even when that status is stale. Matching keeps recent candidates first, then chooses the smaller timestamp delta for accuracy; `disparity_px` remains a bbox horizontal-delta diagnostic in saved rows, not a distance mode. The `내부 파라미터` block uses a collapsible group, while the external-parameter block stays visible and the two blocks are split left/right around the center. Runtime settings are saved from `저장·분석 > 전체 설정 저장`.
- The active and only measurement method is `two_ray_closest_midpoint`; it undistorts bbox-center pixels, solves both rays in internal `left_camera_rdf`, then records Raw `[Forward, Right, Down, Range]` in `baseline_center_frd`. The origin is the midpoint of the left optical origin and selected right-origin estimate; checkerboard `R/T` supplies the optical-center estimate, while Pixhawk records that the GPS/RTK antenna baseline is an optical-baseline proxy without a calibrated lever arm. Axes use the left-to-right baseline, projected mean optical forward, and `Forward × Right`; Range is recomputed from that midpoint. There is no BBox, 3-D bias, or tracking-filter correction path. Missing pose/intrinsics, pixel-grid mismatch, zero baseline, degenerate frame basis, near-parallel rays, non-positive depth, and non-finite range fail closed with no point.
- Each validated detection pair creates one immutable, generation/session-scoped `MeasurementAttempt`. A stable `pair_id` combines the generation identity and monotonic sequence. It freezes the selected detections, nearest attitude samples, consecutive GPS pair, applied K/D and zoom/profile content hash, and checkerboard R/T or selected GPS baseline. Class mismatch and unresolved multiple candidates of the selected class are rejected before geometry. Repeated refreshes reuse the same ready attempt; an incomplete attempt can be replaced only when required inputs arrive. In Pixhawk mode, the right-ray rotation, right optical-forward axis, and NED-to-left-RDF translation use the same selected attitude/GPS snapshot; the resulting `baseline_center_frd` instance is scoped to that attempt. Valid rows serialize it instead of querying latest GUI values again.
- `측정 품질 > 현재 측정 품질` is a display-only panel for the current 3-D ray result. It shows ray gap, midpoint Range, baseline, method, the Raw `baseline_center_frd` point, an explicit source `left_camera_rdf` diagnostic, and a colored Pixhawk attitude timing row with left/right sample age and pair Δt. `FRESH` is green, `STALE` is yellow, and `VERY_STALE` is red. Checkerboard geometry shows `N/A — PIXHAWK ATTITUDE NOT USED`. The main distance line carries only the short `ATT:*` badge; stale timing degrades its display status but does not reject or add a correction to the measurement.
- `YOLO 측정 시작` restarts camera streams in YOLO/depth mode; while measuring, the same button changes to `YOLO 측정 중지` and returns to camera preview mode while keeping Pixhawk connections active.
- The tab shows left/right camera views, current distance status, sync quality, why a sample is not added to the graph when applicable, and baseline-center Forward/Right/Down/Range history over time.
- The live camera status chips show `수신` Hz and YOLO Hz in measurement mode. `수신` Hz counts frames decoded by the stream reader, while YOLO Hz counts frames whose inference completed successfully.
- Recording controls keep one live row for every accepted left/right matched detection frame pair. A pair that is initially missing geometry is retained with its status and is updated in place if the same pair later produces a valid distance.

`카메라 Preview 연결` and `YOLO 측정 시작` are intentionally separate. Preview connection verifies stream availability and frame rate without loading YOLO. YOLO/depth mode is started only when depth measurement is needed.

### 저장·분석

Use this tab to review the current recording summary, inspect Raw recorded samples, and persist data/settings. It is split into `세션 저장` and `기록 샘플`.

- `세션 저장` contains `세션 파일 저장` and `전체 설정 저장`. `세션 파일 저장`, `세션 기록 중지`, camera Preview disconnect, and normal GUI shutdown finalize the active structured record under the active UI's data root: Workstation uses `workstation_data/recordings/sessions/<session_id>/`, while Legacy uses `StereoWIRIS/recordings/sessions/<session_id>/`. The artifact names remain `<session_id>_session_static.json`, `<session_id>_distance_measurements.csv/.parquet`, `<session_id>_camera_left.csv/.parquet`, `<session_id>_camera_right.csv/.parquet`, `<session_id>_px4_left.csv/.parquet`, `<session_id>_px4_right.csv/.parquet`, and `<session_id>.xlsx`. Raw camera MP4 files remain under the session's `video/` directory.
- CSV files are compact human-check files. Parquet files carry the same resource rows with typed analysis-friendly storage. The integrated workbook remains a review file with `Summary`, `DistanceMeasurements`, `CameraLeft`, `CameraRight`, `Px4Left`, `Px4Right`, and `CalibrationStatic` sheets. Dedicated `${session_id}_px4_left.xlsx` and `${session_id}_px4_right.xlsx` workbooks split Pixhawk data into `ATTITUDE`, `GPS1`, `GPS2`, `MAG`, `GLOBAL_POSITION`, `LOCAL_POSITION_NED`, `RAW_IMU`, and `MAG_CONFIG` tabs without a Summary or SyncSnapshot tab. Session-level GNSS-relative data is written to `${session_id}_session_rtk.xlsx` in the `GPS_RTK` tab.
- Pair, GUI-delivered camera, and PX4 callback rows are durably appended under `.spool/` while recording. The GUI table keeps only the most recent 256 pair rows, but final CSV generation reads the complete spool.
- Final artifacts are prepared in one `.generations/<generation_id>` directory. Stable filenames resolve through the single `published` symlink; replacing that pointer is the session-level publish boundary, so readers see either the previous complete CSV generation or the new complete generation.
- `session_manifest.json` records preparation, atomic publish, finalized, and retryable failure states. XLSX is derived: an XLSX failure does not prevent the authoritative JSON/CSV/Parquet resource files from being published, and its stable link may remain unresolved until retry succeeds.
- On successful finalization `.spool/` is removed. If saving or shutdown fails, spool and recovery metadata remain; the next GUI start discovers the newest retryable session, restores its complete pair/camera/PX4 rows, disables new recording, and exposes `세션 파일 저장` as the retry action.
- A recovered retry uses the original `.spool/session_metadata.json` as the authoritative capture provenance. Current GUI calibration, RTSP URL, timing, and configuration values do not replace it; only final row counts and finalization audit fields are added. Video paths are captured after the session paths have been prepared.
- `<session_id>_distance_measurements.*` contains one row per matched detection pair that entered the measurement pipeline. Its primary time axis is `recording_time_ns`, defined as integer nanoseconds from `세션 기록 시작`. The compact row keeps pair id/status, class, bbox centers/confidence, distance/baseline-center result, baseline, sync delta, quality, and warning code.
- `<session_id>_camera_left/right.*` contain one row per frame actually delivered to the GUI `frame_changed` callback while the session is active. They are GUI-delivery logs, not complete decoded-stream logs and not one-to-one MP4 frame indexes. Compact rows keep `recording_time_ns`, optional device UTC, image size, timestamp source, RTP timestamp, RTCP sender-report count, receive delay, and timing status.
- `<session_id>_px4_left/right.*` contain one row per PX4 attitude, vehicle GPS, SensorGps, GNSS-relative, magnetometer, MAG-slot-status, or requested optional MAVLink status message received by the GUI. The only synchronization time axes in PX4 artifacts are `recording_time_ns` (always available host/session timeline) and `gps_utc_time_ns` (GPS-provided UTC generation time when GPS UTC is valid); `gps_utc_source` identifies GPS1 or GPS2. ATTITUDE rows additionally store roll/pitch/yaw rates and `time_boot_ms`. GPS rows fill explicit `gps1_*` or `gps2_*` columns, including position, fix/satellite quality, course heading, ground speed, GPS yaw, and heading accuracy. MAVLink `SCALED_IMU*` rows store magnetometer x/y/z/unit plus acceleration x/y/z, gyro x/y/z, and `time_boot_ms`. Requested `GLOBAL_POSITION_INT`, `LOCAL_POSITION_NED`, `VFR_HUD`, `SYS_STATUS`, `BATTERY_STATUS`, and `RAW_IMU` messages are stored in `mavlink_*` columns. Fields that require ROS2/uORB physical sensor identity, such as MAG device IDs/types/interfaces and selected physical sensor IDs, remain intentionally excluded. `sample_type=mag_slot_status` stores only MAVLink-readable CAL_MAG priority/rotation/enabled/selection status, UAVCAN/EKF2 parameter values, a parameter snapshot JSON, armed state, status, and warning text.
- Fixed session conditions such as schema, stream URLs, calibration/intrinsics/extrinsics, timestamp contract, video paths, and calculation mode are stored once in `<session_id>_session_static.json` instead of being repeated in every CSV row.
- `기록 샘플` contains `매칭 프레임쌍 실시간 보기`. Selecting a row shows Raw baseline-center Forward/Right/Down/Range, source left-camera RDF diagnostics, detector pixels, P1/P2 camera/Pixhawk/GNSS geometry provenance, and timing. It does not show target NED truth, axis/range error, or a UAM alignment profile.
- Recorded YOLO values use `baseline_center_frd`: origin at the midpoint of the left optical origin and selected right-origin estimate, axes `Forward/Right/Down`, public vector `[Forward, Right, Down, Range]`. Checkerboard records the `R/T` optical-center model; Pixhawk records its GPS/RTK antenna baseline as an optical-baseline proxy without a calibrated lever arm. V2 rows use `baseline_center_*` and `raw_baseline_center_*`; they do not emit `x_m/y_m/z_m`, old `left_camera_*`, or `stereo_center_*` public aliases. `source_left_camera_rdf_*` is diagnostic provenance only. Each row records an attempt-scoped frame instance id because Pixhawk/GPS geometry can differ between measurement attempts.
- Live stereo pairing uses `RTCP/NTP 우선` by default. If the two WIRIS RTCP clocks do not produce a pair inside `최대 Δt`, the app falls back to the common app-receive monotonic clock and labels the pair `app fallback`. This preserves live pairing while keeping the RTCP/NTP/PC-UTC result as a separate diagnostic; an `app fallback` pair must not be treated as an absolute UTC-synchronized pair.
- The original session files do not contain `drone_*`, `ulg_*`, or target truth/error fields. A future user-selected drone ULog comparison must be emitted as a separate derived `<session_id>_ulg_comparison.csv/.xlsx` pair without modifying the original session data.
- `전체 설정 저장` persists current runtime settings to `workstation_data/stereo_distance_settings.json` in Workstation mode or `StereoWIRIS/stereo_distance_settings.json` in Legacy mode, including setup camera, WIRIS control, YOLO stream, ROS2, MAVLink, and 이미지-tab FTP/download fields.
- Closing the GUI also saves the current settings automatically.

### 시간

Use this tab for WIRIS RTC synchronization from PC UTC and for watching the resulting stream clock offset over time. Still-image capture and EXIF comparison live in the 이미지 tab.

- `PC UTC로 시간 설정` sends one shared `STUS` UTC value to both WIRIS RTC targets, reads `GDTI`, then automatically reconnects the active RTSP streams before verification. Synchronization uses TCP interleaving by default so RTCP Sender Reports are not lost on a separate UDP path.
- Synchronization state is explicit: `reconnecting` → `warming` → `verified` or `failed`. A failed or incomplete check keeps preview available but blocks an absolute-UTC distance result; the failure reason and raw GDTI responses are written to the session metadata.
- `RTCP/NTP 동기화 확인` selects recent left/right frames received close together, compares their stream-frame `rtcp_frame_ntp_unix_s` values, and reports each frame's `rtcp_frame_receive_delay_s` separately. The status line also shows the cumulative left/right RTCP Sender Report receive counts (`RTCP SR 수신`). The receive delay includes encoder/network/decoder transit and is not treated as a camera-clock offset. This is a stream-timing diagnostic, not proof that RTCP NTP equals WIRIS `GDTI` or an external absolute UTC reference. The status line refreshes on its own while this tab is visible; the button also writes a snapshot to the log.
- `RTCP/NTP 오프셋 추이` plots three panels against a seconds-ago axis: `rtcp_frame_receive_delay_s` (transit delay), `rtcp_frame_transport_delay_s` (transport delay), and `rtcp_sr_age_s` (Sender Report anchor age), each for both cameras. Samples are collected regardless of which tab is showing, so the trend is not broken by tab switches.
- Transit delay contains the camera's absolute clock error, so it grows without bound while that clock drifts, and shrinking it means re-running `STUS` and reconnecting the stream. Transport delay adds the Sender Report's own PC offset back, which cancels the `sr_ntp` term and leaves `(frame arrival - SR arrival) - (media time between them)`; that figure stays flat under clock drift, so use it to judge the actual network/encoder path. Anchor age separates the third case: if it climbs, the anchor stopped refreshing and both delay figures are extrapolated from a stale report.
- The RTP/RTCP timestamp anchor is taken when the stream starts, so an `STUS` correction only reaches the stream timestamps after the RTSP stream is reconnected. The `PC UTC로 시간 설정` action performs this restart automatically when both streams are active.
- Verification requires the configured minimum number of fresh Sender Reports per camera (default 2), Sender Report age ≤7 s, a temporally paired left/right frame history within 1 s with receive skew within 350 ms, left/right RTCP/NTP frame timestamp Δt ≤50 ms, RTCP Sender Report-to-PC offset ≤100 ms, and NTP/RTP clock-rate drift within ±0.5%. GDTI is checked against the midpoint of its request/response interval with the camera's 1-second integer resolution (rather than against the older STUS send time); the two GDTI values must agree within 1 second. The warmup window is 30 seconds. Transit delay is retained as a diagnostic and is not confused with clock offset.
- Timing contract: WIRIS `STUS/GDTI` is the camera body-clock check, RTP is a relative stream counter, and RTCP Sender Report NTP is a stream-derived wall-clock estimate. The app pairs frames by close PC receive time, uses RTCP/NTP delta only for the relative stereo gate, and records RTCP frame transit delay as a transport diagnostic. A stereo pair is within the relative timing criterion when its RTCP/NTP delta is at most the configured `최대 Δt` value, which defaults to 100 ms. Do not use this relative result as proof of synchronization to an external absolute UTC source.

### 이미지

Use this tab to capture WIRIS still images over the Ethernet SDK path, download the latest FTP images, and inspect EXIF clock offset.
The default FTP download directory is `<STEREO_WIRIS_APP>/clock_captures/`; relative paths in this tab are resolved from the StereoWIRIS app root, not from the shell's current working directory.

- `CPTR 촬영 / FTP 다운로드` sends `CPTR` to both WIRIS cameras, checks `ICPT`, downloads the newest JPG/TIFF from each camera's FTP folder, and can run the pair EXIF analysis in one sequence.
- FTP settings use the current left/right WIRIS control hosts for the FTP host addresses; set the FTP user, password, port, remote image directory, and local download folder before running the sequence.
- `파일 쌍 비교` reads one left image and one right image and reports `right-left` capture timestamp offset.
- `폴더 비교` scans left/right folders recursively, pairs images by matching filenames when possible, otherwise by sorted order, and reports per-pair offsets plus median/mean offset.
- The parser reads JPEG/TIFF EXIF `GPSDateStamp`/`GPSTimeStamp` first, then falls back to `DateTimeOriginal` or `DateTime`.
- The table shows capture timestamp source, subsecond availability, GPS coordinates when present, and warnings for second-only timestamps or mixed timestamp sources.

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
- Pixhawk Host IP / uXRCE Agent IP, normally `10.0.0.10`
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
- Optional uXRCE setup through MAVLink `PARAM_SET`/readback for `UXRCE_DDS_CFG`, `UXRCE_DDS_KEY`, `UXRCE_DDS_DOM_ID`, `UXRCE_DDS_AG_IP`, and `UXRCE_DDS_PRT`. This does not replace a vehicle-side namespace/client restart configuration.

MAVLink and ROS2 connection paths are separate and can be used together. Rover operation is MAVLink-first because MAVLink carries RTCM injection, PX4 parameter control, shell diagnostics, reboot, Ethernet MAVLink bootstrap, and telemetry fallback. ROS2 still requires PX4 topics to be published through XRCE-DDS and should be treated as actually connected only when `ros2 topic info -v` shows PX4 publishers and `ros2 topic hz` receives samples; a topic name alone can come from this GUI's subscriptions.

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
- Left/right camera status lines in the same `카메라 상태` group as stream Hz and YOLO Hz in the `측정` tab, showing UTC only when the timestamp basis is marked GNSS-UTC and RTCP/NTP-derived stream time otherwise, with source timestamps as fallback.
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
- Start in `연결 > Pixhawk` from `0 공통 네트워크` to confirm Pixhawk Host IP or swap the current P1/P2 mapping, then use `1 연결 후보` to discover PC IP, Ethernet MAVLink URL, USB control, and ROS2 namespace candidates and assign them with `Host에 적용`, `P1에 적용`, or `P2에 적용`.
- Use `2 Rover MAVLink / RTCM` to confirm MAVLink stream rate, each side's Ethernet MAVLink URL, USB control port, and Ethernet IP, then confirm P1/P2 MAVLink heartbeat first. If Ethernet MAVLink is not arriving, use `P1/P2 ETH MAV 시작(USB)`. Use `3 ROS2 Telemetry` after MAVLink/RTCM readiness to confirm ROS_DOMAIN_ID, uXRCE port, namespaces, and connect ROS2.
- In `정렬 > GNSS Base`, select the Rover RTCM injection target only after the target Pixhawk MAVLink heartbeat is available.
- Use `정렬 > Baseline` to confirm GPS1/GPS2 recognition and baseline reliability before measurement.
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
- Use the live GPS baseline after confirming both Pixhawks have usable GPS fixes; `eph` and sample age remain visible as diagnostics.
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
- Every accepted left/right matched detection pair receives one record row, including pairs whose 3D geometry is not yet valid. Use `pair_status`, `measurement_valid`, and `warnings` to distinguish pending/rejected geometry from successful triangulation. `Pixhawk 자세 + GPS/RTK baseline` still needs calibrated intrinsics and live GPS/RTK translation for a valid Raw distance; GPS-only fixes work with degraded accuracy, while RTK Fixed remains the preferred field setup.
- Stream processing is process-isolated per camera, but total CPU/GPU, memory bandwidth, and RTSP network capacity can still bottleneck the system.
- `Auto` decoder mode falls back from NVIDIA to software when the NVIDIA decoder cannot be created. It cannot fix missing frames from the RTSP source itself.
- The RTSP pipeline drops stale buffered frames when it falls behind. This favors current frames over complete historical frame delivery, which is better for live distance measurement.
- The `PyAV` stream backend depends on the `av` package. If it is missing from a Python environment, the app reports `PyAV 사용 불가` and other backends continue to work.
- If YOLO dependencies or model loading fail, the measurement stream falls back to camera preview only and reports the failure in the YOLO/camera log. Distance calculation resumes only after YOLO inference produces detections.
- The app treats Pixhawk attitude as synchronization/geometry input, not as a replacement for camera extrinsic calibration. Attitude timing status is intentionally non-blocking: stale samples stay visible and recorded for later filtering, while the UI marks the measurement `DEGRADED` and exposes the exact ages.
- Actual selected MAG can only be shown when px4_ros_com publishes `vehicle_magnetometer.device_id`; MAVROS2 `/mavros/imu/mag` usually does not expose that ID, so verify with `listener vehicle_magnetometer` when the UI reports it unavailable.
- The MAG manager node can use MAVROS2 parameter get/set services, but parameter save and PX4 reboot remain documented operator actions unless the deployment wires a confirmed command service.
- Pixhawk-based 3D triangulation uses a fixed camera mount contract, not a UI-selectable preset: world frame `NED`, Pixhawk body frame `PX4_FRD`, camera frame `OpenCV optical`, with camera `+Z -> body +X`, camera `+X -> body +Y`, and camera `+Y -> body +Z`.
- GPS/RTK antenna baseline assumes the Pixhawk/GPS baseline is a usable proxy for camera baseline. Mount offsets still matter.
- Live triangulation uses loaded camera matrices and RTK/Pixhawk geometry; full field accuracy still depends on calibration, sync, baseline quality, and mounting repeatability.
- EXIF clock-offset checks are limited by the timestamp fields present in the saved image. If WIRIS stores only whole seconds and no subsecond tag, subsecond camera skew cannot be proven from metadata alone.
- EXIF `DateTimeOriginal` is timezone-less on many cameras. GPS EXIF time is preferred because it is UTC; mixed GPS and camera-local timestamps are reported as a warning.
- If OpenCV/NumPy cannot import in the active Python environment, a backend that does not need OpenCV may still preview images, but intrinsic calibration and live ray measurement with non-zero distortion are unavailable. These packages and PyYAML are declared runtime dependencies.

### Calibration Dependency Bootstrap

If OpenCV calibration dependencies need repair:

```bash
cd <STEREO_WIRIS_APP>
PYTHON_BIN=.venv/bin/python ./scripts/bootstrap_calibration_deps.sh
```

Or run the bootstrap before opening the GUI:

```bash
STEREO_BOOTSTRAP_CALIBRATION_DEPS=1 ./run.sh
```

The bootstrap installs the self-contained OpenCV/NumPy/PyYAML subset in `requirements/calibration.txt` and verifies `import cv2`. The same packages are also listed in `requirements/runtime.txt` because live undistortion/ray geometry and alignment use them.

## Change Log

Entries below describe the state at their original dates. The current contract
is authoritative: as of 2026-07-24, public measurement is
`baseline_center_frd:[Forward,Right,Down,Range]`; target/UAM telemetry, ULog comparison,
Telemetry Range, target truth/error fields, and the target coordinate-profile
workflow have been removed from StereoWIRIS. P1/P2 Pixhawk telemetry remains
only for stereo-rig geometry and diagnostics.

### 2026-07-24

- Migrated Raw/public measurement from left optical-center RDF to
  attempt-scoped `baseline_center_frd` with midpoint Range recomputation.
- Bumped paired rows to `stereowiris-paired-frame-v2`, added explicit
  `baseline_center_*` fields, and stopped emitting ambiguous coordinate aliases.

### 2026-07-20

- Removed `연결 > UAM`, `저장·분석 > UAM 비교`, ULog loading, target telemetry
  fallback, Telemetry Range, and generated `uam_analysis.csv` artifacts.
- Kept Raw `left_camera_rdf:[Right,Down,Forward,Range]` measurement and P1/P2
  rig attitude/GNSS/MAG/status data.

### 2026-06-09

- Moved low-level/manual tools to `고급`: `WIRIS SDK`, `필드 감시`, and `Bias 보정`.
- Kept `연결` to the primary `WIRIS`, `Pixhawk`, and `UAM` connection workflow, and kept `측정` to live measurement, calculation settings, and triangulation quality.
- Marked `연결 > UAM` as the standard ground-truth telemetry entry point for the central UAM/drone GPS antenna and Pixhawk body-origin N/E/D assumption.
- Removed duplicate local settings-save buttons from the YOLO and measurement tabs; use `저장·분석 > 전체 설정 저장`.
- Removed legacy UAM center watch registration from `고급 > Bias 보정`; use `연결 > UAM` for live ground-truth telemetry.
- Split ULog actions into `ULog 선택/로드` and `현재 경로 다시 로드`.
- Added UAM coordinate-alignment profile estimation, JSON save/load, and application in `저장·분석 > UAM 비교` so final analysis can report estimated N/E/D and position error after origin correction.
- Added a compact legend and hover tooltip to telemetry graphs, including `Raw NED`, so operators can identify series color and inspect point time/value.

### 2026-07-17

- Added an RTCP timing contract for cameras whose Sender Report NTP arrives later than the encoded frame: preserve the RTCP-derived relative pair timestamp for stereo matching, report frame transit delay separately, and keep UAM UTC claims unverified until an independent clock check passes.
- Method: `RTCP pair-preserve / transit-delay-separation`. Inputs are RTCP Sender Report NTP, app receive monotonic time, and PC wall UTC. The relative pass criterion is pair delta <= the configured `최대 Δt` value, currently defaulting to 100 ms; frame transit delay is diagnostic only, while absolute UAM UTC use requires repeatable GDTI-to-RTCP-to-PC validation and the motion-error budget.

### 2026-06-02

- Added `연결 > UAM` and `저장·분석 > UAM Ground Truth 비교` so ULog-first post-flight analysis can fall back to live ROS2/MAVLink UAM telemetry or embedded recorded drone rows.
- Replaced the alignment workflow with `정렬 > Baseline`, including GPS recognition, ROS2/MAVLink field browsing, baseline calculation, reliability evaluation, and YAML profile save/load.
- Added RM3100 priority logic and tests for the current Pixhawk 6X Pro values and the two-RM3100 backup case.
- Kept measurement-tab camera preview alive when YOLO model loading or per-frame inference fails, including a CUDA prediction failure path that retries on CPU.
- Preserved recent valid detections across intermittent `검출 없음` frames so sparse left/right detections can still match and produce distance estimates.
- Removed the GUI's simple Disparity distance mode; bbox horizontal delta remains available as the `disparity_px` diagnostic field.
- Decoupled stream readers from YOLO inference so measurement-mode `수신` Hz counts decoded reader frames while YOLO Hz counts successful inference frames.
- Reordered tabs for the field workflow: `연결`, `시간`, `보정`, `파라미터`, `정렬`, `YOLO`, `측정`, `저장·분석`, `고급`; moved `WIRIS SDK` after `Pixhawk` inside `연결`.
- Split stereo detection-pair validation from the old simple disparity distance formula so 3D triangulation does not depend on that calculation path.
- Applied the fixed front-facing camera mount frame contract to Pixhawk-based GPS/RTK translation and ray rotation, and record `world_frame`, `body_frame`, `camera_frame`, and `camera_mount_profile` in distance CSV rows.
- Split WIRIS SDK controls into the separate `연결 > WIRIS SDK` subtab.

### 2026-05-17

- Changed WIRIS control discovery from a single configured-port probe to candidate-host TCP `1-65535` scanning with `HIWS` verification.
- `Telnet port 탐색` and `WIRIS 자동 연결` now update each side's discovered SDK/Telnet port before running control checks or RTSP enable commands.
- Changed WIRIS SDK command preset descriptions to Korean and enabled wrapped description text/tooltips.
- Removed the global operation status chip/header and next-action button from the top of the window.
- Unified start/stop style button pairs for camera preview, YOLO measurement, ROS2, and MAVLink into single state-changing buttons.
- Compressed the `인터페이스`, `PC IP`, `포트`, `타임아웃`, `IP Mask`, and `STIP 후 재부팅` camera network settings into one compact top row.
- Replaced the free-form YOLO `추론 장치` input with a CPU/CUDA toggle that stores `cpu` or `cuda:0`.
- Moved the `R/T 삼각측량 사용` toggle from `보정` into `파라미터 > 계산 설정`.
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
- Added a `계산 설정 저장` button inside the `파라미터 > 계산 설정` group so the active stereo calculation settings can be saved without leaving the parameter tab.

### 2026-05-18

- Requested MAVLink `GPS_RAW_INT` and `GPS2_RAW` streams automatically on heartbeat so `정렬 > Baseline` can inspect GNSS data even when ROS2 GPS topics are unavailable.
- Added `--start-pixhawk` / `--connect-pixhawk` startup flags to open MAVLink and show the Baseline alignment view immediately during field checks.
- Added GStreamer RTP timestamp and RTCP Sender Report based NTP estimate CSV fields for left/right stream timing diagnostics.
- Kept the measurement-tab live timing details in tooltips/record metadata while the real-time summary shows both left/right RTCP-derived NTP times alongside receive Hz, YOLO Hz, and Pair status.
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
