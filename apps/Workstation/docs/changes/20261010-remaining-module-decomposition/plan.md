# Workstation 대형 모듈 분리 계획

**최종 소프트웨어 상태(2026-10-11):** M1–M15의 고정 책임 분리 및 G1 재현 체계 마감. 감사에서 추가로 확인한 M2/M7/M8/M10/M15의 잔여 정책도 이관했다. 최종 전체 회귀2,696passed/1NVENCskip/0failed/0warnings(327.64s),649Python+stub 해시 일치. [현재 분류와 한계](status-audit.md), [최종 검증](remaining-final-verification.json). 실장비·장시간/GPU 검증은 범위 밖이며 커밋·푸시는 미실행이다.

**재개 당시 실행 범위(이하 경과 기록):** 사용자가 "남은 것도 마저 진행해"라고 명시했다. 앞서 완료한 M2 마감은 유지하고, 아래 고정된 남은 목록을 재개한다. 기존 계획 밖 새 기능·장치 정책·좌표 공식 변경은 추가하지 않는다. 전체 완료는 책임/상태/자원 경계, 기존 공개 흐름, 좌표·저장 계약, 관련 및 결합 회귀를 항목별로 확인한 뒤 판정한다. 메모리/실장비 검증의 실패·미검증은 구조 분리 PASS와 구별한다.

재개 당시 배분: M5/M12/M13 화면 components, M4 canonical/context/index/Excel streaming, M7 남은관리작업, M10 operation/base/corrections/evidence는독립agent가구현한다. Root는M6 evidence/query와공통composition/검증을맡고, 이어M1/M2/M8/M15의남은책임을처리한다. 이것이이번에끝낼고정목록이며기존완료항목을반복분리하지않는다.

Workstation에 남은 대형 모듈의 공유 상태, 비동기 수명주기, 저장 실패 전파를 줄이기 위한 계획이다. 각 책임의 상태와 자원을 한 소유자에게 옮기고, 다른 기능은 명시적인 명령과 읽기 전용 스냅샷으로만 접근하게 한다. 화면 조작, 계산 방식, 좌표, 기록 형식은 유지한다. 저장 시 메모리 증가를 해결하는 스트리밍 전환은 구조 이동과 별도 단계로 검증한다.

계획 수립 후 사용자가 "안전한 범위에서 수정 시작해"라고 지시했고, 단계 완료를 이유로 중단하지 말라고 지적했다. **S1 자료형·해석·transport·명령 소유권과 S2의 연결 작업 소유권은 검증됐으며, 남은 계획을 계속 구현 중**이다. 실제 slice 범위와 결과는 [검증 기록](verification.md)에 남긴다. 중간 단계의 성공을 전체 계획 완료로 해석하지 않는다.

## 기준과 범위

기준은 2026-10-10에 읽은 HEAD `42b16e0`와 당시의 미커밋·신규 Workstation 파일을 합친 작업 트리다. HEAD만 checkout하면 이 기준을 재현할 수 없다. [baseline.json](baseline.json)에 대상 파일의 SHA-256, 줄 수, 함수 수와 최대 함수 길이를 기록한다. 구현 시작 전에 파일 해시와 작업 트리 변경분을 다시 확인하고, 다른 작업은 보존한다.

기존 [구조 문서](../../ARCHITECTURE.md), [측정 실행 계약](../../MEASUREMENT_RUNTIME_CONTRACT.md), [기록 스키마](../../RECORDING_SCHEMA.md), [이전 분해의 호환 경계](../20261010-god-module-decomposition/compatibility-exit.md)를 따른다. 계획은 2,000줄 이상인 운영 Python 파일 15개와 생성 코드 관리 1건을 다룬다. 줄 수는 주석·공백 포함이며 테스트·도구·생성 MAVLink 파일은 운영 파일 집계에서 제외했다.

| 대상 | 줄 수 | 핵심 위험 | 계획 절 |
|---|---:|---|---|
| `optical_pose_controller.py` | 5,511 | 동결 입력과 실시간 panel 참조, 후보 적용·복구 결합 | M1 |
| `calibration_controller.py` | 5,257 | 촬영·선택·solver·품질·적용의 상태 결합 | M2 |
| `workstation_core/yolo_stream.py` | 5,222 | 디코딩·추론·녹화·IPC·종료 결합 | M3 |
| `workstation_core/recording_session.py` | 4,435 | 발행·rollback·스키마·전체 행 메모리 적재 | M4 |
| `workspace_views/calibration.py` | 4,136 | 화면과 파일 선택·품질 판단·파일 접근 결합 | M5 |
| `evidence.py` | 4,094 | 실시간 조회·계산·파일 조회·표시 투영 결합 | M6 |
| `device_runtime_controller.py` | 3,838 | 연결·장치 변경·복구·네트워크 역할 결합 | M7 |
| `panel_camera.py` | 3,778 | 184개 선언 의존성과 UI·연결 동작 결합 | M8 |
| `workstation_core/pixhawk_attitude.py` | 3,539 | 메시지 해석·시간·전송·명령 배타성 결합 | M9 |
| `gnss_runtime_controller.py` | 3,445 | NTRIP·기지국·RTCM·기록 문맥·UI 결합 | M10 |
| `wiris_runtime_workers.py` | 2,612 | 여러 장치 명령과 FTP·촬영 상태기계 집중 | M11 |
| `workspace_views/optical_pose.py` | 2,592 | 주석·후보·RTK 관측·방식별 화면 상태 집중 | M12 |
| `workspace_views/pixhawk.py` | 2,543 | 장치 역할·자력계·GNSS·표시 상태 집중 | M13 |
| `main_window.py` | 2,413 | 명령 등록 729줄, 기능 worker와 창 수명 결합 | M14 |
| `camera_stream_controller.py` | 2,076 | 준비 판정·재시도·Run 시작·녹화 종료 결합 | M15 |
| `mavlink_dialects/sasl_can_mag_dialect.py` | 27,064 | 생성 출처·재생성 재현성 | G1 |

## 공통 설계와 유지할 계약

의존 방향은 `화면 → 기능의 명령·스냅샷 → 응용 서비스 → 기존 core`로 유지한다. 바깥 조립 계층만 QObject 부모, 장치 어댑터, 저장소를 연결한다. 아래의 새 경로와 클래스 이름은 제안이며, 기존 책임을 재사용할 수 있으면 새 모듈을 추가하지 않는다.

- 새 도메인 서비스는 panel, QWidget, 전체 backend, 범용 `getattr/setattr` 프록시를 받지 않는다. 수백 개 필드에 접근하던 동작을 수백 개 포트로 그대로 옮기는 것도 완료로 인정하지 않는다. 기능별 요청·결과·스냅샷과 좁은 작업 인터페이스로 바꾼다.
- 가변 상태, 소켓, serial endpoint, 타이머, worker, 큐, 파일 핸들마다 실제 소유자를 하나 지정한다. 호환 속성은 같은 소유자에게 위임하며 사본을 저장하지 않는다. 이미 존재하는 잠금과 승인 장부는 쪼개서 복제하지 않는다.
- 비동기 작업의 요청 시점에 필요한 설정과 식별자를 고정한다. 기능별 `request_id`, 연결 또는 촬영 세대, 원래 session/run/burst 식별자를 함께 전달한다. 완료를 받을 때 현재 소유권과 비교한다. 유효한 과거 측정 결과는 원래 Run으로 기록하고, 폐기된 연결의 응답은 새 연결 상태를 바꾸지 못한다. 모든 늦은 결과를 일괄 폐기하는 정책은 금지한다.
- 각 소유자는 사용 중인 자원을 명시적으로 열거하고 기존 `PanelApplication.resource_inventory()`와 종료 제어에 연결한다. 종료 검사는 자원 조회 때문에 아직 사용하지 않은 서비스를 생성하지 않아야 한다. worker 수, 프로세스 수, 큐 용량, CPU/GPU 배치, fallback 및 종료 시간 정책은 구조 이동 단계에서 유지한다.
- 도메인 상태 변경은 기존 실행 문맥에서 수행한다. QWidget 접근은 GUI thread에 남긴다. QObject receiver를 자유 함수로 바꿔 signal의 실행 thread가 달라지지 않도록 검사한다.
- 명령 접수, ACK, readback, 실제 완료는 서로 다른 상태로 유지한다. 연결 설정만 저장됐는데 LIVE로 표시하거나, 녹화 종료 요청만으로 파일을 finalized로 표시하지 않는다.
- 기존 측정 method ID, 수치 공식, 보정 품질 기준, source 선택·fallback·hysteresis를 유지한다. 정상 Measurement Stop이 앱 수명의 method worker를 취소하지 않는 계약, 계산 시작 후 자동 deadline이 없는 계약도 유지한다. 기록 실패와 MAG advisory가 관측 Run을 차단하지 않는 기존 정책을 보존한다.
- 재생 입력에는 기록 당시 값만 사용한다. 보정 시점의 truth 또는 검증 기준을 runtime 입력으로 끌어오지 않는다. 알고리즘 개선이 필요해지면 분리 작업의 결과로 섞지 않고 별도 변경으로 기록한다.

여러 분리 대상이 함께 건드리는 상태는 아래처럼 한 곳에 귀속시킨다. 이 경계를 먼저 정하지 않으면 작은 모듈 여러 개가 같은 값을 서로 덮어쓰게 된다.

| 공유 경계 | 단일 소유권과 다른 기능의 접근 |
|---|---|
| 활성 K,D·R,T·profile hash·적용 revision | 현재 `CalibrationFeature`의 관련 상태를 Qt/I/O 없는 `workstation_core/active_geometry.py`의 좁은 소유자로 이행한다. M1·M2는 준비한 profile을 적용 명령으로 전달하고 같은 snapshot을 읽는다. 기능별 파일 발행·후보 목록은 각 기능에 남긴다. |
| 광학 적용과 복구 | M1은 후보·선택 이력을 소유하되 활성 profile을 별도로 저장하지 않는다. 공유 geometry 소유자가 이전 상태·적용 revision을 포함한 복구 token을 관리한다. 저장 실패 시 해당 거래만 복구하며 이후 적용을 과거 rollback이 덮지 못하게 한다. |
| MAVLink 연결·명령·RTCM | M9 transport가 실제 연결과 send/exclusive locks를 소유한다. M7은 운용 명령을, M10은 RTCM 주입을 요청한다. 별도 연결을 만들어 잠금을 우회하지 않는다. |
| WIRIS 촬영과 calibration/optical workflow | M11은 한 작업의 장치 I/O·취소·timeout, M1·M2는 요청과 결과의 업무 귀속을 소유한다. 같은 worker/thread를 양쪽에서 생성·해제하지 않는다. |
| 카메라 스트림과 Measurement Run | M3은 프로세스·buffer·녹화, M15는 스트림 수명, M8은 설정·control 연결을 소유한다. Run 상태는 기존 `measurement_run.py`가 계속 소유한다. |
| 기록 승인과 최종 발행 | 기존 `recording/`의 승인 장부와 M4의 닫힌 spool·파일 발행을 구분한다. M4가 active 행의 승인·재시도 정책을 다시 구현하지 않는다. |
| 표시용 evidence | M6·각 화면은 소유자의 snapshot만 읽으며 별도 정답 상태를 유지하지 않는다. cache에는 원본 revision·source time을 함께 둔다. |

새 소유자의 자원 목록에는 충돌하지 않는 기능·역할 식별자를 사용한다. 종료 조립 과정에서 같은 이름의 `_worker`를 덮어써 다른 자원이 목록에서 사라지는 경우도 회귀 검사에 포함한다.

## 좌표와 시간 경계

| 경계 | 보존할 계약 | 검증 방법 |
|---|---|---|
| GNSS 위치·기준선 | 위치 차이는 world NED `[north,east,down]`, m. WGS84 각도는 deg. 타원체고·MSL고·geoid는 구분 | 축마다 서로 다른 값, 높이 종류 누락·불일치 fixture |
| Pixhawk 자세 | quaternion `wxyz`, body FRD → world NED. 표시 RPY deg와 각속도 rad/s 구분 | identity 및 알려진 90도 회전의 독립 기대값 |
| 광학 ray·상대 자세 | optical RDF `[right,down,forward]`; `X_right = R_right_from_left X_left + T`, 오른쪽 중심은 왼쪽 RDF에서 `-R.T @ T` | 알려진 baseline·점으로 ray와 중심을 각각 검증 |
| 공개 측정 | optical baseline-center FRD `[forward,right,down,range]`, m. `source_point_left_rdf_m`는 왼쪽 카메라 원점 | 비대칭 점으로 RDF→FRD 순서와 midpoint 이동 검증 |
| 기준선 중심 표시 | GNSS antenna midpoint local-level FRD, method midpoint local-level FRD, optical baseline-center FRD를 구별 | 동일 `x/y/z`처럼 취급하지 않고 frame·axis label과 저장 필드 대조 |
| MAG·raw telemetry·ULog | 센서별 producer 축·단위·topic 계약을 보존. `x/y/z`를 이름만으로 body/world 축이라고 추정하지 않음 | 원시 메시지·ULog fixture와 producer 변환을 함께 대조 |
| source·host 시간 | RTP tick/SSRC, RTCP mapping, PX4 boot time, UTC, host monotonic의 단위와 epoch 구분 | wrap, reboot, 재연결, 오래된 응답, clock jump fixture |
| JSONL·CSV·Parquet·XLSX | 기존 필드명·열 순서·null·alias·schema·hash 의미 유지. 저장과 UI 축 이름 일치 | 독립 golden 기록, 재열기, 타입·필드·checksum 대조 |

ENU 값이 새 경계에 나타나면 기존 NED 자료형에 넣기 전에 명시적인 변환을 정의한다. 이번 계획은 ENU 경로나 좌표 변환 공식을 새로 도입하지 않는다. 단순 round-trip은 외부 축 계약의 단독 검증으로 사용하지 않는다.

## M1 광학 자세 제어기

대상은 [optical_pose_controller.py](../../../optical_pose_controller.py)다. 현재 `_clone_for_auto_select()`는 입력 일부를 복사하면서 panel도 보유한다. 기존 동결·stale 결과 검사를 보존하면서 이 참조 자체를 계산 경계에서 제거한다.

| 제안 소유자 | 옮길 책임과 경계 |
|---|---|
| `optical_pose/capture.py` | native·FTP burst 수집, 양쪽 frame/hash, GNSS·yaw90·K,D·prior·zoom 입력 고정. `FrozenOpticalPoseInputs` 생성 |
| `optical_pose/solve.py` | horizon·mountain/cloud·terrain·drone·combined 방식 조립. 기존 `pose_solvers.py`, `optical_pose_auto_select.py`, `optical_pose.py` 수치 코드를 호출 |
| `optical_pose/observations.py` | RTK observation·multiview sample의 burst별 소유권과 수정 이력 |
| `optical_pose/selection.py` | 후보·추천·annotation 승인과 source identity·DOF·관측 가능성 검증 |
| `optical_pose/application.py` | profile prepare → 공유 geometry 소유자에 commit 요청 → persist 실패 시 token으로 restore. 후보 선택 이력과 캐시 무효화를 같은 거래에 연결 |
| `optical_pose/presentation.py` | GUI thread에서 결과를 받아 상태를 표시. 바깥 `optical_pose_controller.py`는 기존 명령 이름만 위임 |

동결 자료형은 이미지·numpy 배열·중첩 dict까지 소유권을 고정하고 내보낼 때 mutable alias를 노출하지 않는다. worker는 panel을 가진 Controller 복제본 대신 이 입력과 mode만 받는다. `measurement/boundary.py`가 현재 읽는 후보·prior·selection 접근도 읽기 전용 optical snapshot 계약으로 이행한다.

통과 조건: 계산 도중 profile·zoom·GPS·선택 후보를 바꿔도 접수된 계산은 동일하다. 새 burst 뒤 도착한 추천은 현재 화면에 적용되지 않는다. 적용 저장 실패는 이전 활성 profile과 이력을 복구한다. 원래 관측·후보의 identity와 수치 결과는 동일하다.

기존 기반: `test_optical_pose_controller_integration.py`, `test_optical_pose_workers.py`, `test_optical_pose_session_persistence.py`, `test_optical_pose_telemetry_multiview.py`, `test_optical_pose_geometry.py`. 필요한 alias 변조·완료 순서·저장 실패 사례를 여기에 확장한다.

## M2 캘리브레이션 제어기

대상은 [calibration_controller.py](../../../calibration_controller.py)다. `calibration_workflow/` 아래 촬영, 입력 선택, solver 실행, 품질 판정, profile 적용·저장을 분리한다.

- `capture.py`가 수동·자동 촬영의 timer와 inflight 요청을 소유한다. native frame·timing evidence·zoom을 한 요청으로 고정하고, FTP/OZ 작업은 M11의 작업 어댑터에 전달한다.
- `inputs.py`가 파일 목록·사용 여부·내용 hash를 고정한다. 기존 `calibration_input.py`, `calibration_selection.py`, `calibration_timing.py`를 재사용한다. 화면의 visible root와 solver 입력 root가 별도 선택으로 갈라지지 않게 한다.
- `solver_jobs.py`가 intrinsic·stereo·random-validation worker의 시작·진행·완료·실패를 소유한다. 요청 snapshot hash를 결과까지 전달하고 중복 완료를 한 번만 반영한다.
- `quality.py`가 기존 `calibration_quality.py`의 판정을 감싸며 UI와 apply가 같은 결과를 사용하게 한다. 화면의 승인 여부를 backend의 품질 증거로 대체하지 않는다.
- `profiles.py`가 immutable 파일 발행·load·적용 요청을 소유하고 공유 geometry 소유자를 통해 활성 profile을 변경한다. `presentation.py`가 촬영 feedback과 UI를 갱신한다. 외부 파일은 기존 owned data root로 가져오고 hash 검증 후 상대 경로를 보존한다.

자동 촬영 Stop, solver 취소, 워커 종료는 현재 지원하는 의미를 각각 유지한다. 취소 불가능한 작업의 UI만 멈추고 worker가 사라졌다고 표시하지 않는다.

통과 조건: 이중 Capture 방지, 단안 intrinsic 입력과 stereo pair eligibility 구분, host receive proxy가 precision timing으로 승격되지 않음, 계산 중 파일·선택 변경 검출, 실패 후 재시도, session/root 전환 중 늦은 완료의 올바른 귀속이다.

기존 기반: `test_calibration_input_snapshot.py`, `test_calibration_selection_contract.py`, `test_calibration_capture_feedback.py`, `test_calibration_quality_integration.py`, `test_workstation_calibration.py`.

## M3 영상 처리

대상은 [workstation_core/yolo_stream.py](../../../workstation_core/yolo_stream.py)다. `workstation_core/media_stream/`에 다음 경계를 만든다.

| 소유자 | 책임 |
|---|---|
| `contracts.py` | frame·detection·상태·recording-finalization의 기존 IPC payload와 설정 계약 |
| `rtp_timing.py` | `GstRtpTimingTracker`, RTP/RTCP 시간·gap·access-unit 분석. 관측 결과만 반환 |
| `gstreamer.py`, `pyav_backend.py`, `opencv_backend.py` | backend별 연결·프레임 수신·해제. 서로의 종료 상태를 변경하지 않음 |
| `inference.py` | 모델 실행, rate limiter, 기존 CPU fallback 정책 |
| `recorders.py` | `AsyncVideoWriter`, `DecodedNv12Recorder`의 큐·encoder·finalization. 시작 접수와 사용 가능한 파일 판정을 분리 |
| `child_runtime.py` | 자식 안에서 수신→추론→녹화를 조립하고 producer·consumer 종료 순서 관리 |
| `supervisor.py` | 부모 프로세스와 IPC 큐, parent-death, 재시작, cooperative/terminate/kill 확인. 기존 Qt 신호 어댑터 유지 |

739줄 GStreamer 함수는 먼저 timing·recorder·backend 인터페이스를 뽑고 마지막에 조립을 줄인다. 프레임마다 대형 객체를 추가 복사하는 경계를 만들지 않는다. native frame lease와 buffer 수명, 프레임 유실 지표, 큐의 기존 bounded/drop 정책을 유지한다. 종료 deadline이 지났다고 핸들을 즉시 폐기하거나 중복 자식을 시작하지 않는다.

자식 프로세스 entrypoint와 자료형 이동은 새 Python interpreter의 spawn 경로에서 검증한다. 현재 부모에서 import가 성공하는 것만으로 통과하지 않는다. 기존 pickle/import 이름과 class identity의 호환성을 확인하고, backend import 이동으로 부모에서 CUDA/GStreamer 자원이 먼저 초기화되지 않게 한다.

통과 조건: 프레임과 detection의 identity 일치, RTP wrap·SSRC 변경·packet loss, decoder fallback, 큐 포화, 연결 실패·재연결, Stop 중 finalization, 부모 종료 시 자식 회수다. 성능은 같은 녹화 fixture·해상도·backend에서 worker/thread 수, queue high-water, 처리율, preview 지연, RSS/GPU 메모리를 기준선과 비교한다. 허용 편차는 S0에서 반복 측정 변동폭과 기존 운용 한계로 정하며 구현 후 완화하지 않는다.

기존 기반: `test_workstation_yolo_stream.py`, `test_yolo_stream_parent_death.py`, `test_native_calibration_frame.py`, `test_media_evidence.py`, `test_camera_stream_recovery.py`. 실제 NVDEC/NVENC는 해당 장비에서 별도 검증해야 한다.

## M4 세션 저장

대상은 [workstation_core/recording_session.py](../../../workstation_core/recording_session.py)다. 기존 `recording/`은 앱의 기록 흐름을 소유하므로 중복 구현하지 않고, 순수 저장·발행 계층만 `workstation_core/session_storage/`로 나눈다.

- `spool.py`: `SpoolRows`와 admission·행 교체·복구. 기존 lock과 행 identity를 함께 이동한다.
- `schema_projection.py`: compact row, column, 시간·좌표 필드 투영. 원시 입력을 수정하지 않는 함수로 유지한다.
- `recovery.py`: retry metadata, spool 발견·재열기, legacy 복구.
- `generation.py`: staging, fsync, current pointer 교체, readback 검증, rollback, orphan 정리. 이를 여러 독립 쓰기 주체로 나누지 않는다.
- `finalize.py`: identity 검증→canonical JSONL 작성·검증→기존 `session_parquet.py` 파생 출력→발행·readback→spool 해제 순서를 조립한다.
- `review.py`: 기존 `session_review.py`와 연결해 Excel 파생 보기 재생성을 담당한다. 기존 legacy writer는 명시적 호환 경로에만 남긴다.

1차는 저장 결과와 실패 의미를 그대로 옮긴다. 2차에서 `_normalized_pair_rows()` 등을 통해 전체 행 목록을 동시에 만드는 구조를 바꾼다. 닫힌 spool 또는 한 번만 읽을 수 있는 iterable을 staging canonical JSONL로 순차 처리하고, 검증한 파일에서 bounded batch로 파생 출력을 만든다. context 중복 제거·방법 결과 인덱스도 전체 row 사본을 모으지 않도록 파일 기반 또는 제한된 인덱스로 설계한다. `session_parquet.py`의 기존 row-group 경로를 활용한다.

스트리밍 전환은 별도의 성능·저장 동작 변경이다. 동일 golden 기록의 전체 행·순서·dynamic field·null·타입·hash를 보존한 뒤, 동일 row 크기에서 행 수를 늘리는 측정으로 메모리 증가가 전체 행 복제에서 비롯되지 않음을 입증한다. 큰 단일 행, 인덱스, 파생 Excel의 메모리는 별도 측정한다. Excel까지 bounded라고 입증하지 못하면 전체 저장 메모리 위험을 해결했다고 판정하지 않는다.

통과 조건: JSONL/Parquet/fsync/rename/readback 각 실패점에서 이전 current와 retry spool 보존, 실패 세대 rollback, 성공 확인 후에만 spool 해제, review 실패가 canonical 성공을 무효화하지 않음, 프로세스 재시작 후 복구다. 한 run을 저장하는 동안 다른 run의 active spool을 건드리지 않아야 한다.

기존 기반: `test_recording_raw_artifacts.py`, `test_session_parquet.py`, `test_session_recording_catalog.py`, `test_recording_controller_save_lifecycle.py`, `test_recording_reader.py`.

## M5 캘리브레이션 화면

대상은 [workspace_views/calibration.py](../../../workspace_views/calibration.py)다. `workspace_views/calibration_parts/`에 capture surface, session/file picker, quality/outlier panel, validation panel, preview를 나눈다. `CalibrationWorkspace`는 배치·signal 연결·선택 화면만 소유한다.

파일 선택과 품질 정책은 M2의 서비스로 보내고, 화면은 요청을 만들거나 snapshot을 표시한다. 파일 목록 변경과 thumbnail 읽기는 I/O 경계에서 수행하며 QWidget 갱신은 GUI thread에 둔다. 지연 응답은 선택 root와 요청 세대를 확인한다. 기존 preview/thumbnail worker가 처리할 수 있으면 새 worker를 만들지 않는다.

통과 조건: 양방향 파일 선택과 solver snapshot 일치, 품질 warning 승인·거부 동작 유지, 표시 중 capture 완료·root 전환·큰 파일 목록에서 선택·스크롤·열 너비·preview 모드 유지. 기존 `test_calibration_quality_ui.py`, `test_calibration_photo_time_ui.py`, `test_workspace_architecture.py`와 화면 비교를 사용한다.

## M6 증거 스냅샷

대상은 [evidence.py](../../../evidence.py)다. `evidence_views/`에 telemetry, GNSS/geometry, calibration, recording/catalog, connection/settings 투영을 분리한다. `EvidenceAdapter`는 동일 public snapshot을 조립하는 호환 경계로 남긴다.

실시간 상태 수집은 각 소유자에서 한 번 수행하고, 투영 함수는 명시적 snapshot과 `now_monotonic_s`를 받는다. `drone_pixhawk_geometry.py`, `gnss_baseline_center.py`, readiness와 calibration core의 계산을 재사용하며 표시용 독립 공식을 만들지 않는다. 상태 채집의 원자성 또는 세대 일관성을 각 데이터 소유자의 lock·snapshot 계약으로 보장한다. 투영용으로 모든 기능의 mutable state를 다시 모은 만능 저장소를 만들지 않는다.

session catalog·artifact 파일 조회는 query 서비스로 분리한다. 비싼 조회는 GUI refresh 호출에서 직접 수행하지 않고 기존 작업 실행 경로로 요청한다. 새 executor가 필요하면 수명·동시성·종료 예산을 명시하는 별도 slice로 다룬다. 읽기 실패 시 최신 정상 값처럼 보이는 기본값을 만들지 않고 기존 오류·미확인 상태 의미를 유지한다.

통과 조건: 같은 snapshot과 now로 같은 출력, 렌더 중 장치 명령·저장 쓰기 없음, 라이브 상태를 바꿔도 과거 snapshot 불변, 일부 artifact 손상 시 다른 기능 표시 유지, UI label과 저장 frame 일치다. 기존 `test_drone_pixhawk_geometry_evidence.py`, `test_workstation_contracts.py`, `test_session_recording_catalog.py`, `test_workstation_ui.py`를 활용한다.

## M7 장치 제어

대상은 [device_runtime_controller.py](../../../device_runtime_controller.py)다. 고정 이름으로 확인되는 panel 참조가 212개다. `device_runtime/`을 다음 서비스로 나누되 실제 MAVLink transport는 M9 하나를 공유한다.

| 소유자 | 책임 |
|---|---|
| `connection.py` | role별 연결·해제·재연결, endpoint/identity 검증, connection generation |
| `network_roles.py` | stop owners→주소 적용→설정 commit/실패 종료의 거래. 기존 explicit Connect 정책과 core의 OS rollback을 보존 |
| `parameters.py` | parameter 작업과 readback, board rotation·MAG 설정 결과 검증 |
| `maintenance.py` | shell·MAVFTP·flash·reboot·USB recovery의 기존 명령별 작업 소유권. 필요 시 transfer와 firmware 작업을 별도 cohesive owner로 나눔 |
| `presentation.py` | UI 상태·후보·진행 표시와 명령 payload 생성 |

네트워크 역할 변경의 rollback을 connection 서비스와 양쪽에서 수행하지 않는다. 거래 소유자는 각 서비스의 stop 명령과 OS 적용 결과를 기다리고 host 역할을 저장한다. 성공 후 Pixhawk 연결은 사용자의 explicit Connect를 기다린다. 초기 자동 reconnect 가정은 기존 코드/테스트 계약에 따라 정정했다(검증 기록 M7). parameter, shell, RTCM이 기존 전송 잠금·exclusive command 경계를 우회하는 별도 MAVLink 연결을 열지 못하게 한다. Target discovery의 receive-only transport를 rig 명령 transport와 통합하지 않는다.

통과 조건: 잘못된 SYSID·role·endpoint 차단, endpoint 변경 중 이전 연결 완료 무시, parameter ACK와 readback 불일치 표시, 두 작업의 배타성, 중간 연결 실패 rollback, 종료 중 transfer 회수다. 네트워크 역할 변경·flash·reboot는 계획/소프트웨어 테스트 과정에서 실제 장비에 보내지 않는다.

기존 기반: `test_network_role_application.py`, `test_pixhawk_network_assignment.py`, `test_pixhawk_usb_transport.py`, `test_pixhawk_mavftp.py`, `test_pixhawk_firmware.py`, `test_workstation_pixhawk_gnss.py`.

## M8 카메라 기능과 연결 화면

대상은 [panel_camera.py](../../../panel_camera.py)다. `camera_feature/`에 settings snapshot, discovery/candidate selection, control connection/reactivation, status presentation, 연결 위젯 구성 요소를 분리한다.

M8은 사용자가 선택한 카메라·주소·설정과 control 명령 흐름을 소유한다. media process는 M3, 스트림 시작·준비·Stop은 M15가 소유한다. 같은 reconnect 상태를 두 곳에 두지 않는다. 연결 위젯은 설정·후보 snapshot과 명령만 받고 device·GNSS 서비스의 widget/worker 필드를 직접 받지 않는다.

`CameraFeature.dependency_names` 184개를 기능별 명령·snapshot 인터페이스로 줄인다. 조립에 필요한 widget 참조는 view owner에 둔다. 호환 panel 속성은 같은 소유자에 위임한다. 양쪽 카메라 바꾸기는 기존 side 의미와 녹화·측정 제한을 유지한다.

통과 조건: 후보 탐색 실패 후 수동 선택 유지, reactivation 중 재클릭, 부분 연결, 설정 변경 후 시작, 종료 중 callback, UI/backend 설정 일치다. 기존 `test_workstation_camera.py`, `test_camera_stream_recovery.py`, `test_panel_composition.py`를 사용한다.

## M9 Pixhawk 메시지와 전송

대상은 [workstation_core/pixhawk_attitude.py](../../../workstation_core/pixhawk_attitude.py)다. `workstation_core/pixhawk_link/`에 다음 계층을 만든다.

- `types.py`: attitude, GPS, MAG, relative GNSS 상태와 source/단위 계약. 기존 class identity가 필요한 reexport를 유지한다.
- `decode.py`: MAVLink·shell producer 값을 상태로 변환. 외부 clock이나 widget을 읽지 않는다.
- `timebase.py`: boot time wrap·reboot·timestamp provenance.
- `transport.py`: role별 연결, receive/send, endpoint ownership, peer/system identity, 기존 send/exclusive locks. socket과 serial을 다른 서비스에 넘기지 않는다.
- `commands.py`: 동일 transport를 통한 명령 응답 수집·deferred packet 처리. telemetry와 명령 응답이 서로의 메시지를 소비하지 않게 한다.
- `subscriber.py`: 기존 20 ms Qt polling 및 signal 계약, `mavlink_liveness.py`와 `pixhawk_radio_policy.py` 재사용.

자료형·decoder·timebase를 먼저 이동하고 transport·명령 배타성은 별도 slice에서 옮긴다. 이 단계에서 polling을 새 thread 구조로 바꾸지 않는다. raw payload와 optional timeline 생성도 같은 decoder 계약을 사용한다.

통과 조건: 독립 raw fixture의 q 순서·축·단위, 잘못된 SYSID·duplicate packet, boot-time wrap와 remote reboot, 명령 중 telemetry 수신·RTCM 주입, disconnect 후 stale 응답, lock 해제다. 기존 `test_workstation_pixhawk_gnss.py`, `test_workstation_telemetry.py`, `test_pixhawk_radio_policy.py`, `test_pixhawk_usb_transport.py`, `test_mag_selection.py`에 경계 사례를 추가한다.

## M10 GNSS 운용

대상은 [gnss_runtime_controller.py](../../../gnss_runtime_controller.py)다. `gnss_runtime/`에 아래 역할을 분리한다.

- `operation.py`: 기존 `GnssOperationManager`와 readiness 계산을 사용해 mode/config와 운용 전환을 소유한다.
- `ntrip.py`: GGA source 선택, credential 참조, 연결·실패·재연결. 기존 `gnss_network.py`, `ntrip_credentials.py`를 재사용하고 비밀값을 snapshot/log에 넣지 않는다.
- `base.py`: serial·survey-in·fixed-base transaction과 `FixedBaseInitializer` 조립.
- `corrections.py`: 기존 `RtcmDataPlane`·worker의 단일 runtime 소유자. route config, source 연결, 주입 worker retire/restart를 관리한다.
- `evidence.py`: 원래 measurement/run 문맥을 고정하고 기록 종료 시 drain한다. 새 run이 이전 chunk의 기록 경로를 바꾸지 못한다.
- `presentation.py`: 기존 갱신 주기와 상태·품질·통계를 표시한다.

192개 panel 참조를 이 계약들로 치환한다. RTCM byte 전달 경로를 UI snapshot 경유로 바꾸지 않는다. mode 변경의 운용 상태와 correction route 변경이 서로 다른 시점의 설정을 조합하지 않도록 하나의 명시적 전환 요청에서 고정한다.

통과 조건: 모드별 NTRIP/base→rover/drone 경로 유지, self-loop·중복 주입 방지, source stale·queue full·worker retire 중 callback, stop/start 사이 generation·drain, GGA 높이/시간 source 보존, credential 실패 뒤 기존 연결 정리다.

기존 기반: `test_workstation_ntrip.py`, `test_workstation_gnss_protocol.py`, `test_rtcm_pipeline.py`, `test_rtcm_pipeline_workers.py`, `test_rtcm_pipeline_end_to_end.py`, `test_rtcm_package_evidence.py`.

## M11 WIRIS 작업

대상은 [wiris_runtime_workers.py](../../../wiris_runtime_workers.py)다. `wiris_workers/`의 clock, network/RTSP enable, commands, photo capture, FTP sync/delete, image-format/zoom, stream probe로 기존 Qt worker 어댑터를 옮긴다. QObject의 QThread 배치와 signal receiver 문맥은 유지한다. 먼저 이미 독립적인 class를 이동하고, 큰 `WirisPnPPhotoCaptureWorker`의 command→busy poll→원격 파일 선택→download 흐름은 한 촬영 transaction 안에서 분리한다.

공통 취소·deadline·command/FTP 연결 처리는 좁은 client adapter로 공유한다. capture-only와 sync/delete를 합쳐 새로운 자동 삭제 정책을 만들지 않는다. 좌우 결과는 요청과 side를 유지하고 부분 성공·실패를 각각 보존한다. camera-side `/` 경로는 장치 계약이며 PC asset 상대경로와 구별한다.

통과 조건: 촬영 ACK만으로 파일 성공 판정 금지, 이전 사진 오인 방지, 한쪽 성공·한쪽 timeout, download 실패·취소, sync 뒤 삭제의 현재 조건 유지, worker 종료 후 지연 signal 처리다. 기존 `test_wiris_ftp_capture_source.py`, `test_wiris_ftp_browser.py`, `test_camera_time_sync_continuous.py`, `test_workstation_camera.py`의 가짜 응답과 별도 장치 시험을 구분한다.

## M12 광학 자세 화면

대상은 [workspace_views/optical_pose.py](../../../workspace_views/optical_pose.py)다. `workspace_views/optical_pose_parts/`의 frozen preview/annotation, method controls, suggestion acceptance, candidate details/combine, RTK observations/live reference로 나눈다.

화면은 선택·줌·스크롤 등 표시 상태만 소유한다. 승인·적용 가능한 DOF·기하 준비 여부는 M1 snapshot에서 받으며 solver를 호출하지 않는다. live reference와 frozen observation을 별도 값으로 유지하고, 새 burst·지연 추천·annotation 수정 사이 overlay source identity를 검증한다.

통과 조건: 잘못된 burst 추천 승인 차단, 선택한 후보·DOF 표시 보존, 방법 탭 전환 후 annotation·preview 크기 유지, RTK 관측 선택과 live 값 구분이다. 기존 `test_optical_pose_workspace.py`, `test_annotation_image_view.py`, `test_alignment_live_rt_ui.py`를 사용한다.

## M13 Pixhawk 화면

대상은 [workspace_views/pixhawk.py](../../../workspace_views/pixhawk.py)다. `workspace_views/pixhawk_parts/`의 endpoint/role connection, telemetry tables, magnetometer/parameters, GNSS source/baseline, geometry inspection으로 나눈다.

role과 candidate의 실제 identity 판단은 M7 snapshot을 사용하고, 자력계 readback·적용 상태는 해당 서비스에서 받는다. 화면의 선택 row index를 장치 identity로 보내지 않는다. geometry inspection은 M6 투영을 표시하고 자체 NED/FRD 변환을 만들지 않는다.

통과 조건: 후보 목록 갱신 후 선택 장치 혼동 방지, GPS1/GPS2와 P1/P2/drone 표기 보존, MAG 쓰기 진행·readback 상태, telemetry 갱신 중 선택·열 너비·scroll 유지다. 기존 `test_pixhawk_magnetometer_workspace.py`, `test_pixhawk_usb_transport_ui.py`, `test_drone_pixhawk_geometry_ui.py`를 사용한다.

## M14 메인 창과 명령

대상은 [main_window.py](../../../main_window.py)다. 729줄 `_build_commands()`를 기능별 명령 선언으로 나누고, 각 선언은 ID·payload·실행 핸들러·상태 feedback을 가진다. 기존 command registry를 재사용하며 새 범용 event bus를 추가하지 않는다.

`command_bindings/`에서 calibration, optical pose, devices/GNSS, camera, measurement/recording 명령을 등록한다. shell/navigation은 창에 남기고, optical auto-selection worker와 callback 상태는 M1의 응용 조립 소유자와 Qt 어댑터로 옮긴다. 창은 작업 의도를 보내고 결과를 표시한다. 이 이동과 함께 자원 목록·앱 종료 경로를 갱신한다.

통과 조건: command ID·shortcut·label·payload와 기존 허용/거절 상태 보존, 중복 handler·signal 연결 없음, 닫기 중 진행 작업 회수, 다른 창 경로에서 같은 기능 호출 가능이다. 기존 `test_workstation_navigation.py`, `test_workstation_ui.py`, `test_workstation_lifecycle.py`, `test_workspace_architecture.py`와 command inventory 비교를 사용한다.

## M15 카메라 스트림 수명주기

대상은 [camera_stream_controller.py](../../../camera_stream_controller.py)다. `camera_runtime/`의 startup readiness, measurement admission adapter, live recovery, stop/finalization, Qt presentation으로 나눈다.

`StreamLifecycle` 한 소유자가 side별 실제 시작 세대·준비·재시도·중지 상태를 관리한다. 준비 판정은 제안 설정이 아니라 실제 frame/recorder 시작 증거를 사용한다. 기존 readiness timeout·retry budget·부분 가용성 정책은 유지한다. Measurement Run의 최종 상태는 기존 `measurement_run.py`가 소유하며 camera 계층은 source 사실과 시작 요청만 전달한다.

Stop 소유자는 M3 finalization 결과와 실제 worker 종료를 확인해 video segment 결과를 기존 `recording/video_segments.py`에 전달한다. capture/control 연결과 stream 재시작의 조정은 M8 명령 계약으로 처리하고 timer를 중복 소유하지 않는다.

통과 조건: 중복 Start, 준비 중 Stop, 한쪽 늦은 첫 frame, 이전 세대 frame과 finalization, queue된 Measurement Start, 복구 중 다시 Stop, 기록 실패 중 관측 계속, 종료 watchdog과 실제 자원 정리다. 기존 `test_camera_stream_recovery.py`, `test_workstation_runtime_camera_status.py`, `test_measurement_run_controller.py`, `test_recording_shutdown_integrity.py`, `test_workstation_lifecycle.py`를 사용한다.

## G1 생성 MAVLink 코드

`workstation_core/mavlink_dialects/sasl_can_mag_dialect.py`는 수동 분리하지 않는다. 기존 firmware patch와 manifest를 출발점으로 XML 입력, PX4/pymavlink revision, generator 옵션을 기록하고 repo 내부 명령으로 재생성할 수 있는지 검사한다. 생성 순서·날짜 등 비결정 요소가 있으면 의미 비교 기준을 명시한다. 관련 기존 source는 `firmware/patches/`와 `firmware/px4_fmu-v6x_sasl_can_mag/MANIFEST.md`다.

통과 조건: 노트북 clone에서 외부 checkout의 미추적 수정 없이 재생성 가능, message ID·필드 순서·타입·CRC·wire payload 일치다. 생성 파일을 여러 손수 작성한 decoder로 대체하지 않는다. 이번 계획에는 firmware flash가 포함되지 않는다.

## 단계별 실행 순서

위험 우선순위와 실제 이동 순서는 구분한다. transport 계약을 먼저 고정해야 장치·GNSS controller를 안전하게 나눌 수 있고, 입력·명령 계약을 먼저 고정해야 화면을 얇게 만들 수 있다. 단계 내부도 아래 단위마다 검증하고 다음 의존 작업으로 넘어간다.

| 단계 | 작업 단위 | 선행 조건 | 확인할 결과 |
|---|---|---|---|
| S0 | 현재 호출·상태 쓰기·자원·자료형 소유권 지도, baseline tests와 대표 녹화·UI·성능 fixture | 현재 작업 트리 식별 | 기존 실패·skip을 분리하고 기능/좌표/자원 기준선 확보 |
| S1 | M9 types→decoder/timebase→transport/command→Qt subscriber | S0 | 기존 raw input과 신호·전송 배타성 동등 |
| S2 | M7 connection→network transaction→maintenance/parameters, 이어 M10 operation→source→correction/evidence | S1 | 장치·GNSS 제어에서 실제 자원과 상태 소유권 단일화 |
| S3 | M11 독립 worker→촬영 거래; 공유 active geometry owner; M2 inputs/quality→capture→solver→profile apply | S0, 장치 연결을 건드리는 부분은 S2 | 촬영 요청부터 적용·실패 복구까지 동일, 활성 profile 단일 소유권 |
| S4 | M1 frozen input→solve→observations/selection→apply/restore→Qt callback | S3 입력·작업 계약 | panel 없는 계산, stale 결과와 원래 관측 identity 보존 |
| S5 | M3 IPC/timing→recorders/backend→child/supervisor; M15 lifecycle; M8 settings/control/view composition | S0, S2 장치 계약, S3 WIRIS 계약 | 실제 frame 준비·재연결·Stop·finalization 연결 검증 |
| S6a | M4 spool/projection→generation/recovery→finalize/review | S0, 기존 기록 서비스 계약 | 저장 세대·파일·실패 복구 동등 |
| S6b | M4 canonical JSONL 기반 스트리밍과 bounded 파생 출력 | S6a | 저장 호환성 유지와 행 수별 메모리 근거 |
| S7 | M6 영역별 snapshot/query; M5·M12·M13 화면 구성요소; M14 명령·작업 소유권 이동 | 관련 S2~S6 계약 | 기능별 입력→결과→증거→UI 동일, 화면 상태 유지 |
| S8 | 전체 회귀, 대표 실행 흐름·장시간 replay·종료, 필요한 장치 검증, 호환 경계 축소, G1 재생성·portability | 해당 선행 단계 PASS | 수용 기준과 검증 환경별 한계 기록 |

S6a와 G1의 읽기·설계는 S0 후 독립적으로 진행할 수 있다. 단계 번호는 대형 일괄 패치나 여러 단계 동시 병합을 뜻하지 않는다. 하나의 검증 단위가 실패하면 그 단위를 사용하는 후속 변경을 멈추고, 관련 없는 읽기·분석은 계속할 수 있다.

## 완료 기준과 회귀 방지

1. 새 서비스에서 범용 panel 접근·타 기능 widget/private state 쓰기가 없다. 외곽 호환 binding에 남은 접근은 목록과 제거 조건을 기록한다. type-only import와 런타임 import를 구분해 허용 의존 방향·순환·역참조를 검사한다.
2. 상태·잠금·자원마다 단일 실제 소유자와 종료 경로가 있다. owner별 독립 생성/종료와 전체 앱 종료에서 누락된 thread/process/socket/file/lease가 없다.
3. 접수된 입력은 중첩 자료까지 고정된다. old/new generation, 중복 완료, timeout, 부분 성공, 취소·재시도, 원래 run 기록을 검증한다.
4. 원시 protocol fixture와 독립 좌표 기대값, golden 저장물 재열기를 통과한다. 기존 출력끼리만 비교하는 테스트로 좌표·wire 안전성을 인증하지 않는다.
5. 명령 ID·shortcut·payload·schema·hash·경로·알고리즘·quality 정책과 화면 선택/scroll/focus를 보존한다. 차이가 필요하면 근거와 별도 변경 범위를 먼저 기록한다.
6. 장치 장애·저장 실패·artifact 손상 등이 기존 허용 범위 밖의 다른 기능을 중단시키지 않는다. 소유자 하나의 실패를 전체 snapshot 실패로 확대하지 않는다.
7. 줄 수·함수 길이·의존 수 감소는 보조 지표다. 파일을 여러 개 만들었거나 기존 facade만 짧아졌다는 이유로 완료하지 않는다. 새 공유 state/context가 과거 panel 역할을 재현하면 실패다.
8. 좁은 snapshot·command와 구조 테스트의 양이 실제 위험 감소에 비례해야 한다. 단순 getter마다 새 서비스·새 테스트·wrapper를 만드는 과도한 조립 코드는 피한다.

## 검증 흐름과 이식성

각 slice는 `사용자 명령/입력 → 요청 고정 → worker/장치 어댑터 → 결과 검증 → 상태/저장 → 화면` 전체 연결 중 영향을 받는 부분을 실행한다. 단위 테스트 다음에는 해당 handoff를 확인하고, 구조 변경의 마지막에는 실제 앱 진입점에서 기존 사용자 흐름을 확인한다.

대표 흐름은 연결→양쪽 준비→관측 Start→profile 또는 설정 변경→지연 결과→Stop→저장·재열기→새 Run→앱 종료다. 별도로 캘리브레이션 capture/선택/solve/apply, 네트워크 역할 rollback, NTRIP 끊김·복구, 카메라 재연결, 저장 실패·retry를 실행한다. software-fake, loopback replay, 실제 media backend, 실장비 결과는 구분한다.

저장소 루트에서 기존 격리 runner로 대상 테스트를 먼저 실행한다. 아래는 구현 단계에 사용할 명령 예이며 이번 계획 수립 중 실행한 테스트 결과가 아니다.

```bash
apps/Workstation/tools/run_isolated_tests.sh -q apps/Workstation/tests/test_optical_pose_controller_integration.py apps/Workstation/tests/test_optical_pose_workers.py
apps/Workstation/tools/run_isolated_tests.sh -q apps/Workstation/tests/test_recording_raw_artifacts.py apps/Workstation/tests/test_session_parquet.py
apps/Workstation/tools/run_isolated_tests.sh -q -ra --tb=short
python3 tools/check_portable_paths.py
python3 tools/check_repository_policy.py --allow-existing-debt
git status --short
```

runner는 repo venv와 loopback-only network namespace를 요구한다. 지원 환경이 없으면 `tools/setup_workstation_test_env.sh`를 따라 준비하고, 실패를 일반 pytest 또는 다른 Python 경로로 우회해 성공이라고 기록하지 않는다. 실장비 명령과 firmware 작업은 loopback 테스트와 별도의 검증 활동이다.

새 필수 자원·설정·script는 SASL git tree에 둔다. Workstation 소유 asset 경로는 `SASL_WORKSTATION_DATA_ROOT` 상대 경로로 저장한다. 외부 단일 파일은 owned content-addressed 경로로 복사·hash 검증하고, 외부 큰 directory는 명시적 이전 또는 session-only 정책을 유지한다. `/dev`, camera-side `/`, 기존 제한된 임시 상태 등 명시적 장치·시스템 계약과 immutable provenance만 예외다.

## 이행과 되돌리기

기존 module 경로는 필요한 기간 동안 명시적 reexport/명령 위임으로 유지한다. 새 내부 코드는 실제 소유자를 import한다. facade는 상태·worker·계산을 소유하지 않고, supported caller가 없어지고 동일 기능·자원 시험이 통과하면 항목별로 제거한다. 기존 외부 소비자가 필요하면 호환 이름만 남긴다.

한 slice의 구현·직접 관련 테스트·문서를 하나의 검토 단위로 묶는다. baseline에 이미 있던 미커밋 변경과 분리 작업 patch를 구분해 보존한다. 실패 시 전체 tree reset 대신 해당 slice 변경만 되돌리고 다음 의존 단계로 가지 않는다. 상태 소유자는 옛 경로와 새 경로를 동시에 실행하지 않는다.

저장 schema/version·protocol·설정 형식을 변경하지 않아 이전 버전으로의 복귀가 기존 데이터 마이그레이션을 요구하지 않게 한다. 스트리밍 단계도 발행 전 staging만 만들고, 검증 실패는 기존 current와 spool로 돌아간다. 장치에 이미 적용된 네트워크/parameter 변경은 코드 rollback으로 복구되지 않으므로 기존 장치 readback·복구 거래를 따로 유지한다.

## 초기 검증 기록

| 항목 | 상태 | 근거와 다음 작업 |
|---|---|---|
| 범위·수치 기준 | 정적 확인 | 15개 대상 AST 파싱·줄 수·함수 수, baseline SHA-256 기록 |
| 기존 분리와 중복 여부 | 정적 확인 | `measurement/`, `recording/`, panel ports, lifecycle inventory, core 모듈·기존 테스트 확인 |
| 좌표·시간 계약 | 정적 확인 | `coordinate_contract.py`, `optical_pose.py`, `drone_pixhawk_geometry.py`, `pixhawk_attitude.py`, 기록 스키마 대조 |
| 문서 참조·기준선 | PASS | 계획의 로컬 링크 20개, 기존 테스트 파일 57개, 대상 소스 hash 16개 대조. 누락·변경 없음 |
| 경로 이식성 | PASS | 저장소 루트에서 `python3 tools/check_portable_paths.py`: `portable path policy: PASS` |
| S0 runtime·성능 기준선 | NOT_RUN | 구현 착수 시 현재 작업 트리에서 새로 측정 |
| S1~S8 및 G1 구현·회귀 | NOT_RUN | 본 문서는 계획이며 소스 변경·장치 조작 없음 |

구현 시 slice마다 입력/기대 결과, 보존 계약, 명령·artifact, 코드 identity·환경, `PASS/FAIL/BLOCKED/NOT_RUN`, 남은 위험과 다음 작업을 이 기록에 추가한다. 과거 결과를 덮어쓰지 않는다. 이전 분해 작업의 통과 수를 새 slice의 결과로 복사하지 않는다.

Coordinate contract checked: coordinate_contract.py, optical_pose.py, drone_pixhawk_geometry.py, pixhawk_attitude.py, RECORDING_SCHEMA.md; NED/body FRD/optical RDF/카메라 및 기준선 중심/시간·단위; 정적 검토, runtime tests NOT_RUN.
