# 대형 모듈 분리 상태 감사 — 2026-10-11

현재 최종 상태: 감사에서 남았던 **M2·M7·M8·M10·M15의 지정 책임까지 분리하고 검증했다.** 원래15개 대상의 고정된 소프트웨어 분리 범위는 마감했다. 생성 MAVLink(G1)는 재생성 체계를 유지한다. 최종 전체 회귀는 **2,696 passed / 1 skipped / 0 failed / warnings0, 327.64초**이며, 649개 Python 파일과 delegate stub의 검사 후 해시도 일치한다. [최종 실행 로그](remaining-completion-full-regression-2.log) · [검증 결과](remaining-final-verification.json).

아직 크기가 큰 조립 파일은 다음4개다. 줄 수만으로 분리 완료를 판단하지 않았으며, 아래 파일은 GUI/장치 어댑터·명령 조립을 유지하고 실제 상태·worker·저장·정책 소유자에 위임한다.

| 파일 | 현재 줄 수 |
|---|---:|
| calibration_controller.py | 3,380 |
| device_runtime_controller.py | 3,648 |
| gnss_runtime_controller.py | 2,557 |
| panel_camera.py | 2,414 |

실장비 운용, 장시간 자식/GPU 메모리, 지원되지 않는 NVENC는 별도 미검증 범위다. M4의 단일 행·metadata·동적 열·artifact inventory 크기 한계도 남는다. 따라서 모든 성능·운용 위험이 해소됐다는 판정은 하지 않는다.

새 필수 코드·도구·interface 파일333개가 현재 Git 미추적 상태다(이전 작업의 미추적 파일 포함). 모두 저장소 안에 있으며 ignored 또는 저장소 밖 필수 소스는 없다. 공유 clone에 포함하려면 함께 커밋해야 한다. 커밋·푸시는 실행하지 않았다. [파일 목록](remaining-git-portability.json).

후속 마감 내용:

- M2: 품질 최소 pose 판정, root/manifest 및 profile 저장·archive 정책을 기존 책임 소유자로 이관.
- M7: 불변 입력/결과 builder가 persistent Ethernet parameter·파일·명령 생성을 소유.
- M8/M15: control generation과 명시적 rejoin 거래가 recording/segment/media 재합류를 소유.
- M10: 불변 network admission과 operation/Base reset이 시작 정책을 소유.
- 별도4회귀: pure observation의 Qt 역의존 제거, native GUI 검사 Event barrier, camera fake를 실제 view/runtime owner에 연결. 기존 검사 기준은 유지.

아래 10/5 분류와4실패 기록은 **수정 전 감사 시점**의 근거로 보존한다.

사용자 요청에 따라 두 서브에이전트가 구현된 소유권과 미해결 책임을 각각 읽기 감사했다. 이 표는 `remaining-completion-source.json`의 **647개 Python 파일**을 기준으로 하며 현재 파일과 해시가 일치했음을 확인했다. 이후 수정은 검증 기록의 후속 절로 구분한다.

## 분리 구현이 적용된 10개

아래는 주요 책임·상태·자원이 새 소유자에 실제로 연결된 항목이다. 전체 회귀 통과 또는 모든 운용 위험 해소를 뜻하지 않는다.

| 항목 | 기존 대형 모듈 | 실제 분리 경계 |
|---|---|---|
| M1 | `optical_pose_controller.py` | `optical_pose/`: frozen capture, 관측 이력, 후보·승인, solver, profile 적용·복구, artifact·표시 |
| M3 | `workstation_core/yolo_stream.py` | `media_stream/`: decode backend, 추론, recorder, child runtime, supervisor·IPC |
| M4 | `workstation_core/recording_session.py` | `session_storage/`: canonical streaming, disk index, 세대 발행·rollback, Parquet·Excel·recovery |
| M5 | `workspace_views/calibration.py` | `calibration_parts/`: capture/input/quality/validation/preview 및 선택 상태 |
| M6 | `evidence.py` | `evidence_views/`: 불변 입력 기반 투영, 파일 query, bounded async query 수명 |
| M9 | `workstation_core/pixhawk_attitude.py` | `pixhawk_link/`: 해석·시간, 실제 transport/connection/locks, 명령 |
| M11 | `wiris_runtime_workers.py` | `wiris_workers/`: clock/network/commands/capture/FTP/probe, I/O·policy |
| M12 | `workspace_views/optical_pose.py` | `optical_pose_parts/`: annotation, 후보·추천, RTK 관측, method/live-reference |
| M13 | `workspace_views/pixhawk.py` | `pixhawk_parts/`: connection, attitude, GNSS, magnetometer, geometry 표시 |
| M14 | `main_window.py` | `command_bindings/`와 광학 자동선택 worker 수명 소유자 |

M1에는 아래의 순수 solver import 경계 실패가 남아 있다. 따라서 이 열의 “분리 구현”을 검증 완료로 읽으면 안 된다.

## 추가 책임 분리가 남은 5개

모두 작업·상태 소유권의 상당 부분은 이동했다. 미착수 모듈이 아니라 **남은 정책 또는 거래 조립**이다.

| 항목 | 기존 대형 모듈 | 감사에서 확인한 잔여 책임 |
|---|---|---|
| M2 | `calibration_controller.py` | 최소 유효 pose 수에 따른 품질 FAIL 결정, session root/manifest 및 일부 profile 저장 조립 |
| M7 | `device_runtime_controller.py` | `_persistent_pixhawk_eth_setup`의 SYSID/MAVLink parameter·`extras/net.cfg` 생성 정책 |
| M8 | `panel_camera.py` | `_finish_camera_rtsp_reactivation`의 recording 재시작·segment 할당·media 시작 거래 |
| M10 | `gnss_runtime_controller.py` | `start_gnss_network`의 assisted-base freshness/GGA/높이 admission 및 base 수집 상태 초기화 |
| M15 | `camera_stream_controller.py` 및 연결 adapter | M8과 공유하는 RTSP 재합류 generation·가용성 판단 거래. readiness/recovery/Stop/finalization 소유권은 이미 분리됨 |

기존 큰 controller가 남아 있다는 사실만으로 미완료라 판정하지 않았다. GUI 입력·표시·조립 adapter와 실제 업무 정책을 구분했다. 위 다섯 항목의 잔여만 후속 구현 범위로 고정한다.

## 별도로 남은 검증 문제

최신 전체 회귀: **2,642 passed / 4 failed / 1 skipped, 328.60초**. [원시 로그](remaining-completion-full-regression.log).

- `test_async_native_store_does_not_block_gui_timer`: timer 기대 5회에 실제 4회. 원인 미확정이며 임계값을 낮추지 않는다.
- `test_solver_imports_reject_live_ui_dependencies_in_fresh_process`: 광학 계산 leaf의 Qt 간접 의존성 검사 실패.
- `test_workstation_camera.py` 두 경우: legacy fake가 기존 panel class method를 직접 호출하여 새 view owner를 찾지 못함. 실제 owner를 통한 동일 행동 검증으로 연결해야 한다.
- NVENC 1건은 지원되지 않는 장치로 건너뜀.

GNSS row 중복 소유는 이 전체 실행 전에 수정됐으며, 관련 GNSS/RTCM/Recording/실제 화면 61개 검사가 통과했다. 화면 6개를 1366×900으로 렌더링했고 callback 오류 및 종료 후 thread/process/socket/shared-memory 잔존이 없었다.

M3의 기존 20초 before/after RSS slope FAIL 두 건은 보존한다. 별도 180초 실행은 기존 22개 기준을 통과했지만 부모 RSS만 다룬다. 자식·GPU 메모리와 장시간 실장비 운용은 미검증이다.

M4는 입력 행 수에 따른 주요 materialization과 context/index를 streaming·disk index로 옮겼다. 큰 단일 행, metadata, 동적 열, artifact inventory까지 항상 일정한 메모리로 제한됐다는 뜻은 아니다.

G1 생성 MAVLink 파일은 수동 분할 대상이 아니다. XML/XSD·라이선스·manifest·재생성 도구를 저장소에 두고 기존 byte-for-byte 재생성을 확인했다.

Coordinate contract checked: NED/FRD/RDF·native u/v·GNSS 높이·source/host clock 및 저장 schema 경계를 기준으로 감사했다. 실제 장비 명령은 실행하지 않았다.
