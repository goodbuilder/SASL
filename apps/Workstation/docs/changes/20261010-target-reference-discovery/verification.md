# 검증 계획과 조사 기록

현재: SOFTWARE_ACCEPTANCE_PASS. 아래 초기 계획과 후속 단계별 이력을 함께 보존한다. 실장비 탐색은 수행하지 않았다.

| 기준 | 필요한 증거 | 상태 |
|---|---|---|
| T1 탐색 | 유효 CRC heartbeat만 후보, SYS가3이 아닌 기체, 송신 peer/수신포트 구분, 중복 USB 별칭 제거, 빈/부분 결과 | PASS — software; details below |
| T2 선택/설정 | 선택으로 연결·저장0회, Save/restart 경로, 저장실패 원래 값 보존, None/Any와 ID0 혼동 방지 | PASS — software; details below |
| T3 신원/상태 | 같은 UDP에 다른 IDs/같은 IDs 다른 peer, 연결 시 재검증, heartbeat만 있는 상태와 유효GPS/RTK 구분 | PASS — software; details below |
| T4 수신전용 | 모든 MAVLink 송신 메서드를 실패시키는 fake와 wire-level 송신0 확인, UAM worker/socket0 | PASS — software; details below |
| T5 소유권/수명 | 별도 프로세스가 점유한 UDP/serial, P1/P2/Telemetry active, 취소 직후 재시도·late result, 종료중단 자원0 | PASS — software; details below |
| T6 통합 | 실제 view/controller/loopback/PTY 흐름, 기록·좌표 회귀, UI focus/selection/scroll, 기존 source 계약 | PASS — software; details below |

먼저 isolated runner로 관련 시험: test_target_drone_controller.py, test_target_drone_stream.py, test_drone_network_discovery.py, test_measurement_package_ui.py, test_mavlink_sources.py, test_workstation_lifecycle.py, test_workstation_settings.py, test_panel_composition.py 및 신규 Target 탐색/화면/설정 시험. 실제 파일·선택 범위는 구현 시 확인한다. 다른 작업의 회귀 수치를 이 변경 결과로 재사용하지 않는다.

네트워크 시험은 tools/run_isolated_tests.sh의 namespace 격리에서 loopback 송신기를 사용하고 USB는 PTY/fake transport로 검증한다. 송신 금지 fake뿐 아니라 수신기 transport capture로 실제 write가 없는지 확인한다. peer 검증·exclusive bind·취소는 별도 OS 경계 검증이 필요하다. 단순 widget monkeypatch 통과만으로 연결 성공을 주장하지 않는다.

새 UI는 실제 shell에 붙여 탐색→선택→연결→Stop/Save/restart를 확인한다. 100/150% 배율과 긴 오류/다수 후보에서 화면 크기·table columns·focus·scroll을 비교한다. 이벤트 응답지연과 취소시간을 기록하고 앞서 제시한 자원 한도를 확인한다.

저장/설정 변경 후 root에서 python3 tools/check_portable_paths.py 및 관련 settings/runtime 시험, git diff --check, git status를 수행한다. serial device 경로는 명시적 machine-local 설정이며 새 자산 경로는 owned data root 상대 경로여야 한다.

실장비 USB open의 DTR/RTS 영향, 라디오 baud/무송신 heartbeat 수신, 멀티 NIC와 타 프로세스 포트 점유는 현장 조건을 기록해 별도 검증한다. 물리 송신 설정을 자동으로 수정하지 않는다.

조사 근거(읽기 전용): target/telemetry controller와 view, target_drone_stream, drone_network_discovery, pixhawk_attitude USB probe, 종료/resource owner 및 기존 시험. 좌표 producer 계약 WGS84/NED/body FRD와 camera RDF/center FRD 분리를 확인했다.

계획 문서 점검: portable path policy PASS, git diff --check PASS. 관련12개 소스의 작성 전후 SHA-256이 동일하다. 새 계획14파일은 저장소 내부의 untracked 상태이며 ignored 파일은 없다. 기능 구현·시험·실장비 실행 결과는 추가하지 않았다.

## S0 PASS / 구현 시작
계획의 source hash와 현재 소스 일치. 기존 미커밋 변경 보존. target telemetry WGS84/NED/body FRD 및 camera RDF/center FRD 계약 재확인. 테스트는 격리 runner, USB는 PTY/fake. 실제 사용자 확인용 창에 탐색 명령을 보내지 않는다.

## S1/S2 core PASS
격리된 UDP loopback+PTY 검증6개 통과(0.46s). 독립 고정 HEARTBEAT bytes/CRC, 손상 packet, 동일 ID 다른 peer, 독점 bind, occupied/partial, 취소, 송신0/serial 재개방 검증. Worker owner·UI 통합은 다음 단계.

## S3 integration finding
Target core/receiver/owner 및 기존 settings 그룹:55 PASS/1 FAIL. 실패는 기존 flat 설정 표면 검사에서 새 nested TargetReferenceSettings를 scalar로 센 분류였다. 별도 Target editor/원자저장 계약에 맞춰 nested field로 분류하고 전용 settings 저장·rollback 검증 추가. 동작 gate 실패를 숨기기 위한 skip 없음. 수정 후 재실행 중.

## S3 PASS
격리 Target discovery/stream/controller/settings56개 PASS(7.95s). raw UDP 기체 identity/peer와 GPS1 RTK freshness, GPS2 UNKNOWN, 취소 generation/중복, 후보선택과 저장/연결 분리, 원자저장 실패 rollback 확인. UI 연결 단계 진행.

## S4 PASS / 실제 버튼 경로
Target UI+architecture+discovery50개 PASS(4.84s). 실제 panel/backend에서 loopback 탐색→후보선택→connect→GPS UNKNOWN→stop→save 확인. 수신 peer 일치와 송신0, 기존 P1 점유 거부, inventory에 탐색thread 노출/종료 검증. 최초 UI 회귀109 PASS/1 FAIL은 함수 내 지연 import와 정적 DAG 기대 불일치였으며 concrete view import를 모듈 위로 옮겨 해결. 별도 subprocess 화면 캡처6개, unexpected callback0, 종료 활성자원0. Offscreen GL은 기존 환경제한. 상태문구 마지막 ATT가 잘리는 화면을 확인해2줄 고정 수신상태로 개선하고 재검증 예정.

## S5 integration checkpoint /148 PASS
Target/UI/settings/lifecycle/runtime/既存Discovery148개 PASS(19.42s). 검토 중 비동기 Stop의 Measurement 상태통지가 UI thread로 복귀해야 함을 확인했다. QObject queued relay를 추가하고 GUI thread affinity 회귀 검증을 넣었다. 최종 검증 후보에는 이 수정이 포함되어야 한다.

## 최종 집중 검증 PASS
Target discovery+실제panel/UI21개 PASS(2.83s). 별도 프로세스 UDP 독점검증, 만료후보거부, 이전 transport draft가 새 탐색을 막지 않는 경우 추가. 원래 flatSettings 테스트 분류와 static import DAG 회귀 수정 완료. 경로 검사에서 테스트용 /home literal이 검출되어 동일한 invalid-device 검증을 bounded /tmp 경로로 수정했다. runtime 정책 완화 없음.

전체 Workstation+shared geometry regression 실행 중: tools/run_isolated_tests.sh -q -ra --tb=short, 결과 로그 /tmp/target-reference-full.log. 화면 profile은1366×900/1600×980,100/150% 네 가지에서 실제 panel+shell+loopback source를 사용해 별도 확인 중이다.

## Visual QA environment correction
첫 Target capture matrix는 실제창/loopback접속/종료까지 완료했으나 이전 캡처와 동일한 offscreen pyqtgraph GL capability exception을 오류로 분류해 종료1. 기존 capture_refactor_views와 동일한 정확한 예외 종류/문구/stack path만 rendering_limitations로 분리했다. 다른 callback error는 계속 실패 처리한다. 실제 화면의 FOUND·0 candidates 표시도 handoff후 IDLE로 정리하고 GPS missing 표시를 간결하게 보완했다. 새 후보에서 재실행한다.

## 전체 회귀 중 발견한 settings 호환 결함
실패case: test_stage5_extracted_methods.py::test_workstation_settings_snapshot_merges_in_memory_overrides. 기존 최소 settings adapter에는 target_reference 속성이 없는데 새 snapshot이 이를 직접 읽었다. 실제 AppSettings에는 필드가 존재하지만 기존 호출계약을 보존하도록 기본 TargetReferenceSettings로 fallback한다. 기존 실패시험은 수정하지 않는다. 새 최종후보 재검증 필요.

## 자원 경쟁 검토 보완
GUI 점유 snapshot 이후 다른 탐색/연결이 시작하는 경쟁구간을 막기 위해 기존 Radio lease key를 공유하는 작은 serial_endpoint_lease helper를 추가했다. Target뿐 아니라 기존 USB probe와 Base GNSS worker/probe의 open/close 경계에 lease를 적용한다. 기존 프로토콜 송수신은 바꾸지 않는다. 이는 승인된 점유보호 범위이며 새 전체후보에 관련 회귀를 포함한다.

## S5 final boundary PASS
추가 공유serial lease 후 Target/Pixhawk/GNSS/RTCM/기존 settings adapter178개 PASS(15.78s). 최초 test_gnss_protocol.py 파일명을 잘못 지정한 실행은 collection전 exit4; 실제 test_workstation_gnss_protocol.py로 정정해 실행했다. 원래 전체후보는2264 PASS/1 FAIL/2 SKIP/1 warning(246.47s), settings 호환 실패로그를 evidence/intermediate-regression.log에 보존한다. 수정된 동일시험+인접56개 PASS(12.92s). 최종 source hash inventory를 고정하고 전체 회귀를 시작한다.

## 화면/재시작 software PASS
네 profile(1366×900,1600×980 × scale1/1.5) × UAM/빈 Drone/선택후보/heartbeat만 수신/긴 오류의20개 화면. 각 profile 내 Target 영역 geometry는 동일(758×448 또는992×448), window size 동일, unexpected callback0, shutdown complete. Artifact: evidence/target-views/*/summary.json. 이 GL 환경의 기존 pyqtgraph capability 제한만 rendering_limitations에 기록했다.

새 프로세스 두 개로 Save→프로세스 종료→재시작 UI 복원을 확인했다. SYS42/COMP1과 endpoint가 복원되고 configured=false/worker=false/탐색IDLE/후보0, callback0. 재현코드는 evidence/restart_smoke.py, 출력은 evidence/restart-smoke.log. 이는 공식 실행기 의존성 준비/실장비 연결의 통과 주장과 다르다.

## S6 final PASS
Command: `apps/Workstation/tools/run_isolated_tests.sh -q -ra --tb=short` → **2266 PASS,2 SKIP,1 warning,266.14s,exit0** including teardown. Raw log: evidence/final-regression.log. Source inventory437files SHA256 `59d307b6ca45e611a6c68c211478c9e45fa49c06b50b241235e1b8d9aab46065`, post-run changed=[]; evidence/final-acceptance.json records current acceptance.

T1–T6 software criteria pass with loopback,PTY,process,real Qt composition,settings rollback/restart and full existing regressions. No live hardware qualification inferred. NVENC/pyproj skips and installed SciPy/NumPy warning persist. Portable paths,repository policy,typing-stub drift and whitespace checks pass; required new files are in repo and will be checked for ignore status before final response.

최종 인계 검사: portable paths/repository policy/stub drift/whitespace PASS.437 source hashes 재확인 일치. 새 소스·시험·도구와 패킷의 모든 evidence는 저장소 안에 있고 ignored가 아니다. git status를 SASL root에서 확인했으며 미커밋 상태와 독립 Simulation 변경을 유지한다.
