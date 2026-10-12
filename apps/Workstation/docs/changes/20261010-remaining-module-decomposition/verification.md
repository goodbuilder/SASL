# Workstation 후속 분리 검증 기록

## S1 자료형과 해석 경계

사용자 지시: "안전한 범위에서 수정 시작해". [계획](plan.md)의 S1 중 transport와 장치 제어를 변경하지 않는 자료형·메시지 변환·boot-time 추적 분리를 수행한다. 연결·송신·수신 순서, Qt polling, 잠금, worker, 장치 설정, 계산 공식과 저장 schema는 유지한다. 나머지 계획 단계는 미착수다.

기준은 [baseline.json](baseline.json)의 작업 트리다. `pixhawk_attitude.py`에는 기존 USB probe의 serial endpoint lease 수정이 있으므로 이를 보존한다. HEAD만으로 기준을 복원할 수 없다.

| 기준 | 기대 결과 | 확인 방법 |
|---|---|---|
| S1-A 자료형 | 기존 import와 새 import가 같은 class; 필드·기본값·frozen·pickle 식별자 보존 | 변경 전 fixture, dataclass·class identity·새 프로세스 직렬화 시험 |
| S1-B 메시지 해석 | 같은 raw 값의 축·단위·provenance·null·timeline 출력 유지 | 독립 NED·quaternion·GNSS·MAG fixture, 기존 telemetry/recording 시험 |
| S1-C 시간 | 기존 uint32 wrap·재정렬·remote reboot·지연 이전 epoch 판정 유지 | 독립 clock sequence와 기존 subscriber 통합 시험 |
| S1-D 의존성 | 새 leaf는 Qt·pymavlink·socket·subscriber를 import하지 않음 | 새 interpreter에서 금지 import 검사, headless 계산 사용 |
| S1-E 실행 경계 | 기존 subscriber·명령·USB lease·타이머 본문 불변 | 변경 전후 AST 비교, 격리 회귀 시험 |

좌표 검토: 자세는 body FRD→world NED quaternion wxyz, Euler deg와 입력 rad 구분. GPS_RTK의 NED a/b/c와 ECEF baseline을 구분하며 NED만 heading을 제공한다. LOCAL_POSITION_NED x/y/z는 N/E/D alias로 유지한다. MAG는 원본 topic 축과 mG/G, RAW_IMU는 producer 단위를 유지한다. host monotonic s, boot ms, source us와 GPS UTC를 혼합하지 않는다. 카메라 RDF·baseline-center FRD 변환과 UI/저장 schema는 변경하지 않는다.

### 변경 전 기준선

최초 sandbox 실행은 netlink 권한 오류로 namespace 검사를 통과하지 못해 테스트가 실행되지 않았다. 같은 loopback-only runner를 권한 승인 경로로 재실행했다. 외부 장치 접근을 허용하는 환경 변수나 일반 pytest 우회는 사용하지 않았다.

저장소 루트에서 실행:

```bash
apps/Workstation/tools/run_isolated_tests.sh -q -ra --tb=short apps/Workstation/tests/test_workstation_pixhawk_gnss.py apps/Workstation/tests/test_workstation_telemetry.py apps/Workstation/tests/test_pixhawk_radio_policy.py apps/Workstation/tests/test_pixhawk_usb_transport.py apps/Workstation/tests/test_drone_pixhawk_geometry.py apps/StereoWIRIS/tests/test_coordinate_contract.py apps/StereoWIRIS/tests/test_stereo_geometry_common.py apps/StereoWIRIS/tests/test_triangulation.py
```

결과: **PASS — 155 passed in 8.16s**, exit 0. 저장소 Python venv, Qt offscreen, loopback-only network namespace에서 실행한 소프트웨어 시험이다. 실장비 검증 결과가 아니다.

### 구현과 후속 검증

현재 상태: **이번 S1 부분 범위의 소프트웨어 검증 완료**. 아래에 변경 후 검사와 실패·수정·재실행 근거를 순서대로 보존한다. Transport·장치 제어 분리 및 계획의 후속 단계는 미착수다.

자료형 9개, 해석 관련 심볼 41개, 시간 추적 심볼 12개를 Qt/transport 없는 `pixhawk_link` leaves로 이동했다. 기존 top-level 함수·class 83개의 AST 본문은 변경 전과 같으며, subscriber와 USB lease/명령 본문도 이 비교에 포함된다. [추출 목록](s1-extraction.json)을 보존한다. `alignment_managers.py`는 실제 leaf 소비자로 직접 import하도록 변경했다.

첫 변경 후 시험은 기존 기준선 목록에 `test_pixhawk_link_boundaries.py`와 `test_pixhawk_magnetometer_workspace.py`를 더해 실행했다. 결과는 **FAIL — 1 failed, 209 passed in 8.30s**다. 실패는 새 시험 `test_wxyz_body_frd_to_ned_axes_have_independent_known_euler_values`의 pitch −90도 사례였다. Euler singularity에서 유일하지 않은 roll/yaw를 0도로 강제했으나 기존 수식은 동등한 `(180, -90, 180)`을 반환한다. 원래 수식 AST는 동일하다. 시험을 일반 pitch −45도 사례와 singularity의 독립적인 `Rz * Ry * Rx` 기저 확인으로 나누며, 생산 수식은 변경하지 않는다. 기존 시험과 새 headless/직렬화 시험에는 실패가 없었다.

보완 후 동일 목록 재실행: **PASS — 211 passed in 8.50s**, exit 0. 신규 경계 시험은 25개다. 9개 dataclass의 변경 전 protocol-4 pickle/필드 fixture는 [테스트 fixture](../../../tests/fixtures/pixhawk_link_legacy_types.json)에 보존한다. 이전 구현과 비교한 직렬화 호환성 증거와 독립적인 좌표·단위 기대값 시험은 별개다.

정적 검사: `python3 tools/check_portable_paths.py`와 `python3 tools/check_repository_policy.py --allow-existing-debt` 모두 PASS. 새 leaf의 미해결 global 이름 없음, 새 Python 파일 syntax 검사와 `git diff --check` 통과. 전체 격리 회귀는 다음 명령으로 진행 중이다.

```bash
apps/Workstation/tools/run_isolated_tests.sh -q -ra --tb=short
```

첫 전체 결과: **FAIL — 1 failed, 2292 passed, 1 skipped, 7 warnings in 241.41s**, exit 1. 실패는 기존 `test_camera_time_sync_continuous.py::test_stus_post_io_deadline_preserves_left_ack_and_skips_right`다. 기대한 left STUS fake 호출이 없었다. traceback은 백그라운드 `measurement-data-writer`가 이 시험의 `lambda: next(monotonic_values)`를 호출해 `StopIteration`으로 종료했음을 보여준다. 시험이 worker의 모듈 참조 대신 공유 stdlib `time.monotonic` 함수를 바꾸어 두 값짜리 iterator를 전체 프로세스에 노출한 것이 원인이다. 해당 STUS runtime과 기록 writer는 이번 분리에서 변경하지 않았다.

검증을 안정화하기 위해 이 시험만 보완한다. `wiris_runtime_workers.time` 바인딩에 local clock object를 주고, fake I/O 응답 시각에서 99.9→100.1초로 전환한다. stdlib clock은 원래 함수와 같은지 검사한다. left ACK 보존·right 미전송이라는 기존 기대 결과는 그대로다. 전역 시계를 무한 iterator로 바꿔 증상을 숨기거나 생산 deadline을 완화하지 않는다.

skip은 NVENC 지원 장치가 없는 환경의 `test_camera_stream_recovery.py:284`이며 실장비 media backend 미검증으로 유지한다. 7개 warning은 위 전역 시계로 종료된 기록 thread의 예외 보고다. 수정 후 대상 시간 동기 시험과 전체 회귀를 다시 실행한다.

시계 fixture 수정 후 대상 재검증: **PASS — 67 passed in 1.74s**, exit 0, warning 없음.

```bash
apps/Workstation/tools/run_isolated_tests.sh -q -ra --tb=short apps/Workstation/tests/test_camera_time_sync_continuous.py apps/Workstation/tests/test_pixhawk_link_boundaries.py
```

### 최종 결과

같은 전체 격리 명령을 다시 실행한 결과는 **PASS — 2293 passed, 1 skipped in 261.11s**, exit 0, warning 0이다. 첫 전체 실행의 STUS 실패와 기록 thread 예외는 재발하지 않았다. 최종 [로그](s1-full-regression.log)와 [소스 identity·추출 목록](s1-extraction.json)을 보존한다. 여러 시험 실행의 통과 수를 합산하지 않는다.

| 기준 | 판정 | 근거 |
|---|---|---|
| S1-A 자료형 | PASS | 9개 legacy dataclass의 field 순서·값·frozen·protocol-4 pickle bytes·class identity, 새 interpreter 복원 |
| S1-B 메시지 해석 | PASS | 독립 attitude/NED·GPS RTK·GPS 품질·MAG·nonfinite fixture와 기존 telemetry→evidence/recording/UI 회귀 |
| S1-C 시간 | PASS | explicit boot-time wrap·reorder·reboot·오래된 epoch sequence와 기존 subscriber 통합 회귀 |
| S1-D 의존성 | PASS | Qt/pymavlink/compatibility runtime import를 거부한 새 interpreter에서 leaf와 MagnetometerMonitor 실행 |
| S1-E 실행 경계 | PASS | 기존 함수·class 83개 및 module-level 값·상수 38개 AST 보존, 전체 격리 회귀 |
| 실제 NVENC | NOT_RUN | 현재 지원 장치 없음으로 기존 시험 1개 skip; 이번 분리 대상 밖의 환경 한계 |
| 다른 장비·현장 성능 | NOT_RUN | 실제 연결·명령·장시간 현장 동작을 이 소프트웨어 시험으로 인증하지 않음 |

`pixhawk_attitude.py`는 **3,539→2,428줄**이다. 새 파일은 `pixhawk_link/types.py` 191줄, `decode.py` 878줄, `timebase.py` 208줄, `__init__.py` 5줄이다. 같은 구현을 복제한 것이 아니라 원래 정의를 이동했고, 기존 경로는 명시적 alias를 제공한다. Subscriber/transport가 남아 있으므로 M9 전체 분리가 끝났다고 판단하지 않는다.

기존 함수·class 본문 비교에는 사전에 들어 있던 `probe_usb_mavlink_devices()`의 serial endpoint lease 보호도 포함되며 이를 보존했다. 후속 단계는 S1의 transport/command 소유권 지도를 별도로 고정한 뒤 진행한다. 이번 검증만으로 그 변경까지 완료됐다고 보지 않는다.

문서는 `docs/ARCHITECTURE.md`, 이 계획과 검증 기록을 갱신했다. **README.md update not required for this change.** 실행법·명령·공개 import·runtime 계약을 유지하므로 사전에 수정되어 있던 `Workstation/README.md`를 이 작업에서 변경하지 않았다.

Git root에서 상태를 확인했다. 새 `pixhawk_link/`, 경계 시험과 legacy fixture, 후속 분리 문서는 아직 untracked이며 함께 git에 포함해야 clone에서 실행·검증할 수 있다. 커밋·push는 수행하지 않았다. 새 runtime 의존성이나 저장소 밖의 필수 파일은 없다. 검증용 임시 원본은 비교에만 사용했고 실행·테스트는 저장소 안의 파일을 사용한다.

Coordinate contract checked: pixhawk_link types/decode/timebase, alignment_managers import boundary; body FRD→NED wxyz, NED x/y/z alias, source MAG units, GNSS baseline frames and clock units; 독립 좌표 fixture와 전체 격리 회귀 PASS.

## 계속 진행과 S1 전송 소유권

사용자가 단계 완료 뒤 중단한 이유를 두 번 지적했다. 단계 구분은 내부 검증 절차이며 작업 종료 조건이 아니다. 남은 계획은 기존 승인 범위에서 계속 진행하고, 요구 계약에 실패한 선행 단계나 실제 외부 제약만 의존 작업을 막는다.

S1의 다음 경계는 wire 명령·endpoint I/O·연결 수명이다. 동일 연결의 `recv_match`, deferred telemetry, 수신 예약 잠금과 side별 송신 잠금은 하나의 transport 소유자에 둔다. Qt timer와 source stream 정책은 구분하되 콜백의 실행 문맥·수신 시각·연결 세대·RTCM 자격 확인 순서는 유지한다. 단순 method mixin이나 전체 subscriber를 받는 helper는 사용하지 않는다. Narrow callbacks는 명령/메시지 전달과 source reset만 연결하며 가변 resource state를 복제하지 않는다.

선행: S1 자료형/해석/시간 경계 PASS. 신규 기준: command/telemetry 경합, busy/실패 시 잠금 해제, 단일 연결·lock identity, partial connect cleanup, serial lease, 늦은 telemetry의 원래 수신 시각, stale generation RTCM 거절. 좌표 변환·wire bytes·단위·정책 상수는 변경하지 않는다. 실제 장치에는 테스트 명령을 보내지 않고 기존 격리 runner를 사용한다. 상태: IN_PROGRESS.

복잡도 판단: KEEP — 연결/잠금 소유권을 위해 Qt와 독립된 transport owner와 explicit callback 경계가 필요하다. 새 event bus, 새 worker, 전역 forwarding proxy는 도입하지 않는다.

```yaml
architecture_refresh:
  status: NOT_REQUESTED
  risk: MEDIUM
  affected_nodes: [PixhawkSubscriber, PixhawkTransport, PixhawkCommands]
  reason: resource and command ownership separation
```

S1 wire/endpoint 준비: `policy.py`의 기존 상수 26개, `wire_commands.py`의 함수 21개, `endpoints.py`의 함수 9개를 이동했다. USB probe lease의 import는 실제 core 경로로 바꾸었다. 정적 global 검사에서 MAVFTP helper의 `Path` import 누락을 발견해 보완했다. 기존 Pixhawk/liveness/radio/USB/자료형 경계 시험 **175 passed in 7.77s**.

S1 transport 이동: 연결·수신 예약·송신 잠금·deferred packet·liveness·boot epoch 등 22개 상태 필드와 17개 동작을 `MavlinkTransport`가 소유한다. Qt timer와 source/stream 상태는 subscriber에 남고, 8개 필수 callback이 두 역할을 연결한다. 기존 필드명은 같은 소유자의 객체를 가리키는 명시적 property다. 새 내부 모듈은 compatibility facade를 import하지 않는다.

첫 transport 시험은 **1 failed, 210 passed in 11.60s**. `test_subscriber_start_guards_and_hardens_udpin_boundary`가 이동 전 facade의 private helper를 monkeypatch하여 실제 새 endpoint 호출을 관찰하지 못한 실패다. 이 시험과 같은 주입 방식의 liveness 시험은 mock 주입 대상을 `pixhawk_link.endpoints`로 옮겼고, 기존 postcondition/assertion은 유지했다. 실제 연결을 연 것이 아니라 guard/hardening helper와 fake pymavlink factory의 호출 경계를 검증한다.

필수 callback 누락, 자원/lock identity, 부분 연결 cleanup, 명령 실패 후 receive reservation 해제, 지연 telemetry 수신 시각 보존, headless transport 사용 시험을 추가했다. Pixhawk/liveness/radio/USB/RTCM/Target discovery까지 포함한 재검증은 **232 passed in 12.17s**. 후속 작업은 subscriber의 명령 흐름을 좁은 transport 작업 계약으로 연결하는 것이다. 이 중간 통과는 작업 종료 조건이 아니다.

S1 command 조립: `CommandIO`, 필수 동작 3개의 `CommandPorts`, `MavlinkCommands`로 PARAM/Shell/RTCM/FTP/reboot의 10개 조립 동작과 RTCM 상태 출력 시각을 이동했다. 연결/잠금은 복제하지 않고 transport에서 빌린다. PARAM의 전체 수신 예약·요청 시점 송신 잠금, Shell의 기존 caller-owned reservation 계약을 유지한다.

첫 command 시험은 **2 failed, 219 passed in 11.35s**. RTCM 통계 시험은 이전 subscriber의 health override 대신 새 command health port에 같은 fixture를 제공해야 했다. Stream setup도 raw facade helper를 부르던 마지막 경로를 명령 서비스의 `exchange_shell`로 연결했다. 기존 전송수·메시지·qualification 및 명령 문자열 assertions는 유지했다. 보완 후 **221 passed in 10.83s**.

앱 자원 목록은 이미 생성된 `mavlink_pixhawk.resource_inventory()`도 합성한다. 조회 때문에 subscriber를 생성하지 않는다. 실제 panel에서 연결 객체가 transport와 동일하며, 종료 후 목록에서 제거되는 시험을 추가했다. Panel/lifecycle/contracts/boundary 시험 **108 passed in 13.62s**. 새 source의 undefined name 검사와 diff whitespace 검사 PASS. 전체 격리 회귀를 실행하며 다음 소유권 경계 조사를 계속한다.

S1 transport/command 전체 격리 회귀: **PASS — 2298 passed, 1 skipped in 247.34s**, exit 0, warning 0. Skip은 앞선 실행과 같은 NVENC 환경 한계다. `check_portable_paths.py` 및 `check_repository_policy.py --allow-existing-debt` PASS. 이 결과를 기준으로 S2 장치 작업 소유권 분리를 이어간다.

## S2 장치 연결 작업의 수명

기존 연결/해제 worker는 panel의 네 개 thread/worker 필드에 저장되고 완료/종료 콜백은 현재 panel을 다시 조회한다. 다음 slice는 worker 생성·시작·완료·실패·실제 thread 종료와 작업 식별자를 한 소유자에 묶는다. 기존 preflight, 장치 설정, UI 문구·버튼 정책은 이 slice에서 유지한다. QThread 부모와 GUI callback 실행 thread, shutdown inventory, 중복 Connect/Disconnect 거절, 부분 연결 결과와 기존 USB lease 계약을 검증한다. 상태: IN_PROGRESS.

`device_runtime/connection_jobs.py`의 소유자와 GUI-affine job relay, `device_runtime/state.py`의 네 개 legacy field alias를 추가했다. 완료·실패는 첫 terminal outcome만 전달하고, 현재 ticket/source가 아니면 UI 반영을 하지 않는다. QThread.finished 뒤에만 해당 job의 handle을 비우며, 늦은 예전 retire가 새 job의 handle을 지우지 않는다. 기존 두 Connect/Disconnect entrypoint는 preflight와 표시를 유지하고 실제 worker 조립·수명을 이 소유자에 맡긴다.

첫 시험은 **1 failed, 163 passed in 16.86s**. 구조 검사가 `DiagnosticsFeature.__init__`의 `self.state` 재할당을 거절했다. Allowlist를 확대하지 않고 `PanelFeature`의 생성 시점 state factory로 실제 소유자를 지정하도록 수정했다. 다른 feature의 기본 SimpleNamespace 상태는 그대로다. 실 thread를 통한 GUI delivery, busy 거절, 늦은 결과·중복 결과·stale source, factory 실패, 상태 alias/inventory를 포함한 재실행은 **218 passed in 19.91s**. 전체 회귀를 실행한다.

S2 connection job 전체 격리 회귀: **PASS — 2303 passed, 1 skipped in 250.38s**, exit 0, warning 0. Skip은 기존 NVENC 환경 한계다. 두 repository 정책 검사도 PASS. M7 전체 제어기 분리는 아직 진행 중이며, 이후 calibration/camera에서도 필요한 M11 worker 경계를 독립적으로 분리한다.

## M11 WIRIS worker 경계

현재 12개 Qt worker class는 각자의 작업 설정과 장치 I/O 수명을 이미 소유하지만 하나의 모듈에 모여 있다. Clock, network, capture, FTP, commands, stream probe로 class와 내부 state를 함께 옮기고, stateless effect adapter와 protocol constants만 공유한다. 기존 facade는 동일 class를 재수출한다. Worker 수, 부모/Qt signal, 실행 함수와 실패·취소·timeout 조건은 유지한다. 기존 private I/O/constant monkeypatch는 실제 새 adapter/policy 주입점으로 이동하되 기대값을 유지한다.

좌표·시간 검토: 영상/사진은 기존 좌우 source/frame identity·pixel/image byte contract를 유지하며 RDF/NED 변환을 새로 수행하지 않는다. LRF는 기존 metres 계약이다. STUS UTC seconds/microseconds, RTT seconds, settle milliseconds, host monotonic deadline을 구분한다. 촬영 metadata, hash, FTP 경로와 부분 실패 결과 형식은 유지한다. 상태: IN_PROGRESS.

12개 worker를 6개 실제 소유자 모듈로 이동했다. `wiris_runtime_workers.py`는 54줄의 명시적 호환 export이며 QObject state나 실행 함수를 갖지 않는다. I/O 함수 7개는 stateless adapter, 상수와 parser/barrier helper는 policy 모듈에서 제공한다. 12개 class의 AST는 해당 namespace prefix만 정규화하면 원본과 같다([inventory](m11-extraction.json)). 원래 caller의 thread/parent/signal wiring은 유지한다.

효과·policy mock 주입 지점만 실제 새 모듈로 옮긴 기존 회귀는 **314 passed in 28.20s**. 이어 7개 생산 consumer가 실제 worker owner를 직접 import하도록 이행했다. class identity, capture 상속, facade에 behavior 없음, 독립 interpreter의 역참조 금지와 clock 취소 시험을 추가한 재검증은 **385 passed in 32.82s**. 전체 회귀를 실행한다. 원래 큰 PnP capture class는 한 요청의 command→파일 선택→download 거래를 소유하는 하나의 owner로 유지하며, 작업 상태를 여러 객체에 복제하지 않는다.

M11 전체 격리 회귀: **PASS — 2315 passed, 1 skipped in 257.36s**, exit 0, warning 0. 기존 NVENC skip과 두 policy 검사 PASS를 확인했다. 이 경계는 완료됐으며, 후속 calibration/optical/camera controller 분리는 실제 worker owner를 사용한다.

## S6a 저장 책임 분리

저장 schema/값 codec, spool, canonical JSONL, generation publish/rollback, recovery, derived review/writers와 최종 조립을 분리한다. 실제 `SpoolRows`의 lock·owner_epoch·write-error/volatile 상태를 class와 함께 이동하며 복제하지 않는다. Publish→readback→spool 해제 순서를 그대로 유지한다. Source function/class와 상수는 module dependency prefix만 바꾸며 메모리 방식은 S6b에서 별도 검증한다.

좌표 검토: `baseline_center_forward/right/down_m`는 optical baseline-center FRD, bbox x/y는 좌우 이미지 pixels, GNSS 위경도는 degrees, ellipsoid/MSL 높이와 NED velocities는 별도 필드다. LOCAL_POSITION_NED x/y/z는 N/E/D alias이고 MAG x/y/z는 source/topic 단위를 유지한다. 원래 record_json의 frame/axis/provenance를 보존하며 camera/center/GNSS midpoint 사이 변환을 새로 하지 않는다. recording/UTC/source/host clocks와 ns/s 변환도 보존한다. 기존 projection·schema·readback·fault injection 시험으로 확인한다. 상태: IN_PROGRESS.

첫 저장 회귀는 **22 failed, 52 passed in 4.39s**. 123개 정의의 AST를 보존했지만 module 초기화의 `for _method_result_id in MEASUREMENT_METHOD_IDS` 등록 loop가 추출 목록에서 누락되어 `method_result_pixhawk_rtk_live` 경로 조회가 실패했다. 정의 비교만으로 module 실행 초기화를 검증할 수 없다는 구체적 누락이다. 원래 loop를 schema owner에 그대로 복원하고 전체 runtime method registry→authoritative stream catalog 검사를 추가한다. 다른 두 이전 추출 원본에도 정의 밖 executable statement가 있는지 검사했으며 해당 사례는 없었다. 저장 순서·공식·경로 규칙을 새로 변경하지 않는다.

복원 후 저장 기능은 통과했고 **2 failed, 75 passed in 17.19s**가 남았다. 두 실패는 parameterized `fault_target` 이름을 사용하는 기존 JSONL fault injection이 옛 facade를 patch하고 있었기 때문이다. 실제 canonical owner에 동일 오류를 주입하도록 변경하며 `raises`, 기존 pointer/data bytes, retry spool 보존 assertions는 모두 유지한다.

기존 저장/복구/좌표 저장 시험 **77 passed in 16.35s**. 이후 12개 생산 consumer가 실제 소유자를 import하도록 이행했다. Legacy class identity와 type hint, 실제 save/recovery 진입점, method stream catalog, 새 interpreter에서 Qt·Arrow·Excel·facade 없이 spool을 사용하는 경계 시험을 추가/확인했다. 실제 기록 서비스·저장·계약·좌표 소비자 시험 **64 passed in 13.04s**. 전체 회귀를 실행한다.

복잡도 판단: KEEP — spool을 파생 writer와 분리하고 canonical/publish/recovery의 오류 경계를 명시하기 위한 작은 storage modules. REMOVE — 외부 사용처가 없는 facade의 `_LAST_SESSION_ID_NS`/`_SESSION_ID_LOCK` 사본·alias; 실제 단일 ID 생성 상태는 paths owner에만 둔다. S6b의 streaming 변경은 아직 미착수이며 현재 메모리 개선을 주장하지 않는다.

S6a 전체 격리 회귀: **PASS — 2319 passed, 1 skipped in 255.98s**, exit 0, warning 0. Canonical 저장·retry·published generation rollback·좌표 저장 및 실제 앱 소비자 연결을 포함한다. S6b에서는 우선 파생 Parquet를 위해 만드는 pair/camera/PX4 전체 행 사본을 제거한다. 원본 snapshot과 저장/검증 순서는 유지하면서 기존 bounded Parquet writer에 행별 사본을 전달한다. 원본 materialization·context/index·Excel의 메모리도 별도로 남아 있으므로 이 한 변경으로 전체 저장 메모리가 bounded라고 주장하지 않는다.

S6b 첫 변경은 기존 camera/PX4 iterator와 같은 pair iterator를 사용해 파생 projection의 행 사본을 필요할 때만 생성한다. Source canonical row는 수정하지 않으며 기존 seconds→ns 반올림·별도 시간축을 유지한다. Lazy consumption·원본 불변·독립적인 ns 기대값과 저장/Parquet/readback/좌표 계약 시험 **62 passed in 13.05s**.

`tools/qualify_session_projection_memory.py`는 실제 save 경로에 synthetic 5 streams를 공급하고, 별도 process의 이전 eager allocation을 비교한다. 10,000 rows/stream 사례: eager **285.070 MiB / 11.079 s**, lazy **240.086 MiB / 11.025 s**. 이 한 사례의 peak RSS는 44.984 MiB 감소했다. 처리 속도 일반화나 전체 저장 메모리 상한의 증거는 아니며 review.xlsx는 정상 measurement 경로처럼 deferred다. 더 큰 행 수 사례를 확인한다.

## M6 수집과 표시 투영 경계

먼저 attitude/GPS/detection의 panel 조회와 순수 표시 투영을 나눈다. 외곽 adapter가 기존 source 선택, MAVLink provenance와 frozen-GPS 증거를 수집하고, 명시적 field snapshot과 now를 projector에 전달한다. Projector는 panel/widget/worker/clock을 읽지 않는다. Quaternion wxyz body FRD→NED, WGS84 deg와 높이 m, 영상 u/v pixel 및 source/receive clock 구분을 그대로 유지한다. 이 slice는 전체 EvidenceAdapter와 파일 query의 분리가 끝났다는 뜻이 아니다.

기존 관련 회귀 **165 passed in 12.21s**. 분리된 입력의 원본 변경·읽기 전용 중첩 값, WGS84 높이 종류/µs와 receive s, bbox u/v px, fresh interpreter의 GUI/facade 역참조 금지를 추가 확인한 결과 **176 passed in 12.51s**. 출력 필드/선택 규칙과 숫자 변환은 유지했다. 전체 회귀를 S6b projection 변경과 함께 수행한다.

S6b 두 번째 synthetic RSS 사례(5 streams, 20,000 rows/stream): eager **384.613 MiB / 21.869 s**, lazy **306.195 MiB / 21.492 s**. Peak RSS 78.418 MiB 감소. 두 행 수 모두 파생 사본 제거 효과를 보였으나 원본 materialization은 여전히 행 수에 비례하며, deferred Excel·전체 bounded memory 완료로 해석하지 않는다. 재현 명령은 `python -B apps/Workstation/tools/qualify_session_projection_memory.py --rows 20000 [--eager-reference]`이며 저장소 venv에서 각 strategy를 별도 process로 순차 실행했다.

M6 첫 projector 경계와 S6b 첫 projection-memory 변경의 전체 격리 회귀: **PASS — 2326 passed, 1 skipped in 247.82s**, exit 0, warning 0. 다음 독립 경계인 M1 자동 선택 입력/계산 분리를 진행한다.

## M1 자동 선택의 입력과 계산

기존 job은 전체 OpticalPoseController 복제본을 담고 그 복제본도 live panel을 참조한다. 이를 frozen native image/artifact, 이미 해석한 K,D·prior pose·적용 경고·capture-associated GNSS baseline와 scalar detector 설정을 받는 계산 소유자로 바꾼다. 기존 core detector와 suggestion 출력/승인 규칙은 유지한다. GUI 경계에서 작은 입력 해석을 완료하고 OpenCV 검출은 기존 worker thread에서 수행한다. 원본 image/payload/profile/패널이 이후 바뀌어도 같은 작업 입력은 변하지 않아야 한다.

좌표 계약: prior rotation은 optical RDF→NED, gravity `(0,0,1)`은 NED Down이고 detector expected gravity는 optical RDF다. Baseline은 right-minus-left GNSS NED metres, relative rotation은 right-from-left RDF, 결과 점은 native image u/v pixels다. Source frame/hash와 application-time UTC output metadata를 구분한다. 상태: IN_PROGRESS.

`optical_pose/auto_selection.py`가 frozen frame/artifact, immutable profile/prior, baseline/details/warnings와 scalar settings를 소유한다. Detector 함수 두 개만 명시적으로 받으며 panel/controller를 보관하거나 import하지 않는다. QImage는 캡처 시 복사하고 계산에 COW value copy를 제공한다. 작은 K,D/prior/GNSS 입력 해석은 owner admission에서 수행하고 기존 OpenCV detector는 기존 worker thread에서 실행한다. 원래 승인·stale frame/hash 확인은 그대로다.

기존 관련 optical/controller/worker/geometry/UI 시험 **112 passed in 6.88s**. 원본 image와 payload·설정 변경, detector 자신의 image mutation, panel 접근을 즉시 실패시키는 대체 객체로 변경 후 실행, readonly baseline evidence를 추가 검증해 **114 passed in 7.17s**. Static 검사에서 계산 소유자의 panel/widget/controller 참조 및 facade import가 없음을 확인했다. 이제 사용되지 않는 전체-controller clone과 중복 selector helper를 제거했다.

후보·prior·회전 context·active selection/poses·selection history의 실제 저장 위치는 `OpticalCandidateRegistry`로 이동했다. 기존 이름은 같은 객체를 가리키는 유한한 compatibility property다. 등록·최신 method별 미적용 preview·선택/복구는 registry가 수행하며 file/event I/O는 외곽에 남는다. 기존 controller/measurement 비교 회귀 **142 passed in 5.88s**.

추가 registry fixture 두 번은 각각 canonical `yaw_deg` 이름과 전체 6-DOF observability map을 누락해 **3 failed, 121 passed**(5.95s/5.88s)였다. 생산 검증을 완화하지 않고 fixture를 기존 계약에 맞췄다. NED center, prior yaw의 wrap, 중복 후보 거절, preview의 비적용/독립 출력, history list identity 검증을 포함해 **124 passed in 5.80s**. 전체 회귀를 수행한다.

M1 자동 선택/후보 registry 전체 격리 회귀는 **2331 passed, 1 skipped in 248.38s**, warning 0이었다. 다음 공유 geometry state 변경은 별도 범위로 검증한다.

## M1 M2 공유 active geometry 소유권

`design-boundaries`를 적용해 K,D와 선택/활성 R,T·hash/path·zoom·optical selection identity 17개 필드의 실제 저장을 `ActiveGeometryStore` 하나로 정했다. Calibration/Camera state는 각자의 기존 이름을 같은 store에 연결한다. Legacy getter는 동일 객체의 명시적 live view, 새 snapshot은 detached copy다. Store는 Qt/I/O/좌표 변환을 하지 않는다. 필수 state wiring은 생성 시 확인한다.

Optical Apply와 checkerboard Apply는 준비한 geometry 값들을 한 update로 게시하고, optical 실패 복구와 auto-load 복구도 geometry 부분을 한 publication으로 복원한다. 기존 품질/좌표/파일 발행 gate와 operator Revert 의미는 유지한다. Alignment 파일 provenance·measurement cache·후보 draft는 별도 기존 소유자에 남긴다. 외곽 adapter만 descriptor 없는 legacy fake에 필드를 반영한다.

기존 적용/측정/카메라/구조 회귀 **291 passed in 23.95s**. 단일 store identity, 한 번의 bundle revision, snapshot alias 단절, 알 수 없는 필드의 전체 거절, 누락 attribute 의미를 포함해 **200 passed in 14.11s**. Geometry rollback publication과 auto-load restore까지 포함한 회귀 **153 passed in 14.54s**. 전체 회귀를 수행한다. 이 단계는 장래의 비동기 apply policy를 새로 도입하지 않으며 기존 synchronous apply/restore 계약을 유지한다.

공유 geometry 전체 격리 회귀: **PASS — 2336 passed, 1 skipped in 248.16s**, exit 0, warning 0. 기존 NVENC skip은 유지한다. 이후 M2의 capture feedback을 분리한다.

## M2 촬영 feedback 소유자

촬영 root별 counters/last state/inventory stamp+cache와 이를 변경하는 10개 method를 `calibration_workflow/capture_feedback.py`로 옮긴다. 이 owner는 panel/widget/카메라 명령/solver 설정을 받지 않는다. Controller의 기존 필드와 메서드 이름은 같은 owner에 위임한다. State transition·directory stamp·기존 late-result 표기와 UI/저장 결과는 유지한다. 실제 촬영 작업의 request identity 경계는 별도 후속 단계이며 단순 counters 이동을 전체 capture 수명 분리 완료로 보지 않는다. 상태: IN_PROGRESS.

M2 feedback 분리 targeted 검사: **93 passed in 6.39s**. capture feedback/input/calibration/photo-time/quality UI 기존 검사를 통과했다. 이후 변경과 함께 전체 회귀를 수행한다.

## M1 수치 계산과 후보 등록 경계

Horizon 계산은 `optical_pose/horizon.py`가 명시적 profile/prior/annotation/artifact 입력으로 후보를 반환하고, controller는 입력 수집·등록·UI snapshot을 소유한다. `observations.py`가 frozen frame/hash identity 검증, u/v pixel 해석·undistort를 소유한다. Optical RDF와 world NED·degree/radian 계약 및 기존 observability/품질 문턱은 유지한다.

첫 Horizon extraction 검증은 누락 import(PoseDofMask/Mapping/_payload_sha256)가 test process에 이미 반영된 상태로 시작되어 **7 failed, 94 passed**였다. Production gate나 assertions를 변경하지 않고 import를 수정하고 재실행해 **101 passed in 6.49s**. 이 결과는 이후 terrain 변경의 전체 검증이 아니다.

Terrain PnP는 `terrain_solver.py`에 입력이 명시된 계산으로 이동했다. GeoTIFF import/vertical datum gate·correspondence 파일 읽기는 기존 경계에 남는다. Native u/v와 object_points_ned_m의 일대일 대응, optical RDF→NED pose, per-side observability와 실패 의미를 유지한다. 변경 후 `test_optical_pose_controller_integration.py test_optical_pose_geometry.py test_terrain.py` 격리 검사: **70 passed in 4.46s**, exit 0. 뒤따르는 Mountain/Drone solver 이동은 별도 targeted 검사 후 전체 회귀를 수행한다.


Mountain/Cloud 계산은 `optical_pose/mountain.py`가 촬영 Yaw90 reference, native UV, GNSS NED 기준선과 명시적 lever-arm 보정을 받도록 분리했다. Baseline 방향의 회전 조건·metric translation, Cloud의 simultaneity/rotation-only 정책, 품질 advisory를 유지했다. 기존 검사 **85 passed in 6.39s**. Drone RTK yaw/multiview는 `drone.py`가 후보와 rotation context를 반환하고 파일/source 선택·등록은 바깥에 남는다. 해당 변경 후 UI/저장/기하 포함 **107 passed in 7.44s**.

별도 읽기 전용 AST 검토는 Horizon/Terrain의 수치 본문과 pixel helper가 명시적 의존 치환 외 동일함을 확인했다. 기존 부채인 inline terrain correspondence 자체 hash 누락은 이 분리에서 변경하지 않는다. 새 API의 미사용 terrain_snapshot 인자를 제거하고 profile/prior/artifact와 사전조건을 명시했다.

`test_optical_solver_boundaries.py`에 직접 Horizon 실행, 독립 pinhole식으로 만든 비대칭 NED→native UV terrain fixture의 실제 PnP 결과, stale frame 거절, 새 interpreter에서 Qt/controller import 금지와 금지 fixture 거절을 추가했다. 관련 광학·촬영 feedback 전체 targeted 검사 **141 passed in 7.51s**, warning 0. 이제 전체 격리 suite를 수행한다.


광학 solver/capture feedback 전체 회귀: **PASS — 2339 passed, 1 skipped in 253.66s**, warning 0. 소스는 `solver-source.json`, 출력은 `solver-full-regression.log`다. NVENC skip은 기존 장비 제한이다. Portable path/repository policy/diff whitespace 검사 PASS.

## M7 네트워크 역할 거래의 실제 계약

독립 읽기 전용 검토에서 계획의 자동 reconnect/evidence-before-commit 가정과 현재 코드·`test_active_pixhawk_move_commits_host_state_but_requires_explicit_connect` 계약이 다름을 확인했다. **현재의 explicit Connect 정책을 유지**하고 자동 재연결을 추가하지 않는다. 실제 OS 변경 중 실패의 정확한 역순 rollback은 기존 `apply_network_role_transaction` 소유이며 조정자가 중복 수행하지 않는다.

새 owner 범위는 요청·역할별 주소/endpoint 캡처, 진행 상태, identity를 가진 Qt worker/receiver, stop polling timer, 결과 수용/저장/실패 종료다. `network_role_*` 6개는 원래 Camera feature가 보관하므로 동일 owner에 명시적 alias로 연결하며 Diagnostics의 side별 표시 상태는 복사하지 않는다. 기존 inactive reconnect/rollback 호환 경로를 새로운 자동 정책으로 활성화하지 않는다.

정적 분석상 worker 성공 뒤 설정 저장 오류가 operation을 APPLYING에 남겨 다음 요청을 막는 결함이 있다. 재현 후 실제 적용된 host ownership은 보존하고 저장 실패를 표시하며 작업 수명을 종료하는 범위로 수정한다. core는 이전 OS snapshot receipt를 반환하지 않으므로 설정 저장 실패만으로 호스트가 rollback됐다고 표시하거나 임의 보상 명령을 추가하지 않는다. 네트워크 role/endpoint는 장치 identity이며 좌표 변환 변경은 없다. 상태: IN_PROGRESS.


저장 실패 재현은 **1 failed in 0.19s**: `_apply_network_role_result`의 `_save_current_settings()`에서 `OSError(settings-disk-full)`이 탈출했다. 새 `NetworkRoleRuntime`과 Qt relay/timer/worker identity를 도입하고 CameraFeature의 6개 필드를 단일 owner alias로 연결했다. 성공 뒤 저장 실패는 실제 host ownership을 남기고 `APPLIED · SAVE FAILED`로 끝낸다.

첫 M7 통합 검증은 **3 failed, 108 passed in 13.69s**. 두 실패는 새 캡처/표시 의존을 생략한 경량 fixture(역할 requirements, side status)였다. 나머지 실제 panel 실패는 기존 subscriber 설정 동기화가 handle을 먼저 비워도 새 owner의 `_jobs`에는 Qt retire가 남아 있는 경우였다. 완료 판단을 handle뿐 아니라 실제 job ledger까지 확인하도록 수정하여 다음 요청은 QThread.finished까지 기다리게 했다. 기존 fixture assertions는 유지하고 필요한 명시적 입력만 추가했다.


재실행은 **1 failed, 110 passed in 13.37s**로 동일 실제 panel 회귀가 남았다. `rca`로 기존 'handle 먼저 해제' 추정을 검증했다. 단일 테스트의 admission 진단은 `operation=True, thread=False, jobs=0`였고 public panel operation은 None이었다. 실제 원인은 `panel_contracts.VIEW_FIELDS`에 network role 6개 값이 모두 들어 있어 기존 값이 Camera **views**에 남고 새 alias는 Camera **state**에 설치된 이중 저장 경계였다. 그 6개만 실제 state로 재분류했다. 공유 geometry fields도 대조해 같은 오분류가 없음을 확인했다. 수정 후 기존 네트워크/실제 panel/소유권/종료 회귀 **111 passed in 13.49s**. 앞의 handle 설명은 원인 가설이었으며 이 결과가 대체한다. Job ledger까지 완료 판단하는 보호는 독립 수명 불변식으로 유지한다.


추가 owner·Qt affinity·실제 settings 검사는 **142 passed in 23.74s**, UI mutation/잘못된 interface 결과까지 포함한 묶음은 **165 passed in 24.43s**였다. 독립 검토에서 token 없는 legacy 완료/실패/정리, presentation exception, owners_stopped query exception, legacy shutdown fallback의 미보호 경로를 발견해 보강했다. 실제 admission에는 transaction_id를 부여하고 relay만 내부 완료 처리를 호출한다. 상태 표시 실패는 presentation_error에 남기고 거래 종료는 finally로 보장한다. 이미 생성된 legacy owner도 종료/자원 목록에 포함한다.

비대상 MAVLink URL 캡처 검사는 **1 failed, 146 passed in 23.85s**였다. 문자열 치환이 settings_text 대신 RTSP requirements 조회 구간에 적용된 편집 실수를 확인하고 정확한 입력 캡처 구간으로 고쳤다. RTSP producer 해석은 원상 유지, 양쪽 MAVLink URL까지 접수 시 고정한다. 재검증 **147 passed in 23.58s**. 원래 README도 코드와 달리 자동 reconnect를 설명해 explicit Connect, 적용/저장 실패 구분, 저장 재시도 안내로 정정했다. 현재 추가 검사에서는 실제 Qt worker의 정상/저장 실패/표시 실패 조합을 확인한다.


M7 runtime 최종 targeted 검사: **150 passed in 23.79s**, warning 0. 실제 worker에서 저장 성공/실패와 표시 성공/실패의 조합에서도 원래 거래만 완료하고 resource를 retire한다. 실제 panel에서 접수 후 주소/peer/양쪽 endpoint를 바꾼 경우에도 승인 당시 요청과 저장한 settings가 일치한다. 최신 코드에 전체 회귀를 수행한다(`network-role-source.json`).


M7 전체 격리 회귀 **PASS — 2352 passed, 1 skipped in 241.83s**, exit 0, warnings 0. source는 `network-role-source.json`, log는 `network-role-full-regression.log`. 독립 검토의 5개 누락 경로도 최신 소스에서 해결 확인. 다음 M3는 수치/queue/lifecycle 정책 변경 없이 기존 실제 자원 class와 backend·child/supervisor 조립을 별도 소유자로 이동한다. 전체 regression을 매 작은 추출마다 반복하지 않고, 각 직접 관련 검사와 마지막 결합 검증으로 묶는다.

M3 소스 기준 `yolo_stream.py` 166개 top-level 정의/상수. 정의 밖 실행문 없음. 예상 의존 DAG와 draft는 모두 cycle 없음, static unresolved-name 검사 PASS이며 아직 적용 전이다. 원래 클래스의 lock/queue/thread/resource와 동작을 함께 옮긴다. RTP/RTCP tick/UTC와 host monotonic ns/s, native u/v 및 RGB/BGR/NV12 pixel buffer, source frame identity와 queue drop·decoder fallback·Stop deadline 계약을 유지한다. Spatial NED/FRD/RDF 변환은 이 계층에 추가하지 않는다.


M3 **분리 전** 20초 synthetic qualification은 기본 production 기준을 그대로 사용했으며 `rss_slope_within_limit`만 FAIL이었다. Peak RSS **60,325,888 bytes**, 20초 구간 환산 slope **1,143,825,344.78 bytes/hour**. IPC/evidence/cleanup·절대 RSS/Stop 검사는 통과했다. 기존 코드에서도 발생했으므로 새 구조 회귀로 해석하지 않고, 짧은 구간의 증가만으로 장시간 누수라고 단정하지 않는다. 기준을 완화하지 않고 동일 실행을 분리 후 비교한다. 원시 결과 `media-baseline.json`. 기존 full suite의 2초 qualification 테스트는 메모리 한계를 크게 열어둔 smoke이며 이 기본 memory 기준의 통과 증거가 아니다.

89개 class/function body는 draft 추출 전후 AST가 모두 정확히 같다. 나머지 77개 policy 상수는 값 동일하며 SASL_ROOT만 동등한 기존 runtime_paths.REPO_ROOT로 참조하여 파일 깊이 변경을 보정한다. 6개 타입의 기존 pickle bytes를 추출 전 저장해 새 호환 import와 직렬화 일치를 검증한다.


M3에 17개 media 소유자+얇은 explicit yolo_stream 호환 facade를 적용했다. 11개 생산 consumer는 실제 소유자를 import하고, 4개 test 파일의 private injection/원문 위치 조회는 실제 구현 경계로 이동했다(assertions 유지). 첫 pipeline/timing/recording/IPC/parent-death/MP4 회귀 **160 passed, 1 skipped in 9.00s**, NVENC 장비 제한 skip 유지. 추가 6개 legacy pickle bytes/동일 class identity, 새 interpreter에서 Qt/model/legacy facade premature import 금지, 실제 child_runtime spawn의 lossless PNG BGR→RGB decode 및 열기 실패→종료를 검사했다. Camera/Settings/WIRIS/실제 panel 연결 포함 **147 passed in 25.72s**. Media 이후 별도 command wiring 변경과 마지막 전체 검증을 묶을 예정이다.


M3 분리 후 동일 20초/GUI stall 2초/증거 2초 간격의 기본 qualification도 **rss_slope_within_limit만 FAIL**. Peak RSS **61,956,096 bytes**, slope **1,808,738,175.44 bytes/hour**이며 모든 자원 정리·증거/IPC·절대 RSS·Stop 검사는 통과했다(`media-after.json`). 전보다 peak 1,630,208 bytes 높았다. 이 짧은 구간 비교는 메모리 비퇴행이나 장시간 bounded-memory PASS로 취급하지 않는다. 기준을 완화하지 않았으며 기존 startup 포함 slope 부채는 남긴다. 이번 source relocation의 정책·할당 위치·락/큐 동작은 AST와 실행 회귀로 별도 검증한다.


## M14 기능별 command declarations

기존 MainWindow `_build_commands`의 101개 선언과 파라미터 closure를 기능별 `command_bindings/`로 이동했다. 선언 함수는 registry+명시적 callback만 받고 window/backend/state를 받지 않는다. 85개 기존 action method 연결은 바깥 composition에서 제공하며 등록 순서도 유지한다. Runtime 실행 handler와 optical worker lifecycle까지 이동한 것으로 해석하지 않는다. 원래 함수의 선언만 가짜 registry/callback에 실행한 독립 catalog fixture에 ID·text·tooltip·category·shortcut·checkable·deferred completion 및 호출 파라미터를 저장했다.

추출 helper가 처음에는 `/tmp` 실행 import 경로, 다음에는 MAVLINK_QUALIFICATION_S global을 누락해 적용 전에 중단됐다. 원본을 유지한 채 경로/전역 인자를 보완했다. 그 사이 original command baseline와 새 media RTCP 회귀 **129 passed in 25.28s**. 실제 추출 후 fixture의 모든 101개 항목/순서/handler payload, 누락 action의 composition-time 실패, 금지 window/backend import negative fixture와 UI/navigation/lifecycle 검사 **127 passed in 35.12s**. 이제 M3+M14 전체 검증을 한 묶음으로 수행한다. source는 `media-command-source.json`(후속 문서 한 줄 추가는 algorithm 변경 없음)이다.


M3 독립 읽기 감사는 89개 정의/77개 상수, 자원 생성 위치, 17개 module DAG, source/global import 목적지와 기존 mock assertion을 대조해 이번 추출의 수치/동작 회귀를 찾지 못했다. 남은 한계: legacy __module__을 유지한 IPC wrapper를 spawn으로 unpickle할 때 facade→Qt import가 따라올 수 있다. 기존에도 있었고 이번 fresh leaf import guard는 이를 없앴다는 증거가 아니다. 실제 child decoder 테스트는 현재 plain queue 경로이므로 전체 suite 뒤 실제 bounded IPC wrapper 경로로 확장해 별도 검사한다. private facade globals 재대입 injection까지 외부 호환을 보장하지 않는다. 기본 RSS-slope before/after FAIL도 그대로 유지한다.


M3+M14 전체 격리 회귀: **PASS — 2364 passed, 1 skipped in 247.90s**, exit0, warning0. `media-command-full-regression.log`와 해당 실행 source hash JSON 기록 완료. 이후 변경은 actual bounded IPC wrapper로 spawn test를 강화한 것뿐이며 생산 코드는 그대로다. 이 검사 결과는 별도로 기록한다.


Actual bounded IPC wrapper로 강화한 child spawn/decode/failure/pickle/import 검사 **9 passed in0.75s**. 생산코드는 이전 전체2364 pass 상태와 동일하다.

## M15 first-frame/MP4 readiness state

첫 확인 상태의 4개 authoritative 필드(camera/video generation+readiness)를 Qt/UI 없는 `StreamReadinessTracker` 한 곳으로 옮긴다. Camera와 Presentation feature의 현재 state 필드를 실제 동일 소유자에 alias한다(모두 VIEW_FIELDS가 아님을 먼저 확인). Frame metadata의 current/expected stream_generation gate, partial-start, INPUT UNVERIFIED 표기, MP4 positive-byte-growth와 timeout/실패에도 Measurement 계속하는 정책을 유지한다. 루트 controller는 실제 pane 상태/효과·UI·현재 QTimer scheduling adapter를 맡는다. Retry/recovery/Stop/finalization 전체 수명 이동으로 확대해 주장하지 않는다. 입력 u/v raster·frame identity와 host monotonic seconds만 다루며 NED/FRD/RDF 변환 없음. 수용 기준: 실제 owner identity, stale/new generation, partial/timeout, 기존 저장준비와 Measurement 연동. 상태 IN_PROGRESS.


M15 readiness 추출 후 기존 frame/decoder/MP4/Measurement/Camera/panel 회귀 **187 passed,1 skipped in16.72s**. 실제 공유 owner identity, 초기 expected generation 입력 사본, stale/new generation와 partial/timeout, positive byte growth와 이전 완료가 새 video를 지우지 못함, 카메라 취소와 video 준비 상태의 분리를 검증해 **191 passed,1 skipped in17.16s**. Portable path/repository policy/diff whitespace PASS. Source `readiness-source.json`. 전체 회귀는 다음 독립 변경과 묶는다. 이 owner는 현재 준비 상태/판정을 맡고 기존 Qt scheduling/pane effects/retry/watchdog adapter는 아직 controller에 남는다.


## M14 optical auto-selection Qt lifetime

`optical_pose/auto_runtime.py`가 frozen job의 worker/thread/command identity와 GUI-affine relay를 소유한다. MainWindow는 prepare/commit/표시와 close 요청을 맡고 기존 3개 handle 이름은 같은 소유자의 alias다. 실제 Qt finished까지 새 job admission을 막고, 과거/중복 outcome·finished는 현재 job을 바꾸지 못한다. 기존 cancel의 queued-only 의미와 창 닫기 5초 후 force attempt/500ms wait 정책은 유지한다. 다만 wait 실패/여전히 running이면 handle을 버리지 않고 창을 계속 대기시키도록 확인을 추가했다.

PanelApplication에는 실제 runtime들을 등록해 이름 충돌 없이 inventory에 포함하며 QObject 파괴 때 제거한다. backend shutdown도 이미 등록된 모든 owner에 Stop을 요청하고 완료 여부를 모은다(단락 평가로 뒤쪽 owner Stop을 생략하지 않음). 새 owner는 window/panel/live profile을 받지 않고 worker factory와 completed/failed/retired callbacks만 받는다. 생산 worker 생성 실패도 busy state를 남기지 않는다.

기존 optical/lifecycle/명령/실제 panel/준비 상태 회귀 **102 passed in18.90s**. 직접 Qt worker→GUI delivery, late result, 미완료 force-stop의 handle 보존, 필수 job/worker capability, 복수 resource registration/제거, disabled video 입력 미소비를 추가해 **107 passed in19.38s**. 다음은 실제 MainWindow closeEvent의 force-stop 실패 경로를 확인한다. 좌표는 이미 동결된 native pixels/K,D/NED/RDF 입력을 전달할 뿐 변환하지 않는다.


실제 MainWindow closeEvent의 미완료 force-stop 대기까지 포함해 **44 passed in7.64s**. 독립 감사는 readiness 계약을 확인했고 새 auto runtime의 thread.start 예외 뒤 reservation 누락 정리를 지적했다. 예외/무실행 반환을 확인하고 실제 비실행 상태에서만 reservation을 해제하도록 수정했다. 부분 시작이면 실제 자원은 남겨 종료를 기다린다. worker run/cancel은 admission의 필수 capability다. Legacy 표시 handle이 먼저 비워져도 request_stop은 실제 task의 resource를 다시 사용한다.

시작 실패 주입 첫 묶음은 **2 failed,68 passed in8.60s**였다. 두 실패 모두 테스트가 Qt C-extension method를 `getattr` 결과로 수동 복원해 descriptor binding을 잃은 문제였다. 원래 descriptor를 복원하는 monkeypatch.context 범위로 주입을 제한하고 같은 assertions로 재실행한다. 생산 actor 정리 조건을 완화하지 않는다.


M14 start-failure fixture 수정 후 **70 passed in8.37s**. cancel cleanup 오류가 겹쳐도 비실행 reservation을 해제하도록 보호하고 같은 조건의 fault injection을 추가했다. 마지막 owner/readiness targeted **13 passed in1.96s**, warning0. Portable path/repo policy/diff whitespace PASS. 이제 M15 readiness+M14 auto runtime 전체 회귀를 한 묶음으로 수행한다(`readiness-auto-source.json`).


M15 readiness + M14 optical runtime 전체 격리 회귀: **PASS — 2377 passed, 1 skipped in260.05s**, exit0, warnings0. `readiness-auto-source.json`/`readiness-auto-full-regression.log` 기록 완료. NVENC 장비 제한 skip 유지.

## G1 생성 MAVLink 재현성 조사

설치·pin된 pymavlink2.4.49의 common dialect는205개 message, 현재 vendored artifact는235개다. 단순 common XML+SASL_CAN_MAG patch로는30개 missing(29개 upstream+private180),18개 기존 message field 차이가 있어 재생성하면 호환성이 깨진다. 따라서 설치 wheel의 XML을 원본으로 잘못 채택하지 않는다. 초기 표준 PX4 경로에 checkout이 없었으나 repository environment_report가 기록한 별도 Capstone/source checkout을 발견해 입력/revision을 확인한다. 현재 runtime 생성 파일은 수정하지 않았다.


G1 입력 발견: 환경 보고서가 지목한 별도 checkout의 PX4 HEAD가 기록된 `7b1e54889d5bf8120f843fd88760ab4b6e2825f5`와 일치한다. MAVLink submodule `51a47ee9aed1216532f2eefbfe8d260a2e3cabcc`, bundled pymavlink `5977f44504f01715a44bed44a4601e6c12cef92d`. 원래 common.xml blob `4b37e5eb...`도 repo의 SASL_CAN_MAG patch baseline과 일치한다. 외부 checkout은 읽기만 했으며 새 runtime 의존으로 삼지 않는다.

설치 generator의 기본 XSD는 새 `<superseded>` metadata를 거부했다(검증 FAIL). 검증을 끄지 않고 firmware checkout의 schema로 mavgen.schemaFile을 지정하여 strict XML validation을 유지했다. pymavlink2.4.49 + 원래3개 XML + 원래XSD, language=Python/wire=2.0로 임시 디렉터리에 재생성한 결과 **1,919,084 bytes / SHA256 7ed861c4d01a48e1c972cd1936b4410e29f209f1e428178f704564868f4ea2ba**, 기존 generated file과 **byte-for-byte 동일**하다. Generator의 두 기존 zero-valued bitmask diagnostic은 유지된다. 이제 그 입력·schema·라이선스·hash/revision을 repo 내부에 보관하고, 외부 checkout 없는 --check 명령을 추가한다. 생성 runtime 파일은 바꾸지 않는다.


G1 repo 내부3XML/XSD/COPYING(총761,351 bytes)과 generation_manifest.json, `tools/regenerate_mavlink_dialect.py`를 추가했다. 기본 --check는 입력·generator source hash, pymavlink2.4.49, XSD validation 유지와 기존1,919,084 bytes 전체를 비교한다. 임의 output basename도 canonical DIALECT 이름을 유지하며, 생성 후 mavgen.schemaFile을 원상복원한다. 생성 runtime Python과 firmware 파일은 바꾸지 않았다.

새 잘못된 XML 시험은 strict XSD에서 실제 거부됐지만 mavgen의 SystemExit가 library wrapper 밖으로 탈출해 첫 묶음 **1 failed,139 passed in8.13s**였다. 이를 validation 상태를 포함하는 RuntimeError로 정규화하고, CLI는 명확한 실패 exit를 반환하도록 수정했다. Assertions/validation은 유지한다. 원시CAN packet fixture는 헤더/CRC 포함 고정bytes 및 별도 struct unpack으로 sensor-frame Gauss·boot us·CAN1/2/ID 순서를 검사한다.


G1 수정 후 **140 passed in8.12s**, warning0. CLI --check도 strict XML validation과 235 messages의 byte-for-byte hash 일치를 확인했다. source assets 및 tool/docs/tests는 모두 repository 내부이며 새 필수 원본의 machine-local-only 의존성은 없다. `git status`를 SASL root에서 확인: 새 패키지·테스트·XML/XSD/manifest는 현재 untracked이므로 이후 공유할 commit에 함께 포함해야 한다. Commit/push는 요청되지 않아 수행하지 않았다. Portable path/repository policy/diff whitespace PASS. Firmware artifact와 generated runtime 파일 내용은 변경 없음.


## M10 credential selection / GGA ownership

첫 경계는 account별 credential priority와 GGA sentence 선택/skip 상태다. Credential resolver는 캡처한 password/explicit marker/account key와 network value를 받고 panel을 읽지 않는다. 계정별 우선순위·keyring/env fallback·200 이후 저장 정책은 유지하며 secret은 repr에 표시하지 않는다. NTRIP 연결 worker의 identity/late callback과 pending credential lifetime은 뒤따르는 별도 경계이다.

GGA owner는 현재선택된 GPS1/GPS2/Base sample과 active operation config를 명시적으로 받는다. Source 선택은 기존 controller에 남는다. 실제 skip reason/time/30초 throttle의3개 GNSS state 필드를 같은 owner에 alias한다(VIEW_FIELDS 아님 확인). WGS84 lat/lon degrees, MSL altitude metres, ellipsoid-minus-MSL geoid separation, host monotonic sample age/skew, NMEA wall UTC와 GGA quality enum을 구별한다. BASE는 MSL 없으면 실패하며 SYSTEM_CENTER는 기존 geodetic mean/quality1/geoid blank를 보존한다. NED/ENU/body/camera로 변환하지 않는다. 기존 fresh/source/fallback/success-clear/log-throttle 규칙을 유지한다. 상태 IN_PROGRESS.


M10 credential/GGA 기존 NTRIP·GNSS protocol·RTCM pipeline/evidence·실제 panel 회귀 **146 passed in10.98s**, warning0. 새 owner identity·비밀 repr·독립 WGS84/MSL/GGA UTC 입력 검사를 이어 추가한다. 읽기 감사는 기존 NTRIP connected/failed/diagnostic queued callback에 worker identity가 없어 Stop 또는 새 Start 뒤 상태·pending credential을 바꿀 수 있음을 확인했다. 기존11개 수명 메서드 AST는 leaf 추출 전과 같아 신규 회귀가 아니다. 다음 owner는 동일 worker의 정상 reconnect를 계속 허용하고 stop/terminal/세대만 차단한다. 원시 RTCM→pipeline DirectConnection, evidence/drain 수명은 그대로 유지한다.

새 M10 input 검사 첫 실행 **1 failed,80 passed in1.99s**. 실패는 실제 panel의 controller-owner descriptor를 메서드처럼 호출한 fixture 오류이며 실제 API인 property 접근으로 수정했다. WGS84 남/서 hemisphere, MSL42m/geoid27.5m, NMEA quality4/UTC, center unknown-geoid, Base MSL 누락, account snapshot/secret repr 검사는 통과했다.

M10 input 재검사 **81 passed in2.01s**, warning0. 이어 `gnss_runtime/ntrip.py`에 연결별 실제 thread/worker/pending credential binding과 GUI-affine relay를 추가했다. GNSS의3개 legacy state 이름은 실제 session의 alias이며 inventory에 명시적으로 포함한다. Stop/terminal/old binding 이벤트는 차단하고 동일 binding의 connected→reconnecting→connected는 허용한다. 실제 finished 전 재시작은 거절하며 시작/배선 예외는 UI reservation rollback으로 연결한다. Callback 예외가 자원 정리를 건너뛰지 않게 기록한다. 기존 coalesced pipeline stats의 source-generation 부재는 별도 남은 계약이며 이 control-event 보호로 해결됐다고 주장하지 않는다. 원본 RTCM 연결·좌표·timestamps·retry budget 변경 없음. 상태 IN_PROGRESS.

NTRIP owner 적용 뒤 기존 GNSS/NTRIP/RTCM/panel/종료 회귀 **276 passed in33.86s**, warning0. 새 실제 Qt thread 기반 시험은 GUI queue drain 전 Stop, 같은 worker reconnect, old binding→new admission, terminal duplicate, 진행 중 retire 거절, factory/wire/start 오류와 재시도, 실제 panel Start/Stop controls/inventory/raw GUI receiver=0을 검사한다. 이 추가 검사는 실행 중이다.

추가 NTRIP Qt fixture 첫 실행 **9 failed,152 passed in21.70s**. 9개 모두 ControlledWorker.run의 시작 대기에서 실패했다. 기존 생산 worker의 run은 pyqtSlot인데 시험 subclass가 slot decorator 없이 override해 signal proxy가 GUI에 남았다. production과 같은 Qt slot로 fixture를 정의하고 worker thread/GUI callback thread 구분 assertion을 유지한다.

Qt fixture 수정 뒤 **10 passed in2.30s**, warning0. 후속 연결 통계에는 내부 `NtripSessionStats(token,stats)` envelope를 추가한다. worker child인 Qt forwarder는 producer thread에서 coalesced stats만 태그하며 pipeline QObject 파괴 때 signal 연결도 자동 해제된다. pipeline의 bounded raw envelope·evidence/schema·drain 경로는 변경하지 않는다. UI는 현재 active binding token의 stats만 unwrap한다. 옛 plain stats는 owner 미설치 legacy 경로에서만 수용한다. Stop/새 Start 뒤 오래된 stats가 새 연결을 STREAMING으로 표시하는 경합이 대상이다.

독립 감사가 production signal은 보호되지만 panel에 공개된 무토큰 legacy callback이 같은 gate를 우회할 수 있음을 지적했다. 실소유자 설치 후 legacy connected/reconnecting/failed/diagnostic/stats/chunk 콜백은 거절하고 owner는 별도 내부 effect 함수로 전달한다. 실소유자 없는 legacy fixture/지원 경로는 유지한다. 실제 panel Start→Stop→새 Start에 이전 coalesced snapshot과 무토큰 성공/실패를 주입해 상태·pending credential 보존을 검사한다.

M10 입력/수명/통계/호환콜백·RTCM/종료 targeted **162 passed in12.98s**, warning0. Actual Qt Start→Stop→Start에서 이전 HTTP 200이 새 password를 저장하지 않고 이전 stats가 CONNECTING을 덮지 않음을 확인했다. 가짜 NTRIP source를 실제 panel/pipeline에 연결했으며 장치/외부caster 검증을 주장하지 않는다. 경로 검사에서 이전 checkpoint의 외부 원본 checkout 절대경로1건을 발견해 소스 보고서 참조+revision으로 수정했다. machine-local-only 실행 의존은 없다. README와 architecture에 운용/경계 설명을 반영한다.


M10+G1 전체 격리 회귀 **PASS — 2397 passed,1 skipped in254.09s (4m14s)**, exit0, warnings0. Source `ntrip-source.json`, log `ntrip-full-regression.log`. 실행 중554개 Python source hash 변경 없음. NVENC 미지원 skip은 기존과 동일하다. 마지막 출력 갱신 대기에서 process를 확인했으나 실제 검사 시간은 이전 전체 회귀 범위와 같다. 독립 읽기 감사는 무토큰 compatibility 차단과 stats forwarder의 Qt parent/receiver 수명을 확인하고 추가 결함을 찾지 못했다. 실장비 검증은 수행하지 않았다.

## M7 Shell/PARAM maintenance jobs

다음 경계는 side별 shell/parameter 작업의 실제4개 thread/worker dictionary와 Qt callback/retirement다. 현재 worker의 명령 생성, send/exclusive locks, radio retry/timeout, armed·SYSID preflight, PARAM ACK/readback 및 sensor/board rotation 의미는 바꾸지 않는다. 입력은 접수 시의 subscriber·side·operation·명령/우선순위·실제선택 MAG ID·armed 값이다. 같은 side/kind의 중복은 거절하고 다른 side 또는 기존 독립 kind concurrency는 유지한다. GUI-affine outcome은 접수한 subscriber/connection generation에서만 반영하며 Stop/중복/old callback은 상태를 덮지 못한다. 종료는 실제 thread 종료까지 handles와 inventory를 보존한다. 네 state 필드 모두 VIEW_FIELDS 밖임을 확인했다. CAL_MAG sensor-frame 값·SENS_BOARD_ROT enum·body FRD/NED 변환은 기존 worker/consumer에서 그대로 처리하며 새 owner는 해석하지 않는다. 수용 검사: 실제 Qt affinity·stale link·같은/다른 lane admission·Stop·start 실패·real panel inventory 및 기존 MAG/readback/종료 흐름. 상태 IN_PROGRESS.

M7 Shell/PARAM owner 적용 뒤 기존 MAG/readback·실제 panel·종료·NIC 회귀 **220 passed in20.25s**, warning0. 4개 결과/실패 처리 함수의 AST body는 기존과 동일하다. Worker의 shared subscriber/command locks·radio retry·PARAM decode는 수정하지 않았다. 추가 actual Qt 검사는 side/kind concurrency, duplicate/old identity, Stop 전 queued outcome, legacy dictionary 삭제 뒤 inventory 보존, thread start 실패 재시도, 실제 panel에서 subscriber/connection/generation 교체와 board-rotation readback을 확인한다.

M7 actual Qt+MAG+실제 panel+종료 **205 passed in20.11s**, warning0. 자체 검토에서 legacy thread 교체 시 stop 요청이 원래 job과 교체 handle 중 하나만 볼 수 있음을 확인해 thread identity별로 모두 중단하도록 수정했다. 해당 fault injection+종료 **45 passed in8.53s**. 독립 감사는 worker dictionary만 변경했을 때 inventory에서 원래 worker가 빠지는 조건을 추가로 지적했다. thread 또는 worker identity mismatch 양쪽을 확인하고 worker-only/thread-only/both 삭제 검사를 추가했다.

M7 inventory 보완 후 **47 passed in8.23s**, warning0. worker-only/thread-only/both legacy 삭제와 다른 thread 교체를 포함해 실제 job 자원 및 중단 대상 누락이 없음을 확인했다. README/architecture 반영. 전체 회귀는 다음 독립 timer 변경과 묶는다.

## M15 restart/stop timer ownership

기존100ms 재시작 poll·35초 replacement deadline과250ms 종료 watchdog의4개 state 값과 지연 callback 수명을 Camera feature의 Qt owner로 옮긴다. Pane stop·실제 stream start·timeout rollback·기록 presence/finalization·Run 완료는 명시적 callback이며 다른 업무 상태를 owner에 복제하지 않는다. 실제 cancellable single-shot QTimer를 job identity와 함께 관리하고 inventory에 노출한다. 오래된 timeout은 새 Start/Stop에 영향이 없으며 재시도 도중 중복 요청은 기존 deadline을 늘리지 않는다. 두 pane stop을 항상 각각 호출하고, 기존 recording failure가 Measurement를 차단하지 않는 정책을 유지한다. Timer/time contract는 host monotonic seconds·100/250ms schedule, 영상·NED/FRD/RDF 변환·CSV schema 변경 없음. 수용 검사: 재시작 기다림/Stop/중복/timeout, old watchdog 후 새 Start, 실제 Qt 타이머 취소·파괴·inventory와 기존 카메라/기록/Run 통합 흐름. 상태 IN_PROGRESS.

M15 timer owner 이동 뒤 기존 camera/readiness/recording/Measurement/종료 회귀 **148 passed in19.72s**, warning0. 기존4개 시험의 singleShot mock은 새로운 `_schedule` 경계에도 주입하고 assertions는 유지했다. 새 QTimer 소유권 때문에 완료된 explicit Stop에서는 이미 끝낸 watchdog도 취소해 타이머를 남기지 않는다. 후속 실제 Qt 검사로 같은 deadline 유지, old callback 무효화, 양쪽 pane와 finalization 완료 대기, parent 파괴 시 타이머 취소, actual panel state/inventory를 검증한다.

M15 actual Qt 포함 **153 passed in20.31s**, warning0. 독립 감사는 stop_pair/finalize callback 도중 취소/새 예약이 들어오는 재진입에서 old job이 계속 실행되는 결함을 찾았다. 이를 독립 fault injection으로 **6 failed,5 deselected in0.73s** 재현했다. timer callback 진입 때뿐 아니라 각 외부 callback 직후에도 job identity/generation을 확인하도록 수정했다. 취소된 start 실행·새 pending/deadline 삭제·새 watchdog 덮어쓰기가 실패 조건이다. 수정 후 같은 assertions와 관련 통합 검사를 실행한다.

M15 재진입 보완 뒤 **159 passed in20.71s**, warning0. 6개 실패 재현의 동일 assertions와 실제 Qt parent 파괴·취소·GUI delivery 및 카메라/기록/Run/종료 통합 모두 PASS. 현재 M7+M15 source는 `maintenance-timers-source.json`. 이후 전체 회귀는 관련 없는 캘리브레이션 job-owner 검증과 묶어 수행할 예정이다.

次 M2 조사는 4종 checkerboard job의 실제종료와 FTP 후속queue 및 shutdown admission 계약이 함께 연결됨을 확인했다. 새 M2 runtime 수정 전 현재 M7+M15 source를 먼저전체검증해 이어지는 수명변경의 기준선을 명확히 한다. Path/repository/diff checks 결과는 같은 실행기록에추가한다.


M7 Shell/PARAM + M15 lifecycle timer 전체 격리 회귀 **PASS — 2421 passed,1 skipped in255.37s**, exit0, warning0. `maintenance-timers-source.json`/`maintenance-timers-full-regression.log`. 실행 중558개 Python source hash 변경 없음. NVENC 미지원 skip 유지. Path/repository policy/diff checks도 PASS. M2는 이 시점까지 읽기/계약 조사만 했고 runtime 수정 없음.


## M2 checkerboard shared job owner

4종 capture/intrinsics/extrinsics/random-validation은 기존 단일 배타 slot을 유지한다. 실제 thread/worker와 GUI-affine progress/result/finished identity, 종료 후 후속 admission을 함께 소유한다. Source는 기존 calibration_workers.py와 controller의 captured input/selection snapshot이며 worker 계산·파일 쓰기·완료 effects는 유지한다. Auto Stop은 다음 촬영만 막고 진행 중 파일·결과를 반영하는 기존 정책을 보존한다. App close도 기존 결과를 소비한 뒤 retire까지 기다리며 신규 job/FTP continuation만 차단한다. generic shutdown의 isRunning=False만으로 alias를 먼저 비우지 않는다.

Capture는 native RGB image-grid u/v=[right,down] pixels와 per-side frame identity, host receive monotonic/RTCP/FTP proxy timing 구분을 유지한다. Intrinsic/extrinsic 입력은 기존 immutable CalibrationInputSnapshot, 고정 K,D와 OpenCV optical RDF R,t(source camera→other camera)·metres·profile/selection hashes를 그대로 전달한다. 임계값/보정·검증 방법/quality 정책을 변경하지 않는다. Snapshot root는 기존 captured source/provenance 계약이며 새 경로 직렬화 형식을 만들지 않는다.

기존 FTP queue는 pop(0) 뒤 capture admission=False이면200ms retry만 예약해 해당 pair를 잃는다. 성공 전 head를 유지하고 close 검사를 소비 전에 두며, 후속은 결과 반영→실제 finished→slot 해제→UI cleanup 후 실행한다. 현재 saved counter는 실제 저장 성공이 아니라 기존 admission count이므로 그 의미를 새 저장성공 증거로 주장하지 않는다.

수용 검사:4종 상호배타·GUIaffinity·old/duplicate progress/result/finished·시작실패복구; AutoStop/appclose중 이미저장된capture 결과반영; 두FTPpair연속처리와busy거절시head보존·closing시후속차단; 기존capture→intrinsics→random·fixed-K,D extrinsics 및입력hash/metadata·저장재읽기 회귀. 상태 IN_PROGRESS.


M2 기존 capture/input-hash/selection/quality/feedback/FTP/panel/종료 회귀 **203 passed in17.19s**, warning0. 8개 완료/진행/실패 effect의 body는 내부callback target을 정규화하면 AST동일하다. 사용자 최신 지시(2026-10-11)는 "진행 중인 것만 끝내"이므로 이 M2 변경의 검증·마감만 수행하며 다른 모듈은 착수하지 않는다. 두 agent는 이 범위의 독립읽기감사/순수owner검사만 담당한다. Actual panel AutoStop·AppClose 결과보존, 두FTPpair연속처리, start실패feedback/버튼복구, queuehead/구queue identity 검사를 추가했다.

M2 실제 panel/FTP 새 검사 **5 passed in2.56s**. Native RGB 복사와 PNG 재열기, Auto Stop 뒤 저장결과 반영/auto counter 제외, 물리적 thread 종료 뒤에도 queued 결과 소비 전 shutdown 대기, 두pair연속처리, 시작실패feedback/버튼복구, 거절된queuehead 보존을 확인했다. 독립 읽기 감사는 부분적 thread 시작 뒤 예외를 실패terminal로 처리하는 경로와 완료callback 도중 재진입 retirement를 지적했다. Thread-safe launch evidence를 보존해 실제 시작은 admission으로 유지하고(FTP 중복 재시도 방지), callback 중에는 retirement를 보류하도록 좁게 수정했다. 해당 fault injection은 별도 agent가 추가 중이다.

M2 수정과 새18개 owner/workflow 사례를 포함한 관련 회귀 **221 passed in19.79s**, warning0. Partial-start 뒤 두FTPpair가중복없이처리되고, callback중nestedretire가완료effect를앞서지않음을 실제Qt로확인했다. 독립읽기감사는두수정완료와추가blocker없음을확인했다. README와architecture를현재운용계약에맞게갱신했다. Path/repository policy/diff checks PASS. 사용자지정마감범위를확장하지않고 `calibration-completion-source.json` 고정소스로최종전체회귀를실행한다.


### 2026-10-11 사용자 지정 마감 범위 완료

**PASS — 2439 passed,1 skipped in312.58s (5m12s)**, exit0, warning0. `calibration-completion-full-regression.log`와 `calibration-completion-source.json`에 기록했다. 실행 중562개 Python source hash 변경 없음. Skip은 기존 NVENC 미지원1건이며 M2 변경과 무관하다. 최종 출력 전 종료 정리가 평소보다 오래 걸렸으나 프로세스의 정상 exit0와 전체 결과를 확인했다. 새 작업이나 추가 성능 조사로 범위를 확대하지 않는다.

완료:4종 공유slot/GUI전달·중복/old결과차단, AutoStop/appclose중 이미저장된결과반영, 실제finished및callback완료뒤retirement, FIFO head보존/두pair연속처리/closing후속차단, 비실행시작실패복구/부분시작결과보존, native RGB 재열기와 기존입력hash/K,D/R,t 결과계약. 관련221개와최종전체회귀 모두PASS. README/architecture의실제동작설명갱신. 좌표·수치worker·8개결과/진행/실패effect는원계약을유지한다.

이식성: repo root git status를확인했다(기존변경포함233개entry). 이번필수새파일 calibration_workflow/jobs.py,state.py 및 tests/test_calibration_job_owner.py,test_calibration_job_workflow.py는repository안에존재하며untracked/비ignored상태다. 이후공유할commit에함께포함해야한다. 외부checkout/임시script를runtime필수의존으로추가하지않았다. 실제장비/현장검증과장기계획의나머지모듈은이번마감범위밖이다.

사용자 "그럼 진행 중인 것만 끝내"에 따른 현재 작업은 완료다. 다음 모듈을 자동으로 착수하지 않는다.

Coordinate contract checked: calibration_controller.py, calibration_workers.py, calibration_workflow/jobs.py/state.py; native RGB u/v=[right,down] pixels, per-side grid/identity, host monotonic/RTCP/FTP timing evidence, K,D·optical RDF R,t; input snapshot/selection/quality/PNG 재열기 및 전체회귀 PASS.


## 2026-10-11 남은 계획 재개

사용자가 "남은 것도 마저 진행해"라고 새로 지시했다. 앞선 scope 완료 기록은 유효하며 새 변경의 검증 근거로 전용하지 않는다. 기준 소프트웨어2439pass/1skip 이후 남은 고정목록을 독립 파일 범위로 병렬 구현한다. 변경중 source로 테스트를 섞어 실행하지 않고, 관련 소유자가 쓰기를 마친 뒤 대상/통합 회귀를 묶는다. 새로운 기능·장치명령정책·좌표공식은 이 범위에 넣지 않는다.

M5/M12/M13은 root view에전체상태를남기는mixin대신각widget/model/selection상태를실제components로옮기고, root는layout/명령·snapshot조립과명시compatibility를맡는다. 기존표시축·RDF/FRD/NED·K,D/픽셀·GNSS source·DOF/selection 계약을유지한다. M4는 canonical→diskiterable과generation-owned scratch index로행수비례사본을줄이며schema/order/hash/rollback보존을검증한다. M7/M10은기존coretransport/protocol/state-machine을재사용하고실제thread/state/lifecycle만명확한owner로옮긴다. Root M6는 source선택·GUIwidget추출과순수projection, filequery를분리한다. 상태 IN_PROGRESS.


## 2026-10-11 Implementation status and bounded-memory evidence

사용자의 남은 계획 재개 지시에 따른 **현재 작업은 IN_PROGRESS**다. M1/M2/M8/M15의 마지막 구현과 M6 GUI query runtime 통합이 진행 중이며, 변경 중인 전체 source를 대상으로 한 새 targeted/full regression은 **NOT_RUN**이다. 아래 구조 구현·정적 검토와 별도 source에 묶인 메모리 실측을 전체 완료 PASS로 해석하지 않는다. 앞선 2439 passed/1 skipped 등 감사 기록은 당시 source에만 유효하며 삭제하거나 새 변경의 결과로 재사용하지 않는다.

### 현재 구현과 다음 검증

| Slice / 관찰할 경계 | 구현 및 보존 계약 | 현재 근거 / 다음 단계 |
|---|---|---|
| M5 Calibration view | capture/preview/input/quality/validation이 widgets·선택·표시 상태를 소유한다. 외곽 adapter만 legacy backend/panel을 읽으며 부품에는 명시 snapshot·named action·특정 signal을 전달한다. 500/1000/700 ms timer와 기존 layout·quality 승인·사진 선택 계약을 유지한다. | 독립 읽기 감사에서 broad backend 전달과 preview lambda 수명 문제를 발견해 수정했다. QObject side receiver, 본인 subscription만 disconnect, dispose 후 queued callback 차단을 구현하고 negative/lifetime 검사를 추가했다. syntax/static PASS; 새 실행 회귀 NOT_RUN. |
| M12/M13 Optical/Pixhawk view | frozen annotation·suggestion provenance·candidate ID/DOF·RTK 관측과 연결/MAG/GNSS/자세/geometry 표시를 실제 owner로 분리한다. compatibility properties는 같은 widget/model/state를 참조한다. source_row·revision·frame/hash와 선택/scroll/layout 계약을 유지한다. | 독립 읽기 감사에서 추가 blocker를 찾지 못했다. nested module graph·alias·source/좌표 표시 검사 작성; 새 실행 회귀 NOT_RUN. |
| M6 Evidence | live source/GUI값 선택과 supplied monotonic time은 외곽 adapter, telemetry/calibration/GNSS/MAG/geometry/recording/settings 표시는 detached value projection이 맡는다. `catalog.py`는 explicit data root의 file review query, `artifacts.py`는 package-relative artifact 해석을 맡는다. | 순수 projection/catalog 경계 구현. GUI query runtime은 진행 중이므로 file query 전체가 비동기화됐다는 주장은 하지 않는다. 새 targeted/full regression NOT_RUN. |
| M7 Device | 6종 관리 command와 4종 discovery/address 작업의 request snapshot·실제 worker/thread·GUI relay·partial-start·retirement를 전용 owner로 이관했다. PARAM 결과는 기존 core status를 해석하고 실제 MAG 선택 검증은 별도로 유지한다. | 같은 lane 배타성/기존 병렬성, old/duplicate callback, Stop 후 evidence, 실제 resource inventory, firmware evidence 실패 및 좌표 표시 검사 추가. syntax/static PASS; 새 실행 회귀 NOT_RUN. |
| M10 GNSS | `base.py`가 Base serial/survey/fixed-profile 자원을, `corrections.py`가 raw data-plane/injection 및 subscriber 교체를, `evidence.py`가 원래 measurement/session/context와 drain high-water를 소유한다. 기존 NTRIP·transport lock·protocol/state-machine은 재사용한다. | resource identity·GUI delivery와 raw queue·evidence drain을 구분한다. RTCM replacement fixture도 실제 correction owner/factory를 추적하도록 이행했다. 새 targeted/full regression NOT_RUN. |
| M4 Storage | canonical stream → generation-local disk index → Parquet/review/publication. live admission ledger를 복제하지 않고 기존 row order·schema/hash·정확한 정수·readback·rollback을 유지한다. | 아래 실측은 해당 저장 source의 scaling 관찰이다. 현재 통합 source의 새 targeted/full regression은 별도 NOT_RUN. |

M7 `connection_jobs.py`, `network_roles.py`, `maintenance_jobs.py`의 기존 API와 transport 소유권을 유지한다. 새 `qt_jobs.py`는 admin/discovery의 Qt 실행·결과·종료 메커니즘만 공유한다. thread가 실제 시작된 뒤 시작 API가 예외를 내더라도 접수한 작업을 잃지 않고, native 종료와 GUI terminal 소비가 끝날 때까지 실제 handles를 보존한다. FTP/firmware 결과는 원래 요청의 evidence로 남으며 untagged old callback이 새 작업을 덮지 못한다. GUI 승인, verified USB identity, explicit Connect, upload evidence 경로와 parameter 값/장치 명령 정책은 유지한다.

### M3 별도 180초 synthetic qualification

방법: 기존 `run_qualification`, qualification contract revision 2. 목적은 bounded IPC/native-frame/evidence 경로와 종료 자원 및 **부모 프로세스 RSS**를 같은 기준으로 관찰하는 것이다. 입력은 15 Hz synthetic 양쪽 카메라, native 320×180×3, preview 160×90×3, GUI stall 2초, evidence 간격 2초, 실행 180초다. loopback-only namespace에서 실행했으며 실제 camera/USB/외부 caster/GPU qualification이 아니다.

근거: [원시 결과](media-extended-180.json), [분석](media-extended-180-analysis.json), [source/config snapshot](media-extended-180-source.json), [실행 로그](media-extended-180-run.log). 로그는 `1 passed in 180.29s`이며 원시 결과의 **22 checks 모두 PASS**다. 전체 작업의 pytest pass count에 합산하지 않는다.

| 관찰 | 원시 결과에서 확인한 값 |
|---|---:|
| 부모 RSS 시작 → 종료 | 36.508 → 45.965 MiB |
| 부모 RSS peak | 50.922 MiB |
| 전체 구간 RSS slope | 189.028 MiB/h |
| RSS samples / 실제 샘플 구간 | 180 / 180.106789 s |
| 저장·수집·acknowledged evidence | 88 / 88 / 88 |
| 좌/우 stop latency | 0.027934 / 0.018086 s |
| 종료 후 child PID / shared-memory leak / cleanup error | 0 / 0 / 0 |

통과 기준은 변경하지 않았다: RSS growth ≤128 MiB, high-water ≤512 MiB, 전체 구간 slope ≤256 MiB/h, stop ≤0.5초, saved ratio ≥0.9(이번 실행 최소80쌍), writer shutdown timeout 5초. 시작과 최종 cleanup 샘플을 모두 포함한다. 분석 파일의 후반 window는 설명용이며 새 통과 기준이 아니다.

기존 [20초 baseline](media-baseline.json)과 [20초 after](media-after.json)는 둘 다 `rss_slope_within_limit` **FAIL** 상태로 그대로 보존한다. 이번180초 결과는 별도 현재-source 실험이며 과거 FAIL을 덮지 않는다. 자식별 RSS/GPU memory, 시간 단위의 장기 soak 및 실제 현장 처리율은 이 결과의 검증 범위 밖이다.

### M4 저장 scaling 관찰

방법: `streaming-finalization`의 canonical source, disk indexes, Parquet 및 재생성 review를 같은 profile로 비교했다. 목적은 행 수 및 geometry/epoch cardinality 증가 시 메모리/시간 변화를 분리해 기록하는 것이다. 5개 stream을 사용하며 review는 각 실행에서 `rebuilt and verified`로 보고됐다. 근거는 [source snapshot](storage-streaming-source.json) 및 다음 raw JSON이다. 측정값 자체를 새·완화된 PASS 임계값으로 삼지 않는다.

| 입력 축 / source artifact | 작은 실행 → 큰 실행 | Peak RSS | 경과 시간 |
|---|---|---|---|
| [행 수](storage-streaming-memory-row-scaling.json), 동일 schema/context | stream당 10,000 → 20,000행 | 206.059 → 212.145 MiB (+6.086) | 18.716 → 36.582 s |
| [index cardinality](storage-streaming-memory-index-scaling.json), unique contexts·row별 epoch·method bundles·spool updates | stream당 5,000 → 10,000행; geometry 5,000 → 10,000; epochs 10,000 → 20,000 | 197.965 → 212.371 MiB (+14.406) | 36.967 → 73.196 s |
| [단일 행 크기](storage-streaming-memory-large-row.json), stream당1행 | payload 1 → 4 MiB | 91.789 → 125.320 MiB | 0.703 → 0.892 s |

행 수 증가만으로 전체 Python 행 목록을 유지하지 않도록 바꿨지만, 개별 행 크기·metadata·동적 schema field·session measurement inventory는 여전히 입력 크기에 영향을 받는다. scratch SQLite는 save generation 안에서만 사용되고 성공/실패 뒤 제거되는 임시 자원이며 원본 spool/admission 거래의 두 번째 소유자가 아니다. canonical/readback/publication/rollback 및 원래 method/row identity가 저장 결과의 통과 계약이다. 새 전체 통합 source의 해당 회귀를 실행한 뒤 그 결과를 별도 추가한다.

### 문서와 좌표 계약

README 현재 상태와 Architecture의 view/device/storage/evidence/GNSS 절을 위 구현에 맞춰 갱신했다. 문서 변경은 새 런타임 명령·공식·저장 필드를 정의하지 않는다. Source 선택과 source clock, host monotonic 시간, 실제 worker/process 종료 및 원래 evidence context를 서로 대신하지 않는다.

Coordinate contract checked: README/ARCHITECTURE의 view·device·GNSS·storage 경계; native u/v=[right,down] pixels 및 frame/hash, optical RDF R,t·left-camera origin, public baseline-center FRD, world NED, quaternion wxyz body FRD→NED, WGS84 deg·ellipsoid/MSL m, NAV-SVIN ECEF m·CFG-TMODE3 cm/0.1 mm, MAG producer axes·ACC m/s²·GYRO rad/s·board deg/PX4 enum을 분리해 기술했다. 신규 관련 회귀는 source freeze 후 실행 대기이며 이 문서 갱신을 runtime PASS로 간주하지 않는다.


## 2026-10-11 first combined integration failures

- `remaining-optical-evidence-targeted.log`: 3 failed, 75 passed in 5.39s. Two optical cases exposed missing `_pose_mapping` facade export; restored named export and moved two production helper imports to concrete owners. Pure projection import guard exposed accidental Qt import through the old Pixhawk facade; geometry/magnetometer references now import the same constants/functions from pure policy/decode modules. Guard unchanged; single isolated rerun 1 passed in 0.34s.
- `remaining-integration-targeted.log`: combined 19-file run ended exit139 during QApplication deferred deletion; no trustworthy final pass/fail total. The fatal stack is retained. Do not count partial progress as a completed suite.
- `remaining-panel-debug.log`: first-error isolation 1 failed in 2.10s exposed three missing Presentation read ports after camera widget ownership moved. Added explicit read dependencies for `detect_pc_ip_button`, `apply_pc_ip_button`, `check_button`; added cross-feature read-port gate. No assertion was weakened.
- Independent review also found `stop_all()` still clearing admitted camera/FTP handles before queued retirement. Camera owner is fixing that path; production ownership exception must be resolved before rerunning broad UI tests.
- Plan M8 still had Pixhawk/MAVLink widget/state dependencies in CameraFeature. The remaining work now moves that existing responsibility to Diagnostics; camera retains two named view composition commands. This fulfills the original responsibility boundary, without adding device behavior.


## 2026-10-11 isolated owner/component recheck

- `remaining-components-targeted.log`: **150 passed in 5.62s**, warnings0, exit0. Files: test_calibration_workflow_boundaries, test_calibration_job_owner, test_calibration_view_parts, test_optical_pose_parts, test_pixhawk_view_parts, test_workspace_architecture. This validates standalone ownership/GUI delivery/component/import boundaries, not the unfinished common panel rewire.
- GNSS new owners, excluding actual panel fixture pending M8 composition: **9 passed, 1 deselected in 0.76s**, exit0. Command: `apps/Workstation/tools/run_isolated_tests.sh -q -ra --tb=short apps/Workstation/tests/test_gnss_runtime_boundaries.py -k 'not real_feature'`. The deselected actual panel case remains in the upcoming combined run.
- Two delegated test requests first failed before pytest due sandbox netlink denial. Escalated redirected-shell requests were interrupted while awaiting approval and produced no session/process. Root reran the approved runner without shell redirection; subprocess output was saved after execution. These denied launches are not test failures or test evidence.


## 2026-10-11 shared panel integration stabilized

- Completed M8 Diagnostics view ownership: 52 widgets plus 4 candidate model fields, 21 moved public operations and 3 explicit build/snapshot ports. Camera dependency declarations 177→104; writable declarations 99→38. Added actual widget identity and candidate store tests.
- `stop_all()` now routes actual camera/control, calibration jobs/FTP, optical capture, network roles, device connections/discovery/admin owners through their stop/retirement paths, excluding admitted handles from generic `None` cleanup. Four stopped-native/queued-GUI regression cases added.
- `remaining-panel-recheck.log`: new lazy-owner test initially assumed a preinitialized None field; actual supported contract is absence until first use. Changed the test to assert no owner via getattr(default=None), retaining the no-instantiation and single-resource checks. `remaining-panel-recheck-2.log`: **26 passed in 6.26s**, exit0, no warnings.
- `remaining-integration-targeted-2.log`: **1 failed, 437 passed in 37.04s**, exit1, no warnings/native crash. Sole failure finds a real duplicate GNSS recording-row owner: panel recording binding and new evidence state address different SpoolRows. Fix preserves Recording as authoritative owner and binds GNSS evidence through a finite alias; pending recheck.
- Portable path policy, repository policy(--allow-existing-debt), scoped diff check PASS after final M8 composition.


## 2026-10-11 full regression and subagent status audit

The first frozen remaining-scope candidate (647 Python hashes, `remaining-completion-source.json`) finished **4 failed, 2642 passed, 1 skipped in 328.60s**, warnings0, exit1. Raw: `remaining-completion-full-regression.log`. Two read-only agents then independently separated implemented ownership from strict residual policy; the user-requested classification is preserved in `status-audit.md`. That checkpoint identified M2/M7/M8/M10/M15 residuals rather than claiming all 15 complete. Those specific residuals are the fixed follow-up scope.

Failure repairs so far:
- Pure optical observations imported NativeFrameArtifact at runtime, pulling Qt into a computation leaf. OpticalObservationHistory moved unchanged to observation_history.py; the pure module now uses TYPE_CHECKING for its artifact annotation. The no-Qt guard remains and now also rejects the history/artifact store dependency. Class plus 14 pure function AST bodies remained identical.
- The native-store GUI test expected at least five timer firings during a fixed 200ms writer sleep. Original single-test recheck passed in0.32s; no production native-store change was present. A causal Event barrier now holds actual writer persistence until five GUI timer events release it. Existing10ms/5ticks/enqueue50ms/completion2s criteria remain. This removes the schedule-count race without loosening responsiveness.
- `remaining-optical-native-recheck.log`: **25 passed in 2.02s**, exit0, covering native-store, pure solver boundary and optical workflow.
- The two camera fixture failures and strict residual policies are being handled by their exclusive owners before a new combined candidate. The failed candidate source and log will not be overwritten.


## 2026-10-11 strict residual policy follow-up

The status audit fixed the remaining implementation list to M2/M7/M8/M10/M15. No unrelated feature or device policy was added.

- M2: minimum effective-pose quality decision moved to CalibrationQuality; approved/attempt profile persistence and alias publication moved to CalibrationProfiles; session directories/manifest, archives and zoom provenance moved to their explicit I/O owners. Existing WARNING attempt-only and persistence order retained.
- M7: frozen PersistentEthernetInputs/Configuration own SYSID, MAV_2 parameters, extras/net.cfg and persistent/bootstrap command construction. Golden tests compare full UTF-8 bytes and fallback/role/override behavior.
- M10: frozen network admission preserves guard ordering, assisted-base 15-second monotonic freshness, GGA and ellipsoid checks. Operation begin and Base acquisition reset now have explicit owners; current GGA/serial/survey state and Recording row port are preserved.
- `remaining-calibration-policy-targeted.log`: **24 passed in0.31s**. The orchestrator attempted to store an absent session_id after the already-completed run; this was a log orchestration error, not a test failure. The full pytest output was recovered from stored stdout.
- `remaining-policy-integration.log`: **327 passed in22.67s**, exit0,warnings0. Covers Calibration workflow/existing behavior, admin commands, GNSS/NTRIP, network-role application, real panel composition and application lifecycle.
- M8/M15 production rejoin flow is frozen. Its existing camera fixture migration and final rejoin regression cases are pending before the next combined full candidate.


## 2026-10-11 final candidate 2

M8/M15 rejoin transaction is implemented in CameraControlRuntime plus explicit snapshot/effects in camera_runtime/rejoin.py. The same generation/side/timer owner governs recording preparation, segment reuse/allocation, pane start, recovery generation and peer availability; Stop/accepted Start invalidates old subsequent effects. The enable-detection flag now aliases that same Camera owner; Detection continues owning normal yolo_enabled.

Four old camera fixtures now inject the actual candidate/control/recovery owners and retain their assertions. Seven rejoin cases cover segment rules, persistence failure, reentrant Stop during recording/allocation/start and probe failure without consuming media input. `remaining-camera-rejoin-targeted.log`: **160 passed, 1 skipped in19.52s**, exit0,warnings0. Skip is unchanged unsupported NVENC.

The fixed five-item residual scope is implemented. Final combined regression is running against **649 Python source hashes** plus the delegate stub in `remaining-completion-source-2.json`. Earlier source/log artifacts remain unchanged. No final PASS is claimed until that run finishes and the source manifest is rechecked.


## 2026-10-11 final software closure — PASS

- Command from repository root: `apps/Workstation/tools/run_isolated_tests.sh -q -ra --tb=short`. Loopback-only namespace, offscreen Qt, no hardware startup.
- `remaining-completion-full-regression-2.log`: **2696 passed, 1 skipped in327.64s**, exit0, failures0,warnings0. Skip is existing unsupported NVENC, not a skipped decomposition check.
- `remaining-completion-source-2.json`: all649Python source hashes plus delegate stub rechecked after execution; differences0, newPythonfiles0. Machine-readable result: `remaining-final-verification.json`.
- Fixed M1–M15 responsibility/state/resource scope and G1 repository generation contract are complete in software. The five strict residual policy items identified by the user's subagent audit were implemented and included in this final candidate. The earlier647-source audit and failed full run remain preserved.
- Portable paths, repository policy(--allow-existing-debt), delegate stub check and scoped diff whitespace check PASS. New required source/tool/interface files are inside the repository:333currently untracked(including earlier work),0ignored required source files,0required source targets outside the tree. See `remaining-git-portability.json`. No staging/commit/push. Existing unrelated root/Simulation changes retained.
- Verification limits: unsupported NVENC; no physical camera/Pixhawk/NTRIP command qualification; no long-duration child/GPU memory proof. M3's180s22checksPASS is parent-RSS synthetic evidence and does not erase old20s slopeFAILs. M4 removes row/context/index eager accumulation but single-row, metadata/schema/inventory scale remains a stated bound.
- Coordinate contract checked: full StereoWIRIS coordinate/stereo/triangulation tests plus Workstation regression cover native pixels, optical RDF, body/baseline FRD, world NED, GNSS ECEF/ellipsoid/MSL, source/host clocks and existing JSONL/CSV/Parquet/XLSX schemas. No coordinate conversion policy changed.
