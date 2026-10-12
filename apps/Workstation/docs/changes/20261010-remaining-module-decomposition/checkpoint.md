# Remaining decomposition checkpoint — current 2026-10-11

사용자 지시 "남은 것도 마저 진행해"에 따라 원래 고정 목록의 남은 구현을 진행했다. M1/M2/M4/M5/M6/M7/M8/M10/M12/M13/M15의 남은 소스는 현재 freeze. Root가 공통 FIELD_OWNERS/state aliases, optical measurement snapshot, AppLifecycle 실제 owner shutdown/inventory, PanelApplication query registry, MainWindow view dispose를 통합했다. 초기 기준은 calibration-completion-source.json의 2439 passed/1 skipped/0warnings(312.58s)이며 이번 소스를 검증한 결과는 아니다.

최종 완료 상태(2026-10-11): 원래"남은 것도 마저 진행해"의 고정 M1–M15/G1 소프트웨어 범위를 마감했다. 이후사용자의서브에이전트분류요청은2명읽기감사로10개구현/5개엄격잔여를구분했고 status-audit.md에보존. 그5개(M2/M7/M8/M10/M15)정책/거래까지추가분리한최종후보가전체PASS했다.

최종증거: remaining-completion-source-2.json649Python+stub, remaining-completion-full-regression-2.log **2696passed1skip0failed0warnings327.64s exit0**. session82567종료. 검사후hash변경0/새Python0. remaining-final-verification.json. 첫647source full4failed2642passed1skip328.60s와raw로그는보존. 이전native/solver/두camerafixture실패수정검증완료. 현재테스트/agent 작업/백그라운드프로세스없음.

RelatedPASS: components150/5.62s,panel26/6.26s,GNSSRecordingvisual61/9.00s, optical/native25/2.02s,Calibrationpolicy24/0.31s,strictpolicyintegration327/22.67s,camera rejoin160pass1NVENCskip/19.52s. 최종full이모든현재소스를덮는다. 추가코드변경없으면검사반복불필요.

현재분류:15개모두계획한책임·상태·자원분리구현검증. 큰adapter4개 calibration3380/device3648/gnss2557/camera2414lines는남음(줄수완료기준아님). 미검증:실장비,장시간child/GPU memory,NVENCunsupported. M4단일row/metadata/schema/inventory크기한계. M3old20sec2slopeFAIL유지,180sec22checksPASS(parentRSSonly). G1generatedbyteexact재현체계유지.

README/ARCHITECTURE/plan/status-audit/verification최종상태반영. 최종포터빌리티/정책/stub/whitespace검사후응답마감필요. 현재필수code/tool/interface333개untracked(이전작업포함),ignored필수소스0/저장소밖필수소스0;remaining-git-portability.json. 커밋/푸시요청없음·미실행. root/Simulation기존변경보존.

Final응답은사용자의분류요청에맞춰완료된소프트웨어분리와남은큰adapter/운용검증위험을구분하고status-audit.md링크,2696PASS1skip,untracked/commit미실행,Coordinate contract checked줄포함. 전체위험완전해소/실장비검증주장금지.

아래는 완료 전 경과 기록이다. 현재 상태와 실행 범위는 위 문단 및 verification.md의 마지막 완료 기록이 우선한다.

## 작업 환경과 기록

- CWD: SASL/apps. 많은 사전 미커밋/신규 파일이 있다. unrelated reset/checkout/정리 금지. 허용 쓰기 root는 apps와 /tmp.
- 유일한 계획/ledger: 이 디렉터리 plan.md, verification.md. baseline.json은 최초 15개 대형 모듈 상태이며 이미 변경된 현재 상태가 아니다. 각 source JSON/full-regression.log는 해당 시점 증거다.
- 스킬: maintain-boundaries, design-boundaries 및 routing/minimum-sufficient-change/architecture-refresh; verify-check, run-tests, update-readme, auto-compact. M7의 실제 explicit Connect 계약과 APPLIED · SAVE FAILED 안내를 README에 반영했다. 기존 README의 자동 reconnect 설명은 코드·테스트와 어긋난 과거 설명이었다. 외부 architecture HTML refresh는 NOT_REQUESTED.
- root 실행: `apps/Workstation/tools/run_isolated_tests.sh -q -ra --tb=short [tests...]`. prefix 승인. Sandbox netlink 차단 시 require_escalated로 같은 runner 실행(외부 네트워크 허용/일반 pytest 우회 금지). env는 repo venv/Qt offscreen/loopback namespace.
- CWD apps에서 formatter: `../.venv/bin/python -B -m ruff format --no-cache <Workstation paths>`; root에서는 `.venv/bin/python` 및 `apps/Workstation/...`를 쓴다. Cache 없이는 apps 내 수정만 가능.

## 완료된 검증과 현재 코드

1. **M9 Pixhawk**: pixhawk_link/{types,decode,timebase,policy,endpoints,wire_commands,transport,commands}. 22개 연결/lock/queue/liveness 필드와 17개 동작을 MavlinkTransport가 소유. 8개 필수 callbacks로 Qt subscriber/source 정책과 연결. CommandPorts 3개(io_for/reserve/health), MavlinkCommands는 PARAM/Shell/RTCM/FTP/reboot 조립 및 RTCM 로그 시각을 소유. 기존 subscriber field는 같은 실제 객체를 향하는 명시적 property. Full 2298 pass, 1 NVENC skip, 0 warnings.
2. **M7 일부 연결 jobs**: device_runtime/connection_jobs.py와 state.py. Connect/Disconnect worker+QThread ticket, GUI-affine relay, first terminal, current subscriber 확인, 실제 finished 후 정리. DiagnosticsState 네 handle aliases. PanelFeature에 생성 시 state_factory 추가(직접 self.state 교체는 구조 검사에서 거절되어 올바르게 수정). Full 2303 pass/1 skip/0 warnings. M7 전체와 GNSS controller는 아직 남음.
3. **M11 WIRIS**: 12개 QObject worker를 wiris_workers/{clock,network,capture,ftp,commands,probe}로 이동. io.py=7개 stateless 효과 함수, policy.py=상수/parser/barrier. wiris_runtime_workers.py 54줄 facade. class identity/__module__ legacy marker, normalized AST 모두 보존. 7개 생산 소비자 concrete imports, private mock injection 4개 test 파일 actual owners로 수정. Full 2315 pass/1 skip/0 warnings.
4. **M4 S6a**: workstation_core/session_storage/{schema,values,paths,models,spool,canonical,generation,recovery,review,writers,manifest,legacy,finalize}. recording_session.py 181줄 facade. 123개 정의 AST를 보존했으나 초기 method-result stream 등록 for-loop를 처음 놓쳐 22 failures; 원래 loop를 schema에 복구하고 runtime registry 카탈로그 검사 추가. Dynamic fault_target injection도 actual canonical owner로 수정. 12개 생산 consumer concrete imports. Full 2319 pass/1 skip/0 warnings.
5. **M4 S6b 일부**: 파생 Parquet를 위한 pair/camera/PX4 전체 행 사본을 lazy row copies로 변경. Source materialization/context/index/Excel 메모리는 아직 미해결. `tools/qualify_session_projection_memory.py` synthetic 5 streams offline benchmark, eager reference vs lazy separate sequential processes. 10k/stream: 285.070→240.086 MiB, 11.079→11.025 s. 20k: 384.613→306.195 MiB, 21.869→21.492 s. 전면 bounded-memory 또는 일반 속도 개선 주장 금지.
6. **M6 일부**: evidence_views/{snapshot,telemetry,values}. EvidenceAdapter.attitude/gps/detection은 선언된 attribute와 provenance를 수집하여 detached readonly inputs+now를 순수 projector에 전달. No panel/widget/clock in projector. Explicit frame/unit tests and mutation tests. S6b 일부와 함께 Full 2326 pass/1 skip/0 warnings. 나머지 evidence geometry/query/GUI I/O 경계는 남음.
7. **M1 일부**: optical_pose/{geometry_values,auto_selection,candidates}. Auto job이 전체 controller clone/live panel을 보유하지 않는다. Captured native image(QImage deep capture/COW detector copy), immutable K,D/prior, captured/explicit NED baseline and warnings, scalar settings+detector functions만 가진다. _clone_for_auto_select 및 clone flags/중복 helper 제거. Core detector 정책/commit stale frame checks 유지. CandidateRegistry가 후보/prior/rotation context/active poses-selection/history 실제 소유. Legacy names are identity aliases. Full 2331 pass/1 skip/0 warnings. Root optical controller 약 5k 줄로 다른 solve/capture/apply 분리가 여전히 남음.

## 추가로 완료한 분리와 검증

8. 공유 active geometry: 단일 ActiveGeometryStore를 Camera/Calibration feature에 주입하고 17개 legacy property를 같은 store로 연결. apply/restore는 준비된 geometry bundle을 한 revision으로 반영. 전체 **2336 passed, 1 skipped in 248.16s**. `geometry-full-regression.log` 저장됨.
9. M2 feedback: `calibration_workflow/capture_feedback.py`에 capture root별 counters/last state/inventory cache와 10개 메서드의 실제 상태·행동 이동. Controller는 alias/wrappers. 촬영 worker 자체는 아직 별도 후속.
10. M1 solver: `optical_pose/{horizon,terrain_solver,mountain,drone,observations,policy}.py`. Horizon/Terrain/Mountain/Cloud/Drone yaw·multiview 계산은 명시적 K,D/prior/frozen annotation/artifact/reference/observation inputs만 받고 후보(+drone context)를 반환. Controller가 파일·live source 선택·입력 복사·등록·화면 갱신. 수식/좌표/quality/advisory policy 동일. Horizon/Terrain AST 별도검토 통과, 새 직접 Horizon/PnP 비대칭 NED→UV oracle/stale identity/fresh-process no Qt/controller import tests. Feedback 포함 전체 **2339 passed, 1 skipped in 253.66s**, warning 0. `solver-source.json`, `solver-full-regression.log` 저장됨.

## 지금 진행한 M7 NIC 역할 runtime

- `device_runtime/network_roles.py` NetworkRoleRuntime와 NetworkRolePorts 7개 callback, NetworkRoleJob와 GUI-affine relay가 요청/정지 polling timer/worker/완료/저장 상태를 소유. 새 owner는 panel/whole-controller를 받지 않는다.
- `camera_feature/state.py` CameraFeatureState에 network_role_*6개 properties. CameraFeature가 state_factory로 생성. 실제 6개 필드가 과거 VIEW_FIELDS에 있었던 것을 제거해 실제 state 하나로 이행. (이 분류를 놓쳐 처음 alias와 view 두 저장소가 생겼고 연속 요청이 막힘. RCA로 밝혀 고쳤음.)
- DeviceRuntimeController는 설정/role 요구/양쪽 MAVLink endpoints/candidate capture 및 UI/설정 publication adapter. 실제 OS network mutation과 rollback은 원래 network.py에 남음. explicit Connect 정책 유지. 기존 dead reconnect/rollback adapters는 여전히 호환 부채. 전부 이동됐다고 주장 금지.
- 재현한 save exception 영구 busy: host 성공 이후 저장 실패는 host ownership 유지, `APPLIED · SAVE FAILED`, operation 종료. 실제 OS rollback receipt가 없으므로 임의 rollback/성공주장 없음.
- 결과는 job/operation identity/첫 terminal 검사. actual admission에 transaction_id; token 없는 legacy apply/fail/clear는 admitted job에는 무효. actual relay만 내부 _apply_result/_fail/retire 사용.
- 표시 알림 exception은 presentation_error에 남기고 _finish_status finally로 operation 종료, retire 표시 오류도 cleanup 막지 않음. owners_stopped query failure→명시 ERROR 종료.
- AppLifecycle는 이미 생성된 production owner/legacy fallback owner의 prepare_shutdown 호출, timer를 멈추고 신규 worker를 막음. 기존 진행 중 OS 작업은 결과/retire를 수용. legacy owner resource inventory 포함.
- 입력 mutation test는 거래를 시작한 뒤 주소/peer/양쪽 endpoints와 candidate를 바꾸어도 승인 당시 값으로 OS 요청·설정 저장됨을 실제 panel에서 확인.
- 최신 targeted 결과 **150 passed in 23.79s**, warnings 0. 독립 감사의 5개 잔여문제 모두 최신 source에서 해결 확인(테스트는 root 수행).
- M7 전체 회귀 완료: **2352 passed, 1 skipped in 241.83s**, warning 0. `network-role-source.json`/`network-role-full-regression.log` 저장 완료. 이전 session9851은 닫혔음.

## M3 media owners 적용 완료, 전체 검증 진행 중

- `workstation_core/media_stream/`의 policy, configuration, decoder_policy, detections, events, rates, worker_threads, rtp_timing, recorders, inference, frame_output, gstreamer_backend, opencv_backend, pyav_backend, child_runtime, ipc, supervisor에 기존 실제 class/function/상태/자원 이동.
- `yolo_stream.py`는 이전 imported module object와 166개 정의/상수의 explicit facade. 89개 class/function body AST 동일(ruff의 nonlocal/global 선언 순서만 정규화 가능), 새 top-level resource 생성 없음. 클래스 __module__ legacy marker 유지. 원래 SASL_ROOT의 __file__ depth는 동일한 runtime_paths.REPO_ROOT로 교체.
- 11개 생산 소비자 actual import: camera_stream_controller,settings_controller,panel_support,panel_detection,qualify_camera_backpressure,panel_camera,wiris_workers/probe,recording/metadata,recording/session_stop,rtcp_clock_observer,camera_pane. panel_compatibility_exports와 wiris_runtime_workers 외곽 facade의 기존 imports는 유지.
- 4개 테스트의 private injection은 실제 소비 모듈 gstreamer_backend로 옮김(Tracker도 호출자가 lookup하는 gstreamer module을 patch). direct assignment/restore 포함. test_workstation_yolo_stream의 receipt→clock 샘플링 원문 순서 검사는 actual backend 함수 inspect.getsource로 같은 assertion 유지.
- `test_media_stream_boundaries.py`: 6개 기존 pickle bytes fixture(`tests/fixtures/media_stream_legacy_types.json`), fresh-process에서 Qt/torch/ultralytics/cv2/gi/facade premature import 금지 + negative fixture, **실제 child_runtime spawn**에 local lossless PNG→정확한 RGB bytes/native size/monotonic ns 및 파일 열기 실패/종료 검사.
- 첫 media targeted **160 passed,1 skipped in 9.00s**, 후속 Camera/Settings/WIRIS/real panel +9 boundary tests **147 passed in25.72s**. NVENC unsupported skip 유지.
- 독립 read-only agent는 89개 AST body 동일, single definitions, no top-level resources, root-path 동일 확인. 나머지 import/spawn/audit 결과는 pending일 수 있음.
- Synthetic benchmark(20sec, GUI stall2sec, evidence2sec, 기존 default limits) before/after **둘 다 rss_slope_within_limit만 FAIL**. 기준 완화 안 함. before peak60,325,888 B, slope1,143,825,344.78 B/h; after peak61,956,096 B, slope1,808,738,175.44 B/h. Peak+1,630,208B. 둘 다 durable pairs8, IPC/evidence/absoluteRSS/Stop/cleanup 통과. 장시간 누수/비퇴행/PASS 주장 금지. startup 포함 짧은 구간이므로 한계 명시. 원시 `media-baseline.json`, `media-after.json`. 이전 repo smoke2sec는 큰 메모리 bounds라 slope proof 아님.
- 비교용 /tmp 원본/추출 script는 편의용이며 runtime 의존 아님: workstation-yolo-before-owners.py,workstation-media-owners.json,prepare_media_owners.py,install_media_owners.py. Repo evidence `media-definition-owners.json`, `media-source.json`.

## M14 command declarations 적용 완료

- `_build_commands`의 101개 spec을 `command_bindings/` 12개 능력별 선언 모듈로 이동. 실제 domain/backend 호출 handler들은 MainWindow에 그대로. 등록함수는 registry와 명시적으로 나열한 callback들만 받음(window/backend/state 전달 안 함). 총85개 기존 method 연결이 외곽 composition에 남음. 그룹별 등록 순서 보존.
- `tests/fixtures/main_window_command_catalog.json`: 이전 함수의 선언만 fake registry/callback에 실행해 저장한 ID/text/tooltip/category/shortcut/checkable/deferred completion 및 모든 호출 파라미터. 원래 handler들을 실제로 실행하지 않았고 device effect 없음.
- `test_command_binding_catalog.py`: 모든101개 항목/순서/params 동등, action 누락composition TypeError, 금지 window/backend import와 negative fixture.
- 추출 script는 처음 /tmp sys.path 및 MAVLINK_QUALIFICATION_S globals 누락으로 적용 전 실패; root source 유지. 이후 수정하여 적용. 기존 command baseline+newmedia RTCP **129 passed in25.28s**. 추출 후 command/UI/nav/lifecycle/architecture/optical workspace **127 passed in35.12s**.
- MainWindow._build_commands만 국소 format, 나머지 root source 스타일 보존. AST extraction scripts=/tmp/extract_command_bindings.py, 원문=/tmp/workstation-main-before-command-bindings.py.
- M14 optical worker lifecycle은 아직 창에 남으며 전체 M14 완료 아님.

## M15 readiness 적용 완료

- `camera_runtime/readiness.py` StreamReadinessTracker가 camera/video generation+readiness 4개 실제 필드와 시작/관측/판정/side실패/timeout/취소를 소유한다. Qt/UI import 없음; video stat/time 읽기는 기존 위치/조건대로 수행. CameraFeatureState와 PresentationReadinessState가 PanelApplication의 동일 tracker를 공유한다. 4개 기존 state 필드는 VIEW_FIELDS가 아니었음을 확인했다.
- current generation은 controller의 pane 경계, captured expected generation은 tracker에서 순서대로 검사. partial 시작, INPUT UNVERIFIED, MP4 positive byte growth, 녹화 실패에도 Measurement 계속 정책 보존. Camera cancel과 video cancel 독립. 기대 generation 입력 map은 복사. Legacy raw attrs는 동일 live state aliases.
- controller._readiness_owner는 앱의 실제 tracker 또는 4개 이름만 반영하는 lightweight fake adapter. Pane start/stop/UI/source publication/Qt scheduling/retry/watchdog/live recovery/finalization은 아직 controller에 남는다. M15 전체 완료 아님.
- 원문 `/tmp/workstation-stream-before-readiness.py`, 적용 script `/tmp/extract_stream_readiness.py`. 최초187 pass/1skip16.72s, owner/generation/partial/timeout/byte growth189? **191 pass/1skip17.16s**. disabled video일 때 미사용 path settings를 읽지 않는 예외 의미도 추가로 보존/검증함. source readiness-source.json은 그 이전 targeted version이므로 최종full source를 우선.

## M14 optical auto-selection runtime 적용 완료

- `optical_pose/auto_runtime.py`가 task identity, Qt worker/thread/command_id, queued GUI relay, first terminal, retire를 소유. MainWindow의3개 legacy handle properties는 실제 owner에 연결. prepare/commit/표시는 root GUI adapter가 담당.
- 같은 job이 Qt finished까지 owner.busy를 유지; 이전 task outcome/finished는 새 task에 무효. Worker cancel은 원래 queued-only 의미. 창 close는 기존5초후force attempt +500ms wait를 유지하지만 실제 종료 실패면 참조를 버리지 않고 계속 기다린다. force_stop은 task를 authoritative source로 삼아 legacy 표시 handles가 먼저 비워져도 실제 task 자원을 보존.
- PanelApplication.optical_auto_runtimes는 생성된 owner들을 중복없이 등록, destroyed에서 제거. 고유 resource keys로 inventory에 포함. AppLifecycle는 모든 등록 owner에 stop을 요청한 다음 all 결과를 판단하여 뒤쪽 owner 취소를 단락평가로 건너뛰지 않음.
- 초기입력의 run과 factory의 QObject/run/cancel capability 검사. thread.start 예외 및 시작되지 않은 반환에는 실제 비실행 상태에서 reservation 해제, 부분 시작이면 실제자원 유지. cleanup cancel 오류도 예약 해제를 막지 않음. MainWindow의 시작 실패 busy 표시도 실제 owner상태와 일치.
- 테스트: 기존102 pass18.90s→owner/force/capability/multi-owner 포함107 pass19.38s; closeEvent force실패44 pass7.64s. 독립 감사가 발견한 thread.start 실패 정리 누락 수정. 실패주입 첫2failed68pass는 Qt C-extension descriptor 수동복원 fixture 오류였고 monkeypatch.context로 수정(생산 assertion 완화없음). 후속70 pass8.37s, 마지막owner+readiness13 pass1.96s.
- **M15+M14 전체 완료검증:2377 passed,1 skipped260.05s**, exit0,warnings0. `readiness-auto-source.json`(모든 Workstation Python hash)/`readiness-auto-full-regression.log`. session90931은 완료·닫힘. No active full suite.
- Docs/ARCHITECTURE와 README의 Close 실제종료 대기 안내 반영. Source/units/좌표 변환 변화 없음. 원문 `/tmp/workstation-main-before-auto-runtime.py`, 적용script `/tmp/integrate_optical_auto_runtime.py`.

## G1 generated MAVLink 재현성 완료

- 기존 generated 27k-line Python을 분리/수정하지 않았다. 설치 pymavlink2.4.49 common.xml은205messages vs runtime235messages, missing30/changed18여서 직접기반사용불가.
- Repository environment_report에서 보고서가 가리킨 별도 PX4 checkout의 matching source를 찾아 읽기만 함. PX4 7b1e54889d5bf8120f843fd88760ab4b6e2825f5, mavlink51a47ee9aed1216532f2eefbfe8d260a2e3cabcc, bundledpymavlink5977f44504f01715a44bed44a4601e6c12cef92d. 원 common blob4b37e5eb...; working common.diff가 tracked CAN_MAG patch와 정확히 같음.
- GitHub API는 inaccessible/404/422, 한번 DNS차단은 지시에따라 require_escalated재시도했음. 외부웹 증거를 성공 주장에 사용하지 않음. 원본 확인은 로컬Git+XML+byte comparison 근거.
- `workstation_core/mavlink_dialects/definitions/{common,standard,minimal}.xml,mavschema.xsd,COPYING` 761,351B repo에 보관. generation_manifest.json은 input/generator hash,원revision,trackedpatch,235messages,SASL message180 raw sensor-frame Gauss,boot us,CAN1/2,CRC233 계약.
- `tools/regenerate_mavlink_dialect.py --check`: pymavlink2.4.49+relevant generatorfiles SHA확인→원 firmwareXSD로 schemaFile임시지정→XML validation enabled 확인→임시 canonical filename로generate→기존 runtime와byteexact/sha 비교. finally schemaFile복구. 기본check는 runtime수정안함. --output은preview명시 경로에code생성하며basename과무관DIALECT유지.
- 단순wheel XSD는 superseded metadata를 거부함. validation을끄지 않고 정확한 원XSD 사용하여 **1,919,084B SHA2567ed861c4d01a48e1c972cd1936b4410e29f209f1e428178f704564868f4ea2ba** 완전재현. Generator의 upstream zero-bitmask diagnostic2개 유지.
- 테스트 `test_mavlink_dialect_reproduction.py`: 전체exactbytes, changed input hashfail, 의도적수정XML의strict검증,생성CANrawgoldenpacket+독립struct unpack+CRC/frame/units. 최초1failed139pass8.13s는mavgenSystemExit를libraryAPI가그대로노출한것; validation상태를보존한RuntimeError로정규화. **140passed8.12s** 재검증. generatedPython과firmwarebinary변경없음.
- 패키지 README/초기docstring과ARCHITECTURE에 repo-local 재생성안내. rootportablepaths/repopolicy/diff-whitespace PASS. 새필수input도모두repo내부,untracked상태. commit/push미요청·미실행.

## 다음 실제 우선순위

1. **현재 background process 없음.** 전체2377pass source 뒤 G1 tool/docs/XML/tests만추가됐고 runtime generated내용은변경없음. G1target140pass로검증. 다음runtime 변경과전체회귀를묶을것.
2. **M10 GNSS NTRIP credential/GGA 경계를 읽기 시작했으며 아직 코드변경없음.** `gnss_runtime_controller.py`도 아직분리시점이전내용이다(이전사전변경 worker파일은그대로).
3. M10 첫후보: `NtripCredentialError`, frozen `NtripCredentialResolution`, `_ntrip_credential_account_key` helpers, `_resolve_ntrip_credentials`1238~1320, `_maybe_persist_ntrip_credential`1320, `_commit_pending_ntrip_credential`1335. 현재priority는explicit UI password(accountkeymatch)→keyring→legacy env→untouched deploymentfallback. username없으면legacy env를먼저시도. 새owner는panel대신캡처한network/sessionpassword/explicit/key를받도록. pendingcredential은caster200 이후만save한다. 새state소유권을옮길경우oldconnected callback의sourceidentity도확인하고secret을로그/스냅샷에넣지말것.
4. M10 GGA 후보 `_resolve_gga_sentence`1058~1238. policy BASE_GNSS/MANUAL/rover midpoint,15secstale와30sec로그throttle. `_gnss_gga_provider`가성공시skipreason/time을비움. `.latest_gnss_base_position`/`_selected_rover_gps_state`로원래source선택. 전체본문·producerheight datum(altitude_m vsellipsoid/geoid),UTC/NMEA출력contract를읽은뒤설계. 아직그부분결론없음.
5. M1capture/application/observations, M2capture/solver/profile, M4boundedsource/context/Excel, M5/M12/M13views, M6나머지, M7maintenance/parameters, M8camera, M10, M15retry/recovery/finalization 남음. M14主要command+auto owner/G1완료검증이있어도 전체15moduleplan완료아님.

## 작업 규율/사용자 steering

사용자는 처음분석→각모듈분리계획→안전한범위수정시작을승인했고,두번조기종료를지적했다. 최신"왜오래걸려?"에는범위/반복4분전체검사/실제회귀수정때문이라설명하고,focused검사후독립변경묶음별전체검사로줄이겠다고답변했다. 중지지시가아니므로기존승인작업계속. 질문으로재승인요구하거나중간PASS에서임의최종종료하지말것.

- CWD SASL/apps,쓰기 apps,/tmp. root도구는SASLrootworkdir에서runner `apps/Workstation/tools/run_isolated_tests.sh`(require_escalated prefix승인),pathsrepo-relative. plainpytest/외부device네트워크우회금지. Native/GPU미지원1NVENCskip계속. 실제장비commands/flash없음.
- root정책검사 python3 tools/check_portable_paths.py; python3 tools/check_repository_policy.py --allow-existing-debt. 형식ruff는apps에서 ../.venv/bin/python -B -m ruff format/check --no-cache.
- Unrelated root/Simulation미커밋작업많음,reset/checkout/정리금지. 새owner패키지/테스트/XML/pinnedrequirements지원파일은untracked이므로나중공유commit에포함필요. 외부PX4source는read-only조사후필수XMLrepo복사완료;outsidecheckout을runtime의존으로남기지않음.
- read-only agent `/root/audit_optical_solver_boundaries` idle. 최신verify-check의명시적selective-subagent검증지시에따라1명만필요한감사에사용했음. 일반proactive delegation금지.
- sourcechanges동안runningfullsuite에동시edit하지말것. checkpoint는stale이므로각sourcejson+ledgerlatest를우선.
- 최종답에는 `Coordinate contract checked: ...` 필요. 마지막전체2377pass/1skip +G1별도140pass를합산하지말것. RSS20secqualification은before/after둘다FAIL(상기수치),장시간/메모리비퇴행PASS라고하지말것.

재개최우선: M10 GNSS credential/GGA의 실제state/callback/frame/time계약을읽고다음좁은분리단위를수행한다.


## 최신 재개 상태 — G1 이후 (앞의 M10 미착수 메모를 대체)

- 목표/승인: 전체 남은 plan의 안전한 분리를 계속한다. 사용자가 중간 PASS 뒤 조기 종료에 두 번 항의했다. 최신 "왜 오래 걸려?"는 상태 질문이며 취소가 아니다. 관련 검사 먼저/독립 변경의 전체 회귀 묶기로 시간을 줄인다. 실제 장비 명령·flash·commit/push 수행하지 않는다.
- 가장 최근 전체 검증: `ntrip-source.json` + `ntrip-full-regression.log`, **2397 passed,1 skipped in254.09s**, exit0/warning0. NVENC 미지원 skip만 존재. 실행 중554개 Python source 동일 확인. 이 full PASS는 이후 M7 maintenance/M15 timers 변경까지 포함하지 않는다.
- M10 완료된 scoped boundary: `gnss_runtime/{credentials,gga,state,ntrip}.py`. Explicit CredentialInputs/account tuple, secret repr 제외, priority/keyring/env 동일. GgaSentenceOwner가3개 실제 skip/throttle state 소유. WGS84 degrees, MSL metres, ellipsoid-minus-MSL geoid, NMEA UTC와 hostmonotonic age/skew 구분. Center 기존mean/quality1/geoid blank, Base MSL없으면거절 유지.
- NtripSessionRuntime은 continuous connection의 thread/worker/pending credential+GUI relay 소유. 같은worker reconnect허용, Stop/terminal/oldsession 차단, 실제finished전busy유지. GNSS state3alias/inventory 연결. Controller public untagged callbacks는 owner 미설치legacy만 수용; actual callbacks는 _apply 내부effects. `NtripStatsForwarder`는 worker child QObject로 coalesced stats만 `NtripSessionStats(token,stats)`로감싸 기존pipeline통과, UI현재세대만unwrap. raw RTCM DirectConnection, evidence/drain/schema 변경없음. Factory/wire/start예외정리, GUIcallbackerror기록. 새 tests: test_gnss_input_boundaries.py, test_ntrip_session_runtime.py. Focused162pass12.98s, 위2397full에포함.
- M7 이후 변경: `device_runtime/maintenance_jobs.py`, DiagnosticsState/panel_diagnostics/controller/AppLifecycle 연결. Shell/PARAM4개dict 실제소유, side×kind기존concurrency유지, subscriber/connection object/connection_generationcapturedguard. 4개 result/failure effects AST원본동일. 무토큰compatcallback은 owner없는legacy만수용. capturedcommandlist→tuple; worker의PARAM/Shell lock/retry/decoder코드변경없음. Stop후queuedoutcome차단/실제thread종료까지handle보존; startfailurecleanup; legacyhandle삭제/교체시실제jobinventory와stop을별도identity로보존. 테스트 test_pixhawk_maintenance_jobs.py, 기존220pass20.25s→새205pass20.11s→추가47pass8.23s. 독립감사가 worker-onlyinventory누락을지적해OR조건과3종alias삭제test로수정.
- M15 이후 변경: `camera_runtime/lifecycle_timers.py`, CameraFeatureState/panel_camera/controller/AppLifecycle 연결. 4개state+실제2종cancellable QTimer 소유. restart100ms/35sdeadline,watchdog250ms유지. explicitcompletedStop은남은watchdog취소. pane/finalization/Run효과는 좁은callback. 기존4개test의singleShotmock을newowner._schedule에도주입(assertions동일). 실제Qtparent파괴/timer취소/GUIaffinity/oldjob/deadline검증. 독립감사재진입결함6failed재현후 외부callback직후identity/generation재검사로수정; 최종 **159pass20.71s**. tests/test_camera_lifecycle_timers.py 추가.
- 현재 M7+M15 source: `maintenance-timers-source.json`. docs/ARCHITECTURE.md와README에boundary/운용변경반영. M15 문서만마지막수정, Pythonhash는그대로. **현재 실행중test process 없음**.
- agent `/root/audit_optical_solver_boundaries`는 idle. verify-check가허용한1명read-only독립감사로사용. M15마지막보고는재진입지적이며이미6회실패재현+수정159PASS. 새subagent무단생성하지않음. 필요시동일agent에좁은read-onlyfollowup만.
- 다음검토중 M2: `calibration_controller.py`의4종shared checkerboard job(randomvalidation,capture,intrinsics,extrinsics). 새runtime 아직수정안함. `checkerboard_calibration_thread/worker`는 CalibrationFeature state. WIRISFTP는별도worker지만결과를 capture_calibration_images로넘긴다. `_capture_origin`은controller설정후capturehandler에서reset되는field이며실제job에귀속필요. `_process_next_ftp_oz_sync_pair`는checkerboard thread.finished에연결해retirement뒤다음capture로넘어간다(FIFO GUI handoff보존필요). `_clear_checkerboard_calibration_worker`는identityguard+buttons복구+deleteLater. progress연결현재plainPythonclosure라GUI-affinity/identity실소유이동대상. Intrinsic/Extrinsic은이미 coreinput_snapshot을고정, left/right K,D·R,T opticalRDF/RDF와hash는그대로. numeric정책변경금지.
- M2읽은주요위치: controller1151 randomvalidation,2545 capture/2775 Qt wiring,3705 FTPnextpair,4582 intrinsics/4654 Qt wiring,4707 extrinsics/4782 Qt wiring,5117 clear; calibration_workers.py와tests/test_calibration_input_snapshot.py, test_calibration_selection_contract.py,test_workstation_calibration.py를이어서읽을것. `test_calibration_selection_contract.py:207` workerfactory injection있음.
- 유의: root+Simulation+Workstation에대량기존미커밋변경존재. 우리필수새source도untracked이며포함없이노트북복제가안됨. 스코프밖수정되돌리지않음. checkpoint의외부절대경로1건portablechecker에걸려보고서+revision참조로수정. 마지막path/repo/diffPASS는M10후이며후속M7/M15뒤다시실행필요.
- 명령: repositoryroot에서 `apps/Workstation/tools/run_isolated_tests.sh -q -ra --tb=short ...`(require_escalated+approvedprefix; loopback namespace외우회금지). 전체는~4분, runtime source고정. apps에서 `../.venv/bin/python -B -m ruff ... --no-cache`。 root에서 `python3 tools/check_portable_paths.py`, `python3 tools/check_repository_policy.py --allow-existing-debt`, `git diff --check -- apps/Workstation`, gitstatus.
- 남은큰계획: M1capture/application/observations, M2capture/solver/profile, M4boundedcanonical/context/Excel메모리, M5/M12/M13subviews, M6remainingevidence, M7USB/persistent/MAVFTP, M8camera설정/control, M10base/correction/evidence, M15integrityrecovery/finalization. M3기본20초RSSslope는before/after둘다FAIL(ledger원본수치)이며장시간boundedmemoryPASS라고말하지말것. 실장비검증별도미수행. G1/M14완료증거가있어도전체15moduleplan완료아님.

재개 최우선: M2 checkerboard 4종 job의 기존 입력/진행/완료/FTP후속/종료 계약을 확인하고 다음좁은owner를구현·대상검증한다. M7/M15/그변경의전체회귀를마지막묶음으로돌린다. 작업중60초이상진행설명비우지말것.

최신 갱신: M7+M15 전체 **2421 passed,1 skipped in255.37s**, exit0/warning0, maintenance-timers-source.json/log 기록. Python558개 동일확인. 실행중process없음. M2구현전이므로다음source변경은이전체PASS뒤로분리된다. M2독립감사추가:4종은단일배타slot, AutoStop/appclose도기존완료/실패는반영(신규/FTP후속만차단); generic_shutdown이isRunningFalse보고alias선제삭제하지않고ownerretirement대기해야함. FTPpairqueue는현재pop후admissionFalse에서pair유실됨—head를성공전까지보존해야함. thread.finished직결후속을owner의결과처리→finished→slot해제→UIcleanup→continuation뒤로옮기고closing/세대재확인. captureprogress는operation/id포함별도schema, 계산progress는기존calibration_progress표시; 합치지않음.
