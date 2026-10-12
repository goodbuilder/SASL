# Workstation RCA 수정 결과 — 2026-10-10

## 요청과 실행 범위

발견된 문제를 근거별로 정리하고, 수정 계획·전후 동작·기대효과를 작성한 뒤 수정과 실제 검증을 수행했다.
[사전 계획](implementation-plan.md), [요구사항](change-brief.md), [순차 검증 원장](verification.md)에 연결된다.

구조 개선의 수용 기준은 측정 Run 상태의 독립 소유권이다. 기존 대형 UI 전체를 재작성했다는 의미는 아니다.

## 수정 전후와 기대효과 확인

| 요구 | 수정 전 | 수정 후 | 기대효과를 확인한 증거 | 판정 |
|---|---|---|---|---|
| R1 스냅샷 | frozen 객체 내부 dict/list를 외부에서 변경하면 입력·해시 변경 | 중첩 데이터 불변 소유, 외부 반환값은 독립 JSON | 새 회귀 5 FAIL→6 PASS; 독립 payload/hash, 실제 spawn, 워크북 재현을 포함한 82 PASS | 충족 |
| R2 CI | 삭제된 test_workstation.py 참조, 수집 실패 | Workstation 전체 디렉터리와 공유 기하 시험 실행 | 실제 수집 성공, 전체 실행 2169 PASS/2 SKIP | 충족 |
| R3 이식성 | 과거 기록 7개에서 20 pattern 위반 | 7개 원본을 exact SHA-256으로 분류; 새 파일·변경 내용은 계속 검사 | 원본 bytes/CRLF 변조/인접 파일 시험 포함 최종 16 PASS; gate PASS | 충족 |
| R4 상태 소유권 | Run 판단을 UI 공유 필드에서 수행; camera가 다른 기능의 Run 필드 변경 | Qt-free 서비스가 상태·세대·소스·메타데이터 소유; UI는 호환 투영 | headless import, 투영 변조, 중복 Start, 늦은 종료, 저장 실패, 실제 패널 등 123 PASS/1 SKIP | 충족 |
| R5 계산 복구 | 시작 이후 장기 실행의 상태·선택적 복구 부족 | 상태/경과/대기를 표시하고 식별자가 일치하는 worker만 명시적으로 교체 | SIGTERM 무시 child의 kill/replacement, 동료/대기 작업 유지, GUI timer, 이전 Run 디스크 증거: 8 PASS | 충족 |
| R6 문서 | MAG 차단/advisory, 8/9/10 method, compute timeout 설명 충돌 | 현재 runtime 계약을 명시하고 README/workflow/schema/제품 문서를 정합화 | 등록 method ID 및 오래된 timeout/MAG 주장 검증, 생산 코드 대조 | 충족 |
| R7 검증·보고 | 앞선 RCA의 선별 관찰만 존재 | 새 RED/GREEN, 전체 회귀, 오류 경로, 화면·정책 검증 기록 | 아래 실행 결과와 verification.md, 소프트웨어 파일 해시 inventory | 충족 |

R4에서는 Start가 optional registry 호출 전에 STARTING을 확보한다. registry 콜백에서 다시 Start해도 새 Run을 중복 생성하지 않는다. Stop은 저장소 작업 전에 IDLE을 확보하고 이전 완료가 새 Run을 종료하지 못한다.

R5의 일반 Stop은 이전 계산을 계속 보존한다. 자동 계산 timeout을 추가하지 않았다. 복구 버튼은 현재 계산을 FAILED/operator_restart로 기록하고 대기 계산을 계속한다. 요청 접수, 실제 process replacement, 다음 계산 성공은 서로 다른 상태다. 이미 수신된 완료, 오래된 선택, 확인되지 않은 종료, 교체 실행 실패도 검증했다.

## 변경 파일

- 데이터 소유권: `workstation_core/immutable_json.py`, `method_evaluation.py`, `measurement_attempt.py`.
- 상태 소유권: `workstation_core/measurement_run.py`, `measurement_run_controller.py`, `camera_stream_controller.py`.
- 복구 흐름: `method_evaluation_workers.py`, `measurement_view_controller.py`, `standalone_backend.py`, `method_worker_pane.py`, `workspaces.py`.
- 품질 gate: repository `.github/workflows/quality.yml`, `tools/check_portable_paths.py`, 관련 회귀 시험.
- README: [apps/Workstation/README.md](../../../README.md). 현재 [runtime 계약](../../MEASUREMENT_RUNTIME_CONTRACT.md), workflow, recording schema, 제품 상태/명령 문서도 갱신했다.

## 검증 결과와 한계

전체 CI 대상 실행은 시스템 Python에서 **2169 passed, 2 skipped, 1 warning, 228.43초**였다.
전체 실행 이후 경로 검사의 newline 정규화가 exact-bytes 약속을 약화시키는 점을 추가 보강했고, 해당 영향 범위 최종 시험 **16개가 통과**했다. 앱 runtime 코드는 전체 실행 이후 변경하지 않았다.

- SKIP: NVENC fixture가 해당 encoder의 최소 영상 크기를 만족하지 못함.
- SKIP: 선택적 `pyproj`가 설치되지 않음.
- 경고: 시스템 NumPy 1.26.4가 시스템 SciPy의 선언 범위(<1.25) 밖에 있음. 전역 패키지는 변경하지 않았다.
- `tools/check_portable_paths.py`, `tools/check_repository_policy.py --allow-existing-debt`, `git diff --check` 통과.
- 정식 `run.sh --check`는 시작 전부터 없었던 repository `.venv/bin/python` 때문에 실패했다. 지원 실행환경 설치 및 실장비 현장 검증은 이번 소프트웨어 교정의 검증 범위와 구분한다. 실행 준비는 README의 기존 Current Run Flow를 따른다.
- 외부 카메라/Pixhawk 설정, firmware, 네트워크를 변경하지 않았다. 실제 측정 정밀도나 장시간 현장 운전 인증을 주장하지 않는다.

## 화면 확인

실제 Workstation Qt shell과 controlled FakeBackend로 1366×900 및 1366×768에서 Data Status를 렌더링했다. 기존 스크롤로 계산 상태 영역에 접근하며, 선택/버튼/경과 표시가 겹치지 않는 것을 확인했다. 긴 method 이름은 콤보 폭에 맞추고 전체 이름은 tooltip으로 제공한다.

- [Data Status — 1366×900](evidence/data-status-running.png)
- [Data Status — 1366×768](evidence/data-status-compact.png)
- [실행 중 계산](evidence/method-recovery-running.png), [교체 완료](evidence/method-recovery-complete.png)

## 이식성과 작업 트리

새 소스 3개, 새 시험 5개, runtime 계약 문서 및 이 변경 packet은 repository 안에 있으며 ignore되지 않았다. 아직 commit하지 않아 untracked 상태인 추가 파일들이 있다. 변경한 기존 파일도 미커밋이다. 새 clone에 반영하려면 해당 소스·시험·문서를 함께 commit해야 한다. 테스트 임시 로그는 재현에 필요한 유일한 상태가 아니며 명령과 결과는 이 packet에 남겼다.

동시에 진행 중인 Simulation 변경은 이 작업의 결과에 포함하지 않았고 수정/정리하지 않았다. 커밋·push는 수행하지 않았다.

Coordinate contract checked: coordinate_contract/canonical_transform/target_coordinates/recording schema; NED, body FRD, optical RDF, baseline-center FRD, WGS84; 공유 좌표·기하·워크북 재현 시험. 축 순서·단위·부호·원점·수식·저장 schema 변화 없음.
