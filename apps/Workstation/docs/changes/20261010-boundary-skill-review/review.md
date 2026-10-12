# Workstation 구조를 근거로 보강한 경계 관리 스킬 초안

## 조사 범위와 판정

2026-10-10 현재 작업 트리의 controller, feature composition, measurement/recording 경계,
구조 시험과 기존 분리 작업의 검증 기록을 읽었다. 앱 코드와 시스템 설정은 변경하지 않았다.
모든 모듈을 전수 검증하거나 앱을 실행한 결과가 아니라 선택된 경계의 정적 검토다.
작업 트리에 진행 중인 수정과 미추적 파일이 있으므로 HEAD만으로 조사 소스를 식별하지 않는다.
[source-audit.json](source-audit.json)에 주요 파일의 실제 내용 해시와 AST 집계를 저장했다.

이미 `20261010-god-module-decomposition` 작업에서 개선한 구조와 시험이 존재한다.
해당 스킬의 부재가 과거 문제의 원인이었다는 인과관계는 입증하지 않았다.
기존 문서의 테스트 통과는 과거 작업 증거이며 이번 조사에서 재실행한 결과가 아니다.

## 관찰 -> 예방 규칙

| 관찰 | 증거와 해석 | 스킬 개선 |
|---|---|---|
| 파일이 분리돼도 전체 panel 의존이 남음 | `optical_pose_controller.py:558`은 전체 panel을 보관. 정적 리터럴 참조는 optical pose 56개/쓰기 19개, calibration 160/40, device runtime 212/43, GNSS runtime 192/60. 별칭·계산된 이름은 집계하지 않아 완전한 런타임 그래프는 아님. | 설계 시 역할별 명령/스냅샷을 정의하고 수정 시 상태 접근과 소유권을 함께 이전. 파일 분리만으로 완료하지 않음. |
| 기능명 안에도 다른 변경 이유가 공존 | `device_runtime_controller.py`에 MAVFTP, USB flash, 호스트 IP 탐색, 네트워크 역할, 연결 종료, 센서 shell poll이 함께 존재. calibration에는 파일 선택·캡처·검증·프로필 게시·worker 수명 관리가 포함됨. 이것만으로 실제 장애를 확정하지 않음. | 기능 이름이나 줄 수 대신 변경 시나리오·수명·의사결정 책임을 기준으로 내부 경계 검토. |
| 유한 포트도 매우 넓을 수 있음 | CameraFeature dependency_names 184개, PresentationFeature 188개. 명시적 접근 제한은 개선이지만 독립적 변경 가능성을 자동 증명하지 않음. | 포트 개수 상한 대신 인터페이스의 목적과 관련 없는 기능 변경 파급을 검토. 새 범용 context/field registry로 갓 객체를 재생성하지 않음. |
| 빠진 optional callback이 기능을 조용히 생략했던 이력 | 분리 verification.md:53에 영상 경로가 비는 실패와 allocator 의존 누락이 기록됨. 12개 콜백을 보완했고 현재 test_panel_composition.py에 회귀가 있음. 현재 미해결 버그로 보고하지 않음. | 필수/선택 capability 구분, 필수 연결 누락은 조립/실행 전 드러내기, 실제 composition 시험. |
| 소유권 이동은 종료·스레드 계약을 바꿈 | `panel_feature_runtime.py:67`은 이름 suffix로 자원을 수집하고 PanelApplication은 서비스 inventory를 합침. `test_hidden_feature_thread_is_visible_to_shutdown_resource_check`는 vars(panel)에 없는 자원까지 검사. 실제 누락된 자원을 이번에 발견한 것은 아님. | 생성/종료/취소/join 소유자와 resource registration을 함께 설계·이전. 동적·중첩 자원과 Qt parent/thread 역할 구분. |
| 구조 분리 중 선택 정책이 바뀐 이력 | 분리 verification.md:55에 preferred/fallback/hysteresis 우회 후 복구 기록. 현재 순수 geometry resolver와 캡처 후 live input 변경 시험이 존재. | 기계적 이동과 정책 변경 분리. 설정·선택·시간 입력 고정 시점 및 결과 세대 계약 보존. |
| 호환 facade가 다시 의존 경로가 될 수 있음 | 분리 verification.md:102에 RecordingViewFeature의 facade 역방향 import 수정 기록. compatibility-exit.md는 consumer 기반 제거 조건을 이미 둠. | 새 내부 코드는 실제 owner/contract를 사용. facade 소비자와 제거 조건 관리, 금지 import를 넣은 음성 시험. |
| 분리의 유지비도 증가함 | 기존 report.md는 생성 코드·시험 등을 제외한 앱 Python이 166,402 -> 176,640줄로 증가했다고 기록. 이번 비교를 새로 측정한 수치는 아님. | 책임·변경 영향 개선과 wiring/adapter/API 비용을 함께 보고. 총 줄 수나 성능 개선을 근거 없이 주장하지 않음. |

## 긍정 사례도 보존해야 함

- `workspace_views`의 명시적 DAG, 독립 import와 금지 import 구문별 음성 fixture.
- measurement의 frozen inputs와 순수 resolver/planner, live 설정 변경 후 동일 시도 보존 시험.
- recording의 동일 ledger/lock/row-generation-lease 소유권과 제한된 operation port.
- panel의 상태별 owner, 기본 읽기 전용 포트, 이동한 QObject 콜백 thread 확인.
- 기존 검증 기록은 QA callback lifetime 문제가 제품 결함이 아니었음을 구분한다.
  테스트 fixture와 제품 양쪽의 수명·실행 환경을 확인하는 규칙이 필요하다.

## 개선된 두 초안

- [design-boundaries/SKILL.md](design-boundaries/SKILL.md): 설계 시 책임뿐 아니라 필수 capability,
  스냅샷 시점, 세대, 트랜잭션, callback thread, 종료 소유권, migration exit까지 정의.
- [maintain-boundaries/SKILL.md](maintain-boundaries/SKILL.md): 실제 변경 시 동적 호출·쓰기까지
  조사하고, 기능 누락/선택 정책/상태 동일성/종료 가시성을 보존하며 경계 검사로 재발 방지.

두 초안은 각각 독립적으로 읽을 수 있으며 서로를 필수 설치 의존성으로 요구하지 않는다.
역할별 규칙은 각 스킬에 두고, 요구사항·승인·검증 기록은 기존 GUI/verify-check 절차를 재사용한다.
프로젝트 특유 클래스 이름이나 메서드 개수를 범용 의무 규칙으로 복사하지 않았다.
개인 스킬 디렉터리에 설치하거나 create-gui/modify-gui/verify-check를 수정하지 않았다.

## 검증 범위

- 소스 정적 조사: PASS (선택한 파일의 AST·호출·소유권 경로와 역사 기록 확인).
- 두 초안 형식 검사: 별도 quick_validate 실행 결과로 확인.
- 수동 시나리오 검토: 작은 응집 수정에는 새 계층 강제 없음; 생성 MAVLink 대형 파일은
  유지보수 위험 순위에서 제외; optional callback 누락과 늦은 결과는 명시적 검증 대상으로 포함;
  기존 GUI packet이 있으면 인터뷰/승인/문서 중복 없음.
- 앱 기능 회귀 시험: NOT_RUN. 앱 코드 변경이 없으며 실제 장비·네트워크를 실행하지 않았다.
- 독립 에이전트 행동 평가 및 실제 작업 효과: NOT_RUN. 형식 검사와 수동 검토를 행동 효과의 증명으로 보지 않는다.

Coordinate contract checked: 문서/스킬 초안만 추가. NED [north,east,down], body FRD,
optical RDF [right,down,forward], stereo-center/left/right-camera 구분과 quaternion wxyz,
GPS 고도·각도 및 clock 출처 계약을 검토했으며 좌표 계산·배열·CSV·UI 라벨 변경 없음.
