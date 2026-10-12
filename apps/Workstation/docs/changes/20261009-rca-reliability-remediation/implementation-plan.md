# Implementation and acceptance plan

1. S0: 현재 계약/변경/테스트를 재확인하고 이 문서와 검증 원장을 작성한다.
2. S1 / R1: 중첩 JSON을 불변 자료로 소유하고 모든 외부 serialization은 독립 JSON 값으로 반환한다. canonical bytes/schema/좌표 값은 그대로 유지한다. 회귀를 먼저 RED로 확인하고 수정 후 GREEN을 남긴다.
3. S2 / R2,R3: CI의 Workstation 전체 경로 및 명시적 immutable historical evidence 분류. 포괄 디렉터리 예외로 새 오류를 숨기지 않는다. 자체 gate 시험 및 실제 gate를 실행한다.
4. S3 / R4: Qt-free Run state/service로 전이/세대/소스/메타데이터 소유권 이동. panel 호환 투영은 한 방향으로 제공하며 상태 결정은 서비스에서 수행한다. adapter의 장비·저장 호출은 유지한다. 순수/컨트롤러/실제 패널 경계를 차례로 검증한다.
5. S4 / R5: worker 상태 스냅샷, 비차단 선택적 재시작 명령, 기존 Data Status 화면의 compact 제어. current job만 명시적 FAILED로 종결하고 대기 작업/동료 worker는 보존한다. 자동 compute timeout은 도입하지 않는다. actor에서만 프로세스를 종료·대체한다.
6. S5 / R6: Workstation README, workflow, 관련 제품 계약을 생산 코드에 맞춘다. 역사적 동작 설명은 history로 명시한다.
7. S6 / R7: Workstation 전체 회귀, 공유 좌표 시험, 경로/CI gate, 재현/실제 프로세스/GUI 흐름, 작업 트리 검토. 각 기준의 fresh 증거가 있어야 완료한다.

각 단계의 실패/발견은 verification.md에 append한다. 의존 단계는 이전 단계 GREEN 후 진행한다. 이미 존재하는 무관한 실패는 원인과 범위를 조사하고 통과로 표시하지 않는다.

롤백: 스키마/기록 migration 없이 코드 변경을 되돌릴 수 있다. 과거 데이터는 rewrite하지 않는다. 외부 장비 명령/firmware/config를 변경하지 않는다.
