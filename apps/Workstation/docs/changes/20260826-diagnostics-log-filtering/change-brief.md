# Change Brief

## Operator outcome

운영자는 Diagnostics의 `Log`에서 모든 로그를 발생 순서대로 보면서 Source, Device,
Level, 검색어로 즉시 좁힐 수 있어야 한다. 카메라/Pixhawk/GNSS가 동시에 이상해져도
`All` 타임라인에서 연쇄 원인을 확인하고, 필요할 때 특정 장비만 격리한다.

## Motivation

현재 `Log`는 세 개의 기존 텍스트 위젯을 고정 순서로 이어 붙이므로 발생 시간순이 아니다.
문자열 분류에서 인식되지 않은 메시지는 WIRIS 로그로 들어가며, `[GNSS][NTRIP]`도 그
경로에 놓일 수 있다. 장비별 장애 조사와 교차 장비 상관관계를 모두 어렵게 만든다.

## Included scope

- bounded in-memory structured diagnostic event contract
- existing `_log` path의 timestamp/sequence/source/device/level/message 구조화
- `All` 기본 타임라인과 Source/Device/Level/search 필터
- 표시 일시정지, 새 로그 수, 재개, `Follow latest`
- 기존 메시지 호환 분류와 핵심 GNSS/NTRIP·카메라·Pixhawk 장비 태그 보강
- Log 테이블의 선택/복사/스크롤/포커스 안정성
- 기존 backend text fallback과 FTP 전용 탭 호환

## Excluded scope and non-goals

- 장비 프로토콜, 연결, 측정, 좌표/프레임 계산 변경
- 디스크 로그·세션 audit JSONL 스키마나 보존 정책 변경
- 로그 삭제, 파일 내보내기, 원격 수집, 설정 영구 저장
- 모든 기존 `_log` 호출부의 일괄 재작성
- 다른 활성 변경 패킷의 정리 또는 sibling StereoWIRIS 로그 체계 개편

## Acceptance summary

`All`은 실제 수집 순서를 유지하고, 각 필터 조합은 동일 이벤트 집합의 투영만 바꾼다.
모호한 장비는 추측하지 않고 `Shared/Unassigned`로 표시한다. 로그가 숨겨져 있거나 표시가
일시정지돼도 수집은 계속되며, bounded window를 넘긴 누락은 명시된다. Diagnostics를
열거나 필터링해도 현재 Workspace, Inspector 선택, dock 크기, 키보드 포커스가 불필요하게
바뀌지 않는다.
