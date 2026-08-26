# Change Brief

## Operator outcome

운영자는 Diagnostics의 `Log` 안에서 각 장비의 로그가 서로 분리된 고정 화면으로 보여야
한다. 통합 로그에 필터를 적용해야만 장비를 구분하는 방식은 요구사항이 아니다. 정확한
장비 grouping과 `전체` 보조 화면 포함 여부는 다음 인터뷰 답변으로 확정한다.

## Motivation

현재 `Log`는 세 개의 기존 텍스트 위젯을 고정 순서로 이어 붙이므로 발생 시간순이 아니다.
문자열 분류에서 인식되지 않은 메시지는 WIRIS 로그로 들어가며, `[GNSS][NTRIP]`도 그
경로에 놓일 수 있다. 장비별 장애 조사와 교차 장비 상관관계를 모두 어렵게 만든다.

## Included scope

- bounded in-memory structured diagnostic event contract
- existing `_log` path의 timestamp/sequence/source/device/level/message 구조화
- 장비별 고정 Log view(하위 탭 또는 동등하게 즉시 전환 가능한 고정 구획)
- 정확한 grouping 확정 후 필요한 최소 검색/수준 보조 기능
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

각 장비 화면은 해당 장비의 이벤트만 기본 표시하며 다른 장비 로그가 섞이지 않는다.
공용·다중 장비 이벤트의 소유 화면과 전체 보조 화면 여부는 grouping 결정에 따른다.
Diagnostics를 열거나 장비 화면을 전환해도 현재 Workspace, Inspector 선택, dock 크기,
키보드 포커스가 불필요하게 바뀌지 않는다.
