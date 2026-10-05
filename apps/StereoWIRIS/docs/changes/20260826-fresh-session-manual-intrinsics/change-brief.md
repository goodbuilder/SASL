# Change Brief

## Objective

StereoWIRIS Legacy 시작 시 과거 retryable `.spool`을 자동 복구하지 않고 빈 기록 상태로
시작한다. Intrinsics도 자동 로드·적용하지 않으며 운영자가 명시적으로 불러오고 적용한다.

## In scope

- Legacy startup의 spool recovery 호출 제거
- Legacy startup의 intrinsics auto-load timer 제거
- 기록 시작 시 기존 `session_id_now()` 기반 fresh session 생성 계약 보존
- 집중 tests와 StereoWIRIS 운영 문서

## Out of scope

- 기존 spool 삭제·discard·내용 변경
- session schema, save/finalize writer 변경
- intrinsics JSON 삭제·형식 변경
- calibration 계산·수동 load/apply UI 변경
- Workstation/common runtime 변경

## Success condition

재시작 후 session ID/rows/retry flag가 비어 있고 기존 spool은 그대로 남아야 한다. 기록 시작
전 디렉터리를 만들지 않고, 기록 버튼에서 새 session/spool을 만든다. Intrinsics는 시작 시
미적용이며 수동 load/apply 후에만 active가 된다.
