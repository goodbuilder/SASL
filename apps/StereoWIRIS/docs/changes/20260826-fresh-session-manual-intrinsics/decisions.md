# Decisions

## 2026-08-26 — Safe proposed semantics

- 기존 retryable spool은 보존하지만 Legacy startup에서 자동 복구하지 않는다.
- 빈 session directory를 앱 실행마다 만들지 않는다. 기록 시작 시 fresh session을 생성한다.
- intrinsics 파일과 수동 workflow는 보존하고 startup auto load/apply만 제거한다.
- preview artifact packet의 decoder 변경과 섞지 않는다.

## 2026-08-26 — Approved

- 사용자가 14:03 KST에 이 packet과 preview clean-intra packet을 모두 승인했다.

## 2026-08-26 — Implemented and verified

- constructor의 zero-delay intrinsics auto-load timer와 `_initial_status()`의 retryable spool
  recovery 호출을 제거했다. 명시적 recovery/manual load utility는 유지했다.
- startup은 active intrinsics와 session ID가 빈 상태였고, 실행만으로 session directory를
  만들지 않았다. 기존 spool/intrinsics hash도 변하지 않았다.
- 기록 버튼은 기존 spool을 건드리지 않고 새 ID와 새 spool을 만들었다. 정확한 startup/session/
  explicit-recovery 회귀 5개가 통과했다.
- 실장비 recording-start에서 metadata path만 준비되고 `video/` parent가 없던 결함을 확인했다.
  fresh session 생성 시 계산된 raw-video parent를 먼저 만들도록 보강했고 focused regression이
  실제 directory 존재를 검증한다.
