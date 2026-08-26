# Approval Summary

## Proposed operator behavior

앱을 새로 열면 과거 미완료 세션이 UI에 복구되지 않고 빈 기록 상태로 시작합니다. 기존
`.spool`은 안전하게 디스크에 남지만 앱은 읽거나 폐기하지 않습니다. `세션 기록 시작`을
누르는 순간에만 새 session ID, 정확한 start time anchor, 새 spool/video paths를 만듭니다.

Intrinsics도 시작 시 active calculation state로 자동 불러오거나 적용하지 않습니다. 보정 탭은
저장 파일의 존재/내용을 표시할 수 있지만 `active_intrinsics`는 비어 있습니다. 기존 JSON과
보정 화면의 수동 불러오기/적용 기능은 그대로 남습니다.

## What does not change

- 기존 spool/intrinsics 파일 삭제 없음
- session/recording/calibration schema 변경 없음
- camera/Pixhawk/GUI layout 변경 없음
- preview artifact decoder packet 변경 없음

## Risks and operating rule

운영자는 실제 측정 전에 현재 zoom에 맞는 intrinsics를 수동으로 불러오고 적용해야 합니다.
기존 미완료 spool이 필요하면 Legacy 자동 복구가 아니라 별도의 명시적 복구 절차가 필요합니다.

## Exact scope awaiting approval

`legacy_gui.py`에서 startup spool recovery 호출과 intrinsics auto-load timer만 제거하고,
`tests/test_stereo.py` 집중 tests 및 승인된 StereoWIRIS 문서를 갱신합니다. 기존 recovery/load
utility, 파일, settings schema는 보존합니다.
