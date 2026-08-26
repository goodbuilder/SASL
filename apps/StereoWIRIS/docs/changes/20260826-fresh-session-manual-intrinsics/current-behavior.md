# Current Behavior

## Session startup

`_initial_status()`가 `_recover_retryable_recording_session()`을 호출한다. 이 함수는
`recordings/sessions/*/.spool` 중 manifest가 `finalized/discarded`가 아닌 최신 디렉터리를
선택해 session ID/time anchor/rows/video paths/original metadata를 GUI state로 복구하고
save-retry 상태로 둔다.

현재 관측된 복구 대상은 `20260825_172757_395861883`, `pair=0`이다. 자동 기록·자동 finalization은
하지 않지만 새 앱이 과거 세션 state로 시작한다.

## Intrinsics startup

constructor가 `_auto_load_intrinsics()`를 zero-delay timer로 실행한다. 현재 설정은
`auto_load_intrinsics=true`, UI zoom 기본은 10이며
`calibration_captures/intrinsics_zoom_10.json`을 우선 불러와 거리 계산에 적용한다.

## Fresh recording

`_start_session_recording()`은 기록 버튼 시점에 `session_id_now()`와 time anchor를 만들고
그때 `.spool` 및 video paths를 생성한다. 이 경로는 startup recovery를 제거해도 유지된다.
