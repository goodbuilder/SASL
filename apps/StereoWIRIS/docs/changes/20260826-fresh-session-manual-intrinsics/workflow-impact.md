# Workflow Impact

## Before

앱 시작만으로 최신 미완료 spool이 UI state가 되고 zoom 10 intrinsics가 자동 적용된다.

## After

앱은 session ID 없는 clean state로 시작한다. 기존 retryable spool은 삭제하지 않고 무시한다.
운영자가 `세션 기록 시작`을 누르면 그 시각의 새 ID/time anchor와 spool을 만든다. 보정값은
`보정` 화면에서 파일을 수동으로 불러오고 `계산에 적용`해야 한다.

## Failure and recovery

수동 intrinsics load 실패는 기존 오류 표시를 사용한다. 과거 spool은 디스크에 남기 때문에
별도 forensic/manual tool로 읽을 수 있지만 Legacy startup이 자동 채택하지 않는다.
