# Impact Analysis

## Data safety

기존 spool과 intrinsics 파일은 보존한다. startup에서 읽지 않을 뿐 삭제·rename·discard하지 않는다.

## Runtime

startup I/O와 state mutation이 줄어든다. process/thread/queue/GPU/network resource는 변하지 않는다.

## Compatibility

session/JSON/XLSX/MP4 schema는 그대로다. `auto_load_intrinsics` 필드는 호환용으로 남긴다.

## Risks

- 운영자가 intrinsics 수동 적용을 잊으면 calibrated 3D measurement가 준비되지 않는다.
- 과거 retryable spool은 자동 노출되지 않아 수동 복구 경로가 필요할 수 있다.
- 앱 시작 시 빈 디렉터리를 만들지 않으므로 “세션”은 기록 버튼 시점에 실제 생성된다.
