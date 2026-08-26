# Backend Contract

- startup은 retryable spool discovery/load를 호출하지 않는다.
- 기존 spool bytes, manifest, `.spool` directory를 변경하지 않는다.
- 새 session은 기록 시작 시에만 ID/time anchor/store bindings/video paths를 생성한다.
- 생성된 Left/Right raw-video path의 공통 `video/` parent는 camera worker 시작 전에 존재해야 한다.
- startup은 intrinsics JSON을 active calibration state로 자동 채택하거나
  `apply_intrinsics_to_distance()`를 호출하지 않는다. UI의 saved-file inspection은 허용한다.
- 수동 load/apply는 zoom-specific 우선순위와 metadata 검증을 그대로 사용한다.
- `auto_load_intrinsics` settings field는 기존 JSON 호환을 위해 읽기/저장 가능하지만 Legacy
  startup 동작을 유발하지 않는 deprecated compatibility field로 남긴다.
