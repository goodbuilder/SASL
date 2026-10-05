# Integration Flow

## Startup

settings load → blank recording stores/session ID → normal connection UI. Retryable spool scan과
intrinsics load/apply branch는 호출하지 않는다.

## Recording

operator `세션 기록 시작` → fresh session ID/time anchor → bind new `.spool` rows → prepare
raw video paths → start/restart camera recording owner.

## Calibration

operator file/load action → zoom-specific JSON validation → in-memory result → explicit
`계산에 적용` → active intrinsics.
