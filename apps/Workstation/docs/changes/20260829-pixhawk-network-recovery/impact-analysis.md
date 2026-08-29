# Impact Analysis

## Preserved invariants

하나의 MAVLink subscriber/socket owner, expected sysid 1/2, 60초 qualification, stalled-side RTCM
fail-close, recording mapping lock, coordinate contracts를 보존한다.

## Affected layers and components

network discovery/route logic, device worker/controller, settings, backend facade/evidence, Pixhawk
workspace와 관련 tests/docs.

## Feature dependency, ownership, and coupling changes

Pixhawk feature가 read-only host network snapshot을 소유한다. Camera discovery 내부 상태를 읽거나
변경하지 않으며 동일 NIC 허용은 두 기능 간 새 mutable coupling을 만들지 않는다.

## Worker CPU/GPU resource impact

짧은 subprocess/sysfs discovery worker 하나만 추가한다. GPU/decoder/YOLO/recording queue와 CPU
allocation은 변경하지 않는다.

## Compatibility and migration

settings fields는 additive default-empty다. 구 설정은 기존 URL/kernel route 동작을 유지한다.

## Rollback and recovery

새 assignment fields와 preflight/discovery facade/UI를 제거하면 기존 static 후보 흐름으로 되돌아간다.

## Risks and assumptions

wildcard UDP socket은 inbound interface 자체를 증명하지 않는다; assignment truth는 kernel outbound
route/source와 MAVLink source identity다. shared Camera/Pixhawk NIC 장기 안정성은 검증되지 않았지만
사용자 결정에 따라 UI warning/gate를 추가하지 않는다.
