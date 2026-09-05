# Impact Analysis

## Preserved invariants

하나의 MAVLink subscriber/socket owner, expected sysid 1/2, 60초 qualification, stalled-side RTCM
fail-close, recording mapping lock, coordinate contracts를 보존한다.

## Affected layers and components

network discovery/route logic, device worker/controller, settings, backend facade/evidence, Pixhawk
workspace와 관련 tests/docs.

The amendment additionally affects Camera/Pixhawk role coordination, NetworkManager privilege and
transaction handling, app-owned host-network persistence/readback, confirmation/failure UI and
field rollback procedures. PX4 and media/device workers remain outside the mutation owner.
The coordinator invokes existing public Camera/Pixhawk stop/start contracts but never takes
ownership of their streams, sockets, workers or device-specific state.

## Feature dependency, ownership, and coupling changes

The existing per-feature discovery remains isolated. A narrow host-network role coordinator becomes
the sole owner of NetworkManager mutation so Camera and Pixhawk widgets do not mutate one another's
private state. Same-NIC composition must be represented as combined app-owned roles rather than
two competing profile writers.
The coordinator owns only role intent and app-owned network deltas; Camera stream and MAVLink socket
owners remain independent and must continue if an unrelated role application fails.

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

Host-network mutation can interrupt active camera/MAVLink traffic, strand an old role, or remove an
unrelated route if ownership is too broad. The amendment therefore requires exact app-owned deltas,
operator preview, post-apply readback and an approved cross-NIC rollback policy before implementation.
Same-NIC composition adds shared-entry lifetime risk: removing P1 must not remove `.10/24` while P2
still needs it, and removing Camera must not touch any Pixhawk or unrelated operator entry.
Cross-NIC movement adds a two-device transaction without a native NetworkManager transaction. The
approved mitigation is verify-target-before-cleanup, assignment commit last, exact snapshots and
compensating target rollback; any failed compensation remains visible and blocks a success claim.
Regression scope must cover one combined NIC, two NICs with Camera/Pixhawk split in either order,
P1/P2 separation across additional NICs, hot removal/replacement, zero usable NICs and an unaffected
NIC continuing while a source/target role change is rejected or fails.
It also covers active Camera only, active P1/P2 only, combined active owners, partial reconnect,
rollback/reconnect, close during every phase and mapping-locked Measurement/recording rejection.
