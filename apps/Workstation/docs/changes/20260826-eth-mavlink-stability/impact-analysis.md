# Impact Analysis

## Preserved invariants

- Workstation is the sole UDP listener for 14550/14551.
- P1 is system ID 1 at 10.0.0.2:14550; P2 is system ID 2 at 10.0.0.3:14551.
- Parsed, CRC-valid complete RTCM3 frames remain the only injection unit.
- Measurement/finalization never permits an automatic device reboot.
- One failed side cannot silently borrow the other side's identity or evidence.

## Affected layers and components

- Transport construction, periodic outbound heartbeat, and liveness timestamps in
  `workstation_core/pixhawk_attitude.py`.
- Stale/stall decision and recovery command in `device_runtime_controller.py`.
- RTCM queue target validation in `device_runtime_workers.py` and routing/readiness in
  `gnss_runtime_controller.py`.
- Operator status projection in `status_model.py` and existing workspace status surfaces.
- Product docs and targeted tests.

## Feature dependency, ownership, and coupling changes

Liveness becomes a subscriber-owned snapshot consumed by status, recovery, and RTCM routing;
controllers must not infer health from `connection exists` or `heartbeat was ever seen`.
RTCM routing depends only on the immutable liveness snapshot/API, not private timestamp sets.
UI consumes projected states and does not inspect pymavlink objects.

## Worker CPU/GPU resource impact

- Two 1 Hz heartbeat sends and monotonic timestamp checks are negligible.
- Optional liveness probe must be nonblocking and bounded; no UI-thread ping or socket receive.
- Stream reduction lowers PX4/host packet rate and parser work.
- RTCM queue remains bounded; stalled targets are rejected before enqueue/send.

## Compatibility and migration

No storage/schema or PX4 parameter migration. Existing endpoint settings remain valid. Fake/test
subscribers need a compatible liveness adapter or a conservative fallback. Custom firmware users
may opt into diagnostic streams later, but Stock is the default profile.

## Rollback and recovery

Each behavior is feature-local and can be reverted independently: periodic heartbeat, stream
profile, stall projection, RTCM fail-close, and recovery command. Existing config files and PX4
params are untouched. Rollback restores old behavior but also restores the known silent-failure risk.

## Risks and assumptions

- GCS heartbeat and stream reduction mitigate traffic but cannot repair the open PX4/NuttX bug.
- A 10-minute soak catches the observed 5.5-minute failure, not every possible long-duration failure.
- `system=255/component=190` must not collide with another GCS; sole-owner premise makes this valid.
- Stopping RTCM on an apparently stalled side may delay RTK recovery if the diagnosis is false;
  fail-closed is preferred because the side has no current telemetry evidence.
- USB reboot requires verified physical port↔system-ID mapping and may fail if USB is absent.
- Persisted `MAV_2_RATE=2000000` may aggravate load, but changing it is deliberately deferred until
  controlled A/B evidence and explicit operator approval exist.
