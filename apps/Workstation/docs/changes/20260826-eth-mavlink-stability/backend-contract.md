# Backend Contract

## Commands and validation

- Construct every operational connection with GCS source system 255 and component 190.
- Validate the expected remote system ID per side before marking it connected or sending commands.
- Send GCS heartbeat at 1 Hz from a bounded timer owned by the subscriber.
- Permit stream/parameter commands only when current liveness is not stalled and identity matches.
- A recovery command must name one side and, if USB-based, resolve a currently verified sysid mapping.

## State, ownership, and concurrency

`DualPixhawkMavlinkSubscriber` owns per-side `last_any_rx`, `last_heartbeat_rx`,
`last_attitude_rx`, identity, connection generation, and qualification start. It publishes immutable
snapshots. RTCM workers read a thread-safe eligibility query/snapshot; they do not mutate health.
Connection generation prevents an old worker callback from reviving a replaced link.

## Worker resource budgets, scheduling, and backpressure

- Receive budget stays bounded by existing 256 messages/20 ms tick.
- Heartbeat: at most one send/side/second; missed ticks are not replayed.
- Liveness evaluation: O(number of sides), no blocking I/O.
- RTCM: existing bounded queue; ineligible side rejected before send and counted as `blocked_stall`,
  not transport `fail`.
- No retry loop faster than 1 Hz and no unbounded log/event emission.

## External boundaries and workers

- pymavlink UDP creation is the identity boundary.
- PX4 is treated as an unreliable external process; an open host socket is not proof of health.
- If network reachability is added, it runs in an existing/background bounded worker and is only
  supporting evidence. Stall classification must not freeze the UI waiting for ICMP.
- USB reboot is optional and must use the existing serial discovery/identity boundary.

## Results, errors, timeout, retry, and cancellation

- ATTITUDE stale threshold remains 2 s for display/re-request behavior.
- All-message silence threshold is 10 s after at least one valid remote message.
- Stream requests are one initial standard profile plus existing bounded ATTITUDE recovery only;
  unsupported private stream retry is disabled for Stock.
- On stall: cancel stream/param retries, block RTCM, emit one transition event, keep 1 Hz heartbeat
  probe, and wait for approved recovery. Do not loop host reconnect.
- Any fresh valid remote packet moves `STALLED` to `RECOVERING`, never directly to READY.

## Persistence, restart, and shutdown

Health and qualification timers are process-local and reset on restart. Event/evidence logs persist
using existing application logging/evidence paths. Shutdown stops heartbeat timer and RTCM worker
before closing sockets. No PX4 config or parameter write is performed by this change.
