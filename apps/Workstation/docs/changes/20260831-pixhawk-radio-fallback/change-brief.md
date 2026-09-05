# Change Brief

## Operator outcome

Every P1/P2 board defaults to an independently owned Telemetry Radio MAVLink transport. Ethernet
remains selectable per side. One side uses exactly one active transport at a time, and P1/P2 do not
share a ground radio or serial endpoint.

## Motivation

Field evidence identified multiple board-local Ethernet failure modes. TELEM UART remains an
independent MAVLink path and is easier to isolate by side. The current raw MAVLink URL fields can
describe serial devices, but Ethernet-only preflight and persistent `MAV_2_*` setup prevent a safe
operator workflow.

## Included direction

- Per-side `Telemetry Radio`/`Ethernet` selector, persisted with Radio as the new default.
- One-time migration selects Radio for existing installations while preserving every Ethernet
  endpoint and assignment for reversible operator selection.
- Independent serial device, baud, expected sysid, liveness, RTCM, and error evidence per side.
- Transport-specific preflight and stream policy.
- Existing transport-neutral MAVLink parsing and RTCM fragmentation reused.
- Automatic post-connect parameter/calibration refresh is removed from every Pixhawk transport;
  administrative work becomes operator-command-only.
- No automatic failover unless separately approved.
- Manual administration and bulk MAVLink FTP/log/file transfer remain accessible over Radio; actual
  firmware installation remains a separate USB bootloader operation after Radio staging.
- `Find USB` and `Find network` each replace the transient Discovery snapshot at command start, so
  the result surface contains only the newly requested source while persisted assignments remain
  unchanged.
- USB discovery renders one row per physical serial endpoint and names observed heartbeat identity
  (`P1=sysid 1`, `P2=sysid 2`, `Drone=sysid 3`) separately from saved assignment.

## Non-goals pending interview

- Shared/multipoint radio topology.
- Firmware, PX4 parameter, or radio-firmware mutation.
- Using two transports concurrently for one side.
- Video transfer over the telemetry radio.
- Custom PX4 onboard updater or Radio-capable bootloader implementation.
- Automatic assignment, connection, or settings mutation from Discovery results.

## 2026-09-02 approved and implemented design amendment

### Operator outcome

From `Devices > Pixhawk`, the operator can provision, verify, persist, resume, and roll back the
field topology `P1 Radio · P2 Ethernet · Drone Radio` without editing JSON or using a terminal or
third-party radio configurator.

The application guides but does not pretend to perform physical cable removal, Ethernet insertion,
or full power cycles. Each physical step advances only from observed carrier, expected-sysid, and
cross-endpoint isolation evidence.

### Included

- Fixed five-step `Field topology setup` group below Board transport.
- Read-only Check of identity, armed/owner/session state, ground-radio profiles, P2 NIC/route/carrier,
  and current transport/settings.
- Explicit, identity/isolation-gated Drone remote SiK NETID25→26 write, readback, save, full-power
  reapply wait, and DU/sysid3 verification.
- P2 `10.0.0.3:14551` Ethernet verification and, only when needed, explicit provisioning through a
  unique disarmed direct-USB sysid2 boundary with ACK/readback/reboot evidence.
- Delayed Workstation settings commit to P1 Radio/D30, P2 Ethernet, and Drone Radio/DU; existing
  per-board Connect remains explicit.
- Versioned operation evidence, restart Check, Resume, Cancel-before-write, PARTIAL-after-write,
  Retry, and inverse Guided rollback.
- Existing product styling, fixed geometry, existing manual per-board controls, product docs, and
  the confirmed workspace `README.md`.

### Excluded

- Automatic failover or automatic Connect.
- Software-controlled physical power/cable switching.
- Shared/multipoint radio operation or radio-firmware flashing.
- Arbitrary radio parameter editor or arbitrary topology designer.
- Changing P1 device configuration, P2 firmware, cameras, GNSS algorithms, Measurement, recording,
  calibration, or other unrelated workflows.
