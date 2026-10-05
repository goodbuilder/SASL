# Backend Contract

- Persist a per-side transport enum and separate Ethernet and Radio endpoint configuration.
- Radio preflight validates device existence, exclusive ownership, baud, and expected heartbeat
  sysid; `/dev/serial/by-id` is preferred over unstable tty numbering.
- Ethernet preflight remains unchanged and is never run for Radio.
- Ethernet `MAV_2_*` setup and network assignment are forbidden on Radio transport.
- Radio uses a reduced standard-message request profile and MAVLink2 RTCM delivery.
- Existing per-side identity, liveness, generation, send lock, bounded RTCM queue, and fail-close
  contracts remain authoritative.
- No automatic failover or simultaneous dual transport is included without amended approval.
- A connected-side transport change retires the current generation, disables RTCM eligibility,
  drains/invalidates pending side work, closes the old owner, and leaves the new endpoint idle until
  an explicit connect command.
- Loading settings and discovering serial candidates must not acquire a Radio serial device. Only
  an explicit side connect command may create the connection owner and start identity qualification.
- Missing-device rediscovery is bounded and read-only. It probes one operator-selected candidate,
  verifies the expected remote sysid, rejects duplicate ownership across sides, persists the stable
  endpoint after success, and then continues the original connect generation.
- Serial topology is USB-port independent. Device paths are hints; expected remote sysid is the
  identity oracle. A connection generation is invalidated if identity changes after connect.
- Radio uses an explicit priority policy: RTCM and measurement-critical heartbeat/ATTITUDE/GNSS
  remain eligible before optional diagnostic requests. Saturation decisions use actual age, queue,
  drop, and link evidence; optional retries are bounded and may be suppressed per side.
- Overload adaptation is process-local and must not write PX4 parameters or radio configuration.
- Optional demand restores through a hysteretic state machine: stable evidence interval, one bounded
  restore step, continued observation, then the next step. Any critical regression cancels pending
  restoration and reapplies the reduced profile for that side.
- Radio transport qualification requires 10 continuous healthy seconds. Optional recovery requires
  30 healthy seconds before the first tier and 15 healthy seconds between later tiers. RTCM can flow
  after identity verification; these timers do not weaken GNSS/RTK/Measurement gates.
- Serial removal retires the connection generation, revokes RTCM eligibility, invalidates pending
  side work, and closes the owner. Reappearance is discovery evidence only; a new explicit Connect
  command and expected-sysid verification are mandatory.
- A versioned idempotent settings migration selects Radio for both roles once, preserves legacy
  Ethernet fields byte-for-byte/semantically, performs no I/O, and does not rerun after the schema
  version is recorded.
- No connection callback or first-heartbeat callback may launch parameter/group reads, calibration
  refresh, shell, persistent configuration, or FTP on either transport. Only explicit command
  handlers may acquire the exclusive command boundary for administrative work.
- Explicit Radio administrative/bulk commands use bounded queues, progress, cancellation, timeout,
  side/generation identity, and operation-specific ACK/readback or artifact hash evidence. Active
  RTCM neither rejects nor warns on an explicit command.
- Active-RTCM command start appends one bounded informational event containing side, transport,
  command category, observed time, and current RTCM age. It is audit evidence only and emits no
  modal/approval side effect.
- MAVLink FTP may stage firmware bytes with size/hash/readback evidence. Actual flash requires a
  separately verified USB bootloader connection and the existing uploader. Stage and flash are
  distinct commands, states, owners, evidence, and failure recovery paths.
- Bulk transfer state records side, connection generation, local hash/size, remote path, verified
  offset/prefix evidence, and terminal outcome. Resume requires an explicit command and matching
  remote partial evidence; otherwise only restart is valid.
- Radio command ownership is one active administrative/bulk operation per side with independent P1/P2
  workers and queues. USB flash uses one global owner and rejects concurrent second-board flash.
- Pixhawk discovery owns one transient candidate snapshot. An accepted `Find USB` or `Find network`
  intent clears every transient candidate source before its worker starts; a rejected duplicate
  intent leaves the active snapshot untouched.
- USB candidate aliases resolving to the same device remain deduplicated, and each physical serial
  endpoint produces exactly one result row enriched with baud, sysid, component id, and observation
  time when available.
- Discovery reset/result/failure performs no settings write, endpoint acquisition, connection,
  network mutation, or persisted assignment change.

## Approved guided-topology amendment

- A single topology-setup owner serializes Check, persistent SiK/PX4 writes, power-cycle
  verification, settings commit, Resume, Cancel, and Guided rollback. Ordinary per-board Connect
  cannot acquire an endpoint currently reserved by this owner.
- The owner persists a versioned operation record after each independently verified transition. It
  records requested topology, stage, board UID/sysid, stable endpoints, old/new values, command
  acknowledgement/readback, observed times, and terminal/partial outcome; it never stores an
  unverified operator assertion as device state.
- Restart loads the record read-only and re-probes hardware before offering Resume or Guided
  rollback. It never repeats a write, reconnects a transport, or advances a physical step merely
  because the previous process intended it.
- Resume begins at the first postcondition not re-proven in the current process. Guided rollback
  applies inverse persistent changes only after the same identity, disarm, ownership, and isolation
  gates used by forward setup.
- Final Workstation settings commit is delayed until Drone sysid3 is isolated on DU/NETID26 and P2
  expected-sysid2 Ethernet carrier/route/heartbeat evidence is current. A failed commit preserves
  the previous settings and leaves the device-side partial state visible for retry/rollback.
- Cancel before the first persistent write returns to IDLE. Cancel after any persistent write ends
  as PARTIAL and offers Guided rollback; it never labels the operation cancelled-and-restored.

## 2026-09-03 — Current backend contract after withdrawal

The topology operation owner, staged topology state, operation-record persistence, and topology
worker are removed. Normal per-board Connect, Discovery, transport selection, P2 Ethernet MAVLink,
explicit PX4 shell/provisioning, and Drone Telemetry operations retain their existing owners and
validation. No startup action, automatic Connect, failover, or device write is added.
