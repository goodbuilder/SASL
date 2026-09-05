# Impact Analysis

Mode: backend-coupled device integration.

- UI/settings: per-side transport selection and transport-specific endpoint fields.
- Controller: branch Ethernet network preflight from Radio serial/identity preflight.
- Subscriber: preserve one connection owner per side; expose transport kind in immutable health.
- Stream policy: preserve Ethernet profile; add bounded low-bandwidth Radio profile.
- RTCM: reuse the existing queue/send boundary with per-transport counters and age.
- Persistence: versioned one-time migration selects Radio for existing and new configurations while
  preserving all Ethernet endpoints/assignments for rollback and operator selection.
- Resource impact: two independent serial readers; no shared radio worker and no automatic failover.
- Command concurrency: independent side workers may run concurrently, same-side administration is
  serialized, and USB flashing has one global owner.
- Overload policy: per-side Radio evidence can reduce optional stream demand while preserving RTCM
  and measurement-critical telemetry; degradation is explicit rather than silent.
- Cross-packet impact: global removal of automatic Pixhawk parameter/calibration refresh changes the
  Ethernet workflow documented by `20260826-eth-mavlink-stability` and requires coordinated tests and
  approval-summary disclosure before implementation.
- Bulk-transfer impact: Radio FTP/log/file workflows add long-lived commands, progress/cancel state,
  queue/resource contention, partial artifacts, hash verification, and restart recovery. Radio-only
  firmware installation would additionally require an unproven PX4/bootloader capability and is not
  silently included.
- Rollback: restore Ethernet as selected transport; no PX4/device data migration.
- Discovery follow-up: `standalone_gui.py` owns snapshot replacement and one-row USB projection;
  `device_runtime_controller.py` starts network replacement; `evidence.py` carries identity fields;
  `workspaces.py` renders source-neutral identity/role evidence; focused UI/network tests lock the
  handoffs. No schema, migration, device-owner, worker-count, or CPU/GPU policy changes.
- Failure isolation: camera, GNSS, Telemetry, persisted Pixhawk assignments, and live connections
  remain independent from clearing or failing the transient Pixhawk Discovery snapshot.

## 2026-09-02 guided-topology amendment impact

- UI owner: `PixhawkWorkspace` adds one fixed-height group and five stable rows beneath Board
  transport. Existing transport rows, candidate table, connection indicators, focus, selection,
  scrolling, and manual Connect remain authoritative.
- Command/controller owner: new topology Check/Start/Retry/Cancel/Rollback command handlers delegate
  to one controller-owned operation generation. An active generation locks only affected topology
  mutation controls and endpoints.
- Pure contract/persistence: a new feature-local core module validates topology/stages, parses
  independent evidence, and atomically writes one versioned operation record beneath
  `WORKSTATION_DATA_ROOT`. Existing settings schema is reused; no automatic settings migration.
- Device boundary: one bounded QThread worker at a time performs passive MAVLink/SiK checks or one
  approved persistent action. Serial/USB/network I/O never blocks the Qt owner. Exact old/new AT
  values, ACK/readback, sysid/UID, endpoint and time remain evidence.
- SiK ownership: a remote NETID write is allowed only when D30 has one expected Drone sysid3 air
  peer and DU has no P2 sysid2. Ground-radio local NETIDs are not changed by the forward preset.
- PX4 ownership: P2 Ethernet provisioning accepts only a unique, disarmed direct-USB sysid2/UID and
  requires P2 air radio absence. It reuses the approved P2 profile `10.0.0.3:14551` and existing
  parameter/SD-card contracts; broadcast SERIAL_CONTROL across a shared RF network is forbidden.
- Network ownership: reuse existing P2 network-role transaction and assignment. No default route,
  new NIC role, or Drone Ethernet role is added.
- Final settings ownership: commit occurs only after Drone-on-DU and P2-on-Ethernet postconditions;
  failed persistence restores the prior settings object and leaves device-side PARTIAL evidence.
- Lifecycle: startup reads the operation record without opening devices. Resume/rollback requires
  a new Check; shutdown requests worker cancellation and preserves the last completed evidence row.
- Resource impact: one short-lived serial/network worker and bounded JSON evidence; no new process,
  GPU use, inference/media load, unbounded queue, or steady CPU budget.
- Compatibility: existing settings and manual per-board flows remain valid. Old installations have
  no operation record and render IDLE. Existing Radio/Ethernet connection behavior is unchanged.
- Rollback: guided inverse device steps plus restoration of the pre-operation Workstation settings.
  If identity/isolation is not provable, rollback remains PARTIAL and gives a physical recovery
  instruction rather than issuing a speculative write.
- Planned source map: `workstation_core/pixhawk_topology_setup.py`, `runtime_paths.py`,
  `device_runtime_workers.py`, `device_runtime_controller.py`, `controller_delegation.py`,
  `standalone_gui.py`, `standalone_backend.py`, `main_window.py`, `workspaces.py`, lifecycle cleanup,
  focused tests, affected product docs, and the confirmed workspace `README.md`.
- Forbidden expansion: dependency upgrades, framework/layout redesign, generalized device manager,
  radio firmware, automatic failover, or changes to unrelated dirty-worktree files.

## 2026-09-03 — Withdrawal impact

- Removed the topology-only `PixhawkTopologyController`, topology worker/core contract, operation
  journal path, delegate methods, fixed UI group, and topology-specific lifecycle/lock coupling.
- Preserved the existing DeviceRuntime, Drone Telemetry, per-board transport, NIC assignment,
  Ethernet MAVLink, Radio, USB, and explicit provisioning owners. Removing the topology owner means
  those existing controls are no longer blocked by a setup state.
- No coordinate, telemetry, measurement, recording, camera, GNSS, settings-schema, or device-side
  configuration change is introduced. The current host/Pixhawk state is not altered by this code
  removal; a restart is required for the running GUI to load it.
