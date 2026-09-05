# Approval Summary

> Current field amendment: the operator approved the 2026-09-02 preset below. For that preset,
> P2 uses Ethernet and Drone uses DU/NETID26; the earlier NETID27/third-radio plan remains historical
> context and is not the selected field topology.

Approved by the operator on 2026-08-31 through the reviewed implementation plan.

- P1, P2, and the active Drone Telemetry Pixhawk default to independent Telemetry Radio links;
  Ethernet remains an explicit per-board alternative and only one transport may own a board.
- Identity is authoritative (`P1=sysid 1/NETID 25`, `P2=sysid 2/NETID 26`,
  `Drone=sysid 3/NETID 27`). The receive-only Target Drone artifact path is unchanged.
- Only an explicit board `Connect` action may open a link. Startup, transport changes, device
  return, network reassignment, USB bootstrap, swap, and recovery never reconnect automatically.
- Connect performs identity, GCS heartbeat, process-local stream requests, and RTCM routing only.
  Parameter reads, persistent Ethernet setup, shell, reboot, FTP, logs, and firmware work remain
  explicit commands.
- Radio qualification is 10 continuous healthy seconds. Optional diagnostics restore after 30
  healthy seconds, then at 15-second tier intervals; regressions return immediately to RTCM
  priority without persistent PX4 or radio changes.
- Explicit management and bulk commands remain available during RTCM without a warning or block;
  one informational Log event records their start.
- MAVFTP completion uses remote size/CRC32 plus local size/SHA-256/CRC32. Resume requires exact
  remote-prefix readback. Radio firmware staging is not installation; actual erase/program/verify
  uses a newly integrated, verified USB bootloader worker and never auto-reconnects telemetry.
- Three independent board operations may run concurrently, one per board. USB flash is globally
  serialized. Hardware closure requires a third Drone pair and a simultaneous three-pair soak.

## Approved follow-up — 2026-09-01

The operator directly requested that `Find USB` and `Find network` reset Discovery and show only
new results, together with the diagnosed Radio presentation defect. This approval adds only
transient snapshot replacement, one physical serial row per device, and explicit source/identity/
role presentation. It preserves saved assignments, settings, manual Assign/Connect, active device
owners, and every non-Pixhawk workflow. No device is connected or configured by discovery.

## Approved amendment — 2026-09-02 guided field topology

### Outcome

Add one fixed `Field topology setup` group under `Devices > Pixhawk > Board transport`. It guides
the operator through the supported field preset:

- P1 on D30 Telemetry Radio / NETID25 / sysid1.
- P2 on Ethernet `10.0.0.3:14551` / sysid2, with its air radio physically off.
- Drone on DU Telemetry Radio / NETID26 / sysid3.

The operator can Check, provision, verify, persist, Resume after restart, Retry, Cancel before any
write, and perform Guided rollback without terminal commands, JSON editing, or a third-party radio
tool.

### What the operator will see and do

- The existing per-board transport and Connect rows remain unchanged.
- A stable five-row table shows Device safety, Drone Radio, P2 Ethernet, Save topology, and
  Connect/qualify with exact evidence and one current action.
- The UI pauses for P2 radio removal, Ethernet cable insertion, and full Drone air-radio power
  reapply. It advances only from actual carrier, heartbeat, sysid and readback evidence.
- Persistent SiK/PX4 writes show board/endpoint and old/new values and require explicit confirmation.
- Completion saves the three transport assignments but does not auto-connect. Existing per-board
  Connect remains the only link-opening action.

### Failure, interruption, and rollback

- Wrong/duplicate/ambiguous identity, armed board, busy endpoint, active Measurement/recording,
  missing carrier/route, or cross-sysid delivery fails closed before a write.
- Cancel before a write returns to IDLE. After a saved device change, the operation remains PARTIAL;
  it never claims cancellation restored the system.
- Restart loads intent/evidence without opening devices, performs a new Check, and offers Resume at
  the first unproven step or Guided rollback. No write or reconnect resumes automatically.
- P2 Ethernet failure after completion disconnects only P2. P1 and Drone continue, and no automatic
  Radio fallback occurs because DU belongs to Drone.

### Preserved behavior and excluded scope

- Manual per-board settings, Find USB/network, Assign P1/P2, Connect/Disconnect, RTCM, camera,
  GNSS, Measurement, recording, calibration, FTP, firmware and diagnostics remain available.
- No shared/multipoint Radio, automatic failover, automatic Connect, physical power/cable control,
  radio-firmware flashing, arbitrary radio editor, dependency upgrade, or UI redesign is included.
- Existing settings schema is reused; an old installation without a topology operation record starts
  IDLE. One bounded worker and one atomic evidence record add no steady GPU/process load.

### Verification and residual field condition

- Automated evidence will cover state transitions, raw SiK command/response fixtures, exact
  readback/save, direct-USB P2 identity/disarm guards, NIC/carrier/route/heartbeat, settings commit,
  partial restart/Resume/rollback, UI geometry/focus, shutdown, and preserved adjacent workflows.
- The full Workstation suite and `./run.sh --check` will be rerun. `README.md` and affected numbered
  product docs will be updated.
- Real hardware must still pass forward setup, restart/rollback, P1 Radio + P2 Ethernet + Drone
  Radio identity isolation, RTCM/RTK evidence, and the approved qualification/soak. Automated fakes
  will not be reported as hardware proof.

Approval of this section authorizes only this guided-topology vertical slice and its documentation/
tests. It does not authorize unrelated cleanup or device writes outside explicit UI actions.

## 2026-09-03 — Guided topology withdrawal

The operator withdrew the one-time `Field topology setup` surface because normal board connection
does not require it. The dedicated UI group, commands, operation journal, topology controller/
worker, and topology-specific locks/tests are removed.

Preserved behavior is the existing per-board transport selection, NIC discovery/assignment,
P2 ETH MAV, Connect/Disconnect, explicit PX4 Ethernet provisioning, Radio, USB, RTCM, camera,
GNSS, Measurement, recording, calibration, FTP, firmware, and diagnostics workflows. Existing
Pixhawk settings are not migrated or deleted. A normal run uses the configured transport and an
explicit Connect/Verify action.
