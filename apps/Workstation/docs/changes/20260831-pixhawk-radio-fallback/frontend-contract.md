# Frontend Contract

- Each side exposes `Telemetry Radio` and `Ethernet`; Radio is the default for new/default settings.
- Radio shows serial device, baud, expected sysid, link state, heartbeat/ATTITUDE age, and RTCM age.
- Ethernet preserves NIC/IP/route controls and Ethernet-specific states.
- Exactly one transport is active per side; the inactive transport cannot claim LIVE or receive RTCM.
- Errors name the side, transport, endpoint, identity result, and next operator action.
- Changing transport while connected disconnects only that side, clears its active status, and
  requires an explicit `Connect` on the selected transport. No automatic reconnect occurs.
- Startup displays saved Radio endpoints and discovery/availability evidence without opening them.
  P1 and P2 remain visibly disconnected until their own `Connect` action is pressed.
- A missing saved Radio endpoint produces a side-labelled candidate list. Candidate rows show stable
  path/identity hints and availability; selection is not accepted until expected-sysid heartbeat
  verification succeeds. Wrong, ambiguous, duplicate, and busy candidates show a concrete reason.
- Candidate ordering and USB port are never presented as P1/P2 identity. The UI displays observed
  sysid as the decisive evidence and warns on duplicate/same-side radios.
- A constrained Radio link shows `RADIO DEGRADED · RTCM PRIORITY`; detail includes current RTCM age,
  stale/drop counters, and which optional diagnostics were reduced. Critical telemetry remains
  visibly distinct from optional diagnostics.
- Recovery shows a stable, non-resizing step/qualification detail. Optional diagnostics return in
  bounded stages; the state returns to degraded immediately if critical evidence regresses.
- `RADIO QUALIFYING 00/10s` reserves fixed geometry. `RADIO LIVE` means transport-qualified only;
  RTK and Measurement readiness remain independently labelled. Diagnostic recovery detail uses a
  30-second stable timer and 15-second inter-tier observation without moving controls.
- Radio removal shows `RADIO DISCONNECTED`, disables RTCM for that side, and preserves a visible
  `Connect` action. A returned device is shown as available but never auto-connected.
- Connected status never implies an automatic parameter refresh. Administrative status appears only
  after an explicit operator action and names its command, side, transport, progress, and outcome.
- Radio management controls may expose parameter, shell, reboot, persistent Ethernet configuration,
  FTP, large-file, log, and firmware-image staging actions. Active RTCM produces no warning or
  confirmation and does not block an explicit action. Progress/cancel/partial/unknown states remain
  visible.
- The existing Log surface receives a bounded informational event when such an action starts during
  active RTCM. It does not steal focus, open a dialog, move controls, or alter command availability.
- Firmware staging and firmware installation are separate labels. The UI must not call a staged file
  an installed firmware or imply that a TELEM link survives PX4 bootloader transition.
- A staged image shows remote path, bytes, SHA-256, target board identity, and the next action
  `Connect verified USB and Flash`; USB flash retains its own approval/progress/result workflow.
- Interrupted transfers show verified partial bytes/offset and operator actions `Resume`/`Restart`.
  Resume is disabled with a concrete reason when remote partial integrity cannot be proven.
- Starting `Find USB` or `Find network` immediately clears prior candidate rows, selection, stale
  candidate feedback, and the old Discovery result; the stable assignment cards remain visible.
- While discovery runs, the status names the active source. Success, empty result, and failure show
  only the current command's outcome and never mix rows from a previous source.
- The candidate table uses source-neutral headings. Network rows identify shared NIC assignment;
  Radio rows identify one physical device, heartbeat evidence, and observed/saved role separately.
- Per-board transport controls read `P1 Telemetry Radio` and `P2 Telemetry Radio`; the underlying
  persisted enum remains `radio`. `Drone / sysid 3` is not a `Devices > Pixhawk` row: it drives the
  same `"drone"` MAVLink side that `Devices > Telemetry` already owns, so the Pixhawk board
  transport table shows only a pointer to that workspace.

## Approved 2026-09-02 USB transport

- Each board transport offers `Radio`, `Ethernet`, and `USB`. The persisted enum is
  `radio | ethernet | usb` at `pixhawk_transport_schema_version` 2.
- Radio and USB are both serial: each opens one exclusive device path, takes the shared
  `Serial device`/`Baud` row, and fails Connect preflight when that path is absent or already owned
  by another role. Only the wording separates them (`USB DEVICE MISSING` vs `RADIO DEVICE MISSING`).
- The bandwidth policy is what actually differs. Radio keeps its tiered `RadioStreamPolicy`
  throttle; USB carries the stock Ethernet-class stream set, because a direct cable has no
  air-link budget to protect.
- USB reuses the per-board bootstrap endpoint (`{side}_pixhawk_bootstrap_url` / `_baud`) that
  recovery and firmware work already own, so one device path per board stays current. Radio keeps
  its own saved endpoint; switching transports never overwrites the other one.
- `Devices > Telemetry` owns the Drone transport selector, including `USB`. Selecting a serial
  transport there enables `Serial device` and disables `MAVLink URL`; Ethernet does the reverse.
- USB is not an Ethernet substitute for persistent setup: manual `MAV_2` configuration and eth0
  status stay Ethernet-only.

## Approved 2026-09-02 guided topology setup surface

- `Devices > Pixhawk` adds one `Field topology setup` group immediately below the existing
  `Board transport · explicit Connect only` group; per-board controls remain visible and unchanged.
- The group has no preset selector. It implements the single supported field topology
  `P1 Radio · P2 Ethernet · Drone Radio` and exposes `Check`, `Start guided setup`, `Cancel`, and
  `Guided rollback` actions directly.
- A fixed five-row step table shows Device safety, Drone Radio/NETID26, P2 Ethernet, Save topology,
  and Connect/qualify. Columns are `Step`, `Required state`, `Observed evidence`, and `Action`.
- Only the current safe action is enabled. Persistent SiK/PX4 writes show exact board identity,
  endpoint, old value, and new value and require one explicit operator confirmation.
- Physical steps remain operator-owned. The table shows `WAITING FOR OPERATOR` until actual
  carrier/heartbeat/isolation evidence proves cable connection, P2 air-radio removal, or full Drone
  air-radio power reapply; there is no trust-only completion checkbox.
- Final Apply remains disabled if DU observes P2 sysid2, D30 observes a non-Drone peer during the
  Drone write boundary, P2 Ethernet lacks carrier/route/expected sysid2, any board is armed, an
  affected endpoint is owned, or Measurement/recording is active.
- Partial device persistence is never labelled rolled back or complete. The table retains the last
  verified step and exposes bounded Retry or Guided rollback with the concrete missing postcondition.
- Completing setup persists P1 Radio/D30, P2 Ethernet/`10.0.0.3:14551`, and Drone Radio/DU only;
  it does not auto-connect. The operator then uses the existing per-board `Connect` actions.
- P2 Ethernet failure remains side-only and fail-closed. It does not acquire DU or interrupt P1 or
  Drone, and the setup surface offers Check/Retry rather than automatic transport failover.

## 2026-09-03 — Current frontend contract after withdrawal

The fixed `Field topology setup` group and its Check/Start/Cancel/Rollback controls are no longer
part of `Devices > Pixhawk`. Board transport rows, Discovery, candidate assignment, P2 ETH MAV,
and explicit Connect/Disconnect remain the user-visible controls. These controls are not locked by
a topology operation state.
