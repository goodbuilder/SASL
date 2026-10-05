# Decisions

## 2026-08-31 — Active Drone included with fixed identity

- "Every board" includes P1, P2, and the active Drone Telemetry Pixhawk. The separate receive-only
  Target Drone artifact path is unchanged.
- The authoritative mapping is P1 sysid 1 / NETID 25, P2 sysid 2 / NETID 26, and Drone sysid 3 /
  NETID 27. Drone requires a third independent ground/air radio pair.
- The legacy Drone endpoint currently pointing at the P2 ground radio is preserved as settings
  evidence but is never promoted automatically to a verified Drone Radio assignment.

## 2026-08-31 — Audit corrections before implementation

- Current calibration and parameter refresh controls are already manual; implementation preserves
  them and proves that no connection callback invokes them. The confirmed automatic persistent
  behavior to remove is the heartbeat-driven `MAV_2_*` stream-setup path.
- PX4 MAVFTP exposes remote size and CRC32, not remote SHA-256. Local SHA-256 remains provenance;
  resumable integrity uses explicit remote-prefix readback and final remote CRC32.
- Workstation has no existing in-app USB flasher. The approved scope adds a pinned verified PX4
  uploader worker rather than describing the historical external uploader as an existing feature.

## 2026-08-31 — Independent radio topology

- P1 and P2 use independent radio pairs and independent ground serial endpoints.
- A shared/multipoint ground radio is excluded because it weakens side isolation, bandwidth
  attribution, identity ownership, and fault diagnosis.

## 2026-08-31 — Radio-default selectable transport

- Telemetry Radio is the default transport for every board.
- Ethernet remains selectable per side.
- One side has exactly one active transport; simultaneous Ethernet+Radio is excluded.
- Automatic failover is not inferred from this decision and requires a separate observable workflow
  decision.

## 2026-08-31 — Explicit reconnect after transport change

- Changing the selected transport while a side is connected performs an orderly side-only
  disconnect and disables RTCM eligibility for the old connection.
- Workstation does not automatically connect the new transport. The operator must press `Connect`
  after reviewing the newly selected endpoint.
- This prevents hidden link handoff, duplicate RTCM delivery, and stale-generation callbacks from
  making the new transport appear live.

## 2026-08-31 — Manual connect at application startup

- Workstation never opens saved P1/P2 Radio endpoints merely because the application started.
- Startup may discover and display candidates, saved endpoints, and availability, but each side
  remains disconnected until the operator presses its `Connect` action.
- Identity qualification and RTCM eligibility begin only after that explicit connect intent.

## 2026-08-31 — Operator-selected Radio rediscovery

- If the saved Radio device is missing, `Connect` presents discovered serial candidates instead of
  requiring raw Settings entry.
- The operator selects one candidate for the named side. Workstation temporarily probes heartbeat
  identity and applies the endpoint only when the expected sysid matches.
- Wrong-sysid, duplicate, ambiguous, or busy devices fail closed and remain disconnected.
- A verified selection continues the original explicit Connect intent; it is not startup auto-connect.

## 2026-08-31 — RTCM and measurement-critical Radio priority

- Radio transport prioritizes RTCM plus measurement-critical heartbeat, ATTITUDE, GNSS, and bounded
  system-health telemetry over optional diagnostic streams.
- When actual bandwidth/age/drop evidence indicates saturation, Workstation suppresses or reduces
  optional IMU, magnetometer, debug, and other nonessential stream requests/retries for that side.
- The UI exposes `RADIO DEGRADED · RTCM PRIORITY` with RTCM age, queue/drop, and link evidence.
- No PX4 persistent parameter or radio-firmware setting is changed by this policy.

## 2026-08-31 — Hysteretic automatic diagnostic recovery

- After a degraded Radio link satisfies a stable recovery interval, optional diagnostics restore
  automatically in bounded steps rather than all at once.
- Each step requires continued healthy RTCM age, critical telemetry freshness, and no new overload
  evidence. A regression immediately returns that side to RTCM-priority degradation.
- Recovery is process-local, side-specific, and does not reconnect the transport or persist device
  parameters.

## 2026-08-31 — Radio qualification and recovery timing

- Expected-sysid verification starts RTCM immediately while the side remains `RADIO QUALIFYING`.
- Transport becomes `RADIO LIVE` after 10 continuous seconds of fresh critical telemetry, RTCM age
  within the configured three-second limit, and no queue/drop/link failure.
- Optional diagnostic restoration starts after 30 continuous healthy seconds, restores one tier,
  then observes for 15 seconds before the next tier.
- Any critical regression immediately cancels restoration and returns the side to RTCM priority.
- Transport LIVE does not override independent GNSS/RTK/measurement readiness gates.

## 2026-08-31 — No automatic reconnect after Radio disappearance

- If an active Radio serial device disappears, that side becomes `RADIO DISCONNECTED`, RTCM
  eligibility is revoked, pending side work is invalidated, and the owner is closed.
- Device return does not reconnect automatically, even for the same stable by-id path in the same
  application session. The operator must press `Connect` and identity qualification starts again.
- Discovery may show the returned device as available but cannot acquire it before Connect.

## 2026-08-31 — Existing installations migrate to Radio selection

- The settings schema adds a versioned one-time migration that selects Telemetry Radio for both
  existing P1/P2 roles as well as new/default configurations.
- Existing Ethernet endpoints, peer IPs, NIC assignments, baud/settings, and recovery evidence are
  preserved unchanged so the operator can select Ethernet again.
- Migration never opens a Radio device. Missing Radio configuration remains disconnected and shows
  candidate guidance after explicit Connect.

## 2026-08-31 — USB-port-independent Radio identity

- Operators may connect the independent P1/P2 ground radios to any USB ports and in any order.
- Device node, by-id, and by-path values are discovery/presentation hints only. The authoritative
  side assignment is the observed remote heartbeat sysid (`P1=1`, `P2=2`).
- Duplicate ownership, same-sysid candidates, identity changes, and a candidate already owned by the
  opposite side fail closed and require operator correction.

## 2026-08-31 — No automatic parameter reads on any Pixhawk transport

- No Ethernet or Radio connection automatically starts full/group parameter reads, calibration
  parameter refresh, shell work, persistent configuration, or FTP after heartbeat identity.
- Connect performs only endpoint ownership, expected-sysid qualification, GCS heartbeat, and the
  transport's critical process-local stream requests.
- Parameter reads/writes and all administrative operations require an explicit operator command.
- This decision intentionally changes the existing Ethernet workflow as well as the new Radio flow;
  the implementation must coordinate with, not silently contradict, the active ETH stability packet.

## 2026-08-31 — Manual Radio administration and bulk transfer are operator-accessible

- Explicit operator commands may use Radio for parameter read/write, bounded shell, reboot,
  persistent Ethernet configuration, MAVLink FTP, large file upload/download, and log transfer.
- No administrative or bulk operation starts automatically. Active RTCM does not produce a warning,
  confirmation, or prohibition; the operator owns when to start the command.
- Identity, side ownership, disarm/lifecycle guards where applicable, progress, cancel, timeout,
  ACK/readback, partial-result, and unknown-outcome evidence remain mandatory.
- Firmware image staging over MAVLink FTP is allowed. Actual stock-PX4 flashing over the same Radio
  link is not yet an accepted capability because reboot leaves the running MAVLink/FTP service and
  the standard documented firmware uploader uses USB/bootloader connectivity.

## 2026-08-31 — No RTCM warning for explicit Radio commands

- Explicit Radio administration, shell, reboot, FTP, log, and large-file actions do not show an
  RTCM-active warning or require an RTCM-specific confirmation.
- If RTCM is active, command start is recorded as an informational application-log event with side,
  transport, command category, timestamp, and current RTCM age; logging does not block or delay it.
- Command progress, cancellation, timeout, failure, partial completion, hash/readback, and unknown
  outcome remain factual execution state and are not suppressed by this decision.

## 2026-08-31 — Firmware stages over Radio and flashes over USB

- Radio may transfer a firmware image to board storage with exact size, SHA-256, remote-path, and
  readback evidence.
- Staging completion is labelled `STAGED`, never installed/flashed/running.
- Actual erase/program/verify uses the existing verified USB bootloader workflow after the operator
  connects USB and initiates that separate command.
- Fully Radio-only flashing, custom onboard updater, and Radio-capable bootloader work are excluded.

## 2026-08-31 — Operator-selected verified transfer resume

- Interrupted Radio bulk transfers never resume automatically.
- Workstation verifies remote partial size, offset, and available prefix/hash evidence before
  offering `Resume`; `Restart` is always available after explicit operator intent.
- Missing, inconsistent, wrong-generation, or unverifiable partial state disables Resume and
  requires restart from byte zero. Completion still requires full size and SHA-256 readback.

## 2026-08-31 — Per-side Radio concurrency and serialized USB flash

- P1 and P2 Radio administration/bulk transfers may run concurrently because each has an independent
  radio pair, serial owner, queue, liveness generation, and RTCM route.
- Each side permits at most one administrative/bulk command at a time; same-side duplicates are
  rejected or queued only where the specific workflow explicitly defines it.
- USB firmware erase/program/verify is globally serialized to one verified board at a time.

## 2026-09-01 — Discovery is a replace-only, source-scoped snapshot

- The operator explicitly approved clearing the transient Discovery result whenever `Find USB` or
  `Find network` starts. Completion displays only that command's new result.
- Clearing Discovery never clears persisted Radio endpoints, Ethernet assignments, connection
  state, or settings and never connects a device.
- A physical serial endpoint appears once. Heartbeat evidence enriches that row instead of creating
  a second `MAVLink` row.
- The shared candidate pool remains intentional because USB order is not identity. The UI must make
  source, observed sysid role, and saved assignment distinct so independent P1/P2 radios cannot look
  like a single shared Radio.

## 2026-09-02 — Drone Ethernet field alternative requires an approval amendment

- Two available ground radios cannot safely carry three boards under the installed peer-to-peer
  SiK firmware; both tested two-air-node shared layouts failed telemetry continuity before RTCM.
- The active Drone hardware and lower-level Workstation transport support Ethernet, but its current
  PX4 profile duplicates P2 and the operator NIC/persistent-setup workflow is incomplete for Drone.
- Treat Drone Ethernet as a backend-coupled workflow amendment to this packet, not as permission to
  bypass NIC ownership, write the live board, or edit source from the feasibility request alone.
- Source and persistent Drone Ethernet configuration remain paused until the operator chooses the
  visible failure/fallback behavior and approves the amended summary. The existing approved Radio
  implementation remains intact.

## 2026-09-02 — P2 Ethernet plus Drone Radio proposed instead

- The operator interrupted the Drone-Ethernet fallback question and proposed P2 on Ethernet with
  Drone on Telemetry Radio.
- This is the lower-change field topology: P1 remains D30/NETID25, P2 uses its already configured
  `10.0.0.3:14551` Ethernet profile, and Drone moves to the dedicated DU/NETID26 pair as sysid3.
- P2's air radio must be physically off/disconnected while Drone owns DU/NETID26; otherwise P2 and
  Drone recreate the failed shared peer-to-peer RF topology.
- No automatic transport failover is proposed. If P2 Ethernet fails, P2 remains visibly
  disconnected while P1 Radio and Drone Radio continue; recovery is Ethernet reconnect or an
  explicit later topology change.
- The proposal uses existing P2 Ethernet and per-board transport controls and should not require the
  missing Drone NIC/persistent-setup GUI extension. It remains an operator outcome awaiting explicit
  confirmation before hardware or saved-settings changes.

## 2026-09-02 — Operator requests full topology provisioning through the UI

- The requested operator outcome is to configure, validate, apply, persist, and roll back the P1
  Radio + P2 Ethernet + Drone Radio topology without editing JSON or using a terminal/third-party
  radio tool.
- The UI may command and verify PX4/SiK settings, but cannot physically remove P2's air radio,
  connect Ethernet, or power-cycle a vehicle. Those remain explicit operator steps whose completion
  is proven by observed carrier, expected sysid, and cross-endpoint isolation—not by a checkbox.
- A safe UI flow must refuse remote SiK writes until exactly the intended air peer is isolated,
  show old/new NETID and readback/save evidence, require a full air-radio power reapply, and verify
  Drone sysid3 on DU/NETID26 before assigning or connecting it.
- It must also validate P2 `10.0.0.3:14551`, host NIC/address/route/carrier, keep P2's air radio
  absent, persist P2 Ethernet + Drone Radio only after both postconditions pass, and provide a
  guided rollback. Automatic failover remains excluded.
- The remaining open interaction decision is guided fail-closed steps versus a one-click automatic
  attempt. Guided steps are recommended because physical isolation/power evidence cannot be made
  atomic by the application.

## 2026-09-02 — Fixed inline five-step topology surface approved

- The operator approved one fixed `Field topology setup` group directly below the existing Board
  transport controls in `Devices > Pixhawk`.
- The approved interaction is guided and fail-closed, not a one-click automatic topology mutation.
  It retains existing per-board controls and explicit Connect actions.
- The surface uses five stable rows for Device safety, Drone Radio, P2 Ethernet, Save topology, and
  Connect/qualify, with Check, Start guided setup, Cancel, and Guided rollback actions.
- Physical cable/removal/power steps stay operator-owned and advance only from observed device/link
  evidence. This screen-layout approval does not yet authorize source implementation; restart and
  partial-persistence behavior plus the final amendment summary remain open.

## 2026-09-02 — Interrupted topology setup resumes from verified evidence

- The operator approved restart detection for a partially completed topology setup.
- Device writes that were independently saved/read back remain factual after an app or power stop;
  restart must not discard them or claim the prior topology is intact.
- Workstation reloads the operation record, performs a new read-only hardware Check, and offers only
  `Resume` from the next unproven step or `Guided rollback`. It never resumes a pending write or
  power-cycle assumption automatically.
- Missing, changed, ambiguous, busy, or armed devices keep the setup in `PARTIAL` with the concrete
  blocking evidence. Existing unrelated P1/Drone/P2 links remain independently usable where their
  current ownership permits.

## 2026-09-02 — Workspace README target confirmed

- The operator confirmed `apps/Workstation/README.md` (the workspace `./README.md`) as the README
  updated with the accepted topology workflow, interfaces, operational rules, limitations, and
  dated change log when source implementation proceeds.

## 2026-09-02 — Guided field-topology amendment approved for implementation

- The operator approved the complete 2026-09-02 approval-summary amendment after reviewing the
  fixed inline surface, fail-closed physical evidence gates, partial restart Resume/Guided rollback,
  preserved manual controls, exclusions, verification scope, and README target.
- Implementation may now change only the documented P1 Radio + P2 Ethernet + Drone Radio guided
  vertical slice. Real device writes remain explicit UI actions and are not authorized merely by
  implementing or launching the new surface.

## 2026-09-02 — Three ground radios replace the P2 Ethernet leg on real hardware

- A third ground radio (`DU0E4O0F`) became available, so the operator chose to keep P2 on its
  existing radio and run all three boards on Telemetry Radio: P1 `D30JNMVH`/NETID25 sysid1,
  P2 `DU0E49VU`/NETID26 sysid2, Drone `DU0E4O0F`/NETID27 sysid3.
- This supersedes the P1 Radio + P2 Ethernet + Drone Radio preset for the current field rig. The
  approved preset stays valid and its implementation is untouched; it was not exercised because the
  saved NIC `enx0023a40a2e39` was absent and `enx8cb0e9e8f8ee` had no carrier when the rig was set
  up, and because a third pair removes the reason to give up P2's radio.
- The earlier "two air nodes cannot share the RF topology" finding is unchanged. It applied to two
  air radios on one NETID; three pairs on three distinct NETIDs is a different layout.

## 2026-09-02 — Root cause of the P1 and Drone radio silence was 2+2 mis-pairing on NETID 25

- All four radios of the P1 and Drone pairs sat on NETID 25, so they paired ground-to-ground
  (`D30JNMVH` with `DU0E4O0F`) and air-to-air (P1 with Drone). Both ground endpoints reported strong
  symmetric RSSI with zero MAVLink, which reads as a healthy link and is not one.
- Ground-to-ground pairing was proven by `RTI7` remote counters matching the other ground radio's
  local counters. Air-to-air pairing was proven by the Drone's USB endpoint carrying sysid 1
  telemetry plus a `sys 51 comp 68` RADIO_STATUS alongside its own sysid 3.
- Carrier, RSSI, and packet counters are not link identity. Only expected sysid on the endpoint is,
  which is what the guided flow's isolation gate already requires.
- Reaching one air radio requires every other air radio on that NETID to be powered down first.
  P1's air radio was removed from power to expose the Drone's air radio for the remote write.

## 2026-09-02 — Drone pair moved to NETID 27 from the terminal, not the guided UI

- The operator chose a one-off terminal transaction because the guided surface is fixed to the
  NETID26 preset and to `DU0E49VU` as the Drone radio, neither of which matches this rig.
- The transaction kept the approved gates: refuse while `D30JNMVH` is attached, require exactly
  sysid 3 and no armed board, require local and remote NETID 25, write and read back the remote
  before the local, save both, then require a full air-radio power reapply before qualification.
- Evidence is not in `pixhawk_topology_operation.json` because the guided flow did not run. If the
  UI must own this topology later, the preset needs a three-radio variant.
- Qualified end state: `verify_links.py` PASS with sysid 1/2/3 live, HEARTBEAT 11-12, ATTITUDE
  48-52, GPS_RAW_INT 14-18 per 12-second sample, no armed board.
- Three pairs now share 915-928 MHz with `DUTY_CYCLE=100` and `LBT_RSSI=0`. Distinct NETIDs stop
  mis-pairing but not band contention; per-pair `MIN_FREQ`/`MAX_FREQ`/`NUM_CHANNELS` partitioning is
  the next step if sustained throughput degrades.

## 2026-09-03 — Withdraw guided Field topology setup

- The operator confirmed that the guided topology surface is unnecessary for ordinary connection
  and requested its removal.
- Remove the topology-only UI/state/journal/controller/worker and its cross-feature locks. Keep the
  existing per-board transport, NIC discovery/assignment, P2 ETH MAV, Connect/Disconnect, and
  explicit board-side provisioning paths unchanged.
- Existing topology packet sections remain historical evidence of the withdrawn amendment; current
  product docs describe only the normal explicit connection flow.

## 2026-09-04 — RTCM stop/reconnect does not establish a Radio IMU path

- The operator confirmed that stopping RTCM transmission and reconnecting P1/P2 did not restore
  IMU samples. RTCM contention is therefore not accepted as the primary cause of this observation.
- Read-only runtime evidence shows the saved P1/P2 transport selectors remain `ethernet`; recent
  application logs show Ethernet `ROUTE MISMATCH` preflight failures and no Radio stream request.
- Before changing Radio bandwidth policy, the next acceptance step is to select the populated P1/P2
  Telemetry Radio endpoints, save the selection, restart if required by the running process, and
  explicitly Connect each side. No automatic failover is added.
- Once the Radio path is active, IMU remains lower priority than RTCM. Any proposed IMU rate/timing
  change must preserve RTCM/critical telemetry and be validated with real Radio samples.
