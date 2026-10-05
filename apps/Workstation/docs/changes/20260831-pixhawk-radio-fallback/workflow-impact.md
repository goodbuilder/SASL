# Workflow Impact

Current: operator assigns Ethernet NIC/IP/URL, connects a side, and receives transport-specific
Ethernet health and RTCM evidence.

Proposed: operator selects one persisted transport per side. Radio is the default; Ethernet is an
explicit alternative. Connect validates only the selected transport, verifies expected sysid, and
publishes transport-labelled health. Changing transport on a connected side performs an orderly
side-only disconnect; the operator reviews the endpoint and explicitly presses `Connect` again.
On application startup, saved Radio endpoints remain closed until the operator connects each side.
If a saved device moved, Connect opens a candidate-selection step, validates the selected radio's
remote heartbeat identity, and continues only for the named side.
If an active Radio disappears, the side disconnects and remains idle after device return until the
operator explicitly reconnects it.
For every transport, successful Connect no longer triggers parameter/calibration administration;
those operations remain available only through explicit controls.

Preserved: independent side controls, NTRIP source, RTCM parser, measurement consumers, cameras,
recording, calibration, and manual disconnect/reconnect paths.

2026-09-01 follow-up: today each discovery action leaves unrelated old rows in the shared table and
USB heartbeat evidence creates a duplicate row. After the change, pressing `Find USB` or
`Find network` first replaces the transient table with an empty source-labelled searching state,
then shows only that command's new snapshot. The operator still selects a row and explicitly assigns
and connects a side; saved assignments and the manual fallback path are unchanged.

## Proposed 2026-09-02 field-topology workflow — awaiting approval

Current manual workaround: use terminal/temporary probes to isolate air radios, issue SiK AT
commands, power-cycle devices, edit transport settings, assign P2 Ethernet, and separately prove
the resulting links. The application cannot explain partial progress or safely roll it back.

Proposed UI flow:

1. Operator selects `P1 Radio · P2 Ethernet · Drone Radio` and presses `Check`.
2. Workstation inventories saved/current endpoints, board identity/armed state, owners, P2 Ethernet
   NIC/carrier/route, and both ground-radio local profiles without writing.
3. UI pauses and instructs the operator to isolate Drone and remove P2's air radio. It advances
   only after D30 sees only sysid3 and DU sees no sysid2.
4. After explicit confirmation, Workstation writes Drone remote NETID26, reads it back, saves it,
   pauses for a full power reapply, and verifies sysid3 appears only on DU.
5. Workstation validates P2 `10.0.0.3:14551`. If provisioning is required, it accepts only a unique,
   disarmed USB sysid2 board, writes the approved P2 Ethernet profile, verifies readback, and pauses
   for reboot/carrier/heartbeat evidence.
6. Only after both device postconditions pass does `Save topology` persist P1 Radio, P2 Ethernet,
   and Drone Radio assignments. Existing Connect remains explicit and per board.
7. Any interruption retains factual step evidence and offers Retry or the inverse guided rollback;
   no automatic link handoff occurs.

Training impact is one bounded guided sequence instead of terminal commands. Manual per-board
transport editing remains the advanced fallback outside an active setup run.

## 2026-09-03 — Guided topology workflow withdrawn

The proposed guided topology flow is withdrawn. The normal operator flow is now: select the
configured P1/P2 transport, use `Find network` and `Assign` only when the host NIC/route changes,
then press `Connect` and inspect Heartbeat/ATTITUDE evidence. P2 board-side provisioning remains
an explicit per-board operation when needed; no topology journal, Resume, or Guided rollback step
is involved in routine connection.
