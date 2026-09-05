---
schema: modify-gui-change-state-v1
change_id: 20260826-eth-mavlink-stability
status: verifying
mode: backend-coupled
updated: 2026-08-31
baseline_ref: c4397995d864576c966702cfd3c398b6032f8990
last_completed: "Official P1 stock reflash PASS and 10-minute outbound ETH transport PASS: 32034 consecutive packets, ATTITUDE 15Hz, GPS1 5Hz, GPS2 1Hz, zero loss; inbound ARP/GCS/RTCM still FAIL; P2 comparison means physical damage is not yet proven"
next_question: "Authorize a reversible P1 extras.txt duplicate-ifup A/B: preserve the existing backup, temporarily disable extras, cold reboot, and retest ARP/GCS before restoring if it does not help?"
read_next:
  - docs/changes/20260826-eth-mavlink-stability/PLAN.md
  - docs/changes/20260826-eth-mavlink-stability/approval-summary.md
  - docs/changes/20260826-eth-mavlink-stability/change-brief.md
  - docs/changes/20260826-eth-mavlink-stability/current-behavior.md
  - docs/changes/20260826-eth-mavlink-stability/workflow-impact.md
  - docs/changes/20260826-eth-mavlink-stability/impact-analysis.md
  - docs/changes/20260826-eth-mavlink-stability/decisions.md
  - docs/changes/20260826-eth-mavlink-stability/frontend-contract.md
  - docs/changes/20260826-eth-mavlink-stability/backend-contract.md
  - docs/changes/20260826-eth-mavlink-stability/integration-flow.md
  - docs/changes/20260826-eth-mavlink-stability/implementation-plan.md
  - docs/changes/20260826-eth-mavlink-stability/verification.md
  - docs/changes/20260826-eth-mavlink-stability/ui-quality-and-stability.md
affected_product_docs:
  - README.md
  - Pixhawk.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/07_COMMAND_MODEL.md
  - docs/ui/09_IMPLEMENTATION_PLAN.md
blocking_questions: 1
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "PLAN.md software slices 1-7 and recovery policy 1; hardware slices 8-9 remain evidence-gated"
  conditions:
    - "Update Pixhawk.md with cause and remedy only after dual-Pixhawk hardware acceptance succeeds"
    - "Describe the cause as a high-confidence inference and the Workstation change as a verified mitigation unless a PX4 stack trace or firmware fix proves more"
    - "Resolution documentation requires three independent cold-boot cycles of at least 10 minutes, a 60-minute dual-link full-load soak, one-side isolation, and verified recovery"
  recorded_at: "2026-08-26T16:32:20+09:00"
imu_stream_delivery_amendment:
  state: completed
  scope: "P1 SCALED_IMU/2/3 actual wire delivery and existing IMU-magnetometer UI rows"
  current_run: "PASS via verified USB runtime-only mavlink stream commands; approximately 17 Hz per stream; UI LIVE age 0.00-0.05 s"
  implementation: "sample-backed missing/stale stream retry; three attempts plus ten-second cooldown; one-second UI freshness; no retry on all-RX stall"
  verification: "129 focused; 306 expanded; 1682 passed/1 skipped full; actual P1 102/102/102 packets per six seconds; AT-SPI LIVE and STALE-to-LIVE pass"
  persistence: "FAIL: after verified reboot P1 broadcast HB/ATT returned but Ethernet profile commands were not received and SCALED_IMU/2/3 stayed absent"
  observed_boundary: "P1 broadcast telemetry alive while host neighbor 10.0.0.2 was INCOMPLETE and host Ethernet commands were absent on wire"
  forbidden: "no automatic reboot, no PX4 parameter/config/file persistence, no firmware change"
  readme_target: "README.md"
  blocking_decision: null
  recorded_at: "2026-08-31T09:12:00+09:00"
usb_free_persistence_attempt:
  state: failed
  operator_approval: "verified P1 USB reboot and P1/P2 Ethernet cable/port swap explicitly approved"
  reboot_evidence: "ATTITUDE time_boot_ms 3474164 -> 3579; P1 RECOVERING; P2 remained LIVE"
  host_wire_evidence: "static neighbor caused 255/190 heartbeat, full Stock profile and bounded IMU retries to appear on wire; P1 ACK and IMU response remained zero"
  p1_readback: "Ethernet instance received only self sysid=1; external GCS 255/190 absent; default streams omitted SCALED_IMU/2/3"
  cable_swap: "P1 ARP requests/replies 147/0; P2 5/5; fault followed P1 board, not cable/switch port"
  restored_state: "P1/P2 and cameras LIVE; P1 IMU approximately 20 Hz over ETH after verified USB runtime activation"
  next_boundary: "repair/replace P1 board-side PHY/driver/NuttX RX path, then rerun USB-free reboot and uninterrupted 10-minute test"
  recorded_at: "2026-08-31T09:58:00+09:00"
firmware_reinstall_attempt:
  state: verified_failed_to_fix_inbound_rx
  artifact: "Pixhawk/P1/firmware-reinstall-20260831T1003KST.json"
  image: "official PX4 v1.17.0 px4_fmu-v6x_default board_id=53 sha256=8f2c4278...abba0d7f"
  flash: "erase/program/verify/reboot PASS without force; settings and backups preserved"
  inbound_eth: "FAIL unchanged; P1 ARP 90/0 before USB stream activation"
  outbound_soak: "PASS 619.397s; 32034 contiguous MAVLink2 packets; ATTITUDE 15Hz; GPS1 5Hz; GPS2 1Hz; stalls 0"
  gnss_solution: "NO FIX: fix_type=0, satellites=0, lat/lon=0 on both GPS streams"
  p2_comparison: "same HW/git/network params; valid PC ARP; external GCS RX 38.9B/s loss0.1%; P1 self RX 4313B/s loss126.9%"
  physical_damage_proven: false
  remaining_hypotheses:
    - "P1 PHY/MAC receive failure"
    - "NuttX IOB/ARP exhaustion; official v6X CONFIG_IOB_NBUFFERS=24"
    - "P1 duplicate network initialization state (netman plus extras ifup)"
  recorded_at: "2026-08-31T10:32:00+09:00"
---

# Resume Notes

This packet is approved for implementation with recovery policy 1. On 2026-08-31 the named P1 USB
link was re-read as sysid/component `1/1`, PX4 `1.17.0`, NuttX `11.0.0`, with Ethernet UDP
`14550` and the stored `MAV_2_*` profile. P2 inventory and the full hardware acceptance matrix remain
open. Pixhawk.md remains unchanged until every hardware resolution gate succeeds.

Handoff integrity and workspace notes:

- `PLAN.md` SHA-256: `b51477f7d526e9eb0fb959e4bd7391341adef53d11c913d04555405bcea2249d`.
- The entire change packet is currently untracked; it is available to another session in this
  workspace but will not survive a fresh clone until intentionally added/committed.
- ETH owner files already have unrelated/user modifications; preserve their diffs.
- `python3 tools/check_portable_paths.py` currently reports 12 pre-existing violations in
  StereoWIRIS calibration/change evidence, Workstation `intrinsics_zoom_08.json`, Simulation
  `goal.md`, and StereoWIRIS settings. It did not report this ETH plan/packet.
