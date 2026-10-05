# Implementation Plan

Approved software slices implemented on 2026-08-31:

1. Versioned Radio-default settings migration for P1/P2/active Drone with preserved Ethernet data.
2. Three-role subscriber/liveness, sysid 1/2/3 enforcement, endpoint locks, explicit-connect-only
   lifecycle and 10-second qualification.
3. Critical Radio stream profile with 30-second initial and 15-second inter-tier recovery,
   diagnostic `RADIO_STATUS`, status/UI integration, and manual-only persistent administration.
4. Offset-aware MAVFTP upload/download/cancel/restart/verified-resume with transfer evidence and
   firmware `STAGED` state.
5. Hash-pinned official PX4 uploader, direct-USB role verification, globally serialized
   erase/program/verify/reboot, and postflight sysid verification.
6. Implemented 2026-09-01 follow-up: replace-only USB/network Discovery snapshots, one row per
   physical serial endpoint, identity-aware P1/P2/Drone role presentation, and focused reset/empty/
   error/duplicate-intent/UI regression tests.

The original third-radio/Drone-NETID27 field slice is superseded for the selected deployment by the
approved P1 Radio + P2 Ethernet + Drone Radio amendment below. The general independent-radio
capability remains available, but it is not the topology this guided preset commits.

## 2026-09-02 guided-topology amendment — implemented, hardware verification pending

1. Add pure topology/stage contracts, independent SiK/PX4 evidence parsers, invariants, inverse
   rollback plan, and atomic versioned operation-record persistence.
2. Add one bounded device worker with read-only Check first, exact remote-AT fixtures, targeted
   direct-USB P2 provisioning, cancellation, timeout, ownership and late-generation rejection.
3. Add controller/delegation/backend commands and snapshot. Reuse existing network-role and settings
   owners; prevent endpoint acquisition conflicts and preserve unrelated side operation.
4. Add the approved fixed `Field topology setup` group, stable five-row model, bounded statuses,
   current-step actions, one persistent-write confirmation, Resume/Retry/Cancel/Guided rollback,
   and existing explicit Connect handoff.
5. Add startup recovery and shutdown cancellation without startup device acquisition or automatic
   writes/reconnects.
6. Update `README.md`, numbered workflow/status/command/implementation docs, and this packet.
7. Verify pure state/journal recovery; fake raw SiK commands/readback/failure; targeted USB identity,
   armed/wrong/duplicate board rejection; P2 NIC/carrier/route/heartbeat; settings commit rollback;
   UI actions/geometry/focus; restart partial Resume/rollback; lifecycle and adjacent connection
   regressions.
8. Run focused tests after each slice, affected Pixhawk/settings/UI/lifecycle tests, full Workstation
   pytest, `./run.sh --check`, offscreen visual states, then real hardware forward/rollback and
   telemetry+RTCM qualification.

Stopping conditions: unexpected radio/PX4 firmware requirements, need for automatic failover or
physical power control, inability to prove one intended peer, armed device, active Measurement/
recording, settings-generation conflict, or any new cross-feature ownership transfer returns the
packet to approval before further source/device changes.

## 2026-09-03 — Superseding removal

The operator withdrew the guided-topology amendment. Remove its fixed UI, command/delegate
surface, topology controller/worker/core contract, journal path, lifecycle cleanup, lock guards,
and dedicated tests. Preserve the previously implemented per-board Radio/Ethernet/USB, Discovery,
NIC assignment, P2 ETH MAV, and explicit provisioning workflows. Verify the remaining Workstation
UI and Pixhawk connection suites plus the documented launcher.
