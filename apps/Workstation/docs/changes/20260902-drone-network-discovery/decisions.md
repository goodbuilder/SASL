# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-09-02 — New isolated change packet and documentation target

- User approved a new `20260902-drone-network-discovery` packet instead of expanding active P1/P2
  network-recovery work.
- The app-local `apps/Workstation/README.md` is the confirmed README target.
- Requested outcome is to add Telemetry `Find Network` and then connect the currently attached
  Drone Pixhawk by LAN.

## 2026-09-02 — Proposed safe discovery semantics

- Discovery is read-only; a candidate is selectable only with decoded `sysid=3` evidence.
- Selection fills the existing URL but does not silently Apply, Save, Disconnect or Connect.
- Existing Find USB and manual endpoint paths remain as fallbacks.
- Actual success requires live expected-system evidence after explicit Connect.

## 2026-09-02 — Scope amendment and implementation approval

- User reported that Telemetry Apply/Save can restore the previous MAVLink URL and explicitly
  requested this defect be fixed in the same packet.
- The accepted form URL must survive Apply, Save and periodic refresh; a stale live-connection
  endpoint is status evidence only and cannot overwrite configuration input.
- User explicitly approved the combined discovery, URL-preservation, documentation, verification
  and real Drone connection scope on 2026-09-02.
