# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-09-02 - GNSS-derived display requested

- Operator requested F/R/D/Rg from live P1/P2/Drone GNSS positions.
- Current code already supplies scalar Rg only.

## 2026-09-02 - Separate GNSS frame direction accepted

- Recommended and operator accepted a GNSS-only local-level definition: origin at P1/P2 GNSS
  antenna midpoint, horizontal P1->P2 as Right, NED Down as Down, and Right x Down as Forward.
- It must be labelled separately and must not overwrite camera/measurement `baseline_center_frd`.

## 2026-09-02 - Initial technical safety proposal

- Initial proposed fixed display gates were 2.0 s maximum source age and 200 ms maximum three-source
  receive-timestamp spread. Finite stale values remain visible; RTK fix is evidence, not a gate.
- Proposed placement is one fixed one-row engineering table below Telemetry `Live position` with no
  command, setting, worker, persistence, or recording field.
- Existing ideal disparity will reuse the same Rg/readiness to avoid contradictory values.

## 2026-09-02 - Operator selects 1000 ms three-GNSS spread

- The operator replaced the initial 200 ms proposal with a 1000 ms maximum receive-timestamp
  spread. The 2.0 s per-source age limit remains unchanged.
- This improves availability for asynchronous GNSS streams. The UI must keep actual skew visible;
  motion within a one-second spread can bias signed F/R/D and remains an explicit field limitation.

## 2026-09-02 - Complete packet approved for implementation

- The operator explicitly approved the updated approval summary with 2.0 s maximum source age,
  1000 ms maximum three-GNSS receive-timestamp spread, separate GNSS frame identity, fixed
  Telemetry table, shared Rg/readiness, and all documented exclusions.
