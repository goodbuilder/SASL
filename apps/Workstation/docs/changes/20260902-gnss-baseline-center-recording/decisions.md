# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-09-02 - Derived persistence requested

- Operator asked about saved data after approving the runtime display.
- Current implementation saves raw P1/P2/Drone GNSS only; F/R/D/Rg itself is not stored.

## 2026-09-02 - Dedicated stream direction accepted

- Recommended one dedicated `gnss_baseline_center.jsonl` record per primary Drone GNSS callback,
  with exact three-source timestamps/IDs, state and frame contract, never canonical Measurement
  fields. Operator answered `좋아`.

## 2026-09-02 - Complete technical proposal awaiting approval

- Proposed scope includes JSONL plus additive Latest Measurement XLSX sheet, all states, selected
  P1/P2 sources, existing bulk writer/drop semantics, no Parquet/backfill/commands/workers/UI layout.

## 2026-09-02 - Complete persistence packet approved

- Operator explicitly approved the full approval summary: primary-Drone callback trigger,
  self-contained JSONL rows, additive Latest XLSX sheet, existing bulk/drop behavior and all listed
  exclusions.
