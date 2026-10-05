# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-08-31 — Workbook content and lifecycle

- Include Measurement-related data, not all Workstation streams.
- Keep one `latest_measurement.xlsx` for the most recent Measurement only.
- Generate automatically after stop and provide Data Status update/open actions.
- Use Summary plus source-specific sheets; do not time-join unlike-rate streams.
- Represent missing cells with Excel `#N/A`; never invent missing sample rows.
- Preserve original clock fields and add readable KST ISO columns.
- JSONL remains authoritative; XLSX is derived and failure never gates Measurement.

## 2026-08-31 — Implementation approval

The user approved the complete proposed plan with `Implement the plan.` at
`2026-08-31T16:15:22+09:00`. Approval is limited to this packet and excludes unrelated cleanup,
dependency upgrades, session restoration and data migration.

## 2026-08-31 — README targets confirmed

The user answered `응` to confirm both `apps/Workstation/README.md` and
`apps/Workstation/docs/ui/README.md` at `2026-08-31T19:19:12+09:00`. Both now describe the
implemented latest-only derived workbook without changing historical Session guidance.
