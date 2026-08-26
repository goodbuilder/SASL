# UI Quality and Stability

## Visual contract

Reuse the existing dense Qt WorkspaceHeader, compact action rows, EngineeringValue, StatusIndicator,
tables, Inspector, Diagnostics, and semantic severity colors. Introduce no dashboard cards, pills,
animation, gradients, modal storage errors, or decorative layout.

## Before and after

- `Sessions` becomes `Data` in the same navigation/workspace slot.
- Remove folder lifecycle, ordered Measurement package table, completeness/revision/verification,
  Retry Save, and stored Review controls.
- Retain/relocate Target, HR/OZ, ULog, file paths, and source-backed status.
- Measurement keeps Preview and its one Start/Stop command. Replace Session/package lines with
  fixed `Measurement`, `Recording`, `Storage`, and `Next` lines.

## Dynamic stability

Run state uses fixed labels `IDLE/QUEUED/STARTING/RUNNING/STOPPING/ERROR`; storage uses
`OK/DEGRADED/UNAVAILABLE`. Long roots/errors are elided with full tooltips. Stream rows update in
place without resize-to-contents during polling. Status changes cannot resize action rows, steal
focus, reset selection, move scroll, or alter saved dock/splitter/window geometry.

## Verification

Capture normal, queued, running, stopping, storage-degraded, per-stream-error, and overload states
at 1366x768, 1600x900, and 1920x1080. Verify keyboard navigation, non-color text cues, longest
Korean path/error text, rapid updates, selection/scroll stability, and no generic template styling.
