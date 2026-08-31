# Frontend Contract

## Controls and inputs

- `Update Latest XLSX`: enabled only when no Measurement/Manual recording or export is active.
- `Open Latest XLSX`: enabled only for an existing successfully published workbook.
- No path picker; output follows the resolved Measurement data root.

## Payloads and commands

Update calls `request_latest_measurement_xlsx()` with no generation so the controller discovers
the latest saved Measurement. Open uses the exact path from the status snapshot.

## Visible states and transitions

`NOT_CREATED -> WAITING -> GENERATING -> READY`; a newer target yields `STALE`, failures yield
`FAILED`, and cooperative cancellation yields `CANCELLED/WAITING` as applicable. The label shows
published versus target generation when stale.

## Errors and recovery actions

Foreign-file collision, no discoverable Measurement, checkpoint/read/write/validation/replace
failure and cancellation remain visible. Update is the recovery action; no modal loop is used.

## Keyboard, focus, accessibility, and layout

Native tab order, explicit accessible names/tooltips, fixed two-line status geometry, compacted
visible detail and full tooltip. Status refresh never moves focus or selection.

## Cross-feature visible behavior and isolation

Measurement remains immediately reusable; Camera/Pixhawk/GNSS availability only changes workbook
row counts and `#N/A`, never button access to unrelated Data/capture tools.
