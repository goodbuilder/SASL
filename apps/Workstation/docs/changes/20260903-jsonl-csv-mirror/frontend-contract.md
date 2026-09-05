# Frontend Contract

## Controls and inputs

No new input is required. The existing `Data > Status` stream table adds a paired CSV path and
mirror-state projection. Existing `Open Data Folder` and `Retry Storage` remain the operator actions.

## Payloads and commands

No command payload changes. `append_workstation_data()` and `retry_data_storage()` remain the
public controller boundary. The UI consumes `data_storage_snapshot()['streams'][stream]` and does
not access private CSV handles or files directly.

## Visible states and transitions

Each stream continues to expose its JSONL `IDLE/PENDING/ACTIVE/FAILED` state and adds an independent
CSV mirror state. A normal row is `CSV ACTIVE`; a queued or failed mirror is shown as `CSV RETRYING`
or `CSV FAILED` while the JSONL state remains factual. Store-wide `DEGRADED` includes CSV failures or
drops.

## Errors and recovery actions

The table/error detail identifies the CSV path and failure without turning the whole Measurement
workflow into an error. `Retry Storage` is the recovery action. If CSV is unavailable, the operator
can still use JSONL/XLSX and continue Measurement.

## Keyboard, focus, accessibility, and layout

Use the existing non-editable `QTableView`, compact headers, selectable paths, and current fixed
height. Refresh updates the model in place, preserves selection/scroll/focus, and uses text/tooltips
for meaning rather than color alone. Long path/error values are bounded in the visible cell.

## Cross-feature visible behavior and isolation

Camera, Pixhawk, GNSS, Detection, Measurement, and the latest XLSX action remain usable when one CSV
mirror fails. Storage degradation is diagnostic evidence only and never disables `측정 시작` or the
next run.
