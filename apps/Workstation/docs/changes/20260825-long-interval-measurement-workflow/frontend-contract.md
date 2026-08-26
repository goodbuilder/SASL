# Frontend Contract

## Measurement

- The Measurement Start/Stop command remains enabled in every storage and legacy-data state.
- Visible run state: `IDLE`, `QUEUED`, `STARTING`, `RUNNING`, `STOPPING`, `ERROR`.
- Busy camera operations produce `QUEUED` plus one factual owner/reason; duplicates coalesce.
- Manual record is stopped automatically before Measurement starts and stays off afterward.
- Storage failure appears only in the Storage line/status/Diagnostics and never as command failure.
- Retry Save, package completeness, revision, verification, and Session instructions are absent.

## Data

The Data workspace shows the configured root and one stable row per data type: effective path,
writer state, pending count, dropped count, last write time, and bounded error summary. It provides
`Open Data Folder`, `Open Legacy Recordings Folder`, `Retry Storage`, HR/OZ, ULog, and Target tools.
It does not scan, group, preview, verify, or claim completeness for Session data.

## Settings

Storage has one data-root-relative directory field and directory picker. Apply during active capture
is marked pending and takes effect at the next capture. External volumes use
`SASL_WORKSTATION_DATA_ROOT`; an absolute checkout/home path is never persisted.

## Errors and accessibility

No routine success/storage-failure modal is used. Text and severity both convey state. Long dynamic
values are elided with tooltips and do not move controls, steal focus, or reset selection/scroll.
Legacy `tasks.session` deep links resolve to Data.
