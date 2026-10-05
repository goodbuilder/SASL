# Approval Summary

## Approved outcome

New Workstation measurements have no Session create/close/seal/recovery prerequisite. An old
OPEN/RECOVERABLE Session, failed save, missing artifact, or unwritable data destination never
disables or rejects Measurement Start. Only a live camera/resource transition may delay start; the
request is queued and runs when that owner is free.

## Operator workflow

```text
Launch -> prepare devices as available -> Measurement Start -> measure with available inputs
       -> Measurement Stop -> READY/queued next start
```

Manual record remains available. Starting a Measurement while it is active first stops the manual
video writer, then automatically starts Measurement; manual recording does not resume afterward.

## Data behavior

- One configured root under SASL_WORKSTATION_DATA_ROOT owns fixed append files by data type.
- Structured records append only during Measurement or Manual record; explicit HR/OZ, ULog import,
  and Alignment events append when invoked.
- Video, image, and ULog originals use collision-free Korean-local creation-time filenames.
- No new Session folder, package, revision, completeness verdict, Parquet, or review.xlsx is made.
- Existing Session directories are untouched and available only through the file manager.
- Storage errors and overload are visible but never block calculation, display, or the next run.
  Measurement results/events have reserved priority; bulk camera/PX4/GNSS/RTCM records drop first.

## UI and performance

Sessions becomes Data. It shows the root, per-type writer health, dropped counts, Open Folder,
Retry Storage, HR/OZ, ULog, and Target tools. Measurement shows transient run/recording/storage
state without package or recovery controls. One bounded CPU writer replaces the Session save and
media-verification workers; GPU/inference policy and measurement math do not change.

## Compatibility, risks, and verification

No existing evidence is migrated, rewritten, or deleted. Calibration photo-set sessions and NTRIP
transport sessions are different concepts and remain. The approved tradeoff is that Workstation no
longer proves cross-type completeness; the operator gathers data later by factual timestamps.
Verification must reproduce yesterday's old recoverable Session and prove Measurement Start still
runs, then cover storage failure, overload, busy-camera queuing, Manual-to-Measurement handoff,
sequential runs, late callbacks, shutdown, UI stability, and unchanged coordinate contracts.

## Approval record

- State: approved
- Scope: the complete sessionless availability-first plan agreed in the conversation
- Approved at: 2026-08-26T12:36:43+09:00
- Source implementation authorized by: user request, "Implement the plan."
