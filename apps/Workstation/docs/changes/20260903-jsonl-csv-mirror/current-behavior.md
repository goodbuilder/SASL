# Current Behavior

## Version and worktree context

- Repository/worktree: `/home/moontaehyung/바탕화면/SASL-20260821`.
- Baseline identity: current branch `fix/workstation-optical-pose-evidence-geometry-hardening`,
  HEAD `3288aee` (`feat(workstation): show baseline yaw angles`).
- The worktree is heavily dirty with user-owned and previously approved Workstation, Legacy, and
  Simulation changes. This packet must touch only the storage mirror slice and its direct tests/docs.

## Reproduction

1. Start Workstation and run a sessionless Measurement or Manual recording.
2. Open the resolved Measurement data root or `Data > Status`.
3. Observe fixed files such as `events.jsonl`, `measurements.jsonl`, `camera_left.jsonl`, and
   `native_frames.jsonl`; there are no matching CSV sidecars from the sessionless store.
4. `latest_measurement.xlsx` is a separate derived workbook and is not a CSV mirror of all streams.

## Observed frontend behavior

`Data > Status` lists one row per fixed stream with its JSONL path, JSONL state, JSONL drop count,
last write time, and error. It has existing folder/retry actions and the latest XLSX update/open
actions, but no CSV path or CSV health projection.

## Observed backend and persistence behavior

`MeasurementDataStore` owns one daemon writer thread and 16 fixed JSONL paths in
`workstation_core/measurement_data_store.py`. Producers enqueue canonical JSON objects; the writer
adds `recorded_utc_ns`/`recorded_monotonic_ns`, removes Session membership fields, serializes with
`allow_nan=False`, and appends one JSON object per line. Priority and bulk queues, essential retry
reserve, per-stream drop/error state, flush, checkpoint, and restart append behavior are already
part of the contract.

The current writer has no CSV handles, CSV retry state, CSV paths, or CSV status. The existing
legacy/session CSV helpers and the derived latest XLSX reader are separate owners and do not mirror
the sessionless JSONL files.

## Documentation, code, test, and runtime conflicts

The current product documents intentionally describe JSONL as authoritative and list no CSV
sidecars. The latest XLSX packet explicitly excluded a sidecar. The requested change supersedes
only that omission: JSONL remains authoritative, XLSX remains derived, and the new CSV files are
best-effort derived mirrors. Existing dirty documentation and source edits are not evidence that
this new scope is already implemented.

## Field and change audit

The detailed inventory is in `field-inventory.md`. A real 2026-09-03 sessionless recording shows
that the files are not uniform tables: `measurements.jsonl` has 562 top-level fields, camera files
have 58 each, P1/P2 have 157/149, Drone has 134, and GNSS baseline-center has 110. PX4 files mix
attitude, GPS, GNSS-relative, magnetometer/IMU, radio, local/global position, SYS, and VFR records
under one path, so a single wide CSV would contain many empty/sparse cells and be slow to inspect.

The current store is full-snapshot append, not change-only. `enqueue()` serializes the complete
normalized producer mapping; it does not compare with a previous row. `LiveDataRows.replace()` adds
another complete record with the same index and a replace operation. In the representative run,
`measurements.jsonl` had 294 fields constant across the sampled file and 268 changing; camera/PX4/
GNSS rows also repeated static contracts while their clocks and sensor values changed.

For quick operator search, the evidence supports two CSV layers: a lossless stream mirror and
curated quick views split by measurement/telemetry type. Change-only output should be an additional
state-transition summary, not a replacement for the full camera/PX4/GNSS/measurement time series.
The raw `raw/gnss/*.rtcm.jsonl` binary indexes are a separate owner and need a separate decision.
