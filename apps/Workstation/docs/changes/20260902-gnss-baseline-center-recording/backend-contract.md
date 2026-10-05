# Backend Contract

## Commands and validation

No command. On `_update_gps(camera="drone", gps)` after the raw row, call one Recording-owner
method. It returns immediately unless `distance_recording_enabled` is true. `drone_GPS2` and UI
refresh never trigger it.

Record fields include:

- schema/event/trigger/sequence and existing recording/generation/host/source clocks;
- `geometry_state`, `geometry_reason`;
- `coordinate_frame`, `axis_contract`, `vector_contract`, `origin_contract`, and axis definitions;
- `gnss_baseline_center_forward_m/right_m/down_m/range_m`;
- baseline N/E/D, Drone-from-P1 N/E/D and midpoint-to-Drone N/E/D intermediates;
- maximum age and timestamp spread seconds/milliseconds;
- selected P1/P2 receiver labels;
- per-source latitude/longitude/MSL altitude, fix/label, topic, app-receive monotonic ns, GPS UTC,
  MAVLink system/component/sequence/message id.

Missing or decoded non-finite GNSS position/accuracy/timestamp values are JSON null, not zero/empty
aliases; the state remains explicit WAITING/STALE. Any other unsupported producer value rejects
only that row.

## State, ownership, and concurrency

RecordingController owns row construction and per-recording sequence. Sensor presentation only
hands off the event. DataStorageController/MeasurementDataStore remains the sole file owner. The
pure estimator is stateless. All producer work is on the existing callback thread and enqueue is
non-blocking.

## Worker resource budgets, scheduling, and backpressure

Add `gnss.baseline.center` to existing bulk queue, not priority or essential retry reserve. No new
worker/queue. Target row <=4 KiB. At 10 Hz, planned admitted demand <=40 KiB/s. On full bulk queue,
existing oldest queued bulk record is evicted; counters identify the affected stream.

## External boundaries and workers

No device/API boundary. Existing decoded `GpsPositionState` inputs only. File path is fixed
`gnss_baseline_center.jsonl` under current data root.

## Results, errors, timeout, retry, and cancellation

- One attempted append per primary Drone GNSS callback while recording.
- CURRENT/STALE/WAITING all append.
- Writer admission false is evidence, not an exception propagated to Telemetry.
- Existing stream failure/retry, store lifecycle rejection, checkpoint and shutdown semantics.
- No feature-specific timeout/cancel/retry.

## Persistence, restart, and shutdown

UTF-8 append-only JSONL, store metadata clocks, allow_nan=False, fixed stream snapshot. Restart
appends; no backfill/migration. Existing close/checkpoint includes the new stream automatically.
Latest XLSX adds a fixed raw-projection sheet and exact-generation filtering. Legacy Parquet and
session package schemas remain unchanged.
