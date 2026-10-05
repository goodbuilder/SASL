# Backend Contract

## Commands and state

- `toggle_detection` owns a transient Measurement run only.
- `toggle_recording` owns the preserved Manual record path.
- `measurement_run_snapshot` reports `IDLE/QUEUED/STARTING/RUNNING/STOPPING/ERROR`.
- `data_storage_snapshot` reports root, per-stream state/path/error/time, queue age/depth, and drops.
- `retry_data_storage` retries failed streams without touching measurement/device state.
- Live Session/package/save/revision/recovery/verification commands are removed from the facade.

No stored Session/package/retry/completeness value is an admission input. A start may wait only for
the camera/video owner; ancillary input and storage failures are observed data, not lifecycle gates.

## Persistence

`measurement_data_root` defaults to `measurement_data` beneath SASL_WORKSTATION_DATA_ROOT. One
background owner serializes fixed JSONL and RTCM appends. Priority queue capacity is 256 and bulk
capacity is 4096; producers never wait. Bulk records drop first. Each stream fails independently,
retries on explicit command/next capture, and exposes all drops/errors.

Every structured record has its own UTC and monotonic record time and preserves source clock,
coordinate, axis, and unit fields. New top-level records have no Session/package membership.
Binary allocation uses local timestamp+UTC offset and collision suffixes; overwrite is forbidden.
No automatic rotation, manifest, whole-file hash, Parquet, XLSX, or completeness operation exists.

## Concurrency and late results

Manual record transitions to Measurement by stopping only its live camera/video writer and queuing
start. Measurement stop releases camera/detection/video ownership but does not wait for method,
native image, RTCM, ULog, artifact, or storage drains. An in-memory generation isolates late
callbacks; late results may append factually but cannot mutate current UI or counters.

Native frame persistence remains bounded at 16. RTCM routing/injection executes independently from
best-effort evidence append. Alignment state persists until explicit reset/revert or app restart.

## Restart and compatibility

Startup never scans/adopts/reconciles Session or retry spools. Existing recordings are untouched and
available through their paths only. Calibration photo-set and NTRIP transport sessions remain out
of this measurement-lifecycle contract.
