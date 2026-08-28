# Workstation Measurement Data Schema

## Purpose

This document defines the sessionless recording contract for `apps/Workstation`. It covers real
WIRIS, Pixhawk, GNSS/RTK, RTCM, ULog, Target Drone, native-frame, capture, Alignment, and
Measurement records. Storage observability is part of the contract; storage success is never a
Measurement admission condition.

## Ownership and identity

There is no persisted Measurement Session, parent/child package, revision, completeness, seal, or
restart-recovery identity for new data.

- `runtime_generation` identifies one transient Measurement or Manual-record interval when a
  producer needs correlation. It is not a directory or durability owner.
- `measurement_attempt_sha256`, method instance IDs, capture IDs, frame IDs, and source packet IDs
  keep their existing domain meanings.
- Every stored record receives `recorded_utc_ns` and `recorded_monotonic_ns` unless the producer
  already supplied them.
- `session_id`, `parent_session_id`, and `measurement_package_id` are removed recursively before a
  record is written. `measurement_id` may remain as a transient compatibility correlation field.

## Physical layout

`measurement_data_root` is a POSIX-style path relative to `SASL_WORKSTATION_DATA_ROOT`; it defaults
to `measurement_data`.

```text
<measurement_data_root>/
├── events.jsonl
├── measurements.jsonl
├── method_evaluations.jsonl
├── camera_left.jsonl
├── camera_right.jsonl
├── px4_left.jsonl
├── px4_right.jsonl
├── px4_drone.jsonl
├── gnss_base.jsonl
├── geometry_comparisons.jsonl
├── capture_events.jsonl
├── alignment.jsonl
├── artifacts.jsonl
├── application_log.jsonl
├── native_frames.jsonl
├── videos/
├── images/
├── frames/
├── ulog/
└── raw/gnss/
    ├── ntrip_transport_chunk.rtcm
    ├── ntrip_transport_chunk.rtcm.jsonl
    ├── ntrip_rtcm_frame.rtcm
    ├── ntrip_rtcm_frame.rtcm.jsonl
    ├── base_rtcm_frame.rtcm
    └── base_rtcm_frame.rtcm.jsonl
```

Structured files append across runs and are not rotated automatically. No new Measurement folder,
Parquet generation, `review.xlsx`, package manifest, or publication pointer is produced.

## Fixed stream contract

| Stream key | File | Priority | Meaning |
| --- | --- | --- | --- |
| `events` | `events.jsonl` | yes | transient run, command, and producer events |
| `measurements` | `measurements.jsonl` | yes | primary terminal pair results, including blocked results |
| `method_evaluations` | `method_evaluations.jsonl` | yes | atomic eight-method inputs/outcomes and late-result markers |
| `camera.left/right` | `camera_left/right.jsonl` | no | GUI-delivered camera observations |
| `px4.left/right/drone` | `px4_left/right/drone.jsonl` | no | P1, P2, and Target telemetry records |
| `gnss.base` | `gnss_base.jsonl` | no | Base GNSS observations |
| `geometry.comparisons` | `geometry_comparisons.jsonl` | no | analysis-only geometry comparison rows |
| `capture.events` | `capture_events.jsonl` | yes | HR/OZ request, outcome, and exact-import events |
| `alignment` | `alignment.jsonl` | yes | apply/revert/reset projections |
| `artifacts` | `artifacts.jsonl` | yes | binary allocation/close/import inventory |
| `application.log` | `application_log.jsonl` | no | redacted recording-time application log |
| `native.frames` | `native_frames.jsonl` | yes | async native-frame completion/path evidence |

JSONL is UTF-8, one JSON object per line, `allow_nan=False`. Unsupported/non-finite values reject
only that record. A malformed final historical fragment is retained as one bad line; the writer
adds a newline and continues. Historical bytes are never rewritten.

`LiveDataRows.replace(index, row)` does not edit the file. It appends a new record carrying
`_record_operation="replace"` and the same `_record_index`. Readers that want the current-run view
apply later replace records over earlier append records. Readers that need audit history retain both.

## Queue and failure contract

One daemon writer owns the fixed JSONL files.

- Priority capacity: 256 records.
- Bulk capacity: 4,096 records.
- A full priority queue drops the new priority record and increments that stream's drop count.
- A full bulk queue evicts the oldest queued bulk record and admits the new one.
- Open/write/fsync failure marks only the affected stream `FAILED`. `events`, `measurements`, and
  `method_evaluations` enter one FIFO retry reserve capped at 256 encoded records and 8 MiB, with
  1–30 second automatic backoff and immediate `Retry Storage`; overflow rejects the newest record.
  Other streams keep their existing drop behavior.
- Serialization failure drops only the malformed record; a later valid record continues normally.
- `Data > Status` exposes root, writer liveness, pending counts, per-stream state, last error, last
  write time, written count, and dropped count.
- `OK`, `DEGRADED`, or `UNAVAILABLE` never disables Measurement.

## Recording intervals

Camera/PX4/GNSS/RTCM/Target/application-log bulk records are emitted only while Measurement or
Manual record is active. Explicit HR/OZ capture/import, ULog import, and Alignment operations append
their event/artifact records even outside a recording interval.

Target Drone uses a spool-compatible adapter over `px4_drone.jsonl` in sessionless operation.
Writer refusal does not fail the receive-only MAVLink state machine or suppress its live snapshot.

## Binary provenance

MP4, WIRIS FTP stills, exact HR imports, native PNGs, and ULogs use local creation-time names:

```text
YYYYMMDD_HHMMSS_nanoseconds+offset_<label>.<extension>
```

The UTC offset is explicit. A collision receives `-01`, `-02`, and so on. Publication uses
no-replace semantics; an existing path is never overwritten.

- Native PNG: decoded native RGB, lossless PNG, native dimensions, no overlay or resize.
- WIRIS still/import: exact original file bytes plus decode, size, SHA-256, side, serial, purpose,
  and capture/import time evidence.
- ULog: exact validated `.ulg` bytes with source/provenance metadata.
- MP4: camera writer output. Recovery allocates a new
  `runtime_generation`/side/segment-index path instead of reopening an earlier file. Artifact
  events record allocation, writer detail, file presence, queue drops and timing.
- RTCM: exact append-only bytes plus offset, size, SHA-256, source, kind, message type, and host
  receive clocks in the sibling index.

Online post-run classification is file-presence only. Both sides present is `SAVED`, some present
is `PARTIAL`, and none present is `FAILED`. Unreviewed output records
`integrity_validation_status=NOT_RUN` and `recording_usable=false`; `SAVED` is not a playback or
integrity claim. Later OS/offline review never participates in Measurement admission.

Binary close/probe/hash completeness is not awaited before the next Measurement.

## Eight-method evaluation

The eight stable method IDs remain `pixhawk_rtk_live`, `checkerboard_stereo`, `horizon_hybrid`,
`mountain_gnss`, `drone_rtk_rotation_hybrid`, `drone_rtk_multiview`, `terrain_pnp`, and
`combined_hybrid`.

`pixhawk_rtk_live` remains the representative result. Missing structural inputs produce BLOCKED,
solver exceptions produce FAILED, deadline expiry produces TIMEOUT, and a full method work queue
produces BACKPRESSURE_BLOCKED. Comparison outcomes never replace the representative result.

One transient run freezes its method configuration. A callback from an older generation is appended
with `late=true` and cannot mutate the current UI or current-run row sequence.

## Coordinate contracts

- Public measurement: `baseline_center_frd`, midpoint of optical centers, `[Front,Right,Down]`, m.
- Camera optical: RDF `[Right,Down,Forward]`; image pixels `u` right, `v` down.
- PX4 world/local: NED `[North,East,Down]`, m.
- PX4 body: FRD `[Front,Right,Down]`.
- Rotation notation: `R_B_A` means `v_B = R_B_A @ v_A`.
- PX4 attitude: Hamilton `quaternion_wxyz`, body FRD into world NED.

No reader may infer a frame from generic `x/y/z`, combine MSL and ellipsoid height, or treat
OpenCV `T_left_to_right_m` as the right-camera origin `C_right_in_left = -R^T T`.

## Time contract

Clock domains remain distinct: host monotonic ns, host UTC ns, WIRIS RTCP/NTP, RTP media time, PX4
boot time, MAVLink `time_usec`, and GPS UTC. A derived UTC value remains labelled as derived; host
dispatch time never becomes a camera exposure timestamp.

Later collection should primarily use `recorded_utc_ns`, producer receive/source timestamps, and
`runtime_generation`. Filename timestamps identify binary creation, not necessarily sensor exposure.

## Legacy data

Existing `recordings/<id>/` and `recordings/sessions/<id>/` hierarchies remain untouched. Their old
Session/package/manifests/Parquet/workbook contracts apply only to those historical trees. New code
does not scan, recover, seal, convert, or append to them; the Data workspace exposes only a file
manager link.
