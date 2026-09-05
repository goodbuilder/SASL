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

An empty `measurement_data_root` selects `<SASL_ROOT>/recordings` for a normal launch. When
`SASL_WORKSTATION_DATA_ROOT` is explicitly configured, the empty default selects its `recordings/`
child. A non-empty value is a POSIX-style path relative to `SASL_WORKSTATION_DATA_ROOT`. The former
default spelling `measurement_data` is normalized to the empty default when settings load; existing
bytes in that historical directory are not moved or rewritten.

```text
<resolved Measurement data root>/
├── latest_measurement.xlsx
├── events.jsonl
├── events.csv
├── measurements.jsonl
├── measurements.csv
├── method_evaluations.jsonl
├── method_evaluations.csv
├── camera_left.jsonl
├── camera_left.csv
├── camera_right.jsonl
├── camera_right.csv
├── px4_left.jsonl
├── px4_left.csv
├── px4_right.jsonl
├── px4_right.csv
├── px4_drone.jsonl
├── px4_drone.csv
├── gnss_base.jsonl
├── gnss_base.csv
├── gnss_baseline_center.jsonl
├── gnss_baseline_center.csv
├── geometry_comparisons.jsonl
├── geometry_comparisons.csv
├── capture_events.jsonl
├── capture_events.csv
├── alignment.jsonl
├── alignment.csv
├── artifacts.jsonl
├── artifacts.csv
├── application_log.jsonl
├── application_log.csv
├── native_frames.jsonl
├── native_frames.csv
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
Parquet generation, legacy `review.xlsx`, package manifest, or publication pointer is produced.
`latest_measurement.xlsx` is one replaceable derived view of the most recently completed
Measurement; it is not a Session, archive, completeness verdict, or authoritative store.

Each fixed JSONL stream has a same-stem UTF-8 CSV mirror (`.jsonl` → `.csv`) for newly persisted
records. The CSV header is `workstation-measurement-csv-v1` and contains the recorded clocks,
common identity/status fields, and the complete canonical JSON object in `record_json`. JSONL is
written first and remains authoritative; a CSV mirror may be temporarily `RETRYING`, `FAILED`, or
`DROPPED` without rejecting Measurement. Existing JSONL files are not backfilled automatically.

## Latest Measurement XLSX

After a Measurement stops, Workstation requests a background rebuild of
`latest_measurement.xlsx`. `Data > Status` exposes `Update Latest XLSX`, `Open Latest XLSX`, and
the factual `NOT_CREATED/WAITING/GENERATING/READY/STALE/FAILED/CANCELLED` state. The request occurs
after the run has returned to `IDLE`; delay, cancellation, failure, or a locked/foreign destination
never blocks the next Measurement.

The workbook selects exactly one `runtime_generation`, using `runtime_generation` or the
sessionless compatibility `measurement_id` stored on each source row. It contains Summary,
Measurement events, materialized and append/replace result history, nine-method bundles/outcomes/
inputs, camera, P1/P2/Target, Base GNSS, GNSS baseline-center, geometry, native-frame,
file-inventory, and issue sheets.
Manual-record-only and unrelated application/capture/alignment streams are excluded. Differently
sampled sources are not time-joined.

Original clock values remain exact; integers beyond Excel's 15-digit precision are text and UTC
epoch fields gain adjacent nanosecond-precision KST ISO display columns. Missing cells are Excel
`#N/A`; absent samples are never invented. Row, column, and cell limits produce deterministic
shards/continuation columns inside the same workbook.

The writer takes a background-only durable byte-cutoff checkpoint, builds and validates a hidden
temporary XLSX, and atomically publishes it. A prior Workstation-schema workbook may be replaced;
an unmarked same-name file is treated as user-owned and preserved. Malformed/torn source records
are excluded with `Issues` evidence. JSONL remains authoritative in every success and failure path.

## Fixed stream contract

| Stream key | File | Priority | Meaning |
| --- | --- | --- | --- |
| `events` | `events.jsonl` | yes | transient run, command, and producer events |
| `measurements` | `measurements.jsonl` | yes | representative `pixhawk_rtk_live` `VALID`/`ESTIMATED` terminal results accepted while the run is `RUNNING` |
| `method_evaluations` | `method_evaluations.jsonl` | yes | atomic nine-method inputs/outcomes and late-result markers |
| `camera.left/right` | `camera_left/right.jsonl` | no | GUI-delivered camera observations |
| `px4.left/right/drone` | `px4_left/right/drone.jsonl` | no | P1, P2, and Target telemetry records |
| `gnss.base` | `gnss_base.jsonl` | no | Base GNSS observations |
| `gnss.baseline.center` | `gnss_baseline_center.jsonl` | no | one GNSS antenna-midpoint F/R/D/Rg evidence row per primary Drone GNSS callback while recording |
| `geometry.comparisons` | `geometry_comparisons.jsonl` | no | analysis-only geometry comparison rows |
| `capture.events` | `capture_events.jsonl` | yes | HR/OZ request, outcome, and exact-import events |
| `alignment` | `alignment.jsonl` | yes | apply/revert/reset projections |
| `artifacts` | `artifacts.jsonl` | yes | binary allocation/close/import inventory |
| `application.log` | `application_log.jsonl` | no | redacted recording-time application log |
| `native.frames` | `native_frames.jsonl` | yes | async native-frame completion/path evidence |

JSONL is UTF-8, one JSON object per line, `allow_nan=False`. Unsupported/non-finite values reject
only that record. A malformed final historical fragment is retained as one bad line; the writer
adds a newline and continues. Historical bytes are never rewritten. CSV uses standard RFC-style
quoting and copies coordinate/frame and clock fields without conversion.

`LiveDataRows.replace(index, row)` does not edit the file. It appends a new record carrying
`_record_operation="replace"` and the same `_record_index`. Readers that want the current-run view
apply later replace records over earlier append records. Readers that need audit history retain both.

### GNSS baseline-center derived rows

`gnss.baseline.center` is triggered only by primary Drone `GPS_RAW_INT` while Measurement or Manual
Record is active. Drone `GPS2_RAW`, UI polling, and inactive recording do not create rows. Every
trigger attempts one row, including WAITING and STALE; writer admission/drop truth remains in Data
Status. The stream is bulk, non-priority, and has no essential retry reserve.

Each row uses `workstation-gnss-baseline-center-record-v1` and includes:

- standard measurement/generation, sequence, sample/host/source clocks and trigger contract;
- `geometry_state`/`geometry_reason`;
- `coordinate_frame=gnss_baseline_center_local_level_frd`, axis/vector/origin definitions;
- `gnss_baseline_center_forward_m/right_m/down_m/range_m`;
- baseline, Drone-from-P1 and midpoint-to-Drone NED component fields;
- maximum source age and three-source timestamp spread in seconds/milliseconds;
- selected P1/P2 receiver source labels;
- self-contained P1/P2/Drone WGS84 latitude/longitude, MSL altitude, fix/label, satellite/
  accuracy, topic, application receive monotonic ns, GPS UTC, time source and MAVLink provenance.

Unavailable values are JSON null; a real zero remains numeric zero. The row is GNSS antenna-
midpoint local-level evidence and never aliases camera/Measurement `baseline_center_*` fields.
Latest Measurement XLSX projects exact-generation rows to `GnssBaselineCenter`; Manual-record-only
rows stay JSONL-only. Legacy session Parquet/review output is unchanged.

### Representative-before-comparison ordering

`pixhawk_rtk_live` is the only representative method. When its terminal outcome is `VALID` or
`ESTIMATED` and the owning Measurement is still `RUNNING`, Workstation priority-enqueues the
`measurements.jsonl` row before projecting it into the current table, distance history, plot, or UI
banner. The enqueue attempt/result is observed before those projections; a rejected enqueue may
still be shown as degraded evidence. The primary worker is spawned and dispatched first. Its
terminal outcome, including the one-second primary timeout, releases startup of the seven
comparison workers. This acceptance is
independent of the completed nine-method bundle.

Primary `BLOCKED`, `FAILED`, `TIMEOUT`, and `BACKPRESSURE_BLOCKED` outcomes do not create
`measurements.jsonl` rows. They remain explicit outcomes in `method_evaluations.jsonl` alongside all
comparison results. A later method-evaluation bundle never promotes a comparison result or changes
an already accepted representative result. If the early priority enqueue was rejected while the run
is still `RUNNING`, an independent primary-persistence owner retries the same already-built row; a
saved nine-method bundle does not cancel that retry. The row carries an owner-epoch/index lease,
so clear/rebind/index reuse retires the owner instead of overwriting another row. An unavailable
owner falls back to direct priority persistence. After Stop, retry writes a compact
`late_primary_measurement_outcome` with the canonical primary outcome and original generation,
without updating the next run's UI or counters. The retry updates
`primary_committed_before_bundle` and `primary_commit_source` to the actual successful source rather
than retaining metadata from the rejected attempt, and never schedules native capture again.

Successful representative rows add stable join evidence without changing the v5 compatibility
fields: `runtime_generation`, `method_evaluation_pair_id`,
`method_evaluation_pair_sequence`, `method_evaluation_request_generation`,
`method_evaluation_snapshot_sha256`, `primary_outcome_status`,
`primary_outcome_sha256`, `primary_committed_before_bundle`, and
`primary_commit_source`. The method-evaluation pair sequence is distinct from the positional
`sample_index`/`_record_index` used by the append log.

The state check belongs to outcome arrival. If the primary is accepted while the run is `RUNNING`,
a subsequent Stop preserves that queued record. If Stop occurs first, a later primary creates no
representative row and cannot update the distance UI; its method outcome may still be retained as
late method-evaluation evidence. "Before the UI" and "immediate" mean successful bounded priority
enqueue. They do not mean that the writer has completed `write`, flush, or `fsync`.

Primary admission and comparison backlog have separate 20-pair limits. When comparison backlog is
full but a primary slot remains, the pair is admitted to the primary worker and its seven comparison
outcomes are synthesized as `BACKPRESSURE_BLOCKED`. Only a full primary queue rejects the complete
pair with primary backpressure.

A nine-method bundle is committed, counted, and exposed as complete only after its priority
enqueue is admitted. A rejected append to the active `LiveDataRows` owner remains a pending logical
row and retries that same index. If the run stops before admission, the retry stays with the original
row owner while its lease remains valid and is rewritten with `late=true` and the then-current
runtime generation. A retired/closed/cleared owner is detached and retried through the direct
writer. Late bundles are globally deduplicated but never enter the new run's committed set,
counters, last bundle, or distance UI.

Primary and bundle persistence owners each retain at most 1,024 entries and 8 MiB and retry one
entry per timer tick in round-robin order. Bound overflow and the shutdown retry budget produce
explicit event-attempt, counter, log, and status evidence before ownership is released. This bounds
memory and shutdown time; if all storage paths reject even that diagnostic event, it is an explicit
terminal drop rather than a durable-write claim.

Native completion has separate bounded ownership. A request fingerprint makes the first canonical
terminal payload authoritative, exact duplicates no-ops, and conflicts one diagnostic event without
changing the row. If both row projection and `native.frames` admission fail, the payload, original
owner, and lease remain pending for bounded background retry. Exhaustion records terminal-drop
counter/status/log evidence and retains the payload in a bounded diagnostic cache so shutdown is
finite; complete storage unavailability can still prevent durable recording of that final diagnostic.

## Queue and failure contract

One daemon writer owns the fixed JSONL files and their same-stem CSV mirrors.

- Store lifecycle is `OPEN → CLOSING → CLOSED`. Only `OPEN` accepts enqueue. A request observed
  during `CLOSING` or `CLOSED` is rejected, increments lifecycle/stream drop evidence, and never
  enters either bounded queue. `retry_failed()` is an active lifecycle operation: close waits for an
  already-started retry, while a retry that loses the close race cannot move reserve data into a
  dead queue.
- Priority capacity: 256 records.
- Bulk capacity: 4,096 records.
- A full priority queue drops the new priority record and increments that stream's drop count.
- A full bulk queue evicts the oldest queued bulk record and admits the new one.
- Open/write/fsync failure marks only the affected format/stream `FAILED`. JSONL keeps its existing
  retry/drop behavior. A CSV failure retains the successful JSONL row, places the canonical row in
  a separate FIFO CSV retry reserve capped at 256 records and 8 MiB, and uses 1–30 second automatic
  backoff plus immediate `Retry Storage`; CSV overflow increments only the CSV drop count.
  Measurement and unrelated streams remain usable.
- Serialization failure drops only the malformed record; a later valid record continues normally.
- `Data > Status` exposes root, writer liveness, pending counts, per-stream JSONL/CSV paths and
  states, last error, last write time, written count, separate JSONL/CSV drop counts, and
  latest-XLSX request/publication state.
- `OK`, `DEGRADED`, or `UNAVAILABLE` never disables Measurement.

## Recording intervals

Camera/PX4/GNSS/RTCM/Target/application-log bulk records are emitted only while Measurement or
Manual record is active. Explicit HR/OZ capture/import, ULog import, and Alignment operations append
their event/artifact records even outside a recording interval.

An async native-frame request binds its request ID to the row collection, row index, and runtime
generation that created it. Completion after a later run starts updates only that original owner
when available; it cannot reuse the next run's same positional index. Sessionless completion also
attempts a priority `native.frames` event under the original runtime generation, providing an
independent best-effort fallback when row projection is unavailable or late.

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
- MP4: overlay-free decoded NV12 re-encoded by isolated FFmpeg `h264_nvenc` lossless H.264. Camera
  recovery allocates a new `runtime_generation`/side/segment-index path rather than reopening an
  earlier file. There
  is no application bitrate, file-size, duration, or rotation cap and no CPU encoder fallback.
  Artifact events record allocation, writer detail, file presence, queue drops, and timing.
- RTCM: exact append-only bytes plus offset, size, SHA-256, source, kind, message type, and host
  receive clocks in the sibling index.

Binary close/probe/hash completeness is not awaited before the next Measurement.

## Camera media and MP4 provenance

Decoded camera frames are always admitted under `media_integrity_mode=tolerant_decoded`.
`media_integrity_unverified=true` means RTP/IDR/source-media completeness was not proven; it does
not by itself invalidate the decoded frame or a Measurement. Camera and pair records preserve:

- `media_integrity_unverified`
- `source_media_integrity_verified`
- `rtp_integrity_latched`
- `media_random_access_guaranteed`
- `measurement_input_admitted`

An official distance remains `VALID` when the pre-existing timing, calibration, correspondence,
and geometry gates pass. `INPUT UNVERIFIED` is an independent input-provenance label and may coexist
with that result.

GStreamer recording uses `recording_encoding_mode=decoded_nvenc_lossless_reencode` and scope
`decoded-nv12:nvenc-lossless+finalized-mp4;source-media-integrity=tolerant-unverified`. The capture
and Python writer queues each have capacity two and evict the oldest recording frame under load;
they never backpressure Preview, YOLO, native evidence, or Measurement. Fixed-rate duplication of
the preceding decoded frame may represent a source-PTS gap, recorded as
`recording_timing_basis=source_pts_fixed_rate_gap_duplication`, `dropped_frames`, and
`duplicated_frames`. No exact source-to-MP4 frame index is claimed.

Workstation's online post-run classification is file-presence only. Both sides present is `SAVED`,
some present is `PARTIAL`, and none present is `FAILED`. It records size and writer detail but sets
`integrity_validation_status=NOT_RUN` and `recording_usable=false`; `SAVED` is not a playback or
integrity claim. `source_media_integrity_verified` remains a separate fact. Hash/probe/decode review
is outside Measurement admission and the next-run lifecycle.

## Nine-method evaluation

The nine stable method IDs remain `pixhawk_rtk_live`, `pixhawk_rp_gps_baseline_yaw`,
`checkerboard_stereo`, `horizon_hybrid`, `mountain_gnss`, `drone_rtk_rotation_hybrid`,
`drone_rtk_multiview`, `terrain_pnp`, and `combined_hybrid`.

`pixhawk_rtk_live` remains the representative result. Missing structural inputs produce BLOCKED,
solver exceptions produce FAILED, deadline expiry produces TIMEOUT, and a full method work queue
produces BACKPRESSURE_BLOCKED. Primary timeout is one second and releases comparison-worker startup.
Only a `RUNNING` primary `VALID`/`ESTIMATED` outcome enters
`measurements.jsonl`; every other primary status and every comparison status remains in
`method_evaluations.jsonl`. Comparison outcomes never replace the representative result and are not
a barrier to its priority enqueue.

For `mountain_gnss`, the newest Draft present before Measurement start is frozen into that
Measurement. Its translation axes are ready when both capture-bound GNSS samples have finite
positions and numeric `fix_type >= 3`, and their P1-to-P2 NED baseline is finite and non-zero.
Both-side `RTK_FIXED` is not required. A usable non-fixed baseline retains its fix labels/types and
warning provenance and therefore produces `ESTIMATED`, not `VALID`; sub-3D input remains BLOCKED.
This is a gate/quality change only and does not migrate or reinterpret existing rows.

One transient run freezes its method configuration. A comparison callback from an older generation
may be appended with `late=true` and cannot mutate the current UI or current-run row sequence. A
primary outcome that arrives only after its run leaves `RUNNING` likewise cannot create a
`measurements.jsonl` row or update the distance UI.

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

Per-camera signed/raw SR-PC is retained as diagnostic clock evidence. Its absolute magnitude is not
a recording, STUS success, epoch-validity, Measurement, or pair gate. Compact camera rows label a
legacy SR-PC-only invalid marker as `valid_sr_pc_diagnostic`; other mapping-invalid reasons remain
invalid. Time-dependent Left/Right rows require matching logical clock epoch IDs and a frame delta
at or below 100 ms. Occasional unmatched/outlier candidates may be absent while their rejection/drop
evidence remains separate; the schema does not assert one output pair per source frame.

## Workstation live-distance timing schema v5

New live-distance rows freeze `pairing_time_basis_requested` from the Measurement boundary and
record the per-pair `pairing_time_basis` actually used. Canonical requested values are
`app_receive_monotonic` and `camera_rtcp`. `pairing_fallback_active` and
`pairing_fallback_reason` explain why an RTCP preference used app receive instead.

`selected_pair_delta_ns`/`sync_delta_ns` are signed Right minus Left in the effective basis;
`selected_pair_abs_delta_ns` owns the configured gate and 100 ms hard cap. Exact app readings and
`app_receive_pair_delta_ns` are always retained. `rtcp_pair_delta_ns`, raw SR NTP/identity, RTP
timestamp/SSRC/rates, SR-PC, Clock status/reason and clock epoch remain separately named. NTP uses
`ntp_evidence_role=selected_distance_pairing` only for a healthy RTCP-selected pair and
`record_only` for app-selected or fallback pairs.

Schema-v4 rows retain their historical fixed app-receive meaning and schema-v3 rows retain their
historical RTCP meaning. No existing row is rewritten or reinterpreted. Measurement-pairs Parquet
v4 adds typed requested/effective/fallback columns while lossless `row_json` preserves all fields.

Camera timeline rows store `media_recovery_basis`, `media_integrity_mode`,
`media_integrity_unverified`, `source_media_integrity_verified`, and
`media_random_access_guaranteed`. Measurement-pair rows store the same values with
`left_`/`right_` prefixes. Older Parquet schemas remain readable through their schema metadata and
lossless `row_json`; historical direct-copy MP4 records keep their original strict interpretation.

Later collection should primarily use `recorded_utc_ns`, producer receive/source timestamps, and
`runtime_generation`. Filename timestamps identify binary creation, not necessarily sensor exposure.

## Legacy data

Existing `recordings/<id>/` and `recordings/sessions/<id>/` hierarchies remain untouched. Their old
Session/package/manifests/Parquet/workbook contracts apply only to those historical trees. New code
does not scan, recover, seal, convert, or append to them; the Data workspace exposes only a file
manager link.
