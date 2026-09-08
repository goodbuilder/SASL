# Workstation Measurement Data Schema

## Target geographic result fields (2026-09-08)

| Field | Meaning | Type |
| --- | --- | --- |
| `target_latitude_deg` | WGS84 geodetic latitude | nullable float64, degrees |
| `target_longitude_deg` | WGS84 geodetic longitude | nullable float64, degrees |
| `target_altitude_ellipsoid_m` | WGS84 ellipsoid height | nullable float64, metres |
| `target_altitude_msl_m` | Mean-sea-level height; null without a target geoid model | nullable float64, metres |

The fields appear in worker result payloads, `recorded_geometry`, primary measurement
rows (including retry), method results, compact measurement projections and Parquet,
and workbook `02_좌표결과`, method [A], `90_Flat`, `91_Exact` and method comparison output.
Envelope CSV mirrors retain the same JSON payload. Geographic degrees display nine
decimal places in XLSX; numeric values are not rounded before persistence.

`target_coordinate_evidence` (`target-geolocation-v1`) retains the P1 GNSS geodetic
origin, the method's frozen world rotation, source point in left optical RDF [R,D,F],
and derived local NED [N,E,D]. Pixhawk's selected geometry quaternion takes precedence
over raw attitude (including baseline-yaw90). Candidate methods retain their resolved
left optical-to-NED matrix in `measurement_pose_evidence_json` with
`target_origin_model=P1_GNSS_as_left_optical_center`. Their local pose translation is
rebased to the pair's P1 GNSS proxy, not added a second time. Relative profiles lacking
this world anchor remain null. The Pixhawk/optical-center offset is assumed zero;
GNSS antenna offset is still the existing unmodeled proxy, and body/optical mounting
rotation is applied. No new geolocation quality gate blocks existing FRD results.

Conversion uses left optical RDF → world NED → ECEF → WGS84. Ellipsoid height is
required; MSL is never silently substituted. No constant-geoid approximation is used.
The height/Cartesian distinction follows the
[GeographicLib local Cartesian definition](https://geographiclib.sourceforge.io/2009-03/classGeographicLib_1_1LocalCartesian.html).

New fields are additive. Old results remain readable with missing target fields,
without backfilling from live state. Existing canonical attempt payloads stay unchanged
unless new pose evidence was actually captured. The outcome's nested target evidence
has its own version. Journal rebuild copies `recorded_geometry` without recalculation;
explicit `reproduce_target_coordinates_exact(path)` reconstructs rays from workbook
K,D,R,T,u,v and uses the stored world anchor to check geolocation independently of
the saved target point. No hardware changes or original recording migration is needed.

## Purpose

### Mountain/Cloud Yaw90 reference

`yaw90_reference` (`optical-yaw90-reference-v1`) records selected GPS1/GPS2 positions,
P1 Roll/Pitch, P1/P2 branch yaw references, request UTC/monotonic timestamps, sample ages,
Reverse, baseline/selection, assumptions, state/reasons/warnings and SHA-256. The capture
event binds this snapshot to both frame identities. Request time is not exposure time.
Candidate `left_pose` is assumed; `right_pose` is image-derived relative to that reference.
Each contains optical RDF→NED rotation, virtual-FRD RPY degrees and local NED center metres.
OpenCV translation remains `t=-Q_R^T*(C_R-C_L)`.

Optional `measurement_pose_evidence_json` in MeasurementAttempt embeds the reference and
actual pair-used left/right poses and R,t, covered by the canonical checksum. Cloud keeps
capture rotations and pair GNSS positions; Mountain keeps capture positions. Empty legacy
evidence is omitted to preserve old hashes. CSV `record_json` retains all fields; XLSX
method section D expands `optical_pose.*` with units. Raw telemetry attitude fields remain
unchanged. Existing records are not migrated or reclassified.

This document defines the sessionless recording contract for `apps/Workstation`. It covers real
WIRIS, Pixhawk, GNSS/RTK, RTCM, ULog, Target Drone, native-frame, capture, Alignment, and
Measurement records. Storage observability is part of the contract; storage success is never a
Measurement admission condition.

## Ownership and identity

A Measurement is an observation interval. Its persisted start context locates evidence and
supports read-only reconstruction; it never restores active device commands after restart.
`measurement_id`/`runtime_generation` identify the run. One accepted Start request freezes
`measurement_started_utc_ns`, `measurement_started_monotonic_ns` and `measurement_file_stamp`.
Each record also retains its own UTC, monotonic and source timestamps.

For new Measurement-owned records, nested source identity fields are preserved. Legacy
membership fields at the top level are retained as `source_session_membership`, rather than
creating an operational Session. Outside-run legacy streams retain their previous filtering.

## Physical layout

An empty `measurement_data_root` selects `<SASL_ROOT>/recordings`, or the `recordings/` child
of an explicit `SASL_WORKSTATION_DATA_ROOT`. A custom value is relative to that data root.
The KST start stamp is `YYYYMMDD_HHMMSS_nnnnnnnnn+0900`; a whole-run collision adds `-01`, etc.
All files owned by the run contain exactly that stamp, regardless of when they finish writing.

```text
<resolved data root>/<stamp>/
  <stamp>_measurement.xlsx
  raw/<stamp>_<stream>.jsonl
  raw/<stamp>_<stream>.csv
  raw/<stamp>_measurement_journal.jsonl
  raw/<stamp>_measurement_journal.csv
  raw/gnss/<stamp>_<source>_<kind>.rtcm
  raw/gnss/<stamp>_<source>_<kind>.rtcm.jsonl
  raw/gnss/<stamp>_<source>_<kind>.rtcm.csv
  media/<stamp>_left.mp4
  media/<stamp>_right.mp4
  frames/<purpose>/<stamp>_<pair>-<side>.png
  images/hr_imported/<stamp>_<side>_<original-stem>.<extension>
  ulog/<source>/<stamp>_<original-stem>.ulg
  ulog/<source>/<stamp>_<original-stem>.ulg.<artifact-id>.json
```

Video reconnect segments and repeated captures receive unique suffixes. PNG/ULog artifact
metadata goes through the dual-format raw store; RTCM's existing binary/index owner also
mirrors its index to CSV. Native binary contents are not converted to JSON. Reusable app
settings/assets and operations outside Measurement retain their existing ownership rules.
No new-run folder is moved at shutdown, and existing histories are never renamed or backfilled
as a migration. An explicit CSV repair may replace only the derived mirror of its existing JSONL.

## Measurement journal and CSV

Each logical stream has UTF-8 JSONL and a same-stem UTF-8 CSV using
`workstation-measurement-csv-v1`: common clock/identity/status columns plus the complete canonical
object in `record_json`. Payloads, nested values, large integers and long strings are retained.
The existing thin numeric projections keep their producer formatting; full-precision frozen
inputs/outcomes and `recorded_geometry` are independently retained in the journal.

`measurement.journal` records start metadata/calibration, detection candidates, pair decisions,
method outcomes with frozen geometry, calibration/alignment updates, events, pending-job failures
and Stop. `journal_sequence` provides ordering, gap detection and idempotent replay. Rebuild uses
these deltas plus same-run telemetry/artifacts; it does not consult current settings or sensors.
A missing JSONL can be read from its CSV mirror. Torn/corrupt input remains untouched and is
reported as incomplete. An interrupted run is never silently presented as a live run.

## Measurement XLSX v2

`<stamp>_measurement.xlsx` is updated atomically by the background writer at Start/checkpoints,
Stop and late completion. Its name never changes on rebuild. `Open Latest Measurement XLSX`
selects start time plus collision sequence, not the modification time of a late older export.
`Rebuild Latest XLSX` reads saved inputs in a background thread after recording/calculations stop.

The first six sheets are `00_요약`, `01_거리비교`, `02_좌표결과`, `03_진단`, `04_Rt`, `05_그래프`.
Existing Method sheets, detections, calibration, alignment, optical-pose, sensors, events,
`90_Flat`, `91_Exact`, `98_Provenance` and `99_Definitions` remain available (40 base sheets).
Internal analysis and unused Method/telemetry sheets are normally hidden. Every displayed
Method table shows results first and all nine R plus three t components expanded, with a
stable `rt_id` linking to the same pair/method in `04_Rt`. R and t sources and their fixed/changed
summaries are separate. Available transforms from ESTIMATED/BLOCKED/FAILED results are retained.

Coordinates remain `baseline_center_frd`; the canonical transform is
`X_right = R_right_from_left * X_left + T_right_from_left`, with both camera coordinates in
optical RDF and T in right-camera RDF metres. T is not the camera-center vector `C=-R^T T`.
R/t/coordinate values stay numeric and unrounded; only display formats change. Nanosecond and
large exact integer values remain text. Reader missing markers are `—`, while reproduction
sheets retain `N/A`. Tables use numbered continuation sheets at Excel's row limit, and exact
reproduction reads all parts. Compact report widths, 11pt font, filters and frozen identifiers
support reading; wide R,t/Method tables permit horizontal scrolling.

`05_그래프` contains twelve native XY charts in two columns, with four charts per printed
landscape A4 page. Their Y fields are, in order:

1. `forward_m`, `right_m`, `down_m`, `range_m`: existing camera midpoint FRD, metres.
2. `target_level_forward_m`, `target_level_right_m`, `target_level_down_m`,
   `target_level_range_m`: existing Method midpoint / pair-frozen P1 horizontal FRD, metres.
3. `target_latitude_deg`, `target_longitude_deg`, `target_altitude_ellipsoid_m`,
   `target_altitude_msl_m`: geodetic degrees and separate ellipsoid/MSL heights in metres.

X is `(pair.created_monotonic_ns - started_monotonic_ns) / 1e9`, in seconds. Each Method's
style is stable across fields. Only finite values from `VALID`/`ESTIMATED` rows are drawn;
missing coordinates and other statuses remain blank gaps. Single results use a marker;
fields with no finite result keep a `데이터 없음` chart without numeric axes. The summary
links to this sheet. Native series reference hidden provenance ranges, with hidden-data
display enabled. Only chart backing ranges may sample up to 2,000 points per Method per
field, retaining bucket minima/maxima and gap markers. Full result tables, exact numbers
and journal reconstruction are unchanged; historical missing fields are not calculated
during export or rebuild.

## Logical stream contract

| Stream key | File | Priority | Meaning |
| --- | --- | --- | --- |
| `events` | `events.jsonl` | yes | transient run, command, and producer events |
| `measurements` | `measurements.jsonl` | yes | representative `pixhawk_rtk_live` `VALID`/`ESTIMATED` terminal results accepted while the run is `RUNNING` |
| `method_evaluations` | `method_evaluations.jsonl` | yes | atomic nine-method inputs/outcomes and late-result markers |
| `method_results` | `method_results.jsonl` | yes | one thin row per method outcome (status, reasons, F/R/D/Range, ray diagnostics, disparity, timing) — the readable projection of `method_evaluations` |
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
| `measurement.journal` | `measurement_journal.jsonl` | yes | complete per-run reconstruction deltas and frozen Method geometry |

Table filenames above are logical/legacy stems. New run files use `raw/<stamp>_<stem>.jsonl` and `.csv`.

JSONL is UTF-8, one JSON object per line, `allow_nan=False`. Unsupported/non-finite values reject
only that record. A malformed final historical fragment is retained as one bad line; the writer
adds a newline and continues. Historical bytes are never rewritten. CSV uses standard RFC-style
quoting and copies coordinate/frame and clock fields without conversion.

`LiveDataRows.replace(index, row)` does not edit the file. It appends a new record carrying
`_record_operation="replace"` and the same `_record_index`. Readers that want the current-run view
apply later replace records over earlier append records. Readers that need audit history retain both.
`_record_index` is allocated monotonically in the payload's actual destination file. Each logical
row retains its allocation for replacements/retries. A late result delivered through a newer row
collection allocates in the original run, preventing cross-run index collisions.

Numeric row fields are written as JSON numbers (rounded to six decimals; `null` when absent), not
fixed-precision strings; boolean flags stay `"true"`/`"false"` text. `application_log` rows carry an
inferred `level` (`INFO`/`WARNING`/`ERROR`) classified from the message text.

### Readable projections on the `measurements` row

Once a pair's method bundle has been evaluated, the pair's `measurements` row is re-written
(`_record_operation="replace"`) with:

- `m_<method>_status/reason/forward_m/right_m/down_m/range_m/ray_gap_m/ray_angle_deg` for every
  evaluated method (`m_rtk_*` for `pixhawk_rtk_live`, `m_yaw90_*` for
  `pixhawk_rp_gps_baseline_yaw`, otherwise the method id), `m_method_ids`, and
  `m_yaw90_over_rtk_range_ratio`.
- every `measurement_yaw_*` selection field taken from the baseline-yaw attempt, whichever method is
  primary (`measurement_yaw_fields_source` names the source method).
- `optical_relative_*`: the latest accepted optical-pose suggestion's far-field relative
  yaw/pitch/roll, fit inlier count/residual, its age at the pair, `pixhawk_relative_yaw_deg`
  (P2 minus P1 Pixhawk yaw) and `optical_minus_pixhawk_relative_yaw_deg`.

`geometry_comparisons` no longer records placeholder rows whose status starts with
`waiting for inputs`; a method with no inputs simply has no row for that pair.

### Session archive on shutdown

The final lifecycle step (`archive-session-recordings`) runs after the data writer is closed. It
moves that session's token-stamped stream files, the `frames/` and `raw/` trees, and run artifacts
whose `YYYYMMDD_HHMMSS` prefix is not earlier than the session token into
`<root>/<YYYYMMDD_HHMMSS>/` (run artifacts under `runs/<prefix>/`). Native-frame index rows keep
resolving because their `relative_path` is relative to that folder. Earlier sessions' files and
`latest_measurement_*.xlsx` are left in place.

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

One bounded daemon writer owns logical JSONL/CSV streams across run-specific destinations.
Closed-run handles are flushed and released; immutable routes can reopen for late appends.
The RTCM binary/index owner remains separate and serializes CSV repair with its file writes.

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
  CSV retry rebuilds from the durable JSONL, including rows that overflowed its retry reserve.
  `Retry Storage` also checks stopped-run RTCM mirrors off the GUI thread.
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
# Mountain/Cloud availability policy (2026-09-08)

Newly solved candidates carry `gate_policy=mountain-cloud-availability-v1` in provenance
and warnings. Mountain translation uses capture-associated GNSS NED (right minus left);
Cloud measurement translation uses the existing live/retained GNSS provider. Relative
rotation remains OpenCV optical RDF right-from-left; no coordinate transform changes.

Method outcome records add `usable` (VALID/ESTIMATED), `blocking_reasons` (alias of reasons),
`quality_warnings` (alias of warnings, including provenance warnings), and `gate_policy`.
These are derived projections outside the existing canonical checksum; authoritative reasons,
warnings, result and attempt remain checksummed. Absent policy markers serialize as
`legacy_or_unspecified`, never as retroactive approval. Existing CSV/XLSX status and warning
columns retain ESTIMATED results. Historical records are not overwritten or reclassified.

### Additive method-midpoint horizontal target coordinates

New Measurement outcomes, representative results and per-method result rows add
`target_level_forward_m`, `target_level_right_m`, `target_level_down_m`, and
`target_level_range_m` (metres). `target_level_frame=baseline_center_local_level_frd`.
Existing optical `baseline_center_frd` and WGS84 fields retain their meanings;
no ECEF output is added. JSONL and its CSV mirror preserve the new values, and
XLSX coordinate, Method, comparison and flat tables expose them beside existing
results. Older records remain unknown rather than being recalculated on read.

The origin is **each method's existing midpoint**, not a replacement shared GNSS
midpoint. For relative-pose methods the right optical origin is `-R.T @ T` from
the exact left-to-right optical RDF transform used in triangulation. Other
methods use their frozen `translation_right_in_left_rdf_m`. Subtract half that
origin from `source_point_left_rdf_m`, convert RDF to body FRD, then use the raw
pair-frozen P1 attitude quaternion to rotate into NED. Forward is P1 body forward
projected onto the NED horizontal plane; Right is horizontal right, Down is NED
Down. All methods in a pair use that common attitude, never a method-specific
geometry quaternion for levelling. Camera and Pixhawk mounting rotation offsets
are assumed zero. Range is the norm from the preserved midpoint and agrees with
the existing unsmoothed geometric Range up to floating-point roundoff.

`target_level_status` is `AVAILABLE` or `UNAVAILABLE`; `target_level_reason`
explains unavailable values. Missing/nonfinite geometry, missing/invalid attitude,
a zero baseline, or a forward projection below 1e-8 produces null extra numeric
fields without blocking the original measurement. Availability is a mathematical
calculation status, not an attitude accuracy/freshness certification.
`target_level_evidence` in canonical outcomes, representative records and journal
contains the source left RDF point, midpoint in left RDF, normalized quaternion,
attitude receive timestamp/age, detection midpoint time, axis definitions, mount
assumption and baseline source. The existing attitude reuse/quality warnings
remain authoritative. Absolute GNSS position is not required for this addition.
The separate GNSS antenna-midpoint-to-Drone stream retains its own coordinate
contract and is not substituted for these camera-derived target results.
