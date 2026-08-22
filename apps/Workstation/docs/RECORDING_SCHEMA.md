# Workstation Recording Schema

## Purpose

This document defines the authoritative research-recording contract for
`apps/Workstation`. It applies to real WIRIS Enterprise, Pixhawk, GNSS/RTK,
RTCM, ULog, Target Drone, and native-frame evidence. It does not introduce
Gazebo ground truth or change triangulation, detection, correspondence,
calibration, RTK gates, NTRIP, PX4 EKF, or optical-pose solvers.

## Identity and ownership

The hierarchy is:

```text
Session <session_id>
└── MeasurementPackage <measurement_id = mNNNNNN>
    └── terminal MeasurementAttempt / blocked-pair records
```

- `session_id` identifies the parent experiment recording container.
- `measurement_id` continues to identify one `mNNNNNN` package. It is not an
  attempt ID.
- `measurement_package_id` is an exact compatibility alias of
  `measurement_id`.
- `terminal_record_id` is the stable terminal-row primary key:
  `attempt:<measurement_attempt_sha256>` or
  `blocked:<blocked_pair_sha256>`. Pre-contract legacy rows use a deterministic
  content-based `legacy:<sha256>` key that excludes row position.
- `geometry_context_id` joins a terminal row to its static geometry context.
- Camera rows may reference `time_sync_epoch_id`; terminal rows use exact
  `left_time_sync_epoch_id` and `right_time_sync_epoch_id` when resolvable.
  The singular ID remains `null` when the two side-specific mappings are not
  the same epoch.

## Physical layout

New writers use the direct session layout:

```text
.local/workstation_data/recordings/<session_id>/
├── session.json
├── measurements/
│   └── mNNNNNN/
│       ├── measurement.json -> .workstation/publish/current/measurement.json
│       ├── data -> .workstation/publish/current/data
│       ├── .spool/                         # retry source until publish verifies
│       └── .workstation/publish/
│           ├── current -> generations/<generation_id>
│           └── generations/<generation_id>/
│               ├── measurement.json
│               ├── generation_manifest.json
│               └── data/
│                   ├── raw/*.jsonl
│                   ├── context/*.jsonl
│                   ├── context.json
│                   └── *.parquet
├── review.xlsx                             # derived, regenerable
└── measurement_session_manifest.json       # sealed ordered child index
```

`recordings/sessions/<session_id>/` remains a legacy read/recovery boundary.
New evidence is never silently migrated into or out of that tree.

## Authority hierarchy

| Artifact | Authority | One record / byte meaning |
| --- | --- | --- |
| `data/raw/stereo_measurements.jsonl` | authoritative | one terminal matched-pair outcome, including failed/blocked outcomes |
| `data/raw/camera_left.jsonl`, `camera_right.jsonl` | authoritative | one GUI-delivered camera-frame observation |
| `data/raw/px4_left.jsonl`, `px4_right.jsonl` | authoritative | one received PX4 telemetry callback |
| `data/raw/px4_drone.jsonl` | authoritative when present | one Target/Drone PX4 telemetry callback |
| `data/raw/gnss_base.jsonl` | authoritative | one Base-GNSS callback |
| `data/raw/extrinsics_comparison.jsonl` | authoritative when present | one method-comparison observation |
| `data/context/geometry_contexts.jsonl` | authoritative | one unique static geometry/calibration context |
| `data/context/time_sync_epochs.jsonl` | authoritative | one observed WIRIS RTCP/NTP mapping interval or explicit invalid interval |
| WIRIS still, native PNG, MP4, `.ulg`, `.rtcm` | authoritative native bytes | exact producer/downloaded bytes; never base64-wrapped into JSON |
| `context.json`, manifests | authoritative static metadata/index | package identity, contracts, descriptors, and static provenance |
| `*.parquet` | derived | typed analysis projection retaining complete `row_json` |
| `review.xlsx` | derived | regenerable operator review |

Canonical JSONL is UTF-8, one object per line, `allow_nan=False`, and preserves
row order and all source/private fields. Each manifest descriptor uses
`role`, `media_type`, `relative_path`, `size_bytes`, `sha256`, `row_count`,
`schema_version`, `authoritative`, and `derived_from`. A Parquet
`derived_from` entry is the exact descriptor of its source JSONL fragment.

## Coordinate contracts

- Public measurement: `baseline_center_frd`, origin at the midpoint of the
  left/right optical centers, `[X,Y,Z] = [Front,Right,Down]`, metres.
- Camera optical: RDF `[Right,Down,Forward]`; pixels are `u` right, `v` down,
  pixels.
- PX4 local/world: NED `[North,East,Down]`, metres.
- PX4 body: FRD `[Front,Right,Down]`.
- Rotation notation: `R_B_A` means `v_B = R_B_A @ v_A`.
- Points use rotation plus translation. Ray directions use rotation only.
- PX4 attitude is Hamilton `quaternion_wxyz`, rotating PX4 body FRD components
  into world NED.

No reader may infer a frame from generic `x/y/z`. The canonical attempt keeps
the complete K/D/R/T, dynamic attitude/GPS/baseline, measurement geometry, and
the explicit frame/axis contracts that existed at capture.

## Geometry context contract

Method name: `workstation-geometry-context-v1`.

- Purpose: deduplicate and index static calibration/geometry semantics without
  replacing self-contained `MeasurementAttempt` evidence.
- Inputs: available left/right K and D, image dimensions, available/applied
  static R/T, calibration profile paths and hashes, source/operating zoom,
  method and method-instance IDs, component-source mapping, static baseline
  model/source contract, and public transform/output contracts.
- Excluded dynamic inputs: live attitude, live GPS, live RTK baseline values,
  and geometry hashes containing live components.
- Output: `gc:<canonical_context_sha256>` plus the canonical context object.
- Pass criteria: K, D, R, T, profile hash, resolution, method instance, or
  component-source semantic changes produce a new ID; attitude/GPS/live
  baseline value changes alone do not; every terminal reference resolves.

Profile paths inside this context are historical provenance only. Content
hashes and package-relative artifacts, not an old absolute path, are used to
reopen evidence.

## Time contract

Clock domains remain distinct:

- host `CLOCK_MONOTONIC` integer ns;
- host UTC/`CLOCK_REALTIME` integer ns;
- session elapsed time;
- WIRIS STUS/RTCP/NTP-derived time;
- RTP media timestamps;
- PX4 boot/time fields;
- MAVLink `time_usec`;
- GPS UTC.

Method name: `workstation-time-sync-epoch-v1`.

- Purpose: record factual WIRIS RTCP/NTP mapping history.
- Inputs: CameraPane `stream_generation`, observed RTP SSRC, RTCP Sender
  Report fields, explicit RTCP invalid reason, exact host receive clocks, and
  a hash reference to a successful stored STUS result when available.
- Output: `tse:<identity_sha256>` epochs and exact side-specific row links.
- Pass criteria: observed SSRC/generation changes, invalidation, and
  revalidation create deterministic epochs; missing mapping evidence creates
  no epoch and leaves the link `null`.

`source_utc_ns` derived from float seconds is explicitly labelled as a
Decimal/rounding derivation and is never represented as an original camera
integer clock reading. STUS request UTC and camera exposure UTC are different
semantics. Camera exposure UTC remains absent unless the camera supplies such
evidence.

## Native and binary provenance

- WIRIS FTP stills keep the existing fresh-file selection. Downloads use a
  same-directory temporary file, file fsync, atomic rename, directory fsync,
  and post-rename size/SHA-256 verification. Records include remote path/name,
  session-relative local path, side, capture ID, purpose, request clocks, and
  factual zoom readback when available. The absolute local path is historical
  provenance only.
- Native PNG keeps decoded native RGB pixels, lossless PNG, original width and
  height, no overlay, no resize, and no geometry conversion.
- ULog remains exact validated `.ulg` bytes. It is not replaced by JSON.
- RTCM remains exact binary transport/frame bytes plus the offset/hash index.
- Target Drone remains raw MAVLink JSONL. No simulation truth is added.
- Secrets, FTP passwords, and NTRIP passwords are never stored. A credential
  environment-variable name may be retained.

## Crash recovery and publication

Finalization order is: close/fsync spool, materialize rows, write and reread
canonical JSONL, verify bytes/count/SHA, create and verify derived Parquet,
write manifests/completeness, atomically replace `current`, validate through
the published pointer, and only then discard the spool.

JSONL write/validation, Parquet, manifest, or publish failure preserves the
spool and the previous good generation. A post-publish validation failure
atomically restores the previous pointer. Direct ledgers quarantine only a
torn final record; a malformed middle record is a hard failure and is never
silently skipped.

## Reader and export behavior

Python analysis should call:

```python
from Workstation.workstation_core.recording_reader import read_measurement_stream

stream = read_measurement_stream(measurement_root, "raw/stereo_measurements")
rows = stream.rows
```

For storage-v3 packages the reader verifies and reads JSONL first. It never
falls back to Parquet when a declared authoritative JSONL is missing or
corrupt. Parquet-only legacy packages use an explicit compatibility fallback.
MATLAB can read each JSONL line independently with `jsondecode`; integer ns
values must remain integer-capable values rather than IEEE-754 identifiers.

`review.xlsx` is a derived view and its failure cannot invalidate a published
generation. The parent Session manifest stores child order, current generation,
authoritative JSONL/context hashes, binary inventory, and child manifest hash;
it does not concatenate a second authoritative session-level JSONL.

## MP4 mapping limitation

Camera JSONL rows are GUI-delivered observations. MP4 may be encoded
pass-through or decoded-frame recording. There is no proven exact 1:1 mapping,
so `mp4_frame_index_contract` is `not_available`. No reader or manifest may
claim an exact camera-JSONL/MP4 frame index.

## Backward compatibility

- New storage-v3 packages: verified JSONL authority plus derived Parquet.
- Previous direct packages: Parquet-only reader fallback.
- Legacy `recordings/sessions/`: explicit read/recovery fallback.
- Existing `measurement_attempt_id`, `measurement_attempt_session_id`,
  `measurement_attempt_sha256`, and `blocked_pair_sha256` remain unchanged.
- Legacy artifacts are not automatically migrated or rewritten.
