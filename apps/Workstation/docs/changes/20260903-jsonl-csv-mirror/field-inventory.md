# JSONL Field and Change Audit

## Audit basis

This audit uses the current `MeasurementDataStore.STREAM_PATHS` contract and a real recent
sessionless recording under `.local/workstation_data/recordings/0903 1421 헬기 접근/`. Counts below
are representative top-level fields/rows from that run, not a promise that optional producer fields
will never be added. The store adds `_record_index`, `_record_operation`, `schema`,
`recorded_utc_ns`, and `recorded_monotonic_ns` where applicable.

The source of truth for the write behavior is
`workstation_core/measurement_data_store.py`: each accepted payload is normalized and serialized as
a complete JSON object. There is no previous-row comparison or delta encoder.

## File-by-file audit

| File | Real sample | Stored value families | Importance | Change behavior / quick-find judgment |
| --- | ---: | --- | --- | --- |
| `events.jsonl` | 222 rows / 18 fields | run start/stop, recording mode, runtime generation, source ID/state, observed/recorded clocks, detail, camera-side state | Critical for chronology/recovery | Event-driven, not delta-based. Keep every event; filter by `event_type`, `source_id`, `state`, `recorded_utc_ns`. |
| `measurements.jsonl` | 88 rows / 562 fields | primary result, validity/quality, distance/range/disparity, pair timing, both bbox pairs, camera timing, P1/P2 attitude/GPS, baseline frames, K/D/R,t identities, native evidence, hashes/warnings | Critical; primary analysis record | Full result snapshot per pair plus append/replace records. 294 fields constant and 268 varied in this run; no change-only behavior. A single wide CSV is hard to search. |
| `method_evaluations.jsonl` | 45 rows / 15 top-level fields | pair identity, primary method/result hash, full `outcomes` list, full `snapshot_payload`, bundle/snapshot hashes, request generation, clocks | High for method comparison/audit | One complete bundle per pair; nested values change every bundle. Filter by `pair_id`, `pair_sequence`, then inspect `outcomes`. |
| `camera_left.jsonl` | 541 rows / 58 fields | frame sequence, sample/session/host clocks, image size/axis contract, RTP/RTCP timestamps/rates/ages/offsets, RTSP transport, integrity/recovery flags | High for sync/media diagnosis | One row per GUI-delivered frame. 24 fields stayed constant and 34 changed; static contracts repeat. Put frame/time/RTP/RTCP/integrity first. |
| `camera_right.jsonl` | 553 rows / 58 fields | same families as Left, with `side=right` | High for sync/media diagnosis | One row per GUI-delivered frame. 26 constant and 32 changing. Keep the full timeline; do not store only changes. |
| `px4_left.jsonl` | 844 rows / 157 fields | heterogeneous `sample_type`: attitude, GPS/RTK, GNSS-relative, magnetometer/IMU, radio, local/global position, SYS/VFR; endpoint/sysid/source clocks and contracts | Critical for P1 state/geometry | One row per received telemetry callback. 61 constant and 96 changing; many fields are sparse because types share one file. Split quick views by `sample_type`/`topic`. |
| `px4_right.jsonl` | 1,040 rows / 149 fields | same families as P2 | Critical for P2 state/geometry | One row per received telemetry callback. 58 constant and 91 changing. Mixed wide layout is the largest quick-find problem after measurements. |
| `px4_drone.jsonl` | 1,375 rows / 134 fields | target attitude, GPS/RTK, GNSS-relative, magnetometer/IMU, radio, local position, SYS/VFR; target link/source provenance | Critical for target/flight evidence | One row per received callback. 63 constant and 71 changing; eight sample types/topics were present. Split by `sample_type`. |
| `gnss_base.jsonl` | not present in this run; code path exists | Base NAV-PVT latitude/longitude, ellipsoid/MSL height, height type, fixed flag, device ID, operation state, sample/session/host clocks | High when Base is used | One row per Base position sample, not delta. Position and fix values remain a time series. |
| `gnss_baseline_center.jsonl` | 89 rows / 110 fields | P1/P2/Drone positions and fix/source metadata, NED baseline, midpoint-to-Drone NED, local-level F/R/D/Rg, age/spread, coordinate contracts | Critical for GNSS baseline verification | One row per primary Drone GNSS callback, including WAITING/STALE. 65 fields constant and 45 changed; retain every row. |
| `geometry_comparisons.jsonl` | 132 rows / 21 fields | pair sequence/time, method, right/down/forward/range, source state, status, active profile hash, analysis namespace | Medium; analysis-only | Snapshot once per accepted pair/method view; repeated `waiting for inputs` rows are meaningful until policy says otherwise. |
| `capture_events.jsonl` | not present in this run; code path exists | HR/OZ request/result, event/capture ID, purpose, camera states, zoom, storage, FTP flag, paths/hashes, record hash | High for capture provenance | Event-driven; preserve every request/result including partial/failure. Search by `capture_id`, `purpose`, `state`. |
| `alignment.jsonl` | not present in this run; code path exists | reset/update events, selection/profile IDs/hashes, update time, nested active pose snapshot and provenance | Critical for applied R,t | Append-only state transitions; repeated values can prove re-application/reset history. Preserve all events. |
| `artifacts.jsonl` | 6 rows / 33 fields | video allocation/close, side/path, size/presence, finalization/integrity states, encoding mode, hashes/reasons, generation | High for file existence/integrity | Lifecycle events, not delta. Search by `event_type`, `side`, `path`, `recording_usable`, `integrity_validation_status`. |
| `application_log.jsonl` | 769 rows / 5 fields | message, source, recorded UTC/monotonic clocks, schema | Medium for troubleshooting | Every log message is stored; only `schema`/`source` stayed constant. Put time/source/message first. |

## Heterogeneous PX4 value families

The P1/P2/Drone files intentionally combine callback types. The first fields in quick views should
be:

- `pixhawk_attitude`: `roll_deg`, `pitch_deg`, `yaw_deg`, `q_w/q_x/q_y/q_z`, angular rates,
  PX4 time, receive time, source system/component, sequence, age.
- `gps_rtk` / `sensor_gps`: latitude/longitude, MSL/ellipsoid altitude, `fix_type`, RTK label,
  satellites, EPH/EPV, ground speed/heading, GPS/PX4/source time and validity.
- `gnss_relative`: relative-position validity, baseline length/accuracy, carrier fixed/floating,
  differential/moving-base flags, heading and source provenance.
- `magnetometer`: sensor slot/device, `mag_x/y/z`, vector norm, temperature, all SCALED_IMU/2/3
  acceleration/gyro values and boot times.
- `mavlink_local_position_ned`: local N/E/D position and velocity, MAVLink message/source/time.
- `mavlink_global_position_int`: global lat/lon/altitude/heading/velocity and MAVLink provenance.
- `mavlink_radio_status`: RSSI/noise, remote RSSI/noise, TX buffer, RX errors and radio fixed flag.
- `mavlink_sys_status` / `mavlink_vfr_hud`: battery/communication status, altitude, climb, ground
  speed, airspeed, heading and throttle.

## What is and is not change-only

The current implementation is **full-snapshot append**, not change-only:

1. `enqueue()` copies the complete producer mapping, adds timestamps/schema, and JSON-serializes the
   whole object.
2. `LiveDataRows.replace()` appends another complete record with the same `_record_index` and
   `_record_operation="replace"`; it does not write a field delta.
3. Camera/PX4/GNSS records are emitted once per frame/telemetry callback, even if many contract or
   status fields are unchanged.
4. Event/artifact/alignment streams are event-based, but that is not a previous-value comparison.

## Measurement pair semantics

One logical left/right detection pair should be one row in a human-facing current-state table, but
the current JSONL deliberately keeps an append-only audit history. The base row starts with
`measurement_valid="false"` and `pair_status="matched_geometry_pending"`; a later complete result
can append a `replace` row with the same `_record_index`. In a real sample, 45 rows were `append`
and 43 were `replace`, while 88 logical pairs were measured.

Therefore:

- pair existence belongs to `pair_sequence`/`measurement_detection_pair_key` and the row's lifecycle;
- `measurement_valid` describes result validity, not pair existence;
- `measurement_validity` (`VALID`, `ESTIMATED`, `INVALID`) and `quality` describe result quality;
- blocked/unresolved pairs must remain representable even when no distance is available;
- the quick CSV should materialize one latest row per pair, while raw JSONL audit retains append and
  replace history.

For the compact measurement CSV, the recommended columns are:
`run_id`, `pair_id`, `pair_sequence`, `time_utc`, `method_id`, `status`, `quality`, `distance_m`,
`baseline_center_forward_m`, `baseline_center_right_m`, `baseline_center_down_m`,
`baseline_center_range_m`, `disparity_px`, `left_bbox_center_x_px`, `left_bbox_center_y_px`,
`right_bbox_center_x_px`, `right_bbox_center_y_px`, `pairing_time_basis`, and
`selected_pair_abs_delta_ms` — 19 columns total. An optional `status_reason` makes 20 columns for
quickly explaining blocked/failed rows. `measurement_valid` is intentionally omitted because `status`
already expresses result usability and pair existence is represented by `pair_sequence`.

## Multiple methods on one pair

Use one long-format CSV rather than one file per method. Shared columns identify the pair and timing;
`method_id` identifies the result row. A two-method example is:

```text
pair_sequence,pair_id,method_id,status,quality,distance_m,...
12,pair-12,pixhawk_rtk_live,ESTIMATED,GOOD,65.43,...
12,pair-12,checkerboard_stereo,VALID,GOOD,65.18,...
```

This is easier to filter, compare, and extend to eight methods. A failed method remains visible as a
row with status/reason, not as a missing pair. The primary `measurements.jsonl` stream and the
method-evaluation bundle remain separate authoritative owners until a later product decision changes
that contract.

## CSV design consequence

The safest quick-find design is layered:

1. A lossless per-stream raw CSV mirror, one row per JSONL object, with stable envelope columns and
   the complete `record_json` value.
2. Human-facing quick-view CSVs for measurements, camera timing, GNSS baseline center, and each PX4
   `sample_type`, with important changing fields ordered first.
3. A small catalog/index identifying importance, row type, latest row, row count, and quick-view file.

Change-only output should be an additional state-transition summary, not a replacement for the full
camera/PX4/GNSS/measurement time series needed for pairing, trajectory, attitude, and failure
analysis.

The `.rtcm.jsonl` files under `raw/gnss/` are a separate binary/index owner, not one of the fixed
16 `MeasurementDataStore` streams. They contain binary offset, size, hash, source, message type, and
receive-clock metadata; including them in CSV requires a separate paired-index contract.
