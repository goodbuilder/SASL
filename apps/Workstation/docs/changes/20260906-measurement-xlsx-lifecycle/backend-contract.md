# Backend Contract

## Lifecycle

`MeasurementRunController.begin_measurement_run` mints the generation, then calls
`MeasurementRegistry.begin(measurement_id, started_utc_ns, started_monotonic_ns, metadata,
calibration)`; the registry allocates the workbook path and returns the record. Start never waits
on storage; allocation failures become a `STORAGE_FAILURE` event on the record.
`finish_measurement_run` sets IDLE first, then `registry.stop(...)`. A record is `RUNNING` →
`STOPPED_DRAINING` (submitted methods still pending) → `STOPPED`; it stays in memory until every
outcome arrived and the last workbook write confirmed its revision (`registry.tick()`), or until
shutdown (`finalize_all` marks pending methods `FAILED` and writes every record).

## Method execution

`MeasurementViewController._plan_method_evaluations(pair)` returns the requests for ready
methods, `not_attempted: {method_id: reason}` and per-method status evidence. `register_pair`
records the pair with `Attempted=YES/PENDING` or `NO/N/A` rows for all 9 methods.
`MethodEvaluationCoordinator.submit(requests, planned_methods=...)` accepts any canonical-order
subset. `poll()` returns `outcomes` (individual) and `completed_bundles` (subset bundles, JSONL
row per pair). `route_outcome` updates the record and requests a workbook rewrite.

## Canonical transform

Every method result stores `R11..R33`, `T_R, T_D, T_F` such that
`X_right = R_right_from_left_rdf · X_left + T_right_from_left_rdf_m` in OpenCV optical RDF.
Kernel inputs: `R_left_from_right = Rᵀ`, `C_right_in_left = -Rᵀ T` (`canonical_transform.py`,
identical arithmetic to `triangulation.invert_left_to_right_camera_transform`). Public result is
`baseline_center_frd [Forward, Right, Down, Range]` metres.

## Workbook sheets

| Sheet | Rows | Key columns |
| --- | --- | --- |
| `00_Summary` | key/values + 1 per method | Start/Stop local, UTC ns, monotonic ns, duration, pair count; per method Attempted / VALID / ESTIMATED / BLOCKED / FAILED / BACKPRESSURE_BLOCKED / PENDING / N/A counts, Forward/Right/Down/Range min/max (numeric rows only), hyperlink to the method sheet |
| `01_Left_Detections`, `02_Right_Detections` | 1 per detector box | measurement_id, pair_id, pair_sequence, frame_id, frame_key, detection_id, selected, timestamp_s, app_receive_monotonic_ns (text), x1 y1 x2 y2, confidence, class_id, class_name, image size, stream_generation, rtcp_frame_ntp_unix_s |
| `03_Calibration` | 1 per side | profile_sha256, source_path, image size, K11..K33, distortion_model, D_count, D1..D8, reprojection_error_px, image_count |
| `04_Alignment` | 1 per applied R,t / alignment event | timestamp_utc_ns, event_type, method_id, source, R11..R33, T_R T_D T_F, source_sha256, detail |
| `10_Pixhawk_RTK` … `18_Pixhawk_RP_GPS_Yaw` | sections A–E, 1 row per pair | A: identity, time, Attempted, R, T, Forward/Right/Down/Range, Status, Reason, time_delta_s, worker times · B: selected L/R detection (id, bbox, confidence, class) · C: L/R u,v and unit rays (R,D,F) · D: long-format `field, value, source, unit_or_frame` of every scalar behind R,t (attitudes, GNSS, baseline, yaw branch, applied extrinsics, intrinsics provenance, geometry config, component sources, candidate status/quality) · E: ray_gap_m, ray_angle_deg, left/right_depth_m, triangulation_reason, warnings |
| `20..24_PX4L_*`, `30..34_PX4R_*` | 1 per consumed sample | Attitude (`pixhawk_attitude`), GNSS_RTK (`gps_rtk`, `sensor_gps`, `gnss_relative`), IMU_Mag (`magnetometer`, `mavlink_raw_imu`, `mag_slot_status`), Position (`mavlink_local_position_ned`, `mavlink_global_position_int`, `mavlink_vfr_hud`), System (`mavlink_sys_status`, `mavlink_battery_status`, `mavlink_radio_status`); identity/time columns first, producer fields, MAVLink/link provenance last; `*_ns`/`*_us` as text |
| `40_Drone_Attitude`, `41_Drone_GNSS_RTK`, `42_Drone_Position` (+ `44_Drone_IMU_Mag`, `45_Drone_System` when present) | same routing for `px4.drone` | |
| `43_Base_GNSS` | Base NAV-PVT rows | `46_GNSS_Baseline_Center` when derived P1/P2/Drone rows exist |
| `50_Events` | lifecycle/storage | MEASUREMENT_STARTED/STOPPED, STORAGE_FAILURE/RECOVERED, METHOD_FAILED, APPLICATION_SHUTDOWN, plus non-lifecycle `events` stream rows |
| `90_Flat` | pair × 9 methods | identity, Attempted, Status, selected bboxes, u,v, R, T, results, diagnostics, time_delta_s, Reason |
| `91_Exact` | 1 per float | pair_id, method_id, field, value, repr, `float.hex()` |
| `98_Provenance`, `99_Definitions` | key/value, term/definition | |

Values: literals only (no formulas), `N/A` for missing, `YES`/`NO` for booleans, integers with
`_ns`/`_us`/`time_boot_ms` suffix or ≥ 2^53 as text. Header row bold, frozen, autofilter.

## Writer

`MeasurementWorkbookWriter` (one daemon thread) coalesces requests per record; RUNNING records are
rewritten at most every 30 s (checkpoint), Stop/late outcomes/shutdown immediately. Each write uses
`record.snapshot()` (copy under the record lock) and publishes atomically. Result is recorded on the
record (`storage` state, `STORAGE_FAILURE`/`STORAGE_RECOVERED` events) and in
`DataStorageController.measurement_workbook_snapshot()`.

## Storage independence

`MeasurementDataStore.add_enqueue_observer` delivers every producer record to
`MeasurementRegistry.observe_store_record` before JSONL admission. Workbook failures never change
JSONL, MP4, ULog or RTCM behaviour and never gate Measurement Start/Stop.
