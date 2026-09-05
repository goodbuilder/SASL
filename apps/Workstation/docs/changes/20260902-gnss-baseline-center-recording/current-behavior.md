# Current Behavior

## Version and worktree context

- Baseline Git identity: `3288aee5f5fb7af5ab30024f93e838f76724a0b0` plus preserved dirty
  Workstation changes, including completed GNSS baseline-center display packet
  `20260902-gnss-baseline-center-frd`.
- Do not normalize or revert operator-owned worktree changes.
- Baseline storage command:
  `QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q tests/test_measurement_data_store.py tests/test_latest_measurement_xlsx.py tests/test_recording_raw_artifacts.py tests/test_workstation_telemetry.py`
  -> `61 passed in 12.58s`.

## Reproduction

1. Start Measurement or Manual Record.
2. Receive primary P1, P2, and Drone `GPS_RAW_INT`.
3. Stop recording and inspect the fixed JSONL files and Latest Measurement XLSX.
4. P1/P2/Drone raw `gps_rtk` records exist, and the live Telemetry page showed F/R/D/Rg, but there
   is no `gnss_baseline_center.jsonl` or `GnssBaselineCenter` XLSX sheet.

## Observed frontend behavior

- `Devices > Telemetry` shows the completed runtime-only one-row GNSS result.
- `Data > Status` lists fixed `MeasurementDataStore.STREAM_PATHS`; no derived GNSS stream exists.
- Latest Measurement XLSX includes PixhawkDrone, GnssBase and Geometry sheets but no dedicated
  GNSS baseline-center sheet.

## Observed backend and persistence behavior

- `SensorPresentationController._update_gps()` first publishes each current `GpsPositionState`, then
  calls `_record_external_timeline_row(camera, "gps_rtk", ...)`.
- `_recording_side()` maps `left*`, `right*`, and `drone*` to raw PX4 streams. Thus raw primary and
  GPS2 callbacks are retained in `px4_left/right/drone.jsonl` while recording.
- Raw rows contain WGS84 latitude/longitude, MSL/ellipsoid altitude, fix, satellites, accuracy,
  app-receive/source clocks, topic and MAVLink provenance.
- `MeasurementDataStore` owns one priority and one bulk bounded queue and one writer thread. PX4,
  Base GNSS and geometry-comparison streams are bulk; only events/measurements/method evaluations
  have essential retry reserve semantics.
- JSONL is append-only UTF-8, allow_nan=False. Store close/checkpoint already covers every fixed
  stream. Writer failure is per stream and does not reject Measurement.
- Latest XLSX reads immutable byte cutoffs, filters by exact generation, validates, and atomically
  publishes; fixed JSONL remains authoritative.

## Documentation, code, test, and runtime conflicts

- Completed display docs explicitly state the derived row is runtime-only and not persisted.
- User now requests persistence, so that exclusion must be superseded only for a separate derived
  stream; the canonical camera Measurement frame and fields remain unchanged.
- `docs/RECORDING_SCHEMA.md` currently lists no derived GNSS file/stream.
