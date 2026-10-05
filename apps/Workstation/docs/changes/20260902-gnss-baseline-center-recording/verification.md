# Verification

## Requirement-to-evidence matrix

| Requirement | Independent oracle/evidence | Status |
|---|---|---|
| Exact row schema/coordinates | distinct NED/source sentinel plus strict-JSON field assertions | PASS |
| One primary Drone callback/row | direct callback count, raw-before-derived ordering and GPS2 negative fixture | PASS |
| Recording gate | active/inactive fixture plus automatic/manual recording lifecycle regressions | PASS |
| WAITING/STALE/CURRENT | pure timing fixtures and serialized finite/null source/geometry cases | PASS |
| Non-finite source handling | NaN/Inf source fixture becomes explicit WAITING with JSON null fields | PASS |
| Writer isolation/restart | real temporary store append, reopen/append, failed-stream/Measurement isolation | PASS |
| Row <=4 KiB | production-shaped persisted envelope and 10,000-row strict-JSON benchmark | PASS |
| XLSX sheet/latest filtering | old/latest generation fixture, row count and worksheet XML | PASS |
| Raw/Measurement/Parquet unchanged | affected recording, raw, package, canonical, session and Parquet regressions | PASS |

## Focused test evidence

Pre-change storage/recording/XLSX/Telemetry baseline: `61 passed in 12.58s`.

Final pure/schema/trigger/store/XLSX/Telemetry command:
`QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q
tests/test_gnss_baseline_center.py tests/test_gnss_baseline_center_recording.py
tests/test_measurement_data_store.py tests/test_latest_measurement_xlsx.py
tests/test_workstation_telemetry.py` -> `60 passed in 3.94s`.

Independent row-size, real-store, failed-stream isolation and latest-XLSX boundary selection:
`4 passed in 0.77s`.

## Regression and application evidence

Final production-code affected command covering GNSS, data store, XLSX, Recording start/stop/raw/
shutdown/reader, session ownership/Parquet, Measurement package, Pixhawk GNSS and Telemetry:
`252 passed in 24.91s`.

Unfiltered full Workstation pytest:
`QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q`
-> `1887 passed, 1 skipped, 1 failed in 187.20s`. The sole failure is the pre-existing unrelated
`tests/test_camera_stream_recovery.py::test_decoded_nv12_recorder_finalizes_synthetic_lossless_mp4`;
this host exposes the NVIDIA device but rejects the NVENC/CUDA recorder path. No GNSS, recording
JSONL, data-store or XLSX path participates. The unfiltered suite is therefore not reported green.

Qualified full rerun with only that reproduced environment-owned test deselected:
`QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q -k
'not test_decoded_nv12_recorder_finalizes_synthetic_lossless_mp4'`
-> `1887 passed, 1 skipped, 1 deselected in 187.54s`.

`./run.sh --check` exits zero: every required runtime check passes; optional `nvh264dec` remains
unavailable and Auto truthfully selects `avdec_h264`. Python compilation, generated panel delegate
`--check`, and targeted `git diff --check` pass.

The repository portability gate still reports the same 13 pre-existing absolute/dated-path
violations in `Pixhawk/P1/workstation_endpoint.json`, the 20260829 calibration timing packet and a
root 20260828 packet. No new GNSS implementation, test, packet, README or numbered product-doc path
is reported.

## Performance, load, and target-hardware evidence

A final in-process production-shaped benchmark executed the real RecordingController calculation,
row construction and strict persisted-envelope JSON encoding 10,000 times: all 10,000 succeeded,
maximum encoded row was 3,910 bytes, total elapsed time was 0.720181 s and mean callback work was
0.072018 ms on this host. This is well below the 100 ms callback period at 10 Hz. The existing bulk
queue, oldest-bulk-drop accounting, real-store close and failed-stream isolation have independent
automated coverage. No new worker, queue, retry reserve or priority path exists.

At 5-10 Hz the documented conservative disk budget remains approximately 72-144 MiB/hour. Real
hardware rate and long-duration disk growth were not measured and are operational observations,
not acceptance blockers for the deterministic storage boundary.

## Feature isolation and contract evidence

PASS: primary Drone delivery calls the Recording owner after the unchanged raw row; GPS2 does not
trigger; the fixed `gnss.baseline.center` bulk stream is the only new file owner; Data Status reads
its ordinary store snapshot; Latest XLSX is a read-only exact-generation consumer. Canonical
Measurement `baseline_center_*`, Measurement validity/admission, raw PX4 rows, settings, device/
protocol commands and legacy Parquet/review remain outside the slice.

Decoded non-finite GNSS values become JSON null with explicit WAITING/STALE evidence so one bad
number cannot suppress the promised diagnostic row. Any unrelated unsupported producer value keeps
the existing one-record rejection behavior.

## Visual authenticity and dynamic-layout stability evidence

No bespoke layout/control was added. Data Status automatically enumerates the fixed stream using
its existing compact row, and the workbook adds one fixed `GnssBaselineCenter` sheet/Summary count.
Existing UI/application regressions pass. No new screenshot was required or captured.

## Visual and hardware/manual evidence

Software/storage acceptance: PASS. Real three-receiver field logging and manual spreadsheet
inspection: NOT_RUN and optional; no device was commanded during this persistence slice.

## Residual conditions

Rows preserve latest-source asynchronous geometry rather than simultaneous antenna exposure. Bulk
drops, available disk space, physical antenna/lever-arm truth and the operator-selected 1000 ms
spread remain explicit operational limitations. The unrelated NVENC and repository-portability
residuals above remain outside this change.
