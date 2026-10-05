# Verification

## Requirement-to-evidence matrix

| Slice | Required evidence | Status |
| --- | --- | --- |
| Pure angle contract | cardinal, wrap, optional/invalid input unit tests | PASS |
| Evidence projection | current/partial/stale/waiting/source tests | PASS |
| Pixhawk UI | values, plot, rapid refresh, stable layout tests | PASS |
| Preservation | existing Pixhawk/GNSS/UI/lifecycle regressions | PASS |
| Full project | documented pytest command plus isolated boundary | PASS WITH UNRELATED ENVIRONMENT FAILURE |
| Real hardware | P1/P2/GNSS field sequence | UNVERIFIED |

## Focused test evidence

```bash
../../.venv/bin/python -m pytest -q \
  tests/test_baseline_yaw_alignment.py \
  tests/test_baseline_yaw_alignment_ui.py \
  tests/test_workstation_telemetry_plots.py \
  tests/test_workstation_pixhawk_gnss.py
# 122 passed in 6.13s
```

The independent oracle covers cardinal NED bearings, 0/360 and +/-180 wrapping, separate
Baseline-P1/Baseline-P2 values, missing/stale side isolation, selected GPS2, zero-horizontal and
non-finite inputs. Qt coverage renders WAITING/PARTIAL/STALE/CURRENT, checks 1366x768, 1600x900 and
1920x1080 geometry, 40 rapid sign-boundary updates, fixed 250 px plot height, fixed status/evidence
regions, and preserved outer scroll.

## Regression and application evidence

Direct dependent regression:

```bash
../../.venv/bin/python -m pytest -q \
  tests/test_baseline_yaw_alignment.py tests/test_baseline_yaw_alignment_ui.py \
  tests/test_workstation_telemetry_plots.py tests/test_workstation_pixhawk_gnss.py \
  tests/test_status_model_pixhawk.py tests/test_workstation_ui.py \
  tests/test_workstation_contracts.py tests/test_workstation_lifecycle.py
# 214 passed in 26.49s
```

The canonical full command reached one pre-existing hardware/environment boundary:

```bash
../../.venv/bin/python -m pytest -q -x
# 180 passed, then FAILED
# tests/test_camera_stream_recovery.py::
#   test_decoded_nv12_recorder_finalizes_synthetic_lossless_mp4
# DecodedNv12Recorder.submit() returned False for the second synthetic frame.
```

The exact test failed identically in isolation. It does not import or exercise the changed
physical-alignment/evidence/plot/workspace path. After excluding only that named test, a monolithic
run encountered a pre-existing suite-order wait; bounded file-range runs completed the full
selected regression boundary:

```bash
../../.venv/bin/python -m pytest -q tests/test_[a-m]*.py \
  -k 'not test_decoded_nv12_recorder_finalizes_synthetic_lossless_mp4'
# 764 passed, 1 deselected in 59.65s

../../.venv/bin/python -m pytest -q tests/test_[n-z]*.py
# 1041 passed, 1 skipped in 84.81s
```

Application boundary:

```bash
./run.sh --check
# PASS; optional nvh264dec unavailable and Auto selected avdec_h264 (WARN)
```

## Performance, load, and target-hardware evidence

No new worker/resource allocation. Forty in-place refreshes passed without group/plot/status/
evidence height or scroll changes. Runtime check confirmed the canonical interpreter and required
GStreamer elements; optional `nvh264dec` remains unavailable in this environment.

## Feature isolation and contract evidence

PASS. The new dependency direction is pure physical-alignment value -> EvidenceAdapter -> Pixhawk
workspace/plot. No command, worker, setting, socket, persistence, Measurement, Alignment R,t, or
recording owner changed. Existing Pixhawk/GNSS/UI/lifecycle tests passed.

## Visual authenticity and dynamic-layout stability evidence

PASS (automated offscreen plus visual inspection). A 1292x280 full-group capture and 650x250 plot
capture were inspected. The first plot exposed overlapping east-facing B/P2/E labels; labels were
then placed at distinct radial/tangent positions and the corrected capture was re-inspected.
Numeric/unit alignment, separate Baseline-P1/P2 rows, compact evidence, explicit legend, and quiet
native styling match the existing Workstation visual language.

## Visual and hardware/manual evidence

Automated/fake Qt evidence is complete. Real P1/P2/GNSS hardware is not yet exercised by this
packet; no field claim is made.

## Residual conditions

- Real-rig CURRENT/PARTIAL/STALE/recovery and outdoor GNSS readability remain hardware-manual.
- The unrelated NVIDIA lossless-recorder test currently fails in isolation because its recorder
  rejects the second frame; all selected non-NVIDIA regression tests passed.

## 2026-09-03 corrected GNSS installation-angle verification

- GNSS baseline-center pure/controller/Telemetry plus retained P1/P2-yaw compatibility suite:
  `47 passed in 1.52s`.
- Corrected scope is verified in code and tests: angle is `atan2(|Forward|,|Right|)` from
  P1/P2/Drone GNSS only; 90° error is `|90°−angle|`; no Drone ATTITUDE/yaw input and no recording
  schema change.
- Dependent Workstation boundary (`baseline`, `Telemetry`, P1/P2 Pixhawk, status, lifecycle, UI,
  and contract tests): `237 passed in 33.12s`.
- Full Workstation suite: `1899 passed, 2 skipped, 1 failed`; the only failure was
  `tests/test_drone_network_discovery.py::test_passive_discovery_reports_source_port_listen_port_and_sysid`
  because sandbox policy denied UDP socket creation (`PermissionError: [Errno 1] Operation not permitted`).
  Rerunning that exact boundary with socket access produced `1 passed in 0.94s`.
- Launcher after implementation: `./run.sh --check` exited `0`; only optional `nvh264dec`/NVIDIA
  warnings remain.
- Real three-GNSS installation geometry remains hardware/manual evidence. The latest saved session
  row contains the angle inputs but is `STALE` because its three-source timestamp spread exceeded
  1000 ms.
