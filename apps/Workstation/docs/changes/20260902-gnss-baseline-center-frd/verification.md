# Verification

## Requirement-to-evidence matrix

| Requirement | Independent oracle/evidence | Status |
|---|---|---|
| Exact axes and signs | hand-built NED `b=(0,2,0)`, `t=(10,3,4)` -> F=10,R=2,D=4,Rg=sqrt(120) | PASS |
| Side reversal | hand-built swapped P1/P2 sentinel | PASS |
| WGS84 adapter/Rg compatibility | alternate scalar/current-function comparison | PASS |
| Missing/degenerate | None/non-finite/vertical-only fixtures | PASS |
| Age/future/spread | explicit monotonic timestamps around 2.0 s/1.000 s bounds | PASS |
| Fix not gate | identical geometry with fix 3/5/6 | PASS |
| Snapshot handoff | real controller with in-memory GpsPositionState | PASS |
| UI states/stability | offscreen WAITING/CURRENT/STALE/reversed/long values | PASS |
| Preserved Telemetry | existing six-test baseline plus affected regressions | PASS |

## Focused test evidence

Baseline: `tests/test_workstation_telemetry.py` -> `6 passed in 0.66s`.

Pure GNSS coordinate/readiness slice:
`QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q tests/test_gnss_baseline_center.py`
-> `8 passed in 0.67s`.

Pure + snapshot + real Telemetry widget focused command:
`QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q tests/test_gnss_baseline_center.py tests/test_workstation_telemetry.py`
-> final `16 passed in 0.75s`.

## Regression and application evidence

Affected coordinate/Telemetry/geometry/Workstation UI/Pixhawk GNSS/lifecycle plus canonical
Measurement/Recording/Parquet isolation command: `257 passed in 33.59s`.

Unfiltered full Workstation pytest:
`QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q`
-> `1877 passed, 1 skipped, 1 failed in 189.26s`. The sole failure is the pre-existing unrelated
`tests/test_camera_stream_recovery.py::test_decoded_nv12_recorder_finalizes_synthetic_lossless_mp4`;
the current environment rejects NVENC/CUDA initialization, as independently reproduced earlier in
this session. No GNSS/Telemetry path participates. The unfiltered suite is not reported green.

Final-code qualified rerun with only that reproduced environment-owned test deselected:
`QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q -k 'not test_decoded_nv12_recorder_finalizes_synthetic_lossless_mp4'`
-> `1877 passed, 1 skipped, 1 deselected in 186.17s`.

Runtime entrypoint: `./run.sh --check` -> exit 0. Required checks PASS; optional `nvh264dec` is
unavailable and Auto truthfully selects `avdec_h264`.

Repository portability gate from the SASL root failed with 13 pre-existing absolute/dated-path
violations in `Pixhawk/P1/workstation_endpoint.json`, the 20260829 calibration timing packet, and a
root 20260828 packet. No new GNSS module, test, packet, README or product-doc path was reported.

## Performance, load, and target-hardware evidence

No new worker/resource demand. Automated repeated-state UI tests keep one model row and fixed
group/table geometry at the existing refresh path. Hardware is not needed to prove the pure
transform; live field comparison remains manual evidence and was not an approved blocker.

## Feature isolation and contract evidence

PASS: controller owns the mapping, widget reads only that mapping, and no settings/device/worker
path was added. Canonical `MEASUREMENT_FRAME` remains distinct. Existing canonical
Measurement/Recording/Parquet/optical regressions are included in the 257-pass affected result.

## Visual authenticity and dynamic-layout stability evidence

Before screenshot: `/tmp/gnss-baseline-center-before.png`. Final CURRENT screenshot:
`/tmp/gnss-baseline-center-after-final.png`; table 1314x68 px, one row, and group 1328x150 px at
1920x1080. Earlier STALE screenshots:
`/tmp/gnss-baseline-center-after-stale-1600.png` and
`/tmp/gnss-baseline-center-after-stale-1366.png`. Automated UI tests prove identical group/table
height, RTCM position, focus and scroll across CURRENT/STALE/WAITING at one viewport. At 1366 with
both side docks open, the existing central page requires normal vertical scrolling and the table
uses a horizontal scrollbar; dock collapse remains the documented narrow-screen path.
At `QT_SCALE_FACTOR=1.5`, the 1000 px Telemetry workspace reserved a 41 px coordinate-contract
label for a 41 px height-for-width requirement, with no clipping.

## Visual and hardware/manual evidence

Visual automated-fake/offscreen: PASS. Real three-receiver field comparison: NOT_RUN and explicitly separate from
automated mathematical acceptance.

## Residual conditions

GNSS antenna lever arms, tilted camera/body axes, Earth curvature beyond the existing local tangent
scope, GNSS error/latency, and physical P1/P2 side correctness remain visible operational limits.
