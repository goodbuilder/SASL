# Verification

## Requirement-to-evidence matrix

| Slice | Evidence | Status |
|---|---|---|
| canonical prefix / venv config / manual `.pth` rejection | pure temp-venv tests plus real `--execution` | PASS |
| Ubuntu manifest / recreation readiness | real dpkg/GI/GStreamer deployment doctor | PASS (expected deployment FAIL) |
| gi, Gst/GstRtp, element find+make | fake missing/make failures and real factories | PASS |
| Auto/NVIDIA/Software policy | pure matrix, driver unavailable, app camera-start tests | PASS |
| launcher exit 0/1/2 and no fallback | copied-repository launcher tests and real commands | PASS |
| direct entry / child interpreter | real app/Simulation venv rejection and fake spawn contexts | PASS |
| app settings/model/import checks | real app-only checks and Workstation runtime unit boundary | PASS |
| existing camera/lifecycle/recording behavior | app regressions below | PASS with harness residual |
| real dual-camera NVIDIA/software sustained run | target hardware | NOT RUN |

## Focused test evidence

- Automated fake/real mix:
  `QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -p no:cacheprovider -q
  apps/StereoWIRIS/tests/test_operator_runtime.py
  apps/StereoWIRIS/tests/test_operator_runtime_launchers.py
  apps/StereoWIRIS/tests/test_stereowiris_ros_workspace_contract.py
  apps/StereoWIRIS/tests/test_yolo_stream.py` → **93 passed**.
- Automated fake: `.venv/bin/python -m pytest -p no:cacheprovider -q
  apps/Workstation/tests/test_runtime_closure.py
  apps/Workstation/tests/test_camera_stream_recovery.py` → **21 passed**.
- Automated fake: selected StereoWIRIS camera stop/restart, recording finalization, close, and
  decoder-fallback nodes from `test_stereo.py` → **8 passed**.
- New source files pass full Ruff; all affected files pass Ruff `E9,F821`; `git diff --check` passes.

## Regression and application evidence

- Automated offscreen: Workstation full suite → **1159 passed, 1 skipped**.
- Automated offscreen: StereoWIRIS suite excluding `test_stereo.py` → **391 passed**.
- Automated-real app boundaries:
  - StereoWIRIS app-only settings/model/import/GStreamer check → exit **0**, settings/model PASS,
    selected `auto`, effective `avdec_h264`, missing elements `none`.
  - Workstation app-only settings/model/import/version check with the separately proven common
    boundary injected → exit **0**, GStreamer backend and model path reported.
- Repository-root pytest does not reach tests because the pre-existing same-name
  `test_px4_attitude_cal_manager.py` modules in two non-package test directories trigger pytest
  `ImportPathMismatchError`. App suites were therefore executed separately.
- `test_stereo.py` as one process hits a native Qt segmentation fault after about 24 tests at
  `test_gnss_base_candidate_rows_apply_selected_port` (or the following Qt test when that node is
  excluded). The crashing nodes pass alone, and the eight in-scope lifecycle/recording/decoder
  nodes pass together. This is recorded as a test-order/native Qt harness residual, not a Python
  assertion failure.

## Performance, load, and target-hardware evidence

No worker count, queue, thread budget, reconnect schedule, or preview/inference rate changed.
Software tests prove selection and lifecycle only; no CPU/GPU utilization or sustained dual-stream
measurement was collected in this environment.

## Feature isolation and contract evidence

Workstation and Legacy consume the neutral `sasl_common.operator_runtime` policy but retain
separate camera/resource owners. Root launcher tests prove app/Simulation/system interpreter
fallback is absent. Existing Workstation suite covers camera, recording, session, NTRIP, MAVLink,
runtime closure, and UI contracts without regression.

## Visual authenticity and dynamic-layout stability evidence

No widget, layout, token, table, focus, selection, or scroll behavior changed. Error/recovery
states reuse existing status labels/logs and provide text rather than color-only meaning. Offscreen
camera-start tests verify explicit NVIDIA refusal and Auto software status text.

## Visual and hardware/manual evidence

- Real deployment doctor on 2026-08-26: exit **1** because `python3.10-venv` is not installed;
  other declared Ubuntu packages are installed.
- Real GStreamer factory check: all mandatory elements pass find/make; `nvh264dec` factory find
  succeeds but make returns no element. `nvidia-smi` cannot communicate with the driver. Auto is
  therefore an allowed WARN with effective `avdec_h264`.
- Real `--execution` and both launcher `--check` commands correctly exit **1** for the existing
  `include-system-site-packages=false` venv and manual
  `workstation-system-dist-packages.pth`. Help exits **0** without those dependencies.
- Actual WIRIS left/right TCP RTSP, NVIDIA decode, software decode, sustained two-camera software
  Preview, GUI responsiveness, MP4 output, and orphan-process inspection: **NOT RUN** until the
  documented operator migration and NVIDIA repair are complete.

## Residual conditions

1. Install `python3.10-venv`, back up/recreate root `.venv`, install root pip requirements, and
   require deployment/execution/app checks to pass.
2. Repair/confirm the NVIDIA driver before testing explicit NVIDIA.
3. Run both real cameras separately with NVIDIA and software, then sustained dual software Preview;
   verify UI responsiveness, Auto/NVIDIA/Software visible state, MP4 finalization, and no orphan
   camera/ffmpeg process after close.
4. The repository portability audit still reports 10 pre-existing dated absolute paths in user
   calibration JSON, `Simulation/goal.md`, and the existing StereoWIRIS settings file; none was
   introduced by this change.
5. The root pytest import-name collision and StereoWIRIS native Qt test-order crash remain harness
   issues outside this approved runtime slice.
