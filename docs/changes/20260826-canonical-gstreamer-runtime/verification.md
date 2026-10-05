# Verification

## Requirement-to-evidence matrix

| Slice | Evidence | Status |
|---|---|---|
| canonical prefix / venv config / manual `.pth` rejection | pure temp-venv tests plus real `--execution` | PASS |
| Ubuntu manifest / recreation readiness | real dpkg/GI/GStreamer deployment doctor | CURRENT HOST PASS; clean-host recreation NOT RUN |
| gi, Gst/GstRtp, element find+make | fake missing/make failures and real factories | PASS |
| Auto/NVIDIA/Software policy | pure matrix, driver unavailable, app camera-start tests | PASS |
| launcher exit 0/1/2 and no fallback | copied-repository launcher tests and real commands | PASS |
| direct entry / child interpreter | real app/Simulation venv rejection and fake spawn contexts | PASS |
| app settings/model/import checks | real app-only checks and Workstation runtime unit boundary | PASS |
| existing camera/lifecycle/recording behavior | app regressions below | PASS with harness residual |
| real dual-camera NVIDIA/software sustained run | target hardware | short dual NVIDIA PASS; software/sustained recording NOT RUN |

## Focused test evidence

- 2026-08-27 environment-closure follow-up: runtime focused **53 passed**, expanded StereoWIRIS
  runtime **117 passed**, and Workstation runtime/recovery **33 passed**. Tests inject ambient
  user-site/Python/loader/GI/GStreamer variables and verify doctor/deployment/Workstation/Legacy
  boundaries still exit zero. `/usr/bin/python3 -S` imports the deployment doctor before
  third-party `packaging` exists. `pip check`, shellcheck, Python compilation and diff check pass.
- The current system GStreamer factories, including `nvh264dec`, resolve to dpkg-owned
  `/usr/lib/.../gstreamer-1.0` artifacts with RTX 4050 driver 595.84. The later R10 audit reduced the
  repository portability result from 12 findings to PASS without rewriting historical evidence.

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

No worker count, queue, thread budget, reconnect schedule, or preview/inference rate changed by
this runtime slice. On the repaired host, both real WIRIS panes reached current-frame LIVE with
`nvh264dec` selected by Auto; the observed combined run used roughly 315/313 MiB RSS for the camera
children and about 102 MiB GPU memory per child. This is short-path evidence only. Sustained
software-decoder, recording/MP4 and cleanup measurements remain open, and the later combined field
failure was PX4 MAVLink wire silence rather than a decoder/runtime-selection failure.

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

The first 2026-08-26 baseline correctly failed: `python3.10-venv` was absent, the old venv exposed a
manual system-package `.pth`, and NVIDIA driver communication failed. That evidence is retained as
the pre-repair baseline rather than the current state.

Current-host repair evidence on 2026-08-26:

- `PYTHONNOUSERSITE=1` imports resolve wholly inside the canonical root venv: OpenCV 4.11.0,
  Torch 2.13.0+cu130 with CUDA available, Ultralytics 8.4.82, PyYAML 6.0.3, PyArrow 23.0.1,
  pymavlink 2.4.49 and packaging 26.3. `pip check` reports no broken requirements.
- `PYTHONNOUSERSITE=1 apps/Workstation/run.sh --check` exits **0**. GI/Gst/GstRtp, every mandatory
  GStreamer element, `nvh264dec`, NVIDIA GeForce RTX 4050 Laptop GPU driver 595.84, Auto decoder
  selection, app imports and declared distribution ranges all pass.
- `PYTHONNOUSERSITE=1 apps/StereoWIRIS/run.sh --check --mavlink-only` exits **0** with the same
  canonical interpreter and runtime boundary.
- The large CUDA payload needed for this host repair was copied by hard-link from a complete local
  package payload after repeated CDN transfers stopped at 137.3 MiB. This is a local recovery fact,
  not the documented clean-install method; the standard manifest remains authoritative and a
  clean-host recreation is still required.
- Both actual WIRIS RTSP streams reached current-frame LIVE with Auto selecting real `nvh264dec`.
  Real software decode, sustained dual Preview/recording, strict MP4 decode and orphan inspection
  remain **NOT RUN**.

## Residual conditions

1. Recreate the canonical runtime once on a clean host using only the documented manifests; do not
   treat the local CUDA hard-link recovery as the deployment procedure.
2. Run both real cameras separately with NVIDIA and software, then sustained dual software Preview;
   verify UI responsiveness, Auto/NVIDIA/Software visible state, MP4 finalization, and no orphan
   camera/ffmpeg process after close.
3. **Closed 2026-08-27:** active Simulation examples and the Legacy model setting are portable;
   calibration/change evidence is explicitly historical. The audit reports zero findings. A
   same-host fresh checkout/new venv installed the manifest and passed both launchers under isolated
   HOME. It exposed the now-owned `setuptools>=77,<80` Torch/colcon intersection.
4. The root pytest import-name collision and StereoWIRIS native Qt test-order crash remain harness
   issues outside this approved runtime slice.
