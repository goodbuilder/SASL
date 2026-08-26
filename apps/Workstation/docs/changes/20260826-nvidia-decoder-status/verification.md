# Verification

## Requirement-to-evidence matrix

| Requirement | Automated | Real/runtime |
|---|---|---|
| Auto selects NVDEC | selection unit/fake probes | host doctor |
| Actual NVDEC decode | gst pipeline exit/caps | synthetic 1080p30 CUDA-memory decode |
| Per-side actual display | UI/status tests | Workstation screenshot sequence |
| CPU fallback reason | reducer/controller tests | injected/real failure if safe |
| No geometry/focus change | widget geometry tests | screenshot sequence |
| Resource behavior | worker-count contract | `nvidia-smi dmon`, dual RTSP pending |

## Focused test evidence

### Slice 1 — pure decoder presentation: PASS

- Command: `../../.venv/bin/python -m pytest -q tests/test_workstation_runtime_camera_status.py`
- Result: `4 passed in 0.29s`.
- Verified GPU active, explicit Software, Auto CPU fallback with reason, requested-without-actual
  evidence, and NVIDIA/software mismatch.

### Slice 2 — decoder selection/fallback/GPU identity evidence: PASS

- Command: `../../.venv/bin/python -m pytest -q tests/test_workstation_runtime_camera_status.py tests/test_workstation_camera.py`
- Result: `64 passed in 4.17s`.
- Verified requested/effective/candidates/NVIDIA availability, dual-side fallback reason,
  selector-clears-override retry, and static `/proc` model/driver identity parsing.

### Slice 3–4 — Camera/CAM/Performance UI and Auto persistence/docs: PASS

- Command: `../../.venv/bin/python -m pytest -q tests/test_workstation_runtime_camera_status.py tests/test_workstation_camera.py tests/test_workstation_ui.py tests/test_workstation_settings.py`
- Result: `101 passed in 13.92s`.
- Verified primary per-side GPU/CPU/fallback text, CAM WARN detail, Performance identity/capability,
  GPU→fallback geometry and focus stability, selector retry, and Settings compatibility.
- Persisted runtime setting changed from `software` to `auto`; repository default was already Auto.
- Updated approved `./README.md` and UI workflow/status/implementation product contracts.

## Regression and application evidence

### Slice 5 — full regression and application boundary: PASS

- Canonical test environment aligned to pytest 8.4.2/pluggy 1.6.0. A first run under system
  pytest 6.2.5 produced one failure then a native abort; the isolated workbook test passed, and the
  aligned full rerun passed.
- Full command: `../../.venv/bin/python -m pytest -q -x`.
- Result: `1231 passed, 1 skipped in 113.16s`.
- `./run.sh --check`: all required distributions/GStreamer elements PASS; NVIDIA RTX 4050 driver
  595.84 PASS; `decoder.selection: auto -> nvh264dec` PASS.
- Synthetic command path: generated 1920x1080@30 H.264, decoded through `nvh264dec` with
  `video/x-raw(memory:CUDAMemory)` NV12 and EOS in 0.1818 s.
- Actual entry point: `./run.sh` created PID 643286 and a maximized `StereoWIRIS Workstation`
  window; `run.log` has no traceback. It contains only disconnected-device and missing-intrinsics
  notices.
- `git diff --check` and generated delegate stub check PASS.
- Portability command still FAILS with the same 12 pre-existing findings outside this packet;
  scoped changed files contain no home/dated-checkout match.

## Performance, load, and target-hardware evidence

- RTX 4050 Laptop GPU, 6141 MiB, driver 595.84, CUDA 13.2.
- GStreamer 1.20.3 `nvcodec` and `nvh264dec` factory/element PASS.
- Synthetic 1920x1080@30, 150-frame H.264 → `nvh264dec` → CUDA NV12 → EOS in 0.181 s.
- Fast loop stress: NVDEC engine 78–100%, SM 7–9%, memory engine 3–4%.
- Real dual WIRIS preview: NOT_RUN.

## Feature isolation and contract evidence

Pending: software selection, Auto fallback, explicit NVIDIA error, recording-safe restart, shutdown.

## Visual authenticity and dynamic-layout stability evidence

Automated widget sequence verified GPU → explicit Software → CPU fallback → stopped without
changing decoder widget geometry, preview splitter sizes, or Preview-button focus. Offscreen
1600x1000 captures at `/tmp/workstation_decoder_gpu.png` and
`/tmp/workstation_decoder_fallback.png` reuse the existing Camera Status layout and add no card,
badge, navigation, or decorative pattern.

## Visual and hardware/manual evidence

Synthetic NVDEC is automated-real boundary evidence. Camera-specific Left/Right WIRIS RTSP remains
NOT_RUN because another StereoWIRIS process/window is active and normal operation forbids concurrent
device ownership.

## Residual conditions

Persisted mode is Auto and software/application verification passes. This packet remains open until
both real WIRIS streams report `GPU · nvh264dec` with current frames, acceptable sustained rate,
no unbounded queue/drop/thermal behavior, and normal shutdown. Repository portability also retains
12 unrelated pre-existing findings.
