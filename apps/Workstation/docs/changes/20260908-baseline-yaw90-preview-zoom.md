# Baseline Yaw90 preview wheel zoom

Request: zoom the camera display while adjusting Attitude/GNSS 90° in Alignment.
Scope: activate the existing display-only 1×–8× cursor-centered wheel zoom in
the two Baseline Yaw90 live previews. This is a local interaction enablement
and an isolated overlay correction, without a new device or settings workflow.
Pre-existing worktree edits are retained.

## Coordinate contract

Source images use native `(u, v)` pixel indices, right/down from the top left.
Their displayed centers use `(u + 0.5, v + 0.5) * scale + offset`; raster edges
use `edge * scale + offset`. The red guide belongs to the native raster's
vertical midpoint (`width / 2` in raster-edge coordinates), not a new aim point at the
viewport edge. It remains one display pixel wide and is clipped when its
transformed native position is outside the visible image. Source pixels stay
unchanged. No changes to optical RDF, PX4 body-FRD attitude in NED, GNSS NED,
North-referenced clockwise azimuth, camera zoom, K,D, measurement or recording.

## Verification slices

1. Existing preview contract / PASS: source of truth is CameraPreview and its
   existing calibration zoom/center-line tests. Baseline command below passed
   36 tests (53 deselected).
2. Native center-line rendering / PASS: independent 160×90 black raster;
   zooming 1.2× around display x=60 moves center x=80 to x=84. At 8× over the
   left edge the native center leaves the viewport and no red column is valid.
   The regression reproduced a false red column at x=159 before the fix.
   After the fix the five related preview component checks passed.
3. Baseline Yaw90 interaction / PASS: Qt wheel events over each live
   image, independent left/right zoom, new-frame continuity, 1×/8× boundaries,
   empty view behavior, source immutability and return to centered fit. An
   offscreen WorkstationMainWindow with synthetic frames passed the smoke check.
4. Related integration / PASS: 118 workspace/calibration/optical-pose/alignment
   UI tests passed. Offscreen 1366×768 and 1920×1080 views preserve visible
   Attitude/GNSS tables, with independent left 3× and right 1× displays. Resize
   checks allow the existing queued fit/layout events to settle and assert
   that both tables are below the previews, with no overlap.
5. Project regression / PASS with one permission-scoped rerun: the first run
   was stopped on user request with exit 130. The resumed full run finished
   with 2080 passed, 2 skipped and 1 socket-permission failure in 205.73 s.
   The sole failure was the unchanged passive UDP discovery test creating an
   AF_INET/SOCK_DGRAM socket (`PermissionError: Operation not permitted`).
   Re-running that exact test outside the sandbox with approval passed in
   0.37 s. No product/test behavior was changed to bypass the restriction.
6. Documentation / PASS: numbered workspace layout documentation updated.
   The user explicitly confirmed Workstation's `./README.md` on 2026-09-08.
   Its Baseline Yaw90 usage and Change Log now describe wheel direction, 1×–8×
   limits, independent views, new-frame continuity, reset at 1×, center-line
   clipping and the display-only coordinate contract.

No worker, protocol, persistence, dependency or resource ownership changes.
Code, scoped validation and documentation are complete. The operator must restart Workstation
to load the source changes. The running operator application and physical
devices were not restarted or reconfigured by this task.

## Evidence

Baseline:
```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 ../../.venv/bin/python -m pytest -q -p no:cacheprovider --tb=short tests/test_baseline_yaw90_workspace.py tests/test_workstation_calibration.py -k 'baseline_yaw90 or camera_preview or preview_wheel or previews_opt_into_wheel'
```

Overlay regression (failed before fix: actual red columns `[159]`, expected `[]`):
```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 ../../.venv/bin/python -m pytest -q -p no:cacheprovider --tb=short tests/test_workstation_calibration.py::test_camera_preview_zoom_keeps_center_line_on_native_image_center
```

All video/Qt validation here uses synthetic frames, not physical camera tests.

Scoped integration:
```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 ../../.venv/bin/python -m pytest -q -p no:cacheprovider --tb=short tests/test_baseline_yaw90_workspace.py tests/test_workstation_calibration.py tests/test_optical_pose_workspace.py tests/test_baseline_yaw_alignment_ui.py
```

Temporary local smoke harness:
```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 ../../.venv/bin/python /tmp/yaw90_zoom_smoke.py
```
This uses the local Qt5 `sendSpontaneousEvent` entry point for the child-label
wheel propagation path, without adding an application dependency. A plain
`QApplication.sendEvent` does not exercise that propagation; see the
[Qt5 event dispatch source](https://github.com/qt/qtbase/blob/5.15/src/widgets/kernel/qapplication.cpp).
The smoke checks each camera independently, min/max zoom, source immutability,
new frames, status refresh, resizing, clearing/restarting preview and absence
of physical camera zoom commands. Visual artifacts inspected:
`/tmp/yaw90-zoom-1366-fit.png`, `/tmp/yaw90-zoom-1366-left3x.png`,
`/tmp/yaw90-zoom-1920-left3x.png`.

Project regression:
```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 ../../.venv/bin/python -m pytest -q -p no:cacheprovider --tb=short --junitxml=/tmp/yaw90-zoom-regression.xml
```

Permission-scoped rerun (outside sandbox, approved):
```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 ../../.venv/bin/python -m pytest -q -p no:cacheprovider --tb=short tests/test_drone_network_discovery.py::test_passive_discovery_reports_source_port_listen_port_and_sysid
```

`git diff --check` passes for the changed tracked files. The native center-line
regression is stored in `tests/test_workstation_calibration.py`; the Yaw90
preview enables the established option in `baseline_yaw90_pane.py`.

Coordinate contract checked: `workspaces.py` / native raster edges and `(u,v)`
pixel centers / independent transformed-midpoint, clipping and unchanged-source
checks. No NED, body-FRD or optical-RDF geometry calculation changed.
