# Verification

## Baseline

- Current relevant lifecycle/package/session tests: `93 passed` on 2026-08-26.
- Target-hardware Workstation process and usable NVIDIA telemetry were not active during planning.

## Slice matrix

| Slice | Required evidence | Status |
| --- | --- | --- |
| Data contract/writer | append/restart/no-overwrite/path/tail/failure/priority/drop/shutdown tests | FOCUSED_PASS |
| Transient lifecycle | old Session ignored, storage failure start, queue/manual handoff, sequential runs | FOCUSED_PASS |
| Producer isolation | method/native/RTCM/ULog/capture/alignment/late-result tests | FOCUSED_PASS |
| UI/facade | Data navigation, removed controls, enabled Measurement, stable dynamic state | FOCUSED_PASS |
| Compatibility/docs | existing bytes unchanged, offline legacy parse, README/schema/portability | SOFTWARE_PASS; ENVIRONMENT_RESIDUAL |

## Focused evidence recorded

- Data/native/ULog/run tests: `30 passed`.
- Storage, RTCM, HR/FTP, recording, camera slice: `118 passed`.
- Optical-pose/UI/contracts slice: `106 passed`.
- Target Drone fixed-stream and legacy compatibility slice: `18 passed`.
- Sessionless UI/lifecycle/contract groups passed before the final documentation pass; the full
  suite remains the authoritative final count.

## Final automated evidence

- Final `QT_QPA_PLATFORM=offscreen pytest -q tests` after all metadata cleanup:
  `1186 passed, 1 skipped` in `159.59 s`.
- `python tools/gen_panel_delegate_stubs.py --check`: PASS.
- `git diff --check -- .`: PASS.
- Python compilation of changed controller/store/UI modules: PASS.
- `./run.sh --check`: environment-blocked. The existing root `.venv` was not created with
  `--system-site-packages` and contains the unsupported
  `workstation-system-dist-packages.pth`; no environment was replaced during this change.
- Repository `python3 tools/check_portable_paths.py`: environment/worktree-blocked by 10 existing
  violations in StereoWIRIS calibration/settings, Simulation documentation, and the untracked
  Workstation `intrinsics_zoom_08.json`; none was rewritten as part of this change.
- Pytest emitted only one read-only root `.pytest_cache` warning under the managed workspace.

## Commands to record

- Focused pytest commands after each slice.
- `source ../../.venv/bin/activate && pytest -q tests`
- `QT_QPA_PLATFORM=offscreen pytest -q`
- `./run.sh --check`
- repository-root `python3 tools/check_portable_paths.py`

## Field/manual evidence

Required: launch with the real 20260823 OPEN/RECOVERABLE data present; storage failure; Manual to
Measurement transition; two sequential Measurements; busy camera queued start; source partial
failure; actual file paths/timestamps; shutdown. Hardware results must not be inferred from fakes.

Status: NOT_RUN on real hardware. Software/offscreen tests cover each transition and the exact old
RECOVERABLE/storage-unavailable admission case, but the intended WIRIS/Pixhawk/GNSS/RTCM/ULog
field stack was not active and remains a qualification residual.

## Coordinate contract

Golden rows must retain baseline-center FRD, camera optical RDF, PX4 NED/body FRD, WGS84, units,
and source clock semantics. This change does not authorize formula or frame conversion edits.
