# Verification

## Requirement-to-evidence matrix

| Requirement | Evidence | Status |
|---|---|---|
| Mountain candidate exposes numeric relative R/t | `test_gnss_mountain_uses_mountain_rotation_and_gnss_metric_baseline` | PASS |
| Existing Candidate review renders R/t | `test_candidate_detail_panel_shows_provenance_and_warnings_on_selection` | PASS |
| Existing solve/apply behavior is preserved | optical-pose controller/geometry/solver regression | PASS |
| No worker/device/resource regression | Static impact review; no changed worker path | PASS (review) |

## Focused test evidence

Focused owner test:

```text
QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q tests/test_optical_pose_controller_integration.py -k 'gnss_mountain_uses_mountain_rotation_and_gnss_metric_baseline'
1 passed, 43 deselected
```

Focused UI test:

```text
QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q tests/test_optical_pose_workspace.py -k 'candidate_detail_panel_shows_provenance_and_warnings_on_selection'
1 passed, 21 deselected
```

Relevant regression:

```text
QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q tests/test_optical_pose_workspace.py -k 'not import_pair_emits_only_paths_without_zoom_prompt'
21 passed, 1 deselected
QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q tests/test_optical_pose_controller_integration.py tests/test_optical_pose_geometry.py tests/test_pose_solvers.py
82 passed
```

The combined four-file run was `103 passed, 1 failed`; the one failure is the pre-existing
`test_import_pair_emits_only_paths_without_zoom_prompt`. HEAD already contains the zoom
input dialog that causes that failure, so it is unrelated to this R/t display patch.

## Regression and application evidence

Workstation full suite from `apps/Workstation`:

```text
QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q
1882 passed, 30 failed, 2 skipped, 2 warnings
```

The 30 failures are pre-existing dirty-worktree regressions outside this change; the only
directly related failure is the same unchanged zoom-dialog test above. The launcher check
passed:

```text
./run.sh --check
SASL Workstation runtime check ok
```

The repository-root invocation was also attempted as required by the shared pytest config,
but collection stopped on the pre-existing duplicate test module name
`test_px4_attitude_cal_manager` between StereoWIRIS and Workstation.

## Performance, load, and target-hardware evidence

Not applicable to this no-worker/no-device change; no load test is introduced.

## Feature isolation and contract evidence

The patch must keep the controller as owner and the UI as formatter. Existing candidate
methods without the optional projection must remain renderable.

## Visual authenticity and dynamic-layout stability evidence

Automated assertions will cover text and selection. Manual screenshot review may be run
if a Qt display is available; no generic visual components are added.

## Visual and hardware/manual evidence

Real Mountain image/GNSS accuracy is not proven by this display change and remains an
operator/hardware verification item.

## Residual conditions

Focused and relevant software criteria pass. Residual conditions are the pre-existing
dirty-worktree full-suite failures/collection mismatch and unperformed real-camera visual/
accuracy verification; neither changes the read-only R/t projection contract.
