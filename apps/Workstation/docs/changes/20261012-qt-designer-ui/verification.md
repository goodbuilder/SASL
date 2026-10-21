# Verification ledger

## Final acceptance — 2026-10-12

| Criterion | Result | Evidence / boundary |
|---|---|---|
| A1 explicit coverage | PASS | 64 forms; feature map and retained custom/hidden surfaces in docs/ui/QT_DESIGNER.md |
| A2 native Designer editing | PASS | Qt5 Designer opened, edited and saved a copy; retained screenshots |
| A3 authoritative form at runtime | PASS | Saved Designer title loaded through production loader; tests edit text, margins, limits, font/style, table height and tab order |
| A4 preserved workflows | PASS in software environment | Final full suite: 2,793 passed,1 skipped; command/input/readback/state/preview/navigation/shutdown tests. Physical equipment not exercised |
| A5 frames and geometry | PASS for preserved contracts | NED/FRD/RDF/WGS84/ECEF checks; synthetic native frame at 1366x768 ; 57 real-shell captures at 3 sizes; inherited clipping documented below |
| A6 reproducible assets | PASS | Package-relative assets, no new runtime dependency, portability checker PASS, guide/README updated. New source/assets remain untracked until committed |

Authoritative final command from `apps/Workstation`:

```bash
./tools/run_isolated_tests.sh -q -o faulthandler_timeout=45 --tb=short
```

**2,793 passed, 1 skipped in 771.47s (12:51), exit0.** The final process completed
its slow Qt session cleanup after emitting a 45-second faulthandler diagnostic;
there was no final test failure or timeout termination. The proposed batched
rerun was unnecessary and was not executed. Full log at
`/tmp/workstation-designer-final-all.log`; retained terminal result below.

Skip: synthetic decoded NV12 MP4 encoding requires NVENC; FFmpeg reported
`cuInit(0): CUDA_ERROR_UNKNOWN`. No physical camera/Pixhawk/GNSS or live NVENC
success is claimed. Real panel/shell captures used disabled hardware startup,
isolated loopback networking and temporary settings/data, and completed with
no resources left running.

Supporting final checks: Designer/workspace bundle: 237 passed; wider
lifecycle/Calibration/Baseline/USB/camera/boundary bundle: 253 passed;
57 final-source captures with 0 callback errors; portability and diff checks PASS.
These counts overlap the full suite and are not additional unique tests.

## Append-only work ledger

- Baseline: source 6810b22 + pre-existing changes; status saved /tmp/workstation-designer-baseline-status.txt.
- Initial isolated capture inside sandbox FAIL: netlink permission. Rerun with sandbox escalation; hardware network isolation retained.
- S1–S4 NOT_RUN pending implementation and fresh verification. Hardware/manual behavior NOT_RUN; no device commands authorized by this layout task.

- Baseline capture PASS: six real panel/shell views, zero callback errors and shutdown complete (no threads/processes/sockets/shared memory). Artifact /tmp/workstation-designer-before/visual-summary.json.
- S1 shell regression FOUND: PyQt5 uic registers only a plain QWidget as main-window central widget; initial direct QStackedWidget yielded a 30px viewport. Fixed .ui centralwidget wrapper with zero-margin layout. Viewport retest pending.
- S2 initial in-progress device form FAILED: enum alignment serialized as number. Corrected to Qt flag set; focused device suite 12 PASS.
- S2/S3 Settings 29 PASS; Inspector 7 PASS; acquisition 54 PASS; engineering 154 PASS + 2 baseline stale source-path failures; Baseline Yaw90 34 PASS + 4 layout checks deferred until shell fix.
- S1/S2 root UI + ownership suite: 112 PASS; 2 FAIL from pre-existing tests reading removed Workstation/workspaces.py and main_window.py. Current owners are ui/workspaces.py and application/main_window.py; investigate baseline separately, do not attribute to .ui loader.
- A2/A3 actual Qt5 Designer interactive proof PASS: opened copied main_window.ui, used Property Editor to change windowTitle to 20261012, saved, loaded saved copy through Workstation.ui.designer.load_form into QMainWindow. Exact title == 20261012 and centralWidget exists. Artifacts /tmp/workstation-designer-edit/{main_window.ui,designer-open.png,saved.png}. Repository form unmodified by this probe. Designer exited normally.
- Coverage expanded to remaining visible Calibration/Pixhawk/OpticalPose/acquisition part controls rather than treating Python-only static internals as complete. Dynamic custom rendering and hidden legacy adapters remain explicit exceptions.
- A6 portable path gate PASS: `python3 tools/check_portable_paths.py` from repository root.
- Root forms/viewport integration attempt: 77 PASS, 9 FAIL while Calibration edits were in progress (read-only QSpinBox text serialized; reported to owner). No viewport result claimed from aborted window construction. Added standalone form load to schema checks so invalid Qt setters fail before shell integration.

- Architecture verification updated for reviewed one-way presentation dependencies: optical-pose components -> layouts, Pixhawk connections/GNSS/magnetometers -> Pixhawk Designer slot helper. Corrected the two affected architecture tests to read their actual package source paths after the existing module move; assertions remain intact.
- Source-of-truth audit: removed static Python overrides of loaded controls (geometry, ranges, policies, labels and spacing). Acquisition repeat PASS: 63 tests, including a copied form maximumHeight edit 120 -> 177. Optical-pose command factory now transfers editable font/style/icon/shortcut properties from Designer slots; regression pending.
- Expanded visible subviews: recording target reference, worker recovery, session and footer controls; Pixhawk GNSS/magnetometer/candidate/recovery controls; OpticalPose component forms; Calibration capture/input/quality/validation forms. No device/runtime algorithm changes intended.

- Final static form gate PASS: 64 forms, 68 tests passed (includes XML/unique names/Qt5 compiler/actual standalone uic loading and source-edit propagation). Malformed empty calibration_pair_picker item removed; no control/spacer removed.
- Real-shell runtime matrix PASS: 57 captures, 13 workspaces at 1366x900, 1366x768 and 1920x1080 plus Settings/Advanced/Alignment tabs. Zero callback errors, zero rendering-limit messages; shutdown complete with no live threads/processes/sockets/shared memory. Representative artifacts: [Overview](artifacts/overview.png), [Settings](artifacts/settings-gnss.png), [Baseline](artifacts/alignment-baseline.png), [Designer](artifacts/qt-designer.png), [summary](artifacts/visual-summary.json). Full set /tmp/workstation-designer-final. Native inactive/device-unavailable states only; not live hardware evidence.
- Visual review: compared baseline Overview/Camera captures with migrated shell; native appearance and live custom widgets retained. Existing long read-model text/Overview content clips at narrow central/Inspector widths in both baseline and migrated screenshots; this migration does not claim to redesign those pre-existing width limits. Page vertical scrolling remains available.
- Independent read-only review found Diagnostics tab reorder coupling to numeric indices. Changed navigation/refresh dispatch to widget identity; added an actual reordered .ui test. Final targeted regression pending. Other reviewed shell/Overview/Protocol contracts had no concrete regression.
- Full required regression running via `./tools/run_isolated_tests.sh -q --tb=short`; final targeted Designer/workspace/viewport/navigation/boundary regression running independently with separate test isolation.

- Final targeted integration initially 236 PASS / 1 FAIL: Baseline Yaw90 with real QImage fixture at 1366x768 placed table bottom at483px in453px viewport. Root cause extra Designer QWidget/layout around CameraPreview delayed initial size hints. Replaced placeholder directly in the .ui-owned layout (original hierarchy); no sizing formula change. Full Baseline+device form regression now38 PASS, including unchanged small/wide viewport assertions. [Small-screen fixture](artifacts/baseline-small-live-fixture.png).
- First monolithic full-suite attempt stopped progressing after1377 outcomes near NTRIP session tests. Interrupted own test process (SIGINT did not retire it; SIGTERM used); no hardware actions. Independent entire test_ntrip_session_runtime.py rerun PASS:11 tests in3.99s. Running the remaining full suite separately with faulthandler diagnostics; do not call the stalled single-process run PASS.

- FINAL Designer/workspace/viewport/navigation/boundary bundle PASS:237 tests in47.67s after the Diagnostics identity dispatch and Baseline direct-preview fixes. Command enumerates test_designer_forms, test_designer_device_forms, test_designer_engineering, test_designer_pixhawk_forms, test_settings_designer_form, test_inspector_designer_form, test_optical_pose_designer_form, test_workstation_ui, test_baseline_yaw90_workspace, test_navigation_auto_reveal and test_workspace_architecture via tools/run_isolated_tests.sh. Log /tmp/workstation-designer-final-fixed.log.
- Final portability repeat PASS and git diff --check PASS. New required assets ui/forms/*.ui, ui/designer.py and workspace_views/pixhawk_parts/designer.py are repository-local and currently untracked; no required runtime file outside SASL. New tests/docs/artifacts also untracked. Existing user changes and unrelated Simulation files remain untouched; no commit performed.
- Final form content identities retained in artifacts/form-sha256.json. Temporary migration scripts are not needed; the checked-in XML itself is the editable source.

- Broad regression exposed two remaining direct-parent contracts: Calibration's operation/preview tabs must share one camera container, and Alignment's OpticalPosePane must be root-layout item1. Restored both by direct replacement of Designer placeholders, retaining form-defined order/stretch. Full Calibration+viewparts PASS:78. Follow-up lifecycle/Baseline/Calibration/USB/device-boundary/camera suite PASS:253 in97.54s.
- USB full-suite failure reproduced a test-double readback race: the fake transport setter only logged the request while a status poll read the old radio value. Made the fake commit the same panel state as the real setter and forced a refresh before asserting USB remains selected; production transport logic unchanged.
- Corrected stale test source locations for device_runtime boundary modules and panel/panel_alignment.py, retaining original import and coordinate-label assertions. No device policy or coordinate semantics changed.
- The second broad run executed all2783 selected outcomes but session fixture cleanup hung in QWidget.close() (faulthandler stack at tests/conftest.py:104); known failures above were still present in that process. Terminated own stale process. Third final run includes all tests with the fixes and faulthandler diagnostics. No production code or test assertions altered to bypass cleanup.

- Final-source visual matrix repeated after all direct-parent fixes:57 captures PASS; callbacks=[]; rendering_limitations=[]; shutdown complete. Retained screenshots/summary/form hashes updated from /tmp/workstation-designer-final-v2.
- Skip diagnosis: decoded NV12 synthetic MP4 test is SKIPPED because FFmpeg h264_nvenc initialization returns CUDA_ERROR_UNKNOWN from cuInit(0). Independent exact test confirms1 skipped in2.47s. This is an unverified NVENC environment boundary, not evidence of recording/encoding success.

- Process cleanup confirmed: the two interrupted previous test processes and their observed multiprocessing child no longer exist. Qt Designer and final capture process exited0.

- FINAL all-tests completion:2793 passed,1 skipped in771.47s, exit0. The cleanup diagnostic was a long-running fixture cleanup, not a permanent final deadlock. No additional test batch script was created or run. Automatic approval timed out once while preparing a process inventory; the permitted shorter retry found no remaining process because full validation had finished.
- Acceptance A1–A6 reconciled above. User-authorized frontend migration complete; physical-device/NVENC boundaries explicitly unverified. No commit/push performed. All required files are inside the SASL tree.
