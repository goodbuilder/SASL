# Verification ledger

| Slice | Required evidence | Status |
|---|---|---|
| S0 | current baseline/coordinate/ownership audit | PASS |
| S1 | view DAG/isolated imports/compatibility/UI regression | PASS |
| S2 | recording owner/ledger/schema/lifecycle regression | PASS |
| S3 | measurement pure/core/commit/render separation | PASS |
| S4 | feature-owned panel state/resources/composition | PASS |
| S5 | integrated graph and current docs | PASS |
| S6 | full behavioral + architecture regression/final audit | PASS (software scope; environment limits below) |

## S0
Current HEAD 42b16e0 plus authorized RCA changes. The previous 20-file software inventory matches exactly. AST baseline captures line/class/function counts and hashes in baseline-inventory.json. Root AGENTS coordinate/portability rules reviewed. No deeper Workstation AGENTS. All four independent design audits complete; source modification starts after this plan. No hardware operation requested.

## S1 / view ownership extraction
- `workspaces.py`: 17,551 -> 147 lines, explicit import compatibility only. Nineteen concrete view/helper modules, 21 local dependency edges, no cycles. Fifty-four original class/function AST definitions preserved exactly; RCA recovery control/wiring retained.
- Production MainWindow and BaselineYaw90 now import concrete owners; leaf imports do not load unrelated screens or compatibility facade. Public class/alias identity and Data recovery UI handoff have architecture tests.
- Agent focused evidence: architecture + baseline yaw + target coordinates + live RT + photo-time UI **30 passed (3.43 s)**; broader calibration/quality/capture/selection, optical, Pixhawk, telemetry, measurement and shell/recovery UI **282 passed (25.60 s)**. Owned-file diff whitespace check passed.
- Five test files changed only implementation-global monkeypatch targets to the new owner modules; assertions preserved. Independent reviewer and final integration remain separate gates.
- Status: PASS for S1 focused behavior/graph, pending integrated S5/S6. Largest cohesive view module is calibration (~4,136 lines); size alone is not the architecture oracle.

## S1 / independent review and expanded behavior
Independent read-only review found no missing globals, changed view bodies, production facade back-imports or cycles. It requested explicit compatibility exit criteria (now compatibility-exit.md) and all-owner isolated-import coverage; test expansion is in progress.
Additional camera/Pixhawk/network/time-sync/optical controller regression: **301 passed (22.72 s)**, one known SciPy/NumPy environment warning. S1 focused total 613 passes across three selections (do not add to later full-suite totals). Agent portability check passed. Final integration remains pending while other owners are being wired.

## Integration safety finding / external test I/O
A panel compatibility override was not reaching its moved feature command. A camera test therefore created a real WirisRtspEnableWorker and later crashed during teardown (pytest exit 139). The captured stack was blocked in socket.create_connection targeting the configured default cameras; no successful connection/command response was found. Device receipt before the trace cannot be established. The test process has exited and the agent confirmed no task-owned worker process remained. Do not claim that absence of response proves no device receipt.

The finite named override bridge was corrected. Further tests use tools/run_isolated_tests.sh: an unprivileged user/network namespace with only loopback enabled, inherited by subprocesses, plus an opt-in pytest non-loopback-attempt guard. `unshare --user --map-root-user --net true` succeeds in this environment. This prevents recurrence through network transports without changing host networking or camera settings. Existing network fakes must still intercept intended calls; attempted real non-loopback I/O is itself a test failure. This is network isolation, not a claim of hardware qualification.

## S1 / final architecture coverage
All 19 screen/helper modules are now imported in separate processes and checked against their expected transitive dependencies. Graph collection covers absolute/relative Import and ImportFrom forms; negative fixtures expose facade back-imports and a hidden cycle. Updated architecture tests: **34 passed (3.49 s)**. Root reran this gate through loopback-only network isolation.

## S2 / recording branch acceptance
`recording_controller.py` 8,676 -> 65 lines; twenty-two cohesive behavioral owners, explicit finite capability ports and nine recording state groups. Native completion/primary admission share one ledger and lock; original rows/leases, first-terminal outcomes, bounded retries and rollback object identities preserved. Core owners do not receive a panel/controller.

Recording agent final command: `apps/Workstation/tools/run_isolated_tests.sh -q apps/Workstation/tests/test_{recording_owner_architecture,recording_controller_save_lifecycle,measurement_canonical_evidence,recording_shutdown_integrity,recording_hr_evidence_integration,recording_internal_artifacts,gnss_baseline_center_recording,mavlink_nonfinite_timeline,pnp_hr_capture,stage5_extracted_methods,target_coordinate_storage,target_coordinates_ui,native_evidence_writer,recording_raw_artifacts,optical_pose_session_persistence,observation_first_measurement,measurement_methods}.py --tb=short --maxfail=3`.
Result: **201 passed (14.24 s)**. Actual save success/failure/finished callbacks execute on panel.thread(). Independent owner construction, read-only operation ports, state identity across facade recreation and resource visibility are covered. Portability, source compilation and scoped whitespace gates pass. Independent review and integrated S5/S6 are still required.

## S3 / pure measurement contract checkpoint
Isolated `test_measurement_service_boundaries.py`: **7 passed**. The pure resolver accepts FrozenPairInputs/FrozenGeometryInputs without widgets, ports or live clock. Mutating focal length/zoom/manual mode/calibration/selection after capture preserves canonical attempt and plan. Independent numeric oracle: 900 px * 0.4 m / 40 px = 9 m; left optical RDF (0.2,0,9) becomes center FRD (9,0,0). State ownership and worker-resource visibility tested. Broader measurement regression pending.

G2 detailed evidence: [panel-verification.md](panel-verification.md). Isolated focused panel/Qt/lifecycle/camera/calibration/compatibility suite: **198 passed** in 22.52s. Independent decomposition review found no remaining G2 blocker after finite ports, direct imports and owner inventory corrections. Additional settings/Pixhawk/GNSS/UI characterization is in progress.

G2 additional isolated settings/UI/backend/GNSS protocol/runtime camera suite: **103 passed in 24.27s** after updating the settings-wrapper monkeypatch to the extracted provider. No non-loopback attempt was admitted.

G2 integration FAIL discovered: real Measurement start had empty video paths because optional recording allocation callbacks were absent from a feature port list. Recording diagnosis confirmed this was a panel composition defect. Twelve omitted controller-operation ports repaired across five features; runtime callback-availability/read-only regression added. Isolated integrated Start + affected panel/camera/lifecycle rerun pending. See panel-verification.md.

G2 optional-command correction **PASS**: isolated `test_panel_composition.py`, `test_measurement_run_controller.py`, `test_workstation_camera.py`, and `test_workstation_lifecycle.py` produced **136 passed in 16.63s**, including the unchanged formerly failing real-panel Start path. Recording owner independently reran that case plus recording/native/architecture neighbors: **56 passed in 3.81s**. The earlier empty-video-path FAIL is superseded by these observed results.

## S3/S4 / integrated findings and corrections
- Measurement broad suite first returned **458 passed, 1 failure**: real-panel Start created the right run root/prefix but no video paths. Root cause: moved RecordingViewFeature omitted the optional `allocate_measurement_video_segment` dependency; it observed None and stored an empty map. Panel audit found 12 omitted named controller callbacks and added them as read-only operation ports. Original expected left/right paths were retained.
- Correction evidence: isolated panel/Run/camera/lifecycle **136 passed (16.63 s)**; independently rerun failed test plus recording/native neighbors **56 passed (3.81 s)**. No recording allocator/state change was required.
- Independent G4 review found default attempt resolution had bypassed preferred/fallback/hysteresis selection. Existing selection now runs once at capture; chosen method/instance is frozen, with no first-ready fallback. New preferred-second/no-selected/selection-error regressions pass. Final isolated pure-boundary suite **10 passed (0.33 s)**; related methods/evaluation run **142 passed (11.54 s)**. G4 review accepts pure resolver/planner and nested inputs; no live widget/settings/clock reads after capture.
- G2 branch evidence: isolated **198**, **103**, **110**, **30** and integrated **136** test selections passed (overlap; do not sum as full coverage). Root panel is 319 lines, constructor approximately 11 lines. Eleven QObject features own 365 moved methods and feature state/views; callbacks retain Qt GUI affinity. Native and measurement resource inventories are aggregated. Details: panel-verification.md.
- Independent G3/G2 review accepted narrowed capability ports, shared ledger/lock and original row leases, direct provider imports, named state storage and explicit resource visibility. Architecture review covers 244 production modules; no new top-level feature cycle was found.

## S5/S6 / candidate integration started
Candidate Python inventory is recorded in candidate-inventory.json. All branches report source stable. Full command from repository root: `apps/Workstation/tools/run_isolated_tests.sh -q -ra --tb=short`. Output currently captured in bounded temporary log `/tmp/workstation-god-module-full.log`; result is pending and not a pass claim.

## S6 / runner scope correction
First full invocation ended during collection (exit 2), not during product tests: passing only `-q -ra --tb=short` bypassed the runner's no-argument defaults, so pytest selected all repository testpaths and collided on identical StereoWIRIS/Workstation test basenames. Corrected the runner to override default pytest testpaths explicitly while leaving explicit caller paths authoritative. This preserves the intended Workstation+shared-geometry scope for options-only runs. Collection and full execution are being rerun; no test file renamed, skipped or removed to hide this error.

## S5 / architecture documentation and scope gate
Current ownership/dependency/resource contracts are documented in docs/ARCHITECTURE.md and linked from README and the historical UI implementation plan. Compatibility exit criteria are explicit. The final measurement branch broad isolated run passed **461 tests (30.85 s)**, including the corrected real-panel video path.

Corrected runner's options-only collection selects the intended scope: **2231 tests collected in 3.53 s**, exit 0. Full isolated execution restarted on the same stable candidate; this is a new execution after a confirmed terminal collection error, not a duplicate live job.

## S6 / full isolated regression PASS, visual callback issue still open
Full isolated Workstation+shared geometry execution: **2229 passed, 2 skipped, 1 warning in 212.86 s**, exit 0. NVENC is unavailable in this isolated user namespace; optional pyproj is absent. Known system NumPy/SciPy version warning remains.

The real-panel/shell screenshot smoke produced six views and a complete empty-resource shutdown, but emitted `TypeError: 'StandalonePanelBackend' object is not callable` and the analogous BackendStatusAdapter message during initialization. Offscreen OpenGL also reported an environment limitation. The callback TypeErrors are being investigated separately; do not accept visual/runtime completion solely from process exit0 or the passing regression suite.

## Visual callback diagnosis
The two TypeErrors were caused by the QA fixture restoring transient patch lambdas before Qt dispatched zero-delay startup slots. A minimal independent QWidget/QTimer/unittest.mock program with no Workstation imports reproduced `QObject object is not callable` and a segmentation fault. Retaining callback objects removed the errors in the real capture as well. Product code was not changed for this fixture fault.

The capture tool now uses a module-lifetime named no-op and dispatches initial callbacks within patch scope. It records/fails unexpected Qt callback exceptions; the specific pyqtgraph offscreen OpenGL capability exception is recorded separately as a rendering limitation, never a passing 3D-render claim. A fresh capture and regression are in progress.

Visual fixture callback lifetime: new subprocess test was RED on the original transient lambdas and GREEN after root-owned stable-hook correction. Isolated visual-capture + panel-composition regression: **12 passed in 7.29s**; no unexpected callback errors, complete resource cleanup. Offscreen GL limitation remains a separate summary field, not a product success claim. No product code correction required.

Maintenance ownership repair **PASS**: isolated panel composition/write-scope/PX4 magnetometer/Pixhawk-GNSS/typing-stub/real visual-capture suite: **151 passed in 17.04s**. Dynamic PX4 tab references are now view-owned, known aliases share the same registry, and no constructed feature retains mutable data directly on its QObject. Coordinates and producer schemas were unchanged.

## Final ownership maintenance gate
Scope-aware audit covered 1,027 direct/literal feature writes. A real dynamic-write gap remained: two PX4 tab-building sites stored computed references directly on DiagnosticsFeature. They now store references in DiagnosticsViews.px4_mag_tab_widgets, with known legacy aliases pointing to the same registry and custom keys supported. Live construction regression was RED then GREEN. Static assignment/setattr and no-stray-data checks prevent this regression; QObject remains extensible for Qt/explicit command overrides, not falsely claimed runtime-sealed.

Combined isolated panel/PX4/visual/typing checks after correction: **151 passed (17.04 s)**. This is a production ownership correction after the first full pass, so a final full isolated suite has started on a new candidate inventory. The previous candidate inventory is retained in evidence/first-pass-candidate-inventory.json.

## Visual parity evidence
The corrected real-panel capture has zero callback_errors and complete empty-resource shutdown. The same tool rendered preserved pre-decomposition source with unchanged common code. Five views (Overview, Camera, Measurement, Calibration, Advanced) are exactly pixel-identical at 1366x900. Data differs only in the Inspector's temporary storage-root text (2,089/1,229,400 pixels, bbox 1234,536,1366,654); both images were visually compared. evidence/visual-comparison.json records raw comparison, without masking/rewriting images. Existing narrow-window clipping is preserved, not represented as a new visual improvement. Both before/after runs report the same offscreen OpenGL limitation; no hardware/3D-render qualification is claimed.

## Final rerun FAIL / Qt aggregate teardown investigation
After the final dynamic-view ownership correction, the full isolated rerun terminated with exit134 during garbage collection in the workspace AST architecture test, after one earlier failure marker around67%. Several earlier test-created data/workbook threads were still live in the captured stack. This supersedes any completion claim based on the first full pass. The prior failure must be identified and the aggregate Qt/resource lifetime failure explained before acceptance; tests will not be skipped or garbage collection disabled to force green.

RTCM replacement test repair **PASS**: isolated sequence `test_rtcm_pipeline_workers.py` → `test_workspace_architecture.py` → `test_panel_composition.py` produced **85 passed in 13.66s**. Both immediate and pending retirement have deterministic scheduling; the `< 0.05s` admission assertion is unchanged. Both injected early-assertion paths cancel deferred restart, join all constructor-tracked QThreads (including already-deleted wrappers), and explicitly run cyclic GC before later workspace AST parsing. Production `gnss_runtime_controller.py` was not changed. Repro source and observed JSON/pytest output are retained in `evidence/rtcm_retirement_probe.py` and `evidence/rtcm-retirement-probe.txt`; the reproduction explicitly verifies failure of the former unconditional-None assertion under a valid fast-retirement ordering.

## Final rerun diagnosis and independent boundary correction
The first failing case was identified by collection/order as `test_rtcm_pipeline_workers.py::test_pixhawk_worker_is_replaced_when_subscriber_identity_changes`. The abort prevented pytest from printing its original assertion traceback; the exact branch in that original run cannot be asserted. Isolated single execution passed. A full `-x -vv --tb=long` repeat also passed **2234 tests,2 skipped,1 warning in231.40s**, exit0. This is consistent with a scheduling-sensitive test, and predates the corrections below.

The RTCM owner investigator deterministically reproduced two real test defects without changing GNSS production code: immediate retirement legitimately returns a new worker (old test unconditionally required None); the resulting early failure did not track/stop the successor. Separately, accessing a retired/deleted QThread wrapper can raise RuntimeError and bypass later cleanup. New tests control immediate and pending retirement, preserve <50ms admission, and track all created threads before assertions with failure-safe cleanup. Their final results are recorded below when available.

Final independent source review also found one forbidden feature→facade import: RecordingViewFeature imported generation-ID creation from recording_controller rather than recording.contracts. Corrected to the identical concrete owner and added a feature import-direction gate with negative syntax fixtures. Isolated panel-composition+Run regression: **35 passed in6.53s**. The first command used an incorrect relative test path and exited4 before collection; corrected repo-relative arguments produced this pass.

Documentation now distinguishes Qt parent/context handles from unrestricted business-state access and shutdown-report active-resource lists from the persistent ownership inventory. The final full candidate will include these corrections and RTCM regressions.

## Final acceptance — stable candidate PASS
Command: `apps/Workstation/tools/run_isolated_tests.sh -q -ra --tb=short`. **2243 passed,2 skipped,1 warning in237.52s**, exit0 including session teardown. This is the final candidate after the RTCM race/cleanup regressions and concrete-owner import gate. No test was removed or skipped to hide the intermediate failure; the two capability skips pre-existed. Final source inventory contains429 active Python/stub/shell files, SHA256 `18da044d40de67a33819fe488cf52846a6656f20747cb8860df4abb93bb97e97`; all source hashes matched again after execution.

Raw final output: [evidence/final-regression.log](evidence/final-regression.log). Intermediate abort output is retained as immutable history in evidence/intermediate-aborted-regression.log. The exact earlier assertion traceback was never printed; the deterministic race/cleanup proof and successful controlled failure paths remain the evidence for the repair. Source identity and acceptance metadata: [evidence/final-acceptance.json](evidence/final-acceptance.json).

Final repository gates: `python3 tools/check_portable_paths.py`, `python3 tools/check_repository_policy.py --allow-existing-debt`, `python3 apps/Workstation/tools/gen_panel_delegate_stubs.py --check`, and `git diff --check` all PASS. Required429 active-source inventory paths were checked against git ignore rules: none ignored. New source/packages/tests/tools/docs remain uncommitted within SASL; unrelated Simulation changes were preserved.

Limits: NVENC unavailable inside isolation and optional pyproj absent (2 skips); installed SciPy warns about system NumPy1.26.4. Fresh official `./apps/Workstation/run.sh --check` exits1 because the pre-existing canonical root .venv is absent. Offscreen OpenGL and physical devices/field-duration qualification are outside this software acceptance. Six-view parity and real composition/start/stop/save/recovery regressions are the observed operator-flow evidence.

G1–G6 software acceptance is complete: concrete view DAG,11 feature owners/owned state,recording state+ledger/narrow ports,pure frozen measurement core,behavior/coordinate compatibility,and integrated architecture/regression gates. Expected maintenance effects and explicit remaining large modules/compatibility boundaries are described in report.md.
