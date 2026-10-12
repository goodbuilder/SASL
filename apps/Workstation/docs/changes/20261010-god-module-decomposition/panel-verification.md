# G2 panel composition evidence

Source of truth: original Workstation behavior and the accepted G2 ownership plan. Prior uncommitted Run, worker recovery and recording fixes remain the starting behavior. This change does not alter geometry formulas, worker concurrency, wire commands or saved schemas.

- G2.1 / finite compatibility and behavior ownership — PASS. `standalone_gui.py` reduced from 13,702 to 319 lines; its constructor composes `PanelApplication`. The former 365 ordinary panel methods execute on eleven QObject feature owners. Qt overrides and existing controller accessors remain at the outer widget boundary. Named field properties reference one state/view owner; recording and measurement properties use their separate authoritative state objects. No unrestricted attribute proxy or panel mixin was introduced.
- G2.2 / view composition — PASS. `PanelShellParts` names shell layouts; each feature factory receives its required layouts and stores widget references in the owning view bundle. Initial values, visible labels and layout ordering are retained. The first characterization attempt exposed a comprehension-local variable erroneously returned between factories; corrected using lexical scope analysis before the passing run.
- G2.3 / explicit dependencies — PASS. Initial generated dynamic-name analysis incorrectly treated display f-strings as attribute patterns. Independent review caught this before acceptance. The corrected inventories expand only actual dynamic attribute operations, declare finite dependency families, and grant setters only to declared writable names. Features import original providers directly; `panel_support.py` contains 168 lines of constants and small helpers. The old import surface is isolated in `panel_compatibility_exports.py`.
- G2.4 / Qt and resource lifetime — PASS in automated Qt/fake environment. Feature owners are QObjects parented to the panel; QTimers/QThreads/widgets retain the QWidget parent. `test_panel_composition.py` uses a real QThread signal and confirms the moved callback executes on the GUI thread. It also proves independent panel state, one recording storage owner, shared backend/compatibility command identity, and visibility of feature-owned QThreads, measurement actor/queue/timer handles and recording native-writer maps. `AppLifecycleController` reads the composed live inventory rather than relying on `vars(panel)` alone.
- G2.5 / broad compatibility — PASS for the isolated focused command below: 198 tests passed in 22.52 seconds. Obsolete source-location/worker monkeypatch checks now inspect or patch the actual provider. Named `FeatureCommand` descriptors preserve per-instance command overrides; class-method calls with existing bare fixtures retain their outer compatibility path. Stub generation now writes `standalone_gui.pyi`, including both controllers and feature commands/fields.

```sh
apps/Workstation/tools/run_isolated_tests.sh \
  apps/Workstation/tests/test_panel_composition.py \
  apps/Workstation/tests/test_workstation_lifecycle.py \
  apps/Workstation/tests/test_workstation_camera.py \
  apps/Workstation/tests/test_workstation_calibration.py \
  apps/Workstation/tests/test_controller_delegation.py \
  apps/Workstation/tests/test_panel_delegate_bindings.py \
  apps/Workstation/tests/test_controller_method_registries.py \
  apps/Workstation/tests/test_gen_panel_delegate_stubs.py \
  --disable-warnings --maxfail=1 --tb=short -q
```

The preceding unisolated characterization exposed a lost test override and escaped a WIRIS worker; see the root verification ledger incident entry. That process exited, and later software validation uses the loopback-only namespace runner plus the fail-on-unmocked-network fixture. No hardware success claim is made.

Coordinate contract checked: extracted alignment/geometry/calibration/presentation keep GPS/RTK NED, PX4 body FRD attitude, OpenCV optical RDF, and public baseline-center FRD. The explicit camera-frame label is now checked in `panel_alignment.py`. Formula and CSV producer implementations retain their original expressions and field names.

Additional isolated settings/UI/backend contracts/GNSS protocol/runtime-camera characterization: **103 passed in 24.27s**. The initial broader attempt passed the Pixhawk/GNSS cases and stopped at one obsolete settings-wrapper monkeypatch; the check now patches its actual `panel_support` provider and the repaired follow-up passed.

Final isolated Pixhawk/GNSS plus ownership/typing-stub recheck: **110 passed in 8.82s**. All current backend command strings were also checked against the explicit feature/controller command inventories; no unregistered command exists.

## G2 integration correction: optional command ports

The broader Measurement suite exposed an integration FAIL after the focused panel checks: `test_real_panel_ignores_old_session_and_prepares_type_owned_paths` reached RUNNING but had empty video paths. Recording diagnosis found that `RecordingViewFeature._prepare_video_recording_paths()` uses optional `getattr` callbacks; three registered controller callbacks were omitted from its dependency list. The finite inventory generation had intersected literal strings with panel fields/direct feature methods but omitted the controller command registry.

The repaired audit includes literal attribute reads and actual dynamic attribute-key families against the complete controller registry. Twelve omitted read-only operation ports were added across recording view (3), camera (6), presentation (1), calibration (1), and diagnostics (1). No state copies or fallback proxy were added. New parametrized runtime coverage checks that these optional operations resolve callable read-only ports. The integrated Start path and affected panel/camera/lifecycle suites are being rerun in the isolated network namespace; prior focused PASS results do not supersede this integration failure until the rerun passes.

G2 optional-command correction **PASS**: isolated `test_panel_composition.py`, `test_measurement_run_controller.py`, `test_workstation_camera.py`, and `test_workstation_lifecycle.py` produced **136 passed in 16.63s**, including the unchanged formerly failing real-panel Start path. Recording owner independently reran that case plus recording/native/architecture neighbors: **56 passed in 3.81s**. The earlier empty-video-path FAIL is superseded by these observed results.

## Visual QA fixture callback lifetime

Final real-window capture exposed two deferred TypeErrors despite exit status zero and complete shutdown. Both appeared at the first event dispatch, with no Python traceback, for `StandalonePanelBackend` and `BackendStatusAdapter` being called. The QA script temporarily replaces the panel's two startup methods with inline lambdas, schedules the callbacks, then exits the patch scope before dispatch. An independent wrapper retaining the scheduled callable objects removed both TypeErrors while preserving all six captures and cleanup. No application code was changed; the correction belongs to the QA fixture callback lifetime.

A new isolated subprocess regression, `test_panel_visual_capture.py`, checks actual six PNG/context outputs and shutdown and rejects TypeError text even when the process exits successfully. It reproduced both TypeErrors against the unmodified fixture: **1 failed in 2.47s**. The root owner will retain stable named startup hooks in the QA script; a passing rerun is required before this visual gate closes.

Visual-fixture correction **PASS**: root changed the fixture to module-lifetime named startup no-ops and dispatches initial queued calls while patch scope is active. The root also reproduced the transient patched-callable failure in a minimal QWidget/QTimer example with no Workstation import. The subprocess regression now asserts `callback_errors == []`, validates any rendering limitation as the separately identified offscreen pyqtgraph/OpenGL condition, and verifies all six images plus resource cleanup. Isolated visual regression + composition tests: **12 passed in 7.29s**. No panel, backend or MainWindow product code was changed for this QA-only defect; this does not claim OpenGL rendering success.

## Maintenance write-ownership gate

Read-only, scope-aware source inspection covered 1,027 direct/literal writes across the eleven feature classes: every one was backed by its owned field inventory or an explicit writable port. Nested classes' `self` and nested functions shadowing `self` were excluded; closures capturing the feature receiver were included. The three computed write sites were reviewed separately. Calibration restores a fixed 19-field tuple already owned. Two diagnostics builders dynamically stored `px4_mag_tab_widget_{panel_key}` on the feature QObject; live construction confirmed raw `manager` and `detail` references outside the view bundle. The new no-stray-data assertion reproduced this violation (1 failed in 1.51s).

The narrow repair stores all such widgets in `DiagnosticsViews.px4_mag_tab_widgets`, retains named manager/detail/sensor_tab aliases backed by the same dictionary, and supports repeated/custom panel keys without new QObject attributes. The two direct registry/construction checks passed (2 passed in 1.15s). New architecture assertions cover all current direct/literal writes, validate the finite calibration rollback list, reject newly introduced computed receiver writes pending review, and verify receiver scoping against a small independent nested-owner example. The broader affected panel/PX4/visual regression is pending.

Tradeoff: the feature QObject remains extensible for Qt internals and supported explicit command overrides; production does not intercept every arbitrary Python assignment. Source-level writes and live composition are guarded in tests. Reflection outside these forms or future mutations through opaque helper objects still need normal ownership review.

Maintenance ownership repair **PASS**: isolated panel composition/write-scope/PX4 magnetometer/Pixhawk-GNSS/typing-stub/real visual-capture suite: **151 passed in 17.04s**. Dynamic PX4 tab references are now view-owned, known aliases share the same registry, and no constructed feature retains mutable data directly on its QObject. Coordinates and producer schemas were unchanged.

## RTCM replacement test lifecycle investigation

A final full-suite run reported one failure in `test_pixhawk_worker_is_replaced_when_subscriber_identity_changes`, then aborted later during GC. Its original failure traceback was lost to the abort; the exact original timing cannot be recovered from that log. Read-only production inspection found a valid immediate-retirement branch: `_retire_pixhawk_rtcm_worker(wait=False)` may finish immediately and `_ensure_pixhawk_rtcm_worker()` may return the replacement synchronously. The test unconditionally expected `None` and tracked the successor only after that assertion. Its cleanup also called `isRunning()` on a potentially deleted first wrapper before reaching the successor.

A deterministic isolated probe reproduced both hazards: valid synchronous replacement returned in 0.000165s (well below the unchanged 50ms admission limit); recreating the old cleanup list left the new QThread running; dispatching the retired wrapper's deferred deletion made its subsequent `isRunning()` raise `RuntimeError`. The safe outer probe cleanup stopped all resources. Repro source is retained in `evidence/rtcm_retirement_probe.py` and asserts that the original unconditional-None assertion fails under this valid ordering.

No production GNSS code change was warranted. The test now explicitly controls both immediate and pending retirement, still times admission against `< 0.05s`, and tracks every real QThread during construction before any assertion. Its cleanup cancels pending restarts, joins all created threads independently of deleted wrappers, and verifies finish signals. Separate injected-assertion cases cover failure before replacement checks. Initial targeted result: **4 passed in 0.20s**; broader RTCM followed by workspace-AST/GC-safe regression is pending.

RTCM replacement test repair **PASS**: isolated sequence `test_rtcm_pipeline_workers.py` → `test_workspace_architecture.py` → `test_panel_composition.py` produced **85 passed in 13.66s**. Both immediate and pending retirement have deterministic scheduling; the `< 0.05s` admission assertion is unchanged. Both injected early-assertion paths cancel deferred restart, join all constructor-tracked QThreads (including already-deleted wrappers), and explicitly run cyclic GC before later workspace AST parsing. Production `gnss_runtime_controller.py` was not changed. Repro source and observed JSON/pytest output are retained in `evidence/rtcm_retirement_probe.py` and `evidence/rtcm-retirement-probe.txt`; the reproduction explicitly verifies failure of the former unconditional-None assertion under a valid fast-retirement ordering.
