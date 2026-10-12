# Verification ledger

| Slice | Observable acceptance | Status |
|---|---|---|
| S0 | baseline/plan/coordinates/worktree mapped | PASS |
| S1 | deep immutability + canonical/spawn compatibility | PASS |
| S2 | executable CI + portable paths with negative checks | PASS |
| S3 | headless Run owner + preserved adapter flow | PASS |
| S4 | observable worker + explicit isolated recovery | PASS |
| S5 | current contracts/documentation agree | PASS |
| S6 | complete relevant regression + final audit | PASS |

## S0 / 2026-10-09
Source: user RCA remediation request; Workstation current code at 42b16e0.
Observed: Workstation clean; unrelated Simulation edits present; no root .venv. Coordinate contracts reviewed in core/canonical transform/recording schema.
Evidence: git status --short, source reads and prior RCA reproduction. Environment: source inspection.
Result: PASS for baseline identification; earlier test results are not fresh remediation evidence.
Next: current baseline targeted pytest is running; add failing contract probes before implementation.

## S1 / baseline and RED
- Baseline command: `PYTHONDONTWRITEBYTECODE=1 QT_QPA_PLATFORM=offscreen python3 -B -m pytest -p no:cacheprovider -q apps/Workstation/tests/test_method_evaluation.py apps/Workstation/tests/test_measurement_run_controller.py apps/Workstation/tests/test_portable_path_policy.py` from repository root: **87 passed** (13.37 s).
- New independent alias regression: `... pytest -p no:cacheprovider -q --tb=line apps/Workstation/tests/test_method_payload_immutability.py`: **5 failed, 1 passed** on unchanged product code. Snapshot source/export mutation, direct nested mutation, request/outcome/bundle input aliasing fail. Real spawn wire transport passes.
- The first regression draft accidentally allowed expected request data to alias the same input; deep-copying the oracle before mutation exposed the fifth failure. No product change was made before the corrected RED.
- Environment: unit plus real spawned Python child; no devices. Source of truth: documented immutable boundary, hand-authored JSON and independent hashlib digest. Status: FAIL (expected baseline), next implement recursive ownership and detached exports.

## S1 / serializer integration finding
New alias tests passed after freezing; dependent method/reproduction tests exposed 50 failures (32 passed) because MeasurementAttempt's Mapping-annotated canonical JSON helper passed the new immutable Mapping directly to json.dumps. This is a representation integration failure, not a coordinate/formula change. Keep S1 FAIL until all affected readers serialize the declared Mapping contract into ordinary JSON values. No dependent implementation slice has started.

## S1 / GREEN
- Command: `PYTHONDONTWRITEBYTECODE=1 QT_QPA_PLATFORM=offscreen python3 -B -m pytest -p no:cacheprovider -q --tb=short apps/Workstation/tests/test_method_payload_immutability.py apps/Workstation/tests/test_method_evaluation.py apps/Workstation/tests/test_measurement_workbook_reproduction.py apps/Workstation/tests/test_integrity_hardening_end_to_end.py`.
- Result: **82 passed** (12.86 s). Alias regression now 6/6 passes, including real spawn; independent hand-authored payload/hash and numerical geometry/workbook reproduction remain valid.
- Changed mechanism: immutable owned Mapping + tuple trees, detached JSON exports, Mapping-aware canonical serializer. Output schema/axis order/float values/hash encoding preserved.
- Status: PASS for S1. Full Workstation regression remains S6. Hardware timing/accuracy not exercised.

## S2 / baseline
- Current policy inspection reproduces 20 pattern matches in 7 Workstation files. All seven contain historical source provenance or execution evidence; no runtime reader of P1 source_settings was found in the prior RCA. Original bytes will remain intact.
- CI still names deleted test_workstation.py. Next add regression for current CI directory and narrow immutable-content classification, then correct the gate.

## S2 / RED -> GREEN
- New gate regressions first returned **2 failed**: missing CI path and unclassified historical endpoint bytes.
- Current CI now names `apps/Workstation/tests`; canonical shared coordinate/stereo tests remain included.
- Seven historical files are exempt only at their recorded SHA-256. Original evidence bytes unchanged. Modified bytes and adjacent/new files remain checked.
- `... pytest -p no:cacheprovider -q apps/Workstation/tests/test_quality_workflow.py apps/Workstation/tests/test_portable_path_policy.py apps/Workstation/tests/test_portable_paths.py`: **14 passed** (1.23 s).
- `PYTHONDONTWRITEBYTECODE=1 python3 -B tools/check_portable_paths.py`: **PASS**.
- Actual new CI targets with `--collect-only`: **2156 tests collected** (5.56 s), exit 0. Temporary output: `/tmp/workstation-rca-ci-collection.log`; aggregate result retained here. Full execution follows in S6.
- Environment: local system Python, no devices. Status: PASS for gate wiring/classification; no remote GitHub run claimed.

## S3 / RED and state decision
`... pytest -q --tb=short apps/Workstation/tests/test_measurement_run_service.py` fails collection because the Qt-free Run service does not exist. New tests define independent state/source/alias/stale-generation/headless criteria.
Source inspection also found Start leaves public state IDLE during optional registry callbacks. The service will reserve STARTING before optional I/O, preventing reentrant duplicate Start; this is an existing public enum and introduces no new persisted schema. Update the prior test's incidental IDLE expectation and add a reentrant Start regression. Current source-owner calls remain advisory to observation.

## S3 / layer evidence and adapter integration
- Pure service tests: **4 passed**, including a fresh child process that imports no Qt/GUI/recording-controller modules.
- First adapter regression: **58 passed, 1 skipped, 2 failed**. The failures identified the planned STARTING-before-I/O change and a camera fake that had no source-to-Run handoff. The test now uses the real MeasurementRunController source contract, retains peer-camera isolation checks, and asserts authoritative source state.
- Added reentrant registry Start rejection and projection-tampering tests. Multiple compatibility adapters share one service; projection fields never decide lifecycle transitions. Camera retirement publishes source facts instead of writing Run private fields.
- S3 remains pending until expanded adapter/lifecycle/settings regression passes.

## S3 / GREEN
- Command: `... pytest -p no:cacheprovider -q --tb=short apps/Workstation/tests/test_measurement_run_service.py apps/Workstation/tests/test_measurement_run_controller.py apps/Workstation/tests/test_observation_first_measurement.py apps/Workstation/tests/test_camera_stream_recovery.py apps/Workstation/tests/test_workstation_lifecycle.py apps/Workstation/tests/test_workstation_settings.py`.
- Result: **123 passed, 1 skipped** (19.97 s). Pure/headless, real panel, source failure isolation, storage exceptions, repeated runs, old completion, state projection tampering, and reentrant Start covered.
- Before: controller read/wrote panel state and camera controller wrote Run active_sides. After: Qt-free service owns immutable state; adapter only publishes compatibility projections; camera publishes source transitions. STARTING reserves the run before optional registry I/O.
- Status: PASS for affected ownership and flows. Whole UI decomposition is not claimed; remaining compatibility consumers are documented and may migrate independently.

## S4 / worker RED -> core GREEN
- New recovery tests on baseline: **2 failed**, missing status/restart API.
- Worker plus existing method tests: `... pytest -p no:cacheprovider -q --tb=short apps/Workstation/tests/test_method_worker_recovery.py apps/Workstation/tests/test_method_evaluation.py`: **75 passed** (13.44 s).
- Real spawn oracle: Horizon fixture announces start, ignores SIGTERM and remains alive; explicit restart returns within 250 ms, actor escalates to kill, original request emits FAILED/operator_restart, queued next request completes, primary worker generation remains unchanged, duplicate request rejected, shutdown closes.
- Process exit confirmation precedes replacement. Replacement launch failure remains method-local and retryable; no automatic compute deadline added. Cached status is published at most every 100 ms except terminal outcomes.
- S4 core PASS; UI/adapter/persisted evidence and adversarial recovery cases remain required.

## S4 / UI and adversarial boundary progress
- UI/adapter plus existing UI/rebuild regression: **37 passed** (8.38 s), including displayed identity, queued-vs-completed status, stable geometry/focus/selection, malformed state and original-run event ownership.
- Added actual click -> backend -> controller -> actor/child -> old-run disk evidence test. First integration run: **7 passed, 1 failed** because its controlled registry fixture omitted record_for; fixture corrected to the actual registry interface. This did not identify a production failure.
- Adversarial core tests cover actor-side identity recheck, completion racing recovery, unconfirmed termination (no replacement), and spawn failure with retained queued work and explicit retry.

## S4 / vertical GREEN
- `... pytest -p no:cacheprovider -q --tb=short apps/Workstation/tests/test_method_worker_recovery.py apps/Workstation/tests/test_method_worker_recovery_ui.py`: **8 passed** (2.55 s).
- The real Qt click traverses StandalonePanelBackend -> MeasurementViewController -> coordinator -> actor -> SIGTERM-ignoring child -> confirmed replacement. Qt's 10 ms timer continues ticking during the kill fallback. Existing result projection is controlled in this fixture; terminal records and recovery events use the actual MeasurementDataStore.
- After store.close/flush, independent JSON reads find one RESTARTED event and original FAILED/operator_restart outcome in the old run, no event in the active newer run, and unchanged newer generation.
- Fake negative tests prove stale/completed identities do not stop a different job, unconfirmed exit does not create a replacement, failed spawn retains queued work and supports explicit retry.
- Status: PASS for recovery core/adapter/UI/disk ownership. Offscreen screenshot review and full suite still follow. The S3 skip is the optional NVIDIA/GStreamer hardware test, not the modified Run logic.

## S5 / current documentation
- Added docs/MEASUREMENT_RUNTIME_CONTRACT.md with authoritative lifecycle, immutable payload, method execution, recovery and developer checks; linked README/workflow/schema/product command/status docs.
- Corrected MAG advisory admission, current ten-method registry, submitted-method bundles, retired compute deadlines and workbook checkpoint behavior in active sections. Historical Change Log remains explicitly historical.
- New doc/CI regressions: **3 passed**. Registry IDs are present in the runtime contract/schema; obsolete active timeout/MAG-gate claims are rejected.
- Coordinate review: all saved coordinates, axes, units and time bases remain unchanged; only JSON ownership and new operational recovery events are affected.

## S6 / full regression and final review — 2026-10-10

Full command (repository root):

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 QT_QPA_PLATFORM=offscreen python3 -B -m pytest -p no:cacheprovider -q -ra --tb=short apps/StereoWIRIS/tests/test_coordinate_contract.py apps/StereoWIRIS/tests/test_stereo_geometry_common.py apps/StereoWIRIS/tests/test_triangulation.py apps/Workstation/tests
```

- **2169 passed, 2 skipped, 1 warning in 228.43 s**, exit 0. Session 8857 is terminal and closed.
- Skips: NVENC fixture below encoder minimum dimensions; optional pyproj absent. Warning: system NumPy 1.26.4 outside system SciPy's declared <1.25 range. These are visible environment/fixture limitations, not converted to passes.
- Final review found read_text would normalize CRLF before exact historical hashes. Changed only the portability reader to bytes.decode and added an actual CRLF-tampered-file regression. Final affected quality/policy/portable-path selection: **16 passed in 1.15 s**. No runtime source changed after the full suite.
- `git diff --check`, `tools/check_portable_paths.py`, `tools/check_repository_policy.py --allow-existing-debt`: PASS.
- Real shell offscreen visual inspection at 1366x900 and 1366x768: PASS for added control access, readability and native component consistency; files under evidence/. State/focus/geometry sequences were automated in S4.
- Additional deployment observation: `./apps/Workstation/run.sh --check` exits 1 because canonical root .venv/bin/python was already absent. No environment installation or hardware qualification is claimed; this baseline condition is separate from the correction's software acceptance criteria.
- Fresh source review: no frame/value/schema changes, canonical serialization compatible, no automatic compute timeout, explicit restart validates displayed and actor identities, replacement requires confirmed exit, current/old Run evidence remains separate.
- New required source/tests/docs are under the repository and not ignored; they remain untracked until commit. Unrelated concurrent Simulation edits were preserved. No commit/push performed.

## Completion audit R1–R7

R1 PASS: independent alias/hash/spawn and reproduction probes; R2 PASS: CI directory contract, actual collection and full execution; R3 PASS: original-byte hashes, changed/adjacent/CRLF negative cases and real gate; R4 PASS: Qt-free owner, reentrant/late/storage/source tests and full regression; R5 PASS: real child/Qt action/disk evidence plus adversarial failures; R6 PASS: current contract/README/workflow/schema/product docs and drift tests; R7 PASS: fresh evidence and report.md before/after/observed effects, explicit limits and git audit. No required software correction remains open.
