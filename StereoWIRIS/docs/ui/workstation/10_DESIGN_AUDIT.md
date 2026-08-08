# StereoWIRIS Workstation UI Design Audit

- Status: Completed audit specification; post-audit implementation recorded in 11_DESIGN_AUDIT_RESULT.md
- Depends on: 00–09 design documents

Audit result and implementation update (2026-08-08):
[`11_DESIGN_AUDIT_RESULT.md`](11_DESIGN_AUDIT_RESULT.md) records the code-backed baseline
findings, the implemented launcher/lifecycle/status/command/evidence adapters, and assigns
**GO WITH CONDITIONS** to Phase 1 pending hardware and full Qt lifecycle validation.

Post-audit implementation also exposes source-backed Alignment/Pixhawk/GNSS, Calibration,
and Recording/Review/Export evidence. Advanced actions remain legacy-owned; the Workstation
shows their read-only status/profile/session projections without duplicating measurement,
recording, calibration, or coordinate logic.

## 1. Audit Role

Act as an independent senior software architect, Qt engineer, and adversarial UI reviewer. Treat design documents as hypotheses, not truth.

## 2. Audit Against

1. actual code
2. runtime behavior
3. workflows
4. migration risk
5. state ownership
6. command ownership
7. data semantics
8. hardware-dependent behavior
9. regression risk

## 3. Required Code Inspection

Inspect current equivalents of:
- main.py
- gui.py
- camera_pane.py
- measurement_widgets.py
- measurement_controller.py
- measurement_attempt.py
- pixhawk_attitude.py
- px4_mag_manager.py
- rtk_geometry.py
- recording_session.py
- settings.py
- network.py
- calibration.py
- alignment_managers.py
- sync.py
- theme.py

Do not rely on filenames or comments alone; inspect actual classes, handlers, timers, threads, state variables, cleanup paths, and data ownership.

## 4. gui.py Coupling Audit

Classify logic into:
- presentation
- command logic
- business logic
- lifecycle
- state ownership
- file/configuration ownership

Find any UI widget/label text that acts as application state truth.

## 5. Module Classification

A — Pure backend
B — Backend with Qt dependency
C — Reusable UI component
D — UI + business logic mixed
E — Lifecycle/resource owner
F — Unknown/runtime verification required

D/E are migration hotspots.

## 6. Data-Flow Audits

### Camera
Source → receiver/decoder → frame ownership → UI → FPS/state. Distinguish connection, RTSP availability, decoder state, actual frame reception, frame timestamp, FPS, preview state.

### Pixhawk
PX4/MAVLink/ROS2 → receiver → parser/model → UI. Verify connection semantics, heartbeat, attitude, timestamps/freshness, multiple transport paths.

### GNSS/RTK
Receiver → fix/RTK → baseline/geometry → UI. Preserve NO DATA/NO FIX/FLOAT/FIX/stale semantics.

### Detection
Model/runtime → inference → target → UI/measurement. Separate configured/model-loaded/enabled/running/target-present/output-fresh.

### Measurement
camera/detection + timing + attitude + calibration + baseline + geometry → attempt → validation/rejection → final result → UI/recording. Audit frame, units, timestamps, rejection reasons, quality, stale semantics.

### Recording
start request → validation → session creation → active → stop → finalize → saved/error → shutdown. Identify authoritative state owner and asynchronous lifecycle.

## 7. Command Audit

Trace actual paths for camera connect/disconnect, preview start/stop, detection enable/disable, recording start/stop, save/export, calibration, Pixhawk connect, and GNSS/RTK actions.

For each: handler, backend path, preconditions, async behavior, success evidence, failure evidence, state update, cleanup.

## 8. State Ownership Audit

For Camera L/R, FPS, P1/P2, attitude freshness, GNSS, RTK, detection, target, measurement, latest measurement, rejection reason, recording, active session, calibration identity classify:
- AUTHORITATIVE
- DERIVED
- UI-OWNED
- DUPLICATED
- UNKNOWN

UI-OWNED/DUPLICATED require findings.

## 9. Timing Audit

Identify source clock, unit, origin, sample/receive/processing/display time, monotonic vs wall clock. Never equate repaint time with data freshness.

## 10. Coordinate / Unit Protection

For position, baseline, attitude, roll/pitch/yaw, range, pixel coordinates, timestamps record frame, unit, axis convention, rotation convention, reference, timestamp where applicable. Do not normalize semantics without evidence.

## 11. Navigation / Workspace / Inspector Audit

Classify proposed navigation/workspaces as KEEP/MERGE/MOVE/REMOVE/DEFER. Ask whether a dedicated workspace is truly needed or whether Inspector/Diagnostics/local tab is better.

Inspector settings should be classified FREQUENT/OCCASIONAL/ADVANCED/WORKFLOW-SPECIFIC/READ-ONLY.

## 12. Diagnostics Audit

Only promise tabs backed by real data sources. Diagnostics is evidence, not a new source of truth.

## 13. Threading / Coexistence / Shutdown

Audit QThread/threading/queues/timers/callbacks, worker ownership, one-consumer queues, shared mutable state, UI thread affinity, blocking calls, shutdown order. Safe default: one operational UI per run via `--ui legacy` or `--ui workstation`.

Shutdown regressions are HIGH; data-loss risk may be CRITICAL.

## 14. Settings / Reusable Widgets / Theme

Find authoritative settings sources, reusable existing widgets, and current theme/QSS. Prefer adapters/extension over duplicate state stores or a second visual system.

## 15. Density / Screen Sizes

Validate 1920×1080, 1600×900, 1366×768 assumptions. Ensure redesign does not lose density without reason.

## 16. Workflow Audit

Reconstruct actual Startup, Camera, Measurement, Recording, Failure, Calibration workflows from code and test how the proposed architecture supports them.

## 17. Two-Day Feasibility

Rate shell, navigation, overview, status, camera workspace, measurement workspace, recording, diagnostics, inspector, launcher as GREEN/YELLOW/RED.

If needed, define a smaller truthful MVP rather than faking completeness.

## 18. Findings Format

UI-AUDIT-XXX
- Severity: CRITICAL/HIGH/MEDIUM/LOW
- Category
- Design Claim
- Evidence
- Code Location
- Why It Matters
- Failure Scenario
- Recommended Correction
- Verification

HIGH/CRITICAL require concrete code evidence.

## 19. Required Outputs

- confirmed design decisions
- design corrections
- unsupported assumptions
- migration hotspots
- missing capabilities
- Phase 1 scope corrections
- acceptance criteria corrections

Produce `11_DESIGN_AUDIT_RESULT.md` with executive summary, baseline, findings, state/command tables, migration hotspots, two-day feasibility, revised MVP, blockers, and final GO / GO WITH CONDITIONS / NO-GO recommendation.

## 20. Stop Conditions

Block implementation if critical status lacks reliable source, recording lifecycle is trapped in legacy widgets, Workstation would duplicate measurement logic, launcher would break shutdown, backend objects cannot be safely reused, or migration requires simultaneous major UI/backend rewrite.

## 21. Governing Principle

> Architecture is approved only after its assumptions survive contact with the implementation.
