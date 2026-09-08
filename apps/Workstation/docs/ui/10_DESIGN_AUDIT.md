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

Current Workstation boundary (2026-08-09): Pixhawk telemetry is MAVLink-only. ROS2/uXRCE
connection/discovery and still-image/FTP/EXIF clock workflows are Legacy-only and are not
part of the Workstation runtime or its operator sequence. The launcher also branches before
the ROS2/PX4 overlay bootstrap for `--ui workstation`; ROS2 Python bindings in the shared
telemetry module are lazy-loaded only by a Legacy ROS2 operation.

2026-09-02 topology audit amendment: the fixed Pixhawk topology group must preserve the existing
manual controls and fixed workspace geometry. Audit the operation journal, restart-to-PARTIAL
behavior, exclusive serial/UDP ownership, immediate identity/isolation/disarmed revalidation,
explicit mutation confirmation, exact readback, commit-last persistence, guided rollback, and
lifecycle cancellation. `COMPLETE` must never be used as evidence of Connect or field
qualification; physical radio removal, cable insertion, and power cycling stay operator-owned.

2026-09-07 Baseline Yaw90 relocation audit: verify the appended Method tab retains optical
frames/annotations/candidates and restores scroll. Live azimuth is independent of Drone/attitude;
vertex angles and R/P have separate state/age rows. Center line never enters source/capture pixels.
Reverse changes only its original setting and is rejected during active Measurement. The former
Telemetry midpoint table is now collapsed reference data; its backend and recorded schema remain.
Calibration loses only Center line. Verify 1366x768 and 1920x1080 views and no new stream/worker.

2026-09-02 GNSS baseline-center audit amendment: verify that the GNSS midpoint result is named
`gnss_baseline_center_local_level_frd`, uses the P1/P2 GNSS antenna midpoint and local-level axes,
and cannot alias optical `baseline_center_frd`. Audit exact signed sentinel math, 2.0 s/1000 ms
timing boundaries, stale-value retention, missing-data non-zero fabrication, fix-as-evidence,
shared Rg/readiness with ideal disparity, absence of persistence/workers/device I/O, and fixed
one-row table geometry/focus/scroll across live states and target screen sizes.

2026-09-03 GNSS installation-angle audit amendment: verify that the relocated midpoint reference derives the
acute horizontal angle from existing GNSS midpoint `Forward/Right` components as
`atan2(|Forward|,|Right|)`, shows absolute `|90°−angle|` error, keeps Drone ATTITUDE/yaw out of the
contract, preserves CURRENT/STALE/WAITING source timing semantics, and remains fixed-height and
scroll/focus stable at target screen sizes.

2026-09-02 GNSS recording audit amendment: verify primary-Drone-only/recording-only trigger,
one-row-per-callback ordering after raw evidence, self-contained exact source clocks/selection,
CURRENT/STALE/WAITING null semantics, <=4 KiB row, best-effort bulk failure isolation, fixed stream
append/restart/checkpoint, Data Status truth, latest-generation XLSX filtering, and absence of raw
row/canonical Measurement/legacy Parquet/settings/worker/device changes.

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
PX4/MAVLink → receiver → parser/model → UI. Verify connection semantics, heartbeat, attitude, timestamps/freshness, and the single Workstation transport path; Legacy ROS2 remains outside this runtime.

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
