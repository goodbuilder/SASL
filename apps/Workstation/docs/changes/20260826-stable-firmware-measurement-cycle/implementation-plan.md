# Implementation Plan

## Approval status

Awaiting explicit approval for the shutdown-only vertical slice below. Stable-firmware readiness,
MAVLink/NTRIP recovery, Measurement eligibility, and broader whole-rig launch admission remain
unapproved.

## Slices and dependencies

1. Add pure, feature-neutral runtime identity and registry contracts.
   - Resolve a private per-user runtime directory.
   - Capture/verify boot ID, UID, PID start ticks, parent, process group, app, and role.
   - Atomically write metadata and reject PID-reuse/identity mismatches.
   - Define factual status and exit-code models without Qt or device imports.
2. Add the graceful control boundary to both app parents.
   - Register Workstation and StereoWIRIS at their real entry points.
   - Expose one same-user local control endpoint per parent.
   - Marshal `close` onto the Qt main thread and reuse the current retryable `closeEvent` lifecycle.
   - Record owned worker/recorder groups at spawn/exit boundaries; include an app-owned
     MicroXRCEAgent only when StereoWIRIS created that exact process.
3. Add the shared repository command.
   - Implement `tools/stop_rig_apps.sh` as a thin portable wrapper around the common Python
     control command.
   - Implement `--status`, status-first interactive graceful stop, `--yes`, and explicit
     `--force` escalation exactly as documented in `integration-flow.md`.
   - Re-verify every identity immediately before every mutation and verify release afterward.
4. Verify failure paths and update lasting operator documentation.
   - Prove no-op, declined, graceful, timeout, PID reuse, stale metadata, orphan, active writer,
     force, unrelated-process, permission, and repeated/idempotent paths.
   - Run both documented launchers and the affected lifecycle/worker regression suites.
   - Update root/app usage docs and StereoWIRIS `UPDATE.md`; preserve all data-format and
     coordinate contracts.

Each slice stops if it would require broad process-name matching, deletion of evidence, a new GUI
workflow, or changes to MAVLink/NTRIP/camera/measurement semantics.

## File and component map

Planned additions/edits, subject to source inspection during implementation:

| Area | Planned responsibility |
| --- | --- |
| `common/python/sasl_common/rig_runtime.py` | Runtime directory, immutable process identities, registry, `/proc` verification |
| `common/python/sasl_common/rig_control.py` | Status, local graceful request, bounded wait, explicit force policy, exit codes |
| `tools/stop_rig_apps.sh` | Repository-relative operator entry point using the canonical environment |
| `apps/Workstation/__main__.py` and top-level window lifecycle | Parent registration, Qt-thread close request, cleanup |
| `apps/Workstation/workstation_core/yolo_stream.py` | Exact owned worker/recorder process-group registration hooks |
| `apps/StereoWIRIS/wiris_stereo_distance/legacy_gui.py` and actual entry point | Parent registration, Qt-thread close request, cleanup |
| `apps/StereoWIRIS/wiris_stereo_distance/yolo_stream.py`, `xrce_agent.py` | Exact owned worker/recorder/agent registration hooks |
| both app test trees | Common contract plus app-specific lifecycle integration evidence |
| root/app operator docs | One command, safe/force behavior, limits and recovery instructions |

No app imports the other app. No recorded Measurement, settings, calibration, coordinate, MAVLink,
NTRIP, RTSP, or ROS schema changes.

## Migration and rollback steps

- No persistent-data migration. Runtime files are per-user and disposable after verified process
  exit; stale metadata is never trusted as a live identity.
- Existing app close buttons and current parent-death/process-group behavior remain the normal
  lifecycle.
- Rollback removes the shared command/control hooks and runtime files after both apps are closed;
  no measurement artifact conversion or settings rollback is required.

## Stopping conditions

- A target cannot be proven by UID + boot ID + PID start ticks + ownership/ancestry.
- Existing app shutdown cannot be invoked on the Qt main thread without bypassing finalization.
- Safe completion would require killing an unknown process or deleting/modifying evidence.
- Implementation exposes a necessary MAVLink/NTRIP/device reset or measurement-format change.
- A new material workflow choice or destructive behavior is discovered.
