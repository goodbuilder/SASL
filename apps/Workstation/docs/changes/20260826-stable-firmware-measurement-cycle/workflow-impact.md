# Workflow Impact

## Affected operators and workflow IDs

- Field measurement operator using Workstation or StereoWIRIS.
- System engineer validating Stable PX4 and camera/GNSS readiness.
- Workflows: startup, rig ownership, device connection, NTRIP/RTCM, time sync, alignment checks,
  detection, recording, save/recovery, shutdown, and next long-separated Measurement.

## Current workflow

Each app can independently connect the same endpoints. When both run, ownership may split by
resource. Calibration, setup, connection, measurement, and recording lifetimes also differ between
the apps.

## Proposed workflow

Accepted shared cycle:

```text
select one operational owner and acquire whole-rig ownership
  -> load/verify existing calibration (no recalculation)
  -> connect cameras and prove fresh frames
  -> connect P1/P2 using Stable PX4 standard telemetry
  -> start/verify NTRIP and RTCM delivery when required
  -> repeat time-sync, GNSS/RTK, alignment/readiness checks
  -> start Detection + one Measurement writer
  -> stop producers, drain, verify, save/seal
  -> retain current connections as today
  -> repeat readiness checks and start the next Measurement when needed
  -> release ownership only when the selected app closes or explicitly releases every rig resource
```

## Preserved, added, removed, and reordered steps

- Preserve calibration artifacts and coordinate contracts.
- Repeat all non-calibration readiness and capture steps per Measurement.
- Add explicit whole-rig ownership and busy/failure evidence.
- Remove private-firmware telemetry from mandatory readiness.

## Decisions, confirmations, handoffs, waits, and artifacts

One selected app owns the full rig. A second app cannot directly connect any subset of the same rig.
Measurement completion does not automatically release ownership or alter current transport mode.
Intentional simultaneous operation is unsupported; the second launcher reports the live owner and
exits. No read-only/waiting/takeover mode is added.

## Interruption, cancellation, retry, restart, and resume

Each app must preserve its own verified evidence on failure. Cross-app handoff must release all
exclusive ports, child decoders, queues, and writers before another owner reports ready. No
automatic takeover occurs merely because telemetry becomes stale.
Abnormal parent death should terminate children through parent-death/process-group ownership. A
surviving verified orphan is handled explicitly at the next launch rather than mistaken for a live
second app or silently ignored.
Preferred recovery is operator-invoked `tools/stop_rig_apps.sh`, usable before launching either app
or after an abnormal shutdown. Normal app startup does not kill processes. The script's ordinary
interactive path shows status, asks for confirmation, requests normal application close, waits for
writer/finalization completion, and reports any survivor. Only an explicit `--force` run may
escalate verified survivors.

## Degraded, error, and recovery behavior

Missing private CAN identity is informational under Stable firmware. Missing standard heartbeat,
ATTITUDE, selected GPS position/baseline, camera frames, required calibration, or writer
postconditions remains explicit and method-specific.

## Completion and next actions

A completed cycle exposes verified save state and retains the selected app's current resource
lifetime exactly as today. Before the next Measurement, every non-calibration readiness result is
re-evaluated; only disconnected/stale resources require reconnect/recovery.
When complete shutdown is desired, the operator runs the explicit script and verifies its final
resource report before starting another app.

The first implementation slice changes only this recovery path and the runtime identity/control
plumbing required by it. Measurement, calibration, MAVLink, ROS2/uXRCE, NTRIP, camera, and evidence
formats retain their current behavior in that slice.

## Training, operating-procedure, and compatibility impact

Operators need one consistent cycle and one clear app-ownership rule. Existing recordings remain
readable by their owning app.

## Workflow-to-screen, command, state, and regression traceability

To be completed after the ownership decision.
