# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

This approval covers one recovery feature only: a repository command named
`tools/stop_rig_apps.sh` that can completely stop either Workstation or StereoWIRIS and only the
workers proven to belong to them. Both apps use one shared runtime identity/control implementation;
there are no duplicated per-app kill lists.

Measurement, calibration, MAVLink, NTRIP, RTSP, ROS2/uXRCE mode, recording formats, settings, and
coordinate calculations do not change in this slice. The tool cannot reboot Pixhawk or camera
firmware and cannot repair the network.

## Operator workflow before and after

Today, after an abnormal app exit, the operator has no single safe command that distinguishes a
live app, its orphaned workers, and unrelated Python/GStreamer/ROS processes.

After this slice:

1. `tools/stop_rig_apps.sh --status` only reports verified state.
2. `tools/stop_rig_apps.sh` shows the same targets and active-writer risk, asks for confirmation,
   requests each app's existing normal close path, waits, and reports the result.
3. If verified processes survive, the ordinary command stops with a nonzero result. Escalation is
   possible only with an explicit `--force`; confirmation still applies unless `--yes` is also
   explicit.
4. Normal application launch never kills old processes automatically.

## Success, delay, rejection, failure, overload, and recovery

Success means the registered app/children are gone, their control endpoints and runtime ownership
are released, and no verified survivor remains. A declined prompt mutates nothing. A timeout,
identity mismatch, permission error, or remaining process is reported as failure rather than
silently treated as success. The tool waits for the existing writer/finalizer path before any
explicit force sequence.

## Data and compatibility effects

The command never deletes or rewrites spools, MP4, JSONL, manifests, settings, or calibration.
There is no persistent-data migration. Runtime registry files are per-user local state and are not
measurement evidence.

## Performance priorities and allowed degradation

No new camera, inference, ROS, or telemetry worker is added. Registration/status work occurs at
startup/shutdown or on explicit command use, outside frame processing. A bounded wait during
shutdown is intentional; the app must not claim completion while a verified worker remains.

## Visual and interaction changes, including dynamic-layout stability

No GUI screen or control changes. The command produces line-oriented terminal status and cannot
depend on color alone. Its graceful request enters the apps' existing top-level close behavior and
must not move, resize, or navigate the window first.

## How acceptance will be demonstrated

Automated tests will cover both apps, read-only status, confirmation decline, non-interactive
fail-closed behavior, graceful close, timeouts, stale metadata, PID reuse, unrelated same-name
processes, registered orphans, explicit force order, repeated execution, and evidence-file
preservation. Both actual launchers and affected lifecycle/worker regression tests will be run.
Manual hardware checks will be reported separately and will not be inferred from fakes.

## Cost, risks, assumptions, rollback, and field limits

The main risk is signaling a wrong or still-writing process. The design mitigates that by requiring
UID, boot identity, PID start identity, ancestry/registered ownership, status-first confirmation,
and immediate re-verification before every signal. If identity cannot be proven, the process is
left alone. `SIGKILL` can still lose unfinished in-memory work, which is why it requires explicit
force mode and remains last.

Rollback removes the command/control hooks after both apps are closed. Because there is no schema
or evidence migration, recorded data does not need rollback.

## Exact scope awaiting approval

Awaiting explicit approval to implement only the shared shutdown vertical slice described above
and in [implementation-plan.md](implementation-plan.md). Broader Stable-PX4 readiness, MAVLink
reconnect, NTRIP authentication/recovery, Measurement eligibility, Session redesign, and automatic
whole-rig admission remain outside this approval.
