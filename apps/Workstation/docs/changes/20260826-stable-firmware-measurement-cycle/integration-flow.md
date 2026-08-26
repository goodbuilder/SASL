# Integration Flow

## Action-to-postcondition traces

```text
operator starts/connects selected app
  -> atomically acquire whole-rig owner
  -> start current app camera/Pixhawk/NTRIP workers
  -> verify Stable-PX4 standard telemetry and reusable calibration
  -> perform Measurement and verified save
  -> retain current connections as today
  -> revalidate non-calibration readiness before next Measurement
  -> on final app shutdown: drain/stop resources -> release owner
```

## Transition tables

| State | Action | Postcondition |
| --- | --- | --- |
| available | first app operational request | that app owns whole rig before any worker starts |
| owned by other live app | second app launch | show owner and exit before GUI workers/resources start |
| owned, Measurement sealed | next Measurement | same owner retained; readiness re-evaluated |
| owned, owner exits normally | shutdown | evidence drained, resources stopped, lock released last |
| owner process dies | second app later requests | OS lock is free; stale metadata shown only as history |

## Failure and recovery flows

Partial acquisition is forbidden. A second app never owns cameras while the first owns MAVLink.
An OS-released lock plus surviving known child is an orphan case, not permission to start a second
camera/YOLO stack without explicit orphan resolution.

```text
operator runs complete-shutdown script
  -> inspect live rig owner and registered process identities
  -> show status and active-writer/finalization warning
  -> require operator confirmation (unless explicit --yes)
  -> request graceful app close through its local control endpoint
  -> bounded wait and re-check
  -> without --force: report verified survivors and stop
  -> with --force: re-verify, then escalate only verified survivors
  -> verify rig lock, MAVLink ports, child process groups, and known workers released
  -> preserve all recovery evidence and return factual exit status
```

The exact same flow applies whether the registry identifies Workstation, StereoWIRIS, or verified
orphans from either app. A duplicated per-app kill list or process-name `pkill` path is forbidden.

## Command modes

| Invocation | Mutation | Confirmation | Escalation |
| --- | --- | --- | --- |
| `tools/stop_rig_apps.sh --status` | none | none | none |
| `tools/stop_rig_apps.sh` | graceful request after status | required on a TTY | none |
| `tools/stop_rig_apps.sh --yes` | graceful request after status | explicitly bypassed | none |
| `tools/stop_rig_apps.sh --force` | graceful request, then verified survivor escalation | required on a TTY | allowed |
| `tools/stop_rig_apps.sh --force --yes` | graceful request, then verified survivor escalation | explicitly bypassed | allowed |

For any mutating mode without a TTY, omission of `--yes` is a usage failure and no stop request is
sent.

## Cross-feature contracts and dependency direction

The guard is feature-neutral and exposes only owner/readback/acquire/release. Neither app imports
the other's widgets, workers, settings, or persistence.
