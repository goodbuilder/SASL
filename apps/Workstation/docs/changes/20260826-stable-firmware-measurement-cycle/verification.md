# Verification

## Requirement-to-evidence matrix

| Requirement | Planned evidence |
| --- | --- |
| One command covers both apps | Automated fake-boundary and application-launch tests register each app identity and obtain the same status schema |
| Status is read-only | Files/processes/control endpoints are snapshotted before and after `--status`; no signal or close request occurs |
| Default requires confirmation | TTY accept/decline and non-TTY-without-`--yes` command tests |
| Default never force-kills | Graceful-timeout fixture survives, command exits nonzero, and no escalation signal is observed |
| `--force` is explicit and bounded | Signal-order/wait tests with a verified stubborn child; identity rechecked before each step |
| Unknown/PID-reused processes survive | `/proc` identity mismatch, same-name unrelated process, changed PGID, and stale registry adversarial tests |
| Existing finalization is reused | Qt integration tests invoke top-level close and observe the same ordered shutdown coordinator/postconditions |
| Evidence and settings are preserved | Before/after hashes for fixture MP4/JSONL/manifests/spools/calibration/settings and no delete calls |
| App-owned children are released | Parent, camera/inference, recorder group, and owned XRCE fixture release checks |
| No protocol/coordinate behavior changes | Focused diff review plus existing lifecycle/worker/MAVLink-independent regression tests |

## Focused test evidence

Pending implementation. Planned first commands use the canonical repository Python and targeted
pytest files for common runtime identity/control, Workstation lifecycle, and StereoWIRIS lifecycle.

## Regression and application evidence

Pending implementation. Run both documented `run.sh --check` paths, offscreen app close tests, and
the existing Workstation/StereoWIRIS lifecycle, camera worker, recorder, and XRCE-agent tests that
share the affected shutdown boundaries.

## Performance, load, and target-hardware evidence

The registry/control path runs only at app start/stop and on explicit status requests; it adds no
per-frame or telemetry loop. Verify bounded status latency with representative registered child
counts and confirm no additional camera/YOLO/GPU worker is created. Hardware device recovery is
outside this slice and remains unverified by these tests.

## Feature isolation and contract evidence

Verify the common modules import no Workstation/StereoWIRIS/Qt/device implementation, each app can
register/close with the other app absent, an unavailable XRCE feature does not break MAVLink-only
StereoWIRIS shutdown, and no cross-app private import is introduced.

## Visual authenticity and dynamic-layout stability evidence

No new GUI surface. Verify the close request neither raises nor resizes a window and uses the
existing close lifecycle. Capture terminal output sequences instead of before/after GUI
screenshots; assert output remains understandable without ANSI color.

## Visual and hardware/manual evidence

Pending implementation. Manual field checks should run status/graceful/force recovery once with
Workstation and once with StereoWIRIS, including a recording/finalization warning. They cannot
establish Pixhawk, camera firmware, NTRIP, or network recovery.

## Residual conditions

- The tool cannot reset firmware or external devices/services.
- `SIGKILL` cannot finalize in-process state; it remains an explicit last resort after the normal
  close and bounded waits fail.
- A target with unverifiable identity is intentionally left running for manual diagnosis.
