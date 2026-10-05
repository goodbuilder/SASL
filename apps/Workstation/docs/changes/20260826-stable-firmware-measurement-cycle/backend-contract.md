# Backend Contract

## Commands and validation

All rig-mutating entry points validate the same feature-neutral cross-process owner. No individual
socket bind is sufficient evidence of whole-rig ownership.

## State, ownership, and concurrency

- At most one live owner across Workstation and StereoWIRIS.
- Owner acquisition is atomic and records app, PID, process-start identity, acquisition time, and
  rig/profile identity.
- Process death releases ownership through OS-backed locking; stale metadata alone never grants a
  lease.
- Measurement completion does not release the owner. Normal app shutdown or explicit full-rig
  release closes writers/workers/sockets first and releases the guard last.

## Worker resource budgets, scheduling, and backpressure

No new concurrent media/inference workers are approved. Prevent the second app before it spawns
camera/YOLO workers or binds MAVLink/NTRIP resources.

## External boundaries and workers

Preserve current app-specific transport choices. Stable standard MAVLink is required; Workstation
remains MAVLink-only, while StereoWIRIS retains its current selectable ROS2/uXRCE augmentation.

## Results, errors, timeout, retry, and cancellation

Owner conflict is a deterministic rejection with no partial acquisition. Retrying is allowed after
the other process exits/releases. No automatic takeover on telemetry timeout.

## Persistence, restart, and shutdown

The lock is runtime ownership, not Measurement evidence identity. Existing recordings/settings
remain app-owned. Abnormal process exit must not leave an unrecoverable lock.
Known worker children use parent-death/process-group cleanup and must not inherit the owner-lock
file descriptor. Startup distinguishes a live owner from a lock-free verified orphan by PID start
identity and parent/process ancestry; it never kills an unverified process.

The explicit complete-shutdown script uses the same identity registry/verification contract. It
signals verified parent process groups rather than broad process-name matching, requests graceful
shutdown first, and escalates only after bounded waits. It does not delete recovery metadata,
spools, MP4, JSONL, manifests, settings, or calibration. A final nonzero exit reports any survivor
or still-owned required port/resource.

## Shared complete-shutdown command contract

- `tools/stop_rig_apps.sh` is a thin repository-level entry point shared by Workstation and
  StereoWIRIS. It invokes feature-neutral runtime-control code under `common/python/sasl_common`;
  neither app imports the other's implementation.
- Runtime state lives under the current user's `XDG_RUNTIME_DIR`, with a private `/tmp` fallback
  when that variable is unavailable. It is not stored in either app's measurement/settings tree.
- Each app registers an immutable process identity containing app/role, UID, PID, process start
  ticks, boot identity, parent identity, process group, and start time. The registry must never use
  PID or command text alone as authority.
- The app parent exposes a local, same-user control endpoint. A graceful stop request is marshalled
  onto the Qt main thread and invokes the existing top-level `close()`/`closeEvent` lifecycle so
  writers and camera/device workers drain through their current ordered shutdown paths.
- The app parent records the child/descendant process groups it owns. Before every signal, the
  command re-reads `/proc` and verifies UID, process-start identity, ancestry/recorded ownership,
  and process group. An identity mismatch is an unverified survivor and is reported, never killed.
- `--status` performs no mutation. Default interactive execution shows the same status and asks
  before a graceful request. `--force` is the only path that may escalate verified survivors after
  the graceful wait; `--yes` suppresses confirmation only when explicitly supplied.
- The force sequence is bounded and ordered. It does not signal unrelated Python, GStreamer, ROS,
  MicroXRCEAgent, or shell processes. A MicroXRCEAgent is eligible only when the registry proves
  that the selected StereoWIRIS process started and owns that exact instance.
- The registry and control-endpoint descriptors are close-on-exec and are not inherited as false
  owner evidence by camera, recorder, inference, or ROS child processes.
- Exit status distinguishes clean/no-runtime, clean/stopped, declined, verified survivor,
  unverified identity, and command/permission failure so a field script cannot mistake partial
  shutdown for success.
