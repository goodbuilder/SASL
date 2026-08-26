# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-08-26 — Stable firmware and repeated non-calibration cycle

- Type: accepted scope premise.
- Normal operation assumes official Stable PX4 firmware on P1/P2.
- Normal readiness must use standard firmware telemetry and cannot require private
  `SASL_CAN_MAG`.
- Calibration is reusable across Measurements; all other operational steps may repeat because the
  interval between Measurements is long.
- Workstation and StereoWIRIS are both in scope, but their simultaneous/independent ownership mode
  remains undecided.

## 2026-08-26 — One operational app owns the rig

- Type: accepted workflow decision.
- During field operation, exactly one of Workstation or StereoWIRIS owns camera RTSP, Pixhawk
  MAVLink/optional ROS2, NTRIP/RTCM routing, YOLO, and recording resources.
- Both apps remain independently capable of operating the rig, but simultaneous direct device
  ownership is out of scope.
- A shared cross-process rig-owner guard must prevent split ownership and expose the current owner
  when the other app attempts operational access.

## 2026-08-26 — Preserve current resource lifetime between Measurements

- Type: accepted workflow decision inferred from “keep it as current”.
- A selected app does not automatically disconnect devices, stop NTRIP, or exit after a completed
  Measurement.
- Camera/Pixhawk/NTRIP/ROS2 modes remain controlled as they are today. Workstation stays
  MAVLink-only; StereoWIRIS retains its current selectable dual/MAVLink/uXRCE behavior.
- The next Measurement repeats readiness validation. It reconnects only a source that is actually
  disconnected/stale or explicitly restarted by the operator.

## 2026-08-26 — Simultaneous app operation is unsupported

- Type: accepted scope clarification.
- The operator will not intentionally run Workstation and StereoWIRIS together.
- A second normal app launch while the first process owns the rig shows the live owner and exits
  before spawning camera/YOLO workers or binding device resources.
- Read-only second-app mode, waiting takeover, broker/fanout, and simultaneous direct connections
  are excluded.
- The only supported contention exception is a verified orphan worker left after abnormal parent
  termination; its cleanup behavior remains to be decided.

## 2026-08-26 — Startup worker cleanup proposed

- Type: operator proposal and safety analysis; policy choice still open.
- Broadly killing every Python/GStreamer/worker process at launch is rejected because it can stop
  unrelated work or a live writer and corrupt evidence.
- Both apps already use Linux parent-death signals and bounded process-group shutdown; these remain
  the primary prevention.
- Safe fallback candidate: when no live rig owner exists, detect only workers whose stored process
  start identity, executable/command signature, parent ancestry, and app ownership all verify; ask
  or act according to the pending cleanup policy. Unknown processes are never terminated.

## 2026-08-26 — Explicit complete-shutdown script proposed

- Type: operator proposal; interaction policy still open.
- Preferred direction replaces startup auto-kill with an explicit repository script, tentatively
  `tools/stop_rig_apps.sh`, run only when the operator wants all Workstation/StereoWIRIS runtime
  resources stopped.
- The script targets verified app parent/process groups, asks the app to shut down first, waits for
  evidence/video finalization, then escalates `SIGINT -> SIGTERM -> SIGKILL` only for verified
  survivors.
- It reports/status-checks before mutation, captures diagnostic identity, preserves measurement
  spools/artifacts, verifies ports/processes are released, and never kills unknown Python,
  GStreamer, ROS, or user processes.
- The script is a recovery/maintenance command, not a normal app-start side effect.

## 2026-08-26 — One shared, status-first shutdown command

- Type: accepted interaction and deployment decision.
- One repository-level command, `tools/stop_rig_apps.sh`, covers both Workstation and StereoWIRIS.
  The apps do not carry divergent copies of the shutdown algorithm.
- A normal interactive invocation first prints the verified live owner, registered app processes,
  verified child/process-group identities, and active writer/finalization risk. It mutates nothing
  until the operator confirms.
- The default confirmed action requests each verified app's existing graceful close/finalization
  path and waits for a bounded result. It does not escalate a survivor automatically.
- Signal escalation is available only with an explicit `--force` invocation. `--force` still
  reports targets, requests graceful shutdown first, waits, and requires confirmation unless the
  operator also supplies `--yes`.
- `--status` is always read-only. A non-interactive mutating invocation without `--yes` fails
  closed instead of assuming consent.
- App launch never invokes this command automatically. No implementation is approved yet.
