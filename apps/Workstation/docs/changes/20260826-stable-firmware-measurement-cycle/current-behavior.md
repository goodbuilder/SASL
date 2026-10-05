# Current Behavior

## Version and worktree context

- Baseline commit: `5bec92529e0954a96522d1de7c5e1e5f2ece38ca`.
- Both app trees contain substantial pre-existing user-owned modifications; no cleanup or reset is
  authorized.
- Existing long-interval Session packet is paused; this packet owns the broader Stable-PX4
  cross-app workflow.
- Source modification is not approved.

## Reproduction

Current shared endpoints:

```text
P1 MAVLink: udpin:0.0.0.0:14550
P2 MAVLink: udpin:0.0.0.0:14551
Left camera:  rtsp://10.0.0.231:8554/visible
Right camera: rtsp://10.0.0.232:8554/visible
```

Observed simultaneous run on 2026-08-25:

- Workstation exclusively owned UDP `14550/14551`.
- StereoWIRIS could not own those MAVLink listeners but its two child processes continuously
  decoded the left/right RTSP streams.
- Workstation NTRIP remained connected and attempted RTCM routing.
- This split state is not a valid proof that either app can independently complete a Measurement.

## Observed frontend behavior

- Both applications expose camera, Pixhawk, GNSS/RTK, detection, measurement, and recording
  controls as if they can directly own the same rig.
- Workstation separates Session/Measurement and has strict package recovery/finalization.
- StereoWIRIS uses a direct `세션 기록 시작/중지` toggle and starts a new timestamped session;
  starting new currently discards a retryable prior session through its legacy recovery path.
- Neither app currently presents a cross-process owner/busy identity for the whole rig.

## Observed backend and persistence behavior

- Workstation hardens pymavlink `udpin` into exclusive ownership and fails a later bind.
- StereoWIRIS pymavlink still uses reusable `udpin` sockets and lacks the Workstation stale-ATTITUDE
  recovery and asynchronous disconnect hardening.
- Workstation requests one private `SASL_CAN_MAG` stream in addition to standard messages, but
  bounds retries and does not require it for standard ATTITUDE/GPS measurement.
- StereoWIRIS MAVLink requests standard messages only; its default Pixhawk Ethernet mode remains
  `dual` and may also configure/use ROS2/uXRCE unless launched MAVLink-only.
- Both apps own separate recording/session formats and separate runtime state; no common writer or
  broker coordinates their evidence.
- Both current camera worker implementations already arm Linux parent-death `SIGTERM`, use process
  groups for recorder descendants, and escalate cooperative shutdown through interrupt/terminate/
  kill paths. Startup cleanup is therefore a fallback for a verified failure of these protections,
  not the primary lifecycle mechanism.

## Documentation, code, test, and runtime conflicts

- Both are documented as independent operational apps, yet simultaneous direct operation shares
  exclusive device resources without an ownership contract.
- Stable firmware cannot provide private per-CAN `SASL_CAN_MAG`; any UI treating that identity as
  required would be incompatible with the new premise.
- The user's long-interval workflow allows non-calibration readiness checks to repeat, but the
  accepted behavior still retains each app's current camera/Pixhawk/NTRIP lifetime between
  Measurements and reconnects only stale/disconnected sources.
- “Calibration excluded from repetition” is currently interpreted as both camera K,D/R,t and
  QGC/PX4 sensor calibration; this interpretation requires confirmation later.
