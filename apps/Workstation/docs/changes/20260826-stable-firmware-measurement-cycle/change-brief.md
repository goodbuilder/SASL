# Change Brief

## Operator outcome

Both Workstation and StereoWIRIS must be able to perform a truthful, repeatable field Measurement
cycle against official Stable PX4 firmware. Calibration is a persistent prerequisite and is not
repeated automatically. Because intervals between Measurements are long, the remaining connection,
readiness, synchronization, GNSS/RTK, alignment/check, capture, and save checks may repeat for every
Measurement. This does not force a transport teardown: each app retains its current resource
lifetime and reconnects only a disconnected/stale source unless the operator requests shutdown.

## Motivation

- Custom firmware behavior must not be required for normal operation.
- Stable PX4 provides standard MAVLink HEARTBEAT, ATTITUDE, GPS*, SCALED_IMU*, parameter, shell,
  RTCM injection, and optional standard uXRCE behavior, but not private `SASL_CAN_MAG` identity.
- Workstation and StereoWIRIS currently have divergent connection/recovery/session behavior while
  targeting the same physical cameras, Pixhawks, UDP ports, NTRIP route, and host resources.
- Running both today split ownership: Workstation held MAVLink `14550/14551`, while StereoWIRIS
  camera processes held the RTSP streams. Neither process then represented a complete independent
  measurement owner.

## Included scope

- One explicit repeatable non-calibration Measurement cycle for each supported app.
- Stable-firmware MAVLink/GNSS/RTCM readiness and optional diagnostics contracts.
- Cross-app ownership of camera streams, MAVLink ports, NTRIP/Base routing, recording writers,
  CPU/GPU decode/inference, and saved evidence.
- Start/stop/retry/recovery behavior between long-separated Measurements.
- Compatibility of existing settings and recorded data where practical.

## Excluded scope and non-goals

- No PX4 custom-firmware patch or private-message dependency.
- No change to triangulation, calibration algorithms, coordinate frames, units, or RTK semantics.
- No automatic calibration rerun. Existing camera K,D/R,t and QGC/PX4 calibration remain explicit
  reusable prerequisites until the operator replaces them.
- No source implementation before packet-scoped approval.

## Acceptance summary

Accepted: one app owns the whole rig at a time; both apps remain independently operational. Stable
standard telemetry is mandatory, private telemetry is optional, and current device/transport
lifetimes remain unchanged between Measurements. A second live app reports the owner and exits.
Preferred orphan recovery is now an explicit complete-shutdown script rather than startup
auto-kill. One shared command covers both apps: it reports verified targets and asks before a
graceful stop; destructive signal escalation is available only when the operator explicitly adds
`--force`.
