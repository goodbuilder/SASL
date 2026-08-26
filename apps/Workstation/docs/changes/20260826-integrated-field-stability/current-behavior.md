# Current Behavior

## Version and worktree context

- Baseline commit: `c4397995d864576c966702cfd3c398b6032f8990`.
- Branch is ahead of origin by two commits and the repository contains extensive existing tracked
  and untracked Workstation/StereoWIRIS/Simulation changes. They are user-owned and must remain.
- Existing relevant packets are approved/verifying independently: sessionless Measurement,
  canonical runtime, NVDEC status, Ethernet MAVLink, blank Legacy startup/manual intrinsics, and
  WIRIS preview/frame decoupling. This packet owns their integrated qualification and only new
  defects discovered by that qualification.
- No Workstation/StereoWIRIS/ffmpeg/GStreamer runtime owned device resources when this packet was
  initialized; UDP `14550/14551` were free.

## Reproduction

- Interface `enx588694fce07e` is `UP/LOWER_UP` at `10.0.0.10/24`.
- Left/Right camera (`10.0.0.231/.232`) and P1/P2 (`10.0.0.2/.3`) each returned 3/3 ping.
- Both RTSP TCP `8554` and NGII `2101` accepted TCP connections.
- Fresh ffprobe connections to both `/visible` streams reported repeated `non-existing PPS 0`,
  `decode_slice_header error`, `no frame`, and initially `0x0` dimensions.
- A subsequent 12-second GStreamer `nvh264dec` probe on each side reached 1280x720 CUDA-memory
  decode and received RTCP Sender Reports. This proves recovery, not clean start or long-run health.
- NTRIP authentication, RTCM bytes, MAVLink heartbeat/ATTITUDE, STUS and recording were not started
  during the initial setup check.
- A later read-only passive diagnostic bound each MAVLink endpoint for six seconds. Both P1 and P2
  timed out with no heartbeat while ping remained healthy; no ROS2/uXRCE samples were present.
- Secure credential storage is available but has no saved SASL/VRS-RTCM32 password, and the named
  legacy username/password environment variables are absent. Live NTRIP authentication therefore
  requires an operator-entered session password before the protocol gate.

## Observed frontend behavior

The last Workstation launch remained at disconnected initial state until closed. Startup correctly
reported missing active intrinsics rather than auto-applying a stale profile. A prior close emitted
`QObject::killTimer: Timers cannot be stopped from another thread`; its ownership impact remains to
be classified during shutdown verification.

## Observed backend and persistence behavior

Persisted Workstation settings select TCP/100 ms/Auto, MAVLink UDP `14550/14551`, and network RTK direct via
`RTS2.ngii.go.kr:2101/VRS-RTCM32` with `LEFT_GNSS` GGA. Current sync limits are pair delta 50 ms,
SR-PC offset 100 ms, clock-rate error 0.5%, two SRs, 30-second warmup, and SR age 10 seconds.
StereoWIRIS code/repo settings use 7 seconds. The approved target is 7-second PASS; the active
Workstation 10-second value is a cross-app/runtime mismatch to align after baseline evidence.

## Documentation, code, test, and runtime conflicts

- Packet evidence claims large automated suites pass, but required target-hardware gates remain
  open or failed; software PASS cannot close this packet.
- The current settings file permits SR age 10 seconds while the stricter accepted qualification is
  7 seconds; StereoWIRIS already uses 7 seconds.
- Port-open/ping evidence conflicts with prior protocol silence and therefore is only a prerequisite.
- Root pytest import-name collision, a StereoWIRIS native Qt order crash, and portability findings
  remain to be audited rather than hidden.
- Camera encoder truth conflicts across active files: Workstation data-root settings record
  `0/1/90/0`, StereoWIRIS repo settings and `WIRIS.md` record `0/1/90/1`, and Workstation README
  contains `0/17/30/1`. The latest physical readback evidence says both cameras were restored to
  `0/1/90/0`. No Legacy hardware launch is allowed until code inspection proves it cannot auto-apply
  a stale profile or a separate device-setting approval is obtained.
