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

## Observed frontend behavior

The last Workstation launch remained at disconnected initial state until closed. Startup correctly
reported missing active intrinsics rather than auto-applying a stale profile. A prior close emitted
`QObject::killTimer: Timers cannot be stopped from another thread`; its ownership impact remains to
be classified during shutdown verification.

## Observed backend and persistence behavior

Persisted settings select TCP/100 ms/Auto, MAVLink UDP `14550/14551`, and network RTK direct via
`RTS2.ngii.go.kr:2101/VRS-RTCM32` with `LEFT_GNSS` GGA. Current sync limits are pair delta 50 ms,
SR-PC offset 100 ms, clock-rate error 0.5%, two SRs, 30-second warmup, and SR age 10 seconds.
The approved target is 7-second PASS; 7-10 seconds is diagnostic WARN until the setting is aligned.

## Documentation, code, test, and runtime conflicts

- Packet evidence claims large automated suites pass, but required target-hardware gates remain
  open or failed; software PASS cannot close this packet.
- The current settings file permits SR age 10 seconds while the stricter accepted qualification is
  7 seconds.
- Port-open/ping evidence conflicts with prior protocol silence and therefore is only a prerequisite.
- Root pytest import-name collision, a StereoWIRIS native Qt order crash, and portability findings
  remain to be audited rather than hidden.
