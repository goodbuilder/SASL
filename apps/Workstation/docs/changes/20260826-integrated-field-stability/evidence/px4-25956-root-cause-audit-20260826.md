# PX4 #25956 root-cause audit — 2026-08-26

## Verdict

The reproduced external trigger boundary is the former shared camera/Pixhawk Layer-2 topology:
camera-only RTSP load stalled both PX4 MAVLink services there, while two physically isolated
adapters passed direct camera load and application ten-minute gates. The persistent service failure
still appears to occur inside PX4/NuttX rather than the Workstation GUI or Linux socket drain. The
leading internal hypothesis is the IOB exhaustion and blocking `sendto()` deadlock described by official PX4 issue
[#25956](https://github.com/PX4/PX4-Autopilot/issues/25956). The symptom match is strong, but the
exact internal mechanism remains unproven because no PX4 task backtrace or live IOB count was
captured during the stall.

## Why #25956 applies to this firmware

- Both P1 and P2 report PX4 v1.17.0 on FMUv6X.
- The official v1.17.0 FMUv6X NuttX defconfig sets `CONFIG_IOB_NBUFFERS=24` and
  `CONFIG_IOB_THROTTLE=0`:
  [defconfig](https://raw.githubusercontent.com/PX4/PX4-Autopilot/v1.17.0/boards/px4/fmu-v6x/nuttx-config/nsh/defconfig).
- The official v1.17.0 MAVLink implementation creates a blocking UDP `SOCK_DGRAM` socket without
  `O_NONBLOCK` or `SO_RCVTIMEO`, then uses `poll()` followed by `recvfrom()`:
  [mavlink_main.cpp](https://raw.githubusercontent.com/PX4/PX4-Autopilot/v1.17.0/src/modules/mavlink/mavlink_main.cpp),
  [mavlink_receiver.cpp](https://raw.githubusercontent.com/PX4/PX4-Autopilot/v1.17.0/src/modules/mavlink/mavlink_receiver.cpp).
- #25956 remains open and has no linked development branch or pull request. It describes UDP
  MAVLink dropping out, a receive queue consuming the IOB pool, blocking send indefinitely, and
  recovery only after restarting PX4.

Local firmware artifact: `/tmp/px4_fmu-v6x_default.px4`, SHA-256
`3206a5f445d85be730a4f6158d8fdd414b9cfab27fa84bbc05c6a6752deac284`, `git_identity=v1.17.0`.

## Ranked hypotheses

| Rank | Hypothesis | Current assessment | Evidence |
|---|---|---|---|
| 1 | Camera traffic on the former shared Layer-2 path triggers the PX4 service stall | strongest external discriminator; trigger confirmed, internal cause unknown | camera-only/NTRIP-off load stalled both services; isolated old adapter and replacement RTL8156B did not reproduce |
| 2 | NuttX IOB exhaustion causes blocking UDP MAVLink send deadlock | likely; exact mechanism unproven | inbound disappears on wire while host heartbeat and ping survive; host reconnect does not recover; PX4 power does |
| 3 | RTCM ingress plus high telemetry rate crosses a separate threshold | medium | low-rate NTRIP passed; camera-only already reproduced the stall without NTRIP; isolated equivalent-rate repeat pending |
| 4 | High MAVLink stream rate or another PX4 MAVLink state defect | medium-low alone | two isolated application ten-minute runs at requested high rate passed |
| 5 | Host send concurrency or RTCM serialization | low | correct source 255/190, per-side send lock, heartbeat/RTCM decode correctly, outbound continues during failure |
| 6 | GUI, host socket drain, port collision or Linux NIC loss | effectively excluded for the 22:35 stall | one PID owns ports; failed-side Recv-Q was zero; no inbound packet existed; kernel drop/error and link reset were zero/absent |

## Field evidence

- Combined failure: host heartbeat continued to the silent peer, but P2 inbound disappeared from a
  five-second capture while P1 continued. Later both peers went wire-silent with ping still alive.
- The existing host sockets accepted fresh sysid/ATTITUDE immediately after physical recovery; no
  Workstation reconnect was needed.
- Pre-reboot high-rate capture: P1/P2 29,062/28,880 packets over about 207.2 seconds, approximately
  140/139 pps. `/tmp/sasl_eth_mav_workstation_soak_20260826T201424+0900.pcap`, SHA-256
  `926f0e02f7b20704211d651ee64fb3526aac60d50a331f53a821e9d9a1459880`.
- Outage capture: `/tmp/sasl_rtcm_outage_gKggpM.pcap`, SHA-256
  `57a7c6756aa4babf7b2667b2410b3a62db5794cdc788f67f5894a3d67cb0a600`.
- Post-power low-rate and NTRIP-only evidence is recorded in
  `mavlink-noload-ntrip-ab-20260826.md`.
- Old isolated adapter application Preview held P1/P2 for ten minutes and completed PARAM 124/124
  without heartbeat starvation. Replacement RTL8156B then passed dual-RTSP 45 seconds and a latest-
  app 736.229-second gate with 103,610/102,864 messages, exact profiles, zero capture/NIC/app error.
  Artifacts and hashes are in `camera-only-mavlink-stus-mp4-20260827.md`.

USB identity evidence has a limitation: both devices expose serial `0`, so a single USB timeout is
not enough to claim all MAVLink instances dead. The Ethernet wire silence and power-only recovery
remain the stronger oracles.

## Physical topology residual

The Realtek RTL8153/r8152 USB NIC detached during physical power actions at 22:21 and 22:48–22:49;
the latter included error `-71`. This explains why both Workstation sides restarted qualification
when the shared topology was disturbed. No NIC detach or carrier transition occurred at the later
22:35 dual wire-stall, so it does not explain that sustained failure. Cold-boot test timing must
begin only after NIC carrier is stable.

## Required discriminator sequence

Every branch must use the same requested PX4 stream profile and ten-minute window:

1. High-rate telemetry only; Preview and NTRIP off.
2. High-rate NTRIP/RTCM only; Preview off.
3. High-rate camera only; NTRIP off.
4. Only after 1–3 pass, high-rate camera plus NTRIP.
5. If NTRIP-only fails, repeat with one-side RTCM injection and use the other side as the negative
   control.

Interpretation:

- NTRIP-only failure strongly supports RTCM ingress as the trigger.
- Camera-only failure raises the shared NIC/switch/L2 path.
- Independent branches passing while combined fails supports an aggregate threshold/IOB defect.
- High-rate telemetry-only failure supports stream output or another PX4 MAVLink defect.
- A failure that persists more than 60 seconds after stopping load and recovers only after PX4 power
  supports deadlock rather than transient congestion.

Exact #25956 confirmation ultimately needs PX4-side IOB free-count/readahead evidence and a task
backtrace showing the MAVLink receiver blocked in `sendto()` or an IOB semaphore. Testing a patched
nonblocking/timeout or IOB-accounting firmware requires separate explicit firmware approval and is
not part of the currently approved host-only work.
