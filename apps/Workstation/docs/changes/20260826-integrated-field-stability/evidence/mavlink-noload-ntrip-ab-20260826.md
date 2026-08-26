# MAVLink no-load and NTRIP-only A/B — 2026-08-26

## Preconditions

- The user physically reapplied power to both P1 and P2.
- Workstation PID 823876 and its existing `udpin:0.0.0.0:14550/14551` sockets remained open.
- Left/Right Preview was stopped and both RTSP child processes were absent.
- P1/P2 peers were `10.0.0.2:14550` and `10.0.0.3:14551` on
  `enx588694fce07e` (`10.0.0.10/24`).
- No firmware, parameter, camera setting or reboot command was sent by Codex.

## Post-power low-rate MAVLink baseline

Both peers resumed inbound HEARTBEAT and ATTITUDE on the existing host sockets and entered the
10-minute recovery qualification. A 43.98-second wire capture recorded 2,268 inbound P1 datagrams,
2,223 inbound P2 datagrams, and 44/43 host-to-peer datagrams. Independent MAVLink decoding found
660 ATTITUDE and 44 HEARTBEAT messages per side, about 15.02 Hz and 1 Hz respectively. Maximum
observed ATTITUDE/HEARTBEAT gaps stayed below about 72 ms/1.003 s. The NIC reported zero receive,
transmit, drop, error and collision counters.

Capture: `/tmp/mavlink_noload_recovery_2259.pcap`, SHA-256
`e77edffb335d263784853f015c77927cd9febfb795c6e18b803047952ffe45d4`.

The link reached the first 10-minute LIVE boundary. The one-shot automatic calibration PARAM read
then reserved each receive path, after which the conservative liveness model restarted its
post-command qualification timer. Raw traffic remained current; this was not an RX-stall event.

## Stream-profile confound

The physical PX4 reboot did not restart the Workstation process. Its per-side
`_mavlink_stream_requests_sent` memory therefore remained set, so the app did not resend the
`MAV_CMD_SET_MESSAGE_INTERVAL` profile after the remote reboot. The current capture averaged about
51.6/50.6 inbound datagrams per second, while the pre-reboot updated-subscriber capture averaged
about 140/139 per second. The missing requested IMU streams make this a valid **low-rate** branch,
not an equivalent-load reproduction of the earlier combined failure.

Pre-reboot comparison capture:
`/tmp/sasl_eth_mav_workstation_soak_20260826T201424+0900.pcap`.

## NTRIP-only low-rate branch

With Preview still stopped, SASL/NGII connected to
`RTS2.ngii.go.kr:2101/VRS-RTCM32` with `ICY 200 OK`, session credentials, periodic LEFT_GNSS GGA,
CRC-valid station/MSM observations, and zero reconnects. Around 23:06 KST the UI showed 1,636 RTCM
frames at 4.71 Hz, age 0.76 s and GGA sent 63. A ten-second independent wire sample recorded
611/604 inbound P1/P2 datagrams and 52/52 host-to-peer datagrams. A separate six-second decode
identified 20 `GPS_RTCM_DATA` packets plus five GCS heartbeats to each side while both inbound
HEARTBEAT/ATTITUDE streams remained current.

Screenshot: `/tmp/workstation_ntrip_only_2306.png`, SHA-256
`43555b6610a2f84284348ecaec7792e08d8a2e2a62684f54ea881f98541489d6`.

The branch crossed ten minutes and was then stopped normally. Final UI evidence showed 3,085 CRC-
valid RTCM frames at 4.70 Hz, reconnects 0, and P1/P2 delivery 3,081 frames / 160,888 bytes each with
zero failures. Both Pixhawks remained LIVE. Three consecutive end-of-run wire captures retained
continuous bidirectional traffic with zero kernel drops:

- 55 s: P1/P2 inbound 3,620/3,556; outbound 312/312; SHA-256
  `7c377ba6a136c43dcf8443a421251b0e22dd6dba88fc3a3fa66c33e9ce28f5d4`.
- 55 s: P1/P2 inbound 3,618/3,556; outbound 304/305; SHA-256
  `51d96d14cde936a2550554eab8c5e0b7afefecdc054ebc1c8acdff259434d396`.
- 50 s: P1/P2 inbound 3,271/3,223; outbound 280/279; SHA-256
  `c2d27851b27e341764acd497efc287104fddfcaf624b4ebabb3385ce243cf738`.

Final screenshot: `/tmp/workstation_ntrip_lowrate_stopped.png`, SHA-256
`5beb76060f6ceed3b8df3bbcb0e9e44ea6803ef5b3399f2a92e0099bc3212b16`.

Status: **PASS, low-rate branch only**. This PASS cannot exclude PX4 #25956 because the post-reboot
telemetry load is about 63% lower. After the Workstation remote-reboot rearm defect is repaired and
loaded, the same A/B must be repeated at the requested stream rate.

## USB NIC topology evidence

Kernel evidence shows the Realtek RTL8153/r8152 USB Ethernet adapter detached and re-enumerated
during physical power actions at 22:21 and 22:48–22:49; the latter included USB error `-71` before
carrier returned at 22:49:04. This explains why both app sides restarted qualification when one
physical power action disturbed the shared topology. There was no NIC detach/carrier-reset record
at the later 22:35 dual MAVLink wire-stall, so the USB reset does not explain that sustained failure.
