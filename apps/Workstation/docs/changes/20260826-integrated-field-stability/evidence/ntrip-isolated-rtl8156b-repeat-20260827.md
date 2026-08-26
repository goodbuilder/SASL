# Isolated-topology NTRIP reconnect and 10-minute RTCM injection — 2026-08-27

## Preconditions and credential handling

- Cameras were not running. P1/P2 used replacement RTL8156B `enx0023a40a2e39` at
  `10.0.0.10/24`; both sides were already LIVE.
- Caster configuration was `RTS2.ngii.go.kr:2101/VRS-RTCM32`, `LEFT_GNSS` GGA policy.
- Secure storage was available but status was `NOT_SAVED`. The operator-provided password was
  entered into the visible GUI field and applied only to this process; no password value was logged,
  printed or written to settings.
- Preview, Measurement and recording were off.

## First normal run and reconnect decision

The first run returned one HTTP/ICY 200, valid station/MSM RTCM, dual injection failure zero and was
stopped normally after the UI had exceeded ten minutes. Its pcap began before password handoff and
timed out after only 584.9 seconds of server payload, so it was retained as useful evidence but not
used as the independent continuous-wire duration oracle. A second Start after normal Stop created a
new caster TCP connection and is the qualification below.

## Definitive reconnect run

The second caster connection remained continuous for 636.507 seconds. Independent pcap processing
found:

- exactly one HTTP GET with Authorization present and exactly one HTTP/ICY 200 response;
- 120 NMEA GGA sentences, all 120 checksum-valid;
- 2,954 CRC-valid RTCM3 frames / 82,109 framed bytes;
- message counts: 1006/1008/1013/1032/1033/1230 = 67 each and
  1074/1084/1094/1124 = 638 each;
- station-coordinate and multi-constellation MSM observations both present.

The application finished at 3,026 injected RTCM frames / 84,125 bytes on each Pixhawk, with
`fail=0` and `blocked_stall=0`, then reached `phase=STOPPED` after explicit Stop. Wire decode found
2,970 outbound `GPS_RTCM_DATA` messages per side. P1/P2 remained LIVE:

- P1: 92,659 inbound messages, 19,783 ATTITUDE; max ATTITUDE/HB gaps
  38.747 ms / 1.003859 s;
- P2: 92,008 inbound messages, 19,783 ATTITUDE; max ATTITUDE/HB gaps
  38.280 ms / 1.004397 s;
- host GCS heartbeat max gaps 1.025683/1.041553 s;
- RTCM wire-message max gaps 1.142948/1.142958 s.

Both captures reported kernel drop zero and the replacement NIC retained zero error/drop/carrier
events:

- `/tmp/ntrip_isolated_rtl8156b_repeat_10m_caster_20260827.pcap`, 369,955 bytes,
  SHA-256 `6357a58ecb21537cea3f9aa44ba57b2713e5c7234894733f8ad141cda80ad7d7`;
- `/tmp/ntrip_isolated_rtl8156b_repeat_10m_mavlink_20260827.pcap`, 18,825,092 bytes,
  SHA-256 `e18d3be38855a3a6f5fdb56b718348d113b230f390dacf8354ef0d026b2d2cf5`.

## Verdict

NTRIP authentication, normal Stop/reconnect, continuous 10-minute correction, GGA, CRC/parser,
station/MSM content and dual P1/P2 injection are FIELD PASS on the isolated replacement topology.
The indoor rover telemetry remained `NO_FIX` (`fix_type=0`, satellites 0), so RTK Fixed convergence,
open-sky accuracy and final integrated camera/NTRIP acceptance are not established.
