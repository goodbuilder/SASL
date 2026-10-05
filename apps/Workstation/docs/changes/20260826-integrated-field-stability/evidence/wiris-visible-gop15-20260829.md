# WIRIS visible GOP 15 controlled A/B — 2026-08-29

## Scope and acceptance contract

- Purpose: determine whether lowering the two FW `2.18.72` WIRIS Enterprise visible encoders from
  GOP `30` to `15` improves decoded cadence without worsening RTP continuity.
- Device boundary: Left `10.0.0.231:2240/8554`, Right `10.0.0.232:2240/8554`, camera NIC source
  `10.0.0.11`.
- Mutation boundary: one field and one camera at a time; exact `SEGS VISIBLE 15`, followed by
  `GEGS VISIBLE=15`, then exact `SEGS VISIBLE 30` rollback and `GEGS VISIBLE=30`.
- Preconditions: Workstation Preview and recording stopped, no competing RTSP/control owner,
  `HIWS=OK`, `GETH=TRUE`, `IRCR=FALSE`, and both visible profiles read
  `bitrate/QP/GOP/IDR/GSSS=0/1/30/0/0`.
- Probe: 35 seconds per side/profile, first 5 seconds excluded, RTSP/TCP, latency `100 ms`,
  `drop-on-latency=true`, reference-safe H.264 queues, `h264parse` AU alignment, `nvh264dec`,
  1280x720 BGR appsink.
- Independent counters: RTP sequence continuity at the `rtph264depay` sink; Annex-B NAL types and
  GStreamer `DELTA_UNIT` at the parser source; decoded frames at appsink.
- PASS: no service/profile drift, no RTP-continuity regression, and every post-warm-up one-second
  decoded sample at least `17 Hz`.

## Results

| Side | GOP | Median Hz | Min/Max Hz | `<17 Hz` | Decoded/AU | Non-delta AU | RTP packets | Gap events | Missing | Encoded bytes | IDR AU |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Left | 30 | 14.0 | 10/20 | 25/30 | 500/500 | 18 | 176,184 | 2 | 114 | 241,326,409 | 0 |
| Left | 15 | 14.5 | 11/20 | 19/30 | 509/509 | 33 | 186,166 | 1 | 162 | 253,543,247 | 0 |
| Right | 30 | 13.0 | 10/19 | 27/30 | 455/455 | 13 | 163,083 | 9 | 30 | 219,201,664 | 0 |
| Right | 15 | 14.0 | 11/20 | 23/30 | 472/472 | 35 | 174,572 | 84 | 245 | 229,778,432 | 0 |

Post-warm-up decoded one-second series:

```text
Left GOP30: 12 14 19 20 14 12 14 12 12 11 16 14 14 20 16 14 13 18 14 11 12 16 19 13 12 14 14 12 10 15
Left GOP15: 18 14 16 18 18 13 13 17 14 12 13 17 17 13 11 15 12 17 20 17 14 11 11 19 17 12 15 12 16 13
Right GOP30: 15 15 10 14 16 17 14 13 13 11 14 15 11 16 11 13 17 16 15 12 13 11 19 12 12 15 12 12 12 13
Right GOP15: 17 18 11 13 17 16 20 14 14 15 13 14 14 14 20 14 13 15 11 17 17 12 13 12 13 14 13 11 15 15
```

Interpretation:

- The camera honored GOP `15`: non-delta AUs rose from `18→33` on Left and `13→35` on Right.
- Median cadence improved only `+0.5 Hz` on Left and `+1.0 Hz` on Right; neither side approached
  the every-second `17 Hz` contract.
- Encoded bytes rose `5.1%` on Left and `4.8%` on Right. RTP packet count rose `5.7%` and `7.0%`.
- Left gap events fell `2→1`, but missing sequence numbers rose `114→162`.
- Right continuity regressed sharply: gap events `9→84`, missing sequence numbers `30→245`.
- Every 35-second artifact remained NAL1/7/8/9 with zero NAL5 IDR AUs.

Result: **GOP 15 FIELD FAIL**. The small cadence gain does not meet the throughput contract and the
Right RTP-continuity regression is disqualifying. Values below `15` were not attempted because they
would further increase intra frequency/load after this negative boundary.

## Rollback and recovery

- Left and Right separately returned exact `OK` for `SEGS VISIBLE 30` after their candidate probe.
- Final readback: both `HIWS=OK`, `GETH=TRUE`, `IRCR=FALSE`, visible
  `bitrate/QP/GOP/IDR/GSSS=0/1/30/0/0`.
- Post-rollback 10-second probes decoded 118 Left and 109 Right 1280x720 frames with no pipeline
  error; both SDK and RTSP services remained available.
- Workstation Preview was restarted. Both sides returned to `LIVE` with actual `nvh264dec` and
  established RTSP/TCP sockets.
- No settings JSON, automatic encoder profile, firmware, bitrate, QP, IDR, slice-size, recording,
  or thermal-stream value was changed.

## Long-run IDR observation discovered before the A/B

The preceding GOP 30 Workstation run emitted one runtime tracker recovery event per side:

```text
Left:  basis=h264_idr, blocked=2324.466s
Right: basis=h264_idr, blocked=2340.247s
```

The pre-mutation readback after that run was `GEIR VISIBLE=0` on both cameras. These events revise
the operational statement from “IDR is never observed” to “no bounded useful IDR interval is
proven; a runtime IDR candidate appeared only after about 39 minutes.” The old run log was replaced
when the application restarted and no matching raw Annex-B byte artifact was retained, so the two
runtime events are **observation-only**, not an independent replacement for the existing short raw
captures with `NAL5=0`. A future continuous raw capture must preserve the candidate NAL5 bytes,
adjacent SPS/PPS, PTS, and independent decode result before vendor evidence is revised.
