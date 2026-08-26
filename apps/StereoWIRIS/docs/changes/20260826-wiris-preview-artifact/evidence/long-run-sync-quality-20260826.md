# Long-run camera synchronization quality

## Method

- User-triggered STUS completed successfully and intentionally restarted both
  RTSP sessions.
- A live sampler invoked only the read-only `RTCP/NTP 동기화 확인` calculation
  every 10 seconds for 900 seconds. It did not send STUS or change camera time.
- Evidence: `/tmp/wiris-live-sync-quality-20260826T1604KST.tsv`.
- The recording camera CSVs supplied per-Sender-Report NTP/media rates,
  SR-to-PC offsets and transport delay for the pre-fallback session.

## Result

- 90 samples: `verified=70`, `not-verified=19`, `missing=1`.
- 22/90 samples exceeded 50 ms; 21 exceeded 350 ms.
- Maximum absolute latest-frame RTCP/NTP delta: 27,389.5 ms.
- Maximum observed frame transit: 12,761.5 ms after stream restart.
- Sender Report count regressed once because the RTSP workers restarted.

Before that restart, 71 samples were available: 13 exceeded 50 ms and the
maximum was 10,101.2 ms. These spikes recovered to a few milliseconds, which
separates temporary synchronized-frame unavailability from a permanent clock
jump. They coincide with one-side decoded-output freezes while waiting for a
clean intra boundary.

Unique pre-restart Sender Report statistics after the first 60 seconds:

- Left NTP rate min/median/max: 0.999577421 / 0.9999195495 / 1.000068829.
- Right NTP rate min/median/max: 0.999537613 / 0.9999510155 / 1.000108112.
- Nearest-SR Right-minus-Left PC-offset difference: median 19.6407 ms,
  maximum 48.0366 ms.
- Left absolute SR-PC offset changed from -6.3651 to -131.9337 ms and crossed
  100 ms at session elapsed 1091.3 s.
- Right absolute SR-PC offset changed from +0.9098 to -99.8478 ms and first
  crossed 100 ms at session elapsed 1209.6 s.

## Classification

The two camera clock rates remain relatively close and their nearest-SR
relative offset stays just inside 50 ms. Long-run system synchronization still
fails because:

1. PC-UTC absolute error drifts beyond the 100 ms operating threshold;
2. RTP/AU recovery removes synchronized frame availability for seconds;
3. the latched GUI `verified` state does not downgrade on later live metric
   violations; and
4. a recording mux error restarted both streams and reset the SR anchors.

Therefore one post-STUS green snapshot is not evidence that synchronization
quality remains valid for a long run.

## PC-UTC offset growth root cause

Regression over unique Sender Reports separates rate mismatch from network
delay accumulation:

- Left SR-PC offset slope: -114.410 ppm (R² 0.8446; Theil-Sen -115.884 ppm).
- Right SR-PC offset slope: -99.250 ppm (R² 0.8067; Theil-Sen -101.265 ppm).
- Left transport-delay slope: +0.004409 ms/s, R² 0.0006.
- Right transport-delay slope: +0.052811 ms/s, R² 0.0622.

The near-zero transport R² excludes a steadily growing network queue as the
primary mechanism. Camera-reported NTP and media rate errors have the same
direction:

- Left median NTP/media rate error: -80.450 / -80.134 ppm.
- Right median NTP/media rate error: -48.984 / -48.578 ppm.

The PC is disciplined by `systemd-timesyncd`; the readback after the run was
frequency correction +16.587 ppm, NTP offset -62.660 ms, delay 213.158 ms and
jitter 59.777 ms. PC frequency slew contributes common-mode error but is too
small to explain the whole observed slope. Cancelling PC common-mode with a
nearest-SR camera-to-camera comparison leaves Right-minus-Left drift
+16.521 ppm (R² 0.6755), proving that the two WIRIS clocks themselves also run
at different rates.

The exact mechanism is therefore a one-shot STUS phase alignment followed by
undisciplined camera/PC clock-rate mismatch. STUS does not continuously
discipline frequency. Historical PC frequency correction was not logged, so
the common absolute slope cannot be apportioned into an exact per-device ppm
budget after the fact; that limitation does not change the mechanism or the
exclusion of cumulative network latency.
