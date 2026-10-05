# Dual-camera oscillator holdover — 2026-08-27

## Scope

This was a 1,800.022-second read-only dual RTCP observation. Workstation was stopped first so the
probe was the only application RTSP owner. No STUS, SDK setting change, camera reboot, video decode,
MAVLink action or NTRIP action occurred. Each side used one decoder-free RTSP/TCP Sender Report
probe. A 15-second stale watchdog was armed but never fired.

The test estimates two operational clock rates independently:

- RTC/NTP rate: robust regression of Sender Report NTP against host monotonic receipt time.
- RTP media-clock rate: robust regression of unwrapped RTP ticks against host monotonic receipt
  time, divided by the advertised media clock rate.

`Right - Left` cancels common host/transport trend. Per-camera SR-PC absolute values remain
diagnostic-only and were not a success gate.

## Result

- Duration: **1,800.022 s**.
- Sender Reports: **381 Left / 377 Right**.
- Continuous segments: **1 Left / 1 Right**.
- Probe restarts: **0 Left / 0 Right**.
- Final RTC/NTP robust relative rate: **Right faster by 16.605 ppm**.
- Conservative RTC/NTP 95% interval: **+11.863 to +21.356 ppm**.
- Final RTP media-clock robust relative rate: **Right faster by 16.634 ppm**.
- Conservative RTP media-clock 95% interval: **+11.965 to +21.401 ppm**.
- Direct three-SR-median relative delta changed from **-3.049698 s** to **-3.018342 s**:
  **+31.356 ms**, or **+17.420 ppm** over the complete wall interval.
- At the RTC/NTP point estimate, 100 ms of relative drift accumulates in approximately
  **6,022 s / 100.4 min**.

The direction was the same at every checkpoint and RTC/NTP agreed with the independent RTP media
clock. This closes the 30-minute observation question as **FIELD CONFIRMED: the Right operational
camera clock runs faster than the Left by approximately 16–17 ppm in this powered run**. It does
not prove the same rate across temperature, reboot, firmware change or a different device pair.

## Five-minute checkpoints

| Elapsed | L/R SR | RTC/NTP Right−Left | Conservative 95% interval | RTP media Right−Left |
|---:|---:|---:|---:|---:|
| 5 min | 62 / 61 | +16.572 ppm | +13.587…+19.409 ppm | +16.563 ppm |
| 10 min | 127 / 125 | +18.106 ppm | +17.318…+18.888 ppm | +18.126 ppm |
| 15 min | 189 / 187 | +16.090 ppm | +11.421…+21.323 ppm | +16.353 ppm |
| 20 min | 252 / 249 | +14.248 ppm | +2.084…+26.821 ppm | +14.400 ppm |
| 25 min | 318 / 315 | +15.783 ppm | +8.198…+23.516 ppm | +15.905 ppm |
| 30 min | 381 / 377 | +16.605 ppm | +11.863…+21.356 ppm | +16.634 ppm |

## Raw evidence

- Runtime artifact: `/tmp/camera_oscillator_30m_relative_holdover_20260827.jsonl`
- Bytes: `456855`
- Lines: `767`
- SHA-256: `8eadecd6d12e2284589383bf22dd32b09b03e6c34927573a84e4994b463efe07`

The raw JSONL is runtime evidence and is not added to Git. This result establishes clock-rate
difference only. The pre-existing approximately three-second relative phase error came from the
prior STUS application experiment and was deliberately not corrected during this holdover test.
