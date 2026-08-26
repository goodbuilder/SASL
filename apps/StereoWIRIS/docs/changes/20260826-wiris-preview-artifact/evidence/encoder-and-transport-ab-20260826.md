# Encoder and transport A/B after exact motion reproduction

Every camera SET was preceded and followed by all four encoder readbacks. Only
one requested encoder variable changed at a time.

## Right QP 1 → 10

- Left stayed `0/1/90/0`; Right became `0/10/90/0`.
- Right average bitrate was 37.55 Mbps versus Left 44.66 Mbps in the A/B
  session, but the operator still observed the same problem.
- Classification: no fix. Right QP restored to 1 and all values read back.

## Application RTP latency 100 → 500 ms

- Camera settings were unchanged.
- The operator still observed the artifact.
- Classification: post-jitter budget alone is not the complete cause. App
  setting restored to 100 ms.

## Right bitrate cap 0 → 20,000,000

- Right `GEBR` accepted 20,000,000, but recording RTSP timed out and produced a
  zero-byte Right file.
- Clearing the cap caused firmware side effects: QP/GOP became 17/16 and
  Ethernet mode became FALSE.
- Restoration was performed sequentially: SEBR 0, SEQZ 1, SEGS 90, then SETH
  TRUE. Final readback is `GETH=TRUE`, encoder `0/1/90/0` on both cameras.
- Classification: invalid/unusable mitigation, not a fix.

## UDP transport

- Actual UDP sockets were verified for each RTP/RTCP pair.
- 320-second full-snaplen pcap SHA-256:
  `45e45925e5ae627f5957640ec69c0230cb85ad41102d581413ccd379aff46923`.
- Kernel pcap drop: 0.
- Left: 2,220,951 RTP packets, 115 gaps, 241 missing (0.010850%).
- Right: 2,050,879 RTP packets, 306 gaps, 339 missing (0.016527%).
- GUI receive rate fell as low as 2.2 Hz under the clean-intra gate.
- Classification: UDP changes the loss pattern but does not eliminate camera-
  to-PC RTP loss and is worse for live continuity. App setting restored to
  TCP/100 ms.

## Final device/settings state

- Left encoder: `GEBR/GEQZ/GEGS/GEIR = 0/1/90/0`.
- Right encoder: `GEBR/GEQZ/GEGS/GEIR = 0/1/90/0`.
- Left/Right Ethernet mode: TRUE.
- Application transport/latency persisted state: TCP / 100 ms.
