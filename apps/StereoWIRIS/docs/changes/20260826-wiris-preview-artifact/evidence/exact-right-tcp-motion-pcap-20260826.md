# Exact single-session Right TCP capture

## Capture contract

- One Right RTSP-over-TCP session fed one encoded MKV tee plus NVIDIA and
  software decoder branches for 90.09 seconds.
- A simultaneous full-snaplen passive pcap captured the exact Ethernet flow.
- pcap SHA-256:
  `fea18f0e66d90fda85d616ac57b00a153c140c9ef0bd61cb5fc252d7bfde47ca`.
- MKV SHA-256:
  `636b4377a667b568d3e684cae8cb45121fdc69b47d29a4f7093ed0192d22079c`.
- Kernel pcap drop: 0.

## Result

- TCP stream bytes: 477,692,445.
- TCP byte gaps after reassembly: 0.
- Retransmitted bytes: 550,395; TCP fully repaired them.
- Main RTP SSRC packets: 341,823.
- RTP forward gaps: 4; missing packets: 137.

Exact camera-to-PC events:

- 17:00:06.444 KST: missing 69, seq 6851→6921.
- 17:00:52.388 KST: missing 16 inside one H.264 slice timestamp.
- 17:01:08.395 KST: missing 44 inside one H.264 slice timestamp.
- 17:01:08.397 KST: missing 8 more in that same slice timestamp.

Since complete TCP bytes on the receive NIC already contain forward RTP
sequence omissions, the USB NIC, kernel capture, TCP transport, GStreamer
jitter buffer and decoder cannot be the layer that removed these packet
numbers. The omission is at or before the WIRIS RTSP/RTP sender.
