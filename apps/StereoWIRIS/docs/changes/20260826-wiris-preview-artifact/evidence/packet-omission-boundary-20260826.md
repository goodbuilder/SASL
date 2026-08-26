# Packet Omission Boundary Evidence — 2026-08-26

## Reproducing flow

- Camera/session: Left `10.0.0.231:8554 -> 10.0.0.10:42890`
- Capture: `/tmp/wiris-left-harness-flow-42890.pcap`
- SHA-256: `bd0803414620b02a024951c4013b338dc6ced1f9ead2dfc49f2312b41bd2517a`
- Capture/kernel drop: 0
- Reassembled TCP byte gap: 0
- RTP: same SSRC `0x0f078cc9`, `11570 -> 11585`, missing 14
- Boundary: `FU-A(orig=1,S=0,E=0)` -> missing tail/no marker -> next timestamp AUD(NAL 9)
- TCP retransmitted bytes within RTP gap +/-100 ms: 0
- One-session TCP payload burst: 87.69 Mbit/s per 100 ms; 80.24 Mbit/s rolling 1 s

## NIC evidence

- Product: ipTIME U1003C, Realtek RTL8153 (`0bda:8153`), Linux `r8152`
- USB: 3.0 device behind 5 Gbit/s USB 3.x hub path
- Ethernet negotiation: 1000 Mbit/s full duplex
- RX/TX errors, drops, missed, FIFO, CRC, carrier, collisions: all 0
- Kernel log: no reset, disconnect, carrier flap, or NETDEV watchdog during reproduction

## Static two-session follow-up

- Capture: `/tmp/wiris-current-passive-20260826T1218KST.pcap`
- SHA-256: `1060d4d3ac520c9250e6aed55199fcd9fce4ea23fde8b9ec534cf91dcbdbdf67`
- Duration: 34.961495 s; kernel drop 0
- Right session 37076: 45.577 Mbit/s mean TCP payload, TCP byte gap 0, RTP missing 0
- Right session 59578: 44.679 Mbit/s mean TCP payload, TCP byte gap 0, RTP missing 0

## Conclusion boundary

The adapter is not the direct omission layer.  The camera RTSP sender omitted the
RTP fragments before placing bytes on the TCP stream.  Constant-quality burst plus
multiple client backpressure is the leading trigger hypothesis, but firmware queue
overflow remains unproven until an identical-motion session-count A/B or camera-side
telemetry is available.
