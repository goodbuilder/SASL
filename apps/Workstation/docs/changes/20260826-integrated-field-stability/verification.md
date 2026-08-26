# Verification

## Requirement-to-evidence matrix

| ID | Required behavior | Independent evidence | Status |
|---|---|---|---|
| R1 | clean and sustained dual RTSP/NVDEC | fresh-client decode, RTP/RTCP/pcap, frames | FAIL baseline: initial PPS absent; long run NOT_RUN |
| R2 | corruption/freeze-free motion | periodic images + identity and gap events | NOT_RUN |
| R3 | sustained STUS sync | ACK targets, new workers, RTCP series | NOT_RUN |
| R4 | P1/P2 MAVLink qualification | raw inbound HB/ATT/ANY and side state | NOT_RUN current boot |
| R5 | SASL/NGII authentication/RTCM | raw HTTP status, CRC-valid RTCM and injection ACK/counters | NOT_RUN |
| R6 | Measurement admission and restart | UI action through new run ID/files | NOT_RUN hardware |
| R7 | dual MP4 finalization | size series, ffprobe, strict full decode | NOT_RUN current candidate |
| R8 | exact evidence identity | FrameKey plus durable readback hash | NOT_RUN hardware |
| R9 | bounded GUI/resources | time-series RSS/queues/GPU/latency/cleanup | NOT_RUN hardware |
| R10 | canonical reproducible runtime | doctor/launchers/root venv | NOT_RUN current baseline |
| R11 | final integrated acceptance | 3 cold boots + 60-minute full load + failure isolation | NOT_RUN |

## Focused test evidence

Pending fresh execution. Prior packet counts are leads only and will be independently audited.

## Regression and application evidence

Pending fresh execution after the first changed slice. Root-suite collection and native Qt harness
residuals must remain visible.

## Performance, load, and target-hardware evidence

Initial target: RTX 4050 driver 595.84, about 1 TiB free storage. Both cameras recovered to
1280x720 `nvh264dec` CUDA-memory output in separate 12-second probes. No integrated time series yet.

## Feature isolation and contract evidence

No Workstation/StereoWIRIS runtime owner remained after initial setup. UDP 14550/14551 were free.
One owner will be used for all hardware runs; one-side failure and storage/decoder failure isolation
remain required.

## Visual authenticity and dynamic-layout stability evidence

No new UI is yet proposed. Any status correction must reuse existing surfaces and preserve geometry,
focus, selection, splitter/dock and scroll state under rapid updates.

## Visual and hardware/manual evidence

Initial physical prerequisite PASS: four device pings, both RTSP TCP ports, and NGII TCP port.
Protocol/application behavior remains unverified except the short direct media probes.

## Residual conditions

Camera sender RTP/IDR behavior and PX4 Ethernet MAVLink may remain external blockers. The packet is
active and cannot be completed while any required hardware evidence is absent.
