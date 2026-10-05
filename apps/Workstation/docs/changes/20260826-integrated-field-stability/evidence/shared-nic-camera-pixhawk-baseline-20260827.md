# Shared-NIC camera + Pixhawk baseline — 2026-08-27

## Scope and topology

- Purpose: reproduce the previous camera+P1/P2 combination before adding NTRIP.
- Physical state: the isolated Pixhawk RTL8156B NIC had no carrier; cameras and P1/P2 shared
  `enx588694fce07e`.
- Host state was changed only in the active NetworkManager device, not its saved profile:
  camera source `10.0.0.11/32` retained for `.231/.232`; temporary Pixhawk source
  `10.0.0.10/24` added for `.2/.3`. All four devices answered ping.
- NTRIP remained DISCONNECTED. No RTCM, STUS, camera SDK, parameter, reboot or firmware command was
  sent.

## First attempt before the final physical topology reset

With dual experimental Preview already current and NTRIP off, P2 entered
`ETH MAVLINK RX STALLED` with about 26 seconds of silence and P1 was `RECOVERING`. Camera FPS was
17.3/17.0 and both NIC error/drop counters were zero. This attempt stopped at its first checkpoint
and is preserved in `camera-pixhawk-baseline-20260827.jsonl`; it was not promoted to a 10-minute
qualification.

## Fresh application on the completed shared topology

A new Workstation generation started P1/P2 listeners and dual Preview. Both cameras recovered at
complete non-IDR intra boundaries. P1/P2 never produced an expected-system packet:

- status after about 14 seconds: both `MAVLINK INITIAL RX TIMEOUT`, sysid absent;
- host listeners existed at UDP 14550/14551 with Recv-Q zero;
- both `.2/.3` continued answering ping;
- 10-second camera-on pcap: exactly 11 host GCS heartbeats per side, zero PX4 inbound packet;
- stopping Preview, waiting 15 seconds and repeating produced the same 11+11 outbound/zero inbound;
- independent USB reads on ttyACM0 and ttyACM1 returned no HEARTBEAT in five seconds each.

Artifacts:

| Condition | Size | SHA-256 |
|---|---:|---|
| camera on | 1,762 B | `3eabc0138e0ea3ad2c7a6155ce6a23fb6399f6c76d5cff186f5b24f91598b71d` |
| camera intended-off attempt (toggle did not execute; camera remained on) | 1,762 B | `ac35e44e3322d64dfa57a89c908006973a3d4a83b1b237efb19fde0d71bc3ffb` |
| camera confirmed stopped | 1,762 B | `65e93cc472e3a8fdc507616777e7dcc2c2bde13ad09b487d70bcf194b4abcdb6` |

## Current result and next gate

The previous shared-NIC combination is **FIELD FAIL before NTRIP**. The current evidence is stronger
than an application receive failure: host heartbeat reaches each peer address while Ethernet and
USB MAVLink are both silent. Stopping camera traffic does not recover the already-stalled PX4
MAVLink services.

This run cannot establish a clean time-to-failure because P1/P2 were already globally silent at the
new application start. Stop all owners, physically power-cycle both Pixhawks, verify USB heartbeat
and no-camera Ethernet inbound first, then start the shared-NIC camera load from that clean boot.
Only after the NTRIP-off baseline reaches its terminal criterion should NTRIP be added as the single
independent variable.

## Different-adapter shared-NIC A/B

The operator then moved all four devices to the other physical adapter, RTL8156B
`enx0023a40a2e39`. The prior NIC had no carrier. A process-local `.11/32` camera address was added
beside the adapter's existing `.10/24`; saved NetworkManager configuration was not changed. All four
devices answered ping. Both Pixhawks were physically power-cycled and USB heartbeat independently
mapped ttyACM0=sysid1 and ttyACM1=sysid2 for this boot.

Before camera load, a bounded direct Ethernet peer-seed gate passed:

| Side | Host heartbeat | PX4 inbound datagrams | Inbound bytes |
|---|---:|---:|---:|
| P1 | 12 | 602 | 24,739 |
| P2 | 12 | 613 | 25,389 |

A new Workstation then connected P1/P2 strictly over ETH MAVLink and started dual Preview. Initially
both sides were `QUALIFYING` with fresh expected-sysid HB/ATT/ANY and the cameras were LIVE around
15–16 Hz. Before the 10-minute baseline could begin, P2 changed to `ETH MAVLINK RX STALLED`; P1
continued QUALIFYING and both cameras remained LIVE. NTRIP was still DISCONNECTED.

Ten-second wire evidence during the stable failure contained:

- host heartbeat: 11 to P1 and 11 to P2;
- P1 inbound: 1,586 packets;
- P2 inbound: **zero packets**;
- host UDP listener Recv-Q: zero on both ports;
- NIC capture drop: zero.

P2 remained silent for more than 61 seconds. Diagnostic-only USB still returned sysid1 and sysid2
heartbeat, proving the failure was P2's Ethernet MAVLink instance/task rather than its complete
MAVLink service. USB was not used for the ETH PASS decision.

Artifacts:

- pcap: 156,414 bytes, SHA-256
  `0bb0f0ee41eef003925da1a84c583726c83540245262b583ef84365fe97b44e0`;
- fail-fast checkpoint JSONL: 4,741 bytes, SHA-256
  `0d23152ef86acec51d6fb708af0d2465b26d53e7af93b225629a81b83208c00d`.

The application then completed normal Quit with no Workstation process, camera child, device socket
or native ring remaining.

### A/B conclusion

Changing the physical USB Ethernet adapter does **not** close the failure. Two distinct adapters
reproduce ETH MAVLink loss when camera and Pixhawk traffic share one Layer-2 path, while the same
RTL8156B previously passed more than ten minutes when it carried Pixhawk traffic only and cameras
used the other NIC. The adapter-defect hypothesis is therefore weak; shared-L2 load plus a PX4-side
Ethernet MAVLink failure is the strongest discriminator. Do not add NTRIP to a failed baseline.

## Operator-accepted 10-minute shared-NIC + NTRIP rerun

After another completed physical topology change, all four Ethernet devices were reachable on
RTL8156B `enx0023a40a2e39`. A fresh no-camera peer-seed gate received 1,775 P1 datagrams/69,911 B
and 1,377 P2 datagrams/55,242 B in 12.5 seconds. Workstation then ran both experimental camera
previews, P1/P2 strictly over ETH MAVLink, and the NGII NTRIP stream for an official 600-second
window from 21:39:00 KST. Credentials were process-local and are absent from the artifacts.

The operator accepted this requested single-NIC gate as **PASS**:

- every 30-second checkpoint kept both cameras `LIVE`;
- P1/P2 HB, ATT and ANY remained fresh without `STALE` or `RX_STALLED`;
- the pcap contains 85,013 P1 inbound and 84,407 P2 inbound packets, plus 4,341/4,340 outbound
  packets respectively, through the full capture interval;
- tcpdump reported zero kernel capture drops and the adapter reported zero RX/TX errors or drops;
- the GUI remained responsive; maximum diagnostic traversal latency was 0.168 seconds.

Both sides reached the first 10-minute qualification boundary during the observation. The designed
one-shot calibration PARAM read then reset both qualification timers together. Their immediately
fresh HB/ATT/ANY ages and symmetric wire traffic distinguish that command boundary from link loss.

NTRIP remained `RTCM_ACTIVE` through checkpoint 512.8 seconds; its last RTCM chunk arrived at about
529 seconds. It then became `STALE` and finally `ERROR: timed out` after two reconnect indications.
NetworkManager independently recorded the Wi-Fi Internet path changing from global to site-only at
21:48:45 and returning to global at 21:49:21. The caster TCP port was reachable again after the run,
but `NtripNetworkWorker` had already stopped after its three-consecutive-failure limit and did not
resume automatically. Per the operator's acceptance, this is recorded as a separate NTRIP retry
resilience issue, not a failure of the wired single-NIC camera/Pixhawk gate.

The camera run log also contains short experimental non-IDR recovery intervals, even though none of
the 30-second checkpoints caught a camera outside `LIVE`. Therefore this result verifies connection
continuity, not strict visual-integrity or portable-MP4 integrity.

Artifacts:

- status JSONL: 98,008 bytes, SHA-256
  `c270770287e19431326c7044f3edd68097343d73cb4d134d012f60f46eaa0aea`;
- ETH MAVLink pcap: 17,640,392 bytes, SHA-256
  `aebb70817fea717e8cbe9ef2d5a5adfbd2d3e72aea1c61194594928c1d17c8f2`.

## Second 10-minute shared-NIC + NTRIP run (`enx588694fce07e`)

The next physical single-NIC arrangement moved all four Ethernet devices to
`enx588694fce07e`; `enx0023a40a2e39` had no carrier. The camera source and host routes were already
present. A process-local `10.0.0.10/24` Pixhawk address was added for the run and removed after
normal Quit. The saved settings file returned to its pre-run SHA-256
`d3628ca1a41665300da106fbd11a43023d73f25fb7c69b4759c6aa11a8851070`.

The clean no-camera peer-seed gate received 1,773 P1 datagrams/69,860 B and 1,740 P2
datagrams/68,750 B in 12.5 seconds. The official integrated observation ran for 600 seconds from
22:01:33 KST with dual experimental Preview, P1/P2 strictly over ETH MAVLink, and NGII NTRIP.
Credentials remained process-local and are absent from the artifacts.

Connection-continuity result: **PASS**.

- all 21 checkpoints kept both cameras `LIVE`;
- P1/P2 HB, ATT and ANY remained fresh without `STALE` or `RX_STALLED`;
- the pcap contains 85,016 P1 inbound and 84,408 P2 inbound packets, plus 4,222 outbound packets
  to each side;
- tcpdump reported zero kernel drops; the NIC added zero RX/TX errors or drops while receiving
  8,793,177,889 bytes during the 600-second status window;
- NTRIP stayed `RTCM_ACTIVE` at every checkpoint, ending with 704 chunks/430,883 B, 114 GGA
  transmissions and zero reconnects;
- maximum diagnostic traversal latency was 0.199 seconds and Workstation completed normal Quit.

Both Pixhawks crossed the initial 10-minute qualification boundary during the run. The designed
one-shot calibration PARAM read reset both qualification timers together; final HB/ATT/ANY ages
were 0.78/0.02/0.02 seconds for P1 and 0.23/0.00/0.00 seconds for P2. The monitor therefore exits
non-zero because its comparison phase insists on terminal `LIVE`, even though the captured evidence
contains no link failure.

Visual integrity remains a separate residual risk. Thirty-second checkpoints did not catch either
camera outside `LIVE`, but the camera log recorded short non-IDR integrity recovery intervals during
the window. The mapped recovery boundaries totalled 10 on the left (maximum blocked interval
2.387 seconds) and 5 on the right (maximum 1.843 seconds). This run proves connection continuity and
automatic preview recovery, not a zero-artifact image or portable-MP4 guarantee.

Artifacts:

- status JSONL: 98,244 bytes, SHA-256
  `a44c1ab1cc6b15c7ca568b6a22ff3df898a1d621e172b3d4b53f9896a01ac7f5`;
- ETH MAVLink pcap: 17,711,449 bytes, SHA-256
  `926fc6affc7a94ce83893c123810053f4b3f0f1cbadb5447a67492844f359568`.
