# Camera-only MAVLink trigger, STUS and real MP4 — 2026-08-27

## Preconditions

- Latest post-freeze Workstation owned P1/P2 UDP and had passed ten continuous telemetry-only
  minutes with no Preview or NTRIP.
- The camera device profile independently read `0/1/30/0` on both sides before Preview.
- NTRIP, recording and Measurement were initially stopped. NIC drop/error/missed/collision counters
  were zero.

## Camera-only trigger

Preview reached current-frame LIVE on both sides with actual `GPU · nvh264dec`, frame ages about
0.05–0.10 seconds and observed rates about 15.9/16.7 Hz. The operator identified visible image
corruption; therefore Camera R1/R2 remains FIELD FAIL even though decoder/status truth was correct.

Within roughly 20 seconds of starting camera TCP Preview, both P1 and P2 changed to
`ETH MAVLINK RX STALLED`. NTRIP was off. Preview itself remained LIVE.

After stopping Preview:

- P1/P2 ping remained 3/3.
- NIC counters remained zero-error/drop and there was no carrier reset.
- A ten-second pcap contained only 19 host GCS heartbeats and zero inbound Pixhawk packet.
- Both ttyACM0 and ttyACM1 returned no USB heartbeat in eight seconds.
- Stopping camera traffic did not restore MAVLink.

Capture: `/tmp/camera_only_mav_stall_20260827.pcap`, SHA-256
`a65f0374ff2d0040726ed83063508b1e10bf33b2f6b4ee38787307c6b385c5c5`.

This confirms camera traffic as a reproducible trigger for a persistent PX4-side/global MAVLink
service stall. It does not by itself prove the internal IOB/sendto mechanism of PX4 #25956.

## NIC-isolated camera-load retest

The operator physically separated the devices on 2026-08-27. Cameras remain on
`enx588694fce07e`; P1/P2 moved to `enx8cb0e9e8f8ee`. NetworkManager now persists:

- camera NIC: `10.0.0.11/32` plus host routes for `10.0.0.231/232` with preferred source
  `10.0.0.11`;
- Pixhawk NIC: `10.0.0.10/24` plus explicit routes for `10.0.0.2/3`.

All four devices answered 3/3 ping and both camera RTSP/control ports (`8554`, `2240`) accepted TCP.
A 15-second no-camera baseline on the Pixhawk NIC contained 77 P1 and 75 P2 MAVLink datagrams.
Artifact: `/tmp/nic_split_pixhawk_20260827.pcap`, SHA-256
`2ca721ece7d9ca3dd282fa642d081855ba9454419044e3541b367d1b218b3e99`.

Two direct RTSP clients then pulled both camera streams concurrently for 45 seconds. The camera NIC
capture contained 681,221 packets over 45.012 seconds (658,391,361 bytes), while the Pixhawk NIC
continued to receive:

- P1: 220 MAVLink datagrams over 44.999 seconds, maximum inter-packet gap 0.869 seconds;
- P2: 233 MAVLink datagrams over 45.691 seconds, maximum inter-packet gap 0.675 seconds.

Post-load ping remained 3/3 for all four devices and both NICs still reported zero link errors and
drops. Artifacts: `/tmp/nic_split_camera_load_45s_20260827.pcap`, SHA-256
`010521555eacc93c2dfa647170a2bcadac59013967433d58d604f0dc4f87014f`, and
`/tmp/nic_split_pixhawk_during_camera_45s_20260827.pcap`, SHA-256
`e86aa9717e2e81bafab3a27953c6301acc64a5e47a946870dc66d17dfe0dcd22`.

The former approximately-20-second dual MAVLink stall did not reproduce at the network boundary
after isolation. This is a short boundary PASS, not yet the application Preview or final duration
gate. It narrows the trigger to the former shared camera/PX4 Layer-2 path but still does not prove
the PX4-internal #25956 mechanism.

The current Workstation then ran both real GPU/NVDEC Preview streams while connected to both
Pixhawks. Final wire capture duration was 898.987 seconds, including a 116-second pre-app broadcast
baseline and about 783 seconds of application-owned unicast. During the application interval:

- P1 delivered 110,057 decoded MAVLink messages; 23,462 were ATTITUDE and 783 were HEARTBEAT.
  Maximum ATTITUDE/HEARTBEAT gaps were 65.3 ms / 1.003 s.
- P2 delivered 109,289 messages; 23,466 were ATTITUDE and 783 were HEARTBEAT. Maximum
  ATTITUDE/HEARTBEAT gaps were 634.3 ms / 1.004 s.
- Host heartbeat maximum gaps were 1.033 s Left and 1.044 s Right.
- At exactly 600 seconds after connection, automatic PARAM readback sent 124 requests per side,
  received 124 values per side in 0.227/0.250 seconds, and did not starve heartbeat. The UI then
  deliberately began a new post-command ten-minute qualification; this was not STALE.
- Both NICs and both tcpdump captures retained zero kernel/link drops and all UI navigation
  round-trip actions remained responsive under the still-saturated Orca/AT-SPI condition.

Artifact: `/tmp/nic_split_app_preview_mavlink_20260827.pcap`, 21,811,000 bytes, SHA-256
`2b07f7578df2fc6cbc88a8c143f25cf022e2b12907ab1db130c936212edb6f62`.

Therefore the isolated-NIC application Preview short qualification is PASS and the former shared
camera-load MAVLink stall is not reproduced. Final 60-minute/cold-boot acceptance remains pending.

The operator subsequently replaced the P1/P2 adapter. The original ASIX/CDC-NCM device
`enx8cb0e9e8f8ee` disappeared as its upstream USB hub was physically removed; the replacement
RTL8156B enumerated as `enx0023a40a2e39`. NetworkManager now persists the same isolated contract on
the replacement: `10.0.0.10/24`, explicit `.2/.3` host routes, no default route and USB runtime
power forced on for the NIC. P1/P2 each answered 5/5 before load and 3/3 afterward.

A repeated direct dual-RTSP 45-second boundary returned P1 221 packets over 45.656 seconds and P2
232 over 45.779 seconds; maximum gaps were 0.956/0.983 seconds, with zero capture/NIC drop or error.
Artifact: `/tmp/new_rtl8156b_pixhawk_during_camera_45s_20260827.pcap`, 48,783 bytes, SHA-256
`6a1fe989f97d948f3f532a39eb71d1ba0fe0e0e3b563293bf34539c7c91bf55d`.
The replacement NIC short boundary PASS did not inherit the earlier adapter's 10-minute app PASS,
so the exact-adapter gate was repeated after all software fixes.

The latest app then sustained P1/P2 on RTL8156B for the full qualification and post-command window.
The 736.229-second capture contains:

- P1 103,610 decoded messages, including 22,087 ATTITUDE and 736 HEARTBEAT; maximum gaps
  38.672 ms / 1.003910 s;
- P2 102,864 messages, including 22,087 ATTITUDE and 736 HEARTBEAT; maximum gaps
  38.628 ms / 1.003027 s;
- exactly 15 initial interval commands and one 124-parameter refresh per side;
- 124/124 PARAM_VALUE replies in 55 ms Left and 64 ms Right, approximately 600 seconds after
  application ownership;
- host GCS heartbeat maximum gaps 1.025034/1.024667 seconds.

The UI remained responsive, Recv-Q samples peaked at only 8,960 bytes and drained, and no Qt
callback, traceback, `RX STALLED`, timer warning, capture drop or NIC error occurred. Final Quit
returned zero with empty stderr and resource report complete. Artifact:
`/tmp/rtl8156b_latest_app_10m_20260827.pcap`, 21,226,564 bytes, SHA-256
`55df19841ea123ff55dfc3c11132708fb8d9698f2b70b29f2d17bdc7aa8cccdd`.
The replacement-NIC application ten-minute gate is FIELD PASS; final cold-boot/60-minute acceptance
remains pending.

## No automatic encoder mutation

The Preview control capture contained no TCP control packet at all:
`/tmp/wiris_control_camera_only_20260827.pcap`, 24-byte empty pcap, SHA-256
`704e5e5b3234433c01fcfd1b20a306e77e985038120492dc53965c3edd38a4ea`.
Post-run readback returned `0/1/30/0` on both cameras. One Right read returned empty GOP/IDR fields;
a bounded retry returned 30/0. No `SEBR/SEQZ/SEGS/SEIR` was sent. Workstation R13 PASS; Legacy
sequential pre/post evidence remains pending.

RTSP header capture: `/tmp/wiris_rtsp_camera_only_20260827.pcap`, 135,428,089 bytes, SHA-256
`915abfacd57cb62034a62b4c4b848605294f1045ae6f406f57aea0bec6467e29`.

The longer isolated-NIC app run independently reconfirmed sender corruption. Despite RTSP-over-TCP
and zero NIC errors, integrity summaries reached at least Left 31 events/41 missing RTP sequence
numbers and Right 121 events/652 missing numbers. The old recovery path resumed on parser
`non-delta AU`, which was not proof of an independently decodable H.264 random-access point.

The corrected worker now starts and recovers fail-closed: parser output is inspected for a real NAL
type 5 IDR and only the decoded frame with that exact PTS can release Preview/YOLO/recording. On
fresh post-STUS generations both physical senders supplied zero verified IDR for 30 seconds. Each
side terminated `INTEGRITY_RECOVERY_TIMEOUT`, did not reconnect automatically and exposed no frame
evidence. Thus corrupted blocks are no longer presented as healthy video, but usable camera output
remains FIELD FAIL until the sender emits a genuine random-access boundary.

The operator then explicitly approved changing the camera SDK setting. With RTSP and recording
stopped, Workstation-compatible SDK calls changed only `SEIR VISIBLE` on each camera:

| Side | Firmware | Before `bitrate/QP/GOP/IDR` | SET ACK | After readback |
|---|---|---|---|---|
| Left | 2.18.72 | `0/1/30/0` | exact `OK` | `0/1/30/1` |
| Right | 2.18.72 | `0/1/30/0` | exact `OK` | `0/1/30/1` |

Control artifact: `/tmp/wiris_idr_interval_apply_20260827.pcap`, 16,510 bytes, SHA-256
`f77ee9b52aead41dd322a97afa4c74f41d443583c08f19c741f45d4627dc91ca`, capture drop zero.
This is the documented Workswell `SEIR/GEIR VISIBLE` surface; other encoder values remained exact.

Readback success did not change the running encoder. Two fresh 15-second elementary-stream captures
still contained no NAL type 5 IDR and ffprobe classified every I-picture as `key_frame=0`:

- Left: 97,894,016 bytes, 206 AUs, 9 non-IDR I-pictures, IDR 0, SHA-256
  `aa302c5e06eb8353e90ec5177ee03632624d61e24fff858164551fd1ef97d6dd`;
- Right: 46,108,841 bytes, 112 AUs, 3 non-IDR I-pictures, IDR 0, SHA-256
  `6826a79a5dc3e4ec0ea248730e99c7e0408960b6212c7444e18a383effc061f2`.

## Operator-approved sequential camera reboot

The operator approved one SDK reboot per camera. Workstation/recording/RTSP clients were absent;
each side was rebooted and completely verified before touching its peer.

Left returned exact `REBT -> OK`, then control/RTSP and ping went down before both services returned.
After reboot its live profile was `0/1/30/0`: IDR `1` had not persisted. A fresh 15-second stream
contained 219 NAL1 VCL AUs, seven SPS/PPS sets and zero NAL5. With RTSP actively established,
`SEIR VISIBLE 1` again returned exact OK/readback 1; a subsequent stream still contained 214 NAL1
AUs, five non-IDR I-pictures and zero NAL5.

Right also returned exact `REBT -> OK` and proved a down/up transition. Unlike Left, it rebooted to
`GETH=FALSE` and profile `0/17/16/0`, so RTSP remained closed. The approved operating state was
restored with exact `SEQZ VISIBLE 1`, `SEGS VISIBLE 30`, `SEIR VISIBLE 1`, `SETH TRUE` ACKs and
readback `GETH=TRUE`, `0/1/30/1`. Its fresh stream contained 214 NAL1 AUs/eight non-IDR I-pictures
and zero NAL5. Reapplying `SEIR VISIBLE 1` while Right RTSP was actively established still yielded a
269-AU stream with eight non-IDR I-pictures and zero NAL5.

Final device state is healthy/reachable on both sides: `GETH=TRUE`, firmware 2.18.72, encoder
`0/1/30/1`, both 2240/8554 ports open, 3/3 ping and zero NIC errors/drops. Therefore reboot,
inactive-stream SET and active-stream SET have all been exhausted. `GEIR=1` readback is not truthful
NAL5 output on this firmware; Workswell firmware/vendor correction is now the external owner.

Primary artifacts:

- Left reboot pcap `0548bf49f5c9360ec97c01dfdffb35fc0d5e466bb92482cee9b0badbebbb4301`;
  post-reboot H.264 `e2a4225444e8b77a8edb2a99212e629f89c37f176a5a9e3b75538e38573fde15`;
  active-SET H.264 `80dc48f16a5d7888575b1b664fe8a134470f478084037313ceee64af38c02aa3`.
- Right reboot pcap `27069cbf8754c2244955fdd9e41d9679bae86ab842adb7e159849abfb7bcf14e`;
  restore pcap `1781f7d4b3dd8efe8644a4c6b3b3f5f2eb3a9fa4ad8d33ead37f631189991a06`;
  restored H.264 `920496ee0c2b03a381b6dc3e46c0f03f682c15c5c0a35f1c52068bb85783997e`;
  active-SET H.264 `f863130be13517ba856d3d4b4a2bdc642ce7e9fbb69a48775721f39a521511a0`.

## Public SDK/firmware-path audit and final no-mutation check

The official rolling SDK link still serves Workswell revision 241106EN (6 November 2024). Its Stream
surface documents visible bitrate, GOP, IDR interval and QP SET/GET commands, but no per-encoder
restart, force-IDR or runtime-reset command. Workswell's public site directs registered-device
firmware and updates to My.Workswell and technical inquiries to `tech@workswell.eu`; no public
2.18.72 IDR fix or release note was found. No speculative SDK SET was sent from this audit.

A final sequential read-only check after the abnormal-restart regression returned exact values on
both cameras: `HIWS=OK`, `IACT=TRUE`, `GFWV=2.18.72`, `GETH=TRUE`, visible
`GEBR/GEQZ/GEGS/GEIR=0/1/30/1`, `GSSS=0`, and `IRCR=FALSE`. The camera NIC still had zero errors and
zero drops.

The vendor request must ask Workswell to:

1. confirm whether `GEIR VISIBLE=1` on FW 2.18.72 is intended to emit H.264 NAL type 5 or only
   configure non-IDR intra pictures;
2. provide a firmware version that emits genuine periodic random-access boundaries on `/visible`,
   with image hash, compatibility and rollback procedure; or
3. provide the documented encoder restart/force-IDR procedure if it exists outside rev 241106EN.

Until one of those answers is available, further camera mutation has no evidence-backed candidate
and cannot close clean Preview, STUS round-two verification, or portable MP4.

## STUS

The visible SR stale limit was Applied at 7.00 seconds and the canonical data-root setting was
updated to 7.0 seconds.

Independent control pcap reconstruction proves exact ordering:

- Left STUS exact `OK` completed at 00:54:15.558112.
- Right TCP SYN began 238 microseconds later and its first HIWS command 777 microseconds later.
- Right STUS exact `OK` completed at 00:54:19.410950.

Artifact: `/tmp/wiris_stus_20260827.pcap`, SHA-256
`d901c3f8525afc34d649100c3adc9780e9b6fd96b3748386ee0454769c88b00a`.

The real active-stream quality gate failed:

- Left/Right RTCP/NTP delta: 510.6 ms, limit 50 ms.
- RTCP SR/PC UTC offset: 1,312.5 ms, limit 100 ms.
- frame receive delays: Left about +916.9 ms, Right about +1,427.6 ms.

The historical implementation rejected all 0.51–0.93-second HIWS RTT probes because its maximum was
0.25 seconds, so it applied no compensation. The later bounded closed-loop/15-second response slice
is recorded in `stus-closed-loop-correction-20260827.md`; exact Left-then-Right ACK now passes, while
zero-IDR replacement generations still block round-two verification and keep synchronization FIELD
FAIL.

## Real Measurement and MP4

Measurement admitted camera lifecycle despite invalid MAVLink/STUS/geometry and truthfully showed
DEGRADED. Both video rows reached RUNNING only after positive growth. Final files:

- Left: 637,024,427 bytes, 94.4116 seconds, 1,666 H.264 1280x720 frames, SHA-256
  `dfab5f25c7b443bac3d65a9792d1090ab7ff48a3bc8e6364342da85e31fc49eb`.
- Right: 634,730,053 bytes, 92.1790 seconds, 1,503 H.264 1280x720 frames, SHA-256
  `96c4e0e3044916af7aa1f920e5f2d16eb971b9e7021f4a4ef496f79ea73f37ad`.

Both begin with MP4 `ftypmp42`, pass ffprobe, and return zero from the configured `ffmpeg -xerror`
decode command. Container packet PTS and DTS are strictly increasing. Durable `video_finalized`
evidence also reports EOS/mp4mux success and zero RTP gap/missing-packet events.

Those checks are insufficient and the real files remain **FIELD FAIL** for portable bitstream
integrity. Independent elementary-stream accounting found that FFmpeg produced only 1,565 decoded
frames from 1,666 Left H.264 samples and 1,245 from 1,503 Right samples. Both files contain zero IDR
NAL units, expose H.264 `frame_num` discontinuities, and OpenH264 rejects both streams. The final
sample-table duration is zero and Right reports the bogus nominal `r_frame_rate=10000/1`. The
historical RTP sequence metric could not see complete sender-frame omissions, so its zero-gap
success was a false positive. Recording finalization/file persistence PASS; usable, portable MP4
integrity FIELD FAIL. The asynchronous fail-closed validator is now software PASS and rejects these
exact real files in 4.631/5.129 seconds; a valid 60-minute/108,000-sample fixture passes. The sender
must still supply a true IDR-safe stream (or an approved alternate encoding path).
