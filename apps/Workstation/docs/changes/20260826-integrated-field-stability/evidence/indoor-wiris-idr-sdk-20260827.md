# Indoor WIRIS visible-IDR SDK qualification — 2026-08-27

## Method and acceptance contract

- Purpose: determine whether the public WIRIS Ethernet Stream SDK can make the current FW 2.18.72
  visible sender emit real H.264 IDR slices indoors.
- Device boundary: Left `10.0.0.231:2240/8554`, Right `10.0.0.232:2240/8554`, isolated camera NIC
  source `10.0.0.11`.
- Published source of truth: Workswell *WIRIS & GIS Ethernet Stream SDK*, revision 241106EN. Its
  visible encoder controls are bitrate `SEBR/GEBR`, GOP `SEGS/GEGS`, IDR interval `SEIR/GEIR` and
  QP `SEQZ/GEQZ`; it exposes no force-IDR, request-keyframe or visible encoder-restart command.
- Invariant: bitrate/QP/GOP/slice-size remain `0/1/30/0`; only Left `SEIR VISIBLE` is varied. Camera
  internal recording must remain off. No Workstation, Legacy, RTSP or SDK owner may compete.
- Independent oracle: raw fresh-client RTSP is depayloaded to Annex-B without decoding. Start-code
  NAL headers and FFprobe frames are inspected independently of SDK ACK/readback. `SEIR -> OK` and
  matching `GEIR` are internal consistency only.
- PASS: at least one actual H.264 NAL type 5 plus decodable keyframe evidence in a fresh stream.
  Non-IDR NAL1 I pictures do not pass.
- Non-goals: bitrate, QP, GOP, slice-size, firmware, `REBT`, `SETH`, thermal encoder and geometry are
  not changed. Coordinate contract is unaffected.

## Preconditions and baseline

- Workstation PID 1494249 was idle with no device sockets and was stopped before probing.
- Both hosts routed through the camera NIC and answered ping with zero loss.
- Right readback: FW `2.18.72`, `GETH=TRUE`, `GSSS=0`, `IRCR=FALSE`, visible `0/1/30/1`.
- Left repeatedly read visible `0/1/30/0`; this differs from the previously recorded final IDR=1
  state and proves that the IDR setting had returned to zero. Other fields were stable.
- Right fresh baseline produced 18,946,544 bytes, `NAL1=41`, `NAL7=2`, `NAL8=2`, `NAL9=41`,
  `NAL5=0`, plus missing-PPS/decode/macroblock errors. SHA-256:
  `fe52bceded8efe49bba4d35f5fb18ef44ceb5824adc69c310edd0ec2d930d278`.
- Left IDR=0 fresh baseline produced 134,657,314 bytes, `NAL1=273`, `NAL7=9`, `NAL8=9`,
  `NAL9=273`, `NAL5=0`. SHA-256:
  `9a1ddacbfa427bb2f97ee530f1992c6ae5ad9632441e3704600e2394e897c7ec`.

## Left controlled A/B

| Requested IDR | SDK ACK/readback | Raw bytes | Annex-B NAL counts | FFprobe decoded classification | Result |
|---:|---|---:|---|---|---|
| 1 | exact `OK`, `GEIR=1`; all other fields unchanged | 125,668,831 | NAL1=259, NAL7=7, NAL8=7, NAL9=259, **NAL5=0** | 4 non-key I, 127 non-key P | FAIL |
| 2 | exact `OK`, `GEIR=2`; all other fields unchanged | 151,512,315 | NAL1=311, NAL7=10, NAL8=10, NAL9=311, **NAL5=0** | 6 non-key I, 213 non-key P | FAIL |
| 3 | exact `OK`, `GEIR=3`; all other fields unchanged | 132,653,954 | NAL1=272, NAL7=11, NAL8=11, NAL9=272, **NAL5=0** | 10 non-key I, 182 non-key P | FAIL |

Artifact SHA-256 values, respectively:

- `5dd6052478b00bcc8338b0315daabb30094693ee32e0be7b83427308d3c8b1d2`
- `39a94447c27e2c10c0150dabc9a7dbe21b90364a6a59b9edac59fcb16298d300`
- `7ee16c92756fac03468ab914dad40532940e57062426c0ed7578d098fe86e2d8`

The volatile raw artifacts are under `/tmp/wiris-idr-indoor-20260827-8KMaIK/`; they are not source
or required repository state. Left was restored with exact `SEIR VISIBLE 1 -> OK`; after 15 seconds
its final readback was `HIWS=OK`, `GETH=TRUE`, `GSSS=0`, `IRCR=FALSE`, visible `0/1/30/1`, and
FTP/SDK/RTSP ports were open.

## Independent service failure

Right received no SET command. During the final read-only audit its SDK connection changed to
`ECONNREFUSED`; initially FTP remained open while SDK/RTSP were closed, then all ports 21/2240/8554
remained closed for more than 60 seconds while ICMP ping stayed healthy. This reproduces the known
FW 2.18.72 service-collapse boundary from repeated SDK access and is not evidence that a Right IDR
mutation failed. No `REBT` or other recovery command was sent.

The operator physically power-cycled only the Right WIRIS camera. Ping and ports 21/2240/8554 then
remained open across a 20-second observation. Spaced readback returned `HIWS=OK`, `GETH=TRUE`,
`GSSS=0`, `IRCR=FALSE`, visible `0/1/30/0`; the reboot had reset only the IDR interval as previously
observed. One recovery command `SEIR VISIBLE 1` returned exact `OK`. Readback was `GEIR=1` after
five and twenty seconds, `HIWS=OK`, and all three services remained open. No Right A/B mutation or
raw-stream repeat was performed.

## Result and next boundary

- **Public-SDK IDR solution: FIELD FAIL.** Values 1, 2 and 3 are accepted and read back but do not
  alter the elementary-stream truth: every VCL is NAL1 and no NAL5 exists. Prior evidence also
  covers IDR 0/1 with depay keyframe requests, active-stream reapply, fresh clients and sequential
  physical reboot. Repeating value 5 cannot establish a distinct mechanism and was not attempted.
- The defect is therefore below the exposed SDK setting boundary: FW encoder/packetizer behavior or
  an undocumented vendor control. Request a Workswell firmware fix or vendor-only force-IDR/
  encoder-restart procedure with the hashes and counts above.
- Final device state is Left and Right `GETH=TRUE`, `GSSS=0`, `IRCR=FALSE`, visible `0/1/30/1`;
  both camera service sets are open. The Right physical recovery is complete. Do not repeat public
  IDR interval A/B testing; pursue Workswell firmware/support instead.

## Offline non-IDR-intra recovery evaluation

RFC 6184 gives only an IDR picture, or a gradual-recovery procedure explicitly signalled by a
recovery-point SEI, a standardized decoder-refresh meaning. These WIRIS artifacts contain no NAL5
and no NAL6 recovery-point SEI, so a NAL1 I picture is not renamed or persisted as an IDR/keyframe.

To evaluate a device-specific limited workaround, FFprobe independently located every decoded
non-key I picture's raw packet offset. Each byte range from one I offset up to the next I offset was
then piped into a newly created FFmpeg software decoder with `-xerror`, which clears any earlier
decoder reference state and fails on the first decode error.

| Artifact | Adjacent NAL1-I GOP segments | Fresh-decoder result |
|---|---:|---|
| Left IDR setting 1 | 3 | 3 PASS / 0 FAIL |
| Left IDR setting 2 | 5 | 5 PASS / 0 FAIL |
| Left IDR setting 3 | 9 | 9 PASS / 0 FAIL |
| Total Left | 17 | **17 PASS / 0 FAIL** |

The Right baseline had only one decoded non-key I picture before its short stream ended. Decoding
from that I offset still failed later with `P sub_mb_type 11 out of range`, macroblock decode error
and `corrupt decoded frame`: the I picture can provide a practical re-entry point but cannot prevent
a later sender packet omission.

This supports only a conditional, not-yet-approved policy: after startup or a detected RTP/AU
discontinuity, keep all outputs blocked, clear/recreate the side's decoder, and feed it only from a
complete SPS/PPS-backed NAL1 access unit whose slices are intra and whose PTS/AU flags are clean.
Resume Preview first after a bounded clean probation; latch again immediately on the next sequence
gap, missing fragment/PTS, DISCONT, CORRUPTED flag or decoder error. Never call this IDR, never let it
authorize direct-copy MP4 integrity, and do not silently promote Measurement/native evidence before
the separate field quality gate passes.
