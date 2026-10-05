# STUS high-latency closed-loop correction — 2026-08-27

## Field RED and independent oracle

The active-stream STUS run used the applied 7.0-second SR-age limit, 50 ms pair limit and 100 ms
SR-PC limit. It returned exact Left then Right `OK`, replaced the local RTSP workers and received
fresh Sender Reports, but the final UI state was FAILED:

- Left/Right RTCP/NTP delta: 510.6 ms;
- maximum absolute SR-PC offset: 1,312.5 ms;
- `PC frame receive - frame NTP`: Left +916.9 ms, Right +1,427.6 ms.

The independent control artifact is `/tmp/wiris_stus_20260827.pcap`, 7,354 bytes, SHA-256
`d901c3f8525afc34d649100c3adc9780e9b6fd96b3748386ee0454769c88b00a`. Exact reassembly gives:

| Side | HIWS RTT samples | Median | STUS payload to exact OK |
|---|---:|---:|---:|
| Left | 934.225 / 509.790 / 514.990 ms | 514.990 ms | 3.215908 s |
| Right | 506.816 / 542.501 / 555.396 ms | 542.501 ms | 2.245313 s |

Right's first TCP SYN and first HIWS application payload followed the complete Left STUS `OK` by
238 and 777 microseconds respectively. The historical 250 ms RTT cap rejected every successful
probe, so the worker silently used approximately zero target lead.

Independent unique pad-probe Sender Report analysis fixes the correction sign and metric:

- `rtcp_sr_pc_offset_s = camera SR NTP - PC SR receive UTC`;
- Left median `-0.736135482 s` from 20 unique SRs, MAD `0.086 ms`;
- Right median `-1.307044029 s` from 21 unique SRs, MAD `0.376 ms`;
- corrected frame-transport medians were 103.199 ms Left and 114.285 ms Right.

Therefore both camera clocks were behind the PC receive basis and Right was about 570.9 ms behind
Left. The +916.9/+1,427.6 ms values are not signed clock offsets; they also contain encoder/RTSP/
decode transport. They are excluded from the actuator. STUS ACK duration is also excluded because
the ACK does not expose the camera's internal apply instant.

## Accepted correction contract

One explicit Set camera clock action now owns the complete finite state machine:

```text
both-side exact-OK HIWS preflight
  -> initial STUS: Left exact OK -> Right exact OK
  -> replacement RTSP generation
  -> at least 3 unique, stable, post-ACK rtcp-pad-probe SRs per side
  -> if needed, one correction STUS: Left exact OK -> Right exact OK
  -> replacement generation + 3 unique stable SRs
  -> hard quality gate -> VERIFIED -> existing 1 Hz revocation monitor
```

For each side the correction is cumulative:

```text
next_target_lead = previous_target_lead - median(signed SR-PC offset)
```

A negative offset moves the next target forward; a positive offset reduces the lead and may produce
a bounded negative lead. Initial RTT/2 plus 2 ms is only a seed. Automatic correction is limited to
one retry, two total STUS rounds, 90 seconds overall and a signed `±2.0 s` feedback/target bound.
There is no clamp-to-success. Missing/unstable/poll-detected/stale/old-generation/pre-ACK evidence,
either HIWS preflight failure, any non-exact ACK, recording ownership, timeout, cancellation or
second-round nonconvergence ends FAILED and authorizes no further mutation.

ACK-only evidence is `pending-verification`, not `success`. Final stable SR quality changes it to
`success`; a later 1 Hz failure changes it to `revoked`. Raw SDK `STUS ...` is rejected so it cannot
bypass recording, ordering or evidence interlocks. Measurement may remain available, but an active
STUS action is a camera-resource transition and queues it until the action is terminal. Shutdown
directly cancels the worker between I/O boundaries and invalidates queued Measurement retry.

## Software evidence

The initial new focused contract was RED with seven failures: real high RTT was discarded, the
signed formula and explicit correction worker did not exist, no correction was requested, and
operation timeout/recording cancellation were absent. After the slice:

- STUS CRITICAL selector: **24 passed**;
- complete STUS/camera controller files after final shutdown hardening: **95 passed**;
- camera/reconnect/decoder dependent group: **144 passed**;
- time-sync evidence/status group: **34 passed**;
- lifecycle/recording/delegate group: **70 passed**;
- generated delegate check, Python compilation and `git diff --check`: PASS.

Coverage includes field-scale high RTT, 2-of-3 exact-OK preflight, zero mutation on either-side
preflight failure, Left failure skipping Right in both rounds, cumulative signed correction,
negative target preservation, three unique SR deduplication, invalid-sample exclusion,
median/MAD/span gates, poll fallback rejection, generation and post-ACK freshness, convergence,
second-round failure, 90-second timeout, recording/Measurement interlock, raw-STUS rejection, ACK
pending, verified success, revocation, direct worker cancellation and late-callback suppression.

This was the pre-hardware automated checkpoint. The later physical response boundary and its
remaining zero-IDR blocker are recorded below; software evidence alone does not close R3.

## Hardware timeout A/B and zero-IDR interlock

The first corrected hardware action passed the Left HIWS preflight at a roughly 785 ms median and
sent `STUS 1787763529 732618` at pcap time `1787763529.338460`. The camera ACKed the TCP bytes
immediately, but the client closed at `1787763534.343305`, exactly 5.005 seconds later. The camera
sent exact `OK` at `1787763538.214184`, 8.876 seconds after the command and 3.871 seconds after the
client FIN; the host necessarily reset that late payload. Right was not sent. This proves the
generic five-second control timeout, not the clock math or link, caused this FIELD RED.

Artifact: `/tmp/stus_closed_loop_real_control_rtsp_20260827.pcap`, 2,292,605,945 bytes, SHA-256
`e110ef5614cc82342a01446cb561987099a7e75a69d4911d4179e7bb425127a6`, capture drop zero.

A non-persistent UI-only diagnostic changed the generic timeout from 5 to 15 seconds; it was not
saved to settings. One explicit retry then returned exact ACKs on both sides:

- Left STUS payload at `1787763733.952434`, exact `OK` at `1787763737.643175` (3.691 seconds);
- Right STUS payload at `1787763737.643584`, exact `OK` at `1787763741.076349` (3.433 seconds).

Right started only after the complete Left exact OK. Artifact:
`/tmp/stus_closed_loop_timeout15_ab_20260827.pcap`, 7,266 bytes, SHA-256
`01979b218c7e43fcda521e8e2573b4c8003510aad4cb846d600fdcf266679d19`, capture drop zero. The
production fix now gives only the actual STUS mutation a bounded 15-second response timeout; HIWS
and generic commands retain the configured 5-second timeout and the overall action remains 90
seconds.

Post-ACK replacement workers loaded the new bitstream fail-close gate. Both real camera streams
produced no H.264 NAL type 5 IDR for 30 seconds, so neither current-generation decoded-frame ACK was
admitted. Both sides terminated `INTEGRITY_RECOVERY_TIMEOUT` without an automatic reconnect and
STUS truthfully ended `not-verified`; the second correction round was not authorized.

An independent 25-second direct RTSP capture after the two exact ACKs found four unique RTCP Sender
Reports per side. Median signed `camera SR NTP - pcap receive UTC` was -313.712 ms Left and
-808.643 ms Right, leaving about 494.931 ms relative clock offset after the seed round. This is
expected to require the bounded second feedback round, but product code correctly refuses to use
an unverifiable stream generation. Artifact: `/tmp/stus_postack_direct_rtsp_25s_20260827.pcap`,
373,541,679 bytes, SHA-256
`3ea621ec721d506c525616ab3f9d286a2b21b8d992be2ef88b3c82e3a7733968`, capture drop zero.

Hardware result remains **CRITICAL FIELD FAIL / blocked by the zero-IDR sender**: the response-time
bug is fixed and Left-before-Right exact ACK is proven, but convergence and the required 25-minute
continuous quality gate cannot pass until a fresh stream supplies an independently verifiable
random-access frame.

## First corrected-build hardware RED: STUS applies slower than generic control timeout

The next exact control capture proved the high-RTT preflight fix worked: Left HIWS median was about
785 ms. Left `STUS 1787763529 732618` was sent at Unix `1787763529.338460` and TCP-ACKed
immediately, but the old generic 5-second SDK timeout closed the client socket at
`1787763534.343305`. The camera returned exact `OK\r\n` at `1787763538.214184`: 8.876 seconds after
the command and 3.871 seconds after client FIN, so the host reset it. Right remained correctly
untouched.

STUS mutation response waiting is now a separate bound: at most 15 seconds, further reduced by the
remaining 90-second operation deadline. HIWS probes and every generic SDK/zoom command retain their
configured timeout. A returned ACK is still preserved before cancellation/deadline evaluation;
timeout or deadline failure remains terminal and cannot authorize Right or a third round.

Focused automated evidence after this boundary change:

- 8.9-second field-scale STUS ACK with two 15-second command bounds: PASS;
- generic 5-second HIWS timeout unchanged: PASS;
- 15-second STUS timeout leaves Right untouched: PASS;
- remaining-overall-deadline and post-I/O deadline evidence: PASS;
- complete `test_camera_time_sync_continuous.py`: **32 passed**;
- adjacent Workstation STUS/camera-sync/recording interlocks: **9 passed**.

The 15-second physical response boundary is PASS: Left returned exact OK in 3.691 seconds and only
then did Right return exact OK in 3.433 seconds. R3 remains CRITICAL FIELD FAIL because the zero-IDR
replacement generations blocked post-ACK verification and the second closed-loop round; see the
hardware section above.
