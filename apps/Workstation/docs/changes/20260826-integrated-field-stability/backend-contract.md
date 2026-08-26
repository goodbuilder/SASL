# Backend Contract

## Commands and validation

- Preview succeeds only after each side emits a decoded current-generation frame.
- One explicit dual STUS action preflights both sides with bounded exact-OK HIWS samples, sends Left
  first and sends Right only after an exact Left `OK` ACK. After replacement streams, three unique
  stable post-ACK pad-probe SRs may authorize one signed per-camera correction round. It succeeds
  only after final fresh SR quality, never from ACK alone. No startup/periodic STUS or raw-SDK
  bypass exists, and recording ownership rejects the action.
- MAVLink Connect succeeds only after inbound heartbeat identity; READY requires qualification.
- NTRIP succeeds only after HTTP 200 and CRC-valid RTCM data, not TCP connect alone.
- Measurement reaches RUNNING only after at least one accepted camera side emits a decoded frame
  whose stream generation matches the current pane generation. Video readiness is independent:
  each allocated MP4 path remains STARTING until positive byte growth is observed.

Measurement lifecycle admission depends only on camera/resource transition safety. MAVLink,
GNSS/NTRIP, STUS, intrinsics, stereo R,t, correspondence and storage health are not global Start
gates. They are method-input/result-validity gates and must emit exact rejection/advisory reasons.

## State, ownership, and concurrency

One Workstation process owns both RTSP streams, UDP 14550/14551, camera controls and writers during
qualification. Each camera/Pixhawk retains per-side health and failure isolation. Device commands,
recording replacement and STUS are serialized through existing owners; duplicate requests do not
create additional workers or sockets.

Pixhawk transport open remains a worker operation, while status, completion, failure, cleanup and
receive-timer activation that touch Qt are queued to the subscriber/panel owner thread. Generic
progress cannot repaint topology controls before completion starts the receive timer. Successful
RTCM progress text is bounded per side; exact delivery counters and failures remain uncoalesced.

PX4 v1.17 Stock calibration refresh owns `CAL_MAG0_*` through `CAL_MAG3_*`, never nonexistent
`CAL_MAG4_*`; the complete automatic refresh is 75 magnetometer plus 49 attitude parameters. The
generic explicit parameter API remains unrestricted. During a PARAM or shell reservation, the
command worker remains the sole receive owner and holds the per-side re-entrant send lock. That same
owner services the 1 Hz GCS heartbeat from a 50 ms receive poll. The GUI heartbeat path uses only a
nonblocking send-lock attempt, the healthy peer continues independently, and RTCM/other writers
remain excluded until reservation release. Deferred telemetry preserves its original liveness and
sample timestamps and FIFO delivery, but a historical HEARTBEAT cannot execute stream setup, Stock
profile, or stale-ATTITUDE recovery; the next live HEARTBEAT owns those actions. Post-command
qualification reset remains deliberate and unchanged.

A PX4 reboot on an unchanged host UDP socket is a remote boot epoch, not a replacement host-listener
generation. A regression beyond the 2-second reorder tolerance followed by two increasing per-side
`ATTITUDE.time_boot_ms` low-epoch samples confirms it; regressions up to and including 2 seconds and
natural uint32 wrap do not. This includes a rapid reboot before 10 seconds of prior uptime.
Confirmation invalidates only that side's old-boot identity, qualification and ephemeral
Stock-profile request marker. The next expected-sysid HEARTBEAT owns one side-locked profile
re-request. The healthy peer, persistent stream-configuration markers and setup signal remain
unchanged; this path sends no reboot, shell, parameter or persistent device-configuration command.
Delayed prior-epoch ATTITUDE counts as raw transport only and is not published as current attitude.

Decoder intent and decoder activity are separate state. `set_state(PLAYING)` and worker status text
may name only a candidate. `active_stream_decoders[side]` changes only when a decoded frame with the
current pane generation reaches the frame boundary. A crash, terminal child exit, new candidate, or
explicit stop clears only that side until another current-generation frame arrives. Auto software
overrides and reasons are per side, never a shared Left/Right scalar.

Initial STUS verification does not end monitoring. After VERIFIED, one low-rate owner reevaluates
the freshest receive-close Left/Right pair every second. It recomputes frame and Sender Report age
against current monotonic/wall observations and fails immediately on missing/regressed SR, stale
frame/SR, pair delta above 50 ms, SR-PC offset above 100 ms, or NTP/RTP clock-rate error above 0.5%.
The limit remains an explicit operator setting rather than a hidden clamp. The approved
qualification config uses a 7.0-second SR-age target, and its effective value is recorded with the
run evidence.

The correction owner uses `next target lead = previous target lead - median signed SR-PC offset`.
It admits Left/Right SR identities atomically only after per-side freshness and receive-close pair
boundaries pass: three deduplicated `rtcp-pad-probe` SRs per side from the replacement generation
and after that side's STUS ACK. Feedback span is at most 50 ms and MAD at most 20 ms. HIWS RTT is
bounded at 2.0 seconds, feedback/target lead at ±2.0 seconds, and the complete action at two rounds/
90 seconds. Only a STUS mutation may wait up to 15 seconds for its exact ACK, capped again by the
remaining overall deadline; HIWS and generic SDK control keep their configured timeout. Any limit
or second-round failure is terminal; no value is silently clamped. During
command, reconnect and SR warmup, the action remains one camera-resource/control-channel owner so
Measurement queues and all recording or competing camera-control entry points reject it.

## Worker resource budgets, scheduling, and backpressure

Keep one native ring per active camera, bounded metadata/control/preview queues and latest-only GUI
preview. Accepted pairs alone may pin native frames. Evidence writer ACK/hash is independent of GUI.
No unlimited inference or continuous multi-megabyte Qt queued signal is allowed. Record queue depth,
drops, age, RSS, CPU/GPU and Stop latency under normal, stall and sustained load.

## External boundaries and workers

- WIRIS RTSP/H.264/RTCP and control TCP are external device truth.
- PX4 inbound raw MAVLink is the liveness oracle; host socket state and ping are prerequisites only.
- NTRIP raw HTTP status and independently CRC-checked RTCM3 are the caster oracle.
- ffprobe and strict software decode are independent MP4 oracles.
- Durable readback hash is the evidence oracle, separate from in-memory success.

## Results, errors, timeout, retry, and cancellation

Report missing SPS/PPS/IDR, RTP discontinuity, clean-intra wait, server timeout, decoder health,
MAVLink ATT-only stale, RX stall, NTRIP auth/no-data, storage failure and cleanup failure separately.
Retries are bounded and visible. A stalled PX4 side receives no new RTCM, stream or parameter work;
the healthy side continues. Failure of a short gate cancels the candidate before long soak.

STUS errors distinguish HIWS preflight failure, Left/Right ACK failure or unknown application,
replacement-stream failure, missing/old/poll-detected/unstable SR evidence, correction-limit breach,
nonconvergence, recording conflict, cancellation and overall timeout. ACK-only state is pending;
continuous failure or stream reset after VERIFIED revokes prior success. Shutdown directly cancels
the worker between I/O boundaries, preserves any ACK already received and forbids queued restart.

Camera QThread/process creation is request acceptance, not connection success. First-frame readiness
has a 35-second bound. A timed-out/failed side is stopped; a frame-ready opposite side may continue
as an explicit partial run. If no side proves a current-generation frame, automatic recording is
stopped, the transient generation is retired to ERROR, and a future Start remains admissible.

Video projects per-side STARTING/RUNNING/FAILED and aggregate STARTING/RUNNING/PARTIAL/FAILED.
FAILED/PARTIAL video is durable degraded evidence but does not roll a frame-ready Measurement back
from RUNNING or disable a future Measurement.

GStreamer errors are classified by their owning boundary: RTSP/source/parser, MP4 mux/filesink,
decoder health, or unclassified pipeline. Only direct NVDEC health—or a decoded-output timeout while
RTP input remains current—authorizes Auto CPU fallback. Preview-only fallback restarts inside the
failed side's worker and remains inactive until an `avdec_h264` current-generation frame arrives.
During recording, the failed side finalizes and exits; its override applies on the next side start.
The healthy side is never restarted, and the terminal exit is not fed into crash recovery. Explicit
NVIDIA remains fail-closed.

## Persistence, restart, and shutdown

Existing data is immutable. New streams append or create collision-free files. A fresh application
does not adopt an old mutable run or auto-apply intrinsics. Shutdown drains priority evidence,
finalizes MP4 with EOS when possible, releases sockets/processes/shared memory and records incomplete
postconditions truthfully. A forced external termination must still permit a new run after restart.
Native camera rings carry a Workstation prefix, creator PID, and random token. Startup may unlink
only a strict-prefix regular shared-memory entry owned by the current UID whose embedded creator PID
is no longer alive; current/live owners, malformed names, and arbitrary Python `psm_*` objects are
outside that recovery authority.

Every persisted detection snapshot retains all canonical native descriptor inputs, including the
app-receive `timestamp_s`; a verifier must be able to recompute the descriptor without trusting the
writer implementation. Historical rows that predate this field may source only that missing value
from the same native artifact row whose bytes and fields are covered by the persisted pair ACK.

Workstation and Legacy camera-interface defaults use `10.0.0.11/32` plus explicit camera host
routes. `10.0.0.10/24` belongs exclusively to the Pixhawk NIC. Legacy Session Record is a dual-camera
video request: it starts record-capable camera workers even when Preview is disconnected, expects
both sides before the first GUI frame, and rolls visible state back when the start request is
rejected.

The process-wide cooperative deadline cannot preempt a camera's 12-second stop window or its
nine-second GStreamer EOS wait; when a prior lifecycle step delays camera stop, the force boundary
moves with the camera request. Session-save and HR/OZ capture threads are explicit gates. Forced
cleanup persists a source-backed thread/process/socket/shared-memory report and returns pending
while any inspected owner survives. Per-side video finalization success or failure must be accepted
by the append writer before settings flush, and DataStorage closes last so no late camera outcome is
offered to an already-dead writer.
