# Backend Contract

## Commands and validation

- Preview succeeds only after each side emits a decoded current-generation frame.
- STUS succeeds only after both command ACKs, worker replacement, fresh RTCP warmup and quality gate.
- MAVLink Connect succeeds only after inbound heartbeat identity; READY requires qualification.
- NTRIP succeeds only after HTTP 200 and CRC-valid RTCM data, not TCP connect alone.
- Measurement/recording succeeds only after the new run owns resources and durable outputs advance.

Measurement lifecycle admission depends only on camera/resource transition safety. MAVLink,
GNSS/NTRIP, STUS, intrinsics, stereo R,t, correspondence and storage health are not global Start
gates. They are method-input/result-validity gates and must emit exact rejection/advisory reasons.

## State, ownership, and concurrency

One Workstation process owns both RTSP streams, UDP 14550/14551, camera controls and writers during
qualification. Each camera/Pixhawk retains per-side health and failure isolation. Device commands,
recording replacement and STUS are serialized through existing owners; duplicate requests do not
create additional workers or sockets.

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

## Persistence, restart, and shutdown

Existing data is immutable. New streams append or create collision-free files. A fresh application
does not adopt an old mutable run or auto-apply intrinsics. Shutdown drains priority evidence,
finalizes MP4 with EOS when possible, releases sockets/processes/shared memory and records incomplete
postconditions truthfully. A forced external termination must still permit a new run after restart.
