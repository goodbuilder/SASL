# Backend Contract

## Commands and validation

- Preview succeeds only after each side emits a decoded current-generation frame.
- One explicit dual STUS action preflights both sides with bounded exact-OK HIWS samples, sends Left
  first and sends Right only after an exact Left `OK` ACK. After observer replacement, three unique
  stable post-ACK pad-probe SRs may authorize one Left/Right differential correction round. It succeeds
  only after final fresh SR quality, never from ACK alone. Auto requires a successful manual seed
  in the current connection generation; no startup mutation or raw-SDK bypass exists. Manual/Auto
  STUS may run during active Measurement/YOLO/recording without taking
  their video or writer ownership.
- MAVLink Connect succeeds only after inbound heartbeat identity; `LIVE` requires 60 continuous
  seconds of fresh expected-system HEARTBEAT and ATTITUDE.
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
`CAL_MAG4_*`; an explicit full refresh is 75 magnetometer plus 49 attitude parameters. Reaching
`LIVE`, startup, reconnect, status polling, Measurement, and recording never trigger it. The generic
explicit parameter API remains unrestricted. During a PARAM or shell reservation, the
command worker remains the sole receive owner and holds the per-side re-entrant send lock. That same
owner services the 1 Hz GCS heartbeat from a 50 ms receive poll. The GUI heartbeat path uses only a
nonblocking send-lock attempt, the healthy peer continues independently, and RTCM/other writers
remain excluded until reservation release. Deferred telemetry preserves its original liveness and
sample timestamps and FIFO delivery, but a historical HEARTBEAT cannot execute stream setup, Stock
profile, or stale-ATTITUDE recovery; the next live HEARTBEAT owns those actions. Post-command
qualification reset remains deliberate and unchanged.

The qualification duration is an operational readiness gate, currently 60 seconds. Initial
connection, reconnect, confirmed remote reboot, resumed all-RX stall and explicit/manual command
completion require a fresh interval. ATTITUDE stale and all-RX stall thresholds remain independent
and immediately revoke dependent eligibility. Long 10/60-minute field observations do not alter
this reducer state and remain separate acceptance evidence.

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
fresh decoder-independent Left/Right Sender Reports every second. It fails on missing/regressed SR,
stale SR, sustained Left/Right relative clock delta above 100 ms, or NTP/RTP clock-rate error above
0.5%. Per-camera signed and raw SR-PC values remain measured, displayed and persisted diagnostic
evidence; their absolute magnitude never gates success, revokes VERIFIED, triggers Auto or blocks a
timing result. The relative limit remains an explicit operator setting rather than a hidden clamp.
The approved qualification config uses a 7.0-second SR-age target, and its effective value is
recorded with the run evidence.

The correction owner removes only Left/Right differential error. It computes the median common mode
of the two diagnostic SR-PC series and uses
`next lead[side] = previous lead[side] - (side median - common median)`; common absolute offset does
not authorize mutation. It admits Left/Right SR identities atomically only after per-side freshness
and receive-close pair boundaries pass: three deduplicated `rtcp-pad-probe` SRs per side from the
replacement generation and after that side's STUS ACK. Feedback MAD is at most 20 ms. An isolated
span outlier is retained as diagnostic evidence and tolerated when the robust median/MAD remains
stable. HIWS RTT is bounded at 2.0 seconds, resulting target lead at ±2.0 seconds, and the
complete action at two rounds/90 seconds. Only a STUS mutation may wait up to 15 seconds for its
exact ACK, capped again by the remaining overall deadline; HIWS and generic SDK control keep their
configured timeout. A target safety-limit or second-round relative nonconvergence is terminal; no
value is silently clamped. During command, observer restart and SR warmup, only competing WIRIS
control-channel mutations are excluded. Measurement, YOLO and recording continue through their
existing owners.

The Frame layer separately accepts only time-dependent Left/Right pairs whose camera-derived frame
delta is at most 100 ms and whose logical clock epochs match. An occasional unmatched candidate or
outlier may be dropped and reported; Clock VERIFIED does not require every source frame to produce a
pair. Sustained field acceptance uses rolling pair-delta/coverage evidence rather than a zero-drop
contract.

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
observer replacement failure, missing/old/poll-detected/unstable SR evidence, target-limit breach,
relative nonconvergence, competing-control conflict, cancellation and overall timeout. ACK-only state is pending;
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

### Workstation STUS continuity amendment

Workstation clock scheduling is `manual` or `auto`; missing settings migrate to Manual and a
30-minute interval. Auto remains unseeded after every application start/new camera connection until
one explicit manual closed-loop action reaches clock VERIFIED. Its next deadline is based on
monotonic time from the last VERIFIED result. Drift never causes an early mutation: Manual sends
nothing, while Auto waits for its configured deadline. A failed scheduled action has no immediate
retry and may try again only at the next regular deadline.

Clock correction uses a dedicated bounded RTCP-only observation owner. It may recreate only its
own local RTSP session after a STUS round; it never stops or restarts CameraPane, YOLO, inference or
MP4 writers. Its post-ACK evidence is three unique stable pad-probe Sender Reports per side. Media
and decoded-frame truth never substitute for a failed observer and never become valid because the
clock observer passes.

Manual and scheduled STUS may run during Measurement/YOLO/recording. From the first side's command
send until both sides' final post-ACK clock evidence verifies, time-dependent stereo results are
BLOCKED. Image delivery, the current MP4 file and timing-independent results continue. Each command
round creates a logical clock epoch even if stream generation and SSRC do not change; cross-epoch
pairing is forbidden and every transition/result is append-only evidence.

The process-wide cooperative deadline cannot preempt a camera's 12-second stop window or its
nine-second GStreamer EOS wait; when a prior lifecycle step delays camera stop, the force boundary
moves with the camera request. Session-save and HR/OZ capture threads are explicit gates. Forced
cleanup persists a source-backed thread/process/socket/shared-memory report and returns pending
while any inspected owner survives. Per-side video finalization success or failure must be accepted
by the append writer before settings flush, and DataStorage closes last so no late camera outcome is
offered to an already-dead writer.

## Proposed health-aware distance timing fallback — not yet approved

The existing Measurement time-basis setting is retained and made truthful for live cameras:

- `camera_rtc_required`: NTP only; unverified/revoked Clock produces no time-dependent pair.
- `camera_rtcp` (`RTCP/NTP preferred`): require one manual VERIFIED seed, use NTP while current, and
  use app-receive monotonic after that Clock is revoked/unavailable.
- `detection` (`app receive`): explicit always-app behavior, retaining its existing scope.

Live-camera state must no longer silently override the selected `camera_rtcp` policy back to
`camera_rtc_required`. A pure timing-basis decision consumes public Clock status, scheduler seed and
logical epoch only. It emits the selected basis, reason and monotonically increasing basis
generation. Any basis/epoch transition flushes both detection buffers. Mixed basis generations,
`clock_epoch_state=transition`, or unequal nonempty clock epoch IDs are rejected regardless of the
selected time basis.

`app_receive_monotonic_ns` is captured from host `CLOCK_MONOTONIC` immediately after a decoded
appsink sample is pulled and before frame copy/YOLO. It is not camera exposure time. In fallback,
the coordinator selects the nearest Left/Right pair on this field and hard-rejects a selected delta
above 100 ms (or a stricter configured live-pair limit). Missing/outlier frames may be dropped. It
must not reuse NTP merely because an NTP value is present while Clock is revoked.

New authoritative pair rows are additive schema v4 and carry at least:

- `pairing_time_basis`, `pairing_time_basis_reason`, `pairing_time_basis_generation`;
- `selected_pair_delta_ns` and compatibility `sync_delta_ns` with the same selected-basis meaning;
- per-side `app_receive_monotonic_ns` plus signed `app_receive_pair_delta_ns`;
- per-side derived `rtcp_frame_ntp_unix_ns`, signed `rtcp_pair_delta_ns`, raw SR/RTP mapping fields,
  and `ntp_evidence_role=primary|diagnostic_untrusted`;
- Clock status/reason, manual-seed generation, logical clock epoch ID/state and fallback trigger.

Existing rows are never rewritten. Readers use `selected_pair_delta_ns` first and fall back to the
legacy RTCP field only when the new basis field is absent. Storage records both clocks but never
chooses the basis. After a new VERIFIED clock result, buffer flush plus a new basis generation
precedes all NTP pairs.

Recommended result contract is numerical distance with
`measurement_validity=ESTIMATED`, `quality=DEGRADED_TIMING` and an explicit app-arrival warning.
Whether the operator instead wants this fallback promoted to `VALID` remains unapproved.

## Superseding app-receive-authoritative live-distance contract — awaiting approval

The prior health-aware NTP/fallback contract above is superseded. Workstation live distance has one
fixed timing basis: `app_receive_monotonic`. Capture an exact integer
`app_receive_monotonic_ns = time.monotonic_ns()` immediately after decoded appsink sample receipt,
before frame copy, preview publication or YOLO. Retain the existing float seconds only as a
compatibility projection of that integer source.

The detection-pair coordinator selects nearest Left/Right observations only on app-receive
monotonic time. It hard-rejects missing/nonfinite/regressed values and absolute delta above 100 ms
(or a stricter configured live limit). Stream-generation rollback, correspondence failure, exact
native-frame reservation failure and stale-buffer limits remain. NTP presence/value/health, Clock
status, STUS operation and NTP logical clock epoch cannot change pair selection or validity. STUS
transitions therefore do not flush or pause app-timed distance buffers.

All image-based methods consume the same frozen app-timed pair. App receive is also the existing
host clock used for nearest attitude/GNSS association, so no clock-domain conversion is introduced.
When geometry and other method requirements pass, the result is normal `VALID`; Clock failure alone
adds no warning or degraded classification.

Authoritative new-row schema v4 fields are:

- `pairing_time_basis=app_receive_monotonic`, `pairing_time_basis_role=authoritative_distance`;
- per-side exact `app_receive_monotonic_ns`; signed Right-minus-Left
  `app_receive_pair_delta_ns`/`selected_pair_delta_ns`; `selected_pair_abs_delta_ns` for the gate;
  and compatibility `sync_delta_ns=selected_pair_delta_ns`;
- all available derived frame NTP ns, raw SR NTP/identity, RTP timestamp/SSRC/rate, signed
  `rtcp_pair_delta_ns`, SR-PC observations and `ntp_evidence_role=record_only`;
- Clock status/reason and logical NTP clock epoch ID/state as record-only evidence.

Storage never substitutes NTP into a missing app field. Existing schema-v3 rows remain byte-stable;
readers interpret their legacy fields by their original schema and do not retroactively call them
app receive. Live calibration and Legacy timing are outside this amendment.

## Proposed indoor experiment recovery contract — awaiting approval

- Activation: `SASL_WIRIS_NON_IDR_INTRA_EXPERIMENT=1` on one Workstation process only. Missing,
  false or malformed values preserve strict NAL5 behavior.
- Boundary: GStreamer `h264parse`, byte-stream, AU alignment. A candidate requires valid PTS,
  `DELTA_UNIT=false`, no `CORRUPTED`/unsafe `DISCONT`, NAL1 present and current SPS/PPS present in
  the normalized AU. A bare NAL1 header or parser flag alone is insufficient.
- State: startup and every RTP/SSRC/AU discontinuity latch encoded and decoded output. The exact
  candidate PTS enters the decoder; only that exact returned decoded PTS opens Preview/YOLO/native
  output. The next gap/error relatches immediately. Recovery evidence names
  `wiris_non_idr_intra_experimental`, never IDR.
- Recording: the H.264-copy branch may demonstrate bytes/EOS only. Final validation remains strict
  on first-VCL IDR, reference continuity, sample/decode counts and timing, and therefore cannot be
  promoted by the experiment flag.
- Resources: no extra decode/encode worker or queue. Existing encoded queues remain lossless and
  bounded by time; decoded preview remains latest-only. The external monitor polls at low rate and
  does not consume one-consumer product queues.
- NTRIP: the user-supplied SASL credential enters only the launched process environment/session.
  `remember_password=false`; logs/evidence store credential source and non-secret identity only.
- SDK/STUS: send no camera encoder GET/SET and no STUS mutation during the timed soak. The existing
  decoder-independent RTCP observer continues; Clock/NTP stays record-only for distance.
- Stop: both camera sides unavailable, GUI heartbeat failure, P1/P2 dual stall, NTRIP/RTCM required
  failure, storage failure, runaway resources or cleanup failure makes the candidate FAIL and ends
  it before 3,600 seconds.

## Mountain GNSS translation contract — approved 2026-08-28

The Mountain owner resolves P1-to-P2 translation only from the GNSS observation attached to the
same frozen image burst, or from an explicitly labelled finite NED override. Capture-bound samples
must each contain finite latitude/longitude/altitude, numeric `fix_type >= 3`, and produce a finite
non-zero world-NED baseline. There is no fallback to later live panel GPS.

Both-side `RTK_FIXED` no longer owns translation observability. Exact fix types, RTK labels,
accuracy/timestamp evidence and the minimum accepted fix are retained; any usable case without
both labels `RTK_FIXED` adds a candidate warning and therefore evaluates as `ESTIMATED`. A missing,
malformed or sub-3D fix leaves all three translation axes unobservable. Mountain rotation remains
independently gated by its existing robust correspondence solve.

The latest pre-run Mountain Draft is frozen at the new Measurement boundary and evaluated under
stable ID `mountain_gnss`. `pixhawk_rtk_live` remains the representative method. No setting,
storage schema version, migration, coordinate contract or historical-row reinterpretation is added.

## Mountain automatic-suggestion copy contract — approved 2026-08-29

The owner separates proposal quality from structural copy usability. A Mountain proposal is
copy-usable only when it belongs to the current frozen Left/Right frame IDs and hashes and, after
one shared exclusion-index set, yields equal ordered sides with at least four finite in-frame,
unique/distributed pairs. Confidence, quality state and detector reason remain evidence and never
replace structural validation.

`Use suggestion` revalidates identity, exclusions and point geometry in the controller, filters
both sides by the same indices, and records quality, original/retained counts, exclusions and a
canonical filtered-input hash. LOW_CONFIDENCE acceptance adds an explicit warning without
relabelling the detector result.

Horizon copy permission is also independent of detector quality. The owner revalidates current
Left/Right frame IDs, hashes and dimensions, parses exactly two finite in-frame endpoints per side,
and enforces the same 25%-native-width minimum separation used by the manual Horizon solver.
Accepted endpoints are normalized into the annotation schema and hashed; low-quality state remains
evidence and adds a warning. Missing/short/stale/malformed endpoints still fail closed.

`Solve GNSS + Mountain R,t` remains unchanged and consumes only explicit manual/accepted
annotations. Direct proposal consumption, auto-copy, auto-Solve and auto-Apply are not approved.
Existing exact K,D, prior, GNSS and robust Mountain solver gates remain unchanged. No setting,
worker, device operation or persistence schema version is introduced; fields are additive and old
records remain immutable.

## Approved Settings-preferred live-distance timing contract — 2026-08-31

One pure timing-policy owner reads the Measurement-bound normalized setting and emits requested
basis, effective basis, fallback reason and a monotonically increasing policy generation. The
default is `app_receive_monotonic`; `camera_rtcp` actually selects pairs only while public Clock is
VERIFIED and both frame mappings are finite, current, same-epoch and inside the configured pair
bound. App receive is the mandatory fallback for every unusable RTCP condition.

The detection coordinator and Measurement freeze boundary consume the same immutable policy
snapshot; neither may re-read widgets or infer a stricter basis merely because cameras are
connected. A Measurement freezes its requested basis at start. Apply/Save during that Measurement
changes only the next Measurement; an idle policy/effective-basis transition flushes both detection
buffers before another pair is admitted. Exact native-frame identity, correspondence, configured
pair bound, K,D, attitude, GNSS and geometry gates remain unchanged. PX4 attitude/GNSS association
stays on host monotonic time even when RTCP selects the stereo pair.

Every new row retains exact app and RTCP/NTP evidence and adds explicit requested/effective basis
plus fallback reason. Fallback provenance is not a geometry warning and does not lower otherwise
valid output below `VALID`. Existing schema-v3/v4 rows are not rewritten. The former strict
`camera_rtc_required` value normalizes to `camera_rtcp` preferred-with-fallback.
