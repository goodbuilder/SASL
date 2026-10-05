# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

This change makes the existing Workstation workflow truthful and durable under the real combined
camera/Pixhawk/NTRIP/recording load. It preserves camera/PX4 firmware and settings unless a later
explicit approval names one change. Existing evidence is preserved byte-for-byte.

## Operator workflow before and after

Before, individual paths could appear connected while media or telemetry had stopped, and old
capture/storage state could block new field work. After, each boundary qualifies independently,
failure is isolated, and a final 60-minute run starts only after short gates pass.

## Success, delay, rejection, failure, overload, and recovery

Every command needs an external postcondition: decoded frame, inbound MAVLink, HTTP/RTCM bytes,
durable file/hash or released resource. Busy work is queued only where already approved. Failure on
one side must not silently continue injection or claim readiness. A failed short gate ends that
candidate immediately; it does not consume the full 60 minutes.

## Data and compatibility effects

No existing Session, spool, calibration or evidence is migrated, rewritten or deleted. New records
retain exact FrameKey/hash/timestamp provenance and append-only behavior.

## Performance priorities and allowed degradation

GUI controls, Stop/Error and evidence integrity outrank preview freshness. Preview may coalesce to
latest-only within bounded queues. Silent corruption, unbounded memory, timestamp-nearest evidence
substitution and unlimited inference are not allowed. Auto CPU fallback remains visible and safe.

## Visual and interaction changes, including dynamic-layout stability

Reuse existing Camera, GNSS, Data and Diagnostics status surfaces. No new global dashboard or
decorative UI is approved. Dynamic states must not resize controls, steal focus, reset selection or
jump scroll.

## How acceptance will be demonstrated

Focused tests, full regressions, real entry points, problem-specific 10-15 minute qualifications,
three 10-minute cold boots, failure injection and a final 60-minute full-load soak. Evidence includes
pcap/RTCP, MAVLink ages/rates, NTRIP HTTP/RTCM, MP4 strict decode, queues/RSS/GPU, and FrameKey/hash.

## Cost, risks, assumptions, rollback, and field limits

Camera RTP omission/real-IDR absence or PX4-side UDP deadlock may require vendor/firmware work. In
that case app mitigation and the external blocker remain distinct and the packet cannot be closed.
Rollback is per smallest implementation slice and must not restore silent success.

## Exact scope awaiting approval

Approved by the user on 2026-08-26: the complete integrated field-stability goal described in this
packet, using problem-specific early-exit gates and one final 60-minute soak. Excluded are app-to-app
rig ownership, Diagnostics log filtering, unapproved device configuration/firmware changes, and
coordinate/calibration algorithm changes.

## 2026-08-27 approved STUS continuity amendment

The user approved a Workstation-only clock workflow amendment. Settings gains persistent Manual
and Auto modes; Auto starts only after one successful manual STUS in each application run and then
uses the configured interval (default 30 minutes). Manual still detects and records drift but sends
no automatic command. Scheduled STUS may run while Measurement, YOLO and recording are active.

STUS must not restart or stop the existing RTSP, YOLO or MP4 workers. A separate RTCP-only observer
owns clock correction and continuous verification, while Video/Frame timing remain independent
states. Every STUS creates a logical clock epoch; timing-dependent results are blocked across the
transition, but imagery, MP4 and timing-independent results continue. Existing firmware, encoder,
camera reboot, Legacy behavior and coordinate/calibration algorithms remain excluded.

The user further clarified that the absolute per-camera `SR-PC` value is observation, display and
evidence only. It never gates STUS success, revokes VERIFIED, triggers Auto or blocks a result. The
quality objective is sustained Left/Right relative timing at or below 100 ms. The Frame layer may
drop an occasional unmatched/outlier frame; there is no every-frame/no-drop acceptance condition.
Clock observation remains parallel, while external STUS mutation remains ordered Left exact-OK
then Right.

## Proposed app-receive distance fallback amendment — awaiting approval

The newly confirmed 16.6 ppm Left/Right clock-rate difference means a good STUS phase can drift by
about 30 ms in 30 minutes and approximately 100 ms in 100 minutes. The proposed Workstation-only
fallback preserves NTP as the preferred pairing clock while it is VERIFIED. If a previously
verified clock is later revoked by sustained relative drift or its current evidence becomes
unavailable, distance pairing changes to the exact host-monotonic time captured when each decoded
frame reaches its camera process. Active STUS/clock-epoch transitions still drop candidates rather
than pairing across the discontinuity.

Fallback pairs must be within the same 100 ms limit on app-receive time. Occasional unmatched
frames may be dropped. NTP/SR/RTP timestamps, NTP pair delta, clock state, fallback reason and
logical epochs remain stored as diagnostic evidence but do not select the pair while fallback is
active. A recovered, freshly VERIFIED clock flushes old candidates and returns future pairs to NTP.
No camera setting, STUS command, distance geometry, coordinate conversion, calibration, Legacy
behavior or existing stored row is changed.

Recommended visible result: continue displaying and saving a numerical distance, but label it
`ESTIMATED / APP RECEIVE TIMING` rather than `VALID`, because per-side RTSP/network/decode latency
can differ and app arrival is not a camera exposure timestamp. This validity classification is the
one remaining operator decision; source implementation is not authorized until it is approved.

Status update: this fallback amendment is **deferred** while STUS cadence/repeatability is evaluated.
If resumed, the user has decided that app-receive monotonic time is the authoritative distance
pairing truth and fallback distance is `VALID`, while its stored provenance remains explicitly app
receive rather than exposure time. No fallback source implementation is currently authorized.

Shortening Auto cadence is not yet approved as an operating change. The confirmed oscillator rate
adds only about 5 ms per five minutes, whereas the latest exact-OK STUS introduced seconds of phase
error. Repeated Auto remains disabled until single-cycle convergence is repaired and repeated manual
gates pass.

## 2026-08-27 approved app-receive-authoritative distance amendment

This section supersedes the earlier NTP-primary/fallback/degraded proposal. For Workstation live
distance measurement, the only authoritative Left/Right pairing time is the host monotonic timestamp
captured when each decoded frame reaches its camera process. NTP is never used to select a distance
pair, validate its delta, classify its distance, or block a method. A pair at or below 100 ms on app
receive time may produce a normal `VALID` distance when its other geometry/input requirements pass.
An over-limit or missing app timestamp drops that pair; occasional drops are acceptable.

STUS, oscillator drift, NTP loss, SR staleness and logical NTP clock-epoch transitions remain visible
Clock evidence but do not interrupt app-timed distance pairing. Manual/Auto STUS may continue for
clock-quality and stored-time purposes without owning distance validity. This does not alter video,
YOLO, MP4, geometry, calibration, attitude/GNSS association, coordinates, Legacy or device settings.

Every selected pair still stores exact app-receive monotonic values plus all available NTP/SR/RTP
mapping, NTP pair delta, Clock state/reason and clock epoch as `record_only` evidence. New rows make
`sync_delta_ns` and `selected_pair_delta_ns` mean the app-receive delta; separately named NTP fields
cannot overwrite that meaning. Existing rows remain untouched and readable.

The Measurement UI shows `basis=APP RECEIVE` and app delta as the authoritative value, with NTP
delta labelled `record-only`. It does not show degraded timing solely because Clock/STUS is failed.
The stored provenance continues to say `app_receive_monotonic`; the application does not rename it
camera exposure time. The user explicitly approved this exact amendment at
`2026-08-27T14:31:06+09:00`.

Implementation status: the Workstation source, schema-v4 persistence/readers, fixed Measurement UI,
tests and affected product documentation now implement the approved contract. Focused amendment
regression passes 244 tests and the complete Workstation suite passes 1515 tests with 1 skip.
Target-hardware decoded-pair verification remains NOT_RUN because the existing FW 2.18.72 streams
emit zero NAL5; it is not promoted from software PASS to field PASS.

## Proposed indoor 60-minute NAL1 experiment — approval required

The new goal requests one real indoor 60-minute integrated run. Running the current default would
fail before the timer because both cameras emit no NAL5. The proposed change therefore adds a
runtime-only opt-in experiment mode; normal Workstation launches continue requiring a real IDR.

In experiment mode, a side may resume decoded output only at a complete NAL1 intra access unit that
arrives with current SPS/PPS, a valid PTS and no RTP/AU corruption or discontinuity. A later gap or
error immediately blocks that side again. The existing bounded queues, per-side isolation,
app-receive timing and GUI geometry remain unchanged. Status text explicitly says
`EXPERIMENTAL NON-IDR INTRA`; it never says IDR/keyframe.

Preview, YOLO, native evidence and app-timed distance may be evaluated during the experiment.
Direct-copy H.264 recording is still expected to fail portable integrity because it has no NAL5;
file growth/EOS are measured but cannot become MP4 PASS. No host re-encode is added. Camera/PX4
settings and firmware are untouched, and no camera SDK GET/SET loop runs during the soak.

The run starts only after a five-minute gate proves both experimental camera sides current,
P1/P2 `QUALIFYING` with fresh expected-sysid heartbeat/ATTITUDE and no stall, NTRIP HTTP 200 with
CRC-valid RTCM and dual failure-zero injection, recording growth,
responsive GUI and sufficient storage. The NTRIP credential supplied by the user is process-local,
not logged, persisted or saved to keyring. STUS mutation is excluded because its repeated manual
convergence gate remains failed; decoder-independent Clock/SR evidence is observed continuously and
remains record-only for distance.

Because MAVLink `LIVE` now requires 60 continuous seconds, a preflight that has run at least one
minute must prove both sides `LIVE`. The 10-minute checkpoint remains independent sustained-link
evidence rather than the product readiness boundary.

During the 60 minutes an external low-rate monitor writes twelve five-minute checkpoints for
process/child/socket state, frame/pair/RTCM/MAVLink counters and ages, recording growth, RSS/CPU/GPU,
disk and UI heartbeat. Required service loss, GUI nonresponse, unbounded resources, storage failure
or loss of both camera sides ends the candidate early and triggers cleanup. Completion produces a
problem-by-problem PASS/FAIL/NOT_RUN report; it does not manufacture a clean-camera, STUS or MP4
PASS from an experimental workaround.

Approval requested: authorize this opt-in experimental source/UI-status slice, its tests, the real
five-minute preflight and 60-minute run, and the corresponding Workstation `./README.md` update.

## 2026-08-27 approved outdoor execution supersession

The user instructed Codex to proceed with the outdoor experiment at `2026-08-27T19:40:48+09:00`.
This approves the source/UI-status slice, tests, Workstation README update, five-minute preflight and
60-minute run above, while replacing only the proposed indoor location with an outdoor location.
All safety, truth and rollback limits remain unchanged.

Outdoor evidence additionally requires actual P1/P2 rover fix type and accuracy to be recorded;
both sides reaching and sustaining RTK Fixed is a field gate rather than the former indoor
`NO_FIX` limitation. A failed Fixed gate is reported independently and does not permit the monitor
to invent a GNSS PASS. STUS mutation remains excluded; Clock/SR observation remains included.

## 2026-08-27 approved manual-only PX4 calibration PARAM readback amendment

The user approved removing the automatic one-shot calibration PARAM readback that ran after each
P1/P2 side first reached `LIVE`. Normal operation therefore sends no calibration PARAM requests
merely because ten-minute qualification completed and does not reset qualification for an
unrequested diagnostic snapshot.

The existing `Refresh P1/P2 parameters`, advanced MAG/attitude refresh controls, and explicit
readback after reset/apply/save remain available and unchanged. Until the operator requests one,
the current Pixhawk Parameters surface continues to show `PARAM readback 대기 중`; during an
explicit request it still shows `PARAM SYNC` and uses the existing bounded per-side reservation.

This amendment adds no control, setting, schema, device write, worker, or visual layout. Live
ATTITUDE/GNSS, RTCM routing, Measurement admission/validity, recording, shutdown, and the healthy
peer remain unchanged. Historical automatic-readback pcaps and evidence remain valid historical
artifacts. Approval was recorded at `2026-08-27T21:57:08+09:00`.

## 2026-08-27 approved 60-second operational MAVLink qualification amendment

The operator approved reducing the product `QUALIFYING`/`RECOVERING` interval from ten minutes to
60 continuous seconds. A side becomes `LIVE` only while its expected-system HEARTBEAT and ATTITUDE
remain fresh for the whole interval. The existing ATTITUDE-stale and all-RX-stall thresholds still
block dependent attitude measurement and RTCM/commands immediately; a reconnect, confirmed remote
reboot, resumed stall, error, or explicit/manual PARAM command starts a new 60-second interval.

Automatic calibration PARAM readback has already been removed, so ordinary first `LIVE` no longer
causes a hidden command or a second qualification interval. Existing manual P1/P2 parameter refresh,
advanced MAG/attitude refresh, and explicit post-mutation readbacks retain their visible `PARAM SYNC`
and post-command requalification behavior.

This is an operational measurement-admission/readiness gate, not a claim of long-duration field
stability. Ten-minute comparison runs, cold-boot repetitions and the final 60-minute integrated soak
remain separate evidence requirements. Existing historical ten-minute artifacts are not rewritten.
The UI keeps its current geometry and dynamically shows `mm:ss / 01:00`; no setting, control,
schema, migration, coordinate, attitude-frame, or device-configuration change is added. Approval
was recorded after the user confirmed the automatic PARAM read had been removed.

## 2026-08-28 approved Mountain GNSS 3D-fix amendment

The operator requested removing the requirement that both P1 and P2 be `RTK_FIXED` before a
Mountain candidate's GNSS translation can participate in Measurement. The recommended replacement
is a capture-bound usable-baseline gate: both sides have `fix_type >= 3`, finite latitude/longitude/
altitude and a finite non-zero P1-to-P2 NED baseline from the same frozen burst. `RTK_FIXED`,
accuracy, age and other GNSS readiness evidence remain stored and visible as warnings rather than
translation-observability blockers.

`Solve GNSS + Mountain R,t` continues to require paired distributed image points and the existing
robust rotation gates. It creates a Draft; a new Measurement automatically freezes the latest
Mountain Draft before Detection starts. Each accepted target pair can then produce a
`mountain_gnss` outcome in `method_evaluations.jsonl`, including the frozen candidate/R,t identity,
target pair, 3-D point/range, validity and warnings. Global `Apply candidate` is required only to
promote that Draft to Active Alignment, not to retain Mountain comparison outcomes.

The fixed representative `measurements.jsonl` result remains `pixhawk_rtk_live`; making Mountain
the primary result is excluded. No existing record is rewritten. Non-fixed usable GNSS produces an
`ESTIMATED` Mountain outcome with explicit quality/lever-arm warnings, never a silent `VALID`
promotion. The user approved this replacement gate with “3D fix 이상이면 가능하게 해” at
`2026-08-28T00:17:37+09:00`.

## Proposed Mountain automatic-suggestion direct-use amendment — approval required

Today `Auto-select Mountain` draws a proposal only. `Use suggestion` is enabled only for `GOOD`,
and the operator must press it before `Solve GNSS + Mountain R,t`; otherwise Solve sees no Mountain
annotations. This makes a numerical confidence threshold a permission gate even though the robust
Mountain solver already has independent correspondence, inlier, rank, condition and residual gates.

The recommended change keeps `Auto-select Mountain` and `Solve GNSS + Mountain R,t` as separate
explicit actions but removes the mandatory middle click. When no manual Mountain annotations exist,
Solve may directly consume the latest current-frame Mountain proposal if, after operator exclusions,
it still has equal Left/Right point counts, at least four finite in-frame pairs, unique/distributed
points and matching frozen frame IDs/hashes. `GOOD` remains a quality label. A structurally usable
`LOW_CONFIDENCE` proposal may be attempted, but its confidence/reason/provenance is retained as an
explicit warning and the existing robust solver still decides rotation observability.

`Use suggestion` remains as an optional copy-for-editing action and becomes available for a
structurally usable `LOW_CONFIDENCE` Mountain proposal. Complete manual or already accepted points
always take precedence; a partial manual annotation fails visibly rather than being silently
replaced or mixed. `NO_DATA`, fewer than four remaining pairs, duplicate/near-collinear points,
stale frame identity or malformed input remains blocked. The currently advertised paired
right-click exclusion must also be enforced by the backend direct/copy paths, not only hidden in
the overlay.

This does not auto-run Solve, auto-Apply a candidate, or change Measurement start/freeze behavior.
The both-side `fix_type >= 3`, finite non-zero frozen GNSS baseline, exact operating K,D, prior,
lever-arm warning and final Mountain rotation gates remain. `pixhawk_rtk_live` remains
representative. No new control, layout, setting, worker, device operation, schema migration or old
row rewrite is proposed. Existing Manual Mountain and explicit `Use suggestion` remain fallbacks.
The lasting operator instructions would be updated in the Workstation `./README.md` and current UI
workflow documents.

Approval requested: authorize this recommended direct-Solve fallback and optional low-confidence
copy behavior, including exact exclusion/provenance persistence, tests and Workstation `./README.md`
update. Immediate automatic copying on Auto-select completion is deliberately excluded because it
would remove the review interval and could overwrite operator annotations.

## 2026-08-29 approved Mountain Use suggestion quality-separation subset

The user approved the narrower optional-copy portion and confirmed
`apps/Workstation/README.md` as the lasting README target. Mountain `Use suggestion` may accept a
structurally usable `GOOD` or `LOW_CONFIDENCE` proposal; confidence/state remain visible and
persisted quality evidence rather than permission gates for this explicit copy action.

Current frozen-frame identity, paired exclusions, equal Left/Right counts, at least four finite
in-frame unique/distributed pairs and owner-side validation remain hard. Horizon remains
`GOOD`-only. Direct suggestion consumption by Solve, automatic copy, automatic Solve and automatic
Apply remain unapproved and are excluded from this slice.

## 2026-08-29 approved Horizon Use suggestion quality separation

The user approved the same quality-versus-copy separation for Horizon. `GOOD`,
`LOW_CONFIDENCE`, `NO_DATA`, confidence and detector reasons remain truthful review evidence but
do not independently disable `Use suggestion`. In normal output, a `NO_DATA` result has no
endpoints and therefore still fails structural validation.

The controller must independently require current frozen Left/Right identities and native
dimensions, exactly two finite in-frame endpoints per side, and at least 25% image-width endpoint
separation. Horizon Solve and Apply remain separate explicit actions with their existing exact K,D
and both-side observability gates. No automatic action, layout, worker, device or migration change
is approved.

## 2026-08-31 approved Settings-preferred live-distance timing amendment

On 2026-08-31 the user requested that live-distance timing follow the persisted Measurement
Settings selection instead of remaining a fixed read-only policy. New and migrated defaults remain
`App receive`. If the selected timing source becomes unusable, live distance must continue on exact
decoded-frame app-receive monotonic time rather than becoming blocked.

The safe recommendation is a preference-plus-fallback contract: `App receive` always uses app
receipt; `RTCP/NTP preferred` uses RTCP/NTP only while its current pair/Clock evidence is usable and
otherwise changes the effective basis to app receive. The UI and saved evidence must show requested
basis, effective basis and fallback reason separately. App fallback remains normal `VALID` timing
when the existing app-delta and geometry gates pass; it is never renamed camera exposure time.

The user approved operational RTCP selection: when `RTCP/NTP preferred` is selected, Clock is
VERIFIED, both frame mappings are current in the same epoch, timestamps are finite and the selected
delta is inside the configured bound, RTCP/NTP actually selects the distance pair. Any missing,
malformed, unverified, stale, transitioning, epoch-mismatched or over-limit RTCP timing instead
selects an exact app-receive pair inside the same bound. This fallback is timing provenance, not a
geometry warning, and does not by itself reduce an otherwise-valid distance below `VALID`.

The setting default is `App receive`. A running Measurement freezes its requested preference;
Apply/Save during that run is visible as pending and starts with the next Measurement. Outside a
Measurement the setting may take effect after a bounded pair-buffer generation flush. Historical
`camera_rtc_required` settings migrate to `RTCP/NTP preferred` because the approved mandatory app
fallback is incompatible with strict fail-closed behavior. Existing rows are immutable and new
rows separately retain requested/effective basis, fallback reason and app/NTP deltas.

No camera/Pixhawk command, clock mutation, geometry/coordinate change, worker or unbounded queue is
added. The user approved the complete plan and instructed implementation on 2026-08-31; source
implementation and the confirmed `apps/Workstation/README.md` update are authorized.
