---
schema: modify-gui-change-state-v1
change_id: 20260826-integrated-field-stability
status: interviewing
mode: backend-coupled
updated: 2026-09-01
baseline_ref: 1198689fafed1ef392d395e11437352431312ac8
last_completed: "Recorded the Mountain lever-arm UNMODELED removal request, verified the existing zero-vector fallback, and separated assumed-zero provenance from physical offset calibration"
next_question: "Should Mountain formalize a zero/equal lever-arm assumption and remove the UNMODELED warning without changing R,t, or add measured per-rig offsets that physically correct the baseline?"
read_next:
  - docs/changes/20260826-integrated-field-stability/approval-summary.md
  - docs/changes/20260826-integrated-field-stability/implementation-plan.md
  - docs/changes/20260826-integrated-field-stability/verification.md
  - docs/changes/20260826-integrated-field-stability/current-behavior.md
  - docs/changes/20260826-integrated-field-stability/backend-contract.md
  - docs/changes/20260826-integrated-field-stability/frontend-contract.md
  - docs/changes/20260826-integrated-field-stability/workflow-impact.md
  - docs/changes/20260826-integrated-field-stability/impact-analysis.md
  - docs/changes/20260826-integrated-field-stability/integration-flow.md
  - docs/changes/20260826-integrated-field-stability/ui-quality-and-stability.md
  - docs/changes/20260826-integrated-field-stability/decisions.md
affected_product_docs:
  - README.md
  - WIRIS.md
  - workflow.md
  - docs/RECORDING_SCHEMA.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/07_COMMAND_MODEL.md
  - docs/ui/09_IMPLEMENTATION_PLAN.md
  - ../StereoWIRIS/UPDATE.md
  - ../StereoWIRIS/docs/usage.md
  - ../../README.md
blocking_questions: 1
source_changed: true
implementation_allowed: false
approval:
  state: approved
  scope: "Integrated field-stability goal plus the 2026-08-27 Workstation STUS continuity amendment"
  conditions: []
  recorded_at: "2026-08-27T10:26:32+09:00"
proposed_amendment:
  state: approved
  scope: "Workstation live distance always pairs on app_receive_monotonic with a 100 ms bound; NTP/STUS/clock epochs are record-only and cannot block or degrade distance"
  decided_validity: "app_receive_monotonic is authoritative for live distance and result is VALID; provenance must not claim camera exposure time"
  blocking_decision: null
  recorded_at: "2026-08-27T14:31:06+09:00"
timing_settings_amendment:
  state: approved
  scope: "Measurement Settings selects the preferred live-distance timing basis; default and unusable-source fallback are app_receive_monotonic"
  preserved: "Exact app timestamps and 100 ms gate, RTCP/NTP/STUS evidence, geometry, coordinates, existing rows and bounded owners"
  healthy_preference: "camera_rtcp selects distance pairs only with VERIFIED/current/same-epoch finite evidence inside the configured pair bound"
  fallback: "all unusable RTCP timing states fall back to exact app_receive_monotonic without degrading otherwise-valid distance"
  apply_boundary: "a running Measurement keeps its frozen request; saved changes apply from the next Measurement"
  legacy_mapping: "camera_rtc_required migrates to camera_rtcp preferred-with-fallback"
  readme_target: "apps/Workstation/README.md"
  blocking_decision: null
  recorded_at: "2026-08-31T01:52:15+09:00"
field_experiment_amendment:
  state: approved
  scope: "Default-off process-local NAL1 intra experimental recovery plus one real five-minute preflight and 60-minute outdoor integrated soak; strict IDR default and MP4 integrity remain unchanged"
  ntrip_credential: "process-local SASL/NGII session only; never log, persist, or save to keyring"
  stus_scope: "decoder-independent RTCP Clock observation only; no STUS mutation"
  readme_target: "README.md"
  blocking_decision: null
  recorded_at: "2026-08-27T19:40:48+09:00"
px4_param_readback_amendment:
  state: completed
  scope: "Remove automatic post-LIVE calibration PARAM readback; preserve all explicit/manual and post-mutation readbacks"
  ui: "No new control; existing waiting/PARAM SYNC/result/error surfaces remain"
  migration: "none"
  verification: "183 focused passed; 1523 passed, 1 skipped full suite; launcher/delegate/compile/diff checks pass"
  runtime_note: "The pre-change baseline process exited after its 600-second monitor; the next Workstation launch uses the manual-only source"
  blocking_decision: null
  recorded_at: "2026-08-27T21:57:08+09:00"
mavlink_qualification_amendment:
  state: completed
  scope: "60-second operational HB+ATT qualification; stale/stall fail-close unchanged; long 10/60-minute stability tests remain separate"
  param_contract: "automatic readback absent; explicit/manual commands retain post-command requalification"
  ui: "existing dynamic mm:ss / required duration surface, now 01:00; no geometry change"
  migration: "none"
  verification: "124 focused; 192 expanded; 1524 passed, 1 skipped full suite; launcher/monitor/delegate/portability/compile/diff pass"
  hardware: "NOT_RUN after operator disconnected all wired devices; NIC-absent launch exited cleanly"
  blocking_decision: null
  recorded_at: "2026-08-27T22:20:00+09:00"
mountain_gnss_nonfixed_amendment:
  state: completed
  scope: "Remove both-RTK_FIXED translation observability requirement for Mountain only"
  recommended_gate: "same frozen burst; both fix_type >= 3; finite lat/lon/alt; finite non-zero P1-to-P2 NED baseline"
  quality: "non-fixed usable GNSS produces ESTIMATED with explicit warnings; existing rotation gates remain hard"
  measurement: "next Measurement freezes latest Mountain Draft and writes mountain_gnss method outcomes; pixhawk_rtk_live remains representative"
  migration: "none; existing rows immutable"
  verification: "9 focused; 132 expanded; 1529 passed, 1 skipped full suite; launcher/delegate/compile/diff checks pass"
  blocking_decision: null
  recorded_at: "2026-08-28T00:17:37+09:00"
mountain_auto_suggestion_amendment:
  state: completed
  scope: "Approved subset: Mountain-only optional low-confidence copy/edit; direct suggestion consumption on Solve remains unapproved"
  recommended_workflow: "Auto-select -> review/exclude -> Use suggestion -> explicit Solve; no direct/automatic Solve or Apply"
  quality: "GOOD remains quality; structurally usable LOW_CONFIDENCE may be attempted with explicit warning"
  hard_gates: "copy: current frozen identity and paired >=4 finite in-frame unique/distributed points; Solve keeps exact K,D, fix_type>=3 baseline and robust rotation"
  precedence: "Solve consumes only manual/accepted annotations; direct proposal fallback is absent"
  persistence: "additive source/confidence/reason/exclusions/filtered-hash provenance; no migration"
  readme_target: "apps/Workstation/README.md"
  verification: "76 focused; 139 expanded; 1564 passed, 2 skipped full suite; launcher/delegate/compile/diff/UI pass"
  direct_solve: "not approved and not implemented"
  blocking_decision: null
  recorded_at: "2026-08-29T12:29:27+09:00"
horizon_auto_suggestion_amendment:
  state: completed
  scope: "Horizon Use suggestion ignores detector quality state/confidence; quality display and evidence remain"
  workflow: "Auto-select -> review -> Use suggestion -> explicit Solve -> explicit Apply"
  hard_gates: "current frozen identity/dimensions; exactly two finite in-frame endpoints per side; >=25% native-width separation"
  preserved: "detector calculation; manual Horizon; exact K,D; both-side observability; no automatic copy/Solve/Apply"
  readme_target: "apps/Workstation/README.md"
  verification: "82 focused; 145 expanded; 1583 passed, 2 skipped full suite; 771x143 visual geometry; delegate/compile/diff/launcher pass"
  runtime_note: "run.sh --check exited 0 with unrelated optional nvh264dec-unavailable warning and Auto avdec_h264 selection"
  blocking_decision: null
  recorded_at: "2026-08-29T14:33:33+09:00"
mountain_lever_arm_amendment:
  state: interviewing
  scope: "Remove Mountain lever-arm-driven UNMODELED while preserving truthful coordinate and provenance contracts"
  current_numeric_behavior: "missing correction applies [0,0,0] Left-camera-RDF metres"
  option_assumed_zero: "record an explicit assumed-zero/equal-offset provenance state and remove the UNMODELED warning; R,t unchanged"
  option_measured: "configure measured per-rig antenna-to-optical-center offsets and apply a frame-aware baseline correction"
  excluded: "silent warning deletion, historical-row rewrite, non-Mountain geometry changes"
  readme_target: null
  blocking_decision: "assumed-zero provenance or measured physical calibration"
  recorded_at: null
---

# Resume Notes

The user explicitly started the previously designed goal. Preserve all existing dirty changes and
prior packet decisions. Short independent probes precede any 60-minute soak. The user approved
camera SDK setting changes and one sequential reboot per camera. Both reboots and active-stream
`SEIR VISIBLE 1` tests completed; both cameras were restored to `GETH=TRUE`, `0/1/30/1`, but neither
emitted NAL5. No firmware install, destructive recovery or alternate re-encode path is approved.
Continue merging fresh field evidence before selecting each next slice.
The integrated 60-minute soak is Workstation-only; StereoWIRIS Legacy has separate focused blank
restart/new-session/MP4 gates and never owns hardware concurrently with Workstation.
Dual MAVLink QUALIFYING and real NGII GGA/RTCM_ACTIVE previously passed with dual-route failure 0;
that field evidence remains valid and is not replaced by the later STUS automated-fake result.
STUS ordered-ACK, decoder-independent observer and continuous-health software tests pass. The
30-minute holdover confirms a 16.6 ppm relative oscillator rate, while repaired repeated manual
convergence and Auto stress gates remain open. The Workstation process was restarted from the latest
worktree after the 1515-pass regression and is currently idle with no device sockets.
Combined Preview/NTRIP later caused real P2 wire silence and then dual wire silence. Physical power
recovery restored both sides on the existing sockets. The post-power link reached 10 minutes, but a
newly identified host-state defect left PX4 at about 51 pps instead of the requested 140 pps because
the stream profile was not rearmed after the remote reboot. That historical defect is superseded by
the later profile-rearm and isolated-topology application/NTRIP qualifications below.
The former low-rate NTRIP-only branch passed for more than ten minutes. The replacement isolated
topology now also passes a definitive Stop/reconnect run: 636.507 seconds, one HTTP 200, 120 valid
GGA, 2,954 CRC-valid station/MSM RTCM frames and 3,026 failure-zero injections per side while P1/P2
remain LIVE. RTK Fixed remains open because the indoor rover telemetry is NO_FIX.
The first latest-app Connect attempt then froze before completion: wire traffic stayed valid but
Recv-Q filled and the GUI thread stopped in synchronous QStyle polish while a saturated Orca/AT-SPI
service amplified the path. The app was safely terminated. The fix queues Qt callbacks, removes
status-path topology repolish, owner-starts the receive timer, coalesces action refresh and throttles
successful RTCM progress. Focused/scoped tests pass. A subsequent 69.63-second real Connect stayed
responsive, drained both sockets and sent the exact 15-command Stock profile once per side.
The following telemetry-only run reached dual LIVE, then exposed a 27-second automatic PARAM defect:
the app queried 18 nonexistent `CAL_MAG4_*` names, starved GCS heartbeat while the command held its
side lock, and replayed old HEARTBEAT packets as 28/27 redundant ATTITUDE recovery commands. Stock
refresh is now restricted to four firmware slots/124 parameters, the command receive owner services
heartbeat under the same re-entrant lock, the GUI never waits for that lock, and historical replay
cannot execute live stream actions. On the first isolated adapter, the real post-fix refresh sent and
received exactly 124 parameters per side in 0.227/0.250 seconds while host heartbeat gaps stayed
below 1.045 seconds. Camera/Pixhawk NIC isolation passed a 10-minute app Preview gate on that adapter.
The replacement RTL8156B then passed both a repeated direct 45-second camera-load boundary and an
exact latest-app ten-minute gate. The latter captured 103,610/102,864 messages, exact 15-command
profiles, 124/124 PARAM replies, no heartbeat starvation and zero capture/NIC/app errors.
The ordered real STUS then exposed a separate CRITICAL quality failure: every successful 0.51–0.93
second HIWS probe was discarded by a 250 ms cap, yielding effectively zero initial lead and signed
pad-probe SR-PC medians of -736.135 ms Left/-1,307.044 ms Right. The latest source now owns one
manual two-round/90-second closed loop using three atomic, unique, stable post-ACK replacement-
generation pad-probe SRs per side and `next lead = previous lead - signed SR-PC median`, bounded at
±2 seconds. Both rounds preflight both sides and preserve Left exact-OK before Right; all recording,
Measurement, competing control, shutdown, timeout, stale-evidence and late-callback paths fail
closed. The focused CRITICAL selector passes 24 tests and the complete controller pair passes 95.
The first root-run retry proved the new high-RTT preflight but exposed an independent response bound:
Left applied STUS and returned exact OK after 8.876 seconds, while the host had closed at the generic
5.005-second timeout; Right remained untouched. The latest source gives only STUS mutation responses
a 15-second ceiling, still capped by the 90-second operation deadline, and preserves generic/HIWS
timeouts. A 15-second hardware A/B then returned exact Left OK in 3.691 seconds followed by exact
Right OK in 3.433 seconds. Independent post-seed SR-PC medians were -313.712/-808.643 ms, leaving
about 494.931 ms relative offset. Both replacement camera generations emitted zero NAL5 IDR, so the
current-frame gate correctly blocked round two and R3 remains CRITICAL FIELD FAIL.

The camera worker now requires parsed NAL5 plus its exact decoded PTS at startup and after every RTP
gap. Zero-IDR sides become unavailable once without reconnect storms. Both real FW 2.18.72 cameras
accepted operator-approved `SEIR VISIBLE 1` and read back `0/1/30/1`, but fresh 15-second streams
still produced zero NAL5 before reboot. Real prior MP4s also fail the new asynchronous validator
(Left 1666/1565, Right 1503/1245, zero IDR, reference gaps and zero final `stts`); a valid 60-minute
108,000-sample fixture passes. Operator-approved sequential camera reboots then proved `GEIR=1` is
not persistent and does not produce NAL5 even when reapplied during active RTSP. Right's reboot
defaults were restored, and both sides finish healthy at `GETH=TRUE`, `0/1/30/1`. Two consecutive
real failed Measurement starts now normalize
`VFR_HUD.airspeed=NaN` to null with exact provenance, create distinct run IDs, rollback cleanly and
exit with no timer-affinity warning. An active-Measurement `SIGKILL` now leaves no surviving camera
child, established socket or native ring; a strict same-UID/dead-PID startup fallback reclaimed a
controlled tracker-less ring, did not adopt the incomplete run, and admitted a distinct new run.
The independent native-evidence auditor then verified 88 real pairs/176 PNGs and exposed one durable
projection omission: descriptor input `timestamp_s` was absent from Measurement snapshots. New rows
retain it; the historical real run reconstructs it only from the ACK-bound artifact row and passes
all byte/PNG/FrameKey/descriptor/ACK checks. The same real run proves method readiness is not a
global gate: 44 pairs entered while every one of 352 method outcomes remained BLOCKED.
Legacy default camera addressing was corrected from the Pixhawk NIC's `.10/24` to isolated
`.11/32`. A full automatic-path pcap contained zero SDK packets. Blank Legacy record start previously
armed an empty session; it now starts both camera recorders or rolls back visibly. The latest dual
MP4s are created/finalized but remain unusable because the camera still emits zero IDR.
The latest complete Workstation suite is 1515 passed, 1 skipped.
Repository portability now passes with zero findings. A same-host fresh checkout plus new canonical
venv installed the runtime manifest and passed deployment, execution, Workstation and Legacy checks
under an isolated HOME. The recreation exposed and fixed an unconstrained setuptools 84/colcon
conflict by owning `>=77,<80`; fresh resolution selected 79.0.1. R10 now also passes in a separate
official Ubuntu 22.04.5 KVM guest using exact pushed commit `132c185`: all 2,679 archived files
matched, package closure and both launchers passed twice, including isolated HOME. That clean guest
exposed and closed missing `python3-cairo` ownership and the previously untracked/undocumented
Legacy model provisioning contract.

The current STUS amendment removes absolute SR-PC from success/revocation/result gates and changes
the default relative limit to 100 ms. Full Workstation regression reached 1489 passed, 1 skipped
before the latest adaptive parallel HIWS slice; that subsequent slice passes 52 focused tests and
still needs a fresh complete rerun. Two real attempts failed the old 2-of-3/2-second HIWS boundary.
Parallel up-to-five sampling with two exact OKs and a 3.9-second target-safe RTT ceiling then passed
real preflight and returned exact Left/Right STUS OK, but unequal inferred apply delay expanded
relative phase from about 140 ms to about 3.11 seconds. The post-ACK verifier then exposed two
software defects: requiring all three 4–6-second-period SRs to be younger than seven seconds is
impossible, and one Left observer session stopped after two SRs although a new read-only connection
immediately delivered five SRs in 25 seconds. No further STUS should repeat the RTT/2 seed before
latest-SR freshness, per-side observer recovery and common-mode target recentering are implemented.

A subsequent mutation-free 1,800.022-second holdover collected 381 Left/377 Right SRs in one
continuous segment each with zero restart. Robust RTC/NTP `Right-Left` rate is +16.605 ppm
(conservative 95% +11.863…+21.356); independent RTP media-clock rate is +16.634 ppm. Direct phase
changed +31.356 ms. The Right-faster direction held at every five-minute checkpoint, so oscillator
rate difference is FIELD CONFIRMED for this powered run. Raw artifact SHA-256 is
`8eadecd6d12e2284589383bf22dd32b09b03e6c34927573a84e4994b463efe07`; see
`evidence/stus-relative-oscillator-30m-20260827.md`. Reboot/temperature repeatability remains open.

The user then requested a modification direction in which long-run oscillator drift revoking STUS
reliability causes live distance pairing to use decoded-frame app-receive monotonic time while NTP
remains stored. This is a material Measurement validity and persistence amendment outside the prior
approved STUS scope, so source authorization is paused. The draft reuses the existing pairing owner,
requires a prior manual VERIFIED seed, blocks epoch transitions, hard-bounds app pairs to 100 ms,
flushes on basis changes, preserves distinct selected/app/NTP fields in additive schema v4 and
returns to NTP only after fresh VERIFIED evidence. Recommended output remains numerical but is
`ESTIMATED/DEGRADED_TIMING`; whether to accept that visible/persisted classification is the single
open operator decision.

The user rejected degraded fallback validity and specified app-receive monotonic as authoritative
distance truth if that amendment is resumed, but then deferred the fallback to evaluate shorter Auto
STUS cadence first. The 30-minute field rate gives only 4.98/9.96/14.94/29.89 ms drift at
5/10/15/30 minutes. The latest exact dual-OK STUS produced seconds of relative phase error, so
cadence cannot make the current actuator reliable and repeated Auto would be unsafe. Resume the
previously approved STUS implementation: repair post-ACK SR collection, stale-side observer recovery
and common-mode target recentering; require five consecutive manual cycles before a six-cycle,
5-minute Auto stress test. The stress interval is not the field default; derive that later from
measured residual, 16.6 ppm drift and a safety margin.

The user then superseded both the fallback proposal and NTP-primary live distance: Workstation live
distance must always use app-receive monotonic as its only authoritative pair clock. NTP, SR/RTP
mapping, STUS status and NTP logical epochs remain fully stored but are record-only and cannot select,
block or degrade a distance pair. Accepted app delta ≤100 ms yields normal VALID timing when other
method requirements pass; STUS transitions do not pause app-timed pairing. The boundary provenance
must remain `app_receive_monotonic` rather than being renamed exposure time. The complete backend,
frontend, schema-v4, compatibility, UI and verification amendment is in `approval-summary.md` and
the superseding sections of the packet. This is a material scope change, so source authorization is
off pending explicit approval.

The user explicitly approved the exact superseding amendment at
`2026-08-27T14:31:06+09:00`. Implementation may now change only Workstation live decoded-frame
timestamp capture, pairing, Measurement snapshots/results, additive persistence/readers, fixed-basis
UI, tests and affected product documentation. NTP remains record-only; Legacy, calibration timing,
geometry, coordinates, device settings and historical bytes remain outside this amendment.
