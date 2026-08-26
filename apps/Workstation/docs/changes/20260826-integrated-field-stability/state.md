---
schema: modify-gui-change-state-v1
change_id: 20260826-integrated-field-stability
status: implementing
mode: backend-coupled
updated: 2026-08-27
baseline_ref: 1198689fafed1ef392d395e11437352431312ac8
last_completed: "R10 separate clean Ubuntu 22.04 KVM passed exact pushed source, manifest installation, package closure, both launchers and isolated HOME"
next_question: "Provide a Workswell firmware/vendor fix or approve a separately designed alternate encoding path; then complete RTK open-sky, cold boots and the final integrated soak"
read_next:
  - docs/changes/20260826-integrated-field-stability/approval-summary.md
  - docs/changes/20260826-integrated-field-stability/implementation-plan.md
  - docs/changes/20260826-integrated-field-stability/verification.md
  - docs/changes/20260826-integrated-field-stability/current-behavior.md
  - docs/changes/20260826-integrated-field-stability/backend-contract.md
  - docs/changes/20260826-integrated-field-stability/frontend-contract.md
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
blocking_questions: 2
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "Integrated field-stability goal in approval-summary.md and implementation-plan.md"
  conditions: []
  recorded_at: "2026-08-26T21:23:57+09:00"
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
STUS ordered-ACK/continuous-health software tests pass; real R3 requires applying the configured
7.0-second SR-age limit and restarting into the latest source. The current running app predates that
slice. Its focused 13-test file, existing interlocks and fresh 1369-test Workstation regression pass.
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
The latest complete Workstation suite is 1466 passed, 1 skipped.
Repository portability now passes with zero findings. A same-host fresh checkout plus new canonical
venv installed the runtime manifest and passed deployment, execution, Workstation and Legacy checks
under an isolated HOME. The recreation exposed and fixed an unconstrained setuptools 84/colcon
conflict by owning `>=77,<80`; fresh resolution selected 79.0.1. R10 now also passes in a separate
official Ubuntu 22.04.5 KVM guest using exact pushed commit `132c185`: all 2,679 archived files
matched, package closure and both launchers passed twice, including isolated HOME. That clean guest
exposed and closed missing `python3-cairo` ownership and the previously untracked/undocumented
Legacy model provisioning contract.
