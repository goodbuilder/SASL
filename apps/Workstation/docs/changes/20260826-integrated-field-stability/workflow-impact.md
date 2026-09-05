# Workflow Impact

## Affected operators and workflow IDs

Single field operator; camera preview/time sync, Pixhawk connection, network RTK, Measurement,
recording/evidence, failure recovery and shutdown workflows.

## Current workflow

An operator can see ping/port success while media, telemetry or corrections are not usable. Old
capture/storage state may obstruct new work, and a short clean preview can conceal cumulative drift,
memory growth or later protocol silence.

## Proposed workflow

Run prerequisite checks, then short protocol/application gates. Correct only the failing slice and
repeat its gate. After every independent gate passes, run 10-15 minute qualifications, three cold
starts and one final 60-minute combined soak. Stop early on the first required failure.

## Preserved, added, removed, and reordered steps

Preserved: explicit operator controls, manual intrinsics, safe CPU fallback, device settings and
existing evidence. Added: external postcondition gates and durable qualification artifacts.
Reordered: long soak follows independent qualification. Removed: no product behavior here; false
success is replaced only when a reproduced defect justifies a source slice.

## Decisions, confirmations, handoffs, waits, and artifacts

User performs named physical motion, USB connection or power-cycle only on request. Codex owns
software, read-only protocol probes, evidence analysis and bounded test execution. Each wait has a
reason and early-failure condition. Artifacts use the integrated packet/evidence locations.

## Interruption, cancellation, retry, restart, and resume

Interrupted tests retain evidence and release owners before retry. A new candidate starts from clean
process ownership and records its start identity. Resume reads `state.md` and `verification.md` first.

## Degraded, error, and recovery behavior

Partial camera/Pixhawk/storage failures remain explicit and isolated. Silent software fallback,
stale evidence substitution, repeated host reconnect for a remote deadlock, and automatic device
reboot are forbidden.

## Completion and next actions

Completion requires the complete matrix with no unresolved required item. A vendor/firmware blocker
is reported with reproduction evidence and keeps the goal incomplete unless the user explicitly
accepts a named residual condition.

## Training, operating-procedure, and compatibility impact

Operator guidance gains a short-gate versus final-soak distinction, side-specific recovery, and
clear success evidence. Existing stored data remains readable and untouched.

## Workflow-to-screen, command, state, and regression traceability

Traceability is defined in `integration-flow.md` and `verification.md`; every operator control maps
through its controller/worker/external result to persisted and visible evidence.

## Proposed clock-revocation fallback workflow — not yet approved

Current live-camera behavior forces RTCP/NTP-required pairing. An opportunistic app fallback exists
only when an NTP timestamp is missing; NTP that is present but drifted can still select the wrong
pair. The proposed flow is:

1. Before the first manual VERIFIED seed, retain the existing blocked/unverified behavior.
2. While Clock is VERIFIED, pair on camera-derived RTCP/NTP and preserve the 100 ms limit.
3. During an active STUS/logical-epoch transition, drop candidates and wait; never bridge epochs.
4. If a previously VERIFIED clock is revoked by sustained relative drift or current clock evidence
   is unavailable, flush the old pair buffers and select future pairs on per-frame app-receive
   monotonic time, still requiring at most 100 ms.
5. Continue recording NTP/RTP/SR values and the untrusted NTP delta as diagnostics. Display and save
   the numerical distance with an explicit app-timing fallback reason.
6. After a fresh VERIFIED result, flush fallback candidates, advance the timing-basis generation and
   return future pairs to NTP.

No operator retry is required to keep degraded distance output flowing. Manual/Auto STUS continues
independently in the background. The operator sees Clock remain failed/revoked rather than having
fallback falsely turn the clock status green.

## Superseding fixed app-receive workflow — awaiting approval

The fallback workflow above is superseded. As soon as decoded live frames are available, all
distance pairs use app-receive monotonic time; no STUS seed or Clock state is a prerequisite.
Accepted app pairs continue during STUS and NTP epoch changes. NTP is captured and stored when
available but never participates in the operator's pair decision. An app delta over 100 ms causes an
ordinary unmatched/drop wait, while a pair inside the bound proceeds to normal distance validity.
The operator manages STUS only for independent clock/evidence needs, not to unlock distance.

## Proposed indoor 60-minute experiment workflow — awaiting approval

1. Operator keeps a safe moving target in both camera views; Codex owns all software actions.
2. Stop every Workstation/Legacy/raw RTSP/SDK owner and snapshot process, ports, routes, storage and
   non-secret settings. Do not query encoder fields during the soak.
3. Launch one Workstation in the explicit NAL1 experiment mode with process-local NTRIP credential.
4. Connect camera, P1/P2 and NTRIP; start Measurement/YOLO and recording. Run a five-minute gate.
5. If any required gate fails, stop and repair that boundary rather than waiting 60 minutes.
6. If it passes, observe continuously for exactly 3,600 seconds and write a checkpoint every five
   minutes. Clock/SR is observed but STUS is not mutated.
7. Stop Measurement/recording/connections, wait for EOS/writers, quit, and independently inspect
   saved data, MP4 failure truth and resource cleanup.
8. Relaunch normal strict mode. Report each problem independently; known zero-IDR/direct-copy MP4
   and STUS mutation are not converted to PASS.

The only added operator concept is that this is a temporary experiment run. Normal field workflow,
settings and manual recovery remain unchanged after the process exits.

## Approved manual-only PX4 calibration PARAM workflow

Before: after P1/P2 completed ten continuous minutes and first became `LIVE`, Workstation silently
started one calibration PARAM snapshot per side, showed `PARAM SYNC`, then restarted qualification.

After: reaching `LIVE` performs no PARAM operation. When calibration detail or MAG-slot evidence is
needed, the operator presses the existing `Refresh P1 parameters` or `Refresh P2 parameters` command
(or the existing Advanced MAG/attitude refresh). The same progress, error, retry, disconnect, and
post-command qualification behavior applies only to that explicit request. No retraining beyond
knowing that calibration rows remain `PARAM readback 대기 중` until Refresh is required.

## Approved 60-second operational qualification workflow

After Connect or a real recovery, the operator waits one continuous minute of fresh P1/P2
HEARTBEAT+ATTITUDE before attitude-dependent methods may treat that side as LIVE. Reaching LIVE
does not send PARAM traffic. An explicit/manual PARAM action keeps its existing busy/result/error
flow and starts a new one-minute interval. Ten-minute/cold-boot/60-minute experiments continue in
the background as stability evidence and do not keep routine measurement blocked.

## Approved Mountain GNSS 3D-fix workflow

1. Capture the Left/Right images while both P1/P2 GNSS samples report numeric `fix_type >= 3`.
2. Mark or accept at least four same-order distributed Mountain correspondences and press
   `Solve GNSS + Mountain R,t`.
3. Review the independent rotation gate, numeric fix types, baseline and warnings. RTK Fixed is not
   required; non-fixed 3D/RTK-Float evidence remains `ESTIMATED`.
4. Start the next Measurement to freeze the latest Mountain Draft automatically. Each accepted
   target pair may then emit `mountain_gnss` comparison evidence without global Apply.
5. Use `Apply candidate` only when promoting that Draft to the global Active Alignment is desired.

Solving after a Measurement has started affects the next Measurement, never its already-frozen
inputs. The representative result remains `pixhawk_rtk_live`.

## Mountain Auto optional-copy workflow — approved 2026-08-29

1. `Auto-select Mountain` creates the same reviewable overlay and detector-quality evidence.
2. The operator may exclude paired points. The status independently reports `COPY READY` or the
   structural blocker.
3. The operator presses `Use suggestion` to copy the reviewed points. `LOW_CONFIDENCE` is allowed
   when structure passes and remains a visible/persisted warning.
4. The operator separately presses `Solve GNSS + Mountain R,t`, which consumes the copied
   annotations through its unchanged K,D, GNSS and robust-rotation gates.

Direct use by Solve and every automatic copy/Solve/Apply path remain absent. Manual Mountain,
candidate review, Measurement Draft freeze and restart behavior remain unchanged. Operator
retraining is limited to quality-versus-copy-readiness semantics.

## Horizon Auto optional-copy workflow — approved 2026-08-29

1. `Auto-select Horizon` creates the same overlay and quality evidence.
2. The status reports `COPY READY` only for current Left/Right endpoint structure; detector quality
   remains visible but does not disable the button.
3. The operator presses `Use suggestion`; low-quality state/warning persists with the copied
   endpoints.
4. The operator separately presses `Solve Horizon Roll/Pitch` and, if desired, `Apply candidate`.

Manual Horizon and all Solve/Apply K,D/observability checks remain. No automatic action or
retraining beyond quality-versus-copy-readiness semantics is introduced.

## Approved Settings-preferred distance timing workflow — 2026-08-31

1. The operator leaves the default `App receive` or selects a non-app timing preference in
   Measurement Settings.
2. A new Measurement freezes that requested preference. Apply/Save during an active Measurement
   persists the next-run value without changing the current run; idle Apply publishes a new policy
   generation and clears candidates accumulated under the previous policy.
3. Healthy selected RTCP/NTP actually selects the pair. If its current Clock/frame/epoch/delta
   evidence is unusable, the same run continues on exact app receive without an operator retry or
   validity downgrade caused solely by fallback.
4. Measurement shows requested/effective basis and fallback reason while Time / Sync continues to
   report RTCP/NTP/STUS truth independently.
5. Public Clock recovery advances the policy generation and flushes old candidates before future
   pairs return to RTCP; no pair or frozen result may bridge policy generations.

Camera, Detection, Measurement, recording and PX4/GNSS workflows otherwise remain unchanged.
