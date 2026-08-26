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
