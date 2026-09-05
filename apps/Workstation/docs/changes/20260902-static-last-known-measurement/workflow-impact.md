# Workflow Impact

## Affected operators and workflow IDs

## Current workflow

If a required live source is absent when a new Measurement begins, the operator sees no ready
method and must restore P1/P2 telemetry or prepare another geometry method.

## Proposed workflow

After each required side has produced valid attitude and GPS at least once, a later disconnect
automatically retains the last valid sample. The live method remains selectable with a prominent
`RETAINED STATIC` state and sample ages. A newer valid sample automatically replaces the retained
sample and returns that component to `LIVE`.

## Preserved, added, removed, and reordered steps

- Preserved: normal live measurement and all alternate geometry methods.
- Added: automatic transition from live to retained-static input and back to live.
- Removed: no operator Freeze or Clear action.

## Decisions, confirmations, handoffs, waits, and artifacts

No added operator decision, confirmation, wait, or artifact. Existing artifacts keep exact input
values/timestamps with no new retained marker.

## Interruption, cancellation, retry, restart, and resume

Stopping one Measurement does not by itself erase process-local last-valid observations.
Application shutdown/restart always erases them; live eligibility must be established again from
new receiver evidence.

## Degraded, error, and recovery behavior

- Never-observed, invalid, or non-finite inputs remain blocking.
- Disconnect after a valid observation is degraded-but-measurable, not silently live.
- Reconnection with a valid newer sample recovers automatically.
- If the physical rig moves while input remains retained, automatic recovery is impossible until
  live telemetry resumes or the application restarts; this accepted risk must remain visible.

## Completion and next actions

Measurement completion and next actions are unchanged.

## Training, operating-procedure, and compatibility impact

Operators may rely on automatic last-valid fallback only while the rig remains static. Restart
requires new valid samples. No UI retraining is required.

## Workflow-to-screen, command, state, and regression traceability

The backend readiness change projects through the existing method selector; no command path is
added. Regression covers live, retained, invalid, reconnect, restart, other methods, recording,
and unchanged evidence fields.
