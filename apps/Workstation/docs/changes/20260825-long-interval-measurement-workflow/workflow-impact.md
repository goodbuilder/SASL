# Workflow Impact

## Affected operators and workflow IDs

- Measurement operator: repeated field measurements with long idle intervals.
- System engineer/researcher: experiment grouping, calibration/alignment provenance, recovery, and
  saved review.
- Affected workflows: Measurement Operation, Recording, Saved Session Review, restart recovery.

## Current workflow

The operator creates one timestamp Session, leaves it open while performing one or more child
Measurements, and explicitly closes it after all children seal. One incomplete child blocks the
next child and parent close. The UI therefore treats the parent as active work even while the rig
is powered down for hours or days.

## Proposed workflow

Latest accepted direction, superseding the internal-parent compatibility proposal:

```text
Launch Workstation
  -> prepare/connect/calibrate as needed
  -> use the one configured append file for each evidence type
  -> start independent Measurement A
  -> stop and durably finish its type-specific outputs
  -> return READY while reusable device connections remain available
  -> wait any duration or start independent Measurement B
  -> later open the saved outputs and collect the required evidence
  -> repeat sequentially or exit Workstation; no Session close exists
```

This removes manual Session creation/close and decouples Measurement identity from process lifetime.
Every completed Measurement is independently portable, while repeated work does not pay the app
restart, device reconnect, model warm-up, NTRIP/GNSS convergence, Pixhawk PARAM refresh, and time
sync costs unless those sources actually disconnected or became invalid.

Current Workstation already performs the repeated child portion of this sequence, but its durable
ownership is still rooted at `recordings/<date-time-session-id>/`. The new direction does not retain
that parent as an invisible compatibility owner for new writes. The initial replacement proposal
put one common Measurement timestamp/ID on every output, but the operator correctly identified that
this recreates the Session's grouping complexity. The revised proposal is intentionally weaker:
each type owns its own append stream, each row/file records its own factual creation time, optional
start/stop events provide a time-range aid only, and the operator later chooses what to collect.

The earlier strict one-execution/one-Measurement alternative was rejected because of these
disadvantages:

1. Process lifetime is not always measurement intent. A crash, update, accidental close, or
   settings-only launch can create empty or fragmented identities.
2. Crash recovery is ambiguous: a new execution must either resume the old Measurement (an
   exception to the rule) or create a second package for one real-world event.
3. Repeating a measurement immediately requires full app restart, device reconnect, camera/model
   warm-up, NTRIP/GNSS convergence, Pixhawk PARAM refresh, and time-sync revalidation.
4. Multi-Measurement comparison loses its explicit parent. Grouping by target/site/campaign then
   needs tags or an external catalog, and aggregate review must be rebuilt from those identities.
5. Calibration, Alignment, HR/ULog, and other evidence currently shared at the parent root must be
   duplicated into each Measurement or moved to a separate persistent profile/evidence owner.
6. Pre-measurement setup may take a long time or occur in a run that never measures. Such a run
   should not be represented as a failed Measurement merely because the app opened.
7. A strict one-run policy couples field throughput to software startup reliability and makes
   quick retries more expensive.

The prior Experiment recommendation and invisible internal Session are no longer the proposed new-
write owners. Review grouping, if retained, is a later operator action rather than a live workflow
owner:

```text
Select/create a passive Experiment
  -> Start one independent Measurement event
  -> seal complete or preserve explicitly incomplete
  -> return Experiment to IDLE
  -> power down/disconnect for any duration
  -> later start another Measurement in the same or another Experiment
```

This retains grouping and shared evidence but removes any implication that the grouping itself is a
live runtime session. An interrupted child would require an explicit `Resume`, `Preserve as
Incomplete`, or `Discard draft` decision before it can affect later work; silent deletion remains
forbidden.

## Preserved, added, removed, and reordered steps

- Preserve: immutable per-Measurement package, source snapshots, hash/manifests, recovery evidence,
  and saved review.
- Accepted change: remove Session lifecycle and the date-time Session root from new operation.
- Accepted change: configure storage by evidence/data type and gather outputs later.
- Accepted simplification: automatic cross-type Measurement membership and completeness are not
  preserved; the operator later selects evidence by factual time/file evidence.
- Accepted change: repeated structured outputs of one type append to one configured file rather
  than creating a new file for each Measurement.
- Accepted binary behavior: native video/image/ULog originals remain separate and use
  collision-free Korean-local creation-time filenames; their artifact events append by type.
- Candidate addition: explicit resolution of an old incomplete child without forcing immediate
  successful finalization.
- Accepted removal: requirement to keep a runtime-like parent `OPEN` throughout long idle gaps.

## Decisions, confirmations, handoffs, waits, and artifacts

The new workflow has no Session-create or Session-close handoff. Stop flushes the currently active
producers and immediately permits later work. A dangling start event after a crash is factual log
evidence, not an owner, recovery obligation, or gate on the next start. Under the revised proposal,
the later collection step does not promise automatic completeness; the operator selects the desired
time range and files.

## Interruption, cancellation, retry, restart, and resume

Current restart automatically adopts the only open parent and marks an interrupted child
`RECOVERABLE`. Proposed behavior must distinguish reopening the passive experiment from resuming
that specific child and must never silently merge evidence from separate measurement events.
For one-run/one-Measurement, startup pauses new Measurement creation only when it finds interrupted
work and presents `Recover existing` versus `Start new`. Recovering reuses the exact package
identity and spool. Starting new must resolve ownership of the old spool first; two live writers
remain forbidden. The accepted resolution is terminal read-only incomplete: review remains
available, resume is no longer offered for that package, and no captured evidence is deleted.

## Degraded, error, and recovery behavior

An interrupted package remains recoverable until the operator chooses. `Recover existing` resumes
the exact identity. `Start new` preserves all verifiable evidence and the missing-role/failure list
as a read-only incomplete result. If that preservation cannot be verified, new Measurement
creation is rejected and the source recovery bytes remain unchanged.

## Completion and next actions

A successful seal returns the application to `READY FOR NEXT MEASUREMENT`. Starting again creates a
new identity only after every writer/drain/finalization owner from the prior Measurement is closed.
Review/open-folder actions remain available between Measurements.

## Training, operating-procedure, and compatibility impact

Terminology and the first field-work action change: no Session folder step. Existing
`(session_id, measurement_id)` packages remain a required legacy read contract unless a separately
approved migration is chosen. New-write Review and collection identity no longer depends on a
Session.

## Workflow-to-screen, command, state, and regression traceability

- Screens: Navigation Session/Sessions, `SessionWorkspace`, Measurement, Recording/Review,
  Inspector and global REC/MEAS status.
- Commands: create/close parent, measurement start/stop, retry save, review/open folder.
- State: `MeasurementSession`, `MeasurementPackage`, recording retry flags, startup recovery.
- Regression: multiple children, long restart gap, incomplete child, shared calibration/alignment,
  parent review, legacy package discovery, shutdown and retry.
