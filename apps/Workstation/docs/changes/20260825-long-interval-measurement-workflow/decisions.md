# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-08-25 — Investigation opened

- Type: fact and initial recommendation; not an accepted product decision.
- Fact: measurement intervals are very long.
- Fact: the current parent already spans app restarts, but one old `RECOVERABLE` child blocks the
  next child.
- Recommendation to evaluate: retain a passive Experiment/Campaign grouping while keeping each
  Measurement independently live and sealable, instead of treating the grouping as a runtime or
  daily Session.
- Open alternative: remove the parent entirely and use standalone Measurements with optional tags.

## 2026-08-25 — One execution / one Measurement proposed

- Type: operator proposal; not yet an accepted product decision.
- Proposal: each ordinary Workstation execution creates at most one Measurement, avoiding a
  Session that remains open through the long interval before the next measurement.
- Benefit: simple lifecycle, no manual parent close, independent output per normal field run.
- Main unresolved consequence: a crash/restart can either split one physical measurement or must be
  treated as a recovery exception to the one-execution rule.
- Recommendation: interpret the rule as one *normal completed* Measurement per execution, create no
  Measurement for setup-only launches, and allow a recovery execution to resume the same package.

## 2026-08-25 — Startup recovery is an operator decision

- Type: accepted workflow decision for the proposed change.
- When startup discovers an interrupted/incomplete Measurement, Workstation does not automatically
  resume it and does not automatically create a replacement.
- The operator is shown a factual choice between recovering the existing Measurement and starting
  a new Measurement. Runs with no interrupted Measurement do not show this question.
- The choice must display the existing Measurement identity, age, state, and preserved evidence so
  the operator can distinguish a recent crash from unrelated old work.
- Still open: when `Start new` is selected, whether the old Measurement becomes read-only
  incomplete, remains separately recoverable, or may be deleted after confirmation.

## 2026-08-25 — Start new preserves old evidence as read-only incomplete

- Type: accepted workflow decision for the proposed change.
- If the operator selects `Start new`, the interrupted Measurement is durably converted to a
  read-only incomplete result before the new Measurement is created.
- All verifiable captured bytes, hashes, timestamps, warnings, and explicit missing/failure reasons
  remain available in Review. The old package is no longer resumable and no longer blocks later
  measurements.
- No automatic deletion is allowed. If the read-only preservation step fails, no new writer starts
  and the operator receives retry/review guidance.

## 2026-08-25 — Multiple sequential standalone Measurements per app execution

- Type: accepted workflow decision; supersedes the one-execution/one-Measurement proposal.
- After a Measurement seals successfully, Workstation remains open and returns to a ready state in
  which another independent Measurement may start.
- Camera, Pixhawk, NTRIP, model, and other reusable runtime connections may remain live between
  Measurements; no app restart is required.
- Each Measurement receives a new independent identity. Evidence, writers, spools, finalization,
  and recovery never continue into the next Measurement. Whether the existing parent remains as an
  invisible compatibility owner is a technical scope decision, not part of this operator choice.
- At most one Measurement may be mutable/live at a time.

## 2026-08-25 — Current/proposed convergence identified

- Type: implementation finding; awaiting operator scope choice.
- Current Workstation already supports multiple sequential child Measurements in one app while
  reusable device connections stay live.
- Therefore the accepted operator outcomes do not require physically flattening storage or making
  one wrapper per Measurement.
- Minimal recommendation: retain the current Session internally, create/close it automatically,
  remove its manual lifecycle from the normal UI, and add the approved recovery/new decision plus
  read-only incomplete preservation.
- Larger alternative: physically create a separate one-to-one parent wrapper per Measurement and
  flatten Review presentation. This has more migration/ownership work without changing normal
  capture behavior.

## 2026-08-25 — Scope separated from today's other faults

- Type: scope clarification.
- This packet owns only automatic/internal Session lifecycle and interrupted-Measurement operator
  recovery/new behavior.
- Already resolved NTRIP/MAVLink items are excluded from the remaining-issue list.
- Intrinsics metadata, stereo R,t selection, rover GNSS signal/fix, false PARAM-sync `STALE`, Qt
  timer thread affinity, and long-duration MAVLink qualification remain separate work and are not
  considered fixed by this packet.

## 2026-08-26 — Session and date-time storage root removed from the proposed new-write workflow

- Type: accepted operator direction; supersedes the 2026-08-25 minimal recommendation to retain an
  invisible internal Session for new writes.
- New Measurements do not require a Session to be created, reopened, or closed.
- New evidence is routed to operator-designated destinations by data type rather than beneath a
  date-time-named Session directory.
- The operator gathers the needed outputs after a Measurement has ended; that later grouping action
  does not own live capture and cannot block a subsequent Measurement.
- Existing evidence is not deleted or rewritten by this decision, and legacy Session packages must
  remain discoverable until migration/compatibility scope is explicitly approved.
- Still open: for multiple Measurements, whether each type destination is one append-only aggregate
  file or contains separate immutable per-Measurement items. A shared correlation identity remains
  necessary to avoid mixing evidence from different Measurements; its technical representation is
  Codex-owned after the operator-visible retention behavior is decided.

## 2026-08-26 — Repeated data appends to one configured file per type

- Type: accepted operator decision; resolves the repeated-output retention choice for appendable
  structured data.
- Repeated Measurements append to the same configured file for each data type; they do not create
  one new type file per Measurement.
- Every appended record must carry a Measurement correlation identity and factual timing/boundary
  fields so later collection can select one Measurement without relying on a date-time filename.
- Append recovery must preserve all previously committed records and detect/quarantine only a torn
  final record; an earlier malformed record is not silently skipped.
- Still open: whether this literal one-file rule also covers native MP4/still/ULog/RTCM bytes. These
  formats cannot safely be treated as interchangeable JSONL-style append streams.

## 2026-08-26 — Common Measurement grouping identified as a Session reimplementation

- Type: operator objection and design correction; awaiting confirmation of the resulting loss of
  automatic grouping/completeness.
- The proposal to give every output one common Measurement timestamp/ID plus sequence numbers was
  judged not materially simpler than the current Session.
- Revised recommendation: no parent owner and no mandatory cross-type membership key. Each
  appendable type keeps its configured stream, every record/file keeps its own factual creation
  time, and an optional append-only start/stop event log is only a later time-range aid.
- An interrupted event never owns writers after restart and never blocks a new capture. Only an
  actually active producer/resource may reject a duplicate start.
- Consequence requiring operator confirmation: Workstation no longer automatically proves that a
  chosen set is one complete Measurement; later collection is manual by time range/files.

## 2026-08-26 — Complete availability-first implementation approved

- Type: final accepted product decision and implementation authorization.
- One data-root-relative folder with fixed type streams is used; structured files append without
  rotation, while video/image/ULog originals use Korean-local creation-time filenames.
- Measurement and Manual record delimit structured capture. Manual record stops before a queued
  Measurement and does not resume afterward. Alignment persists until explicit reset or app restart.
- Existing Sessions are file-manager-only and are never scanned for startup admission/recovery.
- Storage failure never blocks Measurement. Under overload, results/events have reserved priority
  and bulk camera/PX4/GNSS/RTCM evidence drops first with visible counts.
- Automatic Parquet, review.xlsx, package manifests, revision, hash-chain, and completeness output
  are removed from new writes.
- Sessions becomes Data with writer health, file-manager actions, HR/OZ, ULog, and Target tools.
- Workstation README is the confirmed README target.
- User explicitly authorized source implementation with `Implement the plan.` at
  `2026-08-26T12:36:43+09:00`.
