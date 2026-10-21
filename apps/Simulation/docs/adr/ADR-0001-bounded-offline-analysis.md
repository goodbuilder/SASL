# ADR-0001: Bounded offline calibration workers

- **Status:** accepted, qualified for full v1.2 execution with 8 calibration workers
- **Date:** 2026-10-18
- **Owner:** campaign implementation coordinator, under the user's v1.2 completion instruction
- **Blueprint links:** existing verification ledger; no separate blueprint IDs
- **Supersedes:** none

## Context and decision question

The 39 raw slots have passed fresh QC. The required analysis retains all 100 MC trials, 42,400 primary batches, and every declared sensitivity case. A G0-P0 trial with eight yaw cases and two scopes took 136.54 seconds in one process, with peak RSS 113,876 KiB. This is a representative measurement, not a full-matrix runtime guarantee. How can independent fits use available CPUs without changing scientific inputs, selection, or freeze order?

## Constraints and options

| Option | Benefit | Cost or risk | Reversal |
| --- | --- | --- | --- |
| Keep serial execution | Existing deterministic order and simplest ownership | Long measured per-trial processing; no CPU overlap | Baseline remains `workers=1` |
| Thread pool | Standard library, little process startup | Shared interpreter/cache state and native library threading require additional isolation proof | Remove executor |
| Bounded spawn process pool | Standard library, isolated caches, explicit unique disk owners | Startup/serialization, cancellation and join lifecycle, native thread limits | Select one worker before evaluation; preserve completed immutable cases |

## Proposed decision and boundaries

Use bounded spawn workers for calibration only. One main task owns `(run_id, MC trial)` and all its yaw/scope cases. One sensitivity task owns `(scenario_id, run_id, MC trial)`. The coordinator generates shared camera sensors before submitting each trial, owns ordered indexes, G0 path selection, freeze, evaluation, exports, and the latest pointer. Workers return thin indexes, with G0 trial-zero Y0 results only where path selection requires them. They never evaluate V0.

Keep existing submission order independent of completion order. A single output-root lock excludes a second coordinator. Duplicate task keys fail. On error stop submission, cancel pending tasks, and join active workers before propagating; no freeze follows failure. Retain current fail-closed checksums and partial-output preservation. Native BLAS/OpenMP limits must be inherited before child imports.

## Evidence, consequences, and uncertainty

Evidence: [verification ledger](../changes/20261011-manifest-sweep/verification.md), and campaign `source_evidence/g0_analysis_benchmark_001`. Independent source review confirmed run/trial RTK ownership and coordinator-only freeze requirements. No parallel speedup has yet been measured. V0 accuracy remains unopened.

Memory, disk capacity, byte parity, and actual cancellation/restart behavior remain qualification gates. Storage includes repeated RTK copies and needs a full forecast including sensitivity, V0 results, export and reserve. No raw files, scientific conditions, MC counts, loss functions, or acceptance gates change under this decision.

## Re-evaluation and migration

Qualify deterministic serial/parallel results, reverse completion, duplicate exclusion, output-lock contention, failed-worker drain, partial-file preservation, and freeze barrier before full execution. Run a G0-only real-data parity/resource probe. If parity or ownership fails, keep the decision proposed and repair or use serial execution; never weaken the matrix or inspect V0 to choose the executor. Reassess worker count from actual RSS/I/O/elapsed measurements. Any scientific change after V0 evaluation requires a new disclosed development version.

## Verification

- [x] Baseline and credible alternatives recorded
- [x] Existing user authority and limited scope identified
- [x] Evidence and uncertainties distinguished
- [x] Parallel parity, failure lifecycle, resource and storage gates passed
- [x] Decision accepted after qualification evidence is recorded

## Qualification decision (2026-10-18)

Accepted under the existing instruction to complete v1.2. Current-source final regression: 403 passed in 127.09s across47 files; archived source hashes unchanged. Actual G0 main serial/spawn2 parity756 scientific files; 8-worker overlap754 equal files,128 cases,937189376-byte peak sampled process-family RSS. Actual dual-scope OFAT serial/spawn2 parity1786 files across100 cases each; unchanged resume in both executors. Source evidence: `pre_v0_final_revision_001`, `g0_parallel_probe_001/main_parity_comparison.json`, `capacity8_overlap_comparison.json`, and `dual_scope_parity_comparison.json`.

Capacity admission records198.90GB free against167.01GB planning need, including1.5x fit projection,20GB evaluation,20GB exports/temp and25GB reserve; /tmp has70GB and output over112million free inodes. Forecast uncertainty remains and requires observation during full execution. Full100MC primary42400 batches plus up-to50000 dual-scope OFAT cases remains unchanged. No real V0 positioning errors were used in this decision. Earlier proposed-state text above records the historical decision context; measured evidence here supersedes its pending-gate statements.
