# ADR-0002: Share immutable derived inputs without changing artifact bytes

- **Status:** accepted, qualified for full v1.2 execution with 8 calibration workers
- **Date:** 2026-10-18
- **Owner:** campaign implementation coordinator under the v1.2 completion instruction
- **Blueprint links:** existing campaign verification ledger; no separate blueprint IDs
- **Related ADR:** [bounded calibration workers](ADR-0001-bounded-offline-analysis.md)
- **Supersedes:** none

## Context and constraints

The G0 benchmark retained 49 RTK JSONL files totalling 30,865,272 bytes but only two distinct content hashes. Different MC trials must not be assumed identical. A writer-based forecast including sensitivity, evaluation, export temporary files and operating reserve is about 150–225 decimal GB against 199.35 GB free at observation. The forecast is not a measured full analysis and remains uncertain.

All required paths, bytes, hashes, provenance, trials and scientific settings must remain available. Raw recordings must never share writable storage with derived outputs.

## Options and proposed decision

| Option | Benefit | Cost/risk | Reversal |
| --- | --- | --- | --- |
| Keep byte copies | Existing independent inode ownership | Capacity forecast may exceed free space | Baseline copy remains for external inputs |
| Remove files or change formats | Potential large saving | Breaks required paths or readers; broader migration | Requires schema migration; rejected here |
| Verified same-owner hardlinks | Identical paths and bytes, no reader/schema change | Linked aliases require immutable ownership; same filesystem; archives must preserve links | Materialize copies only after checking capacity |

Opt in to hardlinks only when source and destination resolve inside the dedicated analysis root. Both generated-input writers and case writers must preserve immutability: existing different bytes remain an error. Reject raw paths and ownership escapes. Verify source before linking and both aliases afterwards. Existing matching targets may remain unchanged. Do not silently copy on cross-device failure, because that invalidates the capacity assumption. External fixture/legacy inputs retain the existing byte-copy behavior.

## Evidence and consequences

Evidence: campaign `source_evidence/g0_analysis_benchmark_001`, independent source/metadata storage review, and [verification ledger](../changes/20261011-manifest-sweep/verification.md). The minimum linking change is estimated to save roughly 45–53 GB in the primary matrix; this is a projection requiring actual inode/block measurement. It does not pool content or assume identical ideal RTK across trials. Hardlinks cannot protect against a writer that violates the immutable-file contract; final checksums detect such corruption.

## Qualification and exit contract

Require actual same-inode/byte/hash tests, conflict/source-change/outside/raw/symlink/cross-device rejection, integrated copy-fallback checks, and a G0 serial/parallel artifact parity plus logical-versus-allocated-byte probe. Recheck free space, inode availability and XLSX temporary filesystem before the full matrix. Stop cleanly with completed immutable cases preserved if capacity admission fails; never reduce trials or delete raw data. Keep status proposed until gates pass. Archive or copy the final derived tree with link-aware tooling if relying on its physical-space saving.

## Verification

- [x] Baseline and alternatives compared
- [x] Scientific invariants and owners identified
- [x] Integrated tests and actual G0 resource/parity gate passed
- [x] Full execution capacity admission recorded

## Qualification decision (2026-10-18)

Accepted under the existing instruction to complete v1.2. Current-source final regression: 403 passed in 127.09s across47 files; archived source hashes unchanged. Actual G0 main serial/spawn2 parity756 scientific files; 8-worker overlap754 equal files,128 cases,937189376-byte peak sampled process-family RSS. Actual dual-scope OFAT serial/spawn2 parity1786 files across100 cases each; unchanged resume in both executors. Source evidence: `pre_v0_final_revision_001`, `g0_parallel_probe_001/main_parity_comparison.json`, `capacity8_overlap_comparison.json`, and `dual_scope_parity_comparison.json`.

Capacity admission records198.90GB free against167.01GB planning need, including1.5x fit projection,20GB evaluation,20GB exports/temp and25GB reserve; /tmp has70GB and output over112million free inodes. Forecast uncertainty remains and requires observation during full execution. Full100MC primary42400 batches plus up-to50000 dual-scope OFAT cases remains unchanged. No real V0 positioning errors were used in this decision. Earlier proposed-state text above records the historical decision context; measured evidence here supersedes its pending-gate statements.
