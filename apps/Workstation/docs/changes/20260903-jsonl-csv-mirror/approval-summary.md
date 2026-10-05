# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

Each of the 16 fixed sessionless JSONL files will gain a same-stem lossless CSV mirror in the same
data folder: for example `measurements.jsonl` + `measurements.csv`, `camera_left.jsonl` +
`camera_left.csv`, and `events.jsonl` + `events.csv`. Each mirror has one row per JSONL record and
a lossless `record_json` cell, plus common clock/identity fields.

Because the audit found 562 measurement fields and heterogeneous PX4 rows, the quick-find portion
adds curated CSV views/catalogs rather than forcing all values into one wide table. PX4 quick views
are separated by `sample_type`; measurement, camera timing, GNSS baseline-center, event, artifact,
and log views put the fields operators search most often first. Full values remain in the raw mirror.

JSONL remains authoritative. The existing XLSX, Measurement admission, device behavior, queue
priorities, coordinate contracts, and legacy Session outputs stay unchanged.

## Operator workflow before and after

Before: the operator opens the data folder and must use JSONL-aware tooling for the sessionless
streams. After: the same workflow also offers readable same-stem CSV files automatically; `Data >
Status` shows the paired CSV path and mirror health. No extra save click or modal confirmation is
required.

## Success, delay, rejection, failure, overload, and recovery

Success means a canonical JSONL row and matching CSV row are persisted. JSONL is written first. A
CSV delay/failure is shown as degraded/retrying and never blocks Measurement or the next run. Retry
Storage reopens failed mirror files; bounded retry overflow becomes a visible CSV drop. A foreign,
malformed, symlinked, or unwritable CSV is never overwritten.

## Data and compatibility effects

The CSV envelope keeps the full canonical record in `record_json`; nested values are not silently
flattened or reinterpreted. Existing JSONL files are not rewritten or backfilled in this slice.
Existing valid CSV headers may be appended; conflicting files remain untouched. CSV is a convenience
projection, not a replacement for JSONL audit/recovery or the multi-sheet XLSX review.

## Performance priorities and allowed degradation

The existing single storage writer performs one additional bounded append per record. Priority
JSONL/Measurement work remains first, no GPU or device worker changes, and CSV retries are bounded.
Under disk pressure CSV may lag or drop while JSONL and Measurement continue; the status table
exposes the condition.

## Visual and interaction changes, including dynamic-layout stability

The existing compact Data Status table gains CSV path/state information. The table height, action
row, status strip, navigation, focus, selection, scroll, and latest XLSX controls remain stable.
No generic dashboard styling or new visual surface is introduced.

## How acceptance will be demonstrated

Fresh and restarted stores will be tested with independent JSONL/CSV readers, nested/Unicode/large
values, malformed and foreign files, JSONL and CSV failures, bounded retries, close, and queue
overload. Offscreen UI tests will verify paired paths, factual states, stable geometry, and that
Measurement remains enabled when CSV is degraded. A launcher/affected-suite regression follows.

## Cost, risks, assumptions, rollback, and field limits

Disk write work and storage footprint increase. Cross-file crash atomicity is not claimed; JSONL is
the recovery oracle. Rollback is safe because JSONL/XLSX are unchanged and CSV files are optional.
Hardware/manual verification is not applicable to copying already-created records.

## Exact scope awaiting approval

Implement the stream-wide CSV mirror and quick-find catalog/views in `apps/Workstation`, including
storage ownership/retry, public Data Status visibility, focused tests, affected Workstation
schema/UI documentation, and a confirmed README update. Preserve all full camera/PX4/GNSS/
measurement time-series rows; do not replace them with deltas. Exclude aggregate latest CSV, raw
RTCM index CSV until separately approved, legacy Session exports, historical backfill,
coordinate/time transformation, and unrelated worktree cleanup.

Approved by the user's explicit request on 2026-09-04. Source edits are limited to the
stream-wide sessionless CSV mirror, its independent storage status/retry behavior, directly
coupled Data Status visibility, and focused regression evidence. Historical backfill, aggregate
latest CSV, raw RTCM CSV, and unrelated cleanup remain excluded.
