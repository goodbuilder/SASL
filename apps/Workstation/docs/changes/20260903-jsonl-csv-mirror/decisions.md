# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-09-03 — Stream-wide CSV mirrors requested

- User clarified that the goal is not a single `latest_measurement.csv`; JSONL records themselves
  should also be saved in CSV form.
- Scope: all current fixed Workstation JSONL streams, one same-stem CSV per stream.
- Preserved: JSONL authority, existing XLSX, stream separation, coordinate/clock values, and
  availability-first Measurement behavior.
- Proposed technical contract: one CSV row per JSONL object with a stable envelope and lossless
  `record_json`; CSV is a derived convenience mirror and may be independently degraded.
- Approval: user confirmed the stream-wide direction in conversation; packet implementation remains
  awaiting explicit approval of the documented failure/performance/UI consequences.

## 2026-09-03 — Quick-find audit supersedes the single raw-table assumption

- The user asked for every stored value to be inventoried, its importance evaluated, and the
  change-only behavior checked so files open quickly for inspection.
- Evidence: the current writer persists full snapshots on every accepted row; it does not persist
  only changed fields. Measurements and telemetry need complete time series for pairing and failure
  analysis, while event/config/provenance streams are append-only events.
- Recommendation: implement a lossless per-stream CSV mirror plus curated quick-view CSVs/catalog,
  with PX4 views split by `sample_type`; do not replace time-series records with deltas.
- Separate scope question: `raw/gnss/*.rtcm.jsonl` is a binary/index owner outside the fixed 16
  streams and requires an explicit paired-index CSV decision.

## 2026-09-03 — Review formats narrowed to JSONL and CSV

- The user wants to review each data file's contents one at a time.
- New mirror design should focus on the authoritative JSONL plus a matching CSV; the existing XLSX
  feature is not part of the new per-file schema review and remains unchanged.
- The first schema under review is `events.jsonl`/`events.csv`.

## 2026-09-03 — Events schema accepted for sequential review

- User confirmed the `events.jsonl`/`events.csv` field groups and search-first column order.
- `events` remains an event stream: preserve each event row, including repeated context fields; do
  not turn it into a generic previous-value delta log.
- Continue review with `measurements.jsonl`/`measurements.csv` before requesting final packet approval.

## 2026-09-03 — Pair identity and measurement validity must be separated

- User correctly pointed out that one left/right detection pair should represent one logical row.
- Current JSONL is an append-only audit log: it can append an initial full pair result and later
  append a full `replace` row for the same `_record_index` when asynchronous native evidence completes.
- `measurement_valid` does not mean "a pair exists". It means the computed measurement result passed
  the geometry/result validity decision. A pair may exist but be blocked, unresolved, timed out, or
  have invalid timing/geometry; those cases still need a row for audit.
- Recommendation for the CSV quick view: materialize one final row per logical pair by applying
  append/replace history. Keep `pair_status`/`result_state` (or the existing `measurement_validity`)
  as explicit result state; do not use a boolean as the pair-presence indicator. Raw JSONL/row-level
  CSV audit may retain both append and replace records if exact duplication is required.

## 2026-09-03 — Measurement CSV is reduced to a pair-result view

- User requested the measurement representation be made as simple as possible.
- The compact `measurements.csv` should contain one final row per logical pair and only the fields
  needed to find the result quickly; detailed camera/PX4/GNSS/calibration/native values remain in
  their own stream files and in authoritative JSONL.
- Remove the redundant boolean `measurement_valid` from the compact CSV. Use one explicit `status`
  value (`VALID`, `ESTIMATED`, `BLOCKED`, `FAILED`, or `UNAVAILABLE`) plus `quality` where needed.
- Keep `pair_sequence`/`measurement_id` as identity; do not use `valid` as a pair-existence flag.

## 2026-09-03 — Multiple methods use one long-format measurement CSV

- For two methods evaluated on the same left/right pair, do not create two `measurements.csv`
  files. Use one table with one row per `(pair_id, method_id)` and repeat the shared pair identity.
- Example: the same `pair_sequence=12` has one `pixhawk_rtk_live` row and one
  `checkerboard_stereo` row; `method_id` is the comparison key.
- A method failure remains a row with explicit `BLOCKED`/`FAILED` status and empty result values;
  another method's result is not suppressed. This scales to the current eight method catalog.
- Current implementation detail: `measurements.jsonl` is the representative primary stream, while
  `method_evaluations.jsonl` already stores a bundle with an `outcomes` list. The CSV projection can
  normalize those outcomes into the long-format method rows without changing JSONL ownership.

## 2026-09-03 — Recommended measurement row width

- One pair-method row uses 19 columns: `run_id`, `pair_id`, `pair_sequence`, `time_utc`, `method_id`,
  `status`, `quality`, `distance_m`, `baseline_center_forward_m`, `baseline_center_right_m`,
  `baseline_center_down_m`, `baseline_center_range_m`, `disparity_px`, left/right bbox centers,
  `pairing_time_basis`, and `selected_pair_abs_delta_ms`.
- A `status_reason` column may be added as the 20th column if blocked/failed method explanations
  must be visible without opening JSONL; it is blank for normal rows.
- For two methods on one pair, the CSV therefore has two rows × 19 (or 20) columns, not two files.

## 2026-09-04 — Stream-wide raw CSV mirror implementation authorized

- The user explicitly requested that the current sessionless measurement data also be saved as CSV.
- Implement the approved raw same-stem CSV mirror for every fixed `MeasurementDataStore` JSONL
  stream, with one row per successfully written JSONL record and the complete canonical record in
  `record_json`.
- Keep JSONL authoritative, preserve producer-authored coordinate/frame and clock fields, and make
  CSV failure independently retryable/degraded without rejecting Measurement.
- Curated quick-view projections, historical backfill, aggregate latest CSV, and raw RTCM CSV remain
  outside this implementation slice.
