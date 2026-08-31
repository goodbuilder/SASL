# Change Brief

## Operator outcome

After a Measurement stops, the operator receives one regenerable
`latest_measurement.xlsx` in the resolved data root. It contains the most recent
Measurement only, with a readable summary and source-specific sheets, and can be
regenerated or opened from `Data > Status`.

## Motivation

Sessionless Workstation currently leaves analysis assembly to external JSONL tooling.
The operator asked for one XLSX that gathers the saved Measurement-related records
without restoring Session/package ownership or changing authoritative storage.

## Included scope

- Latest completed Measurement only; automatic background generation plus manual retry.
- Measurement events/results, eight-method evaluation bundles/outcomes/inputs,
  camera, P1/P2/target, Base GNSS, geometry, native-frame and artifact evidence.
- Exact generation filtering, raw timestamps plus KST display columns, and Excel
  `#N/A` for missing fields.
- Data Status controls, status feedback, cancellation, restart discovery and shutdown.

## Excluded scope and non-goals

- Manual-record-only intervals, general application logs, unrelated capture/alignment data.
- Embedding MP4/PNG/ULog bytes, restoring Sessions/Parquet/revisions/completeness gates,
  rewriting JSONL, migrating legacy `review.xlsx`, or changing measurement algorithms.

## Acceptance summary

XLSX work never gates Measurement. A verified temporary workbook atomically replaces
only an app-owned prior workbook; failure leaves the prior workbook and every JSONL byte
untouched. Missing sources remain explicit rather than synthesized.
