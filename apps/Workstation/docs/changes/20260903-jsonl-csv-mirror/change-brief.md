# Change Brief

## Operator outcome

After the change, every fixed Workstation JSONL stream also has a same-stem CSV mirror in the
same resolved Measurement data root. The operator can open the data folder and inspect the CSV
files without writing a JSONL parser, while JSONL remains the authoritative record.

## Motivation

The current sessionless store is durable and lossless-oriented, but JSONL is inconvenient for
quick spreadsheet inspection and lightweight analysis. A stream-by-stream CSV mirror provides a
portable tabular access path without collapsing unrelated camera, Pixhawk, GNSS, event, and
application records into one misleading table.

## Included scope

- Add CSV sidecars for all 16 entries in `MeasurementDataStore.STREAM_PATHS`.
- Keep one CSV row per successfully persisted JSONL record.
- Use a stable CSV envelope containing the record clocks, common identity/status fields, and a
  lossless `record_json` column for the complete canonical JSON object.
- Append CSV mirrors through the existing storage owner, with bounded retry and visible health
  evidence; CSV failure never rejects Measurement or prevents JSONL persistence.
- Expose each CSV path and mirror state in the existing `Data > Status` stream table.
- Add focused storage, restart, failure/retry, UI, and end-to-end regression coverage.

## Excluded scope and non-goals

- No single aggregate `latest_measurement.csv` and no replacement of the XLSX workbook.
- No rewrite or migration of existing JSONL bytes, coordinate conversion, timestamp reinterpretation,
  or schema-specific flattening of measurement fields.
- No change to Measurement admission, camera/Pixhawk/GNSS ownership, queues, device protocols,
  legacy Session files, or the existing latest XLSX worker.
- No automatic deletion of old JSONL/CSV files and no unrelated cleanup of the dirty worktree.

## Acceptance summary

On a fresh data root, each accepted JSONL record has exactly one matching CSV mirror row with the
same canonical `record_json`; CSV files append across store restarts and preserve stream order.
Malformed records are rejected consistently, a broken CSV destination is reported and retried
without blocking JSONL/Measurement, and `Data > Status` shows the paired paths and factual state.
