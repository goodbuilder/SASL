# Backend Contract

## Commands and validation

`MeasurementDataStore.STREAM_PATHS` remains the authoritative 16-stream registry. A derived
`CSV_STREAM_PATHS` maps each `.jsonl` path to the same stem with `.csv`. Each mirror has a fixed
header/schema `workstation-measurement-csv-v1` and one row per JSONL object:
`csv_schema`, `recorded_utc_ns`, `recorded_monotonic_ns`, `runtime_generation`, `measurement_id`,
`event_type`, `sample_type`, `status`, and lossless `record_json`.

The writer validates JSONL as it does today, then uses the already canonical JSON bytes to create
the CSV envelope. Nested objects/lists remain in `record_json`; no coordinate or clock fields are
converted. Invalid/foreign CSV headers are rejected without replacement.

## State, ownership, and concurrency

The existing single writer thread remains the owner of all JSONL and CSV handles. A JSONL append is
attempted first. A CSV append is independent after JSONL success and has separate per-stream state,
written count, error, and bounded retry accounting. Producers never wait for CSV I/O.

## Worker resource budgets, scheduling, and backpressure

There is no new GUI or high-rate worker. The one storage writer performs bounded sequential mirror
I/O; the existing priority queue continues to select events/results ahead of bulk rows. CSV retry
reserve capacity and bytes are bounded by the same configured retry limits unless tests prove a
separate smaller cap is needed. Overflow drops only the CSV mirror and is visible.

## External boundaries and workers

The only new boundary is a same-root regular CSV file. Symlinks, directories, short writes, and
malformed headers fail that mirror. JSONL and CSV handles share the existing I/O lock. CSV rows use
standard RFC-style quoting through Python's `csv` writer and UTF-8 encoding.

## Results, errors, timeout, retry, and cancellation

Normal success yields matching JSONL/CSV rows. JSONL failure follows the existing retry/drop path and
does not create a CSV row. CSV failure marks only the mirror, retains the JSONL success, and queues
the canonical row for bounded retry. `Retry Storage` resets failed mirror handles. A mirror that
cannot recover is reported as dropped; no unbounded queue or blocking retry is allowed.

## Persistence, restart, and shutdown

New CSV files append across application/store restarts and write one fixed header when empty. Close
flushes both handle sets and settles or explicitly drops bounded pending CSV retries before reporting
closed. A crash can leave the derived CSV behind the JSONL cutoff; recovery and XLSX continue to use
JSONL. No existing JSONL or CSV bytes are rewritten automatically.
