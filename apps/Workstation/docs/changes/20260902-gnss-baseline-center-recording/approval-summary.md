# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

While Measurement or Manual Record is active, Workstation will save the displayed GNSS-derived
F/R/D/Rg evidence to a new authoritative `gnss_baseline_center.jsonl`. Existing raw P1/P2/Drone
files remain unchanged. Camera Measurement coordinates, validity, settings, device behavior and
recording admission remain unchanged.

## Operator workflow before and after

No new button. Every primary Drone GNSS update during recording attempts one derived row using the
selected P1/P2 samples and exact Drone sample. Data Status shows this stream. The existing Latest
XLSX update automatically adds a `GnssBaselineCenter` sheet for the latest Measurement.

## Success, delay, rejection, failure, overload, and recovery

- CURRENT, STALE and WAITING rows are all saved; WAITING uses null coordinates with a reason.
- No recording means no row. Drone GPS2 remains raw-only and does not duplicate the derived stream.
- The stream uses the existing best-effort bulk queue. A full queue may drop the oldest bulk row;
  write/drop/error counts are visible. Failure never blocks raw GNSS, Telemetry, Measurement or
  another stream. Existing Retry Storage applies to future rows.
- XLSX failure/delay never changes JSONL and uses existing retry/update behavior.

## Data and compatibility effects

Each self-contained row stores state/reason, F/R/D/Rg, frame/origin/axis contracts, NED
intermediates, age/1000 ms timing evidence, selected P1/P2 receiver labels, and exact source
position/fix/topic/app/GPS/MAVLink provenance. JSON null means unavailable; zero remains a real
value. No historical backfill.

Latest XLSX gains an additive fixed sheet. Legacy session Parquet/review, canonical Measurement
fields and existing raw JSONL schemas are excluded. Older readers may ignore the new file/sheet.

## Performance priorities and allowed degradation

No new worker or queue. One constant-time calculation and non-blocking append attempt occurs per
primary Drone GNSS callback. Row budget is <=4 KiB. At 5-10 Hz, plan for up to roughly
72-144 MiB/hour additional JSONL. Under overload, existing bulk oldest-drop policy is preferred to
slowing device callbacks or Measurement.

## Visual and interaction changes, including dynamic-layout stability

No Telemetry/layout change. Data Status automatically gains one ordinary stream row; Latest XLSX
gains one sheet and Summary row. Existing focus, selection, scroll, docks and controls remain.

## How acceptance will be demonstrated

- Independent row-field and source-timestamp sentinel tests.
- Active/inactive, primary/GPS2, missing, stale, current, writer rejection and restart cases.
- Real temp-store append bytes, status counters and <=4 KiB row size.
- Latest-generation XLSX sheet/XML tests and existing source-sheet compatibility.
- Affected/full tests, runtime and portability checks; unrelated residuals reported separately.

## Cost, risks, assumptions, rollback, and field limits

This duplicates compact source evidence intentionally for reproducibility and adds disk/bulk-queue
load. It represents latest asynchronously received samples, not simultaneous GNSS exposure.
Rollback stops future production but never deletes recorded evidence. Hardware proof is unnecessary
for the deterministic recording boundary.

## Exact scope awaiting approval

Approve only the new bulk JSONL stream, one-row-per-primary-Drone recording trigger, exact row
contract, existing Data Status projection, additive Latest XLSX sheet, tests and docs. Exclude raw
row changes, Measurement fields/gates, legacy Parquet/review, settings, commands, backfill,
devices/protocols, new workers/queues, priority/retry-policy changes and unrelated cleanup.

Approved by the operator on 2026-09-02.
