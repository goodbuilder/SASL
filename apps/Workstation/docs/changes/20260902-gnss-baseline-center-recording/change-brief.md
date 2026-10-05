# Change Brief

## Operator outcome

During Measurement or Manual Record, every primary Drone GNSS position callback writes one
self-contained `GNSS BASELINE_CENTER F/R/D/Rg` evidence row. The operator can inspect the
authoritative JSONL and the derived Latest Measurement XLSX sheet without reconstructing the live
UI calculation by hand.

## Motivation

The completed Telemetry display is runtime-only. Existing P1/P2/Drone raw GNSS rows make offline
recalculation possible, but require a three-stream timestamp join and correct selected-receiver and
coordinate-contract reconstruction. Persisting the exact input snapshot and derived state removes
that ambiguity while preserving raw evidence.

## Included scope

- New bulk stream `gnss.baseline.center` -> `gnss_baseline_center.jsonl` in the fixed Workstation
  measurement data root.
- Exactly one attempted row per primary Drone `GPS_RAW_INT` callback while Measurement or Manual
  Record is active; no UI-poll rows and no Drone `GPS2_RAW` duplicate trigger.
- Store CURRENT, STALE, and WAITING outcomes, exact selected P1/P2/Drone source timestamps/position/
  fix/topic/provenance, state/reason, NED intermediates, F/R/D/Rg, timing, and explicit frame/origin/
  axes.
- Additive `GnssBaselineCenter` sheet in `latest_measurement.xlsx`, filtered to the latest exact
  Measurement generation like other source sheets.
- Data Status automatically exposes stream liveness/count/drop/error through the existing store.
- README, `docs/RECORDING_SCHEMA.md`, affected UI docs, focused storage/recording/XLSX tests, and
  full regression verification.

## Excluded scope and non-goals

- No change to raw `px4_left/right/drone.jsonl`; no time-joining or rewriting historical files.
- No canonical Measurement `baseline_center_*` fields, representative result, validity gate,
  method evaluation, recording admission, session Parquet, legacy package/review workbook, setting,
  command, control, device/protocol, UI layout, or new worker/queue.
- No rows outside Measurement/Manual Record and no retroactive backfill.
- No RTK-Fixed requirement and no camera/body/lever-arm correction.

## Acceptance summary

- A Drone callback produces exactly one row with the same pure 2.0 s/1000 ms state and numeric
  geometry shown live, using the exact selected P1/P2 source samples.
- Missing P1/P2 still produces a WAITING evidence row with null derived numbers, not a silent gap or
  fabricated zero.
- No row is written outside recording; GPS2 does not trigger a second derived row.
- The new stream is bulk/best-effort. Queue/write failure is visible in Data Status and cannot stop
  raw GNSS, Telemetry, Measurement, or other streams.
- JSONL appends across restart without overwriting; decoded non-finite GNSS fields become null with
  explicit state, and any other malformed one-row input is isolated.
- Latest XLSX contains only the selected Measurement generation and leaves JSONL authoritative.
