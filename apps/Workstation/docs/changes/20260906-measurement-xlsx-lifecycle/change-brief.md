# Change Brief

## Operator outcome

Every Measurement (the interval between `측정 시작` and `측정 종료`) produces one standalone,
human-readable workbook `<data root>/YYYYMMDD_HHMMSS[-NN]_measurement.xlsx` named after the
Measurement Start local time. The workbook holds every detection candidate, every stereo pair,
every Method result (attempted or not, good or bad), the K,D and R,t actually used, every consumed
Pixhawk/Drone/Base telemetry sample, lifecycle/storage events, a wide analysis sheet, bit-exact
inputs, provenance and a definitions sheet. Distance geometry can be recomputed from the workbook
alone. Measurement-owned MP4 segments share the same Start prefix (`<prefix>_left.mp4`).

## Motivation

The ~2-week field campaign needs post-mortem analysis of *bad* measurements, not a filtered
"clean" record. The previous pipeline hard-rejected pairs (ambiguous candidates, class mismatch,
>100 ms Δt, vertical delta, small disparity, non-positive depth), forced a 9-method request bundle
per pair with per-method timeouts, terminated a Measurement's workers when the next one started,
and only offered a derived `latest_measurement.xlsx` rebuilt from JSONL.

## Included scope

- Measurement lifecycle with a per-Measurement in-memory record and registry; Stop never waits for
  workers, late outcomes are routed by `measurement_id` to their own workbook.
- App-lifetime method workers: subset submission (ready methods only), per-outcome emission,
  per-method backpressure, no compute timeout; crashed/closed workers become `FAILED` evidence.
- Pairing and geometry gates downgraded to advisories (except the Left/Right class-mismatch gate,
  which stays hard); signed-depth geometry policy.
- Canonical `X_right = R_right_from_left * X_left + T_right_from_left` storage for all 9 methods,
  with the live Pixhawk path round-tripping through the same arithmetic so the workbook replays bit
  for bit.
- Workbook writer: complete temporary file, fsync, atomic replace; background thread; periodic
  checkpoints while RUNNING; rewrite on Stop and on every late outcome; failures are events.
- Data > Status shows the workbook writer state and opens the latest workbook.
- Removal of the derived `latest_measurement.xlsx` exporter.

## Excluded scope and non-goals

- JSONL/CSV streams are unchanged (user decision 2026-09-06); the workbook is additional.
- No historical JSONL -> XLSX converter.
- No re-running YOLO/Horizon/Terrain solvers from images; source evidence is recorded, not replayed.
- No compute deadline for a hung-but-alive worker (spec); such a job stays `PENDING` until shutdown.

## Acceptance summary

Pairing/geometry/method/record/workbook/reproduction/telemetry/storage-failure contracts are
covered by the tests listed in `verification.md`; the workbook reproduces every attempted result
bit-for-bit from `91_Exact` and within 1e-9 from the decimal cells of `90_Flat`.
