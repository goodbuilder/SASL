# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-09-06 — Per-Measurement workbook replaces the derived latest XLSX

- One workbook per Measurement Start→Stop, flat in the resolved data root, named by Start local
  time with `-NN` suffixes on same-second collisions. Never overwritten in place: temp → close →
  fsync → atomic replace.
- `pixhawk_rp_gps_baseline_yaw` stays a ninth Method with its own tab `18_Pixhawk_RP_GPS_Yaw`.
- All 16 JSONL streams and their CSV mirrors stay exactly as they are; the workbook is additional
  and captures producer records through an enqueue observer that runs before JSONL admission.
- `latest_measurement_xlsx.py`, its coordinator and Data > Status buttons are removed.
- Measurement-owned MP4 segments use the workbook prefix; other artifacts keep `videos/` etc.
- Docs: compact packet (this folder) plus `workflow.md`.

## 2026-09-06 — Geometry keeps bad results

- Removed as gates: ambiguous same-class candidates, horizontal disparity ≤ 1e-6,
  Δt > window (100 ms hard cap), vertical bbox delta, non-positive depth. All remain as warnings /
  diagnostics; the pairing window only prefers the nearest partner.
- Kept as a hard gate (user decision 2026-09-06, "class gate 복원"): a Left/Right class mismatch
  rejects the pair (`correspondence_class_mismatch`) at pairing and in the evaluator.
- Only mathematically undefined geometry leaves Forward/Right/Down/Range empty: missing input,
  invalid input, zero baseline, rank-deficient (parallel) rays, non-finite solution, output-frame
  construction failure, invalid quaternion. Inputs, u,v, rays, R,t, status and reason are still
  written.

## 2026-09-06 — Method execution

- Submit only ready Methods per pair; non-ready Methods are recorded `Attempted=NO, Status=N/A`
  with their reasons. Outcomes are published individually; a pair's JSONL bundle row is written
  when its submitted subset completes.
- No compute timeout. Worker exit → `FAILED` + restart; coordinator close → `FAILED`. The spawn
  acknowledgement guard (30 s) is process liveness, not a compute deadline.
- Per-method pending-depth limit 20 → `BACKPRESSURE_BLOCKED` for that method only.
- Workers are application-lifetime; a new Measurement never retires them. Late outcomes go to
  their own Measurement's record and never to the current run's UI/counters/JSONL rows.
- `TIMEOUT` remains readable from old JSONL but can no longer be produced.

## 2026-09-06 — Bit-exact reproduction

- The live Pixhawk triangulation now derives `R_right_from_left` from the attitude quaternions,
  forms `T = -R C`, and re-inverts to kernel inputs — the same arithmetic the reproduction runs
  from stored R,T. Numeric change is at the 1e-16 level; no contract change.
- `91_Exact` stores `float.hex()` of K, D, u,v, rays, R, T, quaternions, results and diagnostics;
  `90_Flat` decimal cells satisfy `math.isclose(rel_tol=1e-9)`.
