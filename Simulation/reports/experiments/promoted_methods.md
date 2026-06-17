# Promoted Methods

No method is currently promoted as hardware-validated complete all-axis
correction.

## Current Simulation Runtime PASS Candidate

- `conservative_down_runtime_guard`
  - Status: `PASS`
  - Experiment: `EXP-20260611-013-conservative-down-runtime-guard`
  - Meaning: current live Simulation Runtime PASS under the five-trial
    non-regression MAE gate with right camera roll/pitch/yaw attitude error
    `0.1 deg`.
  - Policy summary: EXP009 runtime policy with Down cell training-improvement
    threshold raised to `0.10 m`, `40` Down cells, stable Right/Forward guard,
    and Forward raw fallback for `anchor_cell=disparity_bin_3`.
  - Promotion status: simulation-runtime candidate only; not hardware/WIRIS
    validated.

## Current Offline PASS Candidate

- `bounded_gain_forward_cell_guard`
  - Status: `PASS`
  - Experiment: `EXP-20260611-009-bounded-gain-forward-cell-guard`
  - Meaning: current offline all-family all-axis PASS under the
    report+trial holdout gate.
  - Policy summary: stable Right/Forward guard, Down selected by
    `profile_family_range_raw_down_center_delta_down_bin` with max selected
    gain `0.25`, and Forward raw fallback for `anchor_cell=disparity_bin_3`.
  - Runtime promotion status: superseded for live Simulation Runtime by
    `EXP-20260611-013-conservative-down-runtime-guard`.

## Stable Baselines

- `range_only_forward_fusion`
  - Status: `STABLE_BASELINE`
  - Meaning: stable Forward improvement with Right/Down protected as no-op.
- `bbox_anchor_axis_guard_rg0p1_dg0_fg1`
  - Status: stable Right/Forward candidate, not a Down solution.
  - Meaning: current best all-family non-noop bbox-anchor branch for
    Right/Forward, with Down no-op.
