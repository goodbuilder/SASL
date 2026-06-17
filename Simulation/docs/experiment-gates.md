# Experiment Gates

This document defines gates for StereoDistance correction experiments. Use it
together with `docs/experiment-workflow.md`.

## Current Scenario

The current primary scenario is:

- target: 15 m UAM/drone object
- right camera attitude error: roll/pitch/yaw each increased by `0.1 deg`
- internal correction axes: RDF, meaning Right, Down, Forward
- legacy StereoDistance position topic order: FRD+range, meaning Forward,
  Right, Down, Range
- metric: `raw RMSE - corrected RMSE` in meters
- positive delta means corrected is better than raw
- zero delta means non-regression, often caused by no-op fallback

## Exploration Gate

Use this gate for fast candidate rejection.

Required:

- output artifacts are written under an experiment directory or linked from it
- command and inputs are recorded
- at least one completed report or equivalent offline replay exists
- no invalid or partial input is counted as evidence
- per-axis deltas are reported separately

Allowed result statuses:

- `PARTIAL_PASS`
- `NEAR_PASS`
- `UNDER_EVIDENCED`
- `REGRESSION_SMALL`
- `REGRESSION_LARGE`
- `INVALID_TEST`
- `REJECTED`

## Validation Gate

Use this gate before calling a method stable.

Required:

- the truth point being estimated is defined
- the runtime-observable feature or reference point is defined
- calibration-time truth use is separated from measurement-time runtime inputs
- runtime correction does not require PX4/RTK truth, true disparity, or true
  image feature location
- PX4 local NED, internal stereo-center RDF, and any public StereoDistance
  FRD+range topic contracts are explicit
- profile and validation data are separated
- at least five validation trajectories or a justified equivalent holdout set
- every validation trajectory reports Right, Down, Forward, and Position deltas
- every claimed axis has non-negative per-trajectory RMSE improvement
- all failures are reported, not averaged away
- bootstrap or subgroup checks are run when sample-level data is available

For the current offline bbox-anchor report set, a strong validation substitute
is:

- report+trial holdout (`report_trial`)
- all relevant report families included when claiming all-family behavior
- training excludes the held-out report and the held-out validation-trial
  number

## Promotion Gate

A method can be promoted only when the claimed behavior satisfies the relevant
conditions below.

### Estimated-to-Truth Runtime PASS

Use this as the default PASS gate for experiments that learn a calibration-time
mapping from stereo `estimated` measurements to validation `truth`, then apply
that mapping without truth at measurement time.

Required:

- validation corrected range RMSE is lower than validation raw range RMSE
- validation corrected Position RMSE is lower than validation raw Position RMSE
- Right, Down, and Forward validation RMSE are each reported as raw and
  corrected values
- no RDF axis regresses for a normal `PASS` claim:
  `corrected_axis_RMSE <= raw_axis_RMSE`
- all RDF axes strictly improve for a `STRICT_PASS` claim:
  `corrected_axis_RMSE < raw_axis_RMSE`
- validation samples near 1852 m meet the target-window tolerance, normally:
  `abs(mean_range_error_near_1852m) <= 10 m`
- the 1852 m target-window sample count is reported; low sample count prevents
  strong PASS evidence
- train/validation gap is reported and interpreted as overfit evidence when
  validation degrades relative to train
- automated search experiments define the search space, candidate generation
  rule, selection metric, validation split strategy, and overfit control
- selected candidates from automated search are ranked by validation behavior,
  target-range behavior, RDF axis behavior, runtime feasibility, and stability
- runtime inputs are limited to measurement-time observable signals and
  calibration-fixed parameters
- selected results are either exportable as StereoDistance runtime profiles or
  explicitly marked offline-only
- the truth point and runtime feature/reference point are stated in the README
  and verdict
- NED, internal RDF, public FRD+range, and alias contracts are explicit

Suggested verdict mapping:

| Verdict | Required evidence |
| --- | --- |
| `STRICT_PASS` | Range and Position improve on validation, Right/Down/Forward all strictly improve, 1852 m target-window error is within tolerance with adequate sample count, runtime truth is not used, train/validation gap is stable, and coordinate contracts are explicit. |
| `PASS` | Range improves on validation, RDF axes do not regress, 1852 m target-window error is within tolerance with adequate sample count, runtime truth is not used, and coordinate contracts are explicit. |
| `CONDITIONAL_PASS` | Main range target is met but evidence is incomplete due to low 1852 m sample count, small axis regression, limited seed/report coverage, or unresolved train/validation gap. |
| `FAIL` | Validation is worse than raw, 1852 m target fails, runtime truth is required, coordinate contracts are ambiguous, or improvement is train-only/overfit. |

### Strict RDF All-Axis PASS

Use this as the PASS gate when the claim is "RDF all improve." This is stricter
than non-regression and replaces Forward-only PASS for all-axis claims.

Required:

- Right, Down, Forward, and Position RMSE deltas are strictly positive for every
  validation trajectory: `raw RMSE - corrected RMSE > 0`
- aggregate min and mean deltas are strictly positive for Right, Down, Forward,
  and Position
- every claimed corrected RDF axis has nonzero corrected-sample coverage; no
  axis may pass only because it is raw/no-op
- bootstrap row and trial-cluster pass fractions are `>= 0.99` when sample-level
  data is available
- if the result is profile-family-local, mark it `FAMILY_LOCAL_PASS`, not global
  `PASS`

Default strict epsilon:

```text
strict_epsilon_m = 1e-9
```

Any axis with exactly zero delta is a protected no-op or non-regression result,
not a Strict RDF PASS.

### All-Axis Correction

Required:

- Right, Down, and Forward min deltas are non-negative across all validation
  trajectories
- mean deltas are positive for all axes claimed as corrected
- no populated bootstrap subgroup has negative 2.5th percentile for a claimed
  corrected axis
- corrected sample coverage is high enough to be meaningful
- no axis is passing only because it is almost always no-op

Suggested minimums:

```text
pass_trial_count / total_trial_count = 1.0
corrected_samples / total_samples >= 0.20 for claimed corrected axes
Down mean delta > 0.005 m when claiming Down improvement
Right mean delta > 0.005 m when claiming Right improvement
Forward mean delta > 1.0 m when claiming Forward improvement
bootstrap row pass fraction >= 0.99
bootstrap trial-cluster pass fraction >= 0.99
```

### Stable Baseline

A method may be marked `STABLE_BASELINE` when:

- no claimed axis regresses under the validation gate
- at least one axis has meaningful improvement
- protected axes are explicitly documented as no-op

### Family-Local Candidate

A method may be marked `FAMILY_LOCAL_PASS` when:

- it passes inside one report/profile family
- it fails or is unproven outside that family
- the registry and verdict explicitly say it is not all-family evidence

### Stable No-Op

A method must be marked `STABLE_NOOP` rather than `PASS` when:

- deterministic gates pass mainly because correction is rarely applied
- a claimed axis has near-zero mean delta
- corrected sample coverage is too low for the claim

## Rejection Gate

Reject or pause a method when any of the following holds:

- repeated held-out report/trial regressions
- strong average improvement but negative subgroup p025
- no pass-all candidate except no-op or near-no-op
- same feature family already failed under the same gate
- physical model does not match the observation being used
- validation coverage is invalid or too narrow

Rejected methods should be listed in
`reports/experiments/rejected_methods.md` with the condition under which they
may be revisited.

## Reporting Minimum

Every completed experiment must report:

```text
report_count
trial_count
sample_count
corrected_sample_count
truth_point_defined
runtime_feature_defined
automated_search
search_space_defined
candidate_count
selection_metric_defined
validation_split_strategy
overfit_control
stereodistance_applicable
runtime_profile_exportable
runtime_profile_path
offline_only
runtime_truth_used
coordinate_contracts_explicit
alias_contracts_explicit
validation_range_rmse_raw_m
validation_range_rmse_corrected_m
validation_position_rmse_raw_m
validation_position_rmse_corrected_m
validation_right_rmse_raw_m
validation_right_rmse_corrected_m
validation_down_rmse_raw_m
validation_down_rmse_corrected_m
validation_forward_rmse_raw_m
validation_forward_rmse_corrected_m
target_1852m_sample_count
target_1852m_abs_mean_error_m
target_1852m_tolerance_m
train_validation_gap_range_rmse_m
pass_trial_count
total_trial_count
right_min_delta_m
right_mean_delta_m
down_min_delta_m
down_mean_delta_m
forward_min_delta_m
forward_mean_delta_m
position_min_delta_m
position_mean_delta_m
bootstrap_pass_fraction_rows, if available
bootstrap_pass_fraction_trials, if available
```
