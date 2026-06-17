# Experiment Workflow

This document is the operating contract for StereoDistance correction-method
experiments in this workspace. The goal is to make every attempt reproducible,
rankable, and understandable even when it fails.

## Scope

Use this workflow for any new method that changes, selects, gates, fuses, or
validates StereoDistance correction behavior, including bbox-anchor,
Origin-BBOX, ROI/silhouette, extrinsic, tracking, residual, and range-fusion
branches.

Legacy reports already under `reports/` may stay where they are. New method
attempts must create an experiment record under `reports/experiments/`.

## Non-Negotiable Rules

1. Create an experiment ID before running a new method.
2. Write the experiment README before execution.
3. Record the command, inputs, hypothesis, method, rationale, reference level,
   evidence level, acceptance gate, and expected failure mode.
4. Record the truth point, runtime-observable feature/reference point,
   calibration-time truth use, measurement-time runtime inputs, and coordinate
   contracts before execution. Distinguish PX4 NED, internal correction RDF,
   and any public StereoDistance FRD+range topic arrays.
5. Write metrics and verdict after execution, even for failures.
6. For estimated-to-truth correction claims, run or manually satisfy the
   Estimated-to-Truth Runtime PASS gate in `docs/experiment-gates.md`.
7. Update `reports/experiments/experiment_registry.csv` after every completed,
   blocked, rejected, or promoted experiment.
8. Do not claim a method is better unless the registry row links the evidence.
9. Do not overwrite previous experiment directories. Create a new experiment
   ID for reruns with changed inputs, thresholds, gates, or code.

## Experiment ID

Format:

```text
EXP-YYYYMMDD-NNN-short-method-slug
```

Example:

```text
EXP-20260611-002-all-family-micro-down-guard
```

Rules:

- `YYYYMMDD` is the local workspace date.
- `NNN` is monotonically increasing for that date.
- The slug should name the method, not the result.
- Legacy records may use `EXP-LEGACY-YYYYMMDD-NNN-slug`.

## Directory Layout

Each experiment directory must follow this shape:

```text
reports/experiments/EXP-YYYYMMDD-NNN-method/
  README.md
  command.md
  metrics.json
  verdict.md
  references.md
  artifacts.txt
```

Use `reports/experiments/templates/` as the starting point.

## Lifecycle

Use one lifecycle stage at a time:

| Stage | Meaning |
| --- | --- |
| `idea` | Candidate has not been planned enough to run. |
| `planned` | README is written; command may not have run yet. |
| `exploration` | Fast, possibly narrow check to reject weak ideas quickly. |
| `validation` | Stricter rerun on held-out reports, new seeds, or stronger gates. |
| `promotion` | Candidate is being prepared for runtime integration. |
| `complete` | Experiment has final metrics and verdict. |
| `blocked` | Missing authority, data, runtime, or external dependency. |

## Status Values

Use one primary status in the registry:

| Status | Meaning |
| --- | --- |
| `STRICT_PASS` | Meets the estimated-to-truth gate with validation range/position improvement, all RDF axes strictly improved, target-range tolerance satisfied, runtime truth excluded, stable train/validation gap, and explicit coordinate contracts. |
| `PASS` | Meets the stated acceptance gate and is meaningful for the claimed axis/method. |
| `CONDITIONAL_PASS` | Main validation target is met, but evidence is incomplete because of low target-range sample count, small axis regression, limited seed coverage, or unresolved train/validation gap. |
| `FAIL` | Fails the required gate because validation is worse than raw, target-range tolerance fails, runtime truth is required, coordinate contracts are ambiguous, or improvement is train-only/overfit. |
| `NEAR_PASS` | Small residual failure; worth a targeted follow-up. |
| `PARTIAL_PASS` | Improves some axes or subsets but not the full claim. |
| `FAMILY_LOCAL_PASS` | Works only inside one report/profile family. |
| `STABLE_BASELINE` | Reliable fallback baseline, not necessarily a full solution. |
| `STABLE_NOOP` | Passes because protected axes are mostly no-op. |
| `UNDER_EVIDENCED` | Looks promising but lacks enough reports/seeds/coverage. |
| `ALL_FAMILY_FAIL` | Fails when replayed across broader report families. |
| `REGRESSION_SMALL` | Fails with small axis regression. |
| `REGRESSION_LARGE` | Fails with large or repeated axis regression. |
| `INVALID_TEST` | Result cannot be used because the test setup was invalid. |
| `BLOCKED` | Could not run for non-method reasons. |
| `REJECTED` | Should not be rerun without a material change. |
| `PROMOTED` | Approved for runtime integration or next-stage implementation. |

## Evidence Levels

| Level | Meaning |
| --- | --- |
| `L0` | Heuristic or intuition only. |
| `L1` | Supported by previous internal experiment results. |
| `L2` | Supported by stereo geometry, calibration, or measurement-model reasoning. |
| `L3` | Supported by external papers, standards, or authoritative references. |
| `L4` | Supported by multi-seed validation under the project acceptance gate. |

## Reference Levels

| Level | Meaning |
| --- | --- |
| `R0` | No reference; exploratory only. |
| `R1` | Internal prior result or local implementation precedent. |
| `R2` | Standard geometry/calibration textbook or accepted engineering principle. |
| `R3` | Peer-reviewed or primary external source. |
| `R4` | Multiple independent references plus local validation. |

## Failure Taxonomy

Use one or more tags in `verdict.md` and the registry notes:

```text
geometry_mismatch
overfit_to_report
overfit_to_trial
family_local_only
weak_down_signal
near_noop
range_bin_instability
lateral_regression
down_regression
forward_regression
forward_only_improvement
insufficient_coverage
insufficient_seeds
bootstrap_unstable
invalid_validation
runtime_unavailable
runtime_truth_leakage
truth_point_undefined
runtime_feature_undefined
coordinate_contract_ambiguous
target_range_under_sampled
target_range_error_fail
train_validation_gap
```

## Baseline Policy

Every correction experiment should compare against:

1. raw stereo estimate
2. `range_only_forward_fusion`
3. current bbox-anchor all-family axis guard
4. current best family-local Down candidate, when the method claims Down
5. current ROI/silhouette branch, when the method uses image observations

## Stop Conditions

Stop an experiment branch when:

- the acceptance gate passes and promotion criteria are satisfied;
- a stronger gate reveals a clear regression;
- the method becomes no-op-dominated;
- evidence shows the hypothesis is wrong;
- the runtime is blocked and no offline substitute can answer the question.

When stopping, write `verdict.md` and update the registry before starting the
next method.
