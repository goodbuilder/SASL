# Experiment Workflow

This document is the operating contract for StereoDistance correction-method
experiments in this workspace. The goal is to make every attempt reproducible,
rankable, and understandable even when it fails.

## Scope

Use this workflow for any active method that changes, selects, gates, fuses, or
validates StereoDistance correction behavior, including bbox-anchor,
extrinsic, residual, and range-fusion branches.

Legacy reports already under `reports/` may stay where they are. New method
attempts must create an experiment record under `reports/experiments/`.

The former direct-Gazebo Origin-BBOX, ROI/silhouette, and Kalman tracking
search tools were retired on 2026-07-20. Their method names and metrics may
appear in archived reports, but they are not current baselines or executable
workflow steps.

## Non-Negotiable Rules

1. Create an experiment ID before running a new method.
2. Write the experiment README before execution.
3. Record the command, inputs, hypothesis, method, rationale, reference level,
   evidence level, acceptance gate, and expected failure mode.
4. Record the truth point, runtime-observable feature/reference point,
   calibration-time truth use, measurement-time runtime inputs, and coordinate
   contracts before execution. Distinguish PX4 NED, internal correction RDF,
   and any public StereoDistance FRD+range topic arrays.
   Evaluation is in `stereo_center_RDF`: compare `corrected_rdf` with
   `truth_rdf`. `truth_rdf` must come from an explicit
   `T_source_to_stereo_rdf` transform or an already stereo-frame truth producer.
   The source may be PX4 local NED, RTK/GNSS world coordinates, Gazebo
   world/model pose, an external tracker, or a stereo-frame truth topic, but the
   experiment must state the source frame, origin/reference point, transform,
   units, time basis, and target truth point.
   Do not use a direct axis alias such as `rdf_right=east`,
   `rdf_down=down`, `rdf_forward=north`; this is forbidden for real data and
   for synthetic experiments.
5. Record scenario metadata before execution: scenario name, target class,
   target size, range band or target range, stereo baseline, camera height,
   zoom/FOV, image size, intrinsics basis, and injected attitude/noise/outlier
   conditions.
6. Keep fixed scenario/session parameters in metadata or the experiment
   README, and keep time-varying observations in timestamped artifacts. Runtime
   rows must include the active scenario/profile/intrinsics identifiers needed
   to reproduce a mid-run setting change.
7. Write metrics and verdict after execution, even for failures.
8. For estimated-to-truth correction claims, run or manually satisfy the
   Estimated-to-Truth Runtime PASS gate in `docs/experiment-gates.md`.
9. Update `reports/experiments/experiment_registry.csv` after every completed,
   blocked, rejected, or promoted experiment.
10. Do not claim a method is better unless the registry row links the evidence.
11. Do not overwrite previous experiment directories. Create a new experiment
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

## Storage Contract

Experiment records use the same split as StereoDistance session storage:

- Fixed scenario/session parameters belong in `README.md`, `metrics.json`, or a
  profile/deployment manifest. This includes scenario name, target class/size,
  baseline, camera height, zoom/FOV, image size, intrinsics basis, validated
  envelope, truth source, time basis, units, and coordinate/axis contracts.
- Time-varying observations belong in timestamped artifacts listed from
  `artifacts.txt`. This includes raw and corrected StereoDistance RDF/FRD
  samples, truth N/E/D samples, bbox geometry/confidence, disparity/range,
  frame timestamps, latency/sample age, outlier flags, and active profile or
  intrinsics IDs.
- PX4 truth may be recorded as local NED, global position, or GPS position.
  Local NED is the primary calibration/evaluation source. GPS/global
  `lat/lon/alt` must be preserved as raw source data and converted to local NED
  with an explicit origin before any comparison. Live GPS/global conversion in
  StereoDistance is an operational monitor; final calibration/PASS evidence must
  be regenerated offline from saved rows or ULog with the selected origin, time
  basis, and `T_source_to_stereo_rdf` or alignment profile.
- When a Simulation experiment is intended to compare truth-source choices, use
  one shared measurement run and record all three truth-source contracts:
  `local_ned`, `global_position`, and `gps_position`. Build three separate
  profiles from the same measurement rows, one per selected truth source. The
  profile training step must use only that selected source; it must not use the
  Gazebo oracle.
- Gazebo true world/model pose is the independent oracle for Simulation
  evaluation. Convert the Gazebo target truth point to stereo-center RDF/FRD
  with an explicit transform, then evaluate all three profiles against that same
  oracle on the same validation trajectory. Record both profile error versus
  oracle and source-to-oracle error so truth-source noise is separated from
  correction-method error.
- Parameters that may change during a run must be represented both ways: the
  allowed contract or initial value in metadata, and the active value or stable
  identifier on each affected row.

## Reference Policy

Use references to justify the method design, the coordinate contract, and the
runtime applicability claim. A reference can motivate an experiment, but it does
not make a method pass; PASS status requires local validation under the project
gates.

Reference sources are prioritized as follows:

- Internal project evidence first for behavior claims: prior EXP records,
  `metrics.json`, `verdict.md`, `promotion_summary.txt`, replay outputs, saved
  profiles, ULog/session rows, and tests.
- Geometry and calibration theory for model assumptions: stereo
  triangulation, reprojection error, bundle adjustment, Sim3, residual models,
  temporal filtering, and robust statistics.
- Official coordinate and telemetry documents for frame contracts: PX4,
  MAVLink, ROS REP-103/REP-105, ROS message definitions, ULog field producers,
  and producer code. Blog posts or informal examples are not enough to resolve
  NED/ENU/FRD/RDF ambiguity.
- Hardware and simulation documents for scenario parameters: WIRIS camera
  specs/SDK, lens/FOV/zoom/timestamp behavior, Pixhawk/GPS/RTK/RM3100
  datasheets, Gazebo/Ignition model definitions, calibration JSON, and measured
  field logs.

Use these reference levels:

| Level | Meaning |
| --- | --- |
| `R0` | No reference; exploratory only. |
| `R1` | Internal prior result or local implementation precedent. |
| `R2` | Standard geometry, calibration, filtering, or measurement-model principle. |
| `R3` | Official documentation, producer code, primary dataset/schema, or peer-reviewed paper. |
| `R4` | Multiple independent references plus local validation evidence. |

Each experiment's `references.md` must state what each reference supports and
what it does not prove. In particular, coordinate/frame references must name the
producer, field names, axis order, units, and whether the reference applies to
position, orientation, image pixels, or public topic arrays.

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
| `PASS_CROSS_GEOMETRY` | Validated across multiple scenario families, with scenario envelope metadata and safe no-op/profile selection outside validated conditions. |

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
5. any newly implemented image-observation baseline declared by the experiment;
   the retired ROI/silhouette branch is not an active baseline

## Stop Conditions

Stop an experiment branch when:

- the acceptance gate passes and promotion criteria are satisfied;
- a stronger gate reveals a clear regression;
- the method becomes no-op-dominated;
- evidence shows the hypothesis is wrong;
- the runtime is blocked and no offline substitute can answer the question.

When stopping, write `verdict.md` and update the registry before starting the
next method.
