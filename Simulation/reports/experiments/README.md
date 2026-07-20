# Experiments

> Archive notice: these records preserve pre-ray-only correction research and
> provenance. They are not active runtime features or current formal evidence.
> Any new correction study must start a fresh Calibration cycle against the
> current `baseline_center_frd` public contract.

This directory stores structured experiment records for StereoDistance
correction-method work.

## Shared Purpose

StereoDistance correction experiments exist to learn, from calibration-flight
data, a runtime-safe mapping from stereo `estimated` measurements to the
corresponding `truth` measurements. During calibration, PX4 local position NED
may be used as ground truth, but it is not the only allowed truth source. RTK or
GNSS world coordinates, Gazebo world/model pose, external tracker frames, or an
already stereo-center RDF truth topic may also be used when their source frame,
origin/reference point, units, time basis, target truth point, and transform into
stereo-center camera RDF are explicit. During measurement-only operation, truth
is not available and must not be used as a runtime input.

The common objective is to remove the repeatable global gap between `estimated`
and `truth`, including camera attitude error, origin/reference-point offset, and
the systematic gap between estimates derived from target image features and the
truth point being measured. The image feature does not have to be the bounding
box center; each experiment must state which runtime-observable feature or
reference point is used and what physical point it is intended to estimate.

The broader objective is automated search over many plausible correction
families, feature choices, parameterizations, guards, and validation splits.
Search results are not useful unless the selected correction can be exported as
a calibration-fixed profile and applied inside StereoDistance at measurement
time without truth. Each search experiment must therefore record its search
space, selection metric, rejected families or parameter regions, and the exact
runtime contract that would let StereoDistance apply the result.

Every experiment must keep these contracts explicit:

- PX4 local position uses `north/east/down`.
- Stereo correction math and learned profiles use internal RDF:
  `right/down/forward`.
- Validation is performed in stereo-center RDF by comparing `corrected_rdf`
  against `truth_rdf`.
- `truth_rdf` must be produced by a declared `T_source_to_stereo_rdf` transform or
  by a truth producer that is already in the stereo-center RDF frame.
- Do not assume the camera optical frame is aligned with local NED. The direct
  alias `rdf_right=east`, `rdf_down=down`, `rdf_forward=north` is forbidden for
  both real and synthetic experiments.
- Legacy StereoDistance ROS `Float32MultiArray` position topics use public
  FRD+range order: `forward/right/down/range`.
- CSV/report data should prefer named fields such as `estimated_right_m`,
  `estimated_down_m`, and `estimated_forward_m`; unnamed arrays must state
  whether they are internal RDF or public FRD+range.
- Calibration-time truth is allowed only for fitting, selection, and validation.
- Measurement-time correction must use only runtime-observable inputs and
  parameters fixed from calibration.
- Automated search must rank candidates by validation behavior, not by train
  fit alone.
- Any selected result must be representable as a StereoDistance runtime profile
  or explicitly marked as offline-only.
- Final claims should be based on validation performance, target-range error,
  axis-wise RDF behavior, and stability against overfit.

## PASS Conditions

PASS claims are validation claims. A correction is not PASS merely because it
fits calibration/train rows. It must make measurement-time `estimated` values
closer to `truth` on held-out validation rows while using only runtime-available
inputs and calibration-fixed parameters.

Required checks:

- Validation range RMSE must improve over raw estimated range RMSE.
- Validation position RMSE should improve over raw estimated position RMSE.
- RDF axis behavior must be reported separately for Right, Down, and Forward.
- Near the target range of 1852 m, validation range error must satisfy the
  experiment's target-window criterion, normally absolute mean error within
  10 m. If sample count in that window is low, the result is not a strong PASS.
- Train/validation gap must be reported. Large validation degradation relative
  to train is overfit evidence.
- Runtime correction must not require calibration-time truth, PX4/RTK truth,
  true disparity, or true image feature locations.
- The report must state the truth point and the runtime-observable feature or
  reference point used to estimate it.
- NED, internal RDF, and public StereoDistance FRD+range fields must not be
  mixed. Any alias field or unnamed array must have an explicit axis contract.

Use these verdict levels:

| Verdict | Meaning |
| --- | --- |
| `STRICT_PASS` | Validation range and position RMSE improve, Right/Down/Forward all improve, 1852 m target-window error is within tolerance, runtime truth is not used, train/validation gap is stable, and coordinate contracts are explicit. |
| `PASS` | Validation range RMSE improves, RDF axes do not regress, 1852 m target-window error is within tolerance, runtime truth is not used, and coordinate contracts are explicit. |
| `CONDITIONAL_PASS` | Main range objective is met, but evidence is incomplete: low 1852 m sample count, small axis regression, limited seed coverage, or train/validation gap needing more validation. |
| `FAIL` | Validation is worse than raw, the 1852 m target fails, any required RDF axis regresses under a strict all-axis claim, runtime truth is required, coordinate contracts are ambiguous, or improvement is train-only/overfit. |

For any strict claim that all RDF axes improve, the corrected RMSE must be
strictly lower than raw RMSE for Right, Down, Forward, and Position on
validation data. Zero-delta no-op axes are not strict PASS evidence.

New experiments must follow:

- `docs/experiment-workflow.md`
- `docs/experiment-gates.md`
- `docs/method-taxonomy.md`

For any claim that "RDF all improve," use the Strict RDF All-Axis PASS gate in
`docs/experiment-gates.md`; zero-delta no-op axes are not PASS evidence.

## Quick Start

1. Pick the next experiment ID:

   ```text
   EXP-YYYYMMDD-NNN-method-slug
   ```

2. Create a directory:

   ```text
   reports/experiments/EXP-YYYYMMDD-NNN-method-slug/
   ```

3. Copy the templates from `reports/experiments/templates/`.
4. Fill `README.md` before running commands.
5. Run the experiment.
6. Fill `metrics.json`.
7. For estimated-to-truth correction experiments, run:

   ```bash
   Simulation/.venv/bin/python Simulation/scripts/experiment_pass_gate.py \
     reports/experiments/EXP-YYYYMMDD-NNN-method-slug/metrics.json --write
   ```

8. Fill `verdict.md`, `references.md`, and `artifacts.txt`.
9. Add or update one row in `experiment_registry.csv`.

## Required Files

| File | Purpose |
| --- | --- |
| `README.md` | Human-readable plan, hypothesis, method, gate, and failure modes. |
| `command.md` | Exact command lines and environment assumptions. |
| `metrics.json` | Machine-readable metrics matching the project schema. |
| `verdict.md` | Final result, status, failure tags, and next decision. |
| `references.md` | External or internal rationale and source quality. |
| `artifacts.txt` | Paths to report directories, CSVs, JSONs, logs, and plots. |

## Registry

`experiment_registry.csv` is the compact comparison surface. It should contain
one row per meaningful experiment or legacy evidence bundle.

Do not use the registry as the only source of truth. It is a ranking index that
must point to the detailed experiment directory or legacy artifacts.

## Legacy Results

Existing report directories under `reports/` are not moved. When legacy results
matter, add a registry row with `EXP-LEGACY-...` and list the original paths in
`legacy_artifacts`.

## 2026-06-18 Fresh StereoDistance Residual Search

`EXP-20260618-281-fresh-stereodistance-disparity-residual-search` reran a
Simulation-local search for a StereoDistance-applicable correction method after
the result directories were found missing from the current checkout. The fresh
run used `Simulation/.venv/bin/python`, 4 synthetic stress scenarios, 5 fresh
seeds per scenario, 600 samples per seed, and 64 candidate residual settings.

The selected method is
`multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_ray_scale_log_disparity_deg1_ridge1_clip0p5`.
It applies a ray-scale residual from fused `log_disparity` after 7x3
multi-burst observation fusion. Aggregate validation is `PASS`: 4/4 scenarios
strict pass, observation-only target worst `5.091243m`, residual target worst
`2.437792m`, mean range RMSE `1.417889m -> 0.405203m`, runtime truth `false`.

The same sweep continued beyond the first PASS and rejected the weaker
`forward_add` branch for this gate. A representative `forward_add log_disparity
degree2 ridge1 clip0.5` candidate passed only 3/4 scenarios because
`intrinsic_scale_stress` passed 2/5 trials. The failure mode is consistent with
absolute forward-meter correction being less stable than scaling the whole
stereo ray under intrinsic/scale-like error.
