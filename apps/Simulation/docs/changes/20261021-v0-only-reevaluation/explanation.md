# V0-only reevaluation

## Requested outcome

Evaluate a new V0 recording against an already frozen 100-trial calibration analysis. Keep the original analysis intact, create an isolated output directory, reuse only calibration artifacts that match the V0 rig and repetition, and do not run calibration Monte Carlo or sensitivity calibration again.

## Evidence and scope

- Parent analysis: `analysis_9cfd7c6cd10b`, frozen before its original V0 evaluation, with 100 MC trials and selected calibration path P2.
- The selected N=600 recording is `DEV-V0-SPAWN-N600/a01`, linked to source slot `G0-V0-R02`, and is explicitly a development engineering pilot outside the 39-slot campaign.
- The pilot retains the original V0 science reference, while its spawn changes from world ENU `[0,415,6] m` to `[0,600,1.05] m` and it uses the 10 m vertical-liftoff entry gate. Record the result as this combined development condition; do not describe it as an isolated N-only change.
- The parent analysis stores 8,000 calibration batches for G0 repetition 2: five P routes × 100 trials × eight yaw cases × two sample scopes. These are the only calibration batches needed for this V0 evaluation.

## Implementation choice

Add a separate V0 reevaluation runner rather than editing `analyze_campaign.py`. The parent freeze hashes that module, so changing it would invalidate the frozen calibration identity. The runner verifies the parent freeze, algorithm hashes, dependencies, source calibration checksums, and per-case trial configurations; copies the matched calibration result bundles and shared camera observations; preprocesses the pilot's lossless bags with the pinned detector; then calls the existing post-freeze V0 evaluator. It writes a new sibling folder and leaves the parent analysis and source pilot unchanged.

## Method contract

- Method: `FROZEN_CALIBRATION_V0_REEVALUATION_V1`.
- Purpose: recompute V0 position evaluation for a new recorded V0 attempt without refitting cameras or rerunning calibration MC100.
- Inputs: one QC-checked development V0 attempt; identical camera optical geometry and shared trial sensor draws; the original frozen G0-P0..P4-R02 calibration results; the frozen analysis algorithms and dependency versions.
- Output: per-batch V0 evaluation results, a 100-trial summary, CSV metrics, source hashes, and a clear development-only status.
- Pass criteria: exactly 100 existing trial indices are paired; every expected calibration batch for the selected group/repetition is present and hash-verified; the new V0 camera sensor realization matches the saved calibration sensor observation for each trial; no calibration or sensitivity solver is invoked; parent files remain unchanged.
- Fail criteria: any parent freeze/hash/dependency mismatch, changed calibration source, invalid or incomplete raw V0 evidence, changed camera geometry, missing trial/batch, or a V0 evaluation result produced under different source hashes.

## Coordinate contract

Vehicle ground truth and synthetic RTK use Gazebo world ENU `[east,north,up]` in metres. PX4 local position is NED `[north,east,down]`; it is not copied into the world-position fields. Camera rays use optical RDF `[right,down,forward]`, and image observations are pixel `[u,v]`. The reevaluation uses the existing explicit conversions and estimator contracts.

## Architecture refresh

`NOT_REQUESTED` — medium risk; affected areas are analysis ownership, frozen-result reuse, and derived-evidence storage. No architecture reader refresh was requested. Suggested command: `$archify` for the Simulation analysis artifact lifecycle.
