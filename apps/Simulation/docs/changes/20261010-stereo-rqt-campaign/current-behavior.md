# Current Behavior

## Baseline observed before this change

- `campaigns/stereo_v1_1/campaign_run.py` provided `plan`, `run`, `all`, `resume`, and `qc`, and owned a serial headless Gazebo/PX4 lifecycle. `campaign_status.py` initialized the 39 status slots separately.
- `config/campaign/v1_1/G0.env` and `G1.env`, the reference generator, 39-run manifest, split raw capture, YOLO replay, image pairing, GT alignment, and source QC already existed in the worktree.
- There was no RQT campaign plugin, no single action reading campaign settings from `Simulation.env`, and no orchestrated environment report/live short-capture preflight.
- `generate_sensor_observations.py` produced synthetic camera RTK/IMU files. `summarize_campaign.py` still reported `measurement_estimator_status=not_implemented`; no A/B/C solver or V0 evaluation outputs existed in the checked-out source.
- The older `run_auto_attitude_sweep.sh` remained a distinct OFAT/waypoint workflow and uses its own `SWEEP_*` settings.

## Current source and external evidence

- The Git worktree already contains many unrelated modifications across Simulation, Workstation, and repository tooling. This change touches only the Simulation tree and preserves the unrelated work.
- A Desktop campaign root outside the repository currently has 39/39 `VALID_WITH_WARNINGS` slots. Fresh read-only `validate_attempt` checks through `_status_slots` passed. Seven V0 runs did not enter the science window; these remain performance outcomes. The source directory contains no estimator results, so it is used only as a read-only analysis fixture.
- No `PX4-Autopilot` checkout exists at the configured repo-relative `apps/Simulation/PX4-Autopilot` path. A machine-local exact-tag checkout exists outside this repository; the runtime permits an ephemeral `PX4_DIR` override, while the persisted `.env` remains repo-relative.
- The repo-owned `workspace/main/src/px4_msgs` is pinned to the expected commit, but its install overlay is absent. The isolated Simulation YOLO environment is also absent. The standalone launcher now builds the message overlay into the selected workspace; the RQT runtime-preparation action installs the pinned isolated analysis/YOLO dependencies.

## Preserved invariants

World positions are ENU `[east,north,up]`; PX4 control positions/velocities are local NED `[north,east,down]`; camera projection uses optical RDF `[right,down,forward]`. Physical optical poses remain selected by the frozen G0/G1 profiles. Legacy camera PX4/body FRD values remain independent. The campaign reference path stays protocol-generated and does not use `WAYPOINTS_XY_ALT_M`.

## Version and worktree context

## Reproduction

## Observed frontend behavior

## Observed backend and persistence behavior

## Documentation, code, test, and runtime conflicts
