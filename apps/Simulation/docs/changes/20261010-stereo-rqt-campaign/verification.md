# Verification

## Requirement-to-evidence matrix

| Requirement | Evidence | Result | Remaining check |
|---|---|---|---|
| Edit the `STEREO_V1_1_*` block in `config/simulation.env`; execute without an LLM | Strict env parser, QtTest button click completing a 100-trial `reprocess` on a staged source root, and a visible RQT readback of the direct source/output-root configuration | PASS for config readback and button-to-analysis wiring; defaults remain `plan` and 100 Monte Carlo trials | Full 100-trial execution has not been repeated with the new direct input-root setting; native click, focus, and DPI/resize checks also remain |
| Preserve the legacy 20-case attitude sweep | `bash run_auto_attitude_sweep.sh --plan-only` | PASS; 20 cases planned, no flight started | None for plan compatibility |
| Validate the existing 39-slot source data | Fresh `_status_slots()` and `qc_attempt.validate()` on the external campaign root | PASS: 39/39 source slots are `VALID_WITH_WARNINGS`; seven V0 science-entry failures remain performance outcomes | No new flight was added because the v1.1 data change specifies zero additional Gazebo runs |
| Enforce frames and physical camera orientation | `test_stereo_v1_1_camera_frame_contract.py`; converted the recorded SDF camera-link FLU quaternion to optical RDF for all 39 runs/78 cameras | PASS: maximum R_WC error 0 degrees against the frozen G0/G1 protocol | None for recorded pose conversion |
| Fit A/B/C × RPY_IMU/YAW_ONLY and keep YAW_IDEAL separate | Actual-data 100-trial analysis; 758 calibration batches and 6,822 primary/diagnostic results | PASS: method/variant convergence and observability are retained; exact-RTK YAW_IDEAL is separately labeled | C and V0 outcomes remain dataset-specific, not hardware claims |
| Compare all-data and matched-200 data without duplicating frames | 6,732 evaluation rows across the 100-trial analysis | PASS: all-data and `MATCHED_200_UNMET` statuses are retained without frame replication or source-run removal | 3,789 rows report no valid position estimate; retained as outcomes |
| Freeze before V0 metrics; evaluate matching V0 only | `analysis_freeze.json`, its source/code hashes, and 6,732 result rows | PASS: V0 was read after freeze; `v0_recalibration_performed=false` | None for the verified analysis version |
| Run one-factor error sensitivity on P0 and selected path | 25 protocol-derived scenarios × 2 G0 paths = 50 bundles and 300 method/variant rows | PASS: actual-YOLO and separate ideal-projection branches, timestamp re-pairing, and comparable-sample RMSE direction are recorded | One deterministic trial per sensitivity case, as documented |
| Preserve result integrity and provide a handoff | 36,071 entries in `analysis_checksums.sha256`; `analysis_latest.json` points to the frozen version | PASS: zero checksum failures; selected G0 path is P1 | Result tree is under `/tmp`; raw source remains read-only |
| Reprocess a source campaign without writing into it | `STEREO_V1_1_INPUT_ROOT` supports `output`, `repo`, `data_root`, or a SASL-root-relative path; output root is independently selected; direct-root RQT `reprocess` completed a one-trial analysis | PASS: 39/39 source slots QC-valid; direct-root result wrote 3,258 calibration results, 2,376 V0 rows, and 12,905 verified checksums to `/tmp/sasl-direct-root-input-test-20261010` | Full 100-trial button run used a temporary staging root with a read-only source link; separate-root 100-trial run was not repeated |
| Keep persisted paths portable | Simulation-scoped audit across 1,997 paths and repository-wide `tools/check_portable_paths.py` | PASS: zero violations in both scopes |

## Focused test evidence

`/usr/bin/python3 -m pytest -q tests/test_stereo_v1_1_*.py tests/test_v1_1_camera_pose_profiles.py` passed: 81 tests. The system interpreter emitted its existing SciPy/NumPy compatibility warning; the isolated analysis environment uses pinned SciPy 1.15.3.

## Regression and application evidence

`bash -n` passed for the RQT and runtime setup scripts. The Simulation colcon build completed for `simulation_interfaces`, `stereo_ground_sim`, and `stereo_ground_sim_overlay`. The repo-pinned `px4_msgs` source at commit `86d8239e962f6939e05c3737784f60c02fa884db` built and imports through `workspace/main/install/setup.bash`. The isolated YOLO runtime verified PyTorch `2.12.0+cu132`, torchvision `0.27.0+cu132`, Ultralytics `8.4.127`, CUDA, and the RTX 4050 Laptop GPU. The legacy sweep still plans 20 cases independently. Both the Simulation-scoped audit and repository-wide portable-path checker pass. No Workstation changes were made for this task.

## Performance, load, and target-hardware evidence

The actual-data 100-trial analysis completed in a temporary verification root. It wrote 6,822 primary/diagnostic calibration results, 6,732 V0 evaluation rows, and 300 sensitivity rows. The sensitivity report records 73 same-sample improvements, 114 regressions, 28 unchanged cases, and 85 incomparable cases (49 unavailable metrics plus 36 branch-incomparable cases). The right-camera timestamp offsets produced zero pairs at the protocol 1 ms tolerance. After adding the direct input/output split, a separate one-trial reprocess read the original campaign root directly and wrote only under `/tmp/sasl-direct-root-input-test-20261010`; its run manifest records a source identity hash without persisting the source absolute path. These are simulation/reprocessing outcomes, not hardware claims.

## Feature isolation and contract evidence

Campaign actions dispatch through the literal `.env` control parser. `reprocess` accepts a source root separately from the output root; the RQT panel displays both and reports source slot status. The legacy sweep remains a separate action. Camera RTK/IMU inputs are shared by calibration path and matching V0; method C receives no drone RTK; YAW_IDEAL uses exact synthetic drone RTK only in its diagnostic branch. The analyzer writes versioned outputs under `analysis_versions/` and does not write to a separately selected source root.

## Visual authenticity and dynamic-layout stability evidence

The actual standalone RQT window launched from `run_stereo_campaign_rqt.sh`. A desktop screenshot at `/tmp/sasl-rqt-input-root-status.png` shows `reprocess`, the external 39-run input root, a separate repo output, 100 trials, and source status `valid=39, failed=0`. QtTest clicked the panel's **Run configured action** button for a 100-trial reprocess on the staging root and then for a one-trial direct-root reprocess; both child CLIs returned `ANALYSIS_COMPLETE` with exit 0. Dynamic resize/focus behavior and a native mouse click on the visible window were not exercised.

## Visual and hardware/manual evidence

The fresh non-flying preflight passed with the pinned ROS overlay, isolated CUDA runtime, model checksum, and exact PX4 v1.17.0 binary. G0/G1 five-second probes each observed about 100 left and 101 right frames at 50 Hz GT with zero 40 ms GT gaps and 0-degree camera-orientation error. The measured capture estimate projects 118,723,309 bytes for 39 runs plus a 10 GB reserve; the selected repo output filesystem had 242,510,864,384 bytes free. Preflight cleanup reported no active SITL processes. No additional development flight was run because the valid existing 39-run source is reused and the v1.1 change specifies zero additional flights.

## Residual conditions

- `STEREO_V1_1_MC_TRIALS` remains 100 in `config/simulation.env`. The 100-trial button run used a temporary campaign root with a read-only link to the existing source; the explicit separate-input-root setting has a complete one-trial RQT run, direct 39/39 QC, UI readback, and unit-test evidence.
- The existing 39-run source root is outside the Simulation workspace and was read-only. Test-derived files and analysis versions were written to `/tmp`.
- No new 39-slot SITL collection was run because the existing source passed all 39 QC checks and v1.1 requires no additional Gazebo flights. Seven V0 science-entry failures and matched-200 unmet cases remain recorded as outcomes.
- `apps/Simulation/README.md` was updated after the user selected that README as the documentation target.

Coordinate contract checked: `campaigns/stereo_v1_1/{camera_frame_contract.py,calibration_solver.py,analyze_campaign.py,preflight_capture.py,trajectory_contract.py}`; world ENU `[east,north,up]`, PX4 local NED `[north,east,down]`, Gazebo camera-link FLU `[forward,left,up]`, optical RDF `[right,down,forward]`; 78 recorded camera-pose checks and `test_stereo_v1_1_camera_frame_contract.py`.
