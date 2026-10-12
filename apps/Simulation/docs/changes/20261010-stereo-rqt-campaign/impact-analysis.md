# Impact Analysis

| Layer | Owner / change | Preserved invariant | Evidence / risk |
|---|---|---|---|
| Configuration | `control_config.py` reads action, run/output IDs, output base, separate input-root setting, MC count, PX4/YOLO paths, and device settings. | Scientific conditions remain frozen in protocol JSON and G0/G1 profiles. | Unit tests cover separate roots; the RQT panel read back the actual external source root and separate repo output. |
| RQT | `CampaignPanel` owns one controller `QProcess`, bounded log, source/campaign status, and output/input display. | The existing OperatorPanel and RecorderPanel controls remain registered and unchanged. | Visible RQT showed 39 valid source slots; QtTest clicked Run configured action for a 100-trial `reprocess` and received `ANALYSIS_COMPLETE`. Native mouse/focus/resize checks remain open. |
| Controller | `control_cli.py` sequences preflight, one campaign runner, resumable analysis from the campaign output, reprocess from a separately selected input root, or the explicit legacy sweep action. | At most one SITL stack; serial slots; immutable attempts; no automatic performance retries. | Live preflight and no-flight G0/G1 probes pass; the existing valid source was reprocessed without starting a campaign flight. |
| External runtime | PX4 SITL, Gazebo, ROS 2, camera image bags, GT and YOLO runtime. | Only PX4 v1.17.0; no hardware target. | Pinned message overlay, Simulation build, isolated CUDA YOLO runtime, and fresh static/live preflight pass. CLI finds a neighboring exact-tag SITL build only as transient local provenance. |
| Persistence | Campaign root stores environment report, frozen protocol, actual/status manifests, schema map, hashes, raw attempts, per-trial observations and frozen analysis versions. | Original Desktop campaign raw attempts remain read-only and outside the output tree. | Actual-data 100-trial analysis was generated under `/tmp`; 36,071 checksums verified. |
| Calibration | `calibration_solver.py` implements A/B/C and RPY_IMU/YAW_ONLY, with separate YAW_IDEAL diagnostic inputs. | Fixed camera centers; ENU/RDF contracts; no GT/RTK in C; finite IMU prior; no yaw penalty. | 6,822 calibration/diagnostic results written across 758 batches; all method inputs and convergence states persisted. |
| Final evaluation | `analyze_campaign.py` creates G0 covariance path ranking and freeze before V0 error read. | V0 never trains/recalibrates cameras; no invented RMSE acceptance threshold. | 6,732 post-freeze V0 rows and 50 sensitivity cases generated from the existing 39-run data. |
| Legacy sweep | Existing `run_auto_attitude_sweep.sh` is still callable separately. | Its `SWEEP_*` settings are not converted into v1.1 campaign conditions. | 20-case `--plan-only` passed without starting a case. |

No new CPU/GPU worker pool is created. The calibrator is serial and bounded per run; 100 MC is explicit in `Simulation.env`. A progress log is emitted for each calibration batch. The YOLO runtime device is read from the existing `YOLO_DEVICE` setting and checked during preflight; a CPU backend can be selected without changing estimator geometry. The 100-trial profile completed through the RQT button. The complete `all_results` flight collection was not rerun because an existing 39-slot source already passed QC and v1.1 adds no Gazebo flights.

## Preserved invariants

## Affected layers and components

## Feature dependency, ownership, and coupling changes

## Worker CPU/GPU resource impact

## Compatibility and migration

## Rollback and recovery

## Risks and assumptions
