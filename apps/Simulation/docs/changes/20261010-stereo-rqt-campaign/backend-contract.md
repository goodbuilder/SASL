# Backend Contract

## Control settings

`control_config.py` reads only the literal campaign action, run/output IDs, output base, MC count, repo-relative PX4/analysis interpreter paths, and YOLO device/backend fields from `config/simulation.env`. It does not source shell text. Output IDs are path-free; persisted PX4 and analysis-runtime paths must resolve inside the repository. A caller `PX4_DIR` override or an auto-discovered adjacent exact v1.17.0 checkout is transient for the process and is recorded as machine-local provenance rather than serialized as a reusable asset path.

## Runner and state

The campaign CLI owns serial Gazebo/PX4 process startup. Preflight blocks on the wrong PX4 tag/binary, missing ROS/message/runtime/model assets, existing SITL, camera transform mismatch, missing image topics, GT rate/gaps, or storage below the measured campaign estimate. It captures G0 and G1 no-flight samples to validate the actual Gazebo optical pose and lossless storage rate. The full all-results action requires capacity for pending raw runs plus 10 GB headroom.

Each slot/attempt is append-only. `run_status.json`, `manifest_actual.json`, `changes_log.jsonl`, attempt checksums and fresh QC explain retries and interruptions. Stop uses SIGINT; performance failure stays valid when raw data pass QC.

## Estimator data allowlist

The solver accepts camera optical-center RTK/IMU observations, per-pair original-pixel detections, the declared yaw initialization and RTK observations only for A/B. It never receives `ground_truth`, camera truth, or the commanded reference. A uses independent camera fits with the drone RTK covariance propagated into image residual covariance; B has shared per-pair latent ENU points and RTK residuals; C has shared image-only latent points. Camera centers are fixed in every method. Attitude covariance, rank, condition, convergence and latent degrees are recorded.

The analysis producer reads GT only to generate explicitly synthetic sensor observations and in the post-freeze evaluator. It writes a path-selection file from G0 covariance propagation, hashes code/protocol/settings in `analysis_freeze.json`, and then evaluates fixed calibration rotations on the corresponding V0. A repeated analysis with the same frozen hash is idempotent; a code/settings change creates a new analysis version and never overwrites earlier results.

## Commands and validation

## State, ownership, and concurrency

## Worker resource budgets, scheduling, and backpressure

## External boundaries and workers

## Results, errors, timeout, retry, and cancellation

## Persistence, restart, and shutdown
