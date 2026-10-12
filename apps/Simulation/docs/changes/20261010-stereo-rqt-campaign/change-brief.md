# Change Brief

## Operator outcome

A person edits the campaign action and run/output settings in `config/simulation.env`, opens the standalone Stereo Campaign RQT perspective, refreshes the file, and starts the configured operation. The operation must either preserve/resume a v1.1 run or create all 39 source runs, QC them, generate the allowlisted synthetic sensor inputs, fit A/B/C × RPY_IMU/YAW_ONLY, freeze path selection before V0 errors, evaluate V0 once, and leave checksummed artifacts. The legacy sweep remains an independent action.

## Motivation

The accepted user goal explicitly requires the work-instruction results to be producible without an LLM. The current repository had a serial CLI runner and raw-processing code but no RQT campaign control and no A/B/C calibration/evaluation estimator.

## Included scope

- A literal-only control block in `config/simulation.env` and a standalone RQT panel that reads it on Refresh.
- Preflight, plan, development, single-run, all, resume, reprocess, all-results, QC, stop, and separate legacy-sweep actions.
- Automatic pinned ROS message-overlay and RQT plugin builds when their install artifacts are missing; an explicit RQT control to prepare the pinned YOLO/analysis runtime.
- Live no-flight G0/G1 camera-pose and short lossless-capture preflight, including a measured storage estimate.
- Idempotent synthetic camera RTK/IMU and drone RTK inputs, A/B/C estimators, RPY_IMU/YAW_ONLY comparison, YAW_IDEAL and ORACLE diagnostics, matched-200 samples, 100-trial representative Monte Carlo, covariance-based path ranking, analysis freeze, and post-freeze V0 evaluation.
- Persistent status, hashes, logs, QC and analysis summaries with resumable attempts.

## Excluded scope and non-goals

- No hardware flight control; only the configured PX4 SITL stack is eligible.
- No repeat of the legacy OFAT/122-case sweep as a substitute for the 39 manifest slots.
- No camera-center optimization, hidden yaw prior, truth/GT input to A/B/C estimators, or V0 tuning.
- No claim that synthetic sensor settings represent physical sensor performance or that the chosen path is globally optimal.

## Acceptance summary

Acceptance requires a visible RQT click to dispatch the settings read from the current `.env`, stable progress and safe interruption/resume, successful source-QC outputs for the exact 39 slots, complete derived result schemas with A/B/C input separation, V0 freeze provenance, and evidence that the old sweep still plans/runs independently. A short end-to-end development and actual-data reprocessing must prove the control and result paths; full 100-trial output must be executable from the same RQT action.
