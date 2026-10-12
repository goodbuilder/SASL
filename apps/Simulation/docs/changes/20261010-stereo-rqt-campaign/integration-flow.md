# Integration Flow

```text
edit Simulation.env
  -> RQT Refresh (literal validation + config hash)
  -> Run configured action confirmation
  -> control_cli output-root/no-overwrite checks
  -> preflight (static pins + G0/G1 short live captures + storage estimate)
  -> initialize 39-slot root once
  -> campaign_run all/resume (one headless SITL, manifest seed/profile, raw capture, replay YOLO, alignment, QC)
  -> synthetic camera RTK/IMU and drone RTK generation
  -> A/B/C × RPY_IMU/YAW_ONLY calibration (truth-isolated)
  -> covariance-based path selection from G0 only
  -> analysis freeze + checksums
  -> V0-only evaluation with fixed cameras
  -> RQT logs, slot counts, analysis summary, CSV/JSON outputs
```

The legacy branch is independent:

```text
edit existing SWEEP_* values in Simulation.env
  -> set STEREO_V1_1_ACTION=legacy_sweep
  -> RQT Run configured action
  -> run_auto_attitude_sweep.sh
```

The campaign runner refuses to start when another Gazebo/PX4/operator stack is active. The RQT controller perspective is standalone and uses a filename excluded from the legacy operator-window cleanup pattern, so the headless run lifecycle does not close the controller window.

## Action-to-postcondition traces

## Transition tables

## Failure and recovery flows

## Cross-feature contracts and dependency direction
