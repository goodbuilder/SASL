# Implementation Plan

## Approval status

Approved by the user's explicit active-goal instruction. Implementation is allowed only inside the Simulation workspace and temporary directories; the Desktop raw source is read-only.

## Slices and dependencies

1. **Env contract and RQT controller** — strict settings parser, standalone perspective/launcher, process lifecycle, refresh/status/stop, runtime-preparation action. Depends on the pinned workspace overlay and plugin registration.
2. **Preflight and run orchestration** — pinned package/runtime checks, two live no-flight camera/capture probes, storage estimate, immutable status/protocol files, existing `campaign_run` integration, safe resume.
3. **Offline measurement inputs** — idempotent camera RTK/IMU and drone RTK producers with source hashes and shared keys.
4. **Calibration methods** — A independent, B shared-latent+RTK, C image-only; RPY_IMU and YAW_ONLY; explicit rank/covariance/convergence results and YAW_IDEAL diagnostic.
5. **Freeze and evaluation** — G0-only covariance path selection, code/protocol freeze, then V0 triangulation/GT metrics with ORACLE reference and no V0 recalibration.
6. **Handoff and compatibility** — method documentation, checksums, CSV/JSON summaries, existing sweep regression, portability report.

## File and component map

`config/simulation.env`; `campaigns/stereo_v1_1/{control_config,control_cli,campaign_preflight,preflight_capture,campaign_run,campaign_status,generate_sensor_observations,generate_drone_rtk_observations,calibration_solver,analyze_campaign}.py`; `stereo_ground_sim/stereo_ground_sim/campaign_panel.py`; `stereo_ground_sim/plugin.xml`; `config/campaign_control.perspective`; `run_stereo_campaign_rqt.sh`; `scripts/ensure_px4_msgs_overlay.sh`; `scripts/prepare_stereo_campaign_runtime.sh`; focused tests; `README.md` after its path is confirmed.

## Migration and rollback steps

Existing raw campaign folders and old sweep settings are preserved. New campaign IDs create new output roots. Incomplete attempts are never overwritten. The old sweep remains callable independently. A changed algorithm automatically receives a new content-derived analysis ID; a matching analysis version is immutable.

## Stopping conditions

Stop a flight-bearing action before arm if PX4/version/model/ROS checks fail, a G0/G1 camera pose differs from the protocol, lossless capture or GT rate fails, full-campaign storage is insufficient, or an active SITL stack already exists. Stop dependent analysis if a calibration result lacks inputs, C receives RTK/GT, method rank is deficient without an explicit failure record, or the freeze hash does not match. In each case preserve the attempt and report the exact check/result.
