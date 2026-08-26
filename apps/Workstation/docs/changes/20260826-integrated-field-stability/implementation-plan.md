# Implementation Plan

## Approval status

Approved on 2026-08-26. Implementation may change only the integrated scope described here; every
new device setting, firmware change, destructive recovery or unrelated feature needs new approval.

## Slices and dependencies

1. Freeze repo/runtime/device/settings baseline and consolidate prior packet evidence.
   Before any StereoWIRIS Legacy hardware run, prove launch/connect/Preview/record cannot auto-apply
   the conflicting repo encoder profile.
2. Run independent short probes: RTSP/NVDEC, MAVLink, NTRIP/RTCM, STUS, admission/restart,
   MP4/evidence, GUI/resource/shutdown.
3. For each failure, prove the smallest layer, add a regression, implement the minimal slice, and
   rerun its focused tests before any dependent path.
4. Run 10-15 minute problem qualifications plus three repeated record/restart workflows.
5. Run three independent 10-minute cold-start cycles.
6. Run one 60-minute full-load soak and audit all durable evidence.

## File and component map

- Camera/media/time: `camera_stream_controller.py`, `workstation_core/yolo_stream.py`,
  `workstation_core/native_frame_ring.py`, `workstation_core/detection_pair_coordinator.py`,
  WIRIS control/time-sync owners and camera tests.
- MAVLink/NTRIP/RTCM: `device_runtime_workers.py`, `device_runtime_controller.py`,
  `gnss_runtime_controller.py`, `workstation_core/mavlink_liveness.py`,
  `workstation_core/rtcm_pipeline.py` and focused tests.
- Measurement/storage/recording: `measurement_run_controller.py`, `recording_controller.py`,
  `data_storage_controller.py`, native evidence/store modules and lifecycle tests.
- UI/status: existing Camera/GNSS/Data/Diagnostics workspaces and status reducers only when an
  observed contract defect requires them.
- Durable evidence: this packet plus affected current product documents after a contract changes.

## Migration and rollback steps

No data migration. Back up/recreate the canonical venv only if its doctor requires it. Code changes
remain independently revertible; runtime/device settings are snapshotted before any separately
approved experiment and restored/read back afterward.

## Stopping conditions

- Stop a candidate immediately on a failed short or qualification gate.
- Stop source edits on a new scope/ownership/device-setting requirement and request approval.
- Ask the user for the named physical action when USB, power-cycle, motion target or antenna change
  is required.
- Stop before a Legacy hardware run if any path can apply the stale `0/1/90/1` profile; do not use
  runtime experimentation to discover this mutation.
- Do not begin the final soak with any required `FAIL`, `NOT_RUN` or unresolved `BLOCKED` slice.
