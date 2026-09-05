---
schema: modify-gui-change-state-v1
change_id: 20260904-magnetometer-yaw-display
status: completed
mode: backend-coupled
updated: 2026-09-05
baseline_ref: 3288aee
last_completed: "Live Radio run verified P1/P2 Stable PX4 Shell sensor_mag polling, ten populated RAW-yaw cells, fresh ATTITUDE liveness, direct CAL_MAG readback, persistent internal PX4 yaw diagnostics, Raw-yaw-only operator table presentation, user-requested EKF parameter parity readback, CAL_MAG-corrected yaw projection support, direct RM3100 vector/UAVCAN health evidence, and a CAN2 priority trial with active-device readback"
next_question: null
read_next:
  - docs/changes/20260904-magnetometer-yaw-display/change-brief.md
  - docs/changes/20260904-magnetometer-yaw-display/current-behavior.md
  - docs/changes/20260904-magnetometer-yaw-display/approval-summary.md
  - docs/changes/20260904-magnetometer-yaw-display/verification.md
affected_product_docs:
  - README.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/03_WORKSPACE_LAYOUT.md
  - docs/ui/06_STATUS_MODEL.md
blocking_questions: 0
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "Original scope plus DEBUG_VECT compatibility rows and bounded Stable PX4 Shell sensor_mag/vehicle_magnetometer polling into five RM/yaw rows per Pixhawk; no firmware or SASL_CAN_MAG request"
  conditions: []
  recorded_at: 2026-09-04
---

# Resume Notes

The user chose to display both raw and corrected/reference yaw and approved the packet. PX4 local
source evidence supports that priority-selected `ATTITUDE.yaw` is the firmware estimator output
after magnetometer calibration/rotation and tilt-aware fusion, but raw `SCALED_IMU*`/CAN vectors
remain producer-axis telemetry. Source implementation is now allowed for this packet only.
