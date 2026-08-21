# StereoWIRIS Navigation Model

- Status: Revised after audit; logical context tree implemented
- Depends on: 00–04 design documents

## 1. Purpose

Navigation represents system structure, engineering context, and major tasks. It does not execute commands.

## 2. Candidate Tree

StereoWIRIS
- Overview
- Protocol
- Pre-run Calibration
- Devices
  - Cameras
  - Pixhawk
  - Base GNSS
  - Time / Sync
- Alignment
- Session
- Measurement
- Recording / Review
- Advanced
- Settings
- Diagnostics
  - System
  - Camera
  - Pixhawk / MAVLink
  - GNSS
  - Logs
  - Measurement
  - Performance

This hierarchy is now the logical Workstation context tree. It does not own device,
measurement, calibration, recording, review, or export state; each selection only changes
the workspace/Inspector context. Cameras and Pixhawk are paired contexts; their Left/Right
or P1/P2 detail is shown inside the workspace and Inspector rather than as navigation
children. Detection is part of Cameras, synchronization is part of Time / Sync, and
Session owns the daily-work lifecycle, current Measurement readiness, method-input evidence,
and the ordered child summary. Measurement owns live detection and result inspection.
Review/Files are part of Recording / Review.

## 3. Selection Semantics

One active selection determines Workspace and Inspector context. Selection must never stop recording, disconnect a device, reset calibration, or discard settings.

## 4. Protocol Entry

Protocol is the only navigation item that presents the numbered field sequence. It links
to the Cameras, Pixhawk, Base GNSS, time, measurement, alignment, calibration, and
recording contexts. Steps 1 and 4 provide multiple device shortcuts where one page cannot
show all required connection or fix-number evidence.
QGC Pixhawk calibration remains an external prerequisite; selecting Protocol never starts
hardware actions.

## 5. Device Nodes

Disconnected hardware remains selectable so configuration can still be inspected. Stable logical roles should not disappear based on connectivity.

## 6. Navigation vs Local Tabs

Navigation handles system-level context. Local tabs switch views within that context.

## 7. Diagnostics Exception

Selecting Diagnostics may focus/expand the bottom dock while preserving current Workspace.

## 8. Global Commands

Availability depends on runtime state, not navigation location.

## 9. Logical vs Physical Identity

Navigation shows paired logical contexts (Cameras, Pixhawk). Physical side, address, serial,
and device path belong in the paired workspace or Inspector/Diagnostics.

## 10. State Decoration

Compact secondary decoration is allowed but must not replace textual meaning. Detailed state belongs elsewhere.

## 11. Keyboard / Density

Use native QTreeView keyboard behavior. Recommended row height 22–26 px. Target depth approximately 3–4 meaningful levels.

## 12. Phase 1 Minimum

Overview; Protocol; Pre-run Calibration; Devices/Cameras/Pixhawk/GNSS/Time;
Alignment; Session; Measurement; Recording/Review; Diagnostics.

## 13. Compatibility IDs

Old saved or external context IDs remain accepted as aliases: camera Left/Right/Stereo
resolve to Cameras, Pixhawk P1/P2 resolve to Pixhawk, Detection resolves to Cameras,
Synchronization resolves to Time / Sync, and Review/Files resolve to Recording / Review.
They are not rendered as separate navigation nodes.

## 14. Anti-Patterns

Do not execute on selection, hide disconnected nodes, embed primary commands in tree rows, create nodes for every parameter, duplicate local tabs, expose raw module names, or build deeply nested hierarchy without engineering meaning.

## 15. Summary

> Navigation changes context. Commands change the system.
