# StereoWIRIS Navigation Model

- Status: Revised after audit; logical context tree implemented
- Depends on: 00–04 design documents

## 1. Purpose

Navigation represents system structure, engineering context, and major tasks. It does not execute commands.

## 2. Candidate Tree

StereoWIRIS
- Overview
- Devices
  - Cameras
  - Left Camera
  - Right Camera
  - Stereo Pair
  - Time / Sync
  - Pixhawk
    - Pixhawk 1
  - Pixhawk 2
  - GNSS / RTK
- Processing
  - Detection
  - Measurement
  - Synchronization
  - Alignment
- Tasks
  - Calibration
  - Recording
  - Playback
  - Export
- Diagnostics
  - System
  - Camera
  - Pixhawk / MAVLink
  - GNSS
  - Measurement
  - Performance

This hierarchy is now the logical Workstation context tree. It does not own device,
measurement, calibration, recording, or export state; each selection only changes the
workspace/Inspector context.

## 3. Selection Semantics

One active selection determines Workspace and Inspector context. Selection must never stop recording, disconnect a device, reset calibration, or discard settings.

## 4. Device Nodes

Disconnected hardware remains selectable so configuration can still be inspected. Stable logical roles should not disappear based on connectivity.

## 5. Navigation vs Local Tabs

Navigation handles system-level context. Local tabs switch views within that context.

## 6. Diagnostics Exception

Selecting Diagnostics may focus/expand the bottom dock while preserving current Workspace.

## 7. Global Commands

Availability depends on runtime state, not navigation location.

## 8. Logical vs Physical Identity

Navigation primarily shows logical role (Left Camera, Pixhawk 1). Physical address/serial/device path belongs in Inspector/Diagnostics.

## 9. State Decoration

Compact secondary decoration is allowed but must not replace textual meaning. Detailed state belongs elsewhere.

## 10. Keyboard / Density

Use native QTreeView keyboard behavior. Recommended row height 22–26 px. Target depth approximately 3–4 meaningful levels.

## 11. Phase 1 Minimum

Overview; Devices/Cameras/Time/Pixhawk/GNSS; Processing/Detection/Measurement; Tasks/Calibration/Recording; Diagnostics.

## 12. Anti-Patterns

Do not execute on selection, hide disconnected nodes, embed primary commands in tree rows, create nodes for every parameter, duplicate local tabs, expose raw module names, or build deeply nested hierarchy without engineering meaning.

## 13. Summary

> Navigation changes context. Commands change the system.
