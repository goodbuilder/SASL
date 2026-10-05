# StereoWIRIS Navigation Model

- Status: Revised after audit; logical context tree implemented
- Depends on: 00–04 design documents

## 1. Purpose

Navigation represents system structure, engineering context, and major tasks. It does not execute commands.

## 2. Candidate Tree

StereoWIRIS
- Overview
- Protocol
- Devices
  - Cameras
  - Pixhawk
  - Telemetry
  - Base GNSS
  - Time / Sync
- Data
- Calibration
- Alignment
- Measurement
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
Data owns fixed writer status, capture tools, and file-manager access. Measurement owns transient
run state, live detection, method readiness, and result inspection. Legacy Session/Review IDs
resolve to Data but do not render lifecycle controls or catalog rows.

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

Every board transport selector offers Radio, Ethernet, and USB. Radio and USB share the serial
device row and its exclusive-path preflight; they differ only in stream policy, so USB never
inherits the Radio throttle. USB reuses the board's bootstrap endpoint, leaving the saved Radio
endpoint intact across a transport switch.
P1/P2 use their existing transport rows and explicit Connect/Disconnect actions. `Drone / sysid 3`
transport and Connect live in `Devices > Telemetry`, which owns that MAVLink side. Network
discovery/assignment remains an explicit host-side operation; it does not automatically connect a
board.

## 6. Navigation vs Local Tabs

Navigation handles system-level context. Local tabs switch views within that context.

When Navigation is saved as hidden, moving the pointer to the Workstation window's left edge
temporarily reveals the dock. It hides 400 ms after the pointer leaves the dock; moving back
into it cancels hiding. A Navigation dock saved or manually set to visible remains pinned.
Temporary edge reveals do not change the saved dock visibility. The toolbar and View menu
remain available for manual visibility changes.

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

Overview; Protocol; Devices/Cameras/Pixhawk/Telemetry/GNSS/Time; Data; Calibration;
Alignment; Measurement; Advanced; Settings; Diagnostics.

## 13. Compatibility IDs

Old saved or external context IDs remain accepted as aliases: camera Left/Right/Stereo
resolve to Cameras, Pixhawk P1/P2 resolve to Pixhawk, Detection resolves to Cameras,
Synchronization resolves to Time / Sync, and old Session/Recording/Review/Files IDs resolve to Data.
They are not rendered as separate navigation nodes.

## 14. Anti-Patterns

Do not execute on selection, hide disconnected nodes, embed primary commands in tree rows, create nodes for every parameter, duplicate local tabs, expose raw module names, or build deeply nested hierarchy without engineering meaning.

## 15. Summary

> Navigation changes context. Commands change the system.
