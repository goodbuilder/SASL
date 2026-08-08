# StereoWIRIS Information Architecture

- Status: Draft
- Depends on: 00_PRODUCT_VISION.md, 01_USER_WORKFLOW.md

## 1. Philosophy

Organize according to engineering meaning, not implementation modules.

## 2. Six UI Domains

1. Navigation
2. Workspace
3. Inspector
4. Toolbar
5. Diagnostics
6. Global Status

Every feature has one primary owner.

## 3. Navigation

Purpose: select device, subsystem, or engineering task. Navigation does not execute commands or edit parameters.

Candidate hierarchy:

StereoWIRIS
- Devices
  - Cameras: Left, Right, Stereo Pair
  - Pixhawk: P1, P2
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
- Diagnostics / Analysis

## 4. Workspace

Largest area where engineering work is performed. Candidate workspaces: Overview, Camera, Measurement, Calibration, Playback, Analysis, and possibly Pixhawk/GNSS/Detection if justified by audit.

## 5. Inspector

Contextual properties/configuration of selected object. It shall not contain logs, large plots, or camera preview.

## 6. Toolbar

Commands only. No configuration forms and no system-state truth.

## 7. Diagnostics

Engineering evidence such as logs, raw states, communication details, timing, performance.

## 8. Global Status

Compact persistent state. Candidates: Camera, Pixhawk, RTK, Detection, Measurement, Recording, FPS. Include only high-value reliable states.

## 9. Mapping Examples

- Camera Preview → Workspace
- Exposure/Gain → Inspector
- Connect Camera → Command
- Camera Connected → Status
- Record Start → Command
- Recording Active → Status
- Measurement Result/Quality → Workspace
- Measurement Debug → Diagnostics
- YOLO Threshold → Inspector
- Detection Enable → Command
- GNSS details → Workspace or Diagnostics depending on task
- Raw MAVLink → Diagnostics
- Calibration workflow/result → Workspace

## 10. Domain Interaction

Navigation changes context. Workspace/Inspector reflect context. Commands act on the system. Status reports confirmed state. Diagnostics provides evidence.

## 11. Forbidden Structures

Do not default to new top-level tabs, settings pages per object, commands inside Diagnostics, logs inside Inspector, or Global Status buried in Workspace.

## 12. Decision Tree

Command? → Toolbar/Command layer
State? → Status
Configuration? → Inspector
Engineering task? → Workspace
Evidence? → Diagnostics
System structure? → Navigation

## 13. Success Criteria

- engineers can predict where features belong
- new subsystems extend domains rather than top-level tabs
- domain ownership is clear
