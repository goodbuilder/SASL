# StereoWIRIS Product Vision

- Status: Draft
- Product class: Industrial Engineering Workstation
- Primary platform: Desktop
- Primary users: Engineers, researchers, system operators
- UI framework: Qt desktop application

## 1. Purpose

StereoWIRIS shall be a professional engineering workstation for operating, observing, configuring, diagnosing, recording, and validating a multi-sensor stereo measurement system.

It shall integrate cameras, stereo vision, Pixhawk/PX4, GNSS, RTK, attitude sensors, object detection, measurement, recording, calibration, and diagnostics within one coherent desktop environment.

StereoWIRIS is not intended to be a collection of independent configuration screens. It shall behave as one integrated engineering instrument.

## 2. Product Vision

StereoWIRIS shall allow an engineer to understand the state of the entire measurement system and perform the current engineering task without unnecessary screen changes.

Priorities:
1. system visibility
2. engineering accuracy
3. operational efficiency
4. information density
5. diagnostics
6. reproducibility
7. direct access to technical information

> Maximum useful engineering information with minimum unnecessary navigation.

## 3. Product Definition

StereoWIRIS is an Industrial Engineering Workstation for stereo camera, flight-controller, GNSS/RTK, computer-vision, and geometric measurement operation.

It combines characteristics of industrial engineering software, instrumentation software, machine-vision software, robotics diagnostics software, and scientific measurement software.

## 4. What StereoWIRIS Is Not

StereoWIRIS shall NOT be designed as:
- SaaS dashboard
- mobile-first application
- consumer camera application
- card-based analytics dashboard
- presentation-oriented interface
- touch-first HMI panel
- collection of unrelated setup dialogs
- sequence of feature tabs

Do not prioritize decorative whitespace, large rounded cards, animation, hidden controls, or simplified consumer terminology over engineering usability.

## 5. Primary Users

### System Engineer
Configure cameras/communication, inspect sensors/PX4/GNSS/RTK, validate calibration, diagnose failures.

### Measurement Operator
Confirm readiness, view camera streams, enable detection, monitor measurement, record, observe quality, identify abnormal state.

### Researcher / Developer
Inspect raw values, timestamps, coordinate frames, quality, source comparisons, logs, and reproducible experiment data.

## 6. Core Engineering Domains

### Devices
Left/right/thermal cameras, Pixhawk 1/2, GNSS receivers, RTK source, magnetometers.

### Processing
Image decoding, detection, stereo correspondence, triangulation, attitude transformation, RTK baseline estimation, synchronization.

### Tasks
Camera setup, calibration, alignment, live measurement, recording, playback, export.

### Diagnostics
Connection state, raw sensor values, logs, timing, latency, quality, validity, errors.

## 7. Core UX Principles

### Persistent Context
Important operational context remains visible while related tasks are performed.

### Low Navigation Cost
Frequent workflows require minimal screen switching.

### High Information Density
Prefer compact controls, tables, trees, inspectors, dockable panels, plots, and status strips over decorative containers.

### Explicit Engineering Meaning
Where applicable include value, unit, source, validity, timestamp. For pose/vector quantities expose coordinate frame, axis convention, reference point, and rotation convention.

### State Before Decoration
Colors and emphasis communicate operational meaning.

### Expert Efficiency
Frequent operations shall be directly accessible.

### Evidence First
Distinguish measured facts, configured values, derived values, assumptions, warnings, and failures.

## 8. Application-Level UI Model

StereoWIRIS shall separate six responsibilities:
- Navigation: select subsystem/device/context
- Workspace: perform primary engineering task
- Inspector: inspect/edit selected-object properties
- Commands: execute operations
- Diagnostics: inspect technical evidence
- Global Status: show critical runtime state persistently

## 9. Target Interaction Model

1. Select system/task from Navigation.
2. Work in Workspace.
3. Inspect/modify selected-object properties in Inspector.
4. Use global Commands without leaving workspace.
5. Observe important state continuously.
6. Open Diagnostics for deeper evidence.

## 10. Design Constraints

The redesign shall not silently change measurement algorithms, coordinate systems, reference frames, units, timestamps, calibration formats, recording formats, PX4 semantics, or GNSS/RTK semantics.

## 11. Migration Constraint

The existing StereoWIRIS UI shall initially be treated as a reference implementation for functionality, backend interactions, engineering parameters, current states, workflows, and failure cases.

## 12. Success Criteria

- critical state visible without repeated navigation
- work continues while relevant status/diagnostics remain visible
- new subsystems do not automatically create top-level tabs
- engineering values and coordinate semantics remain unambiguous
- required legacy capabilities remain accessible
- diagnostics do not destroy working context
- repeated workflows require fewer unnecessary navigation actions

## 13. Non-Goals for First Redesign

Do not require replacement of every widget, removal of every existing tab, Qt binding migration, backend rewrite, plugin architecture, touch-first operation, or visual cloning of another industrial application.

## 14. Decision Rule

Evaluate UI decisions in this order:
1. engineering correctness
2. system awareness
3. unnecessary navigation reduction
4. context preservation
5. operational efficiency
6. information hierarchy
7. visual consistency

## 15. Vision Statement

> The engineer sees the system, not a collection of screens.
