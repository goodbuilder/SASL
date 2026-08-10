# StereoWIRIS Component System

- Status: Draft
- Depends on: 00–03 design documents

## 1. Purpose

Define reusable semantic UI components so each workspace does not invent a different interaction model.

## 2. Core Categories

- Navigation
- Commands
- Status
- Inspector/Properties
- Data Display
- Diagnostics
- Visualization
- Workflow
- Feedback

## 3. Navigation Tree

Use QTreeView/model-based data. It represents structure, supports one clear selection, concise labels, and no operational commands embedded by default.

## 4. Workspace Header

Compact context title, optional local commands/status. Target height 28–36 px.

## 5. Toolbar Command

Use QAction/QToolBar/QToolButton. Each action has identity, tooltip, enabled state, optional shortcut/icon, and checked state only when genuinely toggleable.

## 6. Command-State Separation

A command is not state truth. Recording, connection, preview, etc. require separate confirmed status.

## 7. Global Status Indicator

Use concise text+icon/semantic cue. Do not rely on color alone.

## 8. Engineering Value

Structure: label, value, unit, validity, optional timestamp/age/source. Missing/invalid data must not be rendered as zero.

## 9. Timestamp / Age

Use explicit time basis. Age for freshness, absolute timestamps for correlation/reproducibility.

## 10. Coordinate / Frame Annotation

For pose/vector values expose frame/axis/reference where applicable. Avoid ambiguous bare X/Y/Z where explicit semantics exist.

## 11. Property Row / Section

Compact aligned Label | Control | Unit/State. Sections such as GENERAL, CONNECTION, IMAGE, MEASUREMENT, ADVANCED. Avoid large card containers.

## 12. Read-only / Pending / Applied

Read-only must not look editable. If deferred application exists, distinguish current/pending/applied and use Apply/Revert only when semantically correct.

## 13. Data Table

Use QTableView/model-based architecture for structured engineering data. Prefer units in headers for homogeneous columns.

## 14. Diagnostic Log

Recommended columns: Time, Level, Source, Message, optional Code/Context. Preserve timestamps and factual wording.

## 15. Plots

Every plot defines title, axis meanings, units, series identity, and range behavior. Do not rely on color alone.

## 16. Camera View

Preserve aspect ratio. Only engineering-relevant overlays. Distinguish no source/connecting/live/paused/error.

## 17. Summary Panel

Compact rows, not individual cards.

## 18. Workflow Step Indicator

Use only for genuinely sequential tasks such as calibration.

## 19. Empty / Loading / Error States

Blank panels are unacceptable when factual explanation is available. Use explicit NO VIDEO, WAITING, STALE, INVALID, ERROR, etc.

## 20. Inline Error vs Dialog

Recoverable/local failures → inline. Destructive/irreversible or blocking decisions → modal confirmation.

## 21. Tabs

Allowed for local views within one context. Not the main application information architecture.

## 22. GroupBox / Splitter / Scroll

GroupBox sparingly. QSplitter for resizable engineering views. Scroll areas acceptable for Inspector/forms, not as giant primary-workspace design.

## 23. Density Targets

Controls 24–30 px, rows 22–28 px, margins/spacing 4–8 px, toolbar 28–34 px.

## 24. Reuse Rule

Same engineering meaning → same component and interaction pattern.

## 25. Qt Mapping

- Navigation → QTreeView
- Workspace host → QStackedWidget or controlled central host
- Inspector → QScrollArea + property widgets
- Diagnostics → QDockWidget + local tabs
- Tables → QTableView
- Logs → QTableView/QPlainTextEdit
- Toolbar → QToolBar + QAction
- Status → QStatusBar + compact widgets
- Resizable subregions → QSplitter

## 26. Phase 1 Core Components

NavigationTree, WorkspaceHeader, StatusIndicator, EngineeringValue, PropertySection, PropertyRow, DiagnosticLogView, EmptyStateView.

## 27. Summary

> Same engineering meaning, same interaction pattern, same visual language.
