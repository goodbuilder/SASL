# Frontend contract

Existing Workspace classes and DataWorkspace/SessionsWorkspace aliases remain import-compatible. Actual screen classes are owned by their feature modules; MainWindow and leaf panes import those owners directly. No new layout or controls are requested.

Panel shell, widget construction and runtime feature state are separate responsibilities. Preserve Qt parent/thread affinity, signal routing, focus/selection/scroll, coordinate labels, saved dock behavior and explicit command error presentation.
