# Baseline

Commit 6810b228e7783c6420fd456e877c978e56a66872 plus pre-existing user edits (snapshot /tmp/workstation-designer-baseline-status.txt). User edits include Pixhawk magnetometer workflows, README, main_window, backend/worker files and earlier packets; preserve them.

Existing UI built programmatically by main_window, workspace_views, ui/settings_workspace. CommandRegistry -> existing backend owners; EvidenceAdapter -> read models. Baseline real-shell offscreen capture: /tmp/workstation-designer-before. Captures use isolated loopback netns and temporary settings/data, with hardware startup disabled.
