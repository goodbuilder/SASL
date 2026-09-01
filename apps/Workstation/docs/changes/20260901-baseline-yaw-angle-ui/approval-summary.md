# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

The Pixhawk page gains a read-only signed horizontal baseline/yaw summary and top view. Existing
raw attitude, NED/FRD baseline, commands, devices, measurement, and recording stay unchanged.

## Operator workflow before and after

Before, yaw and baseline are read separately and mentally compared. After, their signed P1/P2
relationship is visible immediately below the baseline; the physical adjustment/sample procedure
does not change.

## Success, delay, rejection, failure, overload, and recovery

Fresh inputs show CURRENT. One usable side shows PARTIAL. Missing data shows WAITING; stale values
retain their numbers but are visibly stale. No worker/load behavior changes; recovery follows fresh
existing source callbacks.

## Data and compatibility effects

No new persisted field or schema. Existing attitude/GNSS records remain sufficient for offline
reconstruction. No migration is required.

## Performance priorities and allowed degradation

Only constant-time calculation and a small visible repaint are allowed inside the existing 4 Hz
poll. The UI must remain responsive and geometry stable.

## Visual and interaction changes, including dynamic-layout stability

Reuse compact native Workstation values/theme. The new plot has fixed height; updates cannot move
controls, focus, selection, or scroll. There are no new controls or decorative dashboard patterns.

## How acceptance will be demonstrated

Pure geometry, evidence state, Qt integration, rapid-update/layout, dependent regressions, full
pytest, and launcher checks; hardware/manual behavior remains explicitly classified.

## Cost, risks, assumptions, rollback, and field limits

The view is a latest-state physical-alignment aid, not synchronized measurement evidence. Removal
is additive rollback with no data conversion. Real-rig readability remains manual field evidence.

## Exact scope awaiting approval

Approved on 2026-09-01: signed horizontal P1/P2 live values plus top view, no verdict/history/new
persistence, located below the Pixhawk baseline group, with Workstation README documentation.
