# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

The Telemetry page gains two read-only GNSS geometry values: the horizontal angle between the
P1→P2 GNSS baseline and the midpoint-to-Drone GNSS vector, plus its absolute error from 90°.
Existing raw GNSS, F/R/D/Rg, commands, devices, measurement, and recording stay unchanged.

## Operator workflow before and after

Before, the operator reads GNSS F/R/D/Rg and mentally derives the angle to the Drone. After, the
angle and 90° error are visible directly in `Devices > Telemetry`; the operator adjusts the camera
installation and uses the existing GNSS freshness evidence.

## Success, delay, rejection, failure, overload, and recovery

Fresh three-GNSS inputs show CURRENT. Missing or degenerate data shows WAITING; stale geometry
retains finite values and exposes the timing reason. No worker/load behavior changes; recovery
follows fresh existing GNSS callbacks.

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

## Exact scope approved

Approved on 2026-09-03: add GNSS-only horizontal `Baseline↔Drone` angle and absolute `90° error`
to the existing Telemetry `GNSS BASELINE_CENTER` table, using the P1/P2 GNSS antenna midpoint as
the vertex. Keep the existing F/R/D/Rg and freshness contract. Do not use Drone ATTITUDE/yaw, add
a verdict/control, or change persistence, measurement, recording, commands, workers, devices, or
protocols.
