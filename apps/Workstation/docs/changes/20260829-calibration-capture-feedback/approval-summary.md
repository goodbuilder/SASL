# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

The default Calibration Capture tab gains Left/Right checkerboard overlays and factual board,
current-run and current-capture-set counters. Existing capture and calibration decisions remain.

## Operator workflow before and after

Before, recognition/candidate evidence required Advanced. After, it is visible while capturing;
Photo set selection and Compute/Apply remain explicit and unchanged.

## Success, delay, rejection, failure, overload, and recovery

Only worker acceptance/completion and persisted files count. Waiting reasons remain visible without
inflating counters. Stale/unavailable/scan errors are explicit and recover through existing paths.

## Data and compatibility effects

Additive ephemeral snapshot only; no settings or capture-file migration.

## Performance priorities and allowed degradation

No new worker/GPU demand. Existing 2 Hz overlay is projected twice; cached inventory avoids polling
rescans. Preview and capture responsiveness remain priority.

## Visual and interaction changes, including dynamic-layout stability

Reuse native compact camera/status components, equal previews, fixed rows and elided details. No
button/focus/tab order change and no generic dashboard styling.

## How acceptance will be demonstrated

Pure inventory, controller, facade, UI and integration tests; target-resolution screenshots;
focused/full pytest and launcher smoke; real WIRIS remains hardware-manual.

## Cost, risks, assumptions, rollback, and field limits

Waiting text may evolve with active Calibration packets but their gates remain untouched. Rollback
does not affect persisted images. Hardware readability is not claimed from offscreen tests.

## Exact scope awaiting approval

Approved by the user on 2026-08-29 with `Implement the plan.` Scope is this packet only.
