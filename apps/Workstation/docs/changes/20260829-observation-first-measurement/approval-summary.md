# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

Measurement opens immediately and sources join independently. Existing accuracy, identity,
calibration and coordinate gates still govern results, not collection.

## Operator workflow before and after

Before: camera readiness and finalization can own the run transition. After: Start first, observe
available data, recover sources during the run, Stop, and immediately admit another run.

## Success, delay, rejection, failure, overload, and recovery

No source/storage/media delay rejects Start. Busy sources wait locally; failed camera recovery is
manual. Essential rows retry within fixed memory; drops remain visible.

## Data and compatibility effects

Additive source/segment fields, no migration or old-data rewrite. SAVED means file exists only.

## Performance priorities and allowed degradation

Observation/YOLO/device receive outrank recording and persistence. No new full-file verification.

## Visual and interaction changes, including dynamic-layout stability

Reuse existing compact labels and buttons; no new page/table or moving live layout.

## How acceptance will be demonstrated

Automated 0/1/2-source, failure/recovery, late callback, storage and UI tests plus real sequential
field runs with camera/device recovery.

## Cost, risks, assumptions, rollback, and field limits

A run may contain no valid result; the application cannot survive process/OS/OOM failure. Rollback
does not alter produced data.

## Exact scope awaiting approval

Approved by the user on 2026-08-29 through the decision-complete proposed plan and explicit
`Implement the plan` request.
