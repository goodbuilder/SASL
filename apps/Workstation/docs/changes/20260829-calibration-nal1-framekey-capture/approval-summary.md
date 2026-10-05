# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

Auto capture accepts only a WIRIS frame positively observed as a NAL type 1, non-delta access unit,
then saves the exact native FrameKey on which the checkerboard was detected. Auto no longer uses the
internal 1 ms nearby-frame search. Manual/FTP/OZ capture, Preview, YOLO, Measurement, recording,
camera settings, calibration math, and pair thresholds stay unchanged.

## Operator workflow before and after

The operator still sets Interval/Max Live pair delta, presses Start, moves the board through new
poses, and presses Stop. Auto may wait longer; the status distinguishes waiting for NAL1, exact
native frame, new pose, timing-eligible pair, or worker completion.

## Success, delay, rejection, failure, overload, and recovery

Success requires one AU, decoded PTS, FrameKey, checkerboard result, native PNG, and metadata record
to agree exactly. Missing classification, dropped/expired key, generation mismatch, stale frame, or
malformed evidence saves nothing and waits for the next candidate. Existing timing-ineligible
K,D-only retention remains. No unbounded retry or queue.

## Data and compatibility effects

Capture JSON gains optional JSON-safe NAL/PTS/FrameKey provenance. Existing settings, photo sets, and
profiles need no migration. Legacy/non-GStreamer input without positive proof cannot use new Auto;
manual Live Pair remains available.

## Performance priorities and allowed degradation

Live Preview, decode, YOLO, Measurement, and recording take priority. Only non-delta AUs are mapped
and the PTS cache is bounded. Slower Auto capture is acceptable; slowing unrelated consumers is not.

## Visual and interaction changes, including dynamic-layout stability

No controls/layout move. Existing status/tooltips gain precise waiting reasons while retaining
wrapping, focus order, scrolling, theme, and target sizes.

## How acceptance will be demonstrated

Hand-built AU tests; exact-PTS/cache tests; exact FrameKey, eviction, stale, and generation tests;
Auto normal/timing-only/duplicate/Stop/failure tests; preserved manual and camera-consumer regression;
full project pytest/application smoke; hardware/manual observation when available. Fakes are not
reported as hardware proof.

## Cost, risks, assumptions, rollback, and field limits

Independent left/right NAL1 cadence may reduce stereo yield. NAL1 non-delta is an operational
capture category, not IDR/integrity proof. Rollback needs no migration. Real WIRIS cadence and exact
decoded-PTS behavior remain field conditions until observed.

## Exact approved scope

Approved scope is positive NAL1/non-delta classification, exact PTS and FrameKey propagation, Auto-only
gating, fail-closed status/provenance, focused regression, and the confirmed workspace README/product
docs. No encoder mutation, global filter, timing-policy change, worker allocation, or unrelated
cleanup.
