# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

When P1/P2 attitude or GPS packets disconnect or become stale, the live Pixhawk+RTK measurement
method will automatically continue with the last valid values observed during the current
application run. Normal live values replace retained values automatically when reception resumes.
Camera, detection, calibration, recording, formulas, coordinate frames, and other measurement
methods stay unchanged.

## Operator workflow before and after

Before: a later Measurement can report no ready method when a required Pixhawk/GPS source stops,
even if the rig is static and earlier values remain known.

After: after every required side has supplied at least one valid value, the operator starts and
uses Measurement normally. No Freeze/Clear command or extra confirmation is required. A stalled or
disconnected source reuses its last valid process-local value.

## Success, delay, rejection, failure, overload, and recovery

- Success: exact operating K,D and a complete finite positive P1-to-P2 geometry remain required;
  earlier valid attitude/GPS values may satisfy the sensor part of readiness.
- Rejection: a side that has never produced a valid sample, a non-finite sample, GPS below 3D fix,
  or a non-positive/non-finite baseline remains blocked.
- Recovery: newer valid packets replace retained values automatically.
- Restart/shutdown: all retained values are discarded; each side must produce fresh valid values
  again after launch.
- No workers, queues, device commands, retries, or resource budgets change.

## Data and compatibility effects

No new setting or evidence-schema field is introduced. Existing exact attitude/GPS payloads,
device identities, geometry hashes, and their real timestamps remain stored unchanged. No
retained/fallback marker or dedicated log event is added, and historical files are never loaded as
active inputs after restart.

## Performance priorities and allowed degradation

No measurable CPU/GPU, latency, throughput, or GUI-load change is expected. The accepted
degradation is that physically moving the rig while telemetry is unavailable cannot be detected;
the retained values remain usable until valid packets resume or the app restarts.

## Visual and interaction changes, including dynamic-layout stability

No new button, toggle, warning, banner, age field, modal, label, row, color state, or keyboard
action. Existing layout, focus, selection, scroll, docks, splitters, and window geometry remain
unchanged. The ordinary ready-method result may become available instead of showing
`준비된 측정 방식 없음`.

## How acceptance will be demonstrated

Focused tests will prove: pre-Measurement attitude reuse; stale/disconnected per-side attitude and
GPS reuse; never-observed/invalid input rejection; source-selection cache invalidation; newer-live
replacement; reset-on-new-process behavior; unchanged raw values/timestamps; readiness/UI result;
and preserved live/alternate-method behavior. Relevant regression tests and the real launcher
check will then run. Hardware evidence will be reported separately from automated tests.

## Cost, risks, assumptions, rollback, and field limits

Risk: retained geometry can be wrong if the rig moves while telemetry is unavailable. This is an
explicitly accepted operating assumption. Rollback is a local reversion to the existing
current-Measurement-only attitude rule and existing cache behavior. The implementation must not
touch coordinate conversion or fabricate missing data.

## Exact scope awaiting approval

Approve only the automatic, process-local last-valid fallback described above, with no new UI,
setting, marker, or log event and no unrelated cleanup.
