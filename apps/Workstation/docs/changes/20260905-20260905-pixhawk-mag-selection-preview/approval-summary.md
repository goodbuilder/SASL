# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

The existing P1/P2 MAG Priority cards gain a live topology and selection preview. Priority
controls remain editable, including CAN2 when it is outside stock PX4's selector range. Existing
PARAM writes, disarm gate, save/reboot actions, and active-device/ATTITUDE verification remain.

## Operator workflow before and after

Before: refresh PARAMs, edit role priority, apply, then infer what PX4 selected from the result.
After: refresh PARAMs, inspect `sensor_mag instance → device_id → role/CAL_MAG slot`, edit any
priority, see the projected candidate update, apply, and compare it with actual PX4 selection.

## Success, delay, rejection, failure, overload, and recovery

No new rejection for CAN2. Missing topology is a bounded waiting state. Apply success still
comes only from fresh active-device and ATTITUDE evidence; a candidate/actual mismatch remains
visible. Existing busy/timeout/reconnect recovery is preserved.

## Data and compatibility effects

No persisted schema or parameter migration. The feature reads existing Shell/PARAM/active-device
data and is compatible with stock PX4; no firmware change.

## Performance priorities and allowed degradation

## Visual and interaction changes, including dynamic-layout stability

Add one bounded text preview per P1/P2 card using existing `statusStrip` styling. It does not
block controls, steal focus, reset combo selection, or rebuild tables during live updates.

## How acceptance will be demonstrated

Focused tests cover five-instance topology (CAN2 instance4) and four-instance topology (CAN2
instance3), candidate prediction, actual mismatch, and combo updates. Real runtime evidence will
show direct PX4 active IDs separately from the preview.

## Cost, risks, assumptions, rollback, and field limits

## Exact scope awaiting approval

Approved scope: implement the selection preview and live instance identity read model only.
Explicitly excluded: automatic priority/slot remapping, hardware removal, firmware changes, and
CAN2 action blocking/reason dialogs.
