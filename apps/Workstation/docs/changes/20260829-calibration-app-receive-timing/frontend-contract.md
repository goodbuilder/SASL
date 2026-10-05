# Frontend Contract

## Controls and inputs

No new control. Existing `Max Live pair Δt` remains an editable 0–1000 ms threshold and the existing
Capture Live Pair/Auto/FTP actions remain in place.

## Payloads and commands

Command IDs and setup payloads are unchanged. Capture consumes the existing per-side native-frame metadata;
the controller freezes the chosen timing basis and result before starting the worker.

## Visible states and transitions

- Normal Live pair: status/detail identifies app-receive basis and signed/absolute gate result.
- Over-limit/missing receive timestamp: existing intrinsics-only state, with receive-specific reason.
- FTP: existing CPTR-dispatch wording and state.
- Photo table: wall-clock source remains separately labelled and is not described as the pair gate.

## Errors and recovery actions

Missing/invalid app-receive evidence tells the operator the pair is K,D-only. Recovery is to wait for fresh native
frames and capture again; no NTP synchronization action is required for this Live gate.

## Keyboard, focus, accessibility, and layout

No focus order, shortcut, widget geometry, tab order, selection, scroll, or table-column change. Existing tooltip
and wrapped compact-contract components expose the longer wording.

## Cross-feature visible behavior and isolation

Measurement, Time/Sync, Camera Clock/Media, Recording, Alignment, and FTP status remain unchanged. NTP health may
still show a warning/failure independently while Calibration Live accepts an app-timed pair.
