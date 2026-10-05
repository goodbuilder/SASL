# Frontend Contract

## Controls and inputs

Measurement Start/Stop is always available except shutdown. Existing per-source connect/recover
controls remain available during Measurement; mutation commands stay locked.

## Payloads and commands

No new operator input. Recovery commands bind success to current-generation source evidence.

## Visible states and transitions

Measurement immediately RUNNING with source counts/details. Recording reports ACTIVE then
SAVED/PARTIAL/FAILED based only on path existence.

## Errors and recovery actions

Source failure names the source and reason. Camera recovery is manual; busy predecessor release is
automatic. Open Data Folder delegates file inspection to the OS.

## Keyboard, focus, accessibility, and layout

Existing shortcuts, focus order and geometry remain. Status is textual and bounded.

## Cross-feature visible behavior and isolation

One source, recorder or storage failure cannot change run admission or unrelated source controls.
