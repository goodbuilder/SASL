# Frontend Contract

## Controls and inputs

No new control. The primary Capture tab adds equal Left/Right overlay views and read-only status.

## Payloads and commands

Existing command IDs and payloads are unchanged. UI polls
`workstation_calibration_capture_snapshot()` and consumes existing `calibration_preview_changed`.

## Visible states and transitions

- Board per side: DETECTING, FOUND, NOT_FOUND, NO_FRAME, STALE, UNAVAILABLE
- Run row: inspection requests, stereo saves, K,D-only saves, failures
- Set row: L/R images against configured minimum, stereo stored candidates, side K,D-only counts
- Last row: authoritative controller status; full text in tooltip

## Errors and recovery actions

Snapshot or inventory errors show `-- / UNAVAILABLE`; Preview or a new capture can recover normal
states. No modal monitoring alert is introduced.

## Keyboard, focus, accessibility, and layout

Previews do not enter tab order. Status uses text plus color. Fixed rows/elision prevent focus,
button, selection or scroll movement under live updates.

## Cross-feature visible behavior and isolation

Camera Preview, YOLO, Measurement, Recording and Advanced remain available and unchanged when this
projection is unavailable.
