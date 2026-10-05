# Frontend Contract

## Controls and inputs

No new button, toggle, dialog, or editable setting is added. Retained-static fallback is automatic.

## Payloads and commands

None added or changed.

## Visible states and transitions

No persistent retained-static warning, banner, confirmation, sample-age display, or minimal
retained label is added. The existing ready-method projection simply reflects the changed backend
readiness result.

## Errors and recovery actions

Recovery is automatic on a newer valid live sample. Restart clears retained values. There is no
manual clear action.

## Keyboard, focus, accessibility, and layout

No new dynamic warning surface may resize panels, move controls, steal focus, reset selection, or
change scroll position.

## Cross-feature visible behavior and isolation

Camera, detection, recording, calibration, and other measurement-method controls keep their
existing state and geometry. Live recovery must not reset the selected method or steal focus.
