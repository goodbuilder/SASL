# Frontend Contract

## Controls and inputs

Existing P1/P2 role priority combos remain editable for BOARD, CAN1, CAN2, GPS1_MAG, and
GPS2_MAG. Add a read-only selection preview per side; no new blocking control.

## Payloads and commands

The combo payload remains `{role: raw_priority}` and is sent through the existing
`PixhawkParameterWorker(operation="mag_apply")`. The preview consumes the current PARAM slot
map, live sensor instance map, and latest actual selected ID; it does not write parameters.

## Visible states and transitions

Each side shows:

- `Topology`: instance → device/role map or `Topology readback waiting`;
- `Priority candidate`: the highest-priority sensor among stock-consumed instances 0..3;
- `Actual PX4`: latest `vehicle_magnetometer.device_id` and role;
- existing verification state and ATTITUDE age.

Candidate and actual may differ without disabling Apply.

## Errors and recovery actions

Partial or stale topology is shown as waiting/partial text. Existing Refresh MAG and Apply &
Verify remain the recovery actions. No modal warning is added for CAN2 instance 4.

## Keyboard, focus, accessibility, and layout

Combo focus and selection remain unchanged during live refresh. Preview updates in place with
bounded text and is selectable for copying IDs.

## Cross-feature visible behavior and isolation
