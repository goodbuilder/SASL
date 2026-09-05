# Implementation Plan

## Approval status

Approved by the user on 2026-09-02, including the Apply/Save URL-preservation amendment.

## Slices and dependencies

1. Pure candidate/result validation and immutable state contract.
2. Bounded passive network discovery adapter/worker with timeout, malformed and ambiguity paths.
3. DroneTelemetryController lifecycle, generation, snapshot and shutdown integration.
4. Backend facade and CommandRegistry handoff.
5. Telemetry controls/table, stable refresh, accessibility and enablement.
6. Apply/Save accepted-value projection, dirty-edit preservation, persistence/restart compatibility
   and existing manual/USB fallback regression.
7. Real application launch, visual states and current `sysid=3` hardware connection.

## File and component map

Expected: `workstation_core/` network candidate logic, `device_runtime_workers.py` or a focused
worker, `drone_telemetry_controller.py`, `standalone_backend.py`, `main_window.py`, `workspaces.py`,
`standalone_gui.py`, focused tests, `README.md`, `docs/ui/01_USER_WORKFLOW.md`,
`docs/ui/06_STATUS_MODEL.md`, and `docs/ui/07_COMMAND_MODEL.md`.

## Migration and rollback steps

No schema migration. Preserve the existing saved URL. Runtime rollback is manual URL/Find USB;
code rollback removes only the optional discovery surface.

## Stopping conditions

Stop and re-approve if implementation requires host NetworkManager mutation, PX4 configuration,
automatic connect/disconnect, a second MAVLink connection owner, dependency installation, or any
change to P1/P2/Camera/Measurement behavior.
