# Frontend Contract

| Control | Input | Transition / output | Failure behavior |
|---|---|---|---|
| Refresh Simulation.env | File path from `SIM_CONFIG`, then strict literal block | Displays action, run ID, output ID, MC count, repo-relative PX4/YOLO paths, and config SHA | Invalid/missing/duplicate/unsafe values are shown; Start remains disabled. |
| Prepare pinned YOLO runtime | Current `YOLO_TORCH_BACKEND`, package pins, and repo-relative venv path reread from `.env` | Starts `ensure_yolo_runtime.sh` in a child process and appends bounded output | Confirmation precedes downloads; process failures remain visible; existing venv is not silently removed. |
| Run configured action | Validated `.env` action; run actions show an action-specific confirmation | Starts `control_cli.py` using `QProcess`; outputs lifecycle/QC events and slot counts | Duplicate starts disabled; output root never overwritten; failed start/status is visible. |
| Stop safely | Active child process | Sends SIGINT to the controller; controller forwards it and the runner preserves/finalizes the attempt | Stop button disables after request; result is the persisted QC state, not the click. |

The panel's progress line displays valid/failed/running/planned counts. The bounded log is the source for active run ID/attempt ID and phase events. Closing the plugin requests a safe stop. A normal result is confirmed only by process completion plus fresh persisted QC/analysis readback.

## Controls and inputs

## Payloads and commands

## Visible states and transitions

## Errors and recovery actions

## Keyboard, focus, accessibility, and layout

## Cross-feature visible behavior and isolation
