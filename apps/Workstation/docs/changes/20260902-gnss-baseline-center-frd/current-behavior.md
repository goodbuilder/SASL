# Current Behavior

## Version and worktree context

- Baseline Git identity: `3288aee5f5fb7af5ab30024f93e838f76724a0b0`.
- The Workstation worktree is intentionally dirty with operator-owned and previously approved
  changes, including the 2026-09-02 Pixhawk topology slice. This packet must not normalize or
  revert them.
- Relevant current files are already modified in the worktree for prior approved work:
  `README.md`, `drone_telemetry_controller.py`, `workspaces.py`, and numbered UI documents.
- Baseline focused command:
  `QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q tests/test_workstation_telemetry.py`
  -> `6 passed in 0.66s`.

## Reproduction

1. Open `Devices > Telemetry` with P1, P2, and Drone positions present.
2. Observe the three source rows in `Live position`.
3. Observe `Ideal disparity from three GNSS positions + zoom`.
4. The group shows P1/P2 baseline length, scalar `GNSS target range`, focal length, and ideal
   disparity, but it has no Forward, Right, or Down midpoint-relative components.

Offscreen current-code baseline at 1920x1080 is `/tmp/gnss-baseline-center-before.png`:
Telemetry workspace 1344x790 px; existing ideal-disparity group 1328x235 px.

## Observed frontend behavior

- `TelemetryWorkspace` owns a connection form, three-row position table, RTCM delivery group, and
  six-row ideal-disparity form.
- `GNSS target range` renders the scalar midpoint-to-Drone distance and source text.
- The page is already in the central scrollable Workstation host and refreshes visible values in
  place. No user-entered target-distance control exists.
- Values use ordinary Qt GroupBox/QLabel/QTableView styling, compact 4-8 px spacing, explicit
  units, factual WAITING/READY text, and no decorative dashboard elements.

## Observed backend and persistence behavior

- `DroneTelemetryController.drone_telemetry_snapshot()` reads selected P1/P2 rover states through
  `_selected_rover_gps_state()` and Drone from `latest_gps["drone"]`.
- `target_distance_from_gps()` converts the three WGS84/MSL positions to a local tangent
  approximation and returns only the Euclidean distance from the P1/P2 GNSS midpoint.
- `estimate_ideal_disparity()` applies the existing 2.0 s age limit when called by the controller,
  but does not gate the three-source result on their maximum timestamp spread.
- Fix type does not gate the reference. `RTK_FIXED`, `RTK_FLOAT`, `3D_FIX`, or `NO_FIX` remains
  evidence in the position table.
- No Telemetry geometry value is persisted by this path. It owns no extra worker, queue, timer,
  file, settings field, or device.

## Documentation, code, test, and runtime conflicts

- README accurately documents scalar `Z` from the P1/P2 midpoint to Drone GNSS and says it is a
  geometry reference, not image disparity or pose.
- The canonical product coordinate contract reserves `baseline_center_frd` for the midpoint of
  optical centers with camera-derived axes. A GNSS antenna-midpoint result cannot reuse that frame
  name without becoming a documentation/code conflict.
- The requested display is therefore compatible only when named and annotated as a separate
  GNSS-derived local-level frame.
