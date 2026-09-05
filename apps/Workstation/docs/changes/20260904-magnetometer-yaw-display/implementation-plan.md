# Implementation Plan

## Approval status

Approved by the user on 2026-09-04 (`응`) for the exact scope in `approval-summary.md`, including
the `apps/Workstation/README.md` documentation target. Implementation is allowed.

Implementation and live verification are complete. The approved amendments add the consolidated five-row IMU-table
projection, plus bounded Stable PX4 Shell `sensor_mag`/`vehicle_magnetometer` readback; no private
stream request or firmware change is required.

## Slices and dependencies

1. Pure calculation: finite raw sensor-axis yaw and independent wrapped angular difference — PASS.
2. Evidence projection: add raw yaw, active identity, PX4 yaw, freshness, and correlation reason — PASS.
3. UI: add stable columns/detail formatting to both Magn tables and Inspector, and append the
   existing DEBUG_VECT RM3100 rows to the IMU view — PASS.
4. Focused tests: normal, missing, stale, near-zero, non-selected, identity mismatch, rollover,
   and rapid refresh/selection stability.
   PASS for the new focused cases, including source/topic preservation and RM3100 row selection.
5. Dependent regression: existing Pixhawk magnetometer, telemetry, calibration, and UI tests;
   launcher/readiness check; update confirmed README and product docs after approval.
   Pixhawk-dependent run has unrelated pre-existing failures; launcher check PASS.
6. Stable-firmware vertical slice: Shell command contract, parser, state routing, active-ID handoff,
   five-row/RAW/PX4/Δyaw Qt projection, concurrent manual-command yield, timeout/stale behavior,
   then hardware/manual P1/P2 Shell evidence.

## File and component map

- `workstation_core/` new small pure magnetometer-yaw helper (exact path chosen after approval).
- `evidence.py` additive magnetometer row projection.
- `workspaces.py` existing IMU/CAN tables and Inspector.
- `device_runtime_controller.py` bounded per-side Shell polling lifecycle and exclusive command handoff.
- `standalone_gui.py` Shell result handoff into existing sensor presentation state.
- `tests/` focused pure/evidence/UI regression coverage.
- `README.md` and selected `docs/ui/*` only after approval/README confirmation.

## Migration and rollback steps

No migration. Roll back the additive read-model/UI changes together; existing raw telemetry and
PX4 priority behavior remain usable.

## Stopping conditions

Stop and return for amended approval if exact identity cannot be represented, if implementation
would need a new stream/firmware request, if PX4 calibration math cannot be independently matched,
or if unrelated dirty changes make a narrow patch unsafe.
