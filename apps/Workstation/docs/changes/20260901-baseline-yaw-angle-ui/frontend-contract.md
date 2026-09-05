# Frontend Contract

## Controls and inputs

No controls. Inputs are the selected P1/P2 GNSS positions and the primary Drone GNSS position.
Drone PX4 ATTITUDE/yaw is not an input.

## Payloads and commands

None. The workspace reads `DroneTelemetryController.drone_telemetry_snapshot()` and its existing
`gnss_baseline_center_frd` mapping.

## Visible states and transitions

- `CURRENT`: finite three-GNSS geometry, all source ages <=2.0 s, and timestamp spread <=1000 ms.
- `STALE`: finite geometry remains visible but a source is old/future or timestamp spread exceeds
  the existing bound.
- `WAITING`: a GNSS position is missing/invalid, the horizontal baseline is undefined, or the
  midpoint-to-Drone horizontal vector has no direction.

Visible fields are F/R/D/Rg, `Baseline↔Drone angle`, `|90°−angle|`, selected receiver/fix states,
three-source ages and timestamp spread.

## Errors and recovery actions

Missing/non-finite/zero-horizontal inputs fail closed to `--`/WAITING with a factual reason.
Stale finite values remain visible with the existing STALE status and tooltip. Recovery occurs
automatically when fresh GNSS source evidence returns; no local retry is offered.

## Keyboard, focus, accessibility, and layout

The read-only table is non-selectable as before; status meaning is available in text and tooltips.
Fixed columns/table height prevent live-layout motion; the existing outer horizontal scroll is the
recovery path for narrow widths.

## Cross-feature visible behavior and isolation

Raw attitude and baseline groups remain unchanged. A missing/stale Drone GNSS source affects only
the GNSS angle row; camera, Base GNSS/NTRIP, Alignment R,t, Measurement, recording, and Diagnostics
remain usable.
