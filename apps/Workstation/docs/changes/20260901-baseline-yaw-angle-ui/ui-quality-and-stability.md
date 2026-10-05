# UI Quality and Stability

## Existing product-specific visual language and component tokens

Reuse compact `QGroupBox`, `EngineeringValue`, status-strip text, chart background/axis/text tokens,
and the existing P1/P2 series colors. No new card, pill, gradient, glow, shadow, or animation.

## Affected screens, components, text, tables, plots, and conditional controls

Extend the existing `GNSS BASELINE_CENTER F/R/D/Rg` group on `Devices > Telemetry` with two read-only
columns: `Baseline↔Drone angle°` and `90° error°`. It contains no controls or new plot. The angle is
the acute horizontal angle between the P1→P2 baseline line and midpoint→Drone GNSS vector.

## AI/template-like pattern rejection audit

The numeric table is the primary engineering evidence; a separate attitude/yaw plot is not used for
Drone GNSS installation alignment.

## Dynamic value formats, precision, units, longest extents, and update rates

- GNSS baseline↔Drone angle: `00.00°` through `90.00°`.
- 90° error: `00.00°` through `90.00°`.
- Ages: two decimal seconds; GNSS pair delta: one decimal millisecond.
- States: `CURRENT`, `PARTIAL`, `STALE`, `WAITING`.
- Update cadence: the existing bounded 250 ms poll while Pixhawk is visible.

## Reserved geometry, overflow, wrapping, elision, and detail access

Reserve numeric table columns from the widest angle/error values. Keep the existing table height
fixed and group width responsive; the existing horizontal scroll handles narrow widths. Long
source/contract text is bounded in the visible status line and retained in tooltips.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

Refresh table values in place. Do not rebuild layouts, resize columns, call `adjustSize`, steal focus,
change table selection, or change the scroll position.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

Use text labels and solid/dashed line styles in addition to color. Verify 1366x768, 1600x900, and
1920x1080 offscreen construction plus representative font scaling.

## Before/after screenshots and dynamic-state sequence evidence

Baseline screenshot/state: F/R/D/Rg only. Required post-change sequence:
WAITING -> CURRENT -> STALE -> CURRENT, 0°/90°/long-value angle cases, and rapid updates.
Automated/offscreen visual evidence does not replace real-hardware manual verification.
