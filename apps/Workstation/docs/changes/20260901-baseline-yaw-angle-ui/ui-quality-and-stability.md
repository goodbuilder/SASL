# UI Quality and Stability

## Existing product-specific visual language and component tokens

Reuse compact `QGroupBox`, `EngineeringValue`, status-strip text, chart background/axis/text tokens,
and the existing P1/P2 series colors. No new card, pill, gradient, glow, shadow, or animation.

## Affected screens, components, text, tables, plots, and conditional controls

Add one `P1→P2 baseline / Pixhawk yaw` group below the raw baseline group. Its left side is a compact
numeric grid; its right side is a fixed-height top-view plot. It contains no controls.

## AI/template-like pattern rejection audit

The plot is justified by the directional relationship of three arrows. The numeric evidence stays
primary and the graphic remains an engineering aid, not a dashboard decoration.

## Dynamic value formats, precision, units, longest extents, and update rates

- Compass angles: `000.00°` through `359.99°`.
- Signed angles: `-180.00°` through `+179.99°`.
- Ages: two decimal seconds; GNSS pair delta: one decimal millisecond.
- States: `CURRENT`, `PARTIAL`, `STALE`, `WAITING`.
- Update cadence: the existing bounded 250 ms poll while Pixhawk is visible.

## Reserved geometry, overflow, wrapping, elision, and detail access

Reserve numeric label width from the widest signed value. Keep the plot height fixed and group
width responsive. Long source/contract text is bounded in the visible evidence line and retained
in tooltips/Inspector detail.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

Refresh text and plot state in place. Do not rebuild layouts, resize columns, call `adjustSize`,
steal focus, change table selection, or change the scroll position.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

Use text labels and solid/dashed line styles in addition to color. Verify 1366x768, 1600x900, and
1920x1080 offscreen construction plus representative font scaling.

## Before/after screenshots and dynamic-state sequence evidence

Baseline screenshot/state: raw attitude and baseline only. Required post-change sequence:
WAITING -> PARTIAL -> CURRENT -> STALE -> CURRENT, plus longest signed values under rapid updates.
Automated/offscreen visual evidence does not replace real-hardware manual verification.

Post-change offscreen evidence was captured at 1292x280 for the complete group and 650x250 for the
plot. Visual review found and corrected east-facing B/P2/E endpoint-label overlap. Automated tests
render all four states, exercise +/-180-degree rollovers, and verify the three target window sizes,
fixed plot/status/evidence extents, and stable scroll. Real hardware/manual evidence remains open.
