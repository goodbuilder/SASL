# UI Quality and Stability

## Existing product-specific visual language and component tokens

Reuse compact Run labels, EngineeringValue/status indicators and existing command buttons.

## Affected screens, components, text, tables, plots, and conditional controls

Measurement Run/source status, Recording state and enablement of existing recovery controls.

## AI/template-like pattern rejection audit

No new cards, badges, dashboard, animation or decorative styling.

## Dynamic value formats, precision, units, longest extents, and update rates

Bound source summaries; full reasons remain in tooltips/detail. Existing refresh cadence remains.

## Reserved geometry, overflow, wrapping, elision, and detail access

Keep current fixed Run region and compact-text limits.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

State refresh must not rebuild widgets, move controls, reset focus/selection/scroll or resize panes.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

Text communicates state independently of color; existing focus order and target sizes remain.

## Before/after screenshots and dynamic-state sequence evidence

Required at implementation verification for RUNNING with 0/1/2 camera sources and storage failure.
