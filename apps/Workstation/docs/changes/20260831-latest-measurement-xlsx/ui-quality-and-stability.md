# UI Quality and Stability

## Existing product-specific visual language and component tokens

The Data workspace uses compact Qt group boxes, native push buttons, engineering values,
plain tables and `statusStrip`/`compactContract` labels. The change reuses those elements.

## Affected screens, components, text, tables, plots, and conditional controls

Only the sessionless `Data > Status > Files` action area gains a second compact row with
`Update Latest XLSX`, `Open Latest XLSX`, and one bounded status label. No navigation, table,
plot or modal changes.

## AI/template-like pattern rejection audit

No card shell, badges, gradients, animation, decorative metrics or new inline visual language.

## Dynamic value formats, precision, units, longest extents, and update rates

Status uses a stable `Latest XLSX · STATE · generation/time/rows` format. Full path/error is
in a tooltip; visible text is compacted. It refreshes only through the existing Data refresh.

## Reserved geometry, overflow, wrapping, elision, and detail access

The status label reserves a fixed two-line height and never resizes the action row. Long values
are elided/compacted without changing button geometry; tooltip and selectable text retain detail.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

Background state refresh does not steal focus, alter the stream table model selection, scroll,
window size, splitter or dock geometry. Buttons keep native keyboard order.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

Buttons have explicit accessible names/tooltips and remain usable at 1366x768 and supported
font scaling. State is conveyed by text, not color alone.

## Before/after screenshots and dynamic-state sequence evidence

Before: three existing Data file actions. After: those remain on the first row, two adjacent XLSX
actions occupy a fixed second row, followed by the fixed status line. Offscreen geometry/state-
sequence tests cover NOT_CREATED, WAITING, GENERATING, READY,
STALE and FAILED; final visual evidence will be recorded in verification.
