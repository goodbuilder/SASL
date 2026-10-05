# UI Quality and Stability

## Existing product-specific visual language and component tokens

Reuse the existing Calibration action bar, collapsed Auto settings, `statusStrip`, button labels,
tooltips, and per-camera logs. No new component or style token.

## Affected screens, components, text, tables, plots, and conditional controls

Only existing Auto tooltip/status text gains NAL1/exact-frame waiting reasons. No new table or plot.

## AI/template-like pattern rejection audit

No card, badge, modal, animation, icon, decorative surface, or generic dashboard pattern is added.

## Dynamic value formats, precision, units, longest extents, and update rates

Bounded phrases identify `NAL1 non-delta waiting` and `exact native frame expired`; existing valid
pair count and pair timing formats remain. Existing preview/timer throttles own update cadence.

## Reserved geometry, overflow, wrapping, elision, and detail access

The existing wrapping status area owns long messages. Detailed AU/PTS/key evidence remains in logs
and capture JSON; controls do not move.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

No widget creation/removal, focus transfer, selection reset, scroll mutation, or geometry change.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

Preserve existing text/non-color semantics and target sizes. Offscreen tests cover the longest new
waiting and failure text.

## Before/after screenshots and dynamic-state sequence evidence

No layout change is expected; automated state assertions are required. Real dual-camera cadence is
hardware/manual evidence, not a software screenshot claim.
