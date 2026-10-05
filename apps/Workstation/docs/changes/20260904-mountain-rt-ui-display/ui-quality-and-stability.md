# UI Quality and Stability

## Existing product-specific visual language and component tokens

Reuse the existing compact `QTableWidget` candidate review and read-only
`QPlainTextEdit` detail component. No new surface, badge, color, animation, or decorative
dashboard element is introduced.

## Affected screens, components, text, tables, plots, and conditional controls

Only the Alignment Candidate detail text changes. The candidate table columns, buttons,
method tabs, and preview layout remain unchanged.

## AI/template-like pattern rejection audit

The change is a plain engineering readout using existing text/detail conventions. It adds
no generic cards, pills, gradients, shadows, or invented status indicators.

## Dynamic value formats, precision, units, longest extents, and update rates

- Rotation entries: signed fixed `9` decimal places.
- Translation entries: signed fixed `9` decimal places with `m` in the field name.
- Frame: explicit `OpenCV optical RDF` and equation `X_right = R X_left + t`.
- Values update only when a snapshot refreshes or the selected candidate changes, not on a
  high-rate timer.

## Reserved geometry, overflow, wrapping, elision, and detail access

The existing fixed-height read-only detail pane remains bounded and scrollable. Matrix
rows use a stable one-row-per-line format; no resize-to-content or table-column mutation
is introduced.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

`set_candidates()` continues preserving selection by candidate ID. Numeric detail refresh
does not change candidate table columns, focus, splitter sizes, or surrounding controls.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

Existing native text widgets, read-only semantics, keyboard selection, and theme are
retained. Labels carry frame/unit meaning, so color is not needed to interpret the value.
Long values remain readable through the existing scrollable detail control.

## Before/after screenshots and dynamic-state sequence evidence

Automated UI assertions will verify the selected Mountain candidate detail. A live
hardware screenshot is not required for this read-only projection and remains unverified
until the operator runs the actual Alignment workflow.
