# UI Quality and Stability

## Existing product-specific visual language and component tokens

Reuse the existing compact Pixhawk engineering cards, `statusStrip` labels, semantic priority
combo controls, and fixed table typography. No new cards, gradients, badges, or modal workflow
are introduced.

## Affected screens, components, text, tables, plots, and conditional controls

Only the existing P1/P2 MAG Priority cards change. A bounded preview label/table is added below
the current controls and status. It uses fixed labels and numeric IDs rather than long prose.

## AI/template-like pattern rejection audit

## Dynamic value formats, precision, units, longest extents, and update rates

Device IDs and instance indices are integers; priority uses the existing PX4 semantic label plus
raw value. The topology preview is updated at the existing runtime status cadence and is bounded
to one compact wrapped block per side. No live update may rebuild the card or alter combo values
unless a fresh PARAM status explicitly does so.

## Reserved geometry, overflow, wrapping, elision, and detail access

The preview uses a fixed/minimum height and word wrapping. Long topology maps are elided to a
bounded summary with the full mapping available through the existing selectable status text or
tooltip. Combo rows, Apply buttons, and the outer workspace scroll position remain stable.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

The preview is text-readable and does not rely on color alone: labels explicitly say `Candidate`
and `Actual`. Existing combo accessible names and button sizes remain unchanged.

## Before/after screenshots and dynamic-state sequence evidence
