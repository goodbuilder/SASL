# UI Quality and Stability

## Existing product-specific visual language and component tokens

Reuse the existing P1/P2 status rows, severity colors, detail text, action buttons, and typography.
Do not add a new dashboard/card for this narrow state distinction.

## Affected screens, components, text, tables, plots, and conditional controls

Only current Pixhawk status/readiness surfaces and the side-specific recovery action change.
NTRIP, camera, calibration, and measurement result layouts stay unchanged.

## AI/template-like pattern rejection audit

No decorative cards, gradients, icons, generic progress dashboard, or redundant summary text.
The added UI is factual device evidence and one contextual action.

## Dynamic value formats, precision, units, longest extents, and update rates

- Ages: one decimal second after 10 s, two decimals below 10 s only in details.
- Qualification: fixed `mm:ss / 10:00`, updated at 1 Hz.
- Longest state label: `ETH MAVLINK RX STALLED`; reserve its width or elide detail, never state.

## Reserved geometry, overflow, wrapping, elision, and detail access

Status column reserves the longest label. Endpoint/cause details elide in-row with full tooltip or
details pane. Timer updates must not resize columns or move the recovery button.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

State changes preserve focus, selection, scroll, splitter, and window geometry. Conditional recovery
action occupies reserved space or appears without changing row height.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

State is always text plus color. Existing focus ring and target size apply. Verify Korean labels at
125%/150% font scaling and normal/minimum window widths in both supported themes if present.

## Before/after screenshots and dynamic-state sequence evidence

Pending implementation. Capture one sequence per side through QUALIFYING → LIVE → ATTITUDE STALE
and LIVE → ETH RX STALLED → RECOVERING without layout movement.
