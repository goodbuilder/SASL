# UI Quality and Stability

## Existing product-specific visual language and component tokens

Reuse the native compact Qt group, buttons, status strip and table conventions already present in
Telemetry and Pixhawk. No cards, badges, animation or new styling.

## Affected screens, components, text, tables, plots, and conditional controls

Only `Devices > Telemetry > Drone Pixhawk MAVLink`: add `Find Network` beside `Find USB` and a
compact read-only candidate table/status plus `Use selected`.

## AI/template-like pattern rejection audit

No generic dashboard decoration, pills, gradient, glow, shadow, oversized heading or invented
metric. Every added value is device evidence: state, NIC, source IP, UDP port and system ID.

## Dynamic value formats, precision, units, longest extents, and update rates

Candidates are one scan snapshot, not a continuously resizing feed. IP is dotted decimal, port and
sysid are integers, and reason text is bounded/elided with full tooltip/detail access.

## Reserved geometry, overflow, wrapping, elision, and detail access

The existing action row retains one-line height. The bounded candidate area uses fixed/maximum
height and horizontal scrolling or elision so lower live-position controls do not jump.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

Refresh preserves URL focus, row selection and workspace scroll. Discovery completion does not
steal focus; selecting a candidate requires an explicit action.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

Buttons retain textual names and native keyboard focus. Table values do not rely on colour. Verify
normal DPI and the current maximized operator layout, plus long interface/error strings.

## Before/after screenshots and dynamic-state sequence evidence

Baseline screenshot captured 2026-09-02: USB-only action row and stale serial endpoint. After
implementation capture idle, scanning, candidate, empty, ambiguous and connected states.
