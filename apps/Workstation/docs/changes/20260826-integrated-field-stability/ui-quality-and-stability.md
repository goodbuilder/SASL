# UI Quality and Stability

## Existing product-specific visual language and component tokens

Retain the current dense operator workspace, status strip, existing health tones, tables, docks and
detail/tooltip patterns. This packet does not authorize a visual redesign.

## Affected screens, components, text, tables, plots, and conditional controls

Potentially Camera status/CAM tooltip, Time/Sync, Pixhawk/GNSS state, Data writer state and existing
Diagnostics Performance. Only a reproduced truth/interaction defect may change them.

## AI/template-like pattern rejection audit

No new dashboard shell, card grid, pills, gradient, glow, decorative metric or invented health score.
Status remains factual, side-specific and action-oriented.

## Dynamic value formats, precision, units, longest extents, and update rates

Reserve stable formats for `mm:ss / qualification`, HB/ATT/ANY age seconds, RTCP pair/SR-PC ms,
frame/drop counts, MiB/GiB, decoder name and long boundary-specific errors. UI refresh remains
throttled independently from data collection.

## Reserved geometry, overflow, wrapping, elision, and detail access

Use existing reserved regions and elision/tooltip/detail paths. Error strings may not resize Camera,
GNSS or status rows on every update.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

Rapid state transitions and 60-minute updates must preserve focus, row selection, manual scroll,
column widths, dock/splitter sizes and window geometry.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

Retain text plus color semantics and existing target sizes. Verify Korean/English long states at the
current target display and offscreen regression resolutions if UI changes.

## Before/after screenshots and dynamic-state sequence evidence

No screenshot is required before source changes that do not affect UI. Any visible change requires
before/after plus CONNECTING/QUALIFYING/LIVE/STALE/STALLED/ERROR/RECOVERING sequences.
