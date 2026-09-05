# UI Quality and Stability

## Existing product-specific visual language and component tokens

The Telemetry page uses native compact Qt GroupBox/QGridLayout/QTableView/QLabel components,
existing theme tokens, 4-8 px spacing, explicit engineering units, text-plus-state semantics, and
selectable evidence. The change reuses those components and adds no card, pill, icon, gradient,
shadow, animation, modal, or custom painting.

## Affected screens, components, text, tables, plots, and conditional controls

- One new GroupBox immediately below `Live position`: `GNSS BASELINE_CENTER F/R/D/Rg`.
- One non-editable, non-selectable, one-row QTableView with columns:
  `State`, `Forward m`, `Right m`, `Down m`, `Range Rg m`, `Max age s`, `Max Δt ms`,
  `Fix P1/P2/Drone`.
- One bounded status/detail line and one static coordinate-contract hint.
- Existing connection, position, RTCM, ideal-disparity, navigation, Inspector, Diagnostics, and
  status-bar geometry/style remain unchanged except for normal downward displacement in the
  existing scroll area.

## AI/template-like pattern rejection audit

PASS by design: the feature is a compact engineering table tied directly to source evidence. It
does not introduce generic dashboard cards, marketing text, invented scores, decorative charts,
or color-only status.

## Dynamic value formats, precision, units, longest extents, and update rates

- F/R/D: signed fixed three decimals, e.g. `-12345.678`; missing `--`.
- Rg: non-signed fixed three decimals; missing `--`.
- Maximum age: two decimals in seconds; maximum timestamp spread: one decimal in milliseconds.
- Fix column: bounded `P1 <label> / P2 <label> / D <label>` using existing labels.
- State: `CURRENT`, `STALE`, or `WAITING`; full reason retained in tooltip/detail.
- Updates use the existing visible-workspace status cadence (nominally 4 Hz) and replace item text
  in place; no resize-to-contents occurs during live refresh.

## Reserved geometry, overflow, wrapping, elision, and detail access

- Header widths are initialized once; last evidence column stretches. The table reserves header,
  one row, and horizontal-scrollbar allowance at construction.
- The state/detail line has a fixed single-line height and a bounded visible string; full text is
  available through tooltip and text selection where appropriate.
- The static coordinate hint may wrap within its reserved group region but never changes at
  runtime. Long values do not resize the table, group, docks, splitter, or window.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

- Table selection is disabled and values are read-only, so 4 Hz updates cannot steal focus or
  reset selection.
- The existing outer scroll position is not programmatically changed. No conditional show/hide or
  group expansion occurs across WAITING/STALE/CURRENT.
- P1/P2/Drone connection controls and the live-position table stay above the new group.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

- Text labels and units carry all semantics; color is optional and not authoritative.
- Native focus indicators remain on existing controls; the new table is inspection-only.
- Verify 1920x1080 primary, 1600x900, and 1366x768 with docks open/collapsed as documented, plus
  long signed values and 125/150% font scaling where available.

## Before/after screenshots and dynamic-state sequence evidence

- Before: `/tmp/gnss-baseline-center-before.png`, automated-fake/offscreen, 1920x1080.
- After approval/implementation: capture WAITING, CURRENT sentinel, STALE long reason, and
  side-reversed sign states without changing group/table height, focus, or outer scroll.
