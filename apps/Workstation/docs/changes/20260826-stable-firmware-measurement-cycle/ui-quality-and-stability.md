# UI Quality and Stability

## Existing product-specific visual language and component tokens

The first shutdown-only slice adds no GUI element and reuses each app's existing top-level close
and finalization path.

## Affected screens, components, text, tables, plots, and conditional controls

None. The only new interaction is stable, line-oriented terminal output from the repository tool.

## AI/template-like pattern rejection audit

Not applicable: no visual surface is introduced.

## Dynamic value formats, precision, units, longest extents, and update rates

Terminal status uses one process per line with fixed field names. PID/start time/process group may
grow without changing an application window because output is not embedded in either GUI.

## Reserved geometry, overflow, wrapping, elision, and detail access

Not applicable to GUI geometry. Full identities remain available in terminal output without
elision.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

No GUI geometry or focus changes. The control request enters the existing close flow; it must not
raise, resize, or navigate the window before closing.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

No new GUI control. Terminal output cannot rely on color alone and must remain readable with color
disabled.

## Before/after screenshots and dynamic-state sequence evidence

Screenshots are not required for this slice because the visible application surfaces are
unchanged. Verification records terminal sequences for clear, live, writing/finalizing, declined,
graceful-survivor, forced, and unverified-identity states.
