# UI Quality and Stability

## Existing product-specific visual language and component tokens

No new layout/control is proposed. Existing Data Status stream table and Latest XLSX workbook are
the visible surfaces. They already use compact engineering rows, explicit states/counts/errors and
native spreadsheet sheets.

## Affected screens, components, text, tables, plots, and conditional controls

- Data Status gains the automatically enumerated `gnss.baseline.center` stream row.
- Latest XLSX gains `GnssBaselineCenter` in its fixed sheet order and Summary row-count table.
- Telemetry, Measurement, toolbar, navigation, Inspector, Diagnostics and status-bar layout do not
  change.

## AI/template-like pattern rejection audit

PASS by proposal: no GUI component or style is added.

## Dynamic value formats, precision, units, longest extents, and update rates

Data Status uses existing state/count/error formatting. XLSX raw projection preserves numeric types,
exact large clocks, KST companion columns, explicit units in field names, and `#N/A` for absence.
Row production follows Drone GNSS rate, not UI cadence.

## Reserved geometry, overflow, wrapping, elision, and detail access

Existing model-driven Data table absorbs one fixed stream row. Existing XLSX row/column sharding
handles large datasets. No dynamic widget extent changes are introduced.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

The store snapshot may add one row at startup but live counters update in place. No focus,
selection, splitter, dock, window or Telemetry scroll behavior changes.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

No new GUI text beyond the stream key. Existing non-color state/error semantics apply.

## Before/after screenshots and dynamic-state sequence evidence

Verify Data Status at 1920x1080 and 1366x768 with the added stream row; no dedicated Telemetry
screenshot change is expected. Verify generated workbook sheet and Summary through XLSX XML tests.
