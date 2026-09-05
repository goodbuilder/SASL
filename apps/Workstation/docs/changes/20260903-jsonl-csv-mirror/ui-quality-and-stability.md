# UI Quality and Stability

## Existing product-specific visual language and component tokens

The Workstation uses compact native Qt tables, fixed-height status strips, factual state text, and
existing `Data > Status` controls. This change reuses the current `QTableView`/table model and
status conventions; it introduces no cards, pills, gradients, animations, modal workflow, or new
visual theme.

## Affected screens, components, text, tables, plots, and conditional controls

Only the sessionless `RecordingWorkspace`/Data Status stream table changes. The table will identify
the JSONL path and its paired CSV path, plus the existing stream state and a compact CSV status.
Latest XLSX buttons, measurement controls, navigation, plots, and legacy review remain unchanged.

## AI/template-like pattern rejection audit

No generic dashboard surface or decorative status badge is added. The CSV state is factual text in
the existing engineering table, and path details remain accessible through the normal table/tooltips
and the existing Open Data Folder action.

## Dynamic value formats, precision, units, longest extents, and update rates

Paths are filesystem strings and states are bounded tokens (`IDLE`, `ACTIVE`, `FAILED`, `RETRYING`,
`CLOSED`). Counts remain integers. Record values are not rendered live in the table; no coordinate,
unit, precision, or timestamp display is changed. Storage refresh keeps the existing polling cadence.

## Reserved geometry, overflow, wrapping, elision, and detail access

The table keeps its existing fixed maximum height and row count behavior. Long CSV paths/errors are
compactly displayed with the existing `_compact_text` convention and remain available in the
existing full-detail path/error surface. No live refresh may resize the workspace, move buttons, or
reset table selection/scroll.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

The table model is updated in place, as today. Refresh does not steal focus or change navigation;
the existing fixed action rows and status strip remain in the same layout. Column headers are
initialized once and use the current compact table policy; horizontal overflow is preferable to
clipping a path.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

Reuse native text, keyboard focus, tooltips, and existing accessible buttons. Verify the changed
headers and status text at the normal 1920×1080 and 1366×768 layouts, offscreen Qt, long paths,
permission errors, and Korean/English text without color-only meaning.

## Before/after screenshots and dynamic-state sequence evidence

Automated UI evidence will assert the fixed table/status geometry and path/state values. A visual
offscreen or manual review will inspect `NOT_CREATED/ACTIVE/FAILED/RETRYING` transitions and ensure
the existing Data controls do not move. A screenshot is not required for the backend acceptance,
but any available visual evidence will be recorded in the packet verification.
