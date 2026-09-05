# UI Quality and Stability

## Existing product-specific visual language and component tokens

Reuse the existing compact Qt `QTableView`, `QStandardItemModel`, status strings, fixed numeric
formatters, and Inspector contract. No new card, badge, plot, animation, color-only indicator,
or navigation context is needed.

## Affected screens, components, text, tables, plots, and conditional controls

Only the two existing `Magn` tables and their row Inspector details change. Add two stable columns:
`RAW yaw` and `PX4 yaw (selected)`, with a compact `Δyaw` in the selected PX4 cell or an adjacent
read-only field if width requires it. The existing raw fields and identity/source columns remain.

## AI/template-like pattern rejection audit

The change adds evidence columns to an existing engineering table and uses existing labels and
status conventions. It does not introduce generic dashboard decoration or invent a quality verdict.

## Dynamic value formats, precision, units, longest extents, and update rates

- Raw and PX4 yaw use fixed signed `±DDD.DD°` formatting, normalized to `[-180, 180)`.
- Difference uses the same wrapped signed format.
- Unavailable states use bounded strings such as `--`, `미선택`, or `identity 확인 대기`.
- The existing evidence refresh cadence remains; no per-packet widget creation or resize occurs.
- Stable-PX4 Shell samples retain a `px4-shell/sensor_mag/<instance>` topic/source, so the operator
  can distinguish diagnostic Shell readback from MAVLink compatibility telemetry.
- Unit/frame text stays in the existing hint/Inspector: raw value is `sensor-axis azimuth`, PX4
  value is firmware `ATTITUDE.yaw` in degrees.

## Reserved geometry, overflow, wrapping, elision, and detail access

Set explicit column policies/minimum widths during construction. Update cells in place so row
count, selection, scroll, splitter, and adjacent controls remain stable. If the table is narrow,
the existing horizontal scroll/detail Inspector remains the recovery path; no auto-resize-on-tick.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

The current row ordering and fixed row counts are preserved. Live values replace cell text only;
selected rows, Inspector refresh, table columns, and surrounding Pixhawk groups must not jump.
Test sign/degree rollovers and transitions between selected, stale, and waiting states.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

Keep text selectable, provide non-color state text, preserve the existing native font/theme, and
exercise 1920x1080 and 1366x768 with normal Qt font scaling. The two extra columns must remain
readable through horizontal scrolling and Inspector detail.

## Before/after screenshots and dynamic-state sequence evidence

Before: existing vector-only tables. Proposed sequence: WAITING -> complete RAW only -> active
identity plus fresh PX4 yaw -> STALE/mismatch -> fresh recovery. Automated Qt geometry/state tests
provide the first evidence; real-rig screenshots remain manual evidence.
