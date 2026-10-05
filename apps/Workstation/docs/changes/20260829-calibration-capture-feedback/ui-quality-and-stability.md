# UI Quality and Stability

## Existing product-specific visual language and component tokens

Reuse `CameraPreview`, `CameraPreviewPair`, `statusStrip`, compact native Qt layouts and existing
semantic theme colors. No cards, pills, gradients, animation or one-off QSS.

## Affected screens, components, text, tables, plots, and conditional controls

Only Calibration's primary Capture page gains a two-pane overlay and four fixed status rows.
Advanced preview/logs and all controls remain.

## AI/template-like pattern rejection audit

The design is image-dominant engineering feedback with compact factual rows, not a dashboard or
decorative metric-card surface.

## Dynamic value formats, precision, units, longest extents, and update rates

- detection age: one decimal second; updates with existing 2 Hz detector / 4 Hz status poll
- counters: non-negative integers or `--`
- longest recent message: elided single line with full tooltip
- labels always include state text; color is supplementary

## Reserved geometry, overflow, wrapping, elision, and detail access

Preview chrome and status rows have stable height. Long details elide instead of wrapping; full
text remains in tooltip. Counter digit rollover must not move actions.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

Previews are non-focusable. Existing button tab order, Advanced state, Photo set selection and
scroll are preserved. Left/Right preview widths remain equal.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

Verify non-color states, keyboard focus, standard Qt fonts, 100/125/150% scale where feasible,
and 1366x768, 1600x900, 1920x1080 layouts.

## Before/after screenshots and dynamic-state sequence evidence

Baseline: `/tmp/workstation-calibration-baseline-1000x700.png`.

After: `/tmp/workstation-calibration-feedback-full-1366x768-v2.png` (empty/no-frame) and
`/tmp/workstation-calibration-feedback-populated-1366x768.png` (Left FOUND 80, Right NOT_FOUND,
two counter scopes and duplicate-pose recovery text). Automated tests cover STALE, unreadable root
and long-message elision. Real-camera pixels remain a hardware/manual condition.
