# StereoWIRIS Design System

- Status: Draft
- Depends on: 00–07 design documents

## 1. Visual Principle

> Visual emphasis must follow engineering importance.

Normal operation is visually quiet. Warnings/errors receive appropriate emphasis. Commands look actionable. Read-only values do not look editable. Decoration is subordinate to data.

## 2. Character

Compact, precise, technical, calm, structured, persistent. Modern engineering software, not retro industrial software and not a consumer dashboard.

## 3. Color Roles

Use centralized semantic tokens: APP_BG, PANEL_BG, PANEL_ALT_BG, PANEL_BORDER, TEXT_MAIN, TEXT_MUTED, ACCENT, INFO, SUCCESS, WARNING, ERROR, CRITICAL.

Do not saturate healthy operation with bright green. Never rely on color alone for critical state.

## 4. Surfaces / Borders / Radius

Use few surface levels, subtle 1 px separators, little/no radius (0–3 px), no heavy shadows, bevels, gradients, glassmorphism, or floating card stacks.

## 5. Typography

Neutral sans-serif with strong numeric readability. Starting sizes:
- application: 12–13 px
- tree/table: 11–13 px
- section header: 12–13 px semibold
- workspace title: 13–15 px semibold
- status: 11–12 px

Use consistent precision and numeric alignment.

## 6. Density

Initial targets:
- Toolbar: ~30 px
- Status Bar: ~24 px
- Tree row: ~24 px
- Table row: 24–26 px
- Input: ~26 px
- Panel margin: ~6 px
- Section spacing: ~6 px
- Border radius: 0–2 px

Validate with screenshots; values are not absolute.

## 7. Toolbar / Menu

Toolbar compact, flat, grouped by separators. Icon size ~16–20 px. Menu bar remains conventional/native.

## 8. Navigation

Compact text-first tree, subtle selection, small icons, no touch-sized tiles or large pills.

## 9. Inspector

Professional property panel with aligned Label | Control | Unit rows and compact section headers such as CONNECTION, IMAGE, MEASUREMENT, ADVANCED. Avoid large cards.

## 10. Tables

Tables are first-class engineering UI: compact headers, right-aligned numeric columns, left-aligned text, subtle separators, clear selection, units in headers where homogeneous.

## 11. Plots

Neutral background, explicit axes/units, compact legends, consistent source mapping, no reliance on color alone.

## 12. Camera View

Image-dominant. Engineering overlays only. Compact readable overlays. Avoid decorative frames.

## 13. Empty / Warning / Error

Empty states are factual and quiet. Warnings attract attention without implying full failure. Errors provide specific evidence and do not turn the whole application red.

## 14. Status Bar

Compact, visually quiet, text/semantic cue based, not a row of large colored badges.

## 15. Diagnostics

Should resemble log viewer/terminal/engineering table, not an alert dashboard.

## 16. Local Tabs

Compact and visually secondary to Navigation.

## 17. Accessibility

Sufficient contrast, non-color cues, visible keyboard focus, scalable native Qt fonts, clear disabled states.

## 18. Screenshot Rejection Criteria

Reject if Workspace is not dominant, controls are touch-first, status colors dominate normal operation, rows are unnecessarily tall, Inspector looks like SaaS cards, toolbar consumes excessive height, spacing is inconsistent, or UI looks like a generic web dashboard.

## 19. Qt Styling Strategy

Prefer standard Qt widgets and centralized theme/style tokens. Avoid per-widget inline QSS and custom painting unless necessary.

## 20. Summary

> If a visual element does not improve structure, state recognition, or engineering comprehension, it probably does not belong.
