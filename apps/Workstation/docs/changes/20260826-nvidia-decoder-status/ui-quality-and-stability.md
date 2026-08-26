# UI Quality and Stability

## Existing product-specific visual language and component tokens

기존 `EngineeringValue`, `StatusIndicator`, Camera workspace Status group, Diagnostics plain-text
view를 재사용한다. 새 badge/card/dashboard를 만들지 않는다.

## Affected screens, components, text, tables, plots, and conditional controls

- Camera workspace `Decoder LEFT/RIGHT`.
- compact/global CAM L/R tooltip/detail.
- Diagnostics > Performance read-only evidence.

## AI/template-like pattern rejection audit

새 card, gradient, icon, animation, invented metric 없음. NVIDIA logo/장식도 추가하지 않는다.

## Dynamic value formats, precision, units, longest extents, and update rates

- `GPU · nvh264dec`
- `CPU · avdec_h264 (selected)`
- `CPU fallback · avdec_h264`
- `Unavailable · <bounded reason>`
- GPU detail: fixed model/driver text; utilization은 기존 Performance polling cadence가 있을 때만
  보조 진단으로 사용하며 readiness metric으로 만들지 않는다.

## Reserved geometry, overflow, wrapping, elision, and detail access

기존 EngineeringValue 영역을 유지하고 긴 reason은 source/tooltip 또는 Performance에 둔다.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

Decoder state change가 preview splitter, window, current focus/selection/scroll을 바꾸지 않는다.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

GPU/CPU/fallback은 text로 표시하고 color-only 의미를 금지한다. 기존 theme/focus를 재사용한다.

## Before/after screenshots and dynamic-state sequence evidence

Camera workspace에서 unknown → GPU active → CPU fallback → stopped를 같은 geometry로 캡처한다.
