# UI Quality and Stability

## Existing product-specific visual language and component tokens

기존 IBM/Carbon 계열 운용 UI와 현재 status/log/error 표면을 그대로 사용한다.

## Affected screens, components, text, tables, plots, and conditional controls

새 widget/layout은 추가하지 않는다. 기존 decoder combo와 카메라 status/log에 fail-closed 또는
Auto fallback 복구 문구만 추가하고, Qt 이전 실패는 terminal에 표시한다.

## AI/template-like pattern rejection audit

새 card, badge, gradient, animation, modal shell을 도입하지 않는다.

## Dynamic value formats, precision, units, longest extents, and update rates

`PASS/WARN/FAIL`, requested/effective decoder와 고정 복구 명령은 CLI line 단위이다. GUI의
기존 status throttling과 geometry는 유지한다.

## Reserved geometry, overflow, wrapping, elision, and detail access

기존 log/status label의 wrapping/scroll 동작을 재사용한다.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

control 생성/배치/selection을 변경하지 않으므로 기존 geometry와 focus를 보존한다.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

상태는 색이 아닌 명시적 텍스트와 복구 문장으로 전달한다.

## Before/after screenshots and dynamic-state sequence evidence

시각 레이아웃 변경이 없어 screenshot은 적용하지 않는다. offscreen tests에서 상태 문구와
카메라 시작 거부를 확인하고 실제 hardware 상태 표시는 수동 검증으로 남긴다.
