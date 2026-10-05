# UI Quality and Stability

## Existing product-specific visual language and component tokens

기존 Qt `QLabel`, tooltip, compact contract/status strip을 재사용한다. 새 card, badge, color,
icon, dialog, 모달, 제어와 스타일을 추가하지 않는다.

## Affected screens, components, text, tables, plots, and conditional controls

- Calibration shared Capture의 `Max Live pair Δt` tooltip/설명
- Legacy/Advanced Calibration의 동일 timing tooltip
- Capture status에 표시되는 `pair_time_basis`/detail
- Compute 사진 wall-clock 테이블은 구조/열/시각 원천을 변경하지 않음

## AI/template-like pattern rejection audit

제어·레이아웃을 추가하지 않으므로 해당 없음. 기존 제품 용어와 구조를 보존한다.

## Dynamic value formats, precision, units, longest extents, and update rates

- Δt: signed seconds에서 계산, 상태 detail은 기존처럼 ms 1자리
- 임계: 기존 ms 1자리, 0–1000 ms
- 기준 문자열: `app-receive-monotonic`
- 표시 갱신은 capture event 당 1회로 기존과 같음

## Reserved geometry, overflow, wrapping, elision, and detail access

새 문구는 기존 wrap 가능 label/tooltip 안에서만 변경한다. 버튼, preview,
tab, table 크기는 변경하지 않으며 live 갱신으로 geometry가 달라지지 않는다.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

제어가 추가/제거되지 않으므로 focus order, selection, scroll, column policy,
dock/splitter/window geometry는 모두 보존된다.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

기존 tooltip/wrapped-label 정책을 사용하고 color-only 의미를 추가하지 않는다.
Offscreen Qt smoke/static text 테스트로 조건부 제어와 geometry 무변경을 확인한다.

## Before/after screenshots and dynamic-state sequence evidence

시각 구조 변경이 없어 screenshot은 필수가 아니다. Offscreen UI에서 tooltip/레이블과
capture status의 normal/missing/over-limit 문구를 검증한다.
