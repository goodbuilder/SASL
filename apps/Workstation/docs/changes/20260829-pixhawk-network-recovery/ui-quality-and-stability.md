# UI Quality and Stability

## Existing product-specific visual language and component tokens

기존 compact QTableView, EngineeringValue, native QPushButton, statusStrip과 theme token을 재사용한다.

## Affected screens, components, text, tables, plots, and conditional controls

Pixhawk top commands, Connection 아래 assignment summary와 Network/USB candidate 영역만 바뀐다.
plot과 telemetry/parameter/baseline 표는 바뀌지 않는다.

## AI/template-like pattern rejection audit

새 dashboard/card/pill/gradient/modal workflow를 만들지 않는다.

## Dynamic value formats, precision, units, longest extents, and update rates

NIC/link/local IPv4/Assigned candidate 열은 discovery 완료 시에만 갱신한다. P1/P2의
State/NIC/Local IPv4/Endpoint/Route는 side별 assignment summary에서 독립적으로 갱신한다.
liveness refresh는 candidate model을 rebuild하지 않는다.

## Reserved geometry, overflow, wrapping, elision, and detail access

표 높이와 header resize policy를 초기화 시 고정하고 긴 interface/route는 elide하며 Inspector에서
전체 문자열을 제공한다.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

stable candidate ID로 refresh 뒤 selection을 복원하고 scroll/focus를 변경하지 않는다.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

텍스트 상태를 사용하고 색만으로 판정하지 않는다. 1366x768/1600x900/1920x1080과 font scale을
offscreen/visual로 확인한다.

## Before/after screenshots and dynamic-state sequence evidence

기준 screenshot은 2026-08-29 `/tmp/workstation_pixhawk_plan.png`. 구현 후
`/tmp/pixhawk_network_after_1366_v2.png`, `/tmp/pixhawk_network_after_1600x900.png`,
`/tmp/pixhawk_network_after_1920x1080.png`을 확인했다. Connection 바로 아래 assignment/table,
고정 header 폭, horizontal scroll, tooltip과 disabled Assign 상태가 clipping 없이 유지된다.
DISCOVERING/FOUND/ASSIGNED/ROUTE ERROR는 source-backed text이며 color-only 의미가 없다.

The amended assignment-centric capture must prove that the candidate row contains no paired
P1/P2 route columns and that each side card exposes only its own route and endpoint.

Verified in `/tmp/pixhawk_network_assignment_cards_20260829.png`: the candidate table has five
candidate-only columns, P1/P2 cards are independent two-row grids, and long endpoint/route values
remain readable without overlap.
