# UI Quality and Stability

## Existing product-specific visual language and component tokens

하단 QDockWidget, compact local tabs, native Qt combo/search/button, engineering table vocabulary를
재사용한다. 카드, badge 군집, gradient, animation은 추가하지 않는다.

## Affected screens, components, text, tables, plots, and conditional controls

Diagnostics의 `Log` 페이지만 변경한다. 한 줄 toolbar와 `QTableView`를 사용하며 열은
`Time (local)`, `Level`, `Source`, `Device`, `Message`다. System/Camera/Pixhawk/GNSS/FTP/
Measurement/Performance 탭은 유지한다.

## AI/template-like pattern rejection audit

운영 목적이 없는 dashboard/card/pill/대형 상태색을 사용하지 않는다. Level은 텍스트를
항상 표시하며 색상만으로 의미를 전달하지 않는다.

## Dynamic value formats, precision, units, longest extents, and update rates

- Time: 고정 폭 `MM-dd HH:mm:ss.SSS`; 전체 ISO UTC와 sequence는 tooltip/detail로 제공.
- Level/Source/Device: canonical 고정 vocabulary.
- Message: 원문 보존, 한 줄 표시, 전체 문자열 tooltip/copy 가능.
- 갱신: 보이는 Log 탭에서 기존 status poll cadence를 사용하되 incremental batch만 반영.

## Reserved geometry, overflow, wrapping, elision, and detail access

Time/Level/Source/Device 열은 초기 폭을 고정하고 Message만 stretch한다. row height와 dock
높이는 live update로 바꾸지 않는다. 긴 message는 elide하고 tooltip 및 `Ctrl+C` TSV 복사를
제공한다.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

행 append는 model reset을 사용하지 않는다. 현재 선택, keyboard focus, 수동 column 폭,
dock/splitter geometry를 보존한다. bottom에 있을 때만 follow하고, 사용자가 위로 이동하면
현재 위치와 새 로그 수를 유지한다. 필터 변경 때만 의도적으로 visible row set을 갱신한다.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

각 filter에 visible/accessible label 또는 accessible name과 tooltip을 둔다. 900 px minimum
window, 1280x720/1600x980, 100/125/150% scale, 긴 한국어 오류 문자열을 확인한다.

## Before/after screenshots and dynamic-state sequence evidence

승인 후 구현 전 baseline, 구현 후 All/filtered/paused/overflow/error/empty 상태를 같은 창
크기에서 캡처한다. rapid append 중 geometry/focus/selection/scroll sequence도 기록한다.
