# UI Quality and Stability

## Existing product-specific visual language and component tokens

기존 구조적인 현장 운용 UI, side별 16:9 preview, 상단 고정 status summary를 보존한다.

## Affected screens, components, text, tables, plots, and conditional controls

`측정 > 실시간`의 Left/Right CameraPane QLabel과 runtime camera status가 직접 영향 범위다.

## AI/template-like pattern rejection audit

새 카드, badge, animation, decoration은 추가하지 않는다. 필요한 계측은 기존 status/log
언어와 밀도를 따른다.

## Dynamic value formats, precision, units, longest extents, and update rates

기존 `1280x720 00.0 Hz`, decoder, NTP, YOLO Hz 형식을 유지한다. frame evidence는 UI를
고빈도로 흔들지 않고 파일/로그에 bounded rate로 남긴다.

## Reserved geometry, overflow, wrapping, elision, and detail access

preview와 status group geometry를 바꾸지 않는다.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

자동 frame update는 focus/scroll/tab selection을 변경하지 않아야 한다.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

UI 변경이 없으면 기존 계약을 회귀 확인한다. status text가 바뀌면 현재 한글 locale과
최장 오류 문구의 clipping을 검사한다.

## Before/after screenshots and dynamic-state sequence evidence

- Baseline: `evidence/baseline-20260826T1103KST.png` (artifact 미관찰, 2026-08-26).
- 재현/수정 후 증거는 packet evidence 경로에 보존할 예정이다.
