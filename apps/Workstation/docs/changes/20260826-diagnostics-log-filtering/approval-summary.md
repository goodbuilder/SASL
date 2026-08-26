# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

Diagnostics의 `Log`를 시간순 표로 바꾸고 Source, Device, Level, 검색어 필터를 추가합니다.
기본 화면은 모든 장비를 함께 보는 `All`입니다. 기존 Workspace, Inspector, 다른 Diagnostics
탭, FTP 전용 탭과 장비 제어 방식은 그대로 둡니다.

## Operator workflow before and after

현재는 Camera/YOLO, Pixhawk, WIRIS 텍스트 묶음을 눈으로 찾아야 합니다. 변경 후에는 먼저
전체 시간순 흐름을 보고, 필요할 때 Left/Right WIRIS, P1/P2, Base/NTRIP 등으로 좁힙니다.

## Success, delay, rejection, failure, overload, and recovery

새 로그는 사용자가 맨 아래를 볼 때만 자동으로 따라갑니다. 위쪽을 읽는 중이면 위치를
유지하고 새 로그 수를 표시합니다. `Pause view`는 화면만 멈추며 로그 수집은 계속됩니다.
최근 2,000개를 넘겨 오래된 로그가 사라지면 조용히 숨기지 않고 만료 수를 표시합니다.

## Data and compatibility effects

저장 파일, 측정 데이터, 세션 audit JSONL, 장비 설정은 바꾸지 않습니다. 기존 raw message와
기존 text backend도 유지합니다. 모호한 legacy 메시지는 잘못된 장비로 추정하지 않고
`Shared/Unassigned`로 표시합니다.

## Performance priorities and allowed degradation

새 thread/process/GPU 작업은 만들지 않습니다. 보이는 Log 탭만 incremental update하고
buffer는 제한합니다. 로그 폭주 중에도 카메라/측정/장비 작업과 GUI 조작 응답성이 우선입니다.

## Visual and interaction changes, including dynamic-layout stability

기존 하단 dock과 compact Qt 스타일을 유지합니다. 한 줄 필터와 engineering table만
추가하며 live update가 dock 크기, 열 너비, 포커스, 선택, 스크롤을 움직이지 않게 합니다.

## How acceptance will be demonstrated

분류/순서/overflow pure tests, worker-thread boundary, 필터 조합, pause/resume, scroll/focus,
NTRIP 오분류 회귀, FTP/Workspace/Inspector/recording/shutdown 회귀를 차례로 통과시킵니다.
그 후 전체 Workstation 테스트와 실제 launcher 화면을 확인합니다.

## Cost, risks, assumptions, rollback, and field limits

가장 큰 위험은 기존 자유형 문자열의 장비를 과도하게 추정하는 것입니다. 명확한 태그만
분류하고 나머지는 unassigned로 남깁니다. 실제 장비 field 검증은 UI/log 표시 확인이며
프로토콜 동작 변경은 없습니다. 문제 시 old text view로 되돌릴 수 있고 저장 데이터 migration은
필요 없습니다.

## Exact scope awaiting approval

이 문서와 `implementation-plan.md`의 S1–S7만 승인 대상입니다. 다른 활성 변경 패킷,
디스크 로그 schema, 장비 protocol, 전체 logging framework 재작성은 포함하지 않습니다.

현재 상태: 승인 요청 중. 소스 수정은 아직 허용되지 않았습니다.
