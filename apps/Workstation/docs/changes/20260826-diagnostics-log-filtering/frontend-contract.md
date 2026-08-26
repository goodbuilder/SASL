# Frontend Contract

## Controls and inputs

- Source: All, Camera, Pixhawk, GNSS/RTK, Detection, Measurement, Storage, FTP,
  Calibration, Alignment, System, Other.
- Device: All, Left WIRIS, Right WIRIS, P1, P2, Base GNSS, NTRIP, Drone,
  Shared/Unassigned.
- Level: All, Info, Warning, Error.
- Search: message/source/device/code에 대한 case-insensitive substring.
- Pause view/Resume, Follow latest. 이벤트 삭제 버튼은 두지 않는다.

## Payloads and commands

모든 control은 local read-model filter/viewport state만 바꾼다. 장비·controller command나
backend mutation을 보내지 않는다.

## Visible states and transitions

- 진입 기본값: All/All/All, 빈 search, running/follow latest.
- 새 event: bottom이면 append+follow; 위를 읽는 중이면 scroll 유지+new count.
- Pause: rows 동결, pending count 증가; Resume: sequence 순으로 반영.
- overflow: 만료된 event 수를 명시하고 남은 event를 계속 표시.
- no data/no match/backend unavailable은 서로 다른 factual empty 상태로 표시.

## Errors and recovery actions

facade malformed result는 UI crash나 정상 표시로 축약하지 않고 `Diagnostic event source
unavailable` 상태를 보이며 다음 refresh에서 재시도한다. text fallback이 있으면 그것을
사용한다.

## Keyboard, focus, accessibility, and layout

Tab order는 Source → Device → Level → Search → Pause/Resume → table → Follow latest다.
`Ctrl+C`는 선택 행을 열 순서 TSV로 복사한다. filter/search 입력 중 live event가 와도
focus를 빼앗지 않는다.

## Cross-feature visible behavior and isolation

FTP 전용 탭과 다른 Diagnostics 탭은 그대로다. Log filtering 실패가 camera preview,
Pixhawk/GNSS connection, Measurement, recording, shutdown을 방해하지 않는다.
