# Frontend Contract

## Controls and inputs

기존 decoder combo `Auto|NVIDIA|Software`와 settings schema를 유지한다.

## Payloads and commands

camera start는 requested decoder를 공통 policy gate에 전달하며 effective decoder가 허용된
경우에만 기존 pane/worker command를 실행한다.

## Visible states and transitions

Auto software는 WARN/유효 `avdec_h264`, NVIDIA 불가는 카메라 시작 거부, 필수 runtime
누락은 Qt 이전 terminal FAIL이다. active decoder 표시는 기존 status signal을 유지한다.

## Errors and recovery actions

NVIDIA 거부 문구는 Auto 또는 Software로 변경하라는 조치와 doctor 명령을 포함한다.
dependency 오류는 대응 Ubuntu package와 수동 복구 명령을 포함한다.

## Keyboard, focus, accessibility, and layout

기존 combo와 focus/layout을 변경하지 않는다. 오류는 color-only가 아닌 텍스트다.

## Cross-feature visible behavior and isolation

카메라 capability 실패가 측정/recording 시작을 거부할 수 있으나 Pixhawk/GNSS 상태와
settings persistence를 변경하지 않는다.
