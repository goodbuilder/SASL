# Change Brief

## Operator outcome

Left/Right WIRIS Enterprise OZ visible RTSP를 동시에 표시할 때 GUI preview에
찢어짐, macroblock corruption, 이전-frame 잔상이 발생하지 않아야 한다. 문제를
실장비에서 먼저 재현하고 RTP부터 QLabel까지 가장 좁은 실패 계층을 확정한 뒤에만
최소 수정한다.

## Motivation

운영자는 GUI 화면에서 artifact를 보았지만 저장된 encoded H.264/MP4는 정상으로
관찰했다. encoder profile 변경은 증상을 가릴 수 있으므로 저장 분기와 live decode/UI
분기를 독립적으로 검증해야 한다.

## Included scope

- `apps/StereoWIRIS`의 OZ visible RTSP 수신, decode, frame 전달, Qt preview 경로
- Left `10.0.0.231`, Right `10.0.0.232` 실장비 readback과 비파괴 진단
- 동일 bitstream의 NVIDIA/software deterministic replay
- 최소 계측, 최소 수정, 집중 regression test, 완전 재시작 및 15분 실장비 검증
- 필요 시 VISIBLE `SEBR/SEQZ/SEGS/SEIR` 한 변수 A/B와 원상복구

## Excluded scope and non-goals

- StereoWIRIS 밖의 관련 없는 코드, camera IP/subnet/time/firmware/factory/reboot
- 카메라 내부 파일 삭제, Git commit/push, 사용자 dirty 변경 정리
- bitrate/GOP/IDR 변경만으로 증상을 가리는 조치
- 좌표, 측정, Pixhawk, GNSS/NTRIP, 세션 schema 변경

## Acceptance summary

초기 재현 증거와 계층별 frame identity를 확보하고, 새 프로세스에서 양쪽 preview를
동시에 15분 이상 운용해 artifact/error/unexpected fallback/stale-reorder가 모두 0이어야
한다. 저장 파일은 ffprobe와 software full decode를 통과해야 한다.
