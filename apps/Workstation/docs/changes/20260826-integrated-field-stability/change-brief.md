# Change Brief

## Operator outcome

한 번의 Workstation 운용에서 Left/Right WIRIS 영상, P1/P2 Ethernet MAVLink,
SASL/NGII NTRIP, STUS 시간 동기, Measurement 시작, MP4와 native evidence 저장이
장시간 함께 동작해야 한다. 장애가 나면 성공처럼 보이지 않고 side와 실패 계층, 복구
조치가 명확해야 한다. 문제별 짧은 qualification을 모두 통과한 최종 후보만 60분
full-load soak에 들어간다.

## Motivation

2026-08-26 현장에서는 RTSP port가 열려 있어도 새 client가 PPS 없이 시작했고, 움직임 중
macroblock/잔상 또는 clean-intra 대기 freeze가 발생했다. STUS 직후 동기화는 시간이 지나며
RTCP/NTP pair delta와 SR-PC offset이 벗어났다. P1/P2는 ping이 살아 있는데 Ethernet
MAVLink가 약 5분 30초 뒤 멎었고, NTRIP은 잘못 선택된 credential로 HTTP 401이 가능했다.
과거 Session/spool과 저장 실패는 새 Measurement/기록을 방해했고, 녹화 시작 후 MP4가
생성되지 않은 경로 및 Qt image queue로 host RSS가 4 GiB 이상 증가한 경로도 재현됐다.

## Included scope

- RTSP start/long-run liveness, H.264 SPS/PPS/IDR and corruption/freeze behavior.
- Auto NVDEC requested/effective projection and safe CPU fallback.
- STUS ACK/reconnect/warmup plus sustained RTCP/NTP quality.
- P1/P2 HB/ATT/ANY liveness, STALE versus RX_STALLED, RTCM fail-close and recovery evidence.
- SASL/NGII `RTS2.ngii.go.kr:2101`, `VRS-RTCM32`, credential selection, HTTP status,
  RTCM3 receive and per-side injection evidence.
- Sessionless Measurement admission, blank restart, new run identity, storage isolation.
- Dual MP4 creation/growth/EOS/strict decode and exact FrameKey/hash evidence.
- Bounded shared-memory/queue/RSS behavior, responsive control and complete cleanup.
- Canonical root runtime/launcher readiness required to reproduce the field result.

## Excluded scope and non-goals

- Workstation/StereoWIRIS simultaneous rig ownership and the shared shutdown tool.
- Diagnostics structured-log filtering implementation.
- Camera or PX4 firmware changes, automatic reboot/power-cycle, destructive device commands.
- Unapproved encoder/IP/subnet/transport/QP/bitrate/GOP/IDR or PX4 parameter changes.
- Measurement mathematics, coordinate-frame conversion, calibration algorithm or RTK semantics.
- Rewriting, migrating or deleting existing Session/spool/evidence bytes.

## Acceptance summary

Every independent software and hardware slice must pass. Three independent cold-start cycles must
each sustain at least 10 minutes. The final candidate must sustain 60 minutes under dual-camera,
dual-Pixhawk, NTRIP/RTCM, YOLO/measurement, recording and evidence load with no required failure.
An app mitigation is not a camera/PX4 firmware fix. A required `FAIL`, `NOT_RUN`, or unresolved
`BLOCKED` prevents completion.
