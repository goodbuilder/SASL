---
tags:
  - paper-candidate
  - workstation
  - alternative
---

# Workstation 대안 — shared-NIC 통합 연결 지속성

## 제안 질문

두 WIRIS preview, 두 Pixhawk Ethernet telemetry, NTRIP가 shared Ethernet adapter에 함께 있을 때 Workstation의 checkpoint 기반 관측이 어떤 연결 실패와 복구 상태를 찾아내는가?

## 저장소 근거

[`Shared-NIC camera + Pixhawk baseline`](../../apps/Workstation/docs/changes/20260826-integrated-field-stability/evidence/shared-nic-camera-pixhawk-baseline-20260827.md)에는 mixed outcome이 보존되어 있다.

- 수락된 10분 integrated run 두 건에서 30초 checkpoint 21회 동안 양쪽 카메라가 `LIVE`, P1/P2 heartbeat/attitude/any fresh였고 packet capture/NIC drop은 0이었다.
- 한 run은 NTRIP가 약 529 s 뒤 stale/error로 끝났고, 다른 run은 704 RTCM chunks/430,883 bytes, 114 GGA, reconnect 0으로 끝났다.
- camera integrity recovery interval이 따로 기록되어 있다. Checkpoint `LIVE`만으로 image/MP4가 무손실이었다고 결론내릴 수 없다.
- 사전 시도에는 PX4 Ethernet receive-stall/fail 사례도 있어 passing run만 고르면 안 된다.

## 한계

이 결과는 연결 지속성과 장애 분리의 시스템 사례다. Stereo measurement accuracy, GNSS accuracy, usable image continuity, 두 camera clock agreement를 증명하지 않는다. 추가 반복과 재현 가능한 setup summary가 필요하다. Clock-drift 후보보다 원시 log가 더 잘 보존되어 있지만, 단일 장비/짧은 환경 범위에 묶여 있다.

## 문헌 연결

시간 동기화 reference와 섞지 말고, 이 paper는 `connection continuity`, `camera integrity`, `telemetry health`, `RTCM continuity`를 분리해 다룬다. [[Workstation - Camera clock drift]]는 별개의 timebase study다.

## Novelty 가설 — 검증 전

가능한 기여는 checkpoint의 `LIVE` 연결 상태, camera integrity recovery, PX4 packet freshness, NTRIP continuity를 별도 결과로 기록하는 field qualification protocol이다. Passing run과 실패 run을 함께 보존하고, `LIVE`가 깨끗한 영상이나 무손실 MP4를 뜻하지 않는다는 운영상 경계도 보여준다.

**Novelty 상태:** case-study 수준의 가설. 두 번의 수락된 10분 run과 제한된 adapter/topology로는 일반적인 network reliability 주장을 뒷받침하지 못한다. 여러 run에서 동일한 실패/복구 기준을 적용하고, 환경·하드웨어 범위를 늘려야 한다.

