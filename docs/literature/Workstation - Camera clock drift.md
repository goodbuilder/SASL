---
tags:
  - paper-candidate
  - workstation
---

# Workstation 후보 — RTSP stereo 카메라 상대 clock drift

## 제안 질문

운용 중인 두 RTSP 카메라의 clock rate가 얼마나 벌어지는가? RTCP Sender Report NTP와 RTP media-clock을 host monotonic receive time에 각각 회귀해 pairwise drift를 독립적으로 추정할 수 있는가?

## 저장소 결과

근거: [`Dual-camera oscillator holdover`](../../apps/Workstation/docs/changes/20260826-integrated-field-stability/evidence/stus-relative-oscillator-30m-20260827.md).

- 1,800.022 s 관측, Left/Right RTCP Sender Report 381/377개, probe restart 없음.
- RTC/NTP 기준 Right가 Left보다 16.605 ppm 빠름; conservative 95% interval +11.863…+21.356 ppm.
- RTP media-clock 회귀 결과는 +16.634 ppm으로 방향·크기가 일치한다.
- 세-SR median 상대 offset은 30분 동안 31.356 ms 변했다. 16.605 ppm이 유지된다고 가정하면 100 ms drift까지 약 100.4분이다.

## 시간·좌표 계약

이 결과는 시간축을 다룬다. RTC/NTP wall-clock, RTP media ticks, host monotonic receive time은 서로 다른 clock domain이며, 논문에서 하나의 공통 time origin으로 취급하지 않는다. 이 실험은 카메라 exposure timestamp나 stereo position accuracy를 측정하지 않았다.

Workstation의 실제 stereo result가 후속 연구에 쓰이면 public camera midpoint output은 `baseline_center_frd` `[Front,Right,Down]` metres다. Camera optical point는 RDF `[Right,Down,Forward]`; image pixel은 `(u right, v down)`; PX4 local/world position은 NED `[North,East,Down]`이다. Time-drift study 자체에는 XYZ coordinate result가 없다.

## 문헌 관계

- [[RFC 3550 - RTP and RTCP]]는 RTCP SR의 NTP/RTP 대응 시점과 nominal RTP clock frequency 추정의 프로토콜 근거다.
- [[Gueguen 2025 - GNSS stereo time synchronization]]은 GNSS trigger와 exposure signal/LED 측정으로 capture-time latency를 평가한다. 이 논문은 그 방식과 다른 free-running stream clock-rate 관측이다.
- [[Seyfu 2025 - Consumer stereo synchronization]]은 움직이는 장면에서 reprojection error를 최소화해 sub-frame lag를 보정한다.
- [[Zhou 2024 - Time-calibrated video synchronization]]은 별도의 time-calibrated video로 camera-local time을 global timeline에 맞추는 방법이다.
- [[Han 2024 - Coded-light camera timestamping]]은 coded LED panel을 공통 시야에 두고 저장된 영상에서 frame timestamp를 재구성한다. RTCP/RTP clock rate 관측과 달리 post-processing timestamp reconstruction 비교군이다.
- [[Meyer 2026 - RocSync exposure and clock alignment]]는 LED Clock의 노출 구간으로 frame exposure start/end를 추정하고 local-to-global clock drift와 offset을 구한다. 현 Workstation 후보에 빠진 exposure-time 검증을 위한 가장 직접적인 후속 비교다.

시간축 구분은 유지한다. 현재 30분 결과의 약 16.6 ppm은 RTCP/NTP와 RTP media-clock rate 차이다. 이는 센서 노출 시각 불일치나 triangulation error를 직접 측정한 값이 아니다. Han/RocSync는 capture-time에 더 가까운 독립 기준 아이디어를 제공하지만, 그 결과와 현재 ppm/offset 통계량을 같은 수치처럼 비교하지 않는다.

## Novelty 가설 — 검증 전

가능한 기여는 **decoder-free RTCP probe에서 NTP와 RTP rate를 각각 host-monotonic receipt time에 대해 추정하고 두 독립 추정치로 camera-pair drift를 교차 확인한 field protocol**이다. 현재 두 추정치가 약 16.6 ppm으로 일치하고 30분 offset 변화와 운영상 누적 시간을 연결한다.

가까운 연구들은 sync 또는 capture-time을 측정·보정한다. [Gueguen et al.](https://isprs-archives.copernicus.org/articles/XLVIII-G-2025/535/2025/index.html)은 GNSS trigger와 exposure 신호를, [[Han 2024 - Coded-light camera timestamping]]은 coded LED 기반 post-capture frame timestamps를, [[Meyer 2026 - RocSync exposure and clock alignment]]은 visually decoded exposure windows에서 local-clock offset/drift를 다룬다. [Seyfu & Yang](https://www.mdpi.com/1424-8220/25/17/5535)은 image/reprojection 기반으로 sub-frame lag를 보정한다. 이 후보는 exposure timestamp를 재구성하거나 alignment를 교정하지 않고 RTCP/NTP·RTP sender-clock rate 차이를 관측한다.

**아직 novelty로 주장하지 않을 내용:** WIRIS 기종 전반에 일반화되는 drift 특성, exposure-time 불일치, stereo position error 개선, 재동기화 알고리즘.

**Novelty 상태:** 가설. 원시 trace 재수집, 여러 camera pair/run/온도 조건 반복, 독립 clock reference, exposure timing 및 실제 triangulation 영향 측정이 있어야 기여를 확정할 수 있다.

## publication 한계

원시 30분 JSONL은 보고서가 `/tmp`에 기록했다고 하지만 현재 파일이 없어 재현할 수 없다. 저장소에는 요약 수치만 남아 있다. 현재 결과는 단일 camera pair·단일 30분 powered run이며, 온도, reboot, 다른 기기 쌍, 외부 reference clock, 실제 exposure skew 또는 triangulation error를 검증하지 않았다.

**추천 framing:** field clock-rate characterization 후보. 논문 제출 전에는 raw trace를 재수집하고 여러 run/device/temperature로 반복하며 clock-rate 추정의 독립 reference와 exposure-time 영향 측정을 추가한다.

