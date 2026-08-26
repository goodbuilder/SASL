# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-08-26 — Accepted problem boundary

- P1/P2 are assumed to run Stock Stable firmware.
- Solve ETH MAVLink stability in Workstation first; NTRIP 401 and broader session/process lifecycle
  work remain outside this slice.
- Long gaps between measurements allow connection/RTK/preflight work to repeat; calibration results
  remain reusable.

## 2026-08-26 — Evidence-backed cause classification

- Boot-time interface initialization and runtime packet loss are separate faults.
- Runtime fault is classified as `PX4/NuttX UDP MAVLink task deadlock — high-confidence inference`,
  based on reboot-only recovery, alive USB MAVLink, alive host network, valid RTCM framing, and the
  matching official PX4 issue. It is not represented as proven without an onboard stack trace.

## 2026-08-26 — Operator-confirmed verified-USB recovery approved

- A confirmed stalled side may be rebooted only after operator confirmation and only through a
  currently observed USB heartbeat whose system ID uniquely matches P1=1 or P2=2.
- Missing, duplicate, ambiguous, or stale USB identity fails closed and yields named-side manual
  power-cycle guidance. No idle-only or other automatic reboot is approved.
- Command send is not recovery success; Ethernet identity and continuous qualification must be
  observed again.

## 2026-08-26 — Actual firmware re-read deferred by current stalled condition

- At 16:32 KST both `10.0.0.2` and `10.0.0.3` answered ping and UDP 14550/14551 had no host owner,
  but passive listeners and direct GCS/version requests received no MAVLink heartbeat for either
  side. No USB serial device was present.
- Therefore the repository v1.17.0 bundle is not treated as installed firmware. The last actual
  device evidence remains PX4 1.14.3, and firmware/sysid/parameter inventory must be re-read after
  safe operator recovery before hardware acceptance.

## 2026-08-26 — Conditional product-documentation requirement

- `Pixhawk.md`에는 실제 P1/P2 검증으로 현장 재현 조건에서 해결됨이 확인된 뒤에만 원인과
  해결 방법을 기록한다.
- 자동 테스트만 통과하거나 10분 soak가 실패한 상태에서는 해결되었다고 기록하지 않는다.
- 성공 시에도 PX4/NuttX 내부 stack trace를 확보하지 못한 원인은 `고신뢰 추정`으로,
  Workstation 조치는 `Stock firmware 운용 완화 및 검증된 복구 방법`으로 표현한다.
- 문서에는 firmware/endpoint/parameter 조건, 변경 전 재현, 변경 후 시험 시간과 packet
  evidence, RTCM fail-close, 재부팅 절차, firmware-level 잔여 한계를 함께 남긴다.
- 문서 기록 전 resolution gate는 최소 3회의 cold-boot cycle(각 10분 이상), RTCM과 정상
  Workstation 부하를 켠 dual-link 60분 soak, 한-side 격리, verified recovery의 전부다.
