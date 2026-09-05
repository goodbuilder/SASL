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

## 2026-08-31 — Actual IMU sample is the stream-delivery postcondition

- P1/P2 `MAV_CMD_SET_MESSAGE_INTERVAL` 전송은 intent일 뿐 성공이 아니다. 각
  `SCALED_IMU`, `SCALED_IMU2`, `SCALED_IMU3` expected-sysid packet의 실제 app-receive
  timestamp가 stream delivery의 authoritative postcondition이다.
- Initial profile 뒤 누락/중단된 표준 IMU stream만 bounded retry한다. sample 수신은 해당
  retry를 즉시 종료하며 all-RX stall에는 retry하지 않는다.
- 기존 IMU 표는 fresh sample만 `LIVE`; no sample은 `WAITING`; 오래된 cached sample은
  `STALE`과 age로 표시한다. stale 값을 삭제하거나 현재값처럼 승격하지 않는다.
- 자동 USB fallback이나 persistent PX4 write는 추가하지 않는다. 현재 P1 USB shell 복구는
  hardware boundary evidence이며 operator-confirmed reboot 검증 전 persistent remedy가 아니다.

## 2026-08-31 — P1 USB-free persistence failed at board-side Ethernet RX

- User explicitly approved the verified P1 USB reboot and requested Ethernet-only persistence.
- Reboot and static-neighbor A/B proved host commands reached P1's known destination MAC on wire,
  while P1 accepted none. Cable/port exchange left P1 at ARP reply `0/147` and P2 at `5/5`.
- Therefore do not persist a startup-only IMU workaround as if it fixed Ethernet operation: P1
  would still reject GCS heartbeat, RTCM and commands. The remaining repair boundary is P1's
  board-side Ethernet PHY/driver/NuttX RX path or replacement transport/hardware.
- Keep `Pixhawk.md` unchanged because USB-free recovery and the required field gates failed.

## 2026-08-31 — Physical board damage remains a hypothesis, not a proven cause

- Exact stock reflash, cable/port swap and P2 same-firmware comparison localize the fault to P1,
  but do not independently distinguish PHY/MAC damage from NuttX IOB/ARP or duplicate-init state.
- P1 heap/DMA capacity was not exhausted globally; ARP ioctl still returned ENOMEM and the official
  build exposes only 24 IOB buffers. P1 also self-received high-rate traffic with impossible-looking
  126.9% MAVLink RX loss, unlike P2's external-GCS RX and 0.1% loss.
- Preserve the truthful split: P1 outbound ETH ATTITUDE/GNSS transport passed 10 minutes; inbound
  Ethernet ARP/GCS/RTCM failed. Do not call the board fully damaged or fully healthy.
- Before hardware replacement, compare/remove the duplicate P1 network initialization under a
  reversible backup, then consider a controlled IOB/UDP firmware A/B as a separate approved patch.
