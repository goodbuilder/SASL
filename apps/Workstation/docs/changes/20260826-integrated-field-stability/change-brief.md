# Change Brief

## Operator outcome

한 번의 Workstation 운용에서 Left/Right WIRIS 영상, P1/P2 Ethernet MAVLink,
SASL/NGII NTRIP, STUS 시간 동기, Measurement 시작, MP4와 native evidence 저장이
장시간 함께 동작해야 한다. 장애가 나면 성공처럼 보이지 않고 side와 실패 계층, 복구
조치가 명확해야 한다. 문제별 짧은 qualification을 모두 통과한 최종 후보만 60분
full-load soak에 들어간다. StereoWIRIS Legacy는 Workstation과 동시에 장비를 소유하지
않고, blank restart/new session/MP4 경로를 별도 집중 gate로 검증한다.

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
- StereoWIRIS Legacy blank startup, fresh `세션 기록 시작`, and its recording finalization.
- Method-specific readiness: missing current-zoom intrinsics, stereo R,t, correspondence, attitude,
  GNSS/RTK or synchronization blocks only the dependent valid result, not Measurement lifecycle.
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

Measurement Start remains available despite MAVLink, GNSS/NTRIP, STUS or best-effort storage
failure. A method that depends on an unavailable input must fail closed with the exact reason and
must not publish a valid result.

## Proposed timing-fallback scope — not yet approved

When one manual STUS seed has been VERIFIED and later clock evidence is revoked, keep live distance
measurement available by selecting Left/Right decoded frames on host app-receive monotonic time.
Continue preserving NTP/RTP/SR evidence in every selected detection and persisted pair. Keep
pre-seed operation, active epoch transitions, calibration timing, Legacy, geometry and coordinate
contracts unchanged. The proposed fallback produces an explicitly degraded/estimated result, not
fabricated exposure-time truth.

## Superseding app-receive-authoritative scope — awaiting approval

The fallback scope above is superseded. Workstation live distance always pairs decoded frames on
app-receive monotonic time and treats an accepted ≤100 ms app pair as normal timing truth. NTP,
STUS status and NTP clock epochs are record-only and cannot block or degrade distance. Preserve
their full provenance without calling app receive a physical exposure timestamp. Calibration,
Legacy, geometry, coordinates, device settings and historical bytes remain excluded.

## Proposed indoor 60-minute experimental recovery scope — awaiting approval

The active goal is to execute one real indoor 60-minute Workstation integration run. Because the
default NAL5 gate truthfully rejects both current cameras, the run requires an explicitly opt-in,
process-local WIRIS experiment mode. That mode may treat only a parser-complete, SPS/PPS-backed,
non-delta NAL1 access unit as a device-specific decoded-output recovery candidate. It never calls
that candidate IDR/keyframe and never makes direct-copy MP4 portable or valid.

The product default remains strict NAL5. The experiment changes no camera/PX4 setting, adds no
encoder and uses the existing decoder/queues. It runs one five-minute preflight before the timed
soak, then observes dual camera/YOLO/app pairing, P1/P2 MAVLink, process-local SASL/NGII NTRIP,
RTCM injection, decoder-independent RTCP Clock evidence, recording growth, GUI responsiveness and
resources at twelve five-minute checkpoints. Known STUS mutation convergence remains a separate
failed gate and is not actuated or relabelled PASS by this soak.

The user subsequently approved execution outdoors. The location change adds real rover RTK
Fixed/accuracy evidence to the gate but does not authorize new device settings, STUS mutation,
firmware, re-encoding or a relaxed MP4 result.
## 2026-08-27 manual-only PX4 PARAM amendment

Normal P1/P2 qualification no longer performs a hidden calibration-parameter snapshot. Operators
retain the existing side-specific Refresh controls and explicit readback after parameter-changing
actions. This removes unsolicited PARAM traffic and requalification without changing telemetry,
RTCM, Measurement, recording, device settings, persistence, or GUI layout.

## 2026-08-27 60-second operational MAVLink qualification amendment

P1/P2 operational readiness now requires 60 continuous seconds of fresh expected-system HEARTBEAT
and ATTITUDE after connection, reconnect, confirmed reboot, stall recovery, or an explicit command
boundary. Stale/stall blocking is unchanged. This permits attitude-dependent measurement after one
minute while preserving ten-minute and 60-minute field tests as separate long-run evidence rather
than product admission delays.

## 2026-08-28 Mountain GNSS 3D-fix amendment

Mountain translation now accepts both P1/P2 at numeric `fix_type >= 3` when their capture-bound
positions and NED baseline are finite and the baseline is non-zero. Both-side RTK Fixed is no
longer required. Non-fixed usable results remain explicitly `ESTIMATED`; sub-3D fixes and all
existing Mountain rotation failures still block their dependent axes. The latest pre-run Draft is
frozen into the next Measurement, while `pixhawk_rtk_live` remains representative. No setting,
layout, schema migration, coordinate change or device operation is included.

## 2026-08-29 Mountain automatic-suggestion copy amendment

Mountain `Use suggestion` now accepts a structurally usable proposal regardless of detector quality
state. LOW_CONFIDENCE remains visible/persisted evidence; the controller still requires current
frozen identity and at least four equal-order finite in-frame unique/distributed pairs after shared
exclusions, then copies and hashes only the retained points. Horizon was unchanged in that slice.
Explicit
Solve is still required and has no direct proposal fallback; auto-copy, auto-Solve and auto-Apply
remain excluded. Manual Mountain, solver gates, Measurement freeze, representative result,
resources and historical data are preserved.

## 2026-08-29 Horizon automatic-suggestion copy amendment

Horizon `Use suggestion` now ignores detector quality state/confidence while retaining all quality
text and evidence. Current frozen identities/dimensions, exactly two finite in-frame endpoints per
side and the existing 25%-width separation remain hard owner/UI gates. Accepted low-quality
suggestions keep their original state and an explicit warning. Solve and Apply remain separate;
manual Horizon, exact K,D, both-side observability, resources and historical data are unchanged.

## Proposed Mountain lever-arm status amendment — interviewing 2026-09-01

The operator requested removing Mountain's lever-arm-driven `UNMODELED` result. The current solve
already applies a zero correction when no explicit
`lever_arm_baseline_correction_left_rdf_m` is supplied, while exposing that assumption as
`lever_arm_status=UNMODELED` and a candidate warning. The existing Mountain tooltip likewise says
that the camera optical center is assumed to equal the GNSS antenna.

Two materially different outcomes remain open. The narrow outcome formalizes the existing
zero-offset calculation as an explicit product assumption, replaces `UNMODELED` with a truthful
assumed-zero provenance state, and removes only the lever-arm warning. The physical-calibration
outcome instead requires measured per-rig antenna-to-optical-center offsets, settings/persistence,
frame-aware conversion, validation, and failure behavior. Merely deleting the text while retaining
an unnamed assumption is excluded from the recommendation. Existing Mountain correspondence,
GNSS, K,D, rotation, Draft-freeze, representative-method, and historical-row behavior stays
unchanged until the operator selects and approves one outcome.
