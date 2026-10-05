# Code Mitigation Plan

## Plan mode and approval status

이 문서는 카메라가 보내지 않은 RTP bytes를 복원하는 계획이 아니라, 불완전한 H.264 reference
chain에서 생성된 frame을 GUI·YOLO·측정 소비자에게 전달하지 않는 코드 완화 계획이다.

- 분류: `backend-coupled`; widget/layout 변경 없는 backend-only GUI contract 변경
- 승인: 2026-08-26 14:03 KST 승인된 same-session clean-intra recovery 범위를 유지
- 이번 계획 작성 turn: 문서만 갱신하며 source/test는 수정하지 않음
- 현재 상태: 핵심 source와 focused tests는 구현됐지만 motion 15분 responsiveness acceptance는 FAIL

## Operator-visible outcome

RTP forward gap이 발생해도 macroblock band, 이전-frame 혼합, 손상 frame을 새 영상처럼 표시하지
않는다. 다음 독립 복구 경계까지 마지막 정상 preview가 유지될 수 있으며, clean non-delta intra
AU와 decoded buffer의 PTS가 일치하면 같은 RTSP session과 같은 decoder에서 자동 재개한다.

## Preserved invariants

- Left/Right endpoint, TCP transport, Auto/NVIDIA/Software 선택 의미를 유지한다.
- RTP gap만으로 decoder를 교체하거나 새 RTSP client를 만들지 않는다.
- Auto의 Software fallback은 first-frame timeout, decoded-frame stall, bus error 같은 decoder
  health failure에만 유지한다.
- `CameraPane`, UI layout, focus, scroll, preview geometry, settings/session schema를 바꾸지 않는다.
- YOLO, 측정, 좌표, Pixhawk, GNSS/NTRIP 계약을 바꾸지 않는다.
- camera encoder readback `bitrate/QP/GOP/IDR = 0/1/90/0`을 변경하지 않는다.
- process/thread/queue 수와 GPU memory budget을 늘리지 않는다.
- user dirty worktree를 reset/restore/checkout/stash/clean하지 않는다.

## Explicit non-goals

- 카메라 firmware 내부 RTP omission 원인 수정
- 누락된 RTP/H.264 bytes 재구성
- 강제 Software decoder 전환, `wait-for-keyframe`, IDR/bitrate masking
- Workstation/common runtime 변경 또는 외부 RTSP client 강제 종료
- encoded recording payload의 오류 정정이나 transcoding

## Runtime state contract

| State | Entry condition | Decoded-output behavior | Exit condition |
|---|---|---|---|
| `HEALTHY` | 새 pipeline 또는 recovery 완료 | preview·YOLO·측정에 정상 전달 | 같은 SSRC의 RTP forward gap |
| `RECOVERY_BLOCKED` | 예상 sequence보다 2 이상 전진 | decoded buffer를 모든 소비자보다 앞에서 차단하고 마지막 정상 GUI frame 유지 | gap PTS 이후 clean non-delta intra AU와 decoded buffer PTS 일치 |
| `HEALTH_FAILURE` | first-frame/stall/bus failure | 기존 Auto fallback 또는 명시 decoder 정책 적용 | 기존 reconnect/fallback 계약 |

다음 입력은 gap이나 recovery로 오인하지 않는다.

- 16-bit sequence 정상 wrap
- duplicate 또는 old/reordered packet
- SSRC 변경 뒤 첫 packet
- gap PTS 이전 intra AU
- delta, `CORRUPTED`, PTS 없는 AU

`RECOVERY_BLOCKED` 자체는 health failure가 아니다. clean intra가 예상 cadence 안에 오지 않으면
손상 frame을 다시 허용하지 않고 acceptance를 실패 처리한다. 새 timeout/reconnect 정책은 별도
승인 없이 추가하지 않는다.

## Implementation slices

### Slice 1 — RTP discontinuity tracker

- Owner: `GstRtpTimingTracker`
- Input: SSRC, 16-bit sequence, RTP/PTS, monotonic observation time
- Output: forward-gap record와 persistent recovery latch
- Invariant: 진단 event를 pop해도 decoded-output latch는 풀리지 않음
- Status: 구현 및 focused pure test 완료

### Slice 2 — Parsed-AU recovery oracle

- `h264parse` output에서 AU PTS, `DELTA_UNIT`, `CORRUPTED` flag를 관찰한다.
- gap 이후 최초 clean non-delta AU를 recovery candidate로 저장한다.
- decoded appsink buffer가 동일하거나 이후의 candidate PTS에 도달할 때만 latch를 해제한다.
- IDR NAL type 5 존재를 전제로 하지 않는다.
- Status: 구현 및 same-session 90초 A/B 완료

### Slice 3 — Consumer gate and race closure

- decoded buffer map 전과 frame publish 직전에 latch를 확인한다.
- block 상태에서는 preview queue, YOLO latest frame, 측정 frame, decoded-rate 통계를 갱신하지 않는다.
- GUI는 `CameraPane`의 마지막 완전 QImage를 유지한다.
- stale stream generation과 latest-only QImage 계약은 기존대로 유지한다.
- Status: 구현 및 producer→queue→QImage identity 검증 완료

### Slice 4 — Fallback, retry, and observability

- gap 발생 시 active decoder를 유지하고 health fallback timer를 recovery 중 정지한다.
- 첫 gap은 SSRC/sequence/missing/decoder와 함께 알리고 반복 gap은 bounded rate로 집계한다.
- recovery에는 resume PTS, 차단 시간, suppressed frame 수를 기록한다.
- Software health fallback 직후 RTSP 503과 initial PLAYING 실패만 기존 bounded retry 대상으로 둔다.
- Status: 구현 및 focused tests 완료

### Slice 5 — Pipeline and recording regression isolation

- decoder 앞 encoded queue는 compressed AU를 임의로 1-frame drop하지 않는 bounded-time queue를
  유지한다.
- preview/recording tee, filesink path, plain MP4 mux, orderly EOS는 별도 회귀 대상으로 유지한다.
- preview gate는 decoder 앞 encoded recording bytes를 수정하지 않는다.
- Status: short-run recording/strict decode 완료; 15분 동시 운용 file 검증 대기

### Slice 6 — Full application and target-hardware acceptance

- 완전 종료 후 새 PID에서 단일 StereoWIRIS owner로 Left/Right를 시작한다.
- frame identity logging을 켜고 양쪽 YOLO, hand/body motion, 기록 시작/종료를 포함해 15분 이상
  운용한다.
- gap이 발생하면 차단과 clean-intra recovery event를 frame evidence와 맞춘다.
- Status: FAIL; 최신 930초 window는 artifact 대신 양쪽 누적 freeze가 약 79~82초 발생

### Slice 7 — Camera burst-control amendment

- 단일 RTSP owner를 확인하고 현재 encoder `0/1/90/0`을 readback한다.
- bitrate/GOP/IDR은 유지한 채 QP만 `1 -> 5`로 바꿔 같은 motion workload를 비교한다.
- freeze, RTP/AU discontinuity, bandwidth, decoded rate와 대표 crop 화질을 측정한다.
- 시험 결과와 무관하게 QP를 `1`로 즉시 원복하고 readback한다.
- Status: awaiting approval; 기존 encoder 불변 범위 밖

## File and ownership map

| Path | Planned responsibility | Scope status |
|---|---|---|
| `wiris_stereo_distance/yolo_stream.py` | sequence tracker, AU oracle, decoded-output gate, bounded logs/retry | source owner; already modified |
| `tests/test_yolo_stream.py` | pure state, failure policy, pipeline and serialization regression | focused test owner; already modified |
| `docs/testing.md` | repeatable focused and hardware acceptance commands | product documentation |
| `docs/usage.md` | operator-visible freeze/recovery limitation | product documentation |
| `UPDATE.md` | final verified change record | close-out only |
| `wiris_stereo_distance/camera_pane.py` | last-clean frame display | preserved; no plan edit |

## Verification matrix

| Layer | Required evidence | Pass condition | Current status |
|---|---|---|---|
| Pure sequence logic | wrap/duplicate/old/SSRC/forward-gap tests | only true forward gap latches | PASS |
| Recovery oracle | pre-gap intra, delta, corrupt, missing PTS, clean intra tests | only valid recovery PTS reopens | PASS |
| Decoder policy | Auto/NVIDIA/Software state tests | gap causes no fallback; health failure retains old policy | PASS |
| Same-bitstream replay | one captured MKV through NVIDIA/software | suspect output suppressed until clean intra; no PTS regression | PASS (90초 A/B 포함) |
| UI handoff | producer/queue/QImage hashes and sequence | hash match, reorder 0, only complete latest frame replacement | PASS in focused evidence |
| Resource policy | process/thread/queue/GPU comparison | no new owner or unbounded queue; GUI remains responsive | short-run PASS, sustained pending |
| Recording regression | worker replacement, MP4 growth, EOS, ffprobe, strict decode | both files finalize and strict decode exit 0 | 71초 PASS; 15분 pending |
| Hardware integration | fresh PID, Left/Right YOLO+motion+recording | artifact/error/unexpected fallback/stale-reorder 0 | PENDING |

## Exact verification sequence

1. Run syntax compilation for `wiris_stereo_distance`.
2. Run the gap/recovery focused node selection, then all `tests/test_yolo_stream.py`.
3. Run the startup/session/recovery PyQt selection documented in `docs/testing.md`.
4. Replay the preserved same-bitstream fixture and compare decoded frame PTS/counts.
5. Fully stop the app and confirm no stale camera worker owns either RTSP session.
6. Start the documented launcher with `STEREO_PREVIEW_IDENTITY_LOG` on a fresh PID.
7. Start both cameras and YOLO; verify normal preview before recording.
8. Start recording and verify new worker ownership, PLAYING/first frame, and both MP4 byte growth.
9. Apply representative hand/body motion for at least 15 minutes while collecting window recording,
   frame identities, RTP gap/recovery logs, CPU/GPU/NIC time series, worker PIDs, and kernel link events.
10. Stop recording and app normally; verify worker/resource release and orderly MP4 EOS.
11. Run `ffprobe` and strict software full decode on both files.
12. Record PASS/FAIL/INVALID per acceptance rules; do not treat one clean screenshot as completion.

## Acceptance, failure, and invalid-run rules

PASS requires all of the following in one valid fresh-process run:

- Left/Right visible preview and YOLO remain responsive for at least 15 minutes.
- macroblock, tearing, previous-frame mixture, or suspect frame emission is 0.
- any RTP gap is followed only by freeze/suppression and clean-intra resume.
- unexpected decoder fallback, GStreamer error, PTS regression, stale generation, queue reorder,
  producer/queue/QImage mismatch is 0.
- no new process/thread/queue owner or material sustained resource regression appears.
- recording-start replacement succeeds and both final MP4 files pass strict decode.

Any visible artifact, corrupt output after a detected gap, silent infinite queue growth, or strict decode
failure is FAIL. USB disconnect, Ethernet carrier loss, camera power loss, or operator-triggered STUS
restart invalidates the affected timing window and requires a fresh run.

## Residual risk and rollback

- 코드 gate는 source RTP omission을 고치지 않으므로 일시 freeze는 남는다.
- gap이 매우 잦거나 clean intra가 오지 않으면 표시가 장시간 멈출 수 있다. 이 경우 손상 frame을
  허용하지 말고 실패 증거로 남긴다.
- packet probe가 보는 post-jitter drop도 안전을 위해 같은 gate를 적용한다. 원인 귀속은 passive
  TCP/RTP capture로만 판단한다.
- rollback이 승인되면 packet-owned hunk만 targeted reverse patch하고 camera 설정과 다른 dirty
  변경은 건드리지 않는다.

## Next execution gate

현재 source gate를 완료 처리하지 않는다. 다음 작업은 사용자가 승인할 경우 단일-owner QP
`1 -> 5` 일시 A/B와 원복이다. 그 결과로 짧은 motion 비교를 통과한 뒤에만 fresh PID 15분
Left/Right motion+recording acceptance를 다시 실행하고 packet을 `completed`로 닫는다.

## Approved Workstation implementation

1. Workstation GStreamer는 camera당 하나의 `rtspsrc`에서 parsed H.264를 tee해 preview/YOLO와
   plain MP4 저장을 함께 소유한다. 별도 FFmpeg RTSP recorder는 제거한다.
2. RTP forward gap과 PTS/DISCONT/CORRUPTED AU를 tee 전에 latch하고 clean non-delta intra까지
   양쪽 branch를 차단한다. 같은 MP4를 유지하고 30초 recovery timeout 뒤 manual reconnect한다.
3. OpenCV/FFmpeg·PyAV는 기존 decoded re-encode 저장을 유지하며 integrity-unverified provenance와
   writer drop 수를 상태/증거에 남긴다.
4. GStreamer EOS 9초, worker terminate 12초, kill 20초 예산으로 MP4를 먼저 finalize한다.
5. focused unit/integration, Workstation regression, 5분 smoke와 15분 motion hardware acceptance를
   순서대로 실행하고 성공 후 `Workstation/WIRIS.md`를 갱신한다.

## Approved GUI-decoupling implementation amendment

1. 고속 producer/느린 GUI 재현에 queue depth, replaced/dropped, GUI heartbeat, RSS
   high-water/slope, Stop ACK 계측을 추가한다.
2. preview latest slot, detection metadata, native request/data, evidence job/ACK, control Event/pipe를
   독립 channel로 분리한다. Stop/Error는 image queue를 거치지 않는다.
3. child가 preallocated shared-memory native ring을 소유하고 모든 decoded frame에 exact
   `FrameKey`/timestamp/pixel contract를 부여한다. Pair 채택 시 좌우 slot을 pin하고 ACK 후
   release한다.
4. pair coordinator가 light detection descriptor만 받아 exact 좌우 pair를 채택한 뒤 native
   slot을 요청한다. overwrite/세대 불일치는 `EVIDENCE_FRAME_UNAVAILABLE`로 거부한다.
5. evidence writer는 temporary write, fsync, atomic publish, decode/dimension check, SHA-256,
   ledger append가 모두 완료된 뒤에만 durable ACK를 보낸다.
6. pairing, distance, optical-alignment compute를 immutable calibration/attitude/GNSS/method snapshot과
   함께 dedicated measurement worker로 이전한다.
7. GUI는 preview/result/status latest snapshot만 timer 주기로 그리고 camera별 ring/writer/age/
   replaced/dropped high-water를 낮은 빈도로 표시한다.
8. 2초 GUI stall, one-side RTSP loss, writer failure, Stop/reconnect, repeated generation 테스트 후
   10분 synthetic qualification과 60분 dual-camera soak를 실행한다.
