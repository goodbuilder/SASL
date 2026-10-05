# Verification

## 2026-08-26 Workstation GUI-decoupling amendment

- PASS: full Workstation automated suite, `1262 passed, 1 skipped`.
- PASS: 600-second synthetic qualification with a deliberate 2-second GUI/consumer stall; bounded
  preview/metadata/control queues, native ring drop 0, 296 durable evidence pairs, writer failure 0,
  and Stop under 12 ms.
- PASS: normal shared-memory cleanup; ring count equals active cameras and returns to zero on exit.
- PASS (Left only): live single-session preview, bounded RSS, repeated integrity recovery.
- BLOCKED (external device): Right port 8554 accepts TCP but its RTSP server does not respond after
  `SETH TRUE` and zero-client cooldown. Dual-camera 60-minute soak awaits explicit reboot approval.

Detailed evidence: [workstation-gui-decoupling-qualification-20260826.md](evidence/workstation-gui-decoupling-qualification-20260826.md).

## Requirement-to-evidence matrix

| Slice | Oracle | Baseline status |
|---|---|---|
| Git/device freeze | git identity + direct SDK readback | PASS |
| Initial artifact reproduction | screenshot/frame evidence + timestamps | PASS |
| RTP/NAL/AU continuity | passive capture/parser | PASS |
| Same-bitstream decoder A/B | one encoded artifact replay | PASS |
| BGR/queue/QImage identity | same representation hash/stride | PASS |
| Stored file | ffprobe + software full decode | FAIL (pre-change sample) |
| Recording-start worker replacement | old/new PID + PLAYING status + MP4 growth | PASS after filesink fixes |
| Stored MP4 mux | same-input mux A/B + actual app strict decode | PASS short run; 17-minute files finalized/ffprobe PASS, strict decode not run after motion FAIL |
| 15-minute Left/Right acceptance | time series + periodic frame evidence | FAIL; motion-triggered recovery freezes violate responsiveness |
| Exact camera sender boundary | full TCP pcap reassembly + RTP sequence | FAIL upstream: 137 RTP packets omitted |
| UDP transport A/B | full UDP pcap + kernel counters + GUI Hz | FAIL: 580 missing, 2.2 Hz freeze |

## Focused test evidence

- RED: `test_gstreamer_initial_playing_failure_uses_bounded_start_retry` failed because
  `gstreamer_start_retry_delay_s` did not exist.
- GREEN: the same focused test passed after implementing the narrow delay policy.
- `tests/test_yolo_stream.py -q` → latest `65 passed`.
- startup/session/recovery exact PyQt selection → `5 passed`.

## Regression and application evidence

Not run before source change.

## Performance, load, and target-hardware evidence

- Hardware baseline: RTX 4050 Laptop GPU, driver 595.84, GStreamer 1.20.3.
- At 11:01 KST: GPU 7%, NVDEC 3%, memory 497 MiB, temperature 49 C.
- Left worker 36 threads; Right worker 22 threads. Time-series still required.

## Feature isolation and contract evidence

No source change. Camera preview is running while unrelated Pixhawk connection reports failure,
which supports current feature isolation.

## Visual authenticity and dynamic-layout stability evidence

Baseline screenshot captured at 2740x1672 app client size. No layout change proposed.

## Visual and hardware/manual evidence

`evidence/baseline-20260826T1103KST.png`: both live frames visible, no artifact in one sample.
This is baseline only and does not satisfy acceptance.

- `evidence/prechange-event1-frame-752.png`: 11:14:38.657 KST, Left 9.3 Hz,
  hand motion 중 macroblock/previous-frame corruption.
- `evidence/prechange-event2-frame-1437.png`: 11:16:55.657 KST, Left 15.2 Hz,
  화면 절반 이상의 명확한 corruption.
- `evidence/prechange-recovery2-frame-1458.png`: 약 4.2초 뒤 정상 복구.
- crop 증거는 `prechange-event1-left-crop-frame-752.png`와
  `prechange-event2-left-crop-frame-1437.png`에 보존했다.

정상 구간 Left passive capture(11:22:50 KST, 8.25 s)는 두 client flow를 분리했다.
StereoWIRIS port 48484는 internal AU incomplete 0, Workstation port 59608은 한 AU에서
RTP sequence 14개가 누락됐다. 이 정상 구간은 재현 사건 시각과 다르므로 원인 확정
근거가 아니라 parser/flow 분리 검증 근거다.

동일-session/deterministic evidence:

- encoded MKV SHA-256: `e52f232da77733c55e517496aa8838ff36ab5937beb6c9bf1f977f56fa3a6826`.
- packet capture SHA-256: `bd0803414620b02a024951c4013b338dc6ced1f9ead2dfc49f2312b41bd2517a`.
- deterministic replay: NVIDIA 1521, software 1106, NVIDIA-only 415.
- paired frame shape/format/stride/bytes 계약은 모두 일치.
- `evidence/deterministic-replay-summary.json`과 `evidence/nvidia-only-frame-*.png`에
  요약/대표 frame을 보존했다.
- target flow passive capture: TCP gap 0, kernel drop 0, RTP missing 14, incomplete AU 1.
- target NIC ipTIME U1003C/RTL8153: USB 3.0, Ethernet `1000/full`, RX/TX
  error/drop/missed/FIFO/CRC/carrier 0, artifact 시각 reset/link flap/watchdog 0.
- omission 전후 100 ms TCP retransmission 0; 단일 harness session의 100 ms 최대
  87.69 Mbit/s, rolling 1초 최대 80.24 Mbit/s.
- static Right two-session follow-up(34.961 s): 45.577 + 44.679 Mbit/s,
  TCP byte gap 0, RTP missing 0. Motion/session-count controlled A/B remains required.
- actual saved MP4 sample `20260825_155445_030256238`: both Left/Right expose H.264
  macroblock/level-prefix errors; strict `ffmpeg -xerror` full decode exits 1. Tolerant decode
  logs the same errors but exits 0, explaining why ordinary playback can appear clean.
- first post-change motion run: producer→queue/QImage mismatch 0 and reorder 0이었지만 양쪽
  decoded preview에 macroblock band가 재현되어 FAIL. 증거는
  `evidence/postfix-v1-motion-corruption-*.png`에 보존했다.
- software overload 6초 pcap: Left 31,054 packets/RTP missing 0, Right 33,248 packets/RTP
  missing 0, TCP byte gap 0, kernel drop 0. 같은 시간 depay tracker의 대량 missing은
  post-jitter drop임을 확인했다.
- NVIDIA same-session gate 90초 stress A/B: Right gap missing 46 → clean intra recovery
  1.714 s, decoded 191/emitted 191/suppressed corrupt output 0/PTS regression 0/error 0.
  Left decoded/emitted 139/139, error 0. recovery 직후 PNG는 clean.
- `discard-corrupted-frames=true` A/B는 NVIDIA-only 415로 변화 없음.
- `wait-for-keyframe/request-keyframe`은 IDR0/IDR1 모두 frame 0으로 실패.
- Software GUI A/B는 artifact 대신 frame 0~1.4 Hz 정지를 보여 성능 저하가 과도했다.
- encoder `SEIR`와 `SEBR` A/B는 효과 없음/RTSP timeout으로 원래값 복원 완료.
- recording-start reproduction at 14:45:54 KST: 새 session은 생성됐지만 교체된 Left/Right
  worker가 NVIDIA와 software에서 initial PLAYING 전환에 실패했고 raw MP4는 생성되지 않았다.
  camera session 해제 뒤 수동 YOLO 재시작은 양쪽 NVIDIA로 성공해 transient owner-release
  경계로 분리했다.
- post-change 14분 29초 window는 14:35:35 KST USB hub 전체 disconnect 때문에 acceptance에서
  제외했다. 후속 window는 Left macroblock smear 1건 때문에 실패 처리했다.
- bounded initial retry가 포함된 새 앱 PID 501423을 실행했다. 그러나 kernel은 14:54:27 KST
  `enx588694fce07e: carrier off`를 기록했고 adapter는 USB에 남아 있는 반면 NetworkManager는
  unavailable, 양쪽 camera ping은 100% loss다. 물리 link 복구 뒤 재검증해야 한다.
- 전원/물리 link 복구 후 encoder readback은 양쪽 모두 다시 `0/1/90/0`, ping loss 0%였다.
- recording-start 실패의 bus detail은 literal-single-quote filesink path ENOENT였다. video parent와
  GStreamer property serialization을 고친 session `20260826_153835_987796434`에서 양쪽
  `nvh264dec` recording worker와 MP4 byte growth를 확인했다.
- same-live-input MP4 mux A/B는 fragmented exit 1/plain exit 0이었다. 실제 앱 plain MP4 session
  `20260826_154943_612706709`의 Left 501,460,047 bytes/71.2922초와 Right 429,074,882 bytes가
  ffprobe와 strict software full decode exit 0을 통과했다.

최신 motion acceptance와 사용자 관찰:

- session `20260826_170809_400538263`은 Left 1059.9166초/7,535,150,621 bytes,
  Right 1057.1679초/6,770,286,099 bytes로 정상 finalization되고 ffprobe를 통과했다.
- 930초 window-id recording `/tmp/wiris-acceptance-v2-final-window.mkv`와
  `/tmp/wiris-acceptance-v2-20260826.jsonl`을 함께 수집했다.
- producer→queue와 queue→QImage mismatch 및 reorder는 양쪽 모두 0이었다.
- 그러나 움직임 구간에서 Left는 0.5초 초과 정지 23회, 합계 81.633초, 최대 9.109초였고,
  Right는 15회, 합계 78.578초, 최대 24.425초였다. 사용자가 화면 끊김을 직접 확인했다.
- 이는 Qt/queue 손상이 아니라 discontinuity 뒤 clean intra까지 encoded/decoded output을
  fail-closed 차단한 시간과 일치한다. artifact 안전성은 개선됐지만 실시간 responsiveness
  acceptance는 FAIL이다.
- TCP `drop-on-latency=false` 진단은 약 108초 만에 Left frame transit/transport delay가
  약 3.25/3.22초까지 누적돼 낮은 지연 계약에 실패했다. 같은 시각 별도 Workstation client가
  Right RTSP를 점유해 양쪽 비교는 invalid였으며, 실험 source 변경은 즉시 원복했다.
- exact Right TCP pcap SHA `fea18f0e...47ca`: 477,692,445 reassembled bytes, TCP byte gap 0,
  kernel drop 0, RTP gaps 4/missing 137.
- valid UDP pcap SHA `45e459...923`: kernel drop 0; Left gaps 115/missing 241, Right gaps
  306/missing 339. UDP는 gap 빈도가 증가하고 receive 2.2 Hz로 FAIL.
- QP10/latency500은 사용자 관찰에서 문제 지속, SEBR20M은 Right RTSP timeout 및 camera setting
  side effect로 FAIL. 모든 camera/app setting을 원래값으로 복원했다.

## Residual conditions

최초 screenshot 사건과 exact same RTP capture는 없지만 독립 same-session tee와 deterministic
replay가 동일 현상을 재현했고 packet capture가 upstream loss를 증명했다. source gate와 MP4
finalization은 적용됐지만 motion 시 누적 freeze 때문에 operator acceptance는 실패했다. 다음
단계는 단일 RTSP owner에서 bitrate `0`을 유지한 QP burst-control A/B이며 별도 승인이 필요하다.

후속 QP10/SEBR20M/UDP A/B도 문제를 제거하지 못했다. 현재 허용 범위에서 빠진 reference
payload를 재생성하거나 real IDR을 강제할 수 없으므로 firmware/vendor sender fix 없이는
corruption 0과 freeze 0을 동시에 만족하는 경로가 확인되지 않았다.
