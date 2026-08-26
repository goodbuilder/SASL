# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-08-26 — Goal scope and evidence contract accepted

- 사용자 `/goal`의 허용·금지·encoder A/B·진단 13단계·완료/중단 조건을 그대로 계약으로
  사용한다.
- 수정 전 재현과 동일 bitstream deterministic decoder A/B가 필수다.
- 정상 screenshot 한 장, 단순 미재현, bitrate/GOP/IDR masking은 완료 근거가 아니다.
- user dirty worktree와 unrelated active change packet을 보존한다.

## 2026-08-26 — Baseline facts

- baseline encoder readback은 양쪽 `0/1/90/0`이며 설정 파일의 IDR 기대값과 다르다.
- 최초 화면은 정상으로 보였고 Left/Right frame rate는 서로 달랐다.
- artifact 원인은 아직 `UNRESOLVED`; source edit approval은 요청하지 않았다.

## 2026-08-26 — Pre-change artifact reproduced

- 두 개의 명확한 Left GUI macroblock/previous-frame corruption 사건을 window-id 녹화에서
  확인했다. hand motion이 높은 변화량을 만들 때 발생했다.
- 사건 당시 Left는 software fallback pipeline이었다. 그러므로 NVIDIA decoder만 바꾸는
  조치는 근본 수정으로 승인하지 않는다.
- 정상 구간 passive capture는 StereoWIRIS flow와 Workstation flow를 분리해 해석한다.
  서로 다른 live session 결과를 decoder A/B 근거로 혼용하지 않는다.

## 2026-08-26 — Root cause isolated and minimal fix proposed

- 시작점은 camera RTSP/RTP의 forward sequence omission과 incomplete H.264 AU다.
- artifact를 보이게 하는 직접 계층은 `nvh264dec` error concealment output이다. NVIDIA-only
  frame은 corrupt flag 없이 downstream에 전달된다.
- Qt stride/lifetime, multiprocessing frame queue, YOLO는 원인이 아니다.
- Software 강제는 artifact를 숨기지만 0~1.4 Hz 장기 정지 구간이 있어 기본 변경으로 채택하지
  않는다.
- depay `wait-for-keyframe`은 WIRIS의 non-IDR intra stream에서 영구 대기하므로 채택하지 않는다.
- 제안은 RTP gap을 기존 tracker에서 조기 검출하고 corrupt decoded output을 막은 뒤 기존
  Auto software fallback을 즉시 실행하는 것이다.
- camera encoder A/B 값은 모두 원래 `0/1/90/0`으로 복원했다.

## 2026-08-26 — RTP omission boundary refined

- 완전 재조립된 TCP byte stream 내부에서 같은 SSRC의 RTP sequence가 14만큼 비어 있으므로
  host NIC, kernel capture, TCP delivery, GStreamer jitter/depay는 직접 누락 계층이 아니다.
- ipTIME U1003C/RTL8153는 USB 3.0 및 `1000/full`이고 모든 link/RX error/drop counter와
  artifact 시각 kernel log가 정상이라 직접 원인 가설을 배제한다.
- `GEBR=0/QP=1` constant-quality burst와 복수 RTSP session에 의한 camera sender
  backpressure를 현재 가장 유력한 내부 trigger로 둔다. 정지 장면의 복수 session에서는
  누락 0이므로 동일 motion을 사용한 single-session/multi-session A/B 전에는 firmware queue
  overflow를 확정 사실로 표현하지 않는다.

## 2026-08-26 — Saved file is visually concealed, not bitstream-clean

- GStreamer encoded recording tee가 NVIDIA decoder 앞에 있으므로 NVIDIA concealment artifact는
  raw MP4 pixel로 저장되지 않는다.
- 실제 pre-change raw MP4는 strict software full-decode를 통과하지 못했다. 일반 player의
  tolerant decoder가 손상 frame을 skip/conceal하여 육안상 정상으로 보인 것이다.
- post-change 완료 판정은 단순 재생 육안 확인이나 ffprobe container open 성공이 아니라
  `-xerror` 상당의 strict software full-decode와 frame continuity evidence를 요구한다.

## 2026-08-26 — Packet-scoped implementation approved

- 사용자가 12:45 KST에 `approval-summary.md`에 제시한 구현을 시작하라고 명시적으로 승인했다.
- 승인 범위는 StereoWIRIS의 RTP forward-gap tracker, decoded-output gate, 기존 Auto software
  fallback trigger, 집중 tests, 승인된 StereoWIRIS 문서다.
- CameraPane/layout/settings schema/encoder/Workstation/common runtime 변경은 승인 범위 밖이다.

## 2026-08-26 — First implementation rejected by motion acceptance

- gap-triggered Software fallback은 정지 장면에서 깨끗했지만 hand/body motion 중 양쪽 GUI에
  macroblock band와 이전 영역 혼합을 다시 출력했다. producer/queue/QImage hash는 모두 같아
  Qt가 아니라 software decoded pixels가 이미 오염된 상태였다.
- Software overload 시 depay tracker는 30초에 1,147 events/5,150 missing을 보고했지만 같은
  시각의 6초 passive TCP capture는 양쪽 RTP missing 0이었다. `rtspsrc drop-on-latency` 이후
  loss feedback이므로 gap에서 Software로 내리는 정책을 폐기한다.
- fallback 직후 양쪽 camera가 RTSP 503을 반환한 실행도 있어 health fallback transition에만
  bounded 0.5/1/2초 retry가 필요하다.

## 2026-08-26 — Same-session clean-intra recovery proposed

- 저장 same-bitstream의 1,521 AU 중 non-delta intra AU 18개를 확인했다. WIRIS는 IDR NAL 5가
  없어도 약 4–9초 간격으로 독립 intra recovery boundary를 제공한다.
- 90초 NVIDIA same-session gate A/B에서 Right RTP 46 packet gap 후 1.714초에 clean intra로
  재개했고 corrupt output/PTS regression/GStreamer error는 0이었다.
- RTP gap은 decoder failure가 아니므로 현재 decoder를 유지하고 모든 decoded output을 clean
  intra PTS까지 차단한다. 기존 Auto Software fallback은 first-frame/stall/bus health failure에만
  유지하는 refined scope로 재승인을 요청한다.

## 2026-08-26 — Same-session clean-intra recovery approved

- 사용자가 14:03 KST에 refined decoder policy와 별도 startup behavior packet을 모두 승인했다.

## 2026-08-26 — Recording-start worker replacement race reproduced

- 14:45:54 KST에 세션 기록을 시작하자 기존 preview worker는 종료됐지만 새 Left/Right worker가
  `nvh264dec`와 `avdec_h264` 모두에서 initial PLAYING 전환에 실패했다. session directory는
  생성됐으나 raw MP4가 생기지 않았고 화면은 `YOLO 시작 중...`에 머물렀다.
- 카메라 session이 해제된 뒤 같은 앱에서 YOLO를 다시 시작하자 두 NVIDIA worker가 즉시
  성공했으므로 model load, Qt, 고정 decoder capability 실패가 아니라 RTSP owner 교체 경계의
  transient start race로 분리했다.
- `failed to enter PLAYING state`에만 0.5/1/2/4초 bounded retry를 추가했다. 다른 오류와 stop은
  확대하지 않으며 회귀 테스트를 포함한 `tests/test_yolo_stream.py` 60개가 통과했다.

## 2026-08-26 — Hardware acceptance remains incomplete

- 한 실행은 14분 29초에 USB hub 전체 disconnect가 발생해 무효 처리했다.
- 후속 실행은 Left macroblock smear가 한 번 보여 무효 처리했고 frame identity hash 일치는
  corruption이 Qt 이전 decoded pixels에 이미 있었음을 다시 확인했다.
- 새 수정 PID 501423을 실행했지만 14:54:27 KST부터 RTL8153가 `carrier off`여서 양쪽 camera
  ping이 실패한다. 물리 링크 복구 전에는 recording-start 및 15분 acceptance를 완료로 기록하지
  않는다.

## 2026-08-26 — Recording-start diagnosis corrected at filesink boundary

- 전원 복구 후 bounded retry를 포함한 새 process에서도 기록 시작은 동일하게 실패했다. 따라서
  앞선 “RTSP owner release race”는 최종 원인 가설에서 제외한다.
- 첫 결함은 `_prepare_video_recording_paths()`가 session root만 만들고 `video/` parent를 만들지
  않은 것이었다.
- parent 생성 후 `GST_DEBUG=2`가 filesink ENOENT의 실제 target을
  `"'/home/.../바탕화면/.../raw_left.mp4'"`로 보고했다. shell용 `shlex.quote()`의 single quote를
  `Gst.parse_launch()`가 실제 property 문자로 보존한 것이 가장 좁은 확정 원인이다.
- GStreamer double-quoted escaping과 video-parent 생성을 적용한 새 process/session에서 양쪽
  recording worker가 `nvh264dec`로 시작하고 MP4 bytes가 증가했다. 집중 회귀는 61+4 passed다.

## 2026-08-26 — Fragmented MP4 corruption isolated and removed

- 실제 앱의 `fragment-duration=1000` MP4에서 strict decoder가 `0x6d646174`(`mdat`)를 NAL
  length로 읽고 exit 1했다.
- 동일 live H.264 tee에서 fragmented branch만 exit 1, plain mp4mux branch는 exit 0이었다.
  camera payload/session 차이가 아니라 MP4 fragment sample-offset 계층으로 분리됐다.
- direct H.264 copy를 유지한 채 fragment-duration만 제거했다. 실제 앱의 71초 Left/Right 파일은
  orderly EOS 2.4초 안에 finalized되고 ffprobe/strict full decode exit 0을 통과했다.

## 2026-08-26 — User-triggered STUS reconnect is planned, not fallback

- 최종 session 초기에 관측한 worker PID 변경은 사용자가 `PC UTC로 시간 설정`을 실행한 결과다.
  로그의 GDTI/STUS 성공과 양쪽 `nvh264dec` 재연결이 일치하므로 unexpected decoder fallback으로
  세지 않는다. 장기 acceptance 기준은 STUS 후 두 새 worker의 frame이 모두 확인된 뒤로 재설정한다.

## 2026-08-26 — Code-mitigation plan-only handoff

- 사용자는 RTP omission의 camera-side 원인 수정이 아니라 코드로 가능한 GUI artifact 완화 계획을
  요청했다.
- 기존 승인 범위인 same-session clean-intra gate를 유지하며, 이번 turn은 구현 재개나 새 source
  수정 승인으로 해석하지 않는다.
- 계획은 last-clean preview 유지, 모든 decoded consumer 차단, health-only fallback 보존,
  단일 RTSP owner, 15분 실장비 acceptance와 strict MP4 regression을 명시한다.
- 기존 핵심 구현과 focused evidence는 보존하고, 남은 실행 gate는 fresh PID의 깨끗한 15분
  Left/Right motion+recording 검증이다.

## 2026-08-26 — Motion acceptance rejects gate-only responsiveness

- session `20260826_170809_400538263`과 930초 window recording에서 macroblock을 숨기는 대신
  움직임 때 수초 단위 freeze가 반복됐다.
- Left는 0.5초 초과 정지 23회/합계 81.633초/최대 9.109초, Right는
  15회/합계 78.578초/최대 24.425초였다. 사용자가 같은 끊김을 직접 관찰했다.
- producer/queue/QImage mismatch와 reorder는 모두 0이므로 GUI thread나 QImage가 아니라
  discontinuity 이후 clean intra를 기다리는 fail-closed gate가 보이는 정지다.
- 장기 MP4는 양쪽 모두 moov를 포함해 finalize/ffprobe됐지만, 실시간 responsiveness가 FAIL이므로
  변경 packet을 완료하지 않고 `WIRIS.md` 성공 문서화도 보류한다.

## 2026-08-26 — TCP unbounded late-packet preservation rejected

- TCP `drop-on-latency=false`는 packet drop을 줄이는 대신 약 108초에 frame transit/transport
  delay를 약 3.25/3.22초까지 누적시켰다.
- 실시간 운용 지연 계약에 맞지 않아 source와 test 실험 변경을 즉시 원복했고
  `tests/test_yolo_stream.py` 65개가 다시 통과했다.
- 실험 중 별도 Workstation이 동일 카메라를 점유해 Right가 시작되지 않았으므로 이 실행은
  양쪽 비교 근거로 사용하지 않는다.

## 2026-08-26 — Encoder burst-control amendment required

- 누락된 H.264 reference bytes는 코드가 복구할 수 없으므로 QP1 고화질 burst를 유지하면서
  corruption, freeze, low latency를 동시에 0으로 만드는 code-only 경로는 확인되지 않았다.
- 다음 권고는 단일 RTSP owner에서 bitrate `0`, GOP/IDR 유지, QP만 `1 -> 5`로 한 변수 A/B하고
  motion freeze/bandwidth/화질을 측정한 뒤 시험 직후 `QP=1`로 원복하는 것이다.
- 이 시험은 기존의 encoder 불변 승인 범위를 넘으므로 구현과 장비 변경을 멈추고 사용자 승인을
  다시 요청한다.

## 2026-08-26 — Exact sender omission and extended A/B results

- exact single Right TCP pcap은 TCP byte gap 0/kernel drop 0인데 main RTP sequence 137개가
  4건에 걸쳐 빠진 것을 보였다. WIRIS RTSP/RTP sender 이전 omission으로 확정한다.
- 사용자가 문제 지속을 확인한 뒤 Right QP10을 시험했다. 평균 bitrate는 44.66 대비
  37.55 Mbps로 낮아졌지만 문제가 남아 QP1로 복원했다.
- RTP latency 500 ms에서도 문제가 남아 100 ms로 복원했다.
- SEBR 20 Mbps는 Right RTSP timeout/0-byte recording을 만들고 firmware가 QP/GOP/GETH까지
  17/16/FALSE로 바꿨다. SEBR0→QP1→GOP90→SETH TRUE 순서로 복원했다.
- 실제 UDP media pcap(kernel drop 0)은 Left 115 gaps/241 missing, Right 306 gaps/339 missing을
  보였고 GUI receive가 2.2 Hz까지 내려갔다. UDP는 더 잦은 freeze를 만들며 해결책이 아니므로
  app을 TCP/100 ms로 복원했다.
- 최종 양쪽 camera readback은 `GETH=TRUE`, encoder `0/1/90/0`; app persisted setting은
  `TCP/100 ms`다.

## 2026-08-26 — Workstation-only single-source recording proposed

- 사용자는 Legacy/Workstation lock 강제 대신 Workstation만 운용하기로 했다.
- Workstation은 현재 preview GStreamer와 encoded-save FFmpeg가 각각 독립 RTSP client를 열어
  recording 중 camera당 두 session을 만든다. Workstation 하나만 실행해도 양쪽 합계 네 RTSP
  session이 되어 camera sender의 per-client 복제 부하를 키운다.
- 권고 구조는 camera당 하나의 `rtspsrc -> rtph264depay -> h264parse -> integrity gate -> tee`를
  두고, 한 branch는 NVIDIA decode/appsink/GUI·YOLO, 다른 branch는 mux/filesink 저장으로 보낸다.
- integrity gate는 tee 앞에 둬 RTP forward gap, PTS 누락, DISCONT/CORRUPTED AU와 dependent
  delta chain이 preview와 운영 저장물 양쪽에 들어가지 않게 한다.
- 이 구조는 app이 만드는 추가 RTSP backpressure를 제거하지만, exact single-session pcap에서
  증명된 camera sender omission 자체를 보장 제거하지는 못한다. 그 부분은 vendor/firmware
  해결이 필요하다.

## 2026-08-26 — Workstation implementation and backend policy approved

- 사용자는 Workstation single-rtspsrc GStreamer tee recording과 pre-tee RTP/AU integrity gate
  구현 계획을 승인했다.
- gap 복구는 같은 RTSP session에서 clean non-delta intra를 기다리고, 30초를 넘기면 파일을
  정상 종료한 뒤 manual reconnect 필요 상태로 전환한다.
- gap이 발생해도 저장 파일은 분할하지 않고 손상 chain을 제외한 하나의 MP4를 유지한다.
- OpenCV/FFmpeg와 PyAV 선택은 허용하며 decoded BGR 재인코딩을 유지한다. 이 경로는
  `decoded_reencode_unverified`로 명시하고 RTP/AU 무결성을 주장하지 않는다.
- 성공 검증 전까지 `Workstation/WIRIS.md` 완료 문서화는 보류한다.

## 2026-08-26 — GUI-decoupled native/evidence/measurement architecture approved

- 실장비 motion/person-detection 시험에서 Workstation parent RSS가 약 0.5 GiB에서
  4 GiB 이상으로 지속 상승하고 GUI가 not-responding으로 전환됐다. 원인은 accepted
  detection마다 1280x720 native raster를 Qt queued `QImage`로 GUI에 전달하는 경계다.
- 임시 `BlockingQueuedConnection`은 RSS를 안정화했지만 GUI stall이 native delivery와
  Stop/control까지 막을 수 있어 최종 구조로 사용하지 않기로 했다.
- 사용자는 child-side bounded native ring, exact `FrameKey=(camera, generation, frame_id)`,
  pair-selected pin/lease, off-GUI measurement/evidence worker, durable ACK/hash, latest-only GUI,
  independent Stop/Error control plane을 승인했다.
- timestamp-nearest native 대체는 제거하고 exact frame이 없으면
  `EVIDENCE_FRAME_UNAVAILABLE`로 fail closed한다.
- CUDA inference는 `max_rate=0` 무제한 상태를 제거하고 decode/recording과 독립적인
  명시적 상한을 사용한다.
