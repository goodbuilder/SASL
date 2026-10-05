# Current Behavior

## Version and worktree context

- Baseline observed: `2026-08-26 11:00 KST`.
- Branch: `fix/workstation-optical-pose-evidence-geometry-hardening`.
- HEAD: `c4397995d864576c966702cfd3c398b6032f8990`.
- 저장소 루트와 StereoWIRIS/Workstation에는 광범위한 기존 사용자 변경이 있다.
  reset/restore/checkout/stash/clean 없이 보존한다.
- 별도 root packet `docs/changes/20260826-canonical-gstreamer-runtime`이 implementing
  상태다. 공통 runtime/decoder policy와 겹치는 변경은 승인 전에 충돌을 재평가한다.

## Reproduction

- `main.py` PID `111845`, Left worker `294317`, Right worker `294318`이 실행 중이다.
- Left/Right RTSP TCP 연결과 ping이 정상이다.
- 최초 baseline screenshot에는 artifact가 없었다.
- baseline GUI: Left `1280x720 14.3 Hz, YOLO 12.0 Hz`, Right
  `1280x720 17.6 Hz, YOLO 16.0 Hz`.
- 최초 재현은 완료됐다. hand motion 중 Left preview에서 macroblock corruption과
  이전-frame 잔상이 두 번 발생했다.
  - `2026-08-26 11:14:38.657 KST`: window recorder frame 752, Left 9.3 Hz.
  - `2026-08-26 11:16:55.657 KST`: window recorder frame 1437, Left 15.2 Hz.
- 두 번째 사건은 약 4.2초 뒤 frame 1458에서 정상 복구됐다. 서로 다른 Hz에서도
  재현되어 단순 FPS 감소와 동일시할 수 없다.

## Observed frontend behavior

`측정 > 실시간`에서 두 preview를 QLabel로 동시에 표시한다. latest-only preview timer는
67 ms 간격으로 pending QImage를 GUI thread에서 소비한다. 화면은 현재 정상처럼 보이나
과거 live 운용에서 찢어짐/macroblock/잔상이 보고됐다.

## Observed backend and persistence behavior

- 설정은 `gstreamer`, `auto`, `tcp`, latency `100 ms`, preview width `640`이다.
- baseline encoder readback은 양쪽 모두 `bitrate=0, QP=1, GOP=90, IDR=0`이다.
- Left serial/article: `20204-046-2505` / `WWE-US-640-SNL-256-LRF`.
- Right serial/article: `20207-046-2507` / `WWE-US-640-FNL-128-LRF`.
- 양쪽 firmware는 `2.18.72`다.
- GPU baseline: GPU 7%, NVDEC 3%, 497 MiB, 49 C.
- Left worker 36 threads, Right 22 threads다. `/proc/<pid>/maps`에서 Left만
  `libgstlibav.so`가 로드되어 Left가 `nvh264dec` stall 후 `avdec_h264`로 fallback한
  상태임을 확인했다. Right는 `libgstnvcodec`/`libnvcuvid`만 로드됐다.
- 따라서 재현된 Left artifact는 NVIDIA-only 가설로 설명되지 않는다.
- 별도 Workstation process도 11:11 KST부터 동일 Left/Right RTSP에 두 번째 client로
  접속 중이다. 이 process는 범위 밖이므로 종료하거나 수정하지 않는다.

## Documentation, code, test, and runtime conflicts

- 저장 JSON은 IDR `1`을 기대하지만 실장비 `GEIR VISIBLE`은 양쪽 `0`이다.
- `Auto`는 visual artifact만 발생하고 frame stall이 없으면 software로 전환하지 않는다.
- 현재 소스는 compressed H.264 queue를 non-leaky로 바꿨지만 Git HEAD의 이전 경로는
  decoder 앞에서 `leaky max-buffers=1`로 compressed frame을 버릴 수 있었다.
- 현재 source가 runtime process에 로드된 뒤 실행됐지만, 실제 artifact 재현과 어느
  pipeline generation에서 발생했는지는 이제 Left software fallback pipeline으로 좁혀졌다.

## Root-cause evidence

- 같은 RTSP session을 encoded/NVIDIA/software 세 branch로 tee한 120초 진단에서
  NVIDIA는 1,534 frame, software는 1,108 frame을 출력했다. software frame은 전부 같은
  PTS의 NVIDIA frame과 pair됐고 NVIDIA에만 426 frame이 남았다.
- 저장된 동일 MKV를 deterministic replay한 결과도 NVIDIA 1,521, software 1,106,
  NVIDIA-only 415로 재현됐다. 따라서 서로 다른 live session 차이가 아니다.
- 두 decoder의 paired BGR 계약은 `1280x720`, `BGR`, stride `3840`, bytes
  `2,764,800`으로 동일했다. NumPy/QImage 이전에 이미 출력 집합이 다르다.
- NVIDIA-only frame은 buffer flags `0`으로 정상처럼 전달되지만 block/잔상 오염을
  포함한다. `nvh264dec discard-corrupted-frames=true`도 동일 replay에서 415 frame을
  제거하지 못했다.
- 같은 harness TCP flow의 25.53초 passive capture는 kernel drop 0, TCP byte gap 0인데
  RTP sequence `11570 -> 11585`로 14 packet이 누락됐고 한 AU가 marker/FU end 없이
  불완전했다. 손실은 GStreamer/Qt 이전의 camera RTSP/RTP stream에 존재한다.
- 이 gap은 같은 SSRC에서 `FU-A(orig=1, S=0, E=0)` 뒤 marker/FU end 없이 발생했고,
  다음 수신 packet은 새 RTP timestamp의 AUD(NAL 9)였다. 즉 IP/TCP packet이 LAN에서
  사라진 것이 아니라 camera RTSP sender가 한 encoded AU의 나머지 RTP fragment를
  해당 TCP byte stream에 넣지 않았다.
- 수신 NIC는 ipTIME U1003C의 RTL8153(`r8152`)이며 USB 3.0, Ethernet `1000/full`로
  동작한다. 누적 RX/TX error/drop/missed/FIFO/CRC/carrier는 모두 0이고 artifact 시각의
  USB reset/link flap/NETDEV watchdog도 없었다. gap 전후 100 ms에 TCP retransmission도
  없으므로 이 adapter는 직접 누락 계층이 아니다.
- `GEBR=0`은 bitrate control 없는 constant-quality mode이고 `QP=1`이다. harness 단일
  session은 100 ms 구간 최대 87.69 Mbit/s, rolling 1초 최대 80.24 Mbit/s까지 burst했다.
  당시 동일 camera의 복수 RTSP client가 있었으므로 camera RTSP sender의 per-client
  복제/송신 queue backpressure가 내부 trigger로 가장 유력하다. 다만 Workswell 공개 SDK와
  datasheet에는 link 정격, 다중 client 한계, sender queue telemetry가 없어 firmware 내부
  overflow까지는 아직 확정하지 않는다.
- 정지 장면에서 Right의 두 session을 34.961초 재검사한 결과 각 45.577/44.679 Mbit/s,
  TCP byte gap 0, RTP missing 0이었다. 따라서 복수 session 자체가 충분조건은 아니고,
  motion에 따른 constant-quality bitrate burst와의 결합이 검증 대상이다.
- software timeline은 손실 후 다음 intra frame까지 frame을 출력하지 않는다. NVIDIA는
  그 구간을 error-concealment frame으로 출력해 GUI artifact를 만든다.
- GStreamer 저장 branch는 `h264parse -> tee`에서 decoder 앞 encoded H.264를 MP4에
  remux하고 GUI branch만 `nvh264dec -> BGR -> QImage`를 통과한다. 따라서 NVIDIA가 만든
  macroblock/잔상 frame 자체는 MP4에 기록되지 않는다.
- 그러나 pre-change 저장 MP4가 bitstream 기준으로 정상인 것은 아니다. 짧은 실제 session
  `20260825_155445_030256238`의 Left/Right 모두 ffprobe에서 `Invalid level prefix`,
  `corrupted macroblock` 등을 보고했고 `ffmpeg -xerror` full decode가 exit 1로 실패했다.
  일반 tolerant decode는 같은 오류를 출력하면서 exit 0으로 끝나므로 플레이어가 손상 frame을
  폐기/보정하고 다음 usable frame으로 진행해 육안상 정상처럼 보일 수 있다.
- deterministic same-bitstream에서도 NVIDIA 1,521 frame 대비 software는 1,106 frame만
  출력했다. software/player 쪽에서 415개 손상 의존 frame을 보여주지 않은 것이 저장 재생과
  GUI가 다르게 보이는 직접 이유다.
- `rtph264depay wait-for-keyframe/request-keyframe`은 실제 `GEIR=0` stream에서 60초간
  frame 0이었다. `SEIR=1` A/B도 7.38초/99 AU에서 IDR NAL type 5가 0이고 frame 0이라
  효과가 없어 `SEIR=0`으로 원복했다.
- `SEBR=12,000,000` A/B는 RTSP 신규 handshake timeout으로 유효 비교가 되지 않아
  즉시 원래 `SEBR=0`으로 복원했다. 최종 camera readback은 양쪽 원래값이다.
