# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

StereoWIRIS는 기존 Auto/NVIDIA/Software 선택과 화면 배치를 유지합니다. 어느 decoder에서든
RTP packet 누락이 확인되면 손상 가능 frame을 GUI로 보내지 않고, 같은 RTSP/decoder session에서
`h264parse`가 확인한 다음 clean non-delta intra AU의 PTS부터 출력을 재개합니다. Auto의 Software
fallback은 기존 first-frame timeout/stall/bus error에만 유지합니다. 카메라 encoder, 저장 형식,
YOLO, 측정, 좌표 계약은 바꾸지 않습니다.

## Operator workflow before and after

운영 절차는 동일합니다. 변경 후에는 첫 RTP gap부터 clean intra recovery까지 화면 갱신을
차단하고 로그에 side, missing packet 수, recovery PTS와 차단 시간을 표시합니다.

## Success, delay, rejection, failure, overload, and recovery

RTP gap 자체는 decoder failure가 아니므로 Auto/NVIDIA/Software 모두 현재 decoder를 유지합니다.
Auto의 기존 health fallback과 명시적 NVIDIA fail-closed 정책은 그대로입니다. Software fallback
직후 camera가 `503`을 반환하면 그 전환에만 0.5/1/2초 bounded retry를 허용합니다. 손실 구간에는
clean intra가 올 때까지 freeze가 있을 수 있지만 macroblock/이전-frame 혼합을 새 frame처럼
표시하지 않습니다.

parsed AU에 PTS가 없거나 `DISCONT/CORRUPTED` flag가 있으면 동일한 손상 경계로 처리해 해당
AU와 dependent delta chain을 decoder와 MP4 양쪽에서 버리고 clean intra부터 재개합니다.

기록 버튼은 raw-video 부모를 먼저 만들고 MP4 tee가 있는 worker를 시작합니다. GStreamer
property path는 shell quoting이 아니라 double-quoted escaping을 사용합니다. 새 pipeline의
PLAYING 전환 실패에만 0.5/1/2/4초 bounded retry를 적용하고, 취소와 그 밖의 오류는 기존처럼
즉시 끝냅니다.

## Data and compatibility effects

settings/session artifact 이름과 H.264 copy는 변경하지 않습니다. MP4 내부는 이 WIRIS stream에서
잘못된 sample offset을 만든 1초 fragmented mode 대신 정상 EOS의 plain MP4로 finalization합니다.
camera encoder readback은 양쪽 원래 `bitrate/QP/GOP/IDR = 0/1/90/0`으로 유지합니다.

## Performance priorities and allowed degradation

정상 구간 NVIDIA throughput은 유지합니다. packet당 sequence 정수 비교와 parsed-AU flag/PTS
비교만 추가하며 process, thread, queue, GPU memory를 늘리지 않습니다. gap만으로 software로
내리지 않으므로 이번 실장비에서 관측된 CPU overload와 jitter-buffer 재손실을 피합니다.

## Visual and interaction changes, including dynamic-layout stability

layout, focus, scroll, preview 크기는 바꾸지 않습니다. 기존 로그에 RTP gap, clean-intra resume,
health fallback/retry 이유만 추가합니다.

## How acceptance will be demonstrated

sequence wrap/duplicate/old/SSRC 변경/forward gap, gap PTS 이전 intra 거부, delta/corrupt AU 거부,
clean-intra resume, missing-PTS/DISCONT/CORRUPTED AU drop, bounded 503/initial-PLAYING retry
집중 테스트를 실행합니다. 앱을 완전히 종료해 새 PID로
재실행한 뒤 사용자의 hand-motion 자극과 Left/Right 동시 YOLO 부하에서 15분 window-id/frame
identity 검증을 수행하고 저장 MP4를 ffprobe/software full decode합니다.

## Cost, risks, assumptions, rollback, and field limits

카메라 자체 RTP omission과 post-jitter drop의 bytes는 앱이 복구할 수 없습니다. 이 변경은 다음
independent intra까지 표시를 멈춰 reference chain을 재설정합니다. 추가 RTSP client는 카메라
frame rate를 1.5–2.1 Hz까지 낮출 수 있으므로 단일 앱 owner가 정상 운용 기준입니다. 실패 시
gap gate/recovery code만 제거하면 기존 health fallback 정책으로 rollback됩니다.

## Exact scope awaiting approval

승인 요청 범위는 `wiris_stereo_distance/yolo_stream.py`의 decoder-independent RTP gap gate,
parsed clean-intra PTS resume, 기존 health-only Auto fallback 및 fallback-transition 503 retry,
recording worker 교체 직후의 bounded initial-PLAYING retry,
GStreamer filesink path serialization과 strict-decodable MP4 mux,
`tests/test_yolo_stream.py`의 집중 tests, StereoWIRIS 운영 문서와 `UPDATE.md` 갱신입니다.
CameraPane/layout/settings schema/encoder/Workstation/common runtime은 제외합니다.

## Amendment awaiting approval — motion freeze

최신 motion run은 깨진 픽셀 대신 Left 최대 9.109초, Right 최대 24.425초 정지를 보여 기존
gate-only 결과를 operator acceptance에서 탈락시켰습니다. TCP late packet을 무제한 보존하는
시험도 약 108초 만에 3.2초 지연이 누적돼 채택하지 않았습니다.

다음 승인 요청은 카메라 bitrate를 `0`으로 유지하면서 QP만 일시적으로 `1 -> 5`로 바꾸는
단일 변수 A/B입니다. 시험 전 다른 RTSP app을 종료하고 양쪽 `0/1/90/0`을 readback합니다.
QP5 motion 비교가 끝나면 성공 여부와 무관하게 즉시 QP1로 원복하고 readback합니다. 영구 설정
변경, bitrate/GOP/IDR 변경, reboot는 이 승인에 포함하지 않습니다.

## Workstation-only source/record unification awaiting approval

Workstation 기록은 현재 preview와 별도의 FFmpeg RTSP session을 엽니다. 제안 변경은 camera당
한 GStreamer `rtspsrc`만 열고 parsed H.264를 tee해 preview/YOLO와 저장이 같은 source를 공유하게
합니다. 별도 FFmpeg RTSP recorder는 제거합니다.

RTP/AU integrity gate는 tee 앞에 두고 손상 reference chain을 GUI와 운영 저장물에서 함께
차단합니다. 일반 network failure의 manual reconnect 정책, UI layout, measurement schema,
camera encoder 설정은 유지합니다. 변경 후보는 Workstation의 `workstation_core/yolo_stream.py`,
camera/recording focused tests, `WIRIS.md`이며 실제 성공 전에는 `WIRIS.md`를 완료 문서로 바꾸지
않습니다.

### Approved backend behavior

- GStreamer: single `rtspsrc`, pre-tee integrity gate, raw H.264 copy to one MP4.
- OpenCV/FFmpeg and PyAV: selected backend remains active and decoded frames are re-encoded to one
  MP4 with `decoded_reencode_unverified` provenance.
- Integrity recovery waits for clean intra in the same session; after 30 seconds it finalizes the
  current file and requires manual reconnect.

## Approved amendment — remove GUI from native evidence and measurement flow

The GUI will no longer receive every native raster as a queued QImage. Camera children retain
native decoded rasters in a bounded preallocated shared-memory ring. Detection messages contain
only an exact camera/generation/frame identifier and lightweight metadata. A pair coordinator pins
the exact left/right slots only after accepting a measurement pair; unavailable or overwritten
slots reject the measurement instead of substituting a nearby timestamp.

Measurement computation and native evidence persistence run outside the GUI. Evidence becomes
SAVED/READY only after atomic write, fsync, verification, SHA-256, and ledger ACK. Preview remains a
scaled size-one latest value. Stop/fatal error use an independent control Event/pipe and cannot be
blocked by preview, detection, native, or writer queues. The temporary BlockingQueuedConnection
experiment is explicitly not the final design.
