# Integration Flow

## Action-to-postcondition traces

`측정/연결 action` → CameraPane → YoloStreamWorker → child process → rtspsrc/jitter/depay/
h264parse → encoded evidence tee → decoder → videoconvert BGR → appsink deep copy → bounded
frame queue → QImage deep copy → GUI-thread QLabel.

## Transition tables

| observation | decoder A/B | queue/QImage identity | classification |
|---|---|---|---|
| encoded 정상 | NVIDIA만 artifact | identity 유지 | NVIDIA live decode |
| encoded AU 손실 | 둘 다 artifact | identity 유지 | RTP/depay/pre-decode |
| decoded 정상 | 둘 다 정상 | identity 불일치 | frame transport/Qt |
| 모든 경계 정상 | GUI screenshot만 artifact | display/compositor |

## Failure and recovery flows

disconnect는 기존 reconnect/stop owner로 복구한다. encoder SET에 따른 계획된 RTSP pause는
원래 장애와 별도 event로 기록한다. fallback은 requested/active decoder와 이유를 보존한다.

`RTP forward sequence gap` -> tracker discontinuity record -> 모든 decoder output suppression
-> `h264parse` clean non-delta/non-corrupt AU + PTS boundary -> decoded sample PTS가 boundary에
도달하면 같은 pipeline 재개. gap 이후 corrupt QImage는 생성하지 않는다. decoder health
failure만 기존 Auto software fallback/explicit NVIDIA fail-closed 결정을 사용한다.

`parsed AU missing PTS/DISCONT/CORRUPTED` -> primary parser probe DROP -> encoded tee/decoder gate
-> dependent delta DROP -> clean intra PTS부터 encoded branch 재개 -> 동일 PTS decoded sample에서
GUI 재개. qtmux가 PTS 없는 buffer를 받아 recording 전체를 종료하지 않게 한다.

`세션 기록 시작` -> fresh session/video parent -> 기존 camera worker 정상 종료 -> MP4 tee가 있는
새 worker -> correctly serialized filesink -> pipeline PLAYING. PLAYING만 실패하면 동일
configuration으로 bounded 0.5/1/2/4초 재시도하며, 성공한 새 worker PID와 MP4 증가를
postcondition으로 삼는다. 종료는 plain MP4 EOS -> ffprobe/strict decode 순서다.

## Cross-feature contracts and dependency direction

camera path는 Pixhawk/GNSS/measurement internals를 직접 참조하지 않는다. root common runtime
정책 변경은 별도 canonical packet 소유이므로 이 packet에서 임의 수정하지 않는다.
