# Backend Contract

## Commands and validation

WIRIS readback은 `GEBR/GEQZ/GEGS/GEIR VISIBLE`를 side별 독립 조회한다. encoder A/B가
필요하면 한 변수만 SET하고 네 값 전체를 전후 readback한다.

## State, ownership, and concurrency

각 camera child가 한 RTSP/pipeline owner다. 완전 decoded frame만 process queue에 들어가며
camera_id/sequence/PTS ordering을 보존한다.

기존 `GstRtpTimingTracker`가 SSRC별 마지막 RTP sequence와 누적 gap/missing packet을
소유한다. probe thread는 bounded immutable discontinuity record만 갱신하고 worker main/read
loop가 이를 읽는다.

## Worker resource budgets, scheduling, and backpressure

compressed H.264를 임의 drop하지 않는다. overload 시 decoded complete frame만 명시된
latest-only 정책으로 coalesce하며 stale/reorder를 출력하지 않는다. 계측 queue는 bounded다.

RTP gap을 관측한 순간 decoder 종류와 무관하게 decoded sample은 process frame queue에 넣지
않는다. `h264parse`의 gap PTS 이후 clean non-delta/non-corrupt AU를 recovery candidate로 삼고,
decoded sample PTS가 그 boundary에 도달할 때만 같은 pipeline 출력을 다시 연다.

primary parser output의 PTS 누락, `DISCONT`, `CORRUPTED`는 RTP sequence gap이 없어도 compressed
stream discontinuity다. 해당 AU부터 encoded tee와 decoder를 함께 닫고 dependent delta AU를
drop한다. valid PTS clean non-delta intra만 recorder/decoder를 다시 열고, decoded appsink가 같은
PTS에 도달할 때 GUI gate를 연다.

## External boundaries and workers

RTP/jitter/depay/NAL-AU → encoded evidence/NVIDIA/software → BGR/stride → process frame queue
→ QImage/QLabel을 각각 독립 oracle로 검증한다.

## Results, errors, timeout, retry, and cancellation

visual artifact만으로 성공을 주장하지 않는다. packet/AU discontinuity, decoder corrupt flag,
queue drop/reorder, QImage identity를 구분해 기록한다. reconnect와 fallback은 이유가 보여야 한다.

- 16-bit wrap(`65535 -> 0`)은 정상 연속이다.
- duplicate/old/reordered packet은 forward loss로 누적하지 않는다.
- SSRC 변경은 sequence baseline reset이다.
- forward delta `2..32767`만 missing packet으로 기록한다.
- RTP gap은 decoder failure가 아니므로 Auto/NVIDIA/Software 모두 same-decoder recovery다.
- Auto first-frame/stall/bus health failure만 기존 Software fallback을 사용한다.
- fallback 전환 직후의 RTSP 503만 0.5/1/2초 bounded retry하고 그 밖의 오류는 확대하지 않는다.
- 새 pipeline 자체가 `failed to enter PLAYING state`로 끝날 때만 0.5/1/2/4초 bounded retry한다.
  stop event가 set되면 대기를 즉시 취소하고 auth/pipeline 구성 오류는 재시도하지 않는다.
- raw-video path parent는 worker 시작 전에 생성한다. `Gst.parse_launch()` path property는
  double-quoted/escaped 값으로 round-trip해야 하고 literal quote가 파일명에 들어가면 안 된다.
- 저장은 direct H.264 copy와 plain `mp4mux`를 사용한다. orderly EOS 후 ffprobe와 strict software
  full decode를 통과해야 하며, power loss로 EOS가 없으면 실패 spool/manifest를 보존한다.

## Persistence, restart, and shutdown

진단 evidence만 추가할 수 있으며 session format은 바꾸지 않는다. 수정 후 기존 process를
재사용하지 않고 정상 shutdown 후 새 process로 시작한다.
