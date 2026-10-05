# StereoWIRIS WIRIS OZ Visible Preview 장애 원인 및 해결안

작성일: 2026-08-26
대상: StereoWIRIS, Left `10.0.0.231`, Right `10.0.0.232`, WIRIS Enterprise OZ visible RTSP

## 1. 결론

GUI tearing, macroblock corruption, 이전-frame 잔상의 주원인은 Qt나 USB 랜카드가 아니다.
WIRIS RTSP/RTP sender가 H.264 access unit을 구성하는 RTP packet 일부를 전송하지 않는 것이
수신 PC의 완전한 TCP byte stream에서 확인됐다.

앱은 누락된 reference pixel을 복원할 수 없다. 현재 camera stream은 `GEIR=0`이고 loss/PLI 뒤에도
실제 IDR NAL type 5를 제공하지 않기 때문에 decoder reference state를 확실히 재설정할 recovery
point도 없다. 앱의 clean-intra gate는 손상 frame 대신 last-clean frame을 유지할 수 있지만,
그 결과 수초 단위 freeze가 반복된다. 따라서 현재 firmware/device 상태에서는 다음 두 조건을
동시에 만족시키지 못한다.

- corruption / tearing / previous-frame residue = 0
- 장시간 실시간 preview freeze = 0

근본 해결에는 WIRIS sender packet omission 수정과 loss 뒤 실제 IDR 제공이 필요하다.

## 2. 장애별 상태

| 장애 | 가장 좁은 원인 계층 | 상태 |
|---|---|---|
| 기록 시작 후 `YOLO 시작 중...` 고착 | GStreamer filesink path serialization 및 `video/` parent 누락 | 앱 수정 완료 |
| MP4에서 `mdat`를 NAL length로 해석 | `mp4mux fragment-duration=1000` sample offset | 앱 수정 완료 |
| 장기 MP4 `Buffer has no PTS` | parsed H.264 AU timestamp 누락 | AU 차단 완화 적용, 장기 재검증 필요 |
| GUI macroblock/잔상 | WIRIS sender RTP/H.264 reference packet omission | 미해결, device/vendor 조치 필요 |
| corruption 차단 후 수초 freeze | real IDR 없이 clean intra를 기다리는 fail-closed gate | 미해결 responsiveness 문제 |
| PC UTC offset 증가 | one-shot STUS 후 camera/PC clock rate mismatch | 원인 확정, 지속 discipline 필요 |
| 거리 그래프 미표시 | 복수 Left 후보, active intrinsics 미적용, GPS/RTK baseline 대기 | preview 장애와 별도 운용 문제 |

## 3. 핵심 원인 증거

### 3.1 Exact TCP/RTP 경계

단일 Right RTSP-over-TCP session을 같은 시각에 pcap, encoded MKV, NVIDIA decoder,
software decoder로 분기했다.

- 관측 시간: 90.09초
- 수신 TCP bytes: 477,692,445
- TCP byte gap: 0
- TCP retransmitted bytes: 550,395 — 최종 byte stream에는 모두 복구됨
- kernel/tcpdump drop: 0
- main RTP packets: 341,823
- RTP forward gap: 4건
- camera가 건너뛴 RTP sequence packet: 137개

대표 사건:

- 17:00:06.444 KST: 69개 누락
- 17:00:52.388 KST: 동일 H.264 slice timestamp 안에서 16개 누락
- 17:01:08.395 KST: 동일 slice에서 44개 누락
- 17:01:08.397 KST: 같은 slice에서 추가 8개 누락

TCP stream이 완전한데 그 payload의 RTP sequence가 이미 비어 있으므로 다음 계층은 packet을
삭제한 원인이 아니다.

- ipTIME U1003C / RTL8153
- Linux NIC/kernel capture
- TCP 재전송/재조립
- GStreamer jitter/depay
- NVIDIA/software decoder
- process queue/QImage/Qt

증거 문서: [exact-right-tcp-motion-pcap-20260826.md](evidence/exact-right-tcp-motion-pcap-20260826.md)

### 3.2 Decoder/Qt 경계

- 같은 encoded bitstream deterministic replay에서 NVIDIA와 software가 서로 다른 conceal/drop
  결과를 보였지만, 손상 원인은 decoder 입력 이전에 존재했다.
- producer RGB → multiprocessing queue → deep-copied QImage hash mismatch는 0이었다.
- queue reorder는 0이었다.
- motion corruption이 저장 bitstream의 software full decode에서도 재현된 실행이 있었다.

따라서 Qt stride, QImage buffer lifetime, shared memory overwrite, GUI thread가 원인이 아니다.

### 3.3 NIC/transport 경계

- artifact/acceptance 구간 NIC RX/TX error/drop/missed counter는 0이었다.
- USB adapter는 SuperSpeed와 gigabit link로 동작했다.
- UDP 320초 pcap도 kernel drop 0이었지만 다음 RTP loss가 있었다.
  - Left: gap 115건, missing 241 (`0.010850%`)
  - Right: gap 306건, missing 339 (`0.016527%`)
- UDP에서는 clean-intra gate 때문에 GUI receive가 2.2 Hz까지 내려갔다.

즉 TCP/UDP 전환은 loss pattern만 바꾸며 해결하지 못한다.

증거 문서: [encoder-and-transport-ab-20260826.md](evidence/encoder-and-transport-ab-20260826.md)

## 4. 장기 동기화 원인

STUS는 시각 phase를 한 번 맞추지만 camera oscillator frequency를 지속적으로 discipline하지 않는다.
그 뒤 WIRIS RTCP NTP clock과 NTP 보정 중인 PC clock이 서로 다른 속도로 free-run한다.

고유 RTCP Sender Report 회귀:

- Left SR-PC offset slope: `-114.410 ppm`, R² `0.8446`
- Right SR-PC offset slope: `-99.250 ppm`, R² `0.8067`
- camera-to-camera Right−Left relative drift: `+16.521 ppm`
- Left absolute offset 100 ms 초과: session 약 18.2분
- Right absolute offset 100 ms 초과: session 약 20.2분

PC는 `systemd-timesyncd`로 frequency `+16.587 ppm` 보정 중이었다. PC slew가 공통 offset에 일부
기여하지만 camera끼리도 16.5 ppm 상대 drift가 있으므로 PC만의 문제는 아니다. transport-delay
회귀의 선형성은 매우 낮아 누적 network queue가 offset 증가의 주원인이라는 가설도 배제됐다.

15분 live sampler 결과:

- 90 samples
- 50 ms 초과: 22 samples
- 최대 latest-frame RTCP/NTP delta: 27.3895초
- 상태: `verified`, `not-verified`, `missing` 모두 발생
- stream restart에서 SR count reset 1회

증거 문서: [long-run-sync-quality-20260826.md](evidence/long-run-sync-quality-20260826.md)

## 5. 앱에서 수정 완료한 항목

### 5.1 Recording start

- 새 session 시작 시 raw-video `video/` parent를 먼저 생성한다.
- shell용 `shlex.quote()`를 GStreamer property에 사용하지 않는다.
- 한글/공백/따옴표 경로를 GStreamer double-quoted syntax로 round-trip한다.
- initial PLAYING failure는 해당 실패에만 0.5/1/2/4초 bounded retry한다.

### 5.2 MP4

- 잘못된 one-second fragmented MP4 mode를 제거했다.
- direct H.264 copy와 orderly EOS를 유지한다.
- 실제 short app MP4는 ffprobe와 strict software full decode를 통과했다.
- PTS 없는 AU, `DISCONT`, `CORRUPTED` AU를 recorder/decoder 전에 차단하는 gate를 추가했다.

### 5.3 Preview evidence/gate

- RTP sequence discontinuity를 SSRC별 계측한다.
- 손상 의존 AU를 clean non-delta intra까지 encoded/decoded branch에서 차단한다.
- recovery AU에 decoder discontinuity를 표시해 reference reset을 시도한다.
- producer/queue/QImage identity JSONL을 선택적으로 기록한다.

이 항목들은 장애를 완화하고 원인을 관측 가능하게 만들지만, camera가 보내지 않은 reference
payload를 복원하는 근본 해결은 아니다.

### 5.4 Startup session/intrinsics

- 앱 시작 시 미완료 spool을 자동 복구하지 않는다.
- 앱 시작 시 intrinsics를 active calculation에 자동 적용하지 않는다.
- 기존 spool과 intrinsics 파일은 보존한다.
- 새 session은 `세션 기록 시작` 버튼 시점에만 생성한다.

## 6. 배제되거나 실패한 해결안

| 시험 | 결과 | 판정 |
|---|---|---|
| 강제 Software decoder | CPU overload, 0~1.4 Hz 구간 및 concealment | 기본 해결책 아님 |
| NVIDIA `discard-corrupted-frames` | deterministic output 차이 없음 | 효과 없음 |
| `wait-for-keyframe/request-keyframe` | IDR0/IDR1 모두 frame 0 | camera가 real IDR 미제공 |
| `SEIR=1` | NAL type 5가 계속 0 | 원복 |
| Right QP 1→10 | bitrate 약 16% 감소, 문제 유지 | 원복 |
| Right SEBR 20 Mbps | RTSP timeout, 0-byte recording, QP/GOP/GETH side effect | 원복 |
| latency 100→500 ms | 사용자 관찰상 문제 유지 | 100 ms 원복 |
| TCP `drop-on-latency=false` | 약 108초에 지연 약 3.2초 누적 | low-latency 실패, 원복 |
| UDP | 더 잦은 gap, receive 2.2 Hz | TCP 원복 |
| clean-intra gate만 사용 | corruption 대신 수초 freeze | responsiveness 실패 |
| decoder DISCONT reset | 일부 motion 구간 개선, 장기 artifact/freeze 0 미달 | 불충분 |

## 7. 해결안

### 7.1 필수 근본 해결 — camera/vendor

우선순위 1이다. 다음 조건을 WIRIS firmware/RTSP server가 제공해야 한다.

1. RTSP-over-TCP에서 sender가 RTP sequence를 건너뛰지 않아야 한다.
2. UDP에서도 socket/kernel drop 0 조건의 sequence continuity를 보장해야 한다.
3. loss, RTCP PLI/FIR 또는 새 client start 뒤 실제 H.264 IDR NAL type 5를 보내야 한다.
4. recovery IDR에 SPS/PPS를 함께 제공해야 한다.
5. encoder SET 후 RTSP/GETH가 임의로 꺼지거나 QP/GOP가 부수적으로 바뀌지 않아야 한다.
6. RTCP NTP clock을 NTP/PTP/GNSS 등으로 지속 discipline하거나 drift 사양을 공개해야 한다.

이 조건 없이 앱만으로 missing reference pixel과 장기 clock drift를 없앨 수 없다.

### 7.2 앱의 안전 완화안

근본 해결 전 사용할 수 있으나 trade-off를 숨기면 안 된다.

1. 손실 시 last-clean frame을 유지하고 화면에 `LOSS RECOVERY / FROZEN`을 명시한다.
2. live sync metric이 50/100 ms threshold를 넘으면 latched `verified`를 즉시 downgrade한다.
3. mux/filesink 오류를 decoder health failure와 분리해 불필요한 software fallback을 막는다.
4. recording을 loss-safe segment로 나누고 각 segment를 true recovery point에서 시작한다.
5. 필요하면 clean decoded frame을 재인코딩한 review MP4를 raw evidence와 별도 artifact로 만든다.
6. image-level artifact detector는 보조 oracle로만 사용하고 정상 motion을 오탐하지 않는 eval을
   통과하기 전 자동 drop 근거로 사용하지 않는다.

이 완화안은 corruption 노출을 줄일 수 있지만 freeze/latency/storage cost가 생기며 sender
omission 자체를 해결하지 않는다.

### 7.3 동기화 운용안

근본적으로는 camera continuous discipline이 필요하다. 그 전 임시 운용은 다음과 같다.

1. 측정 직전에만 수동 STUS를 실행한다.
2. STUS가 RTSP를 재시작하므로 recording은 sync/reconnect/RTCP warmup 완료 뒤 시작한다.
3. recording 중 자동/주기 STUS는 하지 않는다.
4. live RTCP pair delta, SR-PC offset, SR age를 계속 계측하고 threshold 초과 시 측정을 막는다.
5. PC NTP source도 low-delay local NTP/PTP로 개선한다. 단, 이것만으로 camera relative drift는
   없어지지 않는다.

### 7.4 거리 그래프 운용안

preview 문제가 해결돼도 다음 조건이 없으면 거리 그래프는 그려지지 않는다.

1. 양쪽에서 같은 target class를 유일하게 검출해야 한다.
2. `ambiguous_left_candidates` 등 correspondence rejection이 없어야 한다.
3. 현재 zoom의 intrinsics를 수동 불러오기 후 `계산에 적용`해야 한다.
4. `Pixhawk 자세 + GPS/RTK baseline` mode라면 live attitude와 non-zero GPS/RTK baseline이
   필요하다.
5. 그래프는 `measurement_valid=true`인 3D point만 그린다.

## 8. 현재 복원 상태

- Left `GETH=TRUE`
- Right `GETH=TRUE`
- Left encoder `GEBR/GEQZ/GEGS/GEIR = 0/1/90/0`
- Right encoder `GEBR/GEQZ/GEGS/GEIR = 0/1/90/0`
- StereoWIRIS persisted transport/latency: `TCP / 100 ms`
- camera reboot, firmware, IP/subnet, factory reset 없음

## 9. Vendor fix 이후 합격 기준

다음 전부를 새 process와 실제 장비에서 통과해야 해결 완료다.

1. 단일/양쪽 TCP pcap 30분: TCP byte gap 0, RTP sequence missing 0.
2. UDP를 지원 대상으로 삼는다면 UDP pcap도 missing 0.
3. start/PLI/loss 뒤 실제 IDR NAL type 5와 SPS/PPS 확인.
4. Left/Right 동시 motion preview 15분 이상.
5. tearing, macroblock, previous-frame residue 0.
6. 허용 정책 밖 freeze/reorder/stale output 0.
7. GStreamer ERROR와 unexpected decoder fallback 0.
8. producer/queue/QImage identity mismatch 0.
9. recording start/stop 성공, 양쪽 MP4 ffprobe 및 strict software full decode 통과.
10. STUS 후 장기 sync: pair Δt 50 ms 이내, SR-PC offset 100 ms 이내, SR count regression 0.

## 10. 관련 증거

- [Recording-start filesink root cause](evidence/recording-start-filesink-root-cause-20260826.md)
- [MP4 mux A/B](evidence/mp4-mux-ab-20260826.md)
- [Exact Right TCP pcap](evidence/exact-right-tcp-motion-pcap-20260826.md)
- [Encoder/transport A/B](evidence/encoder-and-transport-ab-20260826.md)
- [Long-run synchronization](evidence/long-run-sync-quality-20260826.md)
- [Post-sync 15-minute failure](evidence/post-sync-15min-failure-20260826.md)
- [Final restored settings](evidence/final-restored-settings-20260826T1742KST.txt)

## 11. 현재 판정

앱 내부 recording/startup 결함은 수정됐지만 WIRIS sender omission 때문에 최종 목표는 미완료다.
금지 범위인 firmware 변경 없이 더 진행하려면 vendor가 수정 firmware 또는 real-IDR recovery
방법을 제공해야 한다. 그 전에는 정상 screenshot 한 장이나 짧은 미재현을 해결 완료 증거로
사용하지 않는다.
