# Impact Analysis

## Preserved invariants

카메라 endpoint, encoder 원래값, RTSP/worker/resource owner, 측정·좌표·session schema,
YOLO 입력, GUI layout, user dirty changes를 보존한다.

## Affected layers and components

조사 대상은 `yolo_stream.py`, `camera_pane.py`, `legacy_gui.py`, 관련 tests와 운영 문서다.
실제 변경 파일은 원인 확정 후 최소화한다.

제안 변경은 `wiris_stereo_distance/yolo_stream.py`의 기존 RTP timing probe와 GStreamer
reader/main loop, `tests/test_yolo_stream.py`, decoder 운영 문서에 한정한다. UI layout이나
CameraPane buffer ownership은 변경하지 않는다.

## Feature dependency, ownership, and coupling changes

새 owner나 cross-feature dependency를 만들지 않는다. side별 worker가 RTSP/pipeline owner,
CameraPane이 latest preview owner인 현재 방향을 유지한다.

## Worker CPU/GPU resource impact

baseline은 2 camera process, Left 36/Right 22 threads, GPU 7%, NVDEC 3%, 497 MiB다.
계측은 bounded하고 preview/YOLO throughput을 의미 있게 낮추지 않아야 한다. worker count,
decoder policy, queue budget 변경은 원인 근거와 별도 승인이 필요하다.

제안은 process/thread/queue 크기를 늘리지 않는다. RTP sequence 비교는 packet당 정수 연산과
side별 작은 state뿐이다. gap은 같은 decoder의 intra recovery로 처리해 software 전환에 따른
CPU degradation과 post-jitter drop feedback을 만들지 않는다.

## Compatibility and migration

settings/session schema migration은 예정하지 않는다. root canonical runtime packet의
`Auto만 health-based software fallback`, `explicit NVIDIA fail-closed` 의미를 그대로 보존한다.
RTP gap은 decoder failure가 아니라 same-session corruption recovery event로 분리한다.

## Rollback and recovery

코드 변경은 작은 slice로 유지하고 기존 setting/readback을 기록한다. encoder A/B는 각
변수 시험 직후 원복 가능해야 한다. 데이터 삭제는 없다.

## Risks and assumptions

artifact가 간헐적이며 screenshot만으로 자동 판별하기 어렵다. 동일 PTS/representation
hash와 deterministic replay가 필요하다.

RTP sequence restart/SSRC 변경/16-bit wrap을 loss로 오판할 위험이 있다. side별 SSRC state,
정상 wrap, duplicate/old packet 무시, forward gap만 검출하는 집중 테스트가 필요하다.
