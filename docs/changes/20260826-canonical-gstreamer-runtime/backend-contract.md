# Backend Contract

## Commands and validation

공식 prefix는 repository root `.venv`; mismatch는 exit 2다. `--execution`/앱 `--check`는
필수 Python/GStreamer/app assets를 읽기 전용 검사한다. `--deployment`는 dpkg manifest와
공식 venv 재생성 가능성을 검사한다.

## State, ownership, and concurrency

공통 checker는 immutable report를 반환한다. camera worker는 기존 process/queue/recording
owner를 유지하며 child executable은 `sys.executable`로 고정한다.

## Worker resource budgets, scheduling, and backpressure

변경 없음. 두 camera process, bounded event/frame queues, decode/preview/inference rates를
그대로 유지한다.

## External boundaries and workers

Gst/GstRtp typelib, `rtspsrc rtph264depay h264parse queue videoconvert appsink avdec_h264
mp4mux filesink`의 find/make를 검사한다. `nvh264dec`와 NVIDIA driver는 optional capability다.

## Results, errors, timeout, retry, and cancellation

exit 0=ready/허용된 Auto software, 1=필수 dependency, 2=interpreter/usage. NVIDIA는
software fallback 금지; Auto만 startup/runtime fallback 허용. 기존 RTSP reconnect는 유지한다.

## Persistence, restart, and shutdown

설정 schema/recording format은 변경하지 않는다. Auto runtime fallback은 기존 MP4 EOS/stop
경계를 통과한 후 software restart한다.
