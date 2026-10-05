# Current Behavior

## Version and worktree context

- Baseline commit: `7242452`
- Baseline date: 2026-08-29
- Worktree: 다수의 기존 사용자/다른 패킷 수정이 있음. 특히 `README.md`,
  `workspaces.py`, `standalone_gui.py`, `tests/test_workstation_calibration.py`,
  `workstation_core/yolo_stream.py`, `workstation_core/camera_pane.py`,
  `workstation_core/native_frame_ring.py`는 dirty이므로 관련 hunk만 최소 패치한다.
- `calibration_controller.py`, `workstation_core/calibration_timing.py`,
  `workstation_core/calibration_input.py`, `tests/test_calibration_input_snapshot.py`는 패킷 생성 시점에
  관련 uncommitted diff가 없다.

## Reproduction

1. Live/Auto checkerboard capture에 Left/Right native frame metadata를 전달한다.
2. `_checkerboard_capture_pair_timing_evidence()`를 호출한다.
3. 현재는 `rtcp_frame_ntp_unix_s` 두 값의 signed Right-minus-Left 차이를 임계와 비교한다.
4. 한쪽 RTCP/NTP가 없으면 app-receive metadata가 있어도 stereo pair를 거부한다.

Baseline focused command:

```bash
/home/moontaehyung/바탕화면/SASL-20260821/.venv/bin/python -m pytest -q \
  tests/test_workstation_calibration.py \
  -k 'checkerboard_pair_timing or checkerboard_capture_worker or auto_capture' \
  tests/test_calibration_input_snapshot.py tests/test_native_calibration_frame.py
```

Baseline result: `9 passed, 50 deselected`.

## Observed frontend behavior

- Calibration의 `Max Live pair Δt` tooltip/설명은 Live를 `RTCP/NTP frame time`, FTP를
  `CPTR dispatch proxy`로 표시한다.
- Live/Auto capture 상태는 timing 임계 초과 시 intrinsics-only를 표시한다.
- Compute 사진 시각 표시는 별도의 wall-clock provenance로 RTCP/NTP를 사용하며,
  pair gate 기준을 선택하는 제어가 아니다.
- 신규 control을 추가할 필요가 없고 문구 길이 변화만 예상된다.

## Observed backend and persistence behavior

- `app_receive_monotonic_sample()`은 decoded GStreamer appsink sample이 카메라 프로세스에
  도착한 순간 `time.monotonic_ns()`를 추출한다.
- Native frame metadata는 exact `app_receive_monotonic_ns`와 compatibility `timestamp_s`를 보존한다.
  Native ring descriptor의 `timestamp_s`도 producer app-receive monotonic이다.
- Calibration timing 우선순위는 검증된 exposure timestamp, FTP capture-ID/dispatch,
  Live RTCP/NTP이다. Live app-receive는 freshness/native-frame lookup에만 사용된다.
- capture metadata에는 `pair_time_basis`, `pair_delta_s`, timing evidence, RTCP/NTP 값,
  timing allow/reject decision이 저장된다.
- 거부된 두 사진은 checkerboard가 검출되면 K,D용으로 보존되고
  `stereo_pair_saved=false`로 R,t에서 제외된다.

## Documentation, code, test, and runtime conflicts

- 현재 code/test/docs는 Calibration Live=RTCP/NTP로 일치한다. 이는 버그 수정이
  아니라 사용자가 요청한 제품 계약 변경이다.
- Measurement Live는 이미 app-receive monotonic을 authoritative pairing clock으로 사용하며
  NTP를 record-only로 보존한다. Calibration과 Measurement의 현재 차이는 의도적이다.
- 현재 샘플 Photo set에는 오래된 FTP capture가 RTCP/NTP로 표시된 legacy metadata가
  있지만 신규 현재 FTP 코드는 CPTR dispatch proxy를 사용한다. 기존 파일은
  변경하지 않는다.
