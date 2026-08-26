# Current Behavior

## Version and worktree context

- Baseline commit: `c4397995d864576c966702cfd3c398b6032f8990`.
- Worktree는 ETH MAVLink와 기타 사용자 변경으로 매우 dirty하다. Camera owner 파일의 기존
  diff를 보존하며 reset/broad formatting을 금지한다.
- 현재 root canonical venv와 `run.sh --check`는 정상이다.

## Reproduction

1. `.local/workstation_data/stereo_distance_settings.json`의 `rtsp_decoder`가 `software`다.
2. `./run.sh --check`는 NVIDIA element/driver를 PASS하지만 `decoder.selection`을
   `software -> avdec_h264`로 보고한다.
3. 같은 runtime에서 `current_decoder_selection('auto')`와 `('nvidia')`는 모두
   `nvh264dec`를 반환한다.

## Observed frontend behavior

- Settings에는 Auto/NVIDIA/Software selector가 있다. 변경 시 runtime override는 clear된다.
- Camera workspace Status group에 `Decoder LEFT/RIGHT`가 있지만 raw element name만 표시한다.
- compact CAM L/R status와 tooltip은 frame freshness를 표시하지만 decoder/fallback은 없다.
- Diagnostics Performance는 GPU/decoder availability를 현재 표시하지 않는다.

## Observed backend and persistence behavior

- `camera_stream_controller._apply_decoder_start_policy()`가 current driver/element를 probe한다.
- Auto는 `nvh264dec, avdec_h264` 후보 순서를 사용한다. first-frame timeout 또는 decoded-frame
  stall이면 현재 MP4를 종료하고 software로 한 번 재시작한다.
- explicit NVIDIA는 software fallback 없이 fail-closed다.
- `active_stream_decoders`가 worker의 `GStreamer 디코더:` 상태에서 per-side 실제 decoder를
  보존한다.
- RTX 4050 Laptop GPU, driver 595.84, CUDA 13.2, GStreamer 1.20.3 nvcodec plugin을 관측했다.
- 1920x1080@30 H.264 150 frames가 `nvh264dec` CUDA memory path로 EOS까지 통과했다.
  반복 stress에서 `nvidia-smi dmon` decoder engine 78–100%를 관측했다.

## Documentation, code, test, and runtime conflicts

- README는 Auto를 기본으로 설명하지만 persisted runtime setting은 Software다.
- raw `nvh264dec`/`avdec_h264` 문자열만으로는 operator가 GPU active인지 선택/ fallback인지
  구분하기 어렵다.
- 초기 NVIDIA failure는 이전 sandbox/driver-access 관측이며 현재 host kernel log에는 NVRM/Xid
  오류가 없고 real host probe/decode는 PASS다.
