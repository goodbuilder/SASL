# Change Brief

## Operator outcome

Workstation Camera Preview가 NVIDIA NVDEC를 실제 사용 가능할 때 자동으로 `nvh264dec`를
선택하고, operator가 Left/Right 각각의 요청 mode, 실제 decoder, GPU/CPU 여부, fallback
이유를 화면에서 즉시 확인한다.

## Motivation

2026-08-26 launcher doctor가 한 시점에는 driver 접근 실패로 software fallback을 보고했지만,
현재 RTX 4050/driver 595.84와 `nvh264dec`는 정상이다. 저장 설정
`rtsp_decoder=software`가 현재 `avdec_h264` 선택의 직접 원인이다. UI는 Camera workspace에
raw decoder 이름을 이미 표시하지만 global status/detail과 GPU availability/fallback 이유는
분명하지 않다.

## Included scope

- 저장 decoder mode를 `auto`로 전환해 NVDEC 우선 정책을 사용한다.
- 기존 `active_stream_decoders` owner를 유지하고 per-side 실제 decoder를 읽기 쉽게 표시한다.
- Camera status detail/tooltip에 requested/effective decoder와 fallback을 투영한다.
- Diagnostics Performance에 GPU model/driver, NVDEC availability, requested/effective mode를
  read-only로 표시한다.
- Auto의 기존 one-way runtime fallback과 recording-safe restart를 보존하고 fallback을 WARN으로
  보이게 한다.

## Excluded scope and non-goals

- strict NVIDIA-only를 기본값으로 강제하지 않는다.
- CUDA inference 배치/worker 수, GPU affinity, power/clock limit, RTSP rate/quality를 바꾸지 않는다.
- 새 top-level dashboard/status badge를 추가하지 않는다.
- NVIDIA driver/package를 다시 설치하거나 firmware/BIOS를 변경하지 않는다.
- StereoWIRIS source는 변경하지 않는다.

## Acceptance summary

Auto가 현재 target hardware에서 `nvh264dec`를 선택하고 실제 H.264 decode가 성공한다. UI는
GPU active, explicit software, Auto fallback, unavailable를 구분하며 geometry/focus를 바꾸지
않는다. GPU가 불가해도 Preview는 기존 계약대로 `avdec_h264`로 재시작하고 이유를 남긴다.
