# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

Workstation의 저장 decoder mode를 Software에서 Auto로 바꿔, 현재 정상인 NVIDIA NVDEC를
우선 사용합니다. 기존 RTSP TCP, Left/Right worker, recording path, fallback ordering, Preview,
Measurement 흐름은 그대로입니다.

## Operator workflow before and after

현재는 Software가 고정돼 `avdec_h264`를 사용합니다. 변경 후 Preview를 시작하면 Auto가
`nvh264dec`를 먼저 사용하고 Camera 화면에서 각 side의 실제 GPU/CPU decoder를 확인합니다.

## Success, delay, rejection, failure, overload, and recovery

- 성공: 현재 stream generation의 실제 first frame이 `nvh264dec`에서 나오고 나서만
  `GPU · nvh264dec`가 표시됩니다. `PLAYING`은 candidate입니다.
- GPU 실패: decoder-health로 분류된 경우에만 Auto가 해당 side에 software override와
  이유를 남깁니다. Preview-only는 해당 side만 재시작하고, 녹화 중이면 해당 MP4를
  종료한 뒤 다음 side start에 적용합니다. 정상인 반대쪽 worker/MP4는 유지합니다.
- RTSP/source/parser와 mux/filesink 오류는 decoder fallback으로 위장하지 않습니다.
- strict NVIDIA를 operator가 직접 선택하면 fallback 없이 ERROR입니다.
- 새 worker/queue/rate/quality 변경은 없습니다.

## Data and compatibility effects

Measurement/recording/calibration data schema와 파일은 변하지 않습니다. 기존 세 mode 모두
호환되고 Software 선택으로 즉시 rollback 가능합니다.

## Performance priorities and allowed degradation

GPU 사용 가능 시 H.264 decode를 NVDEC로 옮겨 CPU 여유를 확보합니다. Driver reset/suspend
문제 때 Preview 가용성을 위해 CPU fallback을 허용하는 안을 권장합니다. GPU utilization은
진단일 뿐 readiness/success metric은 아닙니다.

## Visual and interaction changes, including dynamic-layout stability

새 화면/카드/badge 없이 기존 Camera `Decoder LEFT/RIGHT`에 `GPU`, `CPU selected`,
`CPU fallback`을 표시합니다. CAM L/R tooltip에 actual decoder를 추가하고 Diagnostics
Performance에 driver/availability 상세를 둡니다. 상태 변화가 layout/focus/scroll을 바꾸지
않습니다.

## How acceptance will be demonstrated

Driver/element doctor, independent synthetic H.264 NVDEC decode, decoder-engine utilization,
presentation/fallback tests, full regression, 실제 Workstation restart를 검증합니다. WIRIS가
연결되면 Left/Right live preview에서 실제 active decoder와 frame을 확인합니다.

## Cost, risks, assumptions, rollback, and field limits

현재 RTX 4050/595.84와 `nvh264dec`는 통과했습니다. Suspend/driver reset 후에는 다시
불가할 수 있어 Auto fallback을 보존합니다. Synthetic decode는 실제 dual RTSP 검증을
대체하지 않습니다. rollback은 Settings에서 Software 선택입니다.

## Exact approved scope

2026-08-26 17:45 KST에 다음 범위가 승인됐다.

1. decoder mode를 **Auto (GPU 우선, CPU fallback 허용)** 로 저장한다.
2. 실제 decoder는 기존 Camera `Decoder LEFT/RIGHT`에 가장 눈에 띄게 표시한다.
3. CAM L/R tooltip에는 짧은 actual/fallback 상태를, Diagnostics Performance에는
   driver/GPU/NVDEC 상세를 표시한다.
4. 새 global badge나 worker/resource policy 변경은 하지 않는다.
5. workspace README 대상은 `./README.md`로 한다.

Strict NVIDIA-only는 이번 기본 정책에 포함되지 않는다.
