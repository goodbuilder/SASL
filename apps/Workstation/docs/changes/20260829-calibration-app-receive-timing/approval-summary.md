# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

Calibration의 `Capture Live Pair`와 Auto capture가 좌·우 카메라 NTP 시각 차이 대신,
각 decoded native frame이 Workstation에 도착한 host monotonic 수신시각 차이로
`Max Live pair Δt`를 판정합니다. 기존 임계값, 촬영 버튼, Photo set, checkerboard
검출, K,D/R,t 계산과 적용은 그대로입니다.

FTP HR/OZ는 기존 CPTR 명령 dispatch 기준을 그대로 사용하고, 미래에 실제로 검증된
노출시각이 제공되는 경우의 상위 증거 계약도 보존합니다. Measurement pairing은
변경하지 않습니다.

## Operator workflow before and after

운영 순서나 새 제어는 없습니다. Preview → Capture/Auto → Photo set 선택 → 계산을
그대로 수행합니다. 다만 tooltip과 상태/저장 metadata에
`app-receive-monotonic`이 실제 Live gate 기준으로 표시됩니다. NTP 시각은 사진의
wall-clock/진단 정보로 남지만 pair 승인을 결정하지 않습니다.

## Success, delay, rejection, failure, overload, and recovery

- 양쪽 수신 Δt가 임계 이내/경계이면 stereo pair 후보로 저장합니다.
- 임계를 넘거나 수신시각이 누락/오류이면 R,t에서 제외하지만 checkerboard가
  유효한 각 사진은 K,D용으로 보존합니다.
- NTP 미동기에서 수신 Δt가 정상이면 촬영을 다시 할 필요가 없습니다.
- worker, queue, decoder, capture concurrency를 바꾸지 않으므로 새 delay/overload 경로가 없습니다.

## Data and compatibility effects

신규 capture JSON에 양쪽 app-receive monotonic ns, signed Δt, 기준, host-receive proxy 분류가
저장됩니다. 과거 Photo set은 수정·삭제·재판정하지 않습니다. 신규 metadata도
기존 allow/reject boolean을 계속 저장하므로 legacy pair reader와 호환됩니다.

## Performance priorities and allowed degradation

수신시각은 이미 생성되므로 추가 worker/I/O/GPU/CPU 정책이 없습니다. 수신 증거가
없을 때는 정확성을 위해 자동 NTP fallback 대신 해당 pair만 K,D-only로 내립니다.

## Visual and interaction changes, including dynamic-layout stability

기존 tooltip과 wrap 가능 설명/상태 문구만 바뀌고 control·레이아웃·focus·selection·scroll·
table geometry는 바꾸지 않습니다.

## How acceptance will be demonstrated

- 서로 다른 sentinel NTP가 있어도 app-receive Δt만으로 판정하는 단위 테스트
- signed Δt, 임계 동일/초과, 누락/오류의 경계 테스트
- worker JSON에 양쪽 ns/기준/분류/false precision·exposure flag가 저장되는 통합 테스트
- Auto capture 동일 판정, FTP/exposure/legacy Photo set 불변 회귀
- focused suite, 전체 pytest, compile/static/Qt offscreen 또는 launcher smoke
- 실제 dual-WIRIS field capture는 자동화 테스트와 별도의 manual acceptance로 보고

## Cost, risks, assumptions, rollback, and field limits

App receive는 decoded frame의 host 도착시각이며 물리 노출시각이 아닙니다. 네트워크, jitter buffer,
디코딩, process scheduling 지연이 섞이므로 이 변경은 NTP 의존성을 제거하지만 동시 노출을
증명하지 않습니다. 움직이는 보드는 멈춘 후 촬영해야 합니다. Rollback은 이 패킷의
코드/문서 hunk만 되돌리며 기존 사용자 수정은 보존합니다.

## Exact approved scope

2026-08-29에 사용자가 `Calibration Live/Auto app-receive timing`, 관련 capture evidence/UI wording/compatibility tests,
`README.md` 및 영향받는 Calibration product documents의 최소 vertical slice를 승인했습니다.
FTP, Measurement, camera protocol/worker/resource policy, raw RTP timing, settings migration,
Photo set/K,D/R,t 수학과 관련 없는 dirty worktree 정리는 제외합니다.
