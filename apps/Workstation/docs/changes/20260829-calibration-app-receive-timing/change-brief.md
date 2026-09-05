# Change Brief

## Operator outcome

Calibration의 Live/Auto checkerboard pair가 카메라 RTCP/NTP clock 정렬 여부가 아니라,
각 decoded native frame이 Workstation 카메라 프로세스에 도착한 host-monotonic 수신시각의
Right-minus-Left 차이로 `Max Live pair Δt`를 판정한다. 운영자는 촬영 상태와
저장된 capture metadata에서 `app-receive-monotonic`이 실제 판정 기준임을 확인할 수 있다.

## Motivation

서로 다른 WIRIS의 NTP/RTCP clock offset과 epoch 변화가 Calibration pair 선택을 잘못
막지 않게 하고, 이미 Measurement에서 사용 중인 same-host decoded-frame 수신 clock
계약과 일관된 운영 기준을 제공한다. 수신시각은 물리적 노출시각이 아니므로
정밀 노출 증거로 승격하지 않는다.

## Included scope

- Live manual/Auto native-frame pair timing을 app-receive monotonic으로 판정
- 기존 ms 단위 `Max Live pair Δt` 임계값과 signed Right-minus-Left 계산 유지
- `HOST_RECEIVE_PROXY` 증거 분류, 수신시각/선택 기준/Δt의 capture metadata 저장
- 수신시각 누락·오류·임계 초과의 fail-closed stereo 제외
- Calibration UI 설명, 상태 문구, README 및 제품 계약 문서 갱신
- 기존 Photo set 및 FTP timing 계약의 호환성 회귀 테스트

## Excluded scope and non-goals

- 원시 RTP packet 도착시각 계측 또는 decoded frame으로의 추가 mapping
- 하드웨어 trigger, WIRIS firmware/NTP/STUS/RTCP 동작 변경
- FTP HR/OZ의 `CPTR` host-dispatch proxy 계약 변경
- Measurement live pairing, Photo set 선택, K,D/R,t 수학, 임계값 설정 UI 변경
- 시간 기준 선택 combo 또는 RTCP/NTP 자동 fallback 추가
- 기존 capture metadata 재작성·마이그레이션·삭제

## Acceptance summary

- NTP 값이 누락되거나 서로 큰 차이가 있어도 유효한 app-receive Δt가 임계 이내면
  Live/Auto stereo pair가 저장된다.
- app-receive Δt가 임계를 넘거나 한쪽이 없으면 사진은 checkerboard 통과 시 K,D용으로
  보존되지만 R,t pair에서 제외된다.
- metadata는 host receive proxy임과 물리 노출 미검증을 정직하게 표시한다.
- 기존 RTCP/NTP·FTP·legacy Photo set 증거는 재해석하지 않고 그대로 읽힌다.
