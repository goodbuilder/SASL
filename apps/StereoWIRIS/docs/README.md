# StereoWIRIS 문서

## Purpose

StereoWIRIS Legacy와 독립 Workstation의 좌·우 WIRIS 영상, Pixhawk/GNSS,
NTRIP/RTCM, 측정 및 증거 저장 운용 계약을 안내한다.

## Current Run Flow

1. `run.sh`로 앱을 시작하고 camera/Pixhawk endpoint를 확인한다. Legacy는 과거 미완료
   spool이나 intrinsics를 자동 채택하지 않고 빈 세션·미적용 intrinsics 상태로 시작한다.
2. RTSP는 기본 `TCP + Auto decoder`로 연결한다. 같은 SSRC의 RTP forward-sequence gap은
   현재 decoder를 바꾸지 않고 손상 가능 encoded/decoded output을 다음 clean intra AU까지
   차단한다. parsed AU의 PTS 누락, `DISCONT`, `CORRUPTED`도 같은 gate를 사용한다.
   `nvh264dec`의 첫 프레임 30초 timeout, live 30초 stall 또는 bus health failure만 Auto의
   software fallback 근거다.
3. NTRIP username/password를 입력하고 필요할 때만 OS 보안 저장소 기억을 켠다.
4. 기록을 누른 시점에 새 session을 만들고 camera worker를 MP4 tee가 있는 pipeline으로
   재시작한다. raw-video 부모 디렉터리를 먼저 만들고 non-ASCII 경로를 GStreamer property로
   정확히 serialize한다. PLAYING 전환 실패만 0.5/1/2/4초 bounded retry하며, 종료 시 plain
   MP4 mux의 EOS와 strict video 검증이 끝난 뒤 산출물을 확정한다.

## Inputs/Outputs and Interfaces

- Camera: 좌·우 RTSP H.264, WIRIS SDK control TCP.
- Pixhawk: side별 MAVLink UDP와 선택적 ROS2/uXRCE-DDS.
- GNSS: Base serial, NTRIP caster, MAVLink `GPS_RTCM_DATA`.
- Output: session JSON/CSV/Parquet/XLSX와 `video/raw_left.mp4`, `video/raw_right.mp4`.

## Operational Rules

- PX4 Ethernet MAVLink instance는 `MAV_2_CONFIG=1000`만 소유한다. 수동
  `mavlink start/stop`을 실행하지 말고 파라미터 저장 후 재부팅한다.
- 동일 raw MAVLink packet은 앱 경계에서 조용히 무시한다.
- NTRIP password는 settings JSON이나 evidence에 저장하지 않는다.
- MP4가 누락·0바이트·잘못된 header이면 spool을 보존하고 저장을 실패로 표시한다.
- `STEREO_PREVIEW_IDENTITY_LOG=/tmp/<name>.jsonl`은 검증 실행에서만 사용한다.
  producer RGB bytes, process queue bytes, deep-copied QImage의 SHA-256과
  sequence/drop/reorder를 같은 representation으로 기록한다.

## Known Limitations and Workarounds

- RTP gap 없이 `nvh264dec`가 오류 flag도 내지 않는 별도 시각 artifact는 자동 판정할
  수 없다. 명시적 `NVIDIA`는 fail-closed이며, 현장 진단에서는 `Software`와 같은 저장
  bitstream을 replay해 비교한다.
- 현재 WIRIS sender는 완전 재조립된 TCP byte stream 안에서도 RTP sequence를 누락하고,
  `GEIR=0` stream은 실제 IDR NAL을 제공하지 않는다. 앱 gate는 corruption 대신 수초 freeze를
  선택할 수 있지만 빠진 reference pixel을 복구할 수 없다. QP/SEBR/latency/TCP-UDP A/B로도
  corruption 0과 freeze 0을 동시에 만족하지 못했으며 vendor firmware/device fix가 필요하다.
- Legacy startup은 과거 retryable spool을 자동 복구하지 않는다. 파일은 보존되므로 필요하면
  명시적 recovery utility로 처리하고, 측정 전 현재 zoom의 intrinsics를 수동 불러오기·적용한다.
- OS keyring backend가 없으면 NTRIP password는 현재 실행 메모리에만 유지된다.
- 과거의 손상·누락·대용량 MP4는 자동 삭제하지 않는다.
- plain MP4는 정상 EOS에서 엄격 decode 가능한 파일로 finalization된다. 갑작스러운 전원 차단이나
  process kill로 EOS가 실행되지 않으면 해당 MP4가 열리지 않을 수 있으며 spool/video validation이
  실패 상태를 보존한다.

## Change Log

- 2026-08-25: recorder lifecycle, single-owner MAVLink, duplicate packet 무시,
  secure NTRIP credential, TCP+Auto decoder fallback 및 latest-only preview를 정리했다.
- 2026-08-26: RTP sequence gap 이후 현재 decoder output을 clean intra까지 차단하고,
  선택적 frame-identity JSONL, 기록 재시작의 bounded PLAYING retry, 빈 Legacy session 시작과
  수동 intrinsics 적용 계약을 추가했다. GStreamer non-ASCII filesink 경로와 fragmented MP4
  sample-offset 손상을 수정해 실제 앱 저장 파일의 strict full decode를 통과시켰다.
  장기 실행에서 발견된 PTS 없는 AU도 decoder/MP4 전에 차단하도록 gate를 확장했다.

## 문서 목차

이 디렉터리는 프로젝트 문서를 보관한다. 루트에는 변경 이력인 `../UPDATE.md`만 둔다.

핵심 문서:

- `usage.md`: 실행과 운용 안내.
- `PROJECT.md`: 프로젝트 목적, 현재 구조, 측정 흐름, 현장 워크플로, 분리 계획.
- `COORDINATE_CONTRACT.md`: 좌표계, 축 이름과 Raw CSV 필드 계약.
- `Calibration.md`: nominal/corrected nominal 보정 탐색과 truth/reference 사용 계약.
- `DESIGN.md`: UI/스타일 기준.
- `DESIGN-ibm.md`: IBM/Carbon 계열 원본 디자인 레퍼런스.
- `AGENTS.md`: 코딩 에이전트 작업 규칙.
- `../UPDATE.md`: 변경 이력.

세부 문서:

- `testing.md`: 테스트와 검증 방법.
- `changes/20260826-wiris-preview-artifact/root-cause-and-remediation-ko.md`:
  WIRIS preview/recording/sync 장애의 통합 원인 분석, 배제 가설, 해결안과 vendor 재검증 기준.

기타 프로젝트 파일:

- `../requirements/runtime.txt`: 앱 기본 실행 의존성. Live undistortion/ray 계산용 OpenCV·NumPy와 rig alignment용 PyYAML을 포함한다.
- `../requirements/calibration.txt`: calibration bootstrap이 단독으로 설치할 수 있는 OpenCV·NumPy·PyYAML 부분집합.
