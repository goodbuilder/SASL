# StereoDistance

## Purpose

StereoDistance는 좌·우 WIRIS 카메라와 Pixhawk 텔레메트리를 사용해 스테레오 거리를 측정하고 기록·분석하는 GUI 애플리케이션이다.

## Current Run Flow

```bash
./run.sh
```

실행 환경만 점검하려면 `./run.sh --check`를 사용한다. 세부 설치·운용 절차는 [docs/usage.md](docs/usage.md)를 참고한다.

## Inputs/Outputs and Interfaces

- 입력: WIRIS RTSP/TCP/FTP, Pixhawk ROS 2·MAVLink 텔레메트리, 보정 JSON, PX4 ULog.
- 출력: 실시간 스테레오 거리, 측정 CSV, 세션 메타데이터, 보정·비교 결과.
- 카메라 RTC 동기화: `시간 > WIRIS RTC · PC UTC 동기화`에서 양쪽/왼쪽/오른쪽을 선택한다.

## Operational Rules

- PC UTC 동기화는 선택 대상에 `STUS <epoch-seconds> <microseconds>`만 전송한다. 양쪽 대상은 동일한 UTC 값을 사용한다.
- 동기화 정확도는 `저장·분석 > 촬영 시각 검증`에서 촬영 파일의 EXIF `right-left` 차이로 확인한다.
- `파라미터 > 외부/기하`는 선택한 외부 파라미터 소스를 기준으로 표시한다. 체커보드 소스에서는 `rotation_right_to_left` R, `translation_right_in_left_m` t, baseline, pair 수, stereo RMS, epipolar 평균·최대 오차를 `Stereo Geometry`에서 확인한다.
- 좌표계·시각 계약은 [docs/PROJECT.md](docs/PROJECT.md)와 [docs/COORDINATE_CONTRACT.md](docs/COORDINATE_CONTRACT.md)를 따른다.

## Known Limitations and Workarounds

- `STUS` 응답만으로는 좌·우 촬영 timestamp 차이를 증명하지 못한다. EXIF 촬영 시각 검증을 함께 사용한다.
- 실제 정확도는 카메라 RTC, 네트워크 지연, 촬영 파일의 subsecond EXIF 지원에 의존한다.

## Change Log

- 2026-07-16: `파라미터 > 외부/기하`에서 체커보드 stereo R/t와 RMS·epipolar 품질 지표를 함께 표시하도록 했다.
- 2026-07-16: 4분할 거리 그래프의 중복 상단 제목을 제거하고 Y축 상·하한과 하단 범례만 표시하게 했다.
- 2026-07-16: UTC 동기화를 자동 `GDTI` 조회 없는 `STUS` 전용 전송으로 변경했다. 전체 이력은 [UPDATE.md](UPDATE.md)에 있다.
