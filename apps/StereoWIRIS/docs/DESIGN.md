# StereoWIRIS 디자인 기준

## 방향

StereoWIRIS의 UI는 IBM/Carbon 계열을 참고한 현장 운용 대시보드로 유지한다.

원본 레퍼런스는 `DESIGN-ibm.md`다. 이 문서는 그 레퍼런스를 이 프로젝트의 PyQt5 장비 운용 화면에 맞게 적용한 기준이다.

## 핵심 원칙

- 감성적인 마케팅 화면보다 장비 운용자가 빠르게 판단할 수 있는 정보 밀도를 우선한다.
- 장식, 그림자, 과한 그라데이션을 줄이고 얇은 구분선과 명확한 상태색을 사용한다.
- 색상은 흰색/회색/차콜 기반에 IBM Blue를 단일 주요 강조색으로 사용한다.
- 컴포넌트 모서리는 0~2px 수준으로 각지게 유지한다.
- 카메라 프리뷰처럼 영상 자체가 중요한 영역은 어두운 캔버스를 허용한다.
- 상태 표시는 Carbon식 success/warning/error/info 색상으로 구분한다.

## 색상

- Primary: `#0f62fe`
- Primary hover: `#0050e6`
- Primary active: `#0043ce`
- Background: `#f4f4f4`
- Panel: `#ffffff`
- Panel alternate: `#f4f4f4`
- Border: `#e0e0e0`
- Strong border: `#8d8d8d`
- Text: `#161616`
- Muted text: `#525252`
- Success: `#24a148`
- Warning: `#f1c21b`
- Error: `#da1e28`

## 컴포넌트

- 버튼:
  - 기본 버튼은 흰색 배경, 파란 텍스트, 회색 테두리를 사용한다.
  - 주요 실행 버튼은 파란 배경과 흰색 텍스트를 사용한다.
  - 위험 작업은 빨간 배경과 흰색 텍스트를 사용한다.
  - 성공/저장 성격은 초록 배경과 흰색 텍스트를 사용한다.

- 입력 필드:
  - 밝은 회색 배경과 강한 회색 테두리를 사용한다.
  - 포커스 상태는 IBM Blue 테두리로 표시한다.

- 상태칩:
  - 밝은 회색 배경, 얇은 테두리, 왼쪽 파란 강조선을 사용한다.
  - good/warn/bad 상태는 success/warning/error 배경과 테두리로 표시한다.

- 그래프:
  - 흰색 배경과 옅은 회색 grid를 사용한다.
  - Pixhawk 1/Left 계열은 빨간색, Pixhawk 2/Right 계열은 파란색을 우선 사용한다.
  - 거리/성공 계열은 초록색, 선택/강조는 주황색을 사용한다.

## 적용 위치

- `wiris_stereo_distance/theme.py`: 디자인 토큰.
- `wiris_stereo_distance/gui.py`: 전역 PyQt stylesheet, 상태 라벨, Pixhawk/RTK 그래프.
- `wiris_stereo_distance/measurement_widgets.py`: 거리/삼각측량 그래프.
- `wiris_stereo_distance/camera_pane.py`: 영상 오버레이 강조색.

## 화면별 정보 배치

- `정렬 > GNSS Base`는 설정, Base 연결 상태, RTCM 보정 데이터, Rover RTK 상태, Survey-in 진행, Survey-in 품질 진단 순서로 배치한다. RTCM 보정 데이터는 Base correction이 생성되고 Rover로 주입되는 transport 상태를 보여주며, Survey-in 진행은 완료 여부와 수렴 추세를 보여주고, 품질 진단은 NAV-SAT 기반 used/tracked 위성 수, C/N0, 고도각, constellation 요약처럼 "왜 안 끝나는지" 판단하는 보조 정보를 둔다.
- `보정 > 체커보드`는 줌 제어, 체커보드 pattern 설정, 저장 경로, 캡처, 좌/우 preview만 둔 촬영 전용 화면으로 유지한다. 사진셋 기반 계산, 기존 JSON 불러오기, random stability validation, 저장, 적용은 `보정 > 체커보드 검증/계산`에서만 수행해 현장 캡처와 파라미터 적용 단계를 혼동하지 않게 한다.
- `정렬 > Optical 정렬`은 물리적 카메라 R 조절용 preview-only 화면으로 둔다. 상단에는 계산 파라미터를 변경하지 않는다는 상태를 표시하고, 본문은 좌/우 target 관측값, vertical/pitch residual, horizontal/yaw residual, live 거리 영향 순서로 배치한다. 영상 미리보기보다 `dx/dy`, expected `dy`, expected disparity, residual px/mrad, YOLO Range, ray gap, baseline/triangulation source 같은 판단 수치를 우선한다.
- `파라미터 > 외부/기하`의 좌/우 extrinsic 패널은 `계산 설정`의 외부 파라미터 소스와 같은 의미를 표시한다. Pixhawk 모드에서는 live Pixhawk 자세를 보여주고, 체커보드 모드에서는 실제 삼각측량에 쓰는 applied checkerboard stereo `R/T`의 left reference/right-in-left RDF 값을 보여준다. 체커보드 모드의 Pixhawk telemetry는 같은 패널 안에 diagnostic-only로만 표시한다.

## 금지 또는 제한

- 보라/파랑 그라데이션 중심의 장식 테마는 사용하지 않는다.
- 마케팅형 히어로 레이아웃이나 큰 홍보 문구는 앱 첫 화면에 두지 않는다.
- 운용 화면에서 불필요한 카드 중첩과 과한 둥근 모서리는 피한다.
- 텍스트가 버튼이나 라벨 안에서 잘리지 않도록 최소 크기와 줄바꿈을 유지한다.
