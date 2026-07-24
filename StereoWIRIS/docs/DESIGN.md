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

## 금지 또는 제한

- 보라/파랑 그라데이션 중심의 장식 테마는 사용하지 않는다.
- 마케팅형 히어로 레이아웃이나 큰 홍보 문구는 앱 첫 화면에 두지 않는다.
- 운용 화면에서 불필요한 카드 중첩과 과한 둥근 모서리는 피한다.
- 텍스트가 버튼이나 라벨 안에서 잘리지 않도록 최소 크기와 줄바꿈을 유지한다.
