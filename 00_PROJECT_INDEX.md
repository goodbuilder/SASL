---
tags:
  - sasl
  - project-index
---

# SASL 프로젝트 인덱스

Obsidian에서 프로젝트를 읽을 때 사용하는 시작점이다. 생성물, 외부 의존성,
대량 실험 결과는 의도적으로 연결하지 않고, 설계·운용·좌표·검증에 필요한
문서만 모은다.

## 1. 먼저 읽기

- [[./README.md|README]] — 전체 프로젝트 개요와 실행 진입점
- [[StereoWIRIS/docs/README]] — StereoWIRIS 문서 목차
- [[StereoWIRIS/docs/PROJECT]] — 목적, 구조, 측정 흐름, 운영 기준
- [[StereoWIRIS/docs/usage]] — 실제 실행·연결·측정·동기화 사용법
- [[StereoWIRIS/UPDATE]] — 최신 변경 이력

## 2. Workstation UI

- [[Workstation/docs/ui/README]] — Workstation 문서 세트 안내
- [[Workstation/docs/ui/00_PRODUCT_VISION]] — 제품 범위와 UI 방향
- [[Workstation/docs/ui/01_USER_WORKFLOW]] — 현장 운용 순서
- [[Workstation/docs/ui/02_INFORMATION_ARCHITECTURE]] — workspace 구조
- [[Workstation/docs/ui/03_WORKSPACE_LAYOUT]] — 화면 배치와 Time / Sync
- [[Workstation/docs/ui/07_COMMAND_MODEL]] — 명령과 owner 위임 계약
- [[Workstation/docs/ui/09_IMPLEMENTATION_PLAN]] — 구현 상태와 남은 작업
- [[Workstation/docs/ui/11_DESIGN_AUDIT_RESULT]] — 감사 결과와 조건

## 3. Measurement · Time / Sync · 좌표

- [[StereoWIRIS/docs/COORDINATE_CONTRACT]] — NED, body-FRD, optical RDF, stereo-center, CSV 계약
- [[StereoWIRIS/docs/usage#측정]] — 측정 pair, RTCP/NTP, 저장 데이터 설명
- [[Workstation/docs/ui/06_STATUS_MODEL]] — 상태·age·stale 판정
- [[Workstation/docs/ui/07_COMMAND_MODEL]] — Apply / Save 및 명령 상태
- [[./AGENTS.md|Root AGENTS]] — 작업 시 지켜야 할 좌표·실험·포터빌리티 규칙

## 4. 검증과 보정

- [[StereoWIRIS/docs/testing]] — 테스트와 검증 방법
- [[StereoWIRIS/docs/Calibration]] — calibration 입력·출력·검증 계약
- [[StereoWIRIS/docs/DESIGN]] — UI 설계 기준

## 5. 의도적으로 제외한 영역

다음 영역은 소스 저장소에는 필요하지만 Obsidian에서 프로젝트 문서를 읽을 때는
우선순위가 낮다.

- `Simulation/build/` — 생성된 빌드 산출물
- `Simulation/PX4-Autopilot/` — 외부/대형 PX4 소스 트리
- `Simulation/reports/experiments/` — 대량 실험별 결과 문서
- `Simulation/stereo_ground_sim/vendor/` — vendored 의존성
- `ref/` — 참고용 외부 자료
- `__pycache__/`, `.pytest_cache/`, `build/`, `install/`, `log/` — 실행 캐시·빌드 결과

원본 파일은 이동하거나 복제하지 않는다. 이 노트와 북마크를 통해 필요한 문서만
탐색한다.

## 읽기 순서

```text
README
  → PROJECT / usage
  → COORDINATE_CONTRACT
  → Workstation USER_WORKFLOW / WORKSPACE_LAYOUT
  → UPDATE
```

Coordinate contract checked: 이 인덱스는 문서 링크와 파일 분류만 제공하며, 측정값·좌표·CSV 스키마를 생성하거나 변환하지 않는다.
