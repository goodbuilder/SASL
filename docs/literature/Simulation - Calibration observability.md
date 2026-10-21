---
tags:
  - paper-candidate
  - simulation
  - alternative
---

# Simulation 대안 — 카메라 자세 관측 가능성과 보정 경로

## 제안 질문

Stereo image, camera IMU prior, drone RTK 제약과 trajectory excitation을 바꾸면 camera attitude calibration의 conditioning과 held-out position error가 어떻게 달라지는가? 특히 drone RTK가 없는 stereo-only model은 어떤 조건에서 불안정해지는가?

## 현재 저장소 근거

- Analysis summary: [`analysis_summary.json`](../../apps/Simulation/recordings/stereo_v1_1/paper1_v1_2_001/analysis_versions/analysis_9cfd7c6cd10b/analysis_summary.json); methodology: [`analysis_methodology.md`](../../apps/Simulation/recordings/stereo_v1_1/paper1_v1_2_001/analysis_versions/analysis_9cfd7c6cd10b/analysis_methodology.md).
- 39 archived source runs (31 calibration, 8 evaluation), 100 deterministic Monte Carlo trials, 50,000 sensitivity cases. Freeze 기록은 V0 position error를 열기 전에 path P2를 covariance로 고르고 `v0_position_error_read_before_freeze=false`로 남긴다.
- RTK independent/joint 모델의 position RMSE는 약 1.18 m로 비슷하다. Drone RTK 없는 C 모델은 평균 error가 크게 커져 observability/failure-boundary 설명에 더 강한 결과다.
- Path P2와 P0의 predicted mean sigma 차이는 약 0.06%뿐이다. 큰 accuracy gain 주장은 맞지 않는다.

## 좌표와 해석

World reference는 ENU `[east,north,up]` metres, camera optical ray는 RDF `[right,down,forward]`, PX4 local state는 NED `[north,east,down]`이다. Method input/output axes를 서로 alias하지 않고, truth는 explicit source-to-camera transform으로 변환해야 한다.

이 분석은 저장된 39-run archive를 재사용한 synthetic sensor-model analysis다. v1.2 instruction은 아직 `SPECIFICATION_NOT_EXECUTED` 상태이며, 이 분석을 새로 수행한 39-flight v1.2 result라고 부르지 않는다. MC trials는 독립 physical flights가 아니다. Accuracy threshold도 정의되어 있지 않다.

## Novelty 가설 — 문헌 검토 미완료

가능한 기여는 V0 결과를 열기 전에 covariance 기준으로 경로를 선택하고, RTK가 없는 stereo-only 조건의 conditioning/failure boundary를 같은 Monte Carlo framework에서 분석한 점이다. 경로 P2와 P0의 예측 불확실도 차이는 약 0.06%이고 RTK가 있는 A/B의 위치 RMSE도 비슷해, accuracy improvement 주장은 근거가 약하다.

Camera–IMU self-calibration의 observability와 degenerate-motion 문헌을 추가 조사해 비교하기 전에는 novelty를 판단하지 않는다. 현재 상태는 **후보 가설, 문헌 gap 미확인**이다. 핵심 검증은 calibration model/trajectory별 observability 지표와 held-out flight 결과를 동일 조건에서 비교하는 것이다.

## 근거 파일

- [`analysis_freeze.json`](../../apps/Simulation/recordings/stereo_v1_1/paper1_v1_2_001/analysis_versions/analysis_9cfd7c6cd10b/analysis_freeze.json)
- [`campaign verification`](../../apps/Simulation/campaigns/stereo_v1_1/verification.md)
- [[Existing Zotero stereo references]]

