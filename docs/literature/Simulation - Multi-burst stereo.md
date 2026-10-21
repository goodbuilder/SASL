---
tags:
  - paper-candidate
  - simulation
---

# Simulation 후보 — 다중 관측 융합과 disparity 잔차 보정

## 제안 질문

7×3 관측을 robust하게 융합한 뒤 측정 disparity에 따른 ray-scale residual을 학습하면, 1,852 m synthetic stereo ranging에서 여러 관측 오차 조건의 range/position error를 줄일 수 있는가? 잔차 적용 시 PX4/RTK truth를 runtime 입력으로 쓰지 않는 조건을 유지한다.

## 저장소 결과

실험 요약: [`EXP-20260621-288`](../../apps/Simulation/reports/experiments/EXP-20260621-288-b7x3-disparity-residual-all-scenario-fresh50/README.md). 6개 stress scenario에서 50개 fresh seed씩, 모두 `STRICT_PASS` (300/300). Intrinsic-scale와 bbox-outlier 추가 tail check도 각 100/100을 통과했다.

| Scenario | Range RMSE raw → observation → residual (m) | Position RMSE observation → residual (m) | Residual target max abs (m) |
| --- | --- | --- | --- |
| Nominal | 2.4146 → 0.2288 → 0.2206 | 0.2542 → 0.2469 | 1.0411 |
| Yaw/pitch stress | 2.4166 → 0.2322 → 0.2234 | 3.8767 → 3.8763 | 1.3356 |
| Bbox outlier | 2.6697 → 0.4022 → 0.3958 | 0.4201 → 0.4140 | 3.3829 |
| Combined sensor | 3.0271 → 0.7934 → 0.5085 | 6.3263 → 6.2967 | 2.8510 |
| Disparity noise | 4.2893 → 2.0378 → 0.4265 | 2.0409 → 0.4412 | 2.1441 |
| Intrinsic scale | 1.8512 → 2.3850 → 0.2234 | 2.3923 → 0.2910 | 1.2210 |

## 방법과 좌표 계약

- 관측 융합: 7 bursts × 3 image observations, inner/outer trimmed mean.
- Calibration-time train split으로 degree-2 disparity residual을 적합한다. Runtime은 fused measured disparity와 corrected stereo-center point만 사용하며, truth와 target-window label은 사용하지 않는다.
- Correction point/output: OpenCV optical stereo-center RDF `[right, down, forward]`, metres. Runtime disparity는 left/right image observation의 pixel disparity다.
- PX4 local NED truth는 calibration/evaluation에서만 명시적 transform으로 RDF에 들어온다. Public StereoDistance topic은 별도 FRD+range `[forward, right, down, range]` contract다.
- 사용 gate는 range RMSE와 position RMSE 개선, target-window sample 존재, target-window max absolute error ≤10 m이다. Summary에는 모든 seed의 axis별 RMSE 비교가 없으므로 all-axis 개선 주장을 할 수 없다.

## 문헌 연결과 차별화 과제

- [[Malapelle 2016 - Motion stereo fusion]] — 시간·공간 stereo observation을 Kalman filter로 결합한 선행 연구. 여기서는 짧은 관측 burst를 trimmed mean으로 융합하고 disparity에 따라 ray scale을 보정한다.
- [[Chen 2023 - Stereo disparity uncertainty]] — disparity error와 uncertainty를 함께 모델링한다. 본 방법에는 아직 calibrated per-estimate uncertainty가 없다.
- [[Zhuang 2026 - UAV stereo triangulation]] — UAV long-range triangulation과 learned target-center correction을 다루는 가장 가까운 비교 연구. 이 연구는 UWB 거리 truth로 이미지 center offset을 triangulation 전에 보정하고, 본 저장소 후보는 synthetic data에서 burst fusion 후 disparity 기반 3D ray scale을 보정한다. 새로움 주장은 두 방법의 benchmark 차이와 독립 test protocol을 확인한 뒤에만 세운다.
- [[Jaganmohan 2023 - Passive stereo systematic errors]] — induced camera-model parameter error에 민감한 CMM 측정선을 골라 systematic stereo error를 찾는 방법. 후보의 correction 성능 주장을 독립 측정 경로와 calibration-error stress로 검증할 때 참고한다.
- [[Sankowski 2017 - Stereo uncertainty verification]] — checkerboard 재구성 길이로 축별 measurement uncertainty와 recalibration 필요성을 확인한다. Simulation 후보의 error metrics를 실제 stereo system verification으로 확장하는 방법론적 비교다.
- [[Existing Zotero stereo references]] — Hartley–Sturm triangulation, stereo measurement uncertainty, calibration 배경.

새 두 paper는 평가 설계·uncertainty verification을 보완한다. 어느 쪽도 disparity residual correction이나 SASL의 성능을 검증한 연구는 아니다. 두 논문의 XYZ axes는 SASL optical RDF / baseline-center FRD로 alias하지 않는다.

## Novelty 가설 — 검증 전

가능한 기여는 **관측 융합 직후 disparity를 입력으로 하는 train-split ray-scale residual을 적용하고, runtime에는 측정 disparity와 RDF ray만 쓰는 조합**이다. 특히 intrinsic-scale 조건에서 observation-only range RMSE가 1.851 m에서 2.385 m로 나빠진 뒤 residual 적용 결과 0.223 m가 된 현상은 논문에서 검증할 만한 핵심 결과다.

가장 가까운 선행 연구는 [[Zhuang 2026 - UAV stereo triangulation]]이다. Zhuang et al.은 실제 UAV/UWB 자료에서 이미지 중심 오프셋을 triangulation 전에 보정하고, 이 후보는 synthetic stress data에서 다중 관측 융합 뒤 3D ray scale을 보정한다. [[Malapelle 2016 - Motion stereo fusion]]은 시간 관측을 Kalman 방식으로 결합하고, [[Chen 2023 - Stereo disparity uncertainty]]는 disparity uncertainty를 학습한다. 현재 비교는 방법 단계와 입력 차이를 찾은 수준이다.

**아직 novelty로 주장하지 않을 내용:** 새로운 disparity estimator, real-world robustness, 보편적인 고정 profile transfer, 실제 WIRIS 성능.

**Novelty 상태:** 가설. 선행 방법과 같은 데이터·split·측정량으로 직접 비교하고, 계수 고정 후 held-out scenario/device 검증과 실제 image/latency 평가를 해야 기여 범위를 확정할 수 있다.

## 현재 한계와 다음 검증

이 결과는 Simulation의 synthetic stress evidence다. 물리 WIRIS/실제 flight validation, image-domain test, runtime latency, 그리고 기기·환경 간 transfer 검증은 없다. 보고서의 method recipe가 seed마다 다시 적합되는지, 고정 profile이 seed/scenario를 넘어 transfer되는지 논문 실험표에서 분리해야 한다. EXP-288 자체는 default runtime deployment가 아니다. Runtime profile/selector 후보는 scenario label 기반 follow-up과 구분한다.

**추천 framing:** synthetic stress test에서의 range-error correction 방법 비교. 현 상태에서 실제 카메라 성능이나 보편적 profile transfer를 주장하지 않는다.

