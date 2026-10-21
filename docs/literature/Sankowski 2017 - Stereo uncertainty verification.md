---
type: literature-candidate
year: 2017
doi: 10.1016/j.imavis.2017.02.005
zotero_status: not-found-2026-10-20
tags:
  - simulation
  - stereo-metrology
  - uncertainty
---

# Estimation of measurement uncertainty in stereo vision system

**W. Sankowski, M. Włodarczyk, D. Kacperski, K. Grabowski.** *Image and Vision Computing*, 61 (2017), 70–81. [DOI / publisher page](https://doi.org/10.1016/j.imavis.2017.02.005)

## 문제와 방법

논문은 calibrated stereo system이 재구성한 3D 좌표의 uncertainty를 실험으로 확인하는 절차를 제안한다. Checkerboard를 서로 다른 장면 위치와 거리로 옮기고 두 영상에서 내부 corner를 검출한다. 재구성한 이웃 corner 사이 거리를 checkerboard의 알려진 간격과 비교하고, 여러 측정으로 저자 좌표의 X/Y/Z 방향 정확도와 불확실도를 추정한다. 별도 calibration-board test는 현재 calibration이 재실행되어야 하는지를 확인하며, 이 간단한 점검에는 laser distance meter가 필요하지 않다.

## 보고된 근거

Publisher abstract는 계산 절차에 simulation model과 축별 수식을 사용하고, 많은 반복 자료로 통계적 신뢰도를 높였다고 설명한다. 실험은 절차를 검증하고 calibration 상태 점검에 적용할 수 있음을 보인다.

## SASL과의 관계

[[Simulation - Multi-burst stereo]]의 stress simulation 결과를 실제 측정 검증으로 연결할 때 참고할 수 있다. 다만 이 연구는 disparity 기반 ray-scale correction이나 burst fusion을 다루지 않는다. 결과는 checkerboard feature localization과 거리 간격을 전제로 하며, 논문의 XYZ 축을 SASL의 optical RDF 또는 baseline-center FRD로 매핑하지 않는다. [[Simulation - Calibration observability]]의 held-out 측정 정확도와도 구분해 읽는다.

## 근거 링크

- [ScienceDirect publisher record](https://www.sciencedirect.com/science/article/abs/pii/S0262885617300495)
- [DOI](https://doi.org/10.1016/j.imavis.2017.02.005)

