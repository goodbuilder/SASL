---
type: literature-candidate
year: 2023
doi: 10.1016/j.precisioneng.2022.11.005
zotero_status: not-found-2026-10-20
tags:
  - simulation
  - stereo-metrology
  - systematic-error
---

# VDI/VDE 2634-1 Performance Evaluation Tests and Systematic Errors in Passive Stereo Vision Systems

**Prashanth Jaganmohan, Bala Muralikrishnan, Vincent Lee, Wei Ren, Octavio Icasio Hernandez, Ed Morse.** *Precision Engineering*, 79 (2023). [DOI](https://doi.org/10.1016/j.precisioneng.2022.11.005) · [NIST publication record and paper](https://www.nist.gov/publications/vdivde-2634-1-performance-evaluation-tests-and-systematic-errors-passive-stereo-vision)

## 문제와 방법

이 연구는 passive stereo camera model의 systematic parameter error가 3D 점 사이 길이 측정에 미치는 영향을 분석한다. Checkerboard calibration으로 얻은 26개 camera-model parameter 각각에 유도된 오차를 넣고, 측정 체적의 점 격자에서 모든 점 쌍의 길이 오차를 simulation으로 계산한다. 각 parameter에 민감한 model-based line을 고른 다음, Cartesian CMM으로 같은 점 격자를 이동하며 stereo 측정 길이를 기준 길이와 비교한다.

## 보고된 근거

NIST 기록은 model-based line 선택이 VDI/VDE 2634-1에 지정된 line test보다 systematic stereo error를 더 잘 드러냈다고 보고한다. 이 방법의 중심은 관측 체적에서 진단에 민감한 측정 경로를 고르는 것이다.

## SASL과의 관계

[[Simulation - Multi-burst stereo]]의 disparity-conditioned correction과 직접 같은 correction 알고리즘은 아니다. EXP-288 후보의 held-out 검증을 보강할 때 calibration parameter perturbation, 체적 안의 측정선 민감도, 독립 길이 기준 측정을 설계하는 방법론적 비교군이다. 저자들의 XYZ 길이 오차를 SASL optical RDF 또는 baseline-center FRD 좌표 오차로 간주하지 않는다.

[[Simulation - Calibration observability]]와도 연결된다. 두 연구 모두 기하 파라미터의 영향이 측정 결과에 어떻게 드러나는지 조사하지만, 이 논문은 CMM 기준의 systematic-error 진단에 초점을 둔다.

## 근거 링크

- [NIST 공식 publication record](https://www.nist.gov/publications/vdivde-2634-1-performance-evaluation-tests-and-systematic-errors-passive-stereo-vision)
- [DOI](https://doi.org/10.1016/j.precisioneng.2022.11.005)

