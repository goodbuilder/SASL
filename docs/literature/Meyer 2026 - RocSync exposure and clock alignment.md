---
type: literature-candidate
year: 2026
doi: 10.3390/s26031036
arxiv: 2511.14948v1
zotero_status: not-found-2026-10-20
tags:
  - workstation
  - camera-synchronization
  - exposure-time
  - clock-drift
---

# RocSync: Millisecond-Accurate Temporal Synchronization for Heterogeneous Camera Systems

**Jaro Meyer, Frédéric Giraud, Joschua Wüthrich, Marc Pollefeys, Philipp Fürnstahl, Lilian Calvet.** *Sensors*, 26(3), 1036 (2026). [DOI / publisher record](https://doi.org/10.3390/s26031036) · [Full arXiv HTML](https://arxiv.org/html/2511.14948)

## 문제와 방법

이 연구는 RGB와 IR을 포함한 heterogeneous camera recordings에서 hardware trigger 없이 exposure timing을 맞춘다. 저자들은 100개 LED를 1 ms씩 순차 점등하는 원형 배열과 100 ms 단위 binary counter로 구성된 LED Clock을 각 camera 앞에 촬영 시작·종료 때 보여준다. 영상에서 LED arc의 시작·끝으로 exposure window를 복원하고, camera-local time `t`와 global exposure time `T` 사이 `T = αt + β` 관계를 camera별로 적합해 clock drift `α`와 초기 offset `β`를 함께 추정한다. ArUco marker와 homography rectification으로 LED를 검출하며 timestamp는 robust linear regression으로 계산한다(§§3.1–4.2).

## 보고된 근거와 한계

두 Azure Kinect의 hardware-synchronization reference를 이용한 5회, 각 14분 recording에서 average RMSE는 1.34 ms였다(Table 1, §5.1). 대규모 surgical recording에는 RGB/IR 등 25대가 넘는 camera를 사용했지만, 해당 현장 기록은 움직임이 큰 장면을 시각 비교해 수동으로 확인했다고 설명한다(§5.6). 따라서 1.34 ms 수치를 그 25대 현장 기록의 ground-truth 정확도로 해석하지 않는다.

모델은 일정한 clock drift를 가정하고 temperature에 따른 비선형 변화를 목표 조건에서는 무시한다(§3.1). 또한 LED Clock을 촬영 영상에 보여야 하며, 현재 제시된 파이프라인은 live RTCP probe가 아니라 post-processing이다. 저자들은 rolling-shutter 효과가 residual 오차의 주요 원인이라고 논의한다(§5.1).

## SASL과의 관계

[[Workstation - Camera clock drift]]에 가장 직접적으로 추가할 비교 문헌이다. RocSync는 시각적으로 읽은 exposure window로 offset/drift를 추정하고, SASL은 Sender Report의 RTCP/NTP와 RTP media-clock rate를 host monotonic receipt time에 각각 회귀한다. 따라서 두 방법이 같은 clock quantity를 재는 것은 아니다. RocSync는 현재 SASL 보고서에 없는 exposure-time validation을 설계할 후보를 제공하며, SASL의 약 16.6 ppm stream-clock 차이를 sensor exposure skew로 해석하는 근거로 쓸 수는 없다.

[[Han 2024 - Coded-light camera timestamping]]과 함께 post-capture visual timing approaches로 묶고, [[Gueguen 2025 - GNSS stereo time synchronization]]의 GNSS-trigger/exposure-signal measurement와 구분한다.

## 근거 링크

- [Published article, DOI](https://doi.org/10.3390/s26031036)
- [Authors' full-text preprint (arXiv)](https://arxiv.org/html/2511.14948)

