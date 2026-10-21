---
type: literature-candidate
year: 2024
doi: 10.1016/j.measurement.2024.115046
zotero_status: not-found-2026-10-20
tags:
  - workstation
  - camera-synchronization
  - timestamp-reconstruction
---

# Sub-frame timestamping of a camera network using a coded light signal

**Yunhyeok Han, Stefania Lo Feudo, Gwendal Cumunel, Franck Renaud.** *Measurement*, 236 (2024), 115046. [DOI / publisher record](https://doi.org/10.1016/j.measurement.2024.115046)

## 문제와 방법

카메라가 공통 hardware trigger를 지원하지 않는 상황에서 이미 기록된 영상의 frame timestamp를 사후 재구성한다. 여러 카메라의 공통 시야에 coded LED panel을 두고, 연속 프레임에서 LED 영역의 pixel intensity를 읽어 각 frame의 시각을 복원한다. 논문은 coded signal의 선택과 camera frame rate·exposure time이 timestamp uncertainty에 주는 영향을 simulation으로 검토하고, 실내 실험 및 tower-crane outdoor recording으로 확인한다.

## 보고된 근거

Publisher abstract는 uncontrolled outdoor condition에서 timestamp uncertainty가 frame interval의 1.65%였다고 보고한다. 이 값은 **frame timestamp reconstruction uncertainty**이지, SASL의 RTCP/RTP clock-rate estimate와 같은 통계량은 아니다.

## SASL과의 관계

[[Workstation - Camera clock drift]]와 [[Gueguen 2025 - GNSS stereo time synchronization]] 사이의 비교 지점이다. 이 방법은 장면 안 LED 신호를 사용해 frame 시각을 post-process하고, SASL 후보는 RTCP Sender Report의 NTP/RTP clock rate를 host monotonic receive time에 회귀한다. Coded light는 독립적인 영상 timestamp 검증 아이디어를 제공하지만, 공통 시야에 LED를 넣어야 하고 offline reconstruction 방식이다.

[[Zhou 2024 - Time-calibrated video synchronization]]와는 둘 다 사후 frame timing을 다루지만, Zhou는 별도 reference video와 marker/moving-ball correspondence를 사용하고 이 논문은 촬영 장면에 LED panel을 넣는다.

## 근거 링크

- [ScienceDirect publisher record](https://www.sciencedirect.com/science/article/pii/S026322412400931X)
- [DOI](https://doi.org/10.1016/j.measurement.2024.115046)

