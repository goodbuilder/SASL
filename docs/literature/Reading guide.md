---
tags:
  - literature-guide
  - sasl
---

# 문헌 해석 사전 읽기 안내

이 노트는 추가 사용자 배경지식 없이 후보 논문과 저장소 결과를 함께 읽기 위한 안내다. 논문별 요약·출처·Zotero key는 각 reference note를 본다.

## 먼저 구분할 검토 상태

- **Metadata verified:** 제목·저자·연도·venue·DOI를 publisher/공식 proceedings에서 확인.
- **Abstract checked:** 초록 수준에서 목적과 주장을 확인. 방법·실험의 세부 판단에는 부족하다.
- **Sections checked:** 방법, 결과, 한계 등 관련 절을 읽고 절/표/그림 위치를 기록.
- **Full paper reviewed:** 전문을 검토하고 주장·방법·실험·한계와 우리 후보와의 관계를 평가.

초록만 읽은 문헌은 “전문 리뷰 완료”로 쓰지 않는다. 저자가 주장한 novelty와 우리 비교 결과도 별도 항목으로 남긴다.

## Simulation 논문을 읽을 때

### 기하와 좌표

Rectified parallel stereo에서 left-u minus right-u를 양수 disparity로 정의하면 기본 관계는 `z = f_px B / d_px`다. `z`는 OpenCV optical RDF의 양의 Forward 방향 깊이(m), `B`는 baseline(m), `f_px`는 pixel 단위 초점거리, `d_px`는 pixel 단위 disparity다. 작은 disparity 오차의 깊이 영향은 `δz ≈ -(z²/(f_px B))δd`이므로 먼 거리에서 subpixel disparity error가 더 큰 거리 오차를 만든다. 이 관계는 이상화된 기본식이며, 렌즈 왜곡·축 정렬·카메라 자세 오차까지 설명하지는 않는다.

Simulation 후보의 correction output은 stereo-center OpenCV optical RDF `[right, down, forward]` metres다. Public StereoDistance topic은 별도 `FRD+range` `[forward, right, down, range]` contract다. 논문에서 이 이름과 순서를 서로 바꾸어 쓰지 말고 producer의 명시적 변환을 따른다.

### 방법과 평가 지표

- `7×3`은 7 bursts 각각에서 3개 image observations를 모아 총 21개 관측을 쓴다는 뜻이다.
- `trimmed mean`은 평균 전에 양쪽 꼬리의 일정 비율을 잘라 outlier 영향에 덜 민감하게 하는 요약값이다.
- Calibration-time train truth로 disparity residual을 적합하고, runtime에는 fused measured disparity와 corrected RDF point만 넣는다. Train/evaluation truth와 runtime observable 입력을 분리해서 읽는다.
- Range RMSE는 거리 오차를 크게 벌점 주는 평균 지표다. Position RMSE는 3D 위치 벡터 오차다. `target-window max abs error`는 지정한 목표 거리 구간의 최악 편차다.
- `STRICT_PASS`는 해당 실험이 사전에 정의한 gate를 통과했다는 뜻이다. 실제 카메라 정확도나 다른 연구의 검증 기준 통과를 뜻하지 않는다.

### 권장 읽기 순서

1. [[Existing Zotero stereo references]]의 Hartley–Sturm triangulation을 통해 두 ray와 픽셀 잡음의 관계를 확인.
2. [[Malapelle 2016 - Motion stereo fusion]]으로 시간 관측 융합의 기존 접근을 확인.
3. [[Chen 2023 - Stereo disparity uncertainty]]에서 disparity uncertainty를 평가하는 방법 확인.
4. 가장 가까운 비교 대상으로 [[Zhuang 2026 - UAV stereo triangulation]]을 읽고 데이터, correction 위치, label/ground-truth, held-out 조건을 비교.
5. [[Jaganmohan 2023 - Passive stereo systematic errors]]와 [[Sankowski 2017 - Stereo uncertainty verification]]를 통해 systematic calibration error 검출과 실제 측정 uncertainty 검증 절차를 비교.
6. [[Simulation - Multi-burst stereo]]의 결과표와 novelty 가설을 원 실험 보고서에 대조.

## Workstation 시간 동기화 논문을 읽을 때

### 시간 기준

- **RTCP/NTP:** Sender Report에 기록된 wall-clock 계열 시각.
- **RTP media clock:** 영상 stream tick과 rate.
- **Host monotonic:** 수신 시각을 재는 단조 증가 host clock.
- **Exposure time:** 센서가 실제 빛을 모은 시각. RTCP Sender Report의 NTP timestamp를 exposure timestamp라고 가정하지 않는다.

RFC 3550은 RTCP Sender Report에서 NTP와 RTP timestamp를 같은 report time에 대응시키는 방식을 설명한다. 저장소의 30분 probe는 NTP→host monotonic, unwrapped RTP ticks→host monotonic을 따로 회귀한 다음 Right−Left rate 차이를 비교한다.

`ppm`은 상대 rate의 parts per million이다. `δt_ms ≈ Δrate_ppm × duration_s / 1000`으로 drift 규모를 가늠할 수 있다. 예를 들어 16.6 ppm이 1,800초 유지되면 약 29.9 ms다. 이 계산은 대략적인 일관성 확인이며, report에 기록된 실제 offset 변화와 두 방법의 회귀 추정치를 동일한 통계량으로 취급하지 않는다.

**Offset과 drift는 다르다:** 시작 시의 약 3초 상대 phase offset과 관측 중 약 31 ms 변한 relative drift는 별개다. Clock-rate 일치만으로 image exposure 동기화나 3D triangulation accuracy를 증명할 수 없다.

### 권장 읽기 순서

1. [[RFC 3550 - RTP and RTCP]] §6.4.1에서 Sender Report의 NTP/RTP 대응 의미를 확인.
2. [[Gueguen 2025 - GNSS stereo time synchronization]]에서 GNSS trigger·LED·exposure latency 측정과 drift 문제를 확인.
3. [[Seyfu 2025 - Consumer stereo synchronization]]에서 image/reprojection 기반 sub-frame correction을 확인.
4. [[Zhou 2024 - Time-calibrated video synchronization]]와 [[Han 2024 - Coded-light camera timestamping]]을 비교해 video-content 기반 time mapping과 coded-light timestamp reconstruction을 구분.
5. [[Meyer 2026 - RocSync exposure and clock alignment]]에서 exposure-window reading, affine offset/drift, hardware reference 검증을 확인.
6. [[Workstation - Camera clock drift]]의 단일 30분 결과와 재현성 한계를 읽고, actual exposure/triangulation 측정을 추가해야 하는 이유를 점검.

## 각 paper note에 남길 리뷰 근거

전문 리뷰라고 기록하려면 다음을 논문에서 직접 확인해 남긴다.

- 연구 질문과 저자의 핵심 주장.
- 데이터, 장비, 조건, 비교군, train/validation split.
- 주장에 연결되는 결과 수치와 page/section/table/figure locator.
- 실패 사례, 데이터 누락, 평가 범위와 재현 한계.
- 가까운 선행 연구와 겹치는 부분, 실제로 다른 부분.
- 어떤 결과가 우리 가설을 지지하거나 약화하는지.

