# Ground Stereo Drone Simulation

Gazebo Sim + ROS 2 Humble 기반 지상 고정 스테레오 카메라 시뮬레이션입니다.
PyQt와 브라우저 대시보드는 사용하지 않고, 표준 ROS 2 도구인 `rqt_gui`, RViz2, rosbag을 사용합니다.

## 구성

- 카메라: 지상 고정 2대
- baseline: 25 m
- camera height: 10 m
- stereo type: parallel stereo
- resolution / FPS: 1280x720 @ 30 Hz
- horizontal FOV: 0.154997710698 rad, WIRIS Enterprise optical zoom 8x 기준 약 8.88 deg
- distortion: off
- target drone: 약 폭 15 m, 높이 10 m 대형 쿼드콥터 형상
- target motion: QGC/PX4 조종 관찰용. 시작 위치는 카메라 전방 +Y 약 2000 m입니다.
- viewer: 단일 `rqt_gui` 운용 창, 선택적으로 RViz2

## 시뮬레이션 목적

이 시뮬레이션의 목적은 StereoDistance로 실제 본 실험에 가까운 조건의
거리/축별 위치 측정과 보정 방법을 검증하는 것이다. 기준 운용 조건은
25 m stereo baseline, WIRIS visible 8배줌, 약 15 m급 드론/UAM 표적이다.
StereoDistance는 이 조건에서 Stereo WIRIS 영상과 YOLO bbox를 사용해
`estimated_distance`와 카메라 중앙 기준 `Forward/Right/Down` 축 데이터를
산출한다.

측정 수준을 벗어나는 자세각 오차는 단순한 raw stereo 계산만으로 안정적으로
제거하기 어렵다. 따라서 드론/UAM에 Pixhawk와 GPS를 사용해 보정용 비행을
수행하고, 이때 얻은 위치 기준값을 ground truth로 사용한다. 시뮬레이션은
보정용 비행에서 만든 ground truth와 Stereo WIRIS로 측정한 데이터를 비교해
자세각 오차가 거리와 각 축에 만드는 편향을 분석하고, 이를 줄일 보정값과
보정 정책을 찾는 역할을 한다.

그 다음 계측용 실험 조건에서는 보정용 비행에서 얻은 정보를 이용해 고정한
보정 방법을 적용한다. 시뮬레이션에서는 계측용 조건에서도 ground truth를
알 수 있으므로, 보정 후 `ground_truth`, `estimated_distance`, 그리고
`Forward/Right/Down` 축 데이터가 최대한 동일하게 나오고 축별 regression이
발생하지 않는 방법을 찾는다. 실제 계측용 본 실험에서는 ground truth를
런타임 입력으로 사용하지 않고, 시뮬레이션과 보정용 비행에서 검증한 보정
방법을 적용한 뒤 사후 분석으로 정확도를 확인한다.

## RTK 스테레오 보정법 비교 파이프라인

PX4 local position NED를 보정 비행의 ground truth로 두고, 스테레오 카메라의
자세오차와 시차오차를 줄이는 후보 보정법을 단일 Python 실행으로 비교할 수
있다. 실제 Gazebo/PX4/YOLO 토픽 CSV가 아직 없어도 synthetic mode가 기본으로
동작한다.

좌표계 계약은 다음과 같다.

- PX4 local position: `px4_local_position_ned:north,east,down`
- 스테레오 중심 카메라 좌표: `stereo_center_opencv_optical:RDF(right,down,forward)`
- synthetic bridge: `rdf_right=east`, `rdf_down=down`, `rdf_forward=north`

보정 후보는 모두 RDF 배열 순서 `[Right, Down, Forward]`에서 학습/평가된다.
legacy `x/y/z` 필드는 쓰지 않고 CSV에는 `*_right_m`, `*_down_m`,
`*_forward_m`, `truth_north_m/east_m/down_m`처럼 축 이름을 명시한다.

기본 synthetic 실행:

```bash
Simulation/.venv/bin/python Simulation/scripts/rtk_stereo_calibration_pipeline.py
```

결과 위치를 지정하려면:

```bash
Simulation/.venv/bin/python Simulation/scripts/rtk_stereo_calibration_pipeline.py \
  --output-dir Simulation/results/rtk_stereo_calibration_smoke
```

정책별 추천이 seed/split에 얼마나 안정적인지 보려면 synthetic multi-seed
집계를 실행한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_policy_multi_seed_validation.py \
  --output-dir Simulation/results/rtk_stereo_policy_multi_seed \
  --seeds 20260613,20260614,20260615
```

CSV 기록 데이터를 여러 split seed로 재검증하려면 `--mode csv --input-csv <file>`를
같이 지정한다. `--output-dir`와 CSV mode의 `--input-csv`는 wrapper 시작 시
절대 경로로 해석되므로, `Simulation/results/...` 같은 상대 경로를 넘겨도
내부 pipeline subprocess와 후속 replay/promotion check가 같은 산출물 위치를
참조한다. 출력은 `seed_runs.csv`, `per_seed_method_comparison.csv`,
`per_seed_policy_recommendations.csv`, `method_aggregate_summary.csv`,
`policy_stability_summary.csv`, `policy_stability_gate.csv`,
`method_stability_gate.csv`, `selection_near_ties.csv/json`,
`method_family_diagnostics.csv/json`,
`multi_seed_recommendation.json`,
`multi_seed_recommended_profiles.csv/json`,
`selection_near_tie_profiles.csv/json`,
`multi_seed_representative_profile.json`, `representative_runtime_profile.json`,
`representative_profile_checks.json`,
`representative_cross_seed_replay_summary.csv/json`,
`multi_seed_summary.json/txt`, `policy_stability_ci.png`,
`method_aggregate_ci.png`이다. `policy_stability_gate.csv`는 모든 seed 실행
성공, raw 대비 평균 validation RMSE 개선, 1852m 허용오차 통과, 정책별
선택 method의 과반 안정성을 분리해 기록한다. raw 대비 평균 validation RMSE
개선은 기본적으로 최소 `0.01m` 이상이어야 하며
`--min-validation-improvement-m`으로 조정할 수 있다. aggregate summary에는
validation RMSE와 1852m 오차 평균의 bootstrap 90% confidence interval도
함께 기록한다. `method_stability_gate.csv`는 정책과 무관하게 각 보정법이
seed 전체에서 raw 대비 개선, 1852m 허용오차, validation CI 기준을 통과하는지
기록한다. `multi_seed_recommendation.json`은 strict pass method를 먼저
고르고, 없으면 performance pass method 중 validation RMSE 평균, validation
worst-seed gap, 1852m 오차 평균과 seed별 최대값, CI 폭, RDF 축별 RMSE
regression, 복잡도 penalty를 함께 본 후보를 기록한다.
`method_stability_gate.csv`에는 raw 대비 RDF right/down/forward RMSE가
후퇴했는지와 최대/합산 regression도 함께 기록된다. 또한 multi-seed 추천과
같은 점수식으로 계산한 `selection_score`, `selection_rank`,
`selection_score_contract`가 기록되어 최종 추천 후보와 차순위 후보의 trade-off를
같은 CSV에서 비교할 수 있다.
`selection_near_ties.csv/json`은 rank 1 대비 score delta가 작거나 상위 rank에
있는 후보들을 따로 모아, 최종 추천이 단독 우세인지 아니면 사실상 동률 후보가
있는지 validation/1852m 차이와 함께 기록한다.
`method_family_diagnostics.csv/json`은 각 method를 `registration_model`,
`residual_model`, `regularization_model`, `runtime_profile_family` 조합으로
묶어 family별 best method, strict pass 수, validation/1852m 성능을 기록한다.
이 파일은 다음 자동탐색에서 어떤 보정 계열을 더 넓힐지 판단하기 위한 offline
진단이며, runtime truth 입력 계약을 바꾸지 않는다.
`multi_seed_recommended_profiles.csv/json`은 multi-seed 추천 method에 대해
각 seed의 `runtime_profiles/<method>.json` 경로, seed별 validation 성능,
축 계약, runtime 입력 계약을 모아 실제 StereoDistance 적용 후보 검토에
사용한다. `multi_seed_representative_profile.json`은 seed별 profile 중
multi-seed 평균 validation/1852m 성능에 가장 가까운 대표 seed profile을
고른 메타데이터이고, `representative_runtime_profile.json`은 그 profile을
최상위로 복사한 바로 적용 가능한 runtime profile이다.
`selection_near_tie_profiles.csv/json`은 최종 추천과 점수 차이가 작은
near-tie 후보들까지 seed별 runtime profile 경로와 validation/1852m 성능을
함께 남긴다. 이 파일은 단일 winner만 고정하지 않고 StereoDistance에 적용
가능한 후보 profile을 계속 탐색하기 위한 offline 후보 manifest이며, runtime
입력은 raw stereo-center RDF와 measured disparity로 제한된다.
`representative_profile_checks.json`은 해당 대표 profile을 validation split에
replay하고 promotion check까지 실행한 결과를 요약한다. replay는 raw 대비
corrected range RMSE 개선과 1852m 오차 통과를 확인하고, promotion check는
StereoDistance loader 호환성과 runtime truth 입력 금지를 확인한다.
`representative_cross_seed_replay_summary.csv/json`은 동일한 대표 profile을
모든 완료 seed의 validation split에 다시 replay해, 특정 seed에만 맞는
profile인지 아니면 seed 전체에서 raw 대비 개선과 1852m 조건을 통과하는지
검증한다.

오차 주입 조건을 바꿔도 같은 보정 계열이 유지되는지 보려면 robustness
sweep을 실행한다. 이 sweep은 multi-seed 검증을 여러 synthetic error
scenario에서 반복하고, 각 scenario의 `method_stability_gate.csv`를 다시
모아 cross-scenario 기준으로 추천 method를 고른다. 목적은 한 seed나 한
오차 조건에서만 우연히 좋은 후보가 아니라, StereoDistance runtime profile로
적용 가능한 안정적인 estimated-to-truth 보정 후보를 계속 탐색하는 것이다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_robustness_sweep.py \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep \
  --seeds 20260613,20260614,20260615
```

기본 scenario는 `nominal`, `yaw_pitch_stress`, `disparity_noise_stress`,
`intrinsic_scale_stress`, `combined_sensor_stress`, `bbox_outlier_stress`이다.
`combined_sensor_stress`는
자세오차, baseline/focal scale 오차, disparity bias/noise, bbox-center noise를
동시에 키워 단일 오차 조건에만 맞는 후보를 걸러낸다. `sparse_target_window`는
기본에는 포함하지 않는 진단용 scenario이며, 1852m 근방 표본이 부족할 때
target-window gate가 PASS를 막는지 확인할 때만 `--scenarios`에 명시한다.
`bbox_outlier_stress`는 일부 sample의 bbox 중심과 disparity에 큰 튐을
주입해 RANSAC, trimmed, Huber, Tukey 같은 강건 정합 후보가 Gaussian noise
조건뿐 아니라 gross outlier 조건에서도 안정적인지 확인한다. pooled scenario
profile은 기본적으로 여러 ranking 기준에서 뽑은 후보를 transfer replay하며,
`--pooled-candidate-scan-limit`로 replay할 pooled runtime profile 수를
조절한다. `0`으로 지정하면 pooled pipeline이 export한 모든 runtime profile을
replay한다.
추가 synthetic error scenario는 JSON으로 정의해 자동 탐색 범위를 늘릴 수
있다.

```json
{
  "scenarios": [
    {
      "name": "custom_disparity_noise",
      "purpose": "larger disparity and bbox-center noise",
      "args": ["--disparity-noise-px", "0.20", "--bbox-noise-px", "0.40"]
    },
    {
      "name": "custom_bbox_outlier",
      "purpose": "gross bbox-center outlier stress",
      "args": ["--bbox-outlier-rate", "0.10", "--bbox-outlier-px", "3.0"]
    }
  ]
}
```

```bash
Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_robustness_sweep.py \
  --output-dir Simulation/results/rtk_stereo_custom_sweep \
  --scenario-config-json Simulation/config/custom_scenarios.json \
  --scenarios nominal,custom_disparity_noise
```

출력은 `scenario_runs.csv`,
`scenario_recommendations.csv`, `scenario_method_gates.csv`,
`robust_method_summary.csv`, `robust_method_family_summary.csv`,
`robust_policy_recommendations.csv`,
`scenario_near_tie_profiles.csv/json`,
`scenario_near_tie_profile_transfer_replay.csv`,
`scenario_near_tie_profile_transfer_summary.csv/json`,
`scenario_near_tie_method_transfer_summary.csv/json`,
`scenario_near_tie_best_profile.json`,
`scenario_near_tie_best_runtime_profile.json`,
`scenario_near_tie_best_promotion_check.json`,
`robust_policy_profile_candidates.csv`,
`robust_policy_representative_profiles.csv`,
`robust_policy_profile_transfer_replay.csv`,
`robust_policy_profile_transfer_summary.csv`,
`robust_policy_profile_transfer_diagnostics.csv/json`,
`scenario_profile_catalog.csv/json`,
`scenario_profile_catalog_promotion_checks.csv/json`,
`scenario_profile_catalog_transfer_replay.csv`,
`scenario_profile_catalog_transfer_matrix.csv`,
`scenario_profile_catalog_transfer_coverage.csv`,
`scenario_profile_catalog_transfer_summary.json`,
`scenario_profile_catalog_minimal_set.csv/json`,
`pooled_scenario_profile_transfer_replay.csv`,
`pooled_scenario_profile_summary.csv/json`,
`pooled_scenario_profile_transfer_diagnostics.csv/json`,
`deployment_strategy_recommendation.json/txt`,
`robustness_summary.json/txt`이다.
`robust_method_summary.csv`는 method별 scenario 통과 횟수, validation RMSE
평균/최대, 1852m target error 평균/최대, RDF 축별 RMSE regression 합,
복잡도 penalty를 기록한다. `robust_method_family_summary.csv`는
`registration_model`, `residual_model`, `regularization_model`,
`runtime_profile_family`가 같은 후보들을 묶어 계열별 best method와 score를
요약한다. `robustness_summary.json`의
`robust_recommendation`은 모든 scenario strict pass 후보를 먼저 고르고,
없으면 performance pass 전체 통과 후보, 그것도 없으면 partial robustness가
가장 좋은 후보를 기록한다. `robust_policy_recommendations.csv`는 같은 pass
후보를 `balanced`, `validation_best`, `target_guard`, `simple_stable` 기준으로
다시 추천해, validation RMSE 최저 후보와 1852m worst-case 보수 후보가
달라지는지 확인한다. 각 policy method에 대해서는 모든 scenario/seed의
`runtime_profiles/<method>.json` 후보를 모아 policy 평균 validation/1852m
성능에 가장 가까운 대표 profile을
`robust_policy_profiles/<policy>_runtime_profile.json`으로 복사한다.
`scenario_near_tie_profiles.csv/json`은 각 scenario의 multi-seed near-tie
runtime profile 후보를 최상위 sweep 산출물로 모은다. scenario별 winner 하나만
남기지 않고, score delta가 작은 StereoDistance 적용 가능 후보들을 함께 보존해
다음 자동탐색이나 실제 보정 비행 replay에서 비교할 수 있게 한다. 이 manifest도
offline 진단이며 runtime 입력은 raw stereo-center RDF와 measured disparity로
제한된다.
`scenario_near_tie_profile_transfer_replay.csv`와
`scenario_near_tie_profile_transfer_summary.csv/json`은 이 near-tie profile들을
모든 scenario/seed validation split에 다시 replay한다. 이를 통해 특정 scenario의
차순위 후보가 다른 오차 조건에서는 winner보다 더 전이성이 좋은지 확인한다.
PASS 기준은 다른 replay와 동일하게 raw 대비 validation RMSE 개선, target-window
sample 존재, 1852m 근방 error 10m 이하이다.
`scenario_near_tie_method_transfer_summary.csv/json`은 near-tie transfer 결과를
method 단위로 다시 묶어, 특정 seed/profile 하나가 아니라 같은 보정법 계열이
반복적으로 모든 transfer replay를 통과하는지 기록한다. method별 profile 수,
통과 profile 수, best profile, 평균/최악 validation RMSE와 target error를 함께
남기므로 다음 자동탐색에서 어떤 method 계열을 더 넓힐지 판단하는 근거가 된다.
`scenario_near_tie_best_profile.json`은 모든 transfer replay를 통과한 near-tie
후보 중 validation RMSE 평균, 1852m worst-case error, winner 대비 score delta
순으로 가장 좋은 profile을 기록한다. 해당 profile은
`scenario_near_tie_best_runtime_profile.json`으로 복사되어 단일 runtime profile
후보로 바로 검토할 수 있다. `scenario_near_tie_best_promotion_check.json`은 이
복사본이 StereoDistance loader로 읽히고 runtime truth 없이 validation sample에
적용되는지 검증한다. `deployment_strategy_recommendation`도 이 near-tie best
profile이 policy representative profile보다 transfer 성능이 좋고 promotion
check를 통과했을 때만 `single_near_tie_runtime_profile` 전략을 기록한다.
`robust_policy_profile_transfer_replay.csv`는 이 대표 profile 하나를 모든
scenario/seed validation split에 다시 replay해, 특정 synthetic 조건의 profile만
좋은지 아니면 다른 오차 조건에도 전이되는지 검증한다. 이 transfer replay가
실패하더라도 method 자체가 실패했다는 뜻은 아니며, 한 환경에서 fit한
profile을 다른 센서오차 조건에 universal하게 적용하면 위험하다는 뜻이다.
`robust_policy_profile_transfer_diagnostics.csv/json`은 이 실패를
policy/scenario 단위로 요약한다. `failure_reasons`에는
`no_validation_rmse_improvement`, `target_error_over_10m`,
`missing_target_window_validation_samples`, `replay_gate_failed` 같은 원인이
기록되고, 실패 scenario에는 `recommended_action=use_scenario_catalog_profile`이
기록된다.
`robust_method_summary.csv`, `scenario_profile_catalog.csv`,
`robust_policy_recommendations.csv`,
`robust_policy_representative_profiles.csv`는 method 이름만으로 판단하지 않도록
`registration_model`, `residual_model`, `regularization_model`,
`runtime_profile_family`, `selection_note`를 함께 기록한다. 예를 들어
`sim3_smoothed_binned_residual_disparity_bins11`은
`registration_model=sim3_estimated_to_truth`,
`residual_model=forward_disparity_lut`,
`regularization_model=lut_shrinkage_neighbor_smoothing+lut_bin_count_grid`,
`runtime_profile_family=estimated_to_truth_sim3`로 남는다. 이 필드는 outlier나
희소 target-window 조건에서 smoothed/bin-grid 후보가 왜 선택됐는지 해석하기
위한 offline 진단이며, runtime 입력 계약은 바꾸지 않는다.
`scenario_profile_catalog.csv/json`과 `scenario_profile_catalog/`는 각
scenario에서 자체 calibration data로 fit한 대표 profile을 모아 둔다. 이
catalog는 “현재 환경에서 보정 비행을 수행하고 해당 환경 profile을 선택한다”는
운용 방식을 검증하기 위한 산출물이다.
`scenario_profile_catalog_promotion_checks.csv/json`은 catalog의 각 profile이
StereoDistance loader로 읽히고 runtime truth 없이 validation sample에 적용되는지
검사한다. 이 추천 역시 calibration-time truth로 평가만 하며, runtime profile
입력은 raw stereo-center RDF와 measured disparity뿐이다.
`scenario_profile_catalog_transfer_*` 산출물은 catalog의 각 scenario profile을
다른 모든 scenario/seed validation split에 replay해 cross-scenario coverage를
행렬로 기록한다. 이 결과는 catalog를 반드시 scenario별로 나눠야 하는지,
또는 일부 profile이 여러 환경을 커버할 수 있는지 확인하는 진단용이며, 실패한
transfer profile을 runtime에 자동 승격하지 않는다.
`scenario_profile_catalog_minimal_set.csv/json`은 이 transfer matrix에서 모든
validation replay를 통과한 source-target 쌍만 사용해, 전체 scenario를 덮는
최소 후보 profile 집합을 greedy set cover로 산출한다. 이 결과는 catalog
운용을 줄이기 위한 offline 진단이며, runtime에서는 선택된 profile 중 하나를
보정 비행/배포 메타데이터 또는 운용자 설정으로 고른다. live PX4/RTK truth나
validation error를 runtime selection 입력으로 사용하지 않는다.
`pooled_scenario_profile/`과 `pooled_scenario_profile_summary.csv/json`은 여러
scenario의 calibration CSV를 하나로 합쳐 단일 profile을 다시 fitting한 결과를
기록한다. 목적은 scenario catalog 없이도 stress-invariant한 하나의 profile이
가능한지 확인하는 것이다. 이 pooled fitting도 calibration-time PX4/RTK truth를
사용하지만, promotion/replay/runtime 적용은 raw stereo-center RDF와 measured
disparity만 사용한다. pooled pipeline의 최종 추천 하나만 승격하지 않고,
validation RMSE, target-window error, 낮은 복잡도, policy 추천에서 나온 여러
runtime profile 후보를 다시 모든 scenario/seed validation split에 replay한 뒤
선택한다. `pooled_scenario_profile_transfer_replay.csv`에서 선택 후보의 모든
scenario/seed validation replay가 raw 대비 개선되고 1852m target error 10m
이하이며 promotion check를 통과할 때만 `single_pooled_runtime_profile` 후보가
된다. `pooled_scenario_profile_transfer_diagnostics.csv/json`은 선택된 pooled
profile 또는 모든 pooled 후보가 실패한 target scenario, 실패 원인
(`no_validation_rmse_improvement`, `target_error_over_10m` 등), 그리고
catalog/minimal-set profile로 전환해야 하는 scenario를 기록한다. 단순
inverse-depth ray scale 계열에는 identity ray scale로 shrink되는
ridge/Tikhonov 후보도 포함된다. 이 후보는 baseline/focal scale 오차처럼 전역
거리 scale에 가까운 문제에서 과한 Sim(3) 또는 residual LUT 보정을 피하는
안정성 기준 후보이며, output contract는 stereo-center RDF ray scale이다.
`deployment_strategy_recommendation.json/txt`는 sweep 결과를 배포 관점에서
해석한다. 단일 대표 profile이 모든 scenario/seed transfer replay를 통과하면
`single_runtime_profile`을 권장하고, near-tie 후보가 더 잘 전이되면
`single_near_tie_runtime_profile`을 권장한다. 둘 다 실패했지만 pooled profile이
모든 transfer replay와 promotion check를 통과하면
`single_pooled_runtime_profile`을 권장한다. 단일 profile 계열이 모두 깨지지만
scenario별 catalog가 cross-seed replay와 promotion check를 통과하면
`scenario_profile_catalog`를 권장한다.

StereoDistance는 런타임에 단일 profile JSON을 읽으므로, catalog에서 현재
환경 profile을 명시적으로 선택해 active profile로 복사한 뒤 사용한다.
선택 기준은 보정 비행/배포 메타데이터 또는 운용자 설정이어야 하며, runtime
truth로 고르면 안 된다.

2026-06-13 bbox/disparity gross outlier stress를 포함한 기본 6-scenario
sweep에서는 각 scenario별 catalog profile은 모두 promotion check를 통과했지만,
robust policy representative profile의 transfer replay는 18개 scenario/seed
조합 중 15개만 통과했다. 따라서 현재 추천 배포 전략은 단일 universal profile이
아니라 `scenario_profile_catalog`이다.

```bash
Simulation/.venv/bin/python Simulation/scripts/select_stereodistance_profile_from_catalog.py \
  --deployment-strategy-json Simulation/results/rtk_stereo_robustness_sweep/deployment_strategy_recommendation.json \
  --scenario nominal \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep/active_runtime_profile.json
```

생성된 `active_runtime_profile.json`을 기존 StereoDistance profile 경로
파라미터에 지정한다. selector는 cross-seed replay를 통과하지 못한 catalog
profile을 기본적으로 거부한다. `deployment_strategy_recommendation.json`이
`single_runtime_profile`, `single_near_tie_runtime_profile`,
`single_pooled_runtime_profile`이면 `--scenario` 없이도 검증된 단일 runtime
profile을 active profile로 복사할 수 있다. `scenario_profile_catalog`이면 현재
환경을 나타내는 `--scenario`를 반드시 명시해야 하며, 이 선택은 runtime truth가
아니라 보정 비행/배포 메타데이터 또는 운용자 설정에서 와야 한다.

minimal set 산출물이 있으면 scenario별 catalog 전체를 직접 운용하지 않고,
offline transfer matrix에서 검증된 더 작은 profile set으로 매핑할 수 있다.
예를 들어 6-scenario sweep에서 `nominal` profile이 nominal/yaw/disparity/
combined/bbox outlier 계열을 덮고, `intrinsic_scale_stress` profile이 intrinsic
scale mismatch 계열을 덮는다면 다음처럼 현재 배포 조건을 입력한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/select_stereodistance_profile_from_catalog.py \
  --deployment-strategy-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/deployment_strategy_recommendation.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/scenario_profile_catalog_minimal_set.json \
  --scenario combined_sensor_stress \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep_minimal_set/active_runtime_profile.json
```

이 경우 `--scenario`는 runtime truth가 아니라 배포/보정 조건 label이다.
selector는 minimal set의 `newly_covered_target_scenarios`와
`all_pass_target_scenarios`를 사용해 검증된 source profile을 고르고, 선택
summary에는 `requested_scenario`와 실제 복사된 source `scenario`를 함께 기록한다.

배포 조건 label을 확정하기 어렵고 새 보정 비행 CSV에 truth/raw RDF/disparity가
들어 있다면, runtime이 아니라 calibration-time에서 minimal set 후보들을 replay해
active profile을 고를 수 있다. 이 선택은 보정 비행 truth로만 수행되며, 복사된
runtime profile 자체는 계속 raw RDF/disparity만 입력으로 사용한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/select_stereodistance_profile_by_calibration_replay.py \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/scenario_profile_catalog_minimal_set.json \
  --input-csv Simulation/results/rtk_stereo_robustness_sweep_minimal_set/combined_sensor_stress/seed_20260614/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set/calibration_profile_selection \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep_minimal_set/active_runtime_profile.json
```

`calibration_replay_profile_candidates.csv`에는 후보별 validation RMSE, raw 대비
개선량, 1852m target-window count/error, PASS 여부가 기록된다. 또한
`registration_model`, `residual_model`, `regularization_model`,
`runtime_profile_family`, `selection_note`가 함께 기록되어 calibration replay로
선택된 후보가 Sim(3), affine, disparity polynomial, smoothed/bin-grid LUT 중
어떤 계열인지 추적할 수 있다. PASS 후보는 `corrected RMSE < raw RMSE`,
target-window sample 존재, `target_abs_mean_error_corrected_m <= 10m` 조건을
만족해야 한다. 후보 선택은 corrected RMSE만 보지 않고
`replay_selection_score = corrected RMSE + 0.30 * target_abs_error + 0.10 *
target_max_abs_error + 0.05 * catalog_target_max_abs_error + 0.08 *
model_complexity_estimate + stability_penalty`를 사용해 validation 성능,
1852m 근방 평균/최대 오차, catalog worst-case, 모델 복잡도, split 간
안정성을 함께 반영한다. 기본적으로
CSV에 `train`/`validation` split label이 있으면 두 split 모두에서 raw 대비
range RMSE가 개선되어야 하며, target-window sample이 있는 split에서는
1852m 평균 및 최대 오차 허용치도 통과해야 한다. split label이 없는 실제
CSV에서는 이 추가 gate를 건너뛰고 기존 단일 replay 기준을 유지한다.
CSV에는 `pass_reason`, `range_improvement_margin_m`, `target_error_margin_m`,
`target_max_error_margin_m`,
`stability_splits_evaluated`, `stability_min_range_improvement_m`,
`stability_max_target_abs_mean_error_m`, `stability_penalty_m`,
`model_complexity_estimate`, `replay_selection_score`, `replay_selection_rank`가
같이 기록된다. `--stability-splits ""`를 주면 split-stability gate를 끌 수
있고, 다른 split label을 쓰는 보정 비행 CSV라면 comma-separated 값으로
지정할 수 있다.
기본 실행은 `--minimal-set-json`이 있으면 deployment-ready minimal set 후보만
replay한다. 보정 비행에서 sensor-error regime을 더 적극적으로 식별하려면
`--candidate-source-mode all_catalog`를 추가해 catalog의 모든 validated profile을
후보로 자동 탐색할 수 있다. 이 모드도 선택 과정에서만 truth를 쓰며, 복사되는
runtime profile은 동일하게 raw RDF/disparity 입력만 사용한다.

보정 비행 truth를 아직 사용할 수 없거나, runtime에서 profile을 자동 선택해야
하는 경우에는 raw measurement guard를 별도 실험으로 돌린다. 이 selector의 목적은
truth와 estimated 오차를 직접 보는 것이 아니라, 현재 StereoDistance 입력의 raw
분포만으로 어떤 검증 profile family를 적용할지 고르는 것이다. 입력 계약은
stereo-center OpenCV optical RDF `raw_right_m,raw_down_m,raw_forward_m`,
`raw_range_m`, `measured_disparity_px`, 선택적으로 `left_u_px,left_v_px`이다.
PX4 NED truth, RDF truth, true disparity, residual/error 필드는 selection에서
금지하고 audit에서만 쓴다. 출력 profile은 기존 StereoDistance runtime profile과
같이 raw RDF/disparity만 소비한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/select_stereodistance_profile_by_runtime_measurement_guard.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_audit_all_catalog_transfer_risk \
  --candidate-source-mode all_catalog \
  --selection-policy transfer_risk_fallback \
  --audit-all-sweep-seeds
```

`nearest_centroid`는 raw feature centroid가 가장 가까운 scenario profile을 고른다.
`robust_family_fallback`은 같은 raw feature로 preliminary scenario를 고른 뒤,
nominal/yaw/bbox처럼 raw trajectory만으로 잘 분리되지 않는 저응력/자세/outlier
군을 검증된 `bbox_outlier_stress` profile로 접는다. 이 fallback은 runtime truth를
쓰지 않는다. 더 일반적인 `transfer_risk_fallback`은 hardcoded family 이름 대신
offline `scenario_profile_catalog_transfer_matrix.csv`에서 raw-feature preliminary
scenario와 상호 low-risk인 confusion cluster를 자동으로 만든다. 기본 low-risk
기준은 cross-seed transfer PASS, mean corrected range RMSE `<= 1.3m`,
max target abs mean error `<= 2.5m`이다. 그런 다음 cluster 안 source profile을
`mean RMSE + 0.10 * max target error + negative-improvement penalty + fail penalty`
점수로 비교해 더 안전한 source를 고른다. selection은 raw RDF/disparity와 transfer
matrix metadata만 쓰고, truth는 replay audit에서만 쓴다.

PASS 조건은 replay audit 기준으로 validation corrected range RMSE가 raw보다 낮고,
train/validation stability split이 있는 경우 둘 다 개선되며, 1852m target
window의 평균 및 최대 range error가 `±10m` 이내인 것이다. 2026-06-16
monotone-inverse-depth sweep에서 `all_catalog + transfer_risk_fallback`
leave-one-out audit는 18/18 PASS, validation corrected range RMSE mean/max
`1.089/1.676 m`, 1852m target abs mean/max `1.009/2.087 m`, target max abs
worst-case `9.681 m`를 기록했다. 같은 조건의 `nearest_centroid`는 17/18 PASS,
minimal-set centroid는 12/18 PASS였으므로 현재 자동 runtime profile 선택 후보는
`all_catalog + transfer_risk_fallback`이다.

`transfer_risk_fallback`의 low-risk threshold가 특정 sweep에 과하게 맞춰졌는지
확인하려면 threshold grid를 별도 감사한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/sweep_runtime_measurement_guard_thresholds.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_transfer_threshold_sweep \
  --candidate-source-mode all_catalog \
  --rmse-limits-m 0.9,1.1,1.3,1.5,1.8 \
  --target-limits-m 1.5,2.0,2.5,3.0,4.0
```

산출물은 `runtime_measurement_guard_threshold_sweep.csv`,
`runtime_measurement_guard_threshold_sweep_summary.txt/json`,
`runtime_measurement_guard_threshold_sweep_heatmap.png`이다. 2026-06-16
monotone-inverse-depth sweep에서는 25개 threshold 조합 중 9개가 18/18 PASS였고,
추천 threshold는 RMSE `1.3m`, target `2.5m`였다. 이 값은 18/18 PASS plateau의
가장 보수적인 시작점이며, 그보다 낮은 threshold에서는 nominal/yaw/bbox confusion
cluster가 충분히 묶이지 않아 17/18 PASS, target worst-case `10.055m`로 PASS
한계를 넘었다.

runtime-measurement selector 자체도 같은 조건에서 비교한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_runtime_measurement_guard_policies.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog_minimal_set.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_policy_audit \
  --candidate-source-modes minimal_set,all_catalog \
  --policies nearest_centroid,robust_family_fallback,transfer_risk_fallback,feature_vote_transfer_risk,knn_transfer_risk,centroid_knn_union_transfer_risk,centroid_knn_veto_transfer_risk \
  --transfer-low-risk-rmse-limit-m 1.3 \
  --transfer-low-risk-target-limit-m 2.5
```

산출물은 `runtime_measurement_guard_policy_audit.csv`,
`runtime_measurement_guard_policy_audit_summary.txt/json`,
`runtime_measurement_guard_policy_audit.png`이다. 같은 sweep에서
`all_catalog + transfer_risk_fallback`과 `all_catalog + robust_family_fallback`은
둘 다 18/18 PASS, target worst-case `9.681m`였지만, transfer-risk policy는
fallback count가 4로 hardcoded robust-family fallback의 9보다 작고, fallback
cluster를 transfer matrix에서 유도한다. `all_catalog + nearest_centroid`는
17/18 PASS, target worst-case `10.205m`였고, minimal-set 계열은 12/18 PASS였다.
따라서 현재 자동 runtime selector 추천은
`all_catalog + transfer_risk_fallback`이다.

추가 탐색 후보인 `feature_vote_transfer_risk`는 raw RDF/range/disparity/image-center
feature를 `range_geometry`, `disparity_depth`, `bearing_image` subset으로 나누어
각 subset의 centroid vote를 만든 뒤 transfer-risk cluster를 다시 평가한다. runtime
truth 입력은 여전히 쓰지 않는다. 2026-06-16
`runtime_measurement_guard_policy_audit_feature_vote` 결과에서는 all-catalog
기준 15/18 PASS, validation RMSE max `2.780m`로 기존 `transfer_risk_fallback`보다
나빴다. 실패는 `intrinsic_scale_stress` 3개 seed에서 nominal profile을 선택해
raw 대비 range RMSE 개선이 사라진 경우였다. 따라서 이 후보는 보존하지만 현재
추천에서 제외한다.

centroid 평균 대신 개별 보정 비행 run의 raw feature 이웃을 쓰는
`knn_transfer_risk`도 탐색한다. 이 방법은 raw stereo-center
RDF/disparity/image-center feature 공간에서 가장 가까운 `k`개 run의 source 후보를
만든 뒤 transfer-risk score로 최종 source를 고른다. runtime truth 입력은 쓰지
않는다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/sweep_runtime_measurement_guard_knn.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_knn_sweep \
  --candidate-source-mode all_catalog \
  --neighbor-counts 1,3,5,7,9 \
  --transfer-low-risk-rmse-limit-m 1.3 \
  --transfer-low-risk-target-limit-m 2.5
```

산출물은 `runtime_measurement_guard_knn_sweep.csv`,
`runtime_measurement_guard_knn_sweep_summary.txt/json`,
`runtime_measurement_guard_knn_sweep.png`이다. 2026-06-16 결과에서는 `k=5`가
최선으로 18/18 PASS, 1852m target max abs worst-case `9.681m`를 만족했지만
validation RMSE max가 `2.491m`로 기존 `transfer_risk_fallback`의 `1.676m`보다
컸다. 따라서 kNN 후보도 현재 추천에서 제외하고, 모든 run을 통과한 보조 후보로만
보존한다.

kNN과 centroid transfer-risk를 결합하는 hybrid도 감사한다. `centroid_knn_union`
방식은 centroid low-risk cluster와 kNN 후보를 단순 union으로 묶었고,
`centroid_knn_veto` 방식은 centroid preliminary와 양방향 low-risk인 kNN 후보만
추가했다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_runtime_measurement_guard_policies.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog_minimal_set.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_policy_audit_hybrid_veto \
  --candidate-source-modes all_catalog \
  --policies transfer_risk_fallback,knn_transfer_risk,centroid_knn_union_transfer_risk,centroid_knn_veto_transfer_risk \
  --transfer-low-risk-rmse-limit-m 1.3 \
  --transfer-low-risk-target-limit-m 2.5 \
  --knn-neighbor-count 5
```

2026-06-16 결과에서 `centroid_knn_union_transfer_risk`는 15/18 PASS로 나빠졌다.
`centroid_knn_veto_transfer_risk`는 18/18 PASS, validation RMSE max `1.676m`,
1852m target max abs worst-case `9.681m`로 기존 `transfer_risk_fallback`과
동일한 선택을 했다. 동률이면 복잡도 penalty를 적용해 더 단순한
`transfer_risk_fallback`을 추천한다.

보정 비행 truth로 만든 calibration-replay best source를 학습 라벨로 사용하는
runtime selector도 별도 탐색한다. 이 방식은 training label을 만들 때만 truth를
사용하고, leave-one-out 대상 선택에는 raw stereo-center RDF/disparity/image-center
feature만 사용한다. 따라서 실제 runtime profile 입력 계약은 바뀌지 않는다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_runtime_oracle_label_selector.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --calibration-replay-runs-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/profile_selection_strategy_audit_all_catalog/profile_selection_strategy_runs.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_oracle_label_selector_audit_nearest \
  --selection-policy nearest_label_centroid
```

산출물은 `runtime_oracle_label_selector_audit.csv`,
`runtime_oracle_label_selector_summary.txt/json`,
`runtime_oracle_label_selector_selection_counts.png`이다. 2026-06-16 결과에서
`nearest_label_centroid`는 16/18 PASS, calibration-replay label match 11/18,
validation RMSE max `2.710m`, 1852m target max abs worst-case `10.205m`였다.
`label_centroid_transfer_risk`는 14/18 PASS로 더 나빴다. 실패는
`bbox_outlier_stress` 일부 seed의 1852m target max 초과와 `intrinsic_scale_stress`
일부 seed의 raw 대비 RMSE 개선 실패였다. 즉 truth 기반 best-source label을 그대로
학습해도 현재 raw feature만으로는 label noise와 domain overlap을 충분히 분리하지
못하므로, 이 후보도 현재 추천에서 제외한다.

truth를 사용할 수 있는 offline benchmark에서는 runtime selector와 oracle replay
best의 차이도 감사한다. oracle은 같은 CSV에서 catalog profile 전체를 truth로
replay해 가장 좋은 PASS 후보를 고르는 기준이므로 runtime 입력으로 쓰면 안 된다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_runtime_measurement_guard_oracle_regret.py \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --guard-audit-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_audit_all_catalog_transfer_risk/runtime_measurement_guard_audit.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_oracle_regret_transfer_risk \
  --candidate-source-mode all_catalog
```

산출물은 `runtime_measurement_guard_oracle_regret.csv`,
`runtime_measurement_guard_oracle_candidates.csv`,
`runtime_measurement_guard_oracle_regret_summary.txt/json`,
`runtime_measurement_guard_oracle_regret.png`이다. 2026-06-16 sweep에서
`transfer_risk_fallback`은 18/18 PASS이고 oracle best와 8/18에서 같은 source를
골랐다. 평균 RMSE regret은 `-0.036m`로 selector가 oracle-score 후보보다 RMSE만은
약간 낮은 경우가 있었고, 1852m target abs regret mean/max는 `0.146/0.613m`,
target max regret mean/max는 `0.138/0.612m`였다. 따라서 남은 개선 여지는 주로
nominal/yaw/bbox cluster 안에서 target-window error를 1m 미만 더 줄이는
tie-breaker 탐색이며, 현재 PASS 목표에는 충분한 margin이 있다.

transfer-risk score 안에서 target error 가중치만 바꾸는 단순 tie-breaker도
별도로 감사한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/sweep_runtime_measurement_guard_target_weight.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_target_weight_sweep \
  --candidate-source-mode all_catalog \
  --weights 0.0,0.05,0.1,0.2,0.4,0.8 \
  --transfer-low-risk-rmse-limit-m 1.3 \
  --transfer-low-risk-target-limit-m 2.5
```

산출물은 `runtime_measurement_guard_target_weight_sweep.csv`,
`runtime_measurement_guard_target_weight_sweep_summary.txt/json`,
`runtime_measurement_guard_target_weight_sweep.png`이다. 2026-06-16 sweep에서는
weight `0.0`부터 `0.8`까지 모두 18/18 PASS, target abs regret max `0.613m`,
target max regret max `0.612m`, oracle source match 8/18로 동일했다. 따라서 현재
남은 regret은 transfer-risk score의 target 가중치만으로는 줄지 않으며, 추가 raw
feature 또는 calibration-time replay 결합 selector가 다음 탐색 대상이다.

truth 사용 가능 여부에 따라 selector 운용 모드를 분리한다. runtime truth가 없으면
`transfer_risk_fallback` guard를 쓰고, 보정 비행 CSV에 PX4/local-position truth가
있으면 calibration-time replay selector를 쓴다. calibration replay는 profile을
고를 때만 truth를 사용하며, export된 runtime profile은 여전히 raw RDF/disparity만
소비한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_selector_operating_modes.py \
  --runtime-guard-audit-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_audit_all_catalog_transfer_risk/runtime_measurement_guard_audit.csv \
  --calibration-replay-runs-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/profile_selection_strategy_audit_all_catalog/profile_selection_strategy_runs.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/selector_operating_mode_audit
```

산출물은 `selector_operating_mode_comparison.csv`,
`selector_operating_mode_summary.csv`, `selector_operating_mode_recommendation.json`,
`selector_operating_mode_summary.txt`, `selector_operating_mode_comparison.png`이다.
2026-06-16 sweep에서는 runtime guard와 calibration replay가 모두 18/18 PASS였다.
runtime guard의 validation range RMSE mean은 `1.089m`, calibration replay는
`1.125m`였고, 1852m target abs mean error는 runtime guard `1.009m`,
calibration replay `0.863m`였다. calibration replay는 9/18 run에서 target abs
error를 더 낮췄다. 따라서 운용 기준은 다음과 같다: truth가 있는 보정/검증 단계는
`calibration_replay_all_catalog`, truth가 없는 runtime 자동 선택은
`runtime_transfer_risk_guard`.

위 기준을 active runtime profile export에 바로 적용하려면 operating-mode exporter를
사용한다. `--mode auto`는 CSV에 truth field가 있으면 calibration replay를 쓰고,
truth field가 없으면 runtime measurement guard를 사용한다. runtime guard mode는
truth가 없으므로 replay RMSE/target error를 계산하지 않고, raw feature와 transfer
matrix metadata만으로 profile을 선택해 복사한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/export_stereodistance_profile_by_operating_mode.py \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --input-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/combined_sensor_stress/seed_20260613/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/operating_mode_export_active \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/operating_mode_export_active/active_runtime_profile.json \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --mode auto \
  --calibration-candidate-source-mode all_catalog \
  --runtime-candidate-source-mode all_catalog \
  --runtime-selection-policy transfer_risk_fallback
```

산출물은 `operating_mode_profile_export_summary.txt/json`과
`active_runtime_profile.json`이다. summary의 `selected_operating_mode`가
`calibration_replay`이면 보정 비행 truth로 profile을 선택했다는 뜻이고,
`runtime_guard`이면 truth 없이 raw RDF/disparity와 transfer matrix metadata만으로
profile을 선택했다는 뜻이다. 두 경우 모두 `active_runtime_profile.json`은
StereoDistance runtime에서 raw RDF/disparity만 입력으로 사용한다.

intrinsic-scale gap을 runtime-safe product-ratio로 감지한 뒤 catalog에 있는 같은
family profile을 바로 export하려면 runtime selection policy를
`intrinsic_scale_gap_transfer_risk`로 지정한다. 이 경로는 truth field가 CSV에
있더라도 `--mode runtime_guard`를 지정하면 truth를 선택 입력으로 쓰지 않는다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/export_stereodistance_profile_by_operating_mode.py \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --input-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/intrinsic_scale_stress/seed_20260613/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/operating_mode_export_intrinsic_scale_guard_smoke \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/operating_mode_export_intrinsic_scale_guard_smoke/active_runtime_profile.json \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --mode runtime_guard \
  --runtime-candidate-source-mode all_catalog \
  --runtime-selection-policy intrinsic_scale_gap_transfer_risk
```

2026-06-16 smoke 결과에서는 product-ratio p10 `1.003814`가 threshold `1.0037`을
넘어 `intrinsic_scale_stress` profile을 선택했고,
`intrinsic_scale_product_ratio_guard_selects_scale_profile`로 기록됐다. Export된
profile은 promotion check PASS, StereoDistance node loader PASS,
runtime_truth_used `false`였으며, 같은 validation split replay에서 range RMSE가
raw `1.872m`에서 corrected `0.888m`로 감소했다. 따라서 catalog에 같은
intrinsic-scale family profile이 존재할 때는 runtime truth 없이도 적용 가능한
active profile export 경로가 있다. 반대로 LOSO처럼 해당 source profile이 없으면
감지는 되지만 보정 profile이 없어 calibration-time profile export가 필요하다.

새 intrinsic-scale regime을 보정 비행에서 처음 만났고 CSV에 PX4/RTK truth가
있다면, 같은 CSV로 새 runtime profile을 fitting/export할 수 있다. 이 wrapper는
먼저 runtime-safe product-ratio p10 guard를 확인하고, 감지되면
`rtk_stereo_calibration_pipeline.py --mode csv`를 실행해 validation 기준으로
추천 method를 고른 뒤 active runtime profile로 복사한다. Truth는 이 calibration
fit 단계에서만 쓰이고, export된 profile은 runtime에서 raw RDF/disparity만 쓴다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/export_intrinsic_scale_profile_from_calibration.py \
  --input-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/intrinsic_scale_stress/seed_20260613/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/intrinsic_scale_calibration_export_smoke \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/intrinsic_scale_calibration_export_smoke/active_runtime_profile.json \
  --target-range-m 1852 \
  --target-window-m 30 \
  --seed 20260613
```

2026-06-16 smoke 결과에서는 product-ratio p10 `1.003814`로 intrinsic gap을
감지했고, 새 calibration fit의 추천 method는 `global_inverse_depth_linear`였다.
Promotion check는 PASS, StereoDistance node loader는 PASS, runtime_truth_used는
`false`였다. 같은 validation split replay에서 range RMSE는 raw `1.872m`에서
corrected `0.975m`로 감소했고 1852m target abs mean error는 `0.763m`였다. 이
경로가 현재 LOSO intrinsic-scale 실패에 대한 실제 해결 후보이며, 다음 검증은
여러 intrinsic-scale 보정 비행 seed에서 export profile의 cross-seed 전이를 보는
것이다.

같은 family 안에서 전이되는지 확인하려면 active profile transfer audit를
`--target-scenarios intrinsic_scale_stress`로 제한해 실행한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_active_profile_transfer.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --profile-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/intrinsic_scale_calibration_export_smoke/active_runtime_profile.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/intrinsic_scale_calibration_export_smoke/intrinsic_family_transfer \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10 \
  --target-scenarios intrinsic_scale_stress
```

2026-06-16 결과는 intrinsic-scale family 3/3 PASS였다. Validation RMSE corrected
mean/max는 `0.950/1.024m`, 최소 RMSE 개선은 `0.897m`, 1852m target abs mean
error worst-case는 `1.436m`, target max abs worst-case는 `2.410m`였다. 같은
profile을 전체 sweep에 고정 적용하면 4/18 PASS에 그치므로 universal profile은
아니지만, product-ratio로 감지한 intrinsic-scale family에 대해서는 보정 비행
1개 seed에서 export한 profile이 다른 seed까지 전이됐다.

보정 비행 source seed를 하나로 고정하지 않고 family 내부 모든 seed를 번갈아
calibration source로 삼아 자동 export/replay하려면 family calibration export
audit를 실행한다. 이 audit는 각 source seed의 CSV로 runtime profile을 새로 fit한
뒤 같은 scenario family의 모든 target seed에 replay하고, source seed를 제외한
heldout 전이 성능을 따로 집계한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_family_calibration_profile_exports.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/intrinsic_scale_family_calibration_export_audit \
  --target-scenarios intrinsic_scale_stress \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10 \
  --seed 20260613
```

출력은 `family_calibration_profile_exports.csv`,
`family_calibration_profile_transfer.csv`,
`family_calibration_profile_scenario_summary.csv/json`,
`family_calibration_profile_export_summary.json/txt/png`,
`exports/*/active_runtime_profile.json`, `per_run_predictions/*`이다.
2026-06-16 결과는 export 3개, family replay 9/9 PASS, heldout replay 6/6 PASS였다.
Validation RMSE corrected mean/max는 `0.893/1.024m`, 1852m target max abs
worst-case는 `2.410m`였다. 따라서 intrinsic-scale gap은 “runtime-safe 감지 →
family-local calibration export → heldout family replay” 루프를 통과한 첫 family다.

같은 audit를 특정 family로 제한하지 않고 전체 scenario에 대해 실행하면, 각 scenario
family별로 보정 비행 seed를 runtime profile로 export하고 같은 family의 heldout seed에
전이되는지 확인할 수 있다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_family_calibration_profile_exports.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/all_family_calibration_export_audit \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10 \
  --seed 20260613
```

2026-06-16 전체 family 결과는 export 18개, family replay 54/54 PASS, heldout replay
36/36 PASS였다. Validation RMSE corrected mean/max는 `1.167/1.877m`, 1852m target
max abs worst-case는 `9.882m`였다. Scenario별 heldout은 모두 PASS였지만
`bbox_outlier_stress`가 target max `9.882m`로 pass limit `10m`에 가장 가까웠다.
따라서 다음 탐색 우선순위는 bbox/outlier 계열에서 target-window 최대오차 margin을
늘리는 robust weighting 또는 outlier-aware profile selection이다. 이 audit의 runtime
profile은 StereoDistance 적용 시 raw stereo-center RDF/disparity만 소비하며,
truth는 calibration-time fitting과 offline validation/replay metric에만 사용된다.

family calibration export audit는 `--export-policies`로 calibration fit 내부의 여러
policy profile을 동시에 export/replay할 수 있다. `bbox_outlier_stress`의 pass margin을
확인하려면 다음처럼 실행한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_family_calibration_profile_exports.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/bbox_policy_family_calibration_export_audit \
  --target-scenarios bbox_outlier_stress \
  --export-policies final_recommendation,balanced,target_specialist,target_max_stable,lowest_validation_rmse,axis_balanced,lowest_complexity_pass \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10 \
  --seed 20260613
```

2026-06-16 결과에서 `target_specialist`는 1852m target max worst-case를 `9.438m`까지
낮췄지만 validation RMSE max가 `2.788m`이고 최소 RMSE 개선이 `0.016m`뿐이라 모델
안정성이 약했다. 자동 추천 기준은 heldout 전부 통과와 최소 validation RMSE 개선
`0.5m` 이상을 먼저 요구한 뒤 target max와 validation RMSE max를 비교한다. 이 기준의
추천 policy는 `target_max_stable`였고, heldout 6/6 PASS, validation RMSE corrected
max `1.553m`, 최소 RMSE 개선 `1.305m`, 1852m target max abs worst-case `9.681m`였다.
즉 기존 balanced/final 추천의 `9.882m`와 `lowest_validation_rmse`의 `9.787m`보다
target-window 최대오차 margin이 좋아졌다. 다만 bbox/outlier family는 여전히 가장
빡빡한 조건이다.

같은 policy sweep을 전체 scenario family에 적용하면 scenario별 policy 추천도 함께
저장된다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_family_calibration_profile_exports.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/all_policy_family_calibration_export_audit \
  --export-policies final_recommendation,balanced,target_specialist,target_max_stable,lowest_validation_rmse,axis_balanced,lowest_complexity_pass \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10 \
  --seed 20260613
```

출력은 기존 family export 산출물에 더해
`family_calibration_profile_policy_summary.csv/json`,
`family_calibration_profile_scenario_policy_summary.csv/json`,
`family_calibration_profile_scenario_policy_recommendations.csv/json`을 포함한다. 2026-06-16
결과에서 global policy 추천은 `target_max_stable`였고 heldout 36/36 PASS,
validation RMSE corrected max `2.432m`, 최소 RMSE 개선 `0.514m`, 1852m target max abs
worst-case `9.681m`였다. Scenario별 추천은 `bbox_outlier_stress=target_max_stable`,
`combined_sensor_stress=balanced`, `disparity_noise_stress=lowest_complexity_pass`,
`intrinsic_scale_stress=balanced`, `nominal=balanced`,
`yaw_pitch_stress=lowest_complexity_pass`였다. 모든 scenario 추천은 heldout 6/6 PASS를
만족했다.

scenario별 추천 policy를 실제 StereoDistance runtime selection/export에서 쓸 수
있도록 catalog로 묶으려면 policy profile catalog export를 실행한다. 이 단계는
calibration-time audit 결과를 읽어 각 scenario에서 가장 안정적인 source seed의
`active_runtime_profile.json`을 복사하고, 기존 `scenario_profile_catalog`와 같은
manifest 형식으로 저장한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/export_policy_profile_catalog_from_family_audit.py \
  --base-catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --family-audit-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/all_policy_family_calibration_export_audit \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/policy_family_profile_catalog \
  --output-catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/policy_family_profile_catalog/manifest.json
```

2026-06-16 결과는 profile 6/6 PASS, 선택 source profile 6/6 PASS였다. 선택된 source
profile 기준 worst-case target max abs error는 `8.951m`였고, 가장 약한
`bbox_outlier_stress` profile은 `target_max_stable` policy의
`sim3_local_linear_residual_disparity_sigma020_support7`였다. 기존 catalog selector와
호환되므로 다음처럼 scenario 기반 active profile export에 바로 사용할 수 있다.

```bash
Simulation/.venv/bin/python Simulation/scripts/select_stereodistance_profile_from_catalog.py \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/policy_family_profile_catalog/manifest.json \
  --scenario bbox_outlier_stress \
  --output-profile /tmp/sasl_bbox_policy_active_profile.json \
  --selection-summary-json /tmp/sasl_bbox_policy_selection_summary.json
```

선택된 bbox policy catalog profile을 `bbox_outlier_stress` family에 replay한 결과는
3/3 PASS, validation RMSE corrected max `1.553m`, 최소 RMSE 개선 `1.305m`,
1852m target max abs worst-case `8.951m`였다.

policy catalog를 truth-free runtime measurement guard에 직접 넣어 전체 sweep을 감사할
수도 있다. 이 실행은 profile 선택에 truth를 쓰지 않고 raw RDF/disparity/image-center
통계만 사용한다. 단, replay metric 계산에는 offline 검증용 truth를 사용한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/select_stereodistance_profile_by_runtime_measurement_guard.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/policy_family_profile_catalog/manifest.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_audit_policy_catalog_transfer_risk \
  --candidate-source-mode all_catalog \
  --selection-policy transfer_risk_fallback \
  --audit-all-sweep-seeds \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

2026-06-16 결과는 18/18 PASS였다. Validation RMSE corrected mean/max는
`1.154/1.806m`로 기존 runtime guard catalog의 `1.089/1.676m`보다 약간 커졌지만,
1852m target max abs worst-case는 `9.681m`에서 `8.951m`로 낮아졌다. 즉 policy
catalog runtime guard는 평균 RMSE보다 1852m 최대오차 margin을 우선할 때 더 적합한
truth-free 배포 후보이다.

같은 policy catalog에서 `robust_family_fallback`을 사용하면 transfer matrix 없이도
동일하게 18/18 PASS, validation RMSE corrected mean/max `1.154/1.806m`, 1852m target
max abs worst-case `8.951m`를 얻었다. 선택 분포는 `bbox_outlier_stress`,
`nominal`, `yaw_pitch_stress`를 bbox-robust profile로 접고, 나머지 stress family는
각 자기 scenario profile을 선택한다. 같은 target max 성능이면 matrix 의존이 없는
`policy_catalog+robust_family_fallback`이 현재 더 단순한 truth-free runtime 추천이다.

같은 selector를 strict LOSO 조건으로 감사할 수도 있다. 여기서는 target scenario
family profile을 library에서 제거하므로, 새 stress regime이 기존 catalog로 대체
가능한지 확인한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/select_stereodistance_profile_by_runtime_measurement_guard.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/policy_family_profile_catalog/manifest.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_audit_policy_catalog_robust_family_loso \
  --candidate-source-mode all_catalog \
  --selection-policy robust_family_fallback \
  --audit-all-sweep-seeds \
  --exclude-target-scenario-from-library \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

현재 저장된 strict LOSO 결과는 14/18 PASS, validation RMSE corrected mean/max
`1.554/2.375m`, 1852m target abs mean/max `1.814/4.065m`, target-window max abs
worst-case `7.377m`이다. validation-only 진단 기준으로는 17/18 PASS다.
`bbox_outlier_stress` profile이 제외된 경우에는 `disparity_noise_stress` profile을
fallback으로 사용해 1852m 초과 실패를 제거했다. 남은 엄격 PASS 실패는
`intrinsic_scale_stress` 3개 run의 train-side improvement 부족과
`bbox_outlier_stress` 1개 run의 train-side target guard 실패다. validation-only에서
실제로 남는 실패는 `intrinsic_scale_stress/seed_20260613` 하나다. 즉 runtime 배포
후보는 full catalog 18/18 PASS 결과를 기준으로 유지하되, LOSO 탐색은
intrinsic-scale family coverage와 train-side guard 안정화가 다음 과제다.

intrinsic-scale gap 후보로 stereo depth 식 `Z=fB/d`에 기반한
`disparity_product_ray_scale`과 `disparity_product_ray_scale_sim3`도 추가했다. 이
후보는 train truth에서 calibrated `d_measured * truth_forward` product를 fitting하고,
runtime에서는 measured disparity와 raw stereo-center RDF만 사용해 raw ray forward를
`product / disparity`로 재스케일한다. `disparity_product_ray_scale_sim3`는 그 뒤
Umeyama Sim(3)를 적용하므로 StereoDistance runtime profile로 export/apply 가능하다.
좌표 계약은 입력/출력 모두 stereo-center RDF이며, public topic 변환은 기존
`FRD_range(forward,right,down,range)` 계약을 유지한다.

focused intrinsic-scale smoke sweep
`/tmp/sasl_disparity_product_intrinsic_sweep_smoke`에서는
`disparity_product_ray_scale_sim3`가 raw validation RMSE `1.785m`를 `0.998m`로
줄였고 1852m target max abs error는 `1.862m`였다. 그러나 기존
`target_weighted_global_inverse_depth_linear`는 validation RMSE `0.633m`, target max
`1.260m`, `sim3_local_linear_residual_disparity_sigma020_support5`는 validation RMSE
`0.627m`, target max `0.877m`로 더 좋았다. 따라서 product-ray 후보는 현재 추천을
대체하지 않고, reference-based intrinsic-scale diagnostic baseline으로 유지한다.

product-ray 후보를 전 scenario에서 빠르게 확인하려면 focused sweep과 후보 요약을
별도로 실행한다. 이 sweep은 메인 3-seed robustness 결과를 대체하지 않고, 새 후보의
상대 위치를 확인하는 탐색용 산출물이다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_robustness_sweep.py \
  --output-dir Simulation/results/rtk_stereo_product_ray_focused_sweep \
  --scenarios nominal,yaw_pitch_stress,intrinsic_scale_stress,disparity_noise_stress,bbox_outlier_stress,combined_sensor_stress \
  --seeds 20260613 \
  --sample-count 160 \
  --target-range-m 1852 \
  --target-window-m 30 \
  --synthetic-target-oversample-count 12 \
  --pooled-candidate-scan-limit 20

Simulation/.venv/bin/python Simulation/scripts/summarize_method_candidate_across_sweep.py \
  --sweep-dir Simulation/results/rtk_stereo_product_ray_focused_sweep \
  --output-dir Simulation/results/rtk_stereo_product_ray_focused_sweep/disparity_product_ray_candidate_summary \
  --method-substring disparity_product_ray_scale \
  --target-error-limit-m 10
```

현재 focused sweep 요약에서 product-ray 후보 row는 14개이며 target max 10m 기준은
13/14가 통과했다. 하지만 validation RMSE mean/max는 `1.775/3.237m`, target max
worst-case는 `11.826m`, 최소 RMSE 개선은 `-0.179m`였다. 개별 scenario의
`disparity_product_ray_scale_sim3`는 raw보다 좋아졌지만 rank는 대체로 180위권이고,
best method는 여전히 inverse-depth 또는 residual-disparity 계열이다. 이 결과는
intrinsic-scale gap이 단순 `fB` product 보정 하나로 해결되기보다, regime detection과
scenario-specific profile 선택이 더 중요하다는 증거로 남긴다.

raw RDF/disparity-only 조건에서 bundle-adjustment식 카메라 파라미터 보정에 가장
가까운 후보로 `joint_disparity_ray_rotation_nls` 계열도 포함한다. 이 후보는 보정
비행 truth로 `d_corr = a*d_meas + b`와 stereo-center RDF rotation을 scipy
`least_squares`로 동시에 fitting한다. runtime에서는 `raw RDF` ray를
`d_meas/d_corr`로 scale하고 고정 RDF rotation만 적용하므로 PX4/RTK truth, true
disparity, image truth를 사용하지 않는다. Huber loss와 1852m target-range weighting
variant도 함께 생성된다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_robustness_sweep.py \
  --output-dir Simulation/results/rtk_stereo_joint_disparity_ray_rotation_bbox_probe \
  --scenarios bbox_outlier_stress \
  --seeds 20260613,20260614,20260615 \
  --sample-count 520 \
  --target-range-m 1852 \
  --target-window-m 30 \
  --synthetic-target-oversample-count 12 \
  --pooled-candidate-scan-limit 20
```

현재 bbox-only 3-seed probe에서 `huber_joint_disparity_ray_rotation_nls`는
multi-seed representative/catalog profile로 선택됐고 promotion check도 PASS했다.
cross-seed replay는 3/3 PASS, validation RMSE mean/max `1.230/1.540m`,
1852m target max abs worst-case `9.935m`였다. seed별 method aggregate에서는
`huber_joint_disparity_ray_rotation_nls`가 validation RMSE mean/max
`1.223/1.533m`, target abs mean max `1.990m`를 기록했다. 다만 같은 bbox near-tie
transfer에서 `sim3_binned_residual_disparity` seed 20260615 profile은 target max
`9.214m`였고, 기존 full policy catalog runtime 추천은 `8.951m`이므로 이 후보는
최종 추천을 대체하지 않는다. reference-based camera-geometry NLS baseline으로
보존하고, 향후 full 좌/우 feature correspondence가 저장되면 실제 reprojection
bundle-adjustment 후보로 확장한다.

StereoDistance 런타임에 더 직접 적용 가능한 camera-geometry 후보로
`left_disparity_camera_geometry_sim3` 계열도 추가했다. 이 후보의 목적은 raw RDF를
사후 보정하기 전에 `left_u_px`, `left_v_px`, `measured_disparity_px`에서
stereo-center RDF(`right,down,forward`)를 다시 만들고, 보정 비행 truth로 추정한
고정 파라미터만 런타임에 쓰는 것이다. 입력 가정은 좌측 bbox/feature 중심과 measured
disparity가 런타임에 존재한다는 것이며, 출력 contract는 기존과 동일한 internal
RDF 및 public StereoDistance FRD range topic이다. 학습에서는 `d_corr=a*d+b`,
left image center offset, global Sim(3)을 fit하지만, 런타임에서는 PX4/RTK truth,
true disparity, true image feature location을 사용하지 않는다. PASS 조건은 기존과
같이 validation RMSE 개선, 1852m target-window error ±10m, cross-seed/runtime
promotion check 통과 여부로 판단한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_robustness_sweep.py \
  --output-dir Simulation/results/rtk_stereo_left_disparity_camera_geometry_probe \
  --scenarios bbox_outlier_stress \
  --seeds 20260613,20260614,20260615 \
  --sample-count 520 \
  --target-range-m 1852 \
  --target-window-m 30 \
  --synthetic-target-oversample-count 12 \
  --pooled-candidate-scan-limit 80
```

현재 bbox-only 3-seed aggregate에서 target-weighted Huber left-disparity geometry
후보는 validation RMSE mean `1.219m`로 5위였고, 1852m target abs mean worst-case
`2.069m`로 충분히 PASS 범위에 있었다. 그러나 pooled profile validation에서는
65위(`1.634m`)였고, pooled/cross-seed replay 후보에는 선택되지 않았다. 같은 probe의
pooled transfer best는 `sim3_binned_residual_disparity_bins11`로 replay RMSE
mean/max `1.209/1.517m`, target abs worst-case `2.066m`였다. 따라서 이 후보는
StereoDistance에 적용 가능한 reference-based geometry baseline으로 유지하되, 현재
최종 추천을 대체하지 않는다.

reprojection-error 기반 stereo calibration 연구에서는 2D correspondence에 좋은
parameter set과 3D reconstruction에 좋은 parameter set이 달라질 수 있음을 지적한다.
이를 반영해 `left_reprojection_geometry_sim3` 계열도 추가했다. 이 후보는 보정
비행 truth RDF를 left image center/disparity 관측으로 투영했을 때의 pixel/disparity
residual을 Huber NLS로 줄이고, 그 fitted `d_corr=a*d+b`, left-u/v offset으로
runtime 재삼각측량을 수행한 뒤 Umeyama Sim(3)를 적용한다. runtime 입력은
`left_u_px`, `left_v_px`, `raw_disparity_px`, raw RDF뿐이며 PX4/RTK truth와
true image location은 calibration/replay 단계에서만 사용한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/rtk_stereo_calibration_pipeline.py \
  --output-dir Simulation/results/rtk_stereo_left_reprojection_geometry_smoke \
  --seed 20260613 \
  --sample-count 520 \
  --target-range-m 1852 \
  --target-window-m 30 \
  --synthetic-target-oversample-count 12 \
  --bbox-outlier-rate 0.08 \
  --bbox-outlier-px 2.5 \
  --disparity-outlier-rate 0.06 \
  --disparity-outlier-px 0.45
```

2026-06-16 smoke 결과에서 `huber_left_reprojection_geometry_sim3`는 validation
RMSE `1.563m`, 1852m target max `10.015m`였고,
`target_weighted_huber_left_reprojection_geometry_sim3_sigma160`은 validation RMSE
`1.553m`, 1852m target max `9.964m`였다. target-weighted variant는 PASS 범위지만
기존 `target_weighted_huber_left_disparity_camera_geometry_sim3_sigma160`
(`1.530m`, target max `9.797m`) 및 현재 runtime 추천보다 약하다. 따라서
reprojection 후보는 reference-based 탐색 후보로 보존하되 promotion하지 않는다.

추가 탐색으로 같은 left-disparity geometry 출력에 raw-forward quadratic residual LUT를
붙인 variant도 넣었다. 목적은 left-center 재삼각측량 후 남는 장거리 range residual을
raw_forward_m 기반으로 줄이는 것이다. 입력/출력/금지 입력 contract는 위 후보와 같고,
runtime 입력에 `raw_forward_m` residual key만 추가된다. 단일 bbox stress smoke
(`Simulation/results/rtk_stereo_left_geometry_residual_smoke`)에서는 target-weighted
variant가 31위, validation RMSE `1.529m`, target max `9.807m`로 원 left-disparity
geometry보다 좋아지지 않았다. 이 조합은 PASS 범위에는 있으나 현재 추천 후보가 아니다.

outlier가 섞인 bbox/disparity 조건에는 reference-based robust point-set registration
후보도 포함한다. `truncated_ls_sim3`와 `truncated_ls_range_weighted_sim3`는
Umeyama Sim(3) refit에 truncated least-squares inlier gate를 붙인 후보이며,
TEASER/robust registration 계열의 truncated least-squares 관점과 장거리 stereo에서
disparity error보다 depth/range error를 직접 다루는 관점을 반영한다. 관련 참고는
Yang et al.의 TEASER robust registration 계열, Ling and Shen의 stereo extrinsic
calibration, Chuah et al.의 long-range stereo bias/depth loss 문제정의다.

```bash
Simulation/.venv/bin/python Simulation/scripts/rtk_stereo_calibration_pipeline.py \
  --output-dir Simulation/results/rtk_stereo_tls_candidate_smoke \
  --seed 20260613 \
  --sample-count 160 \
  --target-range-m 1852 \
  --target-window-m 30 \
  --synthetic-target-oversample-count 12 \
  --bbox-outlier-rate 0.08 \
  --bbox-outlier-px 2.5 \
  --disparity-outlier-rate 0.06 \
  --disparity-outlier-px 0.45
```

현재 bbox outlier smoke에서 `truncated_ls_sim3`는 validation range RMSE `1.115m`,
1852m target max abs error `2.907m`, inlier fraction `0.922`를 기록했다.
`truncated_ls_range_weighted_sim3`는 validation range RMSE `1.387m`, target max
`2.620m`이다. 둘 다 Sim(3) 계열 robust baseline으로 유효하지만, 같은 smoke의 최종
추천은 validation RMSE `0.819m`, target abs mean `0.027m`인
`ridge_global_inverse_depth_linear_lambda1p0`였으므로 현재 전체 추천은 바꾸지 않는다.

6개 scenario single-seed focused sweep에서는 `truncated_ls` 후보 row 14개가 모두
1852m target max 10m 조건을 통과했다. validation range RMSE mean/max는
`1.374/2.057m`, target max abs worst-case는 `9.912m`, 최소 RMSE 개선은 `0.231m`였다.
따라서 TLS 계열은 reference-based robust Sim(3) fallback 후보로 남기되, 현재
best/recommended family인 inverse-depth 및 residual-disparity 계열을 대체하지 않는다.
요약 산출물은
`Simulation/results/rtk_stereo_tls_candidate_focused_sweep/truncated_ls_candidate_summary/`에
저장된다.

여러 방법 계열을 한 번에 비교하려면 family-level 요약을 사용한다. 이 스크립트는
기존 `method_comparison.csv`만 읽고 재학습하지 않으며, 각 family의 target-window
PASS 비율, validation RMSE, 1852m target worst-case, runtime profile family를
기록한다. family ranking은 absolute pass count가 아니라 all-pass 여부, pass fraction,
target worst-case, validation RMSE 순서로 정렬한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/summarize_method_families_across_sweep.py \
  --sweep-dir Simulation/results/rtk_stereo_tls_candidate_focused_sweep \
  --output-dir Simulation/results/rtk_stereo_tls_candidate_focused_sweep/method_family_summary \
  --target-error-limit-m 10

Simulation/.venv/bin/python Simulation/scripts/summarize_method_families_across_sweep.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/method_family_summary \
  --target-error-limit-m 10
```

현재 TLS focused sweep family 요약에서는 모든 affine row를 그대로 평가해도
`affine_runtime`이 224/224 all-pass, validation RMSE mean/max `1.124/1.978m`,
1852m target worst-case `9.469m`로 가장 보수적인 family 후보였다. 대표 profile
`affine_binned_residual_raw_forward_bins11`은 StereoDistance promotion check도 PASS했고
runtime truth를 사용하지 않았다. 메인 monotone inverse-depth runtime sweep에서 모든
affine row를 그대로 세면 631/640 pass, target worst-case `11.210m`로 all-pass는
아니다. 그러나 scenario/seed별로 affine family 내부 최선 후보를 offline calibration
selector가 고를 수 있다고 보면 `affine_runtime`은 20/20 all-pass, target worst-case
`9.308m`다. 이는 affine RDF axis-wise gain/bias + residual LUT 계열이 다음 자동 탐색
방향이라는 뜻이다. 반대로 family row 전체 기준에서는 `residual_disparity`가
1107/1120 pass, pass fraction `0.988`, target worst-case `11.957m`로 1위라서
단일 family 전체를 무조건 promote할 수는 없다. 따라서 family-level 탐색은 다음 후보
방향을 찾는 진단이며, 현재 full policy catalog runtime 추천
`policy_catalog+robust_family_fallback`을 대체하지 않는다.

affine family 내부에서 실제 runtime profile을 자동 선택하고 export하려면 다음 audit를
사용한다. 이 단계는 calibration-time validation metric으로 family 내부 best profile을
고르지만, export된 JSON은 일반 StereoDistance runtime profile이며 truth를 입력으로
사용하지 않는다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_method_family_profile_selection.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/affine_family_profile_selection_audit \
  --family-name affine_runtime \
  --method-substring affine \
  --target-error-limit-m 10 \
  --export-selected-profiles
```

현재 결과는 20/20 selected PASS, validation RMSE mean/max `1.307/2.114m`,
1852m target max abs worst-case `9.308m`이다. `selected_profiles/` 아래 export된
20개 affine runtime profile은 모두 `stereodistance_profile_promotion_check.py`를 통과했고,
schema load, 실제 StereoDistance node loader, validation sample truth-free apply가 모두
정상이다. dashboard는 이를 `affine_family_profile_selection` 행으로 기록한다.

이 affine family profile들을 runtime에서 truth 없이 고르려면 raw RDF/disparity feature
selector audit를 실행한다. 이 selector의 입력은 보정/운용 샘플의 raw stereo-center
RDF(`right,down,forward`)와 disparity 통계뿐이며, replay truth는 offline audit metric에만
사용한다. `robust_bbox_fallback` 정책은 raw feature상 nominal/yaw/bbox 저거리 계열에
가까운 경우 bbox-outlier source profile을 보수적으로 우선 검토한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_method_family_runtime_feature_selector.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --family-selection-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/affine_family_profile_selection_audit/method_family_profile_selected.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/affine_family_runtime_feature_selector_audit_robust_bbox \
  --selection-policy robust_bbox_fallback \
  --feature-subset all \
  --target-error-limit-m 10
```

현재 nearest/feature-vote 방식은 19/20 PASS였고 bbox-outlier seed 하나에서
1852m target max error가 10m를 넘었다. `robust_bbox_fallback`은 20/20 PASS,
validation RMSE mean/max `1.366/2.648m`, 1852m target max abs worst-case
`9.423m`였다. 이는 affine family가 truth-free runtime selector로도 유망하다는
증거지만, 아직 full policy catalog의 `8.951m` worst-case보다 크므로 현재 배포 추천은
`policy_catalog+robust_family_fallback`을 유지하고, affine runtime selector는 다음
교체 후보로 추적한다. dashboard는 이를 `affine_family_runtime_feature_selector`
행으로 기록한다.

policy catalog selector와 affine runtime selector를 함께 두고, raw feature threshold로
둘 중 하나를 고르는 portfolio audit도 실행할 수 있다. threshold 규칙은 offline replay
metric으로 학습하지만, leave-one-run-out 평가에서 held-out run 선택 입력은
`feature_*` raw RDF/disparity 통계뿐이다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_runtime_selector_portfolio.py \
  --candidate policy_catalog=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_audit_policy_catalog_robust_family/runtime_measurement_guard_audit.csv \
  --candidate affine_runtime=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/affine_family_runtime_feature_selector_audit_robust_bbox/runtime_feature_selector_audit.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_selector_portfolio_policy_vs_affine \
  --target-error-limit-m 10 \
  --objective target_max_then_rmse \
  --feature-names raw_forward_mean_m,measured_disparity_mean_px,disparity_forward_product_ratio_p10_to_nominal,right_over_forward_std,down_over_forward_std
```

현재 LOO 결과는 18/18 PASS, 1852m target max abs worst-case `8.951m`로 policy
catalog와 동일하고, validation RMSE mean/max는 `1.154/1.806m`에서
`1.150/1.758m`로 소폭 개선된다. 따라서 이는 최종 추천 교체가 아니라 policy catalog를
유지하면서 affine family를 보조 후보로 자동 탐색하는 진단 행이다. 전체 25개 feature를
무조건 쓰면 LOO RMSE가 악화되어 과적합 신호가 나왔으므로, 위 명령처럼 저차원 raw
geometry/product-ratio feature subset을 명시한다. dashboard는 이를
`runtime_selector_portfolio_policy_vs_affine` 행으로 기록한다.

portfolio audit은 후보 selector 사이의 offline oracle gap도 함께 기록한다. 기본
`target_max_then_rmse` 기준에서는 policy/affine 2후보 LOO가 oracle과 8/18 일치했고,
oracle match fraction `0.444`, max RMSE regret `0.064m`, target max regret
`1.688m`였다. 이 objective의 oracle은 target max를 우선하므로 RMSE regret 평균은
음수가 될 수 있다. target-cap 기준의 3후보 진단에서는 oracle과 11/18 일치,
RMSE regret mean/max `0.034/0.337m`로 affine이 일부 row에서 더 낮은 RMSE를 내지만,
LOO에서 안정적으로 잡히지 않았다. dashboard는 이 gap을
`runtime_selector_portfolio_oracle_gap` 행으로 기록한다.

같은 audit은 oracle 후보 label을 raw feature threshold 하나로 분리할 수 있는지도
`runtime_selector_portfolio_oracle_separability.csv/txt`에 남긴다. 이 진단의 목적은
“어떤 regime에서 affine family로 바꾸면 이득인지”를 찾는 것이며, oracle label은
calibration/replay truth로만 계산되고 runtime 입력으로 쓰지 않는다. 2026-06-16
재실행 결과에서 2후보 policy/affine 경로의 최고 분리 feature는
`feature_measured_disparity_mean_px`, accuracy `0.778`, effect size `1.159`였다.
3후보 target-cap 경로에서는 최고 feature가 `feature_down_over_forward_std`,
accuracy `0.667`, effect size `0.369`였다. 따라서 affine override 신호는 존재하지만
현재 raw feature threshold만으로는 충분히 안정적이지 않으므로, 최종 추천은 여전히
`policy_catalog+robust_family_fallback`이다. dashboard는 이 추가 진단을
`runtime_selector_portfolio_oracle_separability` 행으로 기록한다.

추가로 high-precision affine override도 감사한다. 이 모드는 기본 후보
`policy_catalog`를 유지하다가, offline separability 진단에서 false positive가 0인
raw feature threshold를 만족할 때만 `affine_runtime`으로 바꾼다. runtime 입력은
해당 raw feature threshold뿐이고 oracle label은 calibration/replay 진단에만 쓰인다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_runtime_selector_portfolio.py \
  --candidate policy_catalog=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_audit_policy_catalog_robust_family/runtime_measurement_guard_audit.csv \
  --candidate affine_runtime=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/affine_family_runtime_feature_selector_audit_robust_bbox/runtime_feature_selector_audit.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_selector_portfolio_policy_vs_affine \
  --target-error-limit-m 10 \
  --objective target_max_then_rmse \
  --feature-names raw_forward_mean_m,measured_disparity_mean_px,disparity_forward_product_ratio_p10_to_nominal,right_over_forward_std,down_over_forward_std \
  --precision-override-label affine_runtime \
  --precision-override-baseline-label policy_catalog
```

2026-06-16 결과에서 zero-false-positive rule은
`feature_disparity_forward_product_ratio_p10_to_nominal <= 1.0018866404242965`였고
5/18 row만 affine으로 override했다. 18/18 PASS와 1852m target max worst-case
`8.951m`는 유지했지만, validation RMSE mean이 `1.150m`에서 `1.211m`로 악화됐다.
target-cap 기준의 별도 run도 18/18 PASS였지만 RMSE mean `1.154m`로 개선은 없었다.
따라서 conservative override는 안전성 감사 기능으로 보존하되, 현재 추천을 대체하지
않는다. dashboard는 이를 `runtime_selector_portfolio_precision_override` 행으로
기록한다.

추가로 reference 기반 robust regression 후보로 Theil-Sen inverse-depth 계열을
검증한다. Theil-Sen은 pairwise slope의 median으로 1D 선형 관계를 잡는 강건
추정이므로, bbox center outlier 또는 disparity spike가 섞인 보정 비행에서
`1/raw_forward -> 1/truth_forward` 계수가 least-squares보다 안정적인지 확인한다.
보정 truth는 calibration-time fitting과 validation metric에만 사용하며, runtime
profile은 raw stereo-center RDF와 measured disparity, 고정 계수만 사용한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_robustness_sweep.py \
  --output-dir Simulation/results/rtk_stereo_theil_sen_inverse_depth_probe \
  --scenarios bbox_outlier_stress \
  --seeds 20260613,20260614,20260615 \
  --sample-count 520 \
  --target-range-m 1852 \
  --target-window-m 30 \
  --synthetic-target-oversample-count 12 \
  --pooled-candidate-scan-limit 80
```

2026-06-16 bbox outlier focused sweep에서
`target_weighted_theil_sen_global_inverse_depth_linear`는 3/3 seed strict PASS,
validation RMSE mean `1.229m`, 1852m target abs max `1.974m`였다. 기존
`target_weighted_global_inverse_depth_linear`의 validation RMSE mean `1.242m`,
target abs max `2.121m`보다 좋아져, bbox outlier 조건에서는 Theil-Sen robust
inverse-depth fitting이 유효한 후보임을 확인했다. 다만 같은 sweep의 추천은
`huber_joint_disparity_ray_rotation_nls`였고, 그 validation RMSE mean은 `1.223m`,
target abs max `1.990m`였다. pooled replay에서는
`target_weighted_theil_sen_global_inverse_depth_linear`가 3/3 PASS,
validation RMSE mean/max `1.229/1.538m`, target abs max `2.044m`였지만,
pooled validation top인 `sim3_binned_residual_disparity_bins11`은 3/3 PASS,
validation RMSE mean/max `1.209/1.517m`, target abs max `2.066m`였다.

따라서 Theil-Sen 계열은 bbox/disparity outlier가 의심되는 보정 비행에서
저복잡도 inverse-depth fallback 후보로 보존한다. 그러나 validation RMSE와
cross-seed replay 기준에서 현 최종 추천인 `policy_catalog+robust_family_fallback`을
대체하지는 않는다. PASS 기준은 raw 대비 validation RMSE 개선, 1852m target-window
존재, 1852m target abs/max 10m 이하, axis-wise regression 없음, train/validation
gap과 model complexity를 함께 보는 기존 gate와 동일하다.

Theil-Sen 후보를 전체 6개 stress family에 넣은 full-family probe도 실행한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/run_rtk_stereo_robustness_sweep.py \
  --output-dir Simulation/results/rtk_stereo_theil_sen_full_family_probe \
  --scenarios nominal,yaw_pitch_stress,disparity_noise_stress,bbox_outlier_stress,intrinsic_scale_stress,combined_sensor_stress \
  --seeds 20260613,20260614,20260615 \
  --sample-count 520 \
  --target-range-m 1852 \
  --target-window-m 30 \
  --synthetic-target-oversample-count 12 \
  --pooled-candidate-scan-limit 100
```

2026-06-16 결과에서 cross-scenario method stability gate의 robust recommendation은
`target_weighted_theil_sen_global_inverse_depth_linear`였다. 6/6 scenario strict
PASS, validation RMSE mean/max `1.071/1.507m`, 1852m target abs mean/max
`0.927/2.126m`, model complexity mean `5`로, 저복잡도 StereoDistance 적용 후보로
가장 안정적이었다. Scenario별 추천은 nominal과 intrinsic-scale에서
`target_weighted_theil_sen_global_inverse_depth_linear`, yaw/pitch에서
`theil_sen_global_inverse_depth_linear`, disparity-noise에서
`target_shrunk_huber_global_inverse_depth_linear_shrink040`, bbox-outlier에서
`huber_joint_disparity_ray_rotation_nls`, combined-stress에서
`huber_global_inverse_depth_linear`였다.

초기 full-family summary에서는 pooled profile이 transfer replay 18/18 PASS였어도
promotion check에서 pooled validation sample 적용 실패로 표시됐다. 원인은
`stereodistance_profile_promotion_check.py`가 left-camera geometry profile에 필요한
runtime 입력 `left_u_px`, `left_v_px`를 sample payload에 전달하지 않은 checker
버그였다. checker 수정 후 같은 pooled profile
`target_weighted_huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_sigma160`
을 재검증한 결과, promotion check는 PASS, schema/node loader OK,
sample apply `50`, invalid `0`이었다. corrected evidence는
`Simulation/results/rtk_stereo_theil_sen_full_family_probe/pooled_scenario_profile/promotion_check_corrected/`
에 저장한다.

따라서 배포 후보는 두 갈래로 둔다. 첫째, scenario-specific catalog profile 6개는
promotion check 6/6 PASS였고, minimal set은 두 source profile만으로 6개 target
scenario를 덮었다:
`bbox_outlier_stress/huber_joint_disparity_ray_rotation_nls`가 bbox, combined,
disparity-noise, nominal, yaw/pitch를 덮고,
`intrinsic_scale_stress/target_weighted_theil_sen_global_inverse_depth_linear`가
intrinsic-scale을 덮는다. 이 선택은 offline 보정/검증 결과이며, runtime에서는
PX4/RTK truth를 쓰지 않고 배포 메타데이터나 운용자 scenario label로 profile을
고른다. 둘째, pooled profile은 단일 universal runtime profile 후보로 다시 유효하다.
평균 RMSE 기준 1등은
`target_weighted_huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_sigma160`이고
transfer replay 18/18 PASS, validation RMSE mean/max `1.433/2.498m`, 1852m
target abs max `3.911m`, corrected promotion check PASS다. 하지만 단일 배포
fallback은 평균 RMSE보다 1852m worst-case와 validation worst-case를 우선해야 하므로,
`target_max_then_rmse_max` 기준으로
`huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut`를 선택한다.
이 후보는 transfer replay 18/18 PASS, validation RMSE mean/max `1.437/2.475m`,
1852m target abs max `3.839m`, corrected promotion check PASS다. 다만 scenario
catalog는 조건별 RMSE와 target-window 여유가 더 좋을 수 있으므로, 최종 배포 선택은
catalog 방식과 pooled 방식의 validation worst-case, 1852m worst-case, operator가
scenario label을 안정적으로 제공할 수 있는지를 함께 본다.

2026-06-16 이후 `run_rtk_stereo_robustness_sweep.py`의 pooled universal profile
자동 선택 기준도 이 목적에 맞춰 수정했다. 후보가 모든 transfer replay를 통과해야
하고, 그 다음에는 1852m target abs worst-case, validation range RMSE worst-case,
validation range RMSE mean, model complexity 순서로 정렬한다. 따라서 앞으로 생성되는
`pooled_scenario_profile` summary는 단순 평균 RMSE 1등이 아니라 단일 StereoDistance
fallback으로 더 안전한 worst-case 중심 후보를 기본 selected profile로 기록한다.
빠른 검증이나 pooled/profile-policy 목적의 반복 실행에서는 `--skip-near-tie-transfer`를
사용할 수 있다. 이 옵션은 scenario near-tie 후보 목록은 보존하되 cross-scenario
near-tie replay와 best near-tie promotion만 건너뛰며, 좌표/런타임 계약과 pooled,
policy, catalog 검증은 그대로 수행한다.

`Simulation/results/rtk_stereo_worstcase_pooled_objective_smoke` smoke EXP에서는
`--skip-near-tie-transfer --pooled-candidate-scan-limit 20`으로 새 기준을 확인했다.
선택된 pooled universal profile은
`target_weighted_huber_left_disparity_camera_geometry_sim3_sigma160`이고, transfer
replay 18/18 PASS, validation RMSE mean/max `1.446/2.527m`, 1852m target abs
max `4.024m`, promotion check PASS, sample invalid `0`, runtime truth 사용
`false`였다. 대표 policy profile들은 intrinsic-scale에서 15/18 이하로 실패했고,
deployment recommendation은 `single_pooled_runtime_profile`이었다.

이후 `Simulation/results/rtk_stereo_left_geom_sigma_grid_smoke`에서는 같은
left-disparity camera-geometry family의 target weighting 폭을 `80/160/250/400m`
grid로 넓혀 자동 탐색했다. `--skip-near-tie-transfer --pooled-candidate-scan-limit 24`
조건에서 새로 선택된 pooled universal profile은
`target_weighted_huber_left_disparity_camera_geometry_sim3_sigma400`이다. 이 후보는
transfer replay 18/18 PASS, validation RMSE mean/max `1.444/2.521m`,
1852m target abs max `4.011m`, promotion check PASS, sample invalid `0`,
runtime truth 사용 `false`였다. 같은 family의 sigma250은 mean/max
`1.444/2.522m`, target abs max `4.013m`, sigma160은 mean/max
`1.446/2.527m`, target abs max `4.024m`였으므로 sigma400이 현 단일 pooled
fallback 후보 중 가장 낮은 worst-case 기준을 기록한다. 입력 계약은
calibration-time PX4/RTK truth로 fit/validation만 수행하고 runtime에서는
left bbox center, measured disparity, raw RDF만 사용하는 StereoDistance 적용
가능 profile이다.

`Simulation/results/rtk_stereo_left_geom_sigma_wide_smoke`에서는 같은 탐색을
sigma600/800까지 확장했다. sigma600과 sigma800도 transfer replay 18/18 PASS였지만
각각 validation RMSE mean/max `1.446/2.528m`, `1.448/2.534m`, 1852m target abs
max `4.025m`, `4.038m`로 sigma400보다 나빴다. 따라서 현재 evidence에서는
target weighting 폭을 sigma400보다 더 넓히는 것은 이득이 없고, sigma400이
단일 pooled fallback의 현 기준점이다.

`Simulation/results/rtk_stereo_left_geom_residual_shrink_smoke`에서는 sigma400
left-disparity geometry profile 위에 raw-forward residual LUT를 약하게 적용하는
shrink 후보를 추가했다. `--pooled-candidate-scan-limit 30` 조건에서 선택된
profile은
`target_weighted_huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_sigma400_shrink050`이다.
이 후보는 transfer replay 18/18 PASS, validation RMSE mean/max `1.436/2.509m`,
1852m target abs max `3.961m`, promotion check PASS, sample invalid `0`,
runtime truth 사용 `false`였다. shrink025도 18/18 PASS였지만 mean/max
`1.439/2.515m`, target abs max `3.986m`로 shrink050보다 약했다. 따라서 현 단일
pooled fallback 3-seed 기준점은 sigma400 단독에서 sigma400 + 50% raw-forward
residual shrink 조합으로 갱신했다. 이 조합도 runtime에서는 left bbox center,
measured disparity, raw RDF forward, calibration-fixed parameters만 사용한다.

`Simulation/results/rtk_stereo_left_geom_residual_shrink_seed5_smoke`에서는 같은
pooling/replay 조건을 seed 5개(`20260611`-`20260615`)로 확장했다. 새 selected
single pooled profile은
`huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut`이며,
transfer replay 30/30 PASS, validation RMSE mean/max `1.403/2.468m`, 1852m
target abs mean/max `1.621/4.888m`, promotion check PASS, sample invalid `0`,
runtime truth 사용 `false`였다. 같은 5-seed run에서 sigma400 기본형은 30/30 PASS,
validation RMSE mean/max `1.409/2.514m`, target abs max `5.050m`였고,
sigma250/160/600/800도 모두 30/30 PASS였지만 target abs max가 약 `5.05m` 이상으로
더 컸다. 반대로 target-weighted inverse-depth, Theil-Sen, joint disparity/ray NLS
계열은 pooled target error가 작거나 combined-sensor 성능이 좋을 수 있었지만 대부분
25/30 transfer replay에 그쳐 단일 fallback 조건을 통과하지 못했다. 따라서 5-seed
기준 단일 StereoDistance fallback은 residual LUT가 포함된 non-target-weighted
left-disparity camera-geometry profile로 갱신한다. 이 결과는 1852m ±10m 조건에는
충분히 들어오지만, 단일 profile만으로 모든 stress family의 target abs max를 full
scenario catalog 수준까지 낮추지는 못한다.

`Simulation/results/rtk_stereo_left_geom_unweighted_residual_shrink_smoke`에서는
5-seed selected family인 non-target-weighted left-disparity camera-geometry residual
LUT에도 shrink025/050/075 후보를 추가해 3-seed smoke로 비교했다. 새 selected single
pooled profile은
`huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_shrink075`이며,
transfer replay 18/18 PASS, validation RMSE mean/max `1.438/2.492m`, 1852m
target abs mean/max `1.583/3.896m`, promotion check PASS, sample invalid `0`,
runtime truth 사용 `false`였다. 같은 run에서 unweighted shrink050은 18/18 PASS,
validation RMSE mean/max `1.440/2.509m`, target abs max `3.952m`였고,
target-weighted sigma400 shrink050은 18/18 PASS, validation RMSE mean/max
`1.436/2.509m`, target abs max `3.961m`였다. 따라서 3-seed 기준에서는
unweighted residual LUT에 75% shrink를 적용한 profile이 1852m worst-case 기준을
가장 낮췄다. 다만 3-seed 결과만으로는 profile 안정성을 확정하지 않고 5-seed 확장
검증 대상으로 둔다.

`Simulation/results/rtk_stereo_left_geom_unweighted_residual_shrink_seed5_smoke`에서는
같은 unweighted residual shrink 후보를 seed 5개(`20260611`-`20260615`)로 확장했다.
선택된 single pooled fallback은 다시
`huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut`이며,
transfer replay 30/30 PASS, validation RMSE mean/max `1.403/2.468m`, 1852m
target abs mean/max `1.621/4.888m`, promotion check PASS, sample invalid `0`,
runtime truth 사용 `false`였다. 3-seed에서 가장 좋았던 shrink075도 30/30 PASS였지만
validation RMSE mean/max `1.404/2.484m`, 1852m target abs mean/max
`1.603/4.940m`로 worst-case validation과 target error가 모두 unshrunk profile보다
약간 나빴다. 따라서 5-seed 기준 단일 StereoDistance fallback은 unweighted residual
LUT를 shrink하지 않는 profile로 유지한다. 이 비교는 calibration-time PX4/RTK NED
truth를 stereo-center RDF truth로 변환해 fit/validation에만 사용하고, runtime에서는
left bbox center, measured disparity, raw RDF forward, calibration-fixed parameters만
사용한다.

`Simulation/results/rtk_stereo_all_pooled_candidates_seed5_probe`에서는 pooled candidate
scan limit을 `0`으로 풀어 66개 pooled 후보 전체를 seed 5개, 6개 stress scenario,
30개 transfer replay 조건에서 다시 비교했다. 12개 후보만 30/30 transfer replay를
통과했고, selected single pooled fallback은 여전히
`huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut`였다.
성능은 validation RMSE mean/max `1.403/2.468m`, 1852m target abs mean/max
`1.621/4.888m`, promotion check PASS, sample invalid `0`, runtime truth 사용
`false`였다. 가장 가까운 target-weighted residual LUT 변형은 30/30 PASS였지만
validation max 또는 1852m target worst-case가 더 컸고, inverse-depth/Sim(3)
residual 계열은 장거리 평균 오차나 combined-sensor 성능이 좋더라도 intrinsic-scale
transfer replay에서 raw보다 악화되어 pass-all 단일 fallback 조건을 통과하지 못했다.
따라서 단일 StereoDistance fallback은 unshrunk left-disparity camera-geometry
residual LUT profile로 유지한다. 동시에 이 결과는 best single profile의 문제가
보정식 형태 자체보다 error-regime 식별과 guard/select policy에 있음을 보여준다.

`Simulation/results/rtk_stereo_runtime_guard_all_catalog_intrinsic_gap_seed5_probe`에서는
같은 5-seed sweep의 full scenario catalog를 대상으로 runtime-measurement guard를
검증했다. 선택 입력은 raw stereo-center RDF(`raw_right/down/forward/range`),
measured disparity, left image center(`left_u/v`) 통계뿐이며, PX4/RTK truth와
validation error는 offline audit에만 사용한다. all-catalog 조건에서
`intrinsic_scale_gap_transfer_risk` policy는 30/30 PASS, validation RMSE mean/max
`1.013/1.670m`, 1852m target abs mean/max `0.936/2.340m`, target max abs max
`9.609m`였다. 같은 조건의 `centroid_knn_veto_transfer_risk`도 동일한 30/30 PASS
요약을 냈다. 단순 nearest-centroid guard
(`Simulation/results/rtk_stereo_runtime_guard_all_catalog_nearest_centroid_seed5_probe`)도
30/30 PASS였지만 validation RMSE mean `1.024m`로 transfer-risk 계열보다 약간 높았다.
이는 단일 pooled fallback보다 validation worst-case와 1852m target worst-case가 모두
낮고, full scenario catalog의 장점을 runtime-like raw feature selection으로 일부
회수한다.

단, `Simulation/results/rtk_stereo_runtime_guard_all_catalog_intrinsic_gap_loso_seed5_probe`
처럼 target scenario family 전체를 feature library에서 제외한 strict LOSO audit에서는
25/30 PASS, validation RMSE mean/max `1.498/2.735m`, 1852m target abs mean/max
`1.778/5.132m`로 떨어졌다. Minimal-set guard
(`Simulation/results/rtk_stereo_runtime_guard_minimal_intrinsic_gap_seed5_probe`)도
24/30 PASS에 그쳤다. 따라서 현재 guard는 알려진 deployment/error regime 사이의
truth-free profile selector로는 유망하지만, 보지 못한 새 regime까지 일반화하는
검출기라고 보기는 어렵다. 실제 배포에서는 operator/deployment metadata 또는 명시적
scenario label이 있으면 full catalog를 우선하고, metadata가 없으면 guard selector를
보조 후보로 두되 unknown regime fallback은 single pooled profile로 유지한다.

Oracle-regret audit도 같은 결론을 지지한다.
`Simulation/results/rtk_stereo_runtime_guard_all_catalog_intrinsic_gap_oracle_regret_seed5_probe`
에서 all-catalog intrinsic-gap guard는 offline oracle과 14/30 일치, selected/oracle
모두 30/30 PASS, target abs regret mean/max `0.040/0.387m`였다.
`Simulation/results/rtk_stereo_runtime_guard_all_catalog_nearest_centroid_oracle_regret_seed5_probe`
에서 nearest-centroid guard는 oracle 일치 20/30, selected/oracle 모두 30/30 PASS,
target abs regret mean/max `0.043/0.387m`였다. 즉 full catalog를 유지할 수 있으면
간단한 raw-feature selector도 offline oracle에 매우 가까우며, 정책 선택은 RMSE 평균을
조금 더 낮출지 또는 oracle replay score 일치를 높일지의 tradeoff로 본다.
`Simulation/results/rtk_stereo_runtime_guard_threshold_sweep_all_catalog_seed5_probe`의
transfer-risk threshold sweep도 기본 low-risk 기준 `RMSE 1.3m / target 2.5m`를 다시
추천했다. 반면
`Simulation/results/rtk_stereo_runtime_guard_threshold_sweep_all_catalog_loso_seed5_probe`
는 best threshold에서도 25/30 PASS에 머물러, strict LOSO 실패가 threshold 튜닝 문제가
아니라 unseen error-regime 일반화 문제임을 확인했다.
`Simulation/results/rtk_stereo_gap_feature_separability_loso_intrinsic_combined_seed5_probe`
에서는 strict LOSO audit row의 raw feature만으로 gap family separability를 점검했다.
combined-sensor는 `down_over_forward_mean <= 0.00641`, intrinsic-scale은
`raw_forward_mean >= 1145.11m` 또는
`disparity_forward_product_ratio_p10_to_nominal >= 1.00374` 같은 단일 feature
threshold로 30-row audit에서 accuracy `1.0`을 보였다. 이는 guard 개선 후보가
있음을 뜻하지만, threshold 자체는 현재 synthetic sweep에서 얻은 offline diagnostic이므로
실제 배포 전에는 별도 비행/환경 데이터로 재검증해야 한다.
`Simulation/results/rtk_stereo_runtime_selector_portfolio_transfer_vs_nearest_limited_seed5_probe`
에서는 transfer-risk guard와 nearest-centroid guard를 4개 핵심 raw feature만으로
depth-2 portfolio rule로 조합했다. 결과는 30/30 PASS, validation RMSE mean/max
`1.013/1.670m`, target max abs max `9.609m`로 transfer-risk guard와 같았고,
추가 이득은 확인되지 않았다. 따라서 현재는 full catalog + simple raw-feature guard를
우선하고, portfolio는 더 다양한 후보가 생긴 뒤 다시 평가한다.

`Simulation/results/rtk_stereo_single_pooled_runtime_candidate_seed5_probe`에서는
all-catalog guard audit의 raw feature 열을 그대로 재사용하고, 단일 pooled fallback
profile을 각 calibration CSV에 offline replay해 runtime-selector 호환 candidate CSV를
만들었다. 이 변환은 selector 입력으로 raw stereo-center RDF, measured disparity,
image-center 통계만 복사하고, PX4/RTK truth는 replay metric 계산에만 사용한다. 생성된
`single_pooled_runtime_candidate.csv`는 30/30 PASS, validation RMSE mean/max
`1.403/2.468m`, target max abs max `8.388m`였다. 실행 예:

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python \
  Simulation/scripts/build_single_profile_runtime_audit_candidate.py \
  --guard-audit-csv Simulation/results/rtk_stereo_runtime_guard_all_catalog_intrinsic_gap_seed5_probe/runtime_measurement_guard_audit.csv \
  --profile-json Simulation/results/rtk_stereo_all_pooled_candidates_seed5_probe/pooled_scenario_profile/pipeline/runtime_profiles/huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut.json \
  --output-csv Simulation/results/rtk_stereo_single_pooled_runtime_candidate_seed5_probe/single_pooled_runtime_candidate.csv \
  --label single_pooled \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

`Simulation/results/rtk_stereo_runtime_selector_portfolio_catalog_vs_single_pooled_seed5_probe`
에서는 full catalog guard와 단일 pooled fallback을 같은 runtime feature portfolio에서
비교했다. 4개 핵심 feature depth-2 rule은 30/30 PASS, validation RMSE mean/max
`1.025/1.670m`, target max abs max `9.609m`로 pass 조건은 만족했지만, 단일 pooled가
더 유리한 일부 seed13 행을 안정적으로 분리하지 못했다. 확장 feature 후보를 넣은
`rtk_stereo_runtime_selector_portfolio_catalog_vs_single_pooled_extended_seed5_probe`도
target max를 낮추지는 못했다. 반면
`rtk_stereo_runtime_selector_portfolio_catalog_vs_single_pooled_single_override_seed5_probe`
의 zero-false-positive override audit은 기본 후보를 full catalog guard로 두고,
`feature_left_v_std_px >= 129.7294559911429`일 때만 단일 pooled fallback을 선택했다.
이 conservative override는 30/30 PASS, validation RMSE mean/max `1.018/1.670m`,
target max abs max `8.388m`로 all-catalog guard의 `9.609m` worst-case를 낮췄다.
다만 override true-positive는 1/30뿐이며, threshold는 synthetic 5-seed replay에서 얻은
offline diagnostic이다. 따라서 현재 추천은 “known scenario catalog가 있으면 catalog
guard를 기본으로 쓰고, raw image-center variance가 zero-FP 조건을 만족하는 경우에만
single pooled fallback으로 보수 전환”이다. 실제 본실험에서는 이 threshold를 고정하기
전에 별도 비행 replay 또는 hardware/WIRIS 데이터로 false-positive 여부를 다시 확인해야
한다.

Strict LOSO 조건에서도 같은 변환을 적용했다.
`Simulation/results/rtk_stereo_single_pooled_runtime_candidate_loso_seed5_probe`의 단일
pooled candidate는 30/30 PASS, target max abs max `8.388m`였다.
`Simulation/results/rtk_stereo_runtime_selector_portfolio_loso_vs_single_pooled_seed5_probe`
의 depth-2 portfolio는 28/30 PASS에 그쳐 pass-all을 회복하지 못했다.
`Simulation/results/rtk_stereo_runtime_selector_portfolio_loso_vs_single_pooled_single_override_seed5_probe`
의 conservative override는 30/30 PASS를 회복했지만 target max abs max가
`9.733m`였다. 따라서 source family가 빠진 unknown-regime 조건에서는 raw-feature
threshold로 LOSO guard를 억지 조합하기보다, 단일 pooled fallback을 그대로 쓰는 편이
현재 synthetic 5-seed replay 기준으로 더 단순하고 worst-case가 낮다.

위 결과를 배포 모드별 추천으로 묶으려면 다음을 실행한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python \
  Simulation/scripts/summarize_runtime_deployment_strategy.py \
  --known-catalog-summary-json Simulation/results/rtk_stereo_runtime_guard_all_catalog_intrinsic_gap_seed5_probe/runtime_measurement_guard_summary.json \
  --known-override-summary-json Simulation/results/rtk_stereo_runtime_selector_portfolio_catalog_vs_single_pooled_single_override_seed5_probe/runtime_selector_portfolio_precision_override_summary.json \
  --single-pooled-candidate-csv Simulation/results/rtk_stereo_single_pooled_runtime_candidate_seed5_probe/single_pooled_runtime_candidate.csv \
  --loso-override-summary-json Simulation/results/rtk_stereo_runtime_selector_portfolio_loso_vs_single_pooled_single_override_seed5_probe/runtime_selector_portfolio_precision_override_summary.json \
  --loso-single-pooled-candidate-csv Simulation/results/rtk_stereo_single_pooled_runtime_candidate_loso_seed5_probe/single_pooled_runtime_candidate.csv \
  --output-dir Simulation/results/rtk_stereo_runtime_deployment_strategy_seed5_probe \
  --target-error-limit-m 10
```

산출물은 `runtime_deployment_strategy_candidates.csv`,
`runtime_deployment_strategy_recommendation.json/txt`이다. 5-seed synthetic replay
기준 자동 추천은 known-catalog 모드에서는
`known_catalog_guard_with_single_pooled_override`이고, runtime rule은
`catalog_guard`를 기본으로 쓰되
`feature_left_v_std_px >= 129.7294559911429`이면 `single_pooled`로 전환하는 것이다.
unknown/unseen-regime 모드에서는 `single_pooled_runtime_profile` 단독을 추천한다. 두
모드 모두 pass-all이고 target max abs max는 `8.388m`로 1852m ±10m 조건을 만족한다.

추천 결과를 실제 배포 패키지 형태로 묶으려면 다음을 실행한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/export_runtime_deployment_package.py \
  --strategy-recommendation-json Simulation/results/rtk_stereo_runtime_deployment_strategy_seed5_probe/runtime_deployment_strategy_recommendation.json \
  --single-pooled-profile-json Simulation/results/rtk_stereo_all_pooled_candidates_seed5_probe/pooled_scenario_profile/pipeline/runtime_profiles/huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut.json \
  --scenario-catalog-json Simulation/results/rtk_stereo_all_pooled_candidates_seed5_probe/scenario_profile_catalog.json \
  --transfer-matrix-csv Simulation/results/rtk_stereo_all_pooled_candidates_seed5_probe/scenario_profile_catalog_transfer_matrix.csv \
  --output-dir Simulation/results/rtk_stereo_runtime_deployment_package_seed5_probe \
  --target-range-m 1852 \
  --target-error-limit-m 10
```

패키지 산출물은 `runtime_deployment_package_manifest.json/txt`,
`known_catalog_runtime_selector.json`, `active_runtime_profile_unknown.json`,
`profiles/single_pooled_runtime_profile.json`, `catalogs/scenario_profile_catalog.json`,
`catalogs/scenario_profile_catalog_transfer_matrix.csv`이다. unknown/unseen-regime에서는
`active_runtime_profile_unknown.json`을 StereoDistance active profile로 바로 사용한다.
known-catalog 배포에서는 `known_catalog_runtime_selector.json`의 baseline catalog guard와
override rule을 사용하되, runtime 입력은 raw RDF/시차/left image-center feature로
제한한다. `active_runtime_profile_unknown.json`은 promotion check에서 schema load,
node loader, sample apply 50/50, forbidden runtime input 선언을 모두 통과했다.

패키지 selector가 특정 raw measurement CSV에서 어떤 profile을 고르는지 확인하려면:

```bash
Simulation/.venv/bin/python Simulation/scripts/apply_runtime_deployment_selector.py \
  --selector-json Simulation/results/rtk_stereo_runtime_deployment_package_seed5_probe/known_catalog_runtime_selector.json \
  --sweep-dir Simulation/results/rtk_stereo_all_pooled_candidates_seed5_probe \
  --input-csv Simulation/results/rtk_stereo_all_pooled_candidates_seed5_probe/bbox_outlier_stress/seed_20260613/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_runtime_deployment_package_seed5_probe/selector_apply_bbox_seed20260613_all_rows
```

이 selector 적용 명령의 `--split`은 replay 평가 split이 아니라 feature window를 줄이는
필터다. 기존 guard audit과 같은 조건을 재현하려면 기본값처럼 split 없이 전체 raw
measurement rows를 사용한다. 예를 들어 bbox outlier seed `20260613` 전체 행에서는
`feature_left_v_std_px=129.739`로 override가 켜져 `single_pooled`가 선택되고, nominal
seed `20260611` 전체 행에서는 `feature_left_v_std_px=127.502`로 catalog guard가 유지된다.

패키지 selector가 30개 seed 전체에서 offline precision-override audit을 재현하는지도
검증할 수 있다.

```bash
Simulation/.venv/bin/python Simulation/scripts/apply_runtime_deployment_selector.py \
  --selector-json Simulation/results/rtk_stereo_runtime_deployment_package_seed5_probe/known_catalog_runtime_selector.json \
  --sweep-dir Simulation/results/rtk_stereo_all_pooled_candidates_seed5_probe \
  --output-dir Simulation/results/rtk_stereo_runtime_deployment_package_seed5_probe/selector_sweep_audit_all_rows \
  --audit-all-sweep-seeds \
  --reference-audit-csv Simulation/results/rtk_stereo_runtime_selector_portfolio_catalog_vs_single_pooled_single_override_seed5_probe/runtime_selector_portfolio_precision_override_audit.csv
```

이 audit은 `runtime_deployment_selector_sweep_audit.csv`와
`runtime_deployment_selector_sweep_summary.json/txt`를 저장한다. 5-seed 결과는
selected candidate match `30/30`, override match `30/30`, override count `1`이며,
유일한 override는 `bbox_outlier_stress/seed_20260613`이다. reference audit은 선택 결과
비교용 offline evidence일 뿐이고, selector 자체는 raw RDF/시차/left image-center feature만
사용한다.

ROS2 런타임에 붙일 때는 `stereo_ground_sim.runtime_deployment_selector`의
`RuntimeMeasurementSample`, `RuntimeFeatureWindow`, `PackagedRuntimeDeploymentSelector`를
사용한다. 토픽 adapter는 `/stereo/estimated_position` 또는 내부 raw RDF 계산 결과를
`raw_right_m/raw_down_m/raw_forward_m/raw_range_m`로, `/stereo/disparity`를
`measured_disparity_px`로, BBox/검출 topic의 left image center를 `left_u_px/left_v_px`로
넣으면 된다. 이 모듈은 feature window와 override rule만 처리하며, PX4/RTK truth,
validation RMSE, target error 같은 replay metric은 runtime input으로 받지 않는다.
기존 StereoDistance 공개 토픽을 바로 연결할 때 `/stereo/estimated_position` 배열은
FRD+range `data=[forward,right,down,range]`로 해석하고,
`RuntimeMeasurementSample.from_public_frd_range(data, disparity, left_u, left_v)`로
내부 selector 입력인 stereo-center RDF `raw_right/raw_down/raw_forward/raw_range`로
변환한다. 이미 내부 계산 단계에서 RDF named fields를 갖고 있다면
`RuntimeMeasurementSample.from_mapping()`에 `raw_right_m/raw_down_m/raw_forward_m`,
`measured_disparity_px`, 선택적 `left_u_px/left_v_px`를 넘긴다. 두 경로 모두 런타임
선택 feature에는 truth, validation error, replay metric을 포함하지 않는다.

Runtime-aligned RDF polynomial residual 계열의 degree/ridge/clamp 탐색은 다음
스크립트로 반복 실행할 수 있다. fitting/evaluation에서만 PX4/local NED truth를
stereo-center RDF로 변환해 사용하고, 후보 runtime contract는 raw RDF와 measured
disparity, 고정 계수만 사용한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_runtime_aligned_rdf_poly_degree_search.py \
  --experiment-id EXP-YYYYMMDD-NNN-runtime-aligned-rdf-poly-search \
  --output-dir Simulation/reports/experiments/EXP-YYYYMMDD-NNN-runtime-aligned-rdf-poly-search \
  --seed-start 20294359 \
  --trial-count 400 \
  --train-sample-count 1260 \
  --validation-sample-count 420 \
  --right-degree 6 \
  --down-degree 7 \
  --forward-degrees 12 \
  --ridge-alphas 0.1 \
  --huber-deltas 8.0 \
  --forward-clamps 1.2,1.6,2.0,2.4
```

2026-06-15 현재 runtime-aligned branch의 synthetic holdout 결론은
`d6/d7/d12 + Huber8 + ridge0.1 + forward clamp 2.0m`가 validation
range/position 기준으로 가장 안정적이라는 것이다. EXP-180은 forward degree
14/16/18 확장이 degree 12 추천을 뒤집지 못함을 보였고, EXP-181은 ridge 0.1을
유지하는 쪽이 aggregate validation RMSE에 유리함을 보였으며, EXP-182는 clamp
2.0m가 1.2/1.6/2.4m보다 validation range/position RMSE 기준으로 낫다는 것을
확인했다. EXP-183/184는 Huber delta 민감도를 확인했으며, delta 6 이상에서는
range/position 결과가 거의 포화되고 delta 1/2처럼 강한 robust weighting은 lateral
RMSE를 조금 낮추는 대신 Forward/range RMSE를 악화한다. EXP-185의 800-seed stability
audit에서도 delta 6/8/10은 사실상 동률이고, delta 6이 aggregate range/position에서
극히 작은 차이로 낮다. 따라서 현재 배포 후보는 기존 Huber8 표기를 유지할 수 있지만,
검증상 핵심은 `delta >= 6` plateau, ridge 0.1, clamp 2.0m, degree d6/d7/d12 조합이다.
1852m target-window error는 모든 후보가 +/-10m 조건을 만족했으므로, 선택 기준은
target만이 아니라 validation RMSE, worst-case, clamp count를 함께 본다.

현재 branch를 StereoDistance runtime JSON으로 내보내려면 다음을 사용한다. 이 export는
z-scored disparity에서 fit한 residual polynomial을 raw-disparity-domain coefficients로
변환해서 `EstimatedToTruthSim3Profile` loader가 그대로 평가할 수 있게 만든다.

```bash
Simulation/.venv/bin/python Simulation/scripts/export_runtime_aligned_rdf_poly_profile.py \
  --experiment-id EXP-YYYYMMDD-NNN-runtime-aligned-d6d7-profile-export-replay-audit \
  --output-dir Simulation/reports/experiments/EXP-YYYYMMDD-NNN-runtime-aligned-d6d7-profile-export-replay-audit/output \
  --seed 20295559 \
  --train-sample-count 1260 \
  --validation-sample-count 420 \
  --right-degree 6 \
  --down-degree 7 \
  --forward-degree 12 \
  --ridge-alpha 0.1 \
  --huber-delta-m 8.0 \
  --forward-clamp-m 2.0
```

EXP-186에서 이 profile은 real StereoDistance node loader와 replay path를 통과했다.
Validation replay는 range RMSE `2.396 -> 0.694m`, position RMSE `4.959 -> 0.704m`,
1852m target-window max absolute error `6.729 -> 1.958m`로 개선됐고, promotion check는
node loader OK, sampled runtime rows `50/50`, invalid `0`, runtime truth unused를 기록했다.
EXP-187은 이 exported profile을 재학습 없이 200개 independent synthetic seed에 고정 적용한
fresh-seed transfer audit이다. 결과는 pass `200/200`, range RMSE mean
`2.419 -> 0.685m`, corrected range RMSE max `0.810m`, 1852m target-window max absolute
error worst `4.607m`, invalid/fallback `0`으로, profile JSON 자체의 synthetic transfer
안정성을 확인했다.
하지만 EXP-188에서 같은 exported profile을 6개 stress scenario에 universal single profile로
고정 적용하면 전체 pass는 `241/300`으로 떨어진다. `nominal`과 `yaw_pitch_stress`는
`50/50` STRICT_PASS였지만, `intrinsic_scale_stress`는 `0/50`, `disparity_noise_stress`는
`49/50`, `bbox_outlier_stress`는 `42/50`이고 bbox/outlier target max abs error는
`19.585m`까지 커졌다. 따라서 이 profile은 nominal/yaw 계열의 강한 runtime profile로
유지하되, 전체 배포에는 scenario/catalog selector 또는 runtime measurement guard가 필요하다.

EXP-190~193은 이 실패를 bbox-outlier 조건에서 더 좁혀 확인한 probe이다. Huber delta,
Forward residual clamp, 기존 inverse-depth/left-disparity geometry profile, 그리고
간단한 temporal/smoothing guard를 비교했지만 fresh bbox-outlier transfer는 최대
`39/50` pass에 머물렀고 1852m target max abs error는 약 `18.8m`로 남았다. 이 조건의
일부 outlier는 measured disparity와 raw Forward를 함께 일관되게 이동시키므로,
단일 프레임의 `raw RDF + measured disparity`만으로는 정상 샘플과 구분되지 않는다.
따라서 이 branch는 더 강한 단일 보정식을 찾는 문제가 아니라, 반복 관측, tracker
association confidence, image-space 품질 신호, 또는 검증된 motion model 기반
multi-frame fusion을 추가해야 하는 관측가능성 문제로 분류한다.

EXP-194~197은 corrected RDF 시계열 위에 runtime-safe causal alpha-beta motion fusion을
추가한 probe이다. 실행은 다음과 같다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_runtime_motion_fusion_sweep.py \
  --experiment-id EXP-YYYYMMDD-NNN-bbox-outlier-causal-alpha-beta-motion-fusion \
  --profile-json Simulation/reports/experiments/EXP-20260615-190-bbox-outlier-runtime-profile-huber-sweep/profiles/huber4/recommended_runtime_profile.json \
  --output-dir Simulation/reports/experiments/EXP-YYYYMMDD-NNN-bbox-outlier-causal-alpha-beta-motion-fusion \
  --scenario bbox_outlier_stress \
  --seed-start 20297659 \
  --trial-count 50 \
  --sample-count 1680 \
  --train-ratio 0.75 \
  --target-oversample-count 72 \
  --axes rdf
```

이 fusion은 timestamped corrected stereo-center RDF만 사용하고 truth, target label,
true disparity를 runtime 입력으로 쓰지 않는다. EXP-194의 best RDF fusion은 `47/50`,
target max abs `13.786m`였고, EXP-196의 forward-only fusion도 `47/50`, target max
`13.788m`였다. EXP-197의 더 강한 smoothing은 pass count와 RMSE가 악화됐다. 따라서
causal alpha-beta fusion은 runtime stabilizer 후보로는 의미가 있지만, 현재
`bbox_outlier_stress`의 final PASS 방법은 아니다.

Corrected deployment comparison은
`Simulation/results/rtk_stereo_theil_sen_full_family_probe/deployment_strategy_corrected_comparison.csv`
에 저장한다. 2026-06-16 기준 비교는 다음과 같다.

- Full scenario catalog: profile 6개, promotion 6/6, validation RMSE mean/max
  `1.090/1.544m`, 1852m target abs max `1.990m`.
- Minimal two-profile catalog: profile 2개, transfer 18/18, validation RMSE
  mean/max `1.293/2.487m`, 1852m target abs max `4.377m`.
- Pooled universal left-disparity geometry profile: profile 1개, corrected
  promotion 1/1, transfer 18/18, validation RMSE mean/max `1.437/2.475m`,
  1852m target abs max `3.839m`.

따라서 scenario/deployment label을 신뢰할 수 있으면 full scenario catalog가 기본
추천이다. scenario label 없이 단일 profile만 배포해야 하면 corrected pooled universal
profile 중 5-seed transfer replay를 통과한
`huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut`를
현재 가장 강한 fallback으로 둔다. Minimal two-profile catalog는 profile 수를 줄일 수
있지만 pooled single profile보다 RMSE worst-case가 거의 같고 target worst-case는 더
나쁘므로, 현재는 우선순위를 낮춘다.

운영 결론은 다음과 같다. 방법론 후보로는
`target_weighted_theil_sen_global_inverse_depth_linear`를 robust low-complexity
baseline으로 승격한다. 실제 StereoDistance 배포 profile은 scenario label을 신뢰할 수
있으면 scenario catalog를 기본으로 사용하고, scenario label 없이 하나의 profile만
배포해야 하면 corrected promotion check를 통과한 pooled left-disparity geometry
profile을 후보로 비교한다. 특히 bbox/outlier 계열 Sim(3) 잔차 profile은 nominal/yaw에는
잘 전이되지만 intrinsic-scale에서는 raw보다 나빠질 수 있으므로, intrinsic-scale
family는 별도 catalog profile로 분리한다. 다음 자동탐색에서는 near-tie replay 폭을
제한해 full-family probe가 부가 replay에 과도한 시간을 쓰지 않도록 한다.

추가로 `audit_runtime_selector_portfolio.py`는 `--objective target_cap_then_rmse`와
`--max-depth 2`를 지원한다. `target_cap_then_rmse`는 1852m target max가
`--target-error-limit-m`을 넘는 선택을 먼저 피하고, 한계 안에서는 validation RMSE를
우선한다. 세 후보(`robust_family`, `transfer_risk`, `affine_bbox`)를 넣은 5-feature
LOO 진단에서는 depth-1 target-cap objective가 기존과 같은 18/18 PASS,
RMSE mean/max `1.150/1.758m`, target max `8.951m`였다. target 포함 oracle
진단은 RMSE mean `1.131m`까지 내려가 잠재 신호는 있었지만, depth-2 LOO는
RMSE mean/max `1.196/2.381m`, target max `9.423m`로 악화됐다. 따라서 현재는
depth-2 tree를 배포 추천으로 쓰지 않고, selector 과적합 진단 기능으로만 유지한다.

strict LOSO에서 intrinsic-scale source profile이 빠진 경우를 별도로 분리하려면
calibration-required gap 요약을 실행한다. 이 요약은 runtime raw RDF/disparity에서
계산한 product-ratio feature만으로 “해당 regime profile이 필요하지만 catalog에 없음”을
표시하고, truth 기반 replay metric은 offline 안전성 감사에만 사용한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/summarize_runtime_calibration_required_gaps.py \
  --runtime-audit-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_audit_policy_catalog_robust_family_loso/runtime_measurement_guard_audit.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_calibration_required_gap_summary_policy_catalog_robust_family_loso \
  --product-ratio-p10-threshold 1.0037 \
  --required-source-scenario intrinsic_scale_stress \
  --target-error-limit-m 10
```

현재 policy catalog robust-family strict LOSO 결과에서는 18개 audit 중 3개가
calibration-required intrinsic-scale regime으로 감지된다. 모두 target max 10m 안에는
있지만 strict PASS는 0/3, validation-only PASS는 2/3이며, 1개 run은 선택된 대체
profile이 validation RMSE를 악화시켰다. 따라서 이 영역은 “대체 profile을 더 고르는
문제”가 아니라 실제 intrinsic-scale 보정 비행 profile을 catalog에 포함하거나,
runtime에서 calibration-required 상태로 보수적으로 처리해야 하는 coverage gap이다.
experiment dashboard는 이를 `runtime_calibration_required_gap_loso` 행으로 기록한다.

필수 intrinsic-scale source profile이 없는 catalog/minimal-set에서 active runtime
profile export를 막는 smoke는 다음처럼 실행한다. 이 경로는 runtime raw
stereo-center RDF/disparity feature만 사용해 차단 여부를 결정하고, calibration-time
truth는 사용하지 않는다.

```bash
Simulation/.venv/bin/python Simulation/scripts/export_stereodistance_profile_by_operating_mode.py \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/policy_family_profile_catalog/manifest.json \
  --input-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/intrinsic_scale_stress/seed_20260613/calibration_samples.csv \
  --output-profile /tmp/sasl_blocked_runtime_profile.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/operating_mode_export_intrinsic_scale_required_block_smoke \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --mode runtime_guard \
  --runtime-candidate-source-mode minimal_set \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_guard_minimal_set_without_intrinsic_scale_for_block_smoke.json \
  --runtime-selection-policy nearest_centroid \
  --intrinsic-scale-gap-source-scenario intrinsic_scale_stress \
  --intrinsic-scale-gap-product-ratio-p10-threshold 1.0037 \
  --block-calibration-required-runtime-export
```

현재 smoke 결과는 `feature_disparity_forward_product_ratio_p10_to_nominal =
1.003814`로 threshold `1.0037`을 넘고, `intrinsic_scale_stress` source가 없으므로
`selected=false`, `calibration_required_blocked_export=true`, `output_profile=""`이다.
즉 안전하지 않은 fallback active profile을 만들지 않는다. dashboard는 이를
`runtime_calibration_required_export_block_smoke` 행으로 기록한다.

여러 audit/export 결과를 한 번에 비교하려면 experiment dashboard를 생성한다. 이
dashboard는 보정 비행 truth를 사용할 수 있는 offline 선택과 truth가 없는 runtime
선택을 분리해서 요약하고, 각 결과의 pass count와 1852m target-window worst-case를
한 곳에 기록한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/summarize_rtk_stereo_experiment_dashboard.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/experiment_dashboard
```

산출물은 `rtk_stereo_experiment_dashboard.csv`,
`rtk_stereo_experiment_dashboard_summary.json`,
`rtk_stereo_experiment_dashboard_summary.txt`,
`rtk_stereo_experiment_dashboard_pass_ratio.png`이다.
2026-06-16 현재 monotone inverse-depth runtime sweep dashboard 기준으로 runtime
truth-free 추천은 `policy_catalog+robust_family_fallback`, truth가 있는 보정/검증 단계
추천은 `calibration_replay_all_catalog`, truth가 없는 runtime 추천은
`runtime_transfer_risk_guard`이다. policy catalog runtime selector는 18/18 PASS,
1852m target max abs worst-case `8.951m`, oracle target abs regret max `0.613m`였다.
dashboard는 `runtime_guard_policy_catalog_robust_family_loso` 행과
`runtime_guard_policy_catalog_robust_family_loso_validation_only`,
`runtime_calibration_required_export_block_smoke`, `method_family_potential`,
`affine_family_profile_selection`, `affine_family_runtime_feature_selector` 행도 기록한다.
strict LOSO는 엄격 기준 14/18 PASS, validation-only 기준 17/18 PASS,
target-window max abs worst-case `7.377m`이며, full catalog runtime 추천을 대체하지
않고 coverage gap 추적에 사용한다. `method_family_potential`은 현재 메인 sweep에서
oracle-style affine family potential 20/20 PASS, target worst-case `9.308m`를
보여준다. `affine_family_runtime_feature_selector`의 robust bbox fallback은 truth-free
runtime feature 선택만으로 20/20 PASS, target worst-case `9.423m`까지 확인했다. 다음
탐색은 full policy catalog보다 낮은 worst-case를 내는 runtime truth-free 선택 정책,
LOSO 조건에서의 coverage gap 축소, affine family 내부 profile 선택 안정화,
policy/affine portfolio의 raw-feature rule 확장이다.
dashboard의 좌표 계약은 stereo-center OpenCV optical
`RDF(right,down,forward)`이고 public topic 계약은 StereoDistance
`FRD_range(forward,right,down,range)`이다.

minimal set의 배포 label 매핑과 calibration-replay 자동 선택 중 어느 쪽이 더
안정적인지는 sweep 전체에 대해 감사할 수 있다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_profile_selection_strategies.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set/profile_selection_strategy_audit
```

`profile_selection_strategy_runs.csv`는 각 scenario/seed에서 label 기반 source
profile, calibration-replay가 선택한 source profile, 두 방법의 validation RMSE와
1852m target error를 나란히 기록한다.
`profile_selection_strategy_scenarios.csv`는 scenario별 pass count, 두 선택이
일치한 횟수, calibration-replay가 label 선택보다 낮은 validation RMSE를 낸
횟수를 요약한다. 이 감사도 calibration-time truth로 선택 전략을 평가하는
offline 절차이며, runtime truth 입력을 허용하지 않는다.
기본 후보 범위는 deployment-ready minimal set이다. 더 넓은 offline 진단에서는
`--candidate-source-mode all_catalog`를 주면 catalog의 모든 validated scenario
profile을 후보로 replay하여, minimal set 밖의 profile이 현재 보정 비행에 더
맞는지 확인할 수 있다.

검증된 catalog profile 둘 이상이 같은 runtime profile family라면, 보정 비행
truth로 profile 사이 alpha를 탐색해 blended runtime profile 후보도 만들 수
있다. 이 방법은 runtime에서 truth를 쓰지 않으며, 선택된 blend는 일반
`estimated_to_truth_sim3` 또는 `estimated_to_truth_disparity_poly` profile JSON으로
저장된다. Sim(3)는 scale/rotation/translation을 보간하고, inverse-depth
ray-scale 계열은 `inverse_depth_coefficients_high_to_low`를 보간한다. 단, blend는
전체 validation RMSE를 조금 희생하고 1852m target error를 크게 줄이는
target-guard 후보가 될 수 있으므로, balanced 추천을 자동 대체하지 말고
`profile_blend_candidates.csv`와 promotion check 결과를 함께 확인한다.
기본 blend 후보도 minimal set의 source profile pair로 제한된다. sensor-error
regime 식별을 더 공격적으로 탐색하려면 `--candidate-source-mode all_catalog`를
추가해 모든 validated catalog profile pair를 blend 후보로 만들 수 있다.
이 경우 조합 수가 늘어나므로 offline audit/보정 단계에서만 사용한다.
`--selection-policy balanced|validation_best|target_guard|simple_stable|replay_best`로
어떤 policy row를 `--output-profile`에 export할지 명시할 수 있다. post-sweep
deployment audit가 `balanced`를 추천했다면 같은 값을 사용해 active profile을
만든다.

```bash
Simulation/.venv/bin/python Simulation/scripts/explore_stereodistance_profile_blends.py \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/scenario_profile_catalog_minimal_set.json \
  --input-csv Simulation/results/rtk_stereo_robustness_sweep_minimal_set/combined_sensor_stress/seed_20260614/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set/profile_blend_exploration \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep_minimal_set/active_runtime_profile_blend.json
```

출력은 `profile_blend_candidates.csv`,
`profile_blend_pareto_frontier.csv`,
`profile_blend_policy_recommendations.csv`,
`profile_blend_selection.json`, `profile_blend_summary.txt`,
`blend_candidate_profiles/*.json`이다. `profile_blend_policy_recommendations.csv`는
같은 alpha 후보를 `balanced`, `validation_best`, `target_guard`,
`simple_stable` 기준으로 다시 고른다. 선택된 blend profile도 실제 적용 전
`stereodistance_profile_promotion_check.py`로 schema/load/runtime input 계약을
검증한다.

blend policy가 특정 보정 비행 하나에만 맞는지, sweep 전체 scenario/seed에서도
안정적으로 동작하는지는 별도 audit로 확인한다. 이 audit는 각 seed의
calibration CSV에 대해 blend 탐색을 반복하고, `balanced`, `validation_best`,
`target_guard`, `simple_stable` 정책별 replay 결과를 집계한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_profile_blend_strategies.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set/profile_blend_strategy_audit
```

출력은 `blend_strategy_policy_runs.csv`,
`blend_strategy_policy_summary.csv`, `blend_strategy_summary.json`,
`blend_strategy_summary.txt`, `per_run_blend_exploration/*`이다.
`blend_strategy_policy_summary.csv`는 정책별 validation RMSE 평균/최대값,
1852m target-window error 평균/최대값, split-stability 최소 개선량,
선택된 alpha 집합을 기록한다. 이 절차도 calibration-time truth로 정책을
평가하는 offline audit이며, 선택된 runtime profile이 truth를 입력으로 쓰는
것을 허용하지 않는다.

monotone inverse-depth runtime sweep에서 all-catalog inverse-depth blend를
추가로 감사하려면 다음처럼 실행한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_profile_blend_strategies.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/profile_blend_strategy_audit_all_catalog \
  --candidate-source-mode all_catalog \
  --alphas 0,0.25,0.5,0.75,1
```

2026-06-16 결과에서 all-catalog `target_guard` blend는 18/18 PASS, 1852m
target abs mean/max `0.175/1.471m`, target max abs worst-case `7.744m`로 기존
runtime selector의 target max abs worst-case `9.681m`보다 보수적이었다. 대신
validation RMSE mean/max는 `1.277/2.186m`로 기존 runtime selector `1.089/1.676m`
보다 높다. 따라서 이 후보는 1852m target-window 정확도를 우선하는
calibration-time offline export 후보로 보존하고, 일반 runtime 자동 추천은 여전히
validation 안정성이 더 좋은 `transfer_risk_fallback`을 기본으로 둔다.

해당 target-guard blend를 실제 active runtime profile 후보로 export하고
StereoDistance loader/promotion check까지 확인하려면 다음을 실행한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/explore_stereodistance_profile_blends.py \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog_minimal_set.json \
  --input-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/combined_sensor_stress/seed_20260613/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_profile_export_target_guard_blend/blend_profile_export \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_profile_export_target_guard_blend/active_runtime_profile_target_guard_blend.json \
  --candidate-source-mode all_catalog \
  --selection-policy target_guard \
  --alphas 0,0.25,0.5,0.75,1

MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/stereodistance_profile_promotion_check.py \
  --profile-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_profile_export_target_guard_blend/active_runtime_profile_target_guard_blend.json \
  --sample-csv Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/combined_sensor_stress/seed_20260613/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_profile_export_target_guard_blend/promotion_check \
  --split validation \
  --max-samples 50
```

2026-06-16 export 결과는 `intrinsic_scale_stress -> combined_sensor_stress`,
alpha `0.5`의 `estimated_to_truth_disparity_poly` profile이었다. Promotion check는
PASS였고, StereoDistance node loader가 profile을 로드했으며 validation sample 50개
적용에서 invalid sample은 0개였다. 해당 active profile의 보정 비행 replay 성능은
validation RMSE `2.186m`, 1852m target abs `0.016m`, target max abs `2.927m`이다.

단일 active profile로 고정한 뒤 다른 scenario/seed에도 전이되는지는 별도 replay로
확인한다. 이 감사는 profile 하나를 모든 calibration CSV validation split에 적용해
raw 대비 RMSE 개선, target-window 존재, 1852m target abs/max 10m 이하를 확인한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_active_profile_transfer.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --profile-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_profile_export_target_guard_blend/active_runtime_profile_target_guard_blend.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_profile_export_target_guard_blend/transfer_replay \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

출력은 `active_profile_transfer_replay.csv`,
`active_profile_transfer_summary.txt/json`,
`active_profile_transfer_summary.png`, `per_run_predictions/*`이다. 2026-06-16 결과는
17/18 PASS, validation RMSE mean/max `2.018/3.804m`, target max abs worst-case
`11.691m`였다. 실패는 `bbox_outlier_stress/seed_20260613`에서 target max error가
10m를 넘은 경우였다. 따라서 target-guard blend는 보정 비행별 offline 선택 후보로는
유효하지만, 이 export profile 하나를 universal runtime profile로 고정하는 것은
아직 권장하지 않는다.

여러 exported active runtime profile을 universal fixed profile 후보로 동시에
비교하려면 portfolio audit를 실행한다. 이 감사는 각 profile을 모든 scenario/seed
validation split에 고정 적용하고, pass count, 1852m target worst-case, validation
RMSE worst-case 순으로 순위를 매긴다. Profile apply는 raw stereo-center RDF와
measured disparity만 사용하며, truth는 offline replay metric 계산에만 사용한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_active_profile_portfolio.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_profile_portfolio_audit \
  --profile runtime_guard_smoke=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/operating_mode_export_smoke_runtime_guard/active_runtime_profile.json \
  --profile calibration_smoke=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/operating_mode_export_smoke_calibration/active_runtime_profile.json \
  --profile deployment_audit=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_runtime_profile_from_deployment_audit.json \
  --profile transfer_risk_single=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_transfer_risk_single_profile_smoke/active_runtime_profile.json \
  --profile target_guard_blend=Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/active_profile_export_target_guard_blend/active_runtime_profile_target_guard_blend.json \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

출력은 `active_profile_portfolio_runs.csv`,
`active_profile_portfolio_summary.csv`,
`active_profile_portfolio_recommendation.json`,
`active_profile_portfolio_summary.txt/png`, `per_profile_transfer_replay/*`이다.
2026-06-16 결과에서는 고정 단일 active profile 후보 중 18/18 PASS가 없었다.
partial best는 `target_guard_blend`로 17/18 PASS, target max abs worst-case
`11.691m`, validation RMSE max `3.804m`였다. `deployment_audit`,
`runtime_guard_smoke`, `transfer_risk_single`은 각각 15/18 PASS였고 target max
abs worst-case는 `9.681m`였지만 일부 run에서 raw 대비 RMSE 개선 조건을
만족하지 못했다. 이 결과는 universal fixed profile 하나보다 raw feature 기반
runtime selector/guard 계층이 현재 문제 설정에 더 적합하다는 근거로 기록한다.

runtime selector의 일반화 범위를 더 엄격히 확인하려면 target scenario 전체를
raw-feature library와 candidate source set에서 제외하는 leave-one-scenario-out
audit를 실행한다. 기존 `include_target_in_centroids=false`는 같은 CSV만 제외하고
같은 scenario의 다른 seed를 학습 library에 남겨 두지만,
`--exclude-target-scenario-from-library`는 target scenario를 완전히 제거한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_runtime_measurement_guard_policies.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog_minimal_set.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_policy_audit_loso \
  --candidate-source-modes all_catalog \
  --policies transfer_risk_fallback,centroid_knn_veto_transfer_risk,knn_transfer_risk,nearest_centroid \
  --exclude-target-scenario-from-library \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

2026-06-16 LOSO 결과에서는 `all_catalog+transfer_risk_fallback`이 여전히 가장
낮은 score였지만 14/18 PASS, validation RMSE mean/max `1.580/2.651m`, target max
abs worst-case `10.055m`로 PASS 기준을 만족하지 못했다. 따라서 현재 18/18 PASS
추천은 catalog에 같은 stress family가 존재하는 보정/검증 sweep 내부 조건에서의
결과로 해석해야 하며, 완전히 새로운 sensor-error regime까지 보장하려면 추가
raw feature, 실제 BBox geometry, 또는 calibration-time replay export가 필요하다.

같은 LOSO 조건에서 transfer-risk threshold 자체도 sweep할 수 있다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/sweep_runtime_measurement_guard_thresholds.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog_minimal_set.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_transfer_threshold_sweep_loso \
  --candidate-source-mode all_catalog \
  --rmse-limits-m 0.9,1.1,1.3,1.5,1.8 \
  --target-limits-m 1.5,2.0,2.5,3.0,4.0 \
  --exclude-target-scenario-from-library \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

이 stricter threshold sweep에서는 `rmse_limit=1.8`, `target_limit=2.5`가
15/18 PASS로 기본 LOSO runtime selector보다 1개 run을 더 통과했다. 1852m target
max abs worst-case도 `9.566m`로 PASS 기준 안에 들어왔지만, 남은 실패 3개는 모두
`intrinsic_scale_stress`이며 실패 이유는 raw 대비 validation/train RMSE 개선이
없다는 것이다. 즉 더 느슨한 transfer cluster는 target-window error를 줄일 수
있지만, intrinsic-scale family 자체가 catalog에서 빠진 조건에서는 보정이 실제로
raw보다 나아졌다고 말할 수 없다.

같은 LOSO 조건에서 calibration-time truth replay selector도 확인한다. 이 감사는
보정 비행 truth로 후보 profile을 replay해 선택하지만, target scenario profile은
후보에서 제거한다. 선택된 runtime profile 자체는 여전히 raw RDF/disparity만
소비한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_profile_selection_strategies.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/profile_selection_strategy_audit_all_catalog_loso \
  --candidate-source-mode all_catalog \
  --exclude-target-scenario-from-candidates \
  --split validation \
  --stability-splits train,validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

2026-06-16 결과는 15/18 PASS였다. `bbox_outlier_stress`는 target scenario 후보
없이도 3/3 PASS로 회복됐지만, `intrinsic_scale_stress`는 0/3 PASS였다. 해당
세 seed의 top candidate는 모두 `nominal`이었고 실패 이유는
`no_positive_range_rmse_improvement`였다. 즉 baseline/focal scale mismatch가
새 regime으로 들어오면 같은 family의 보정 profile 또는 실제 calibration-time
profile export가 필요하며, 다른 regime profile로 대체하는 것은 충분하지 않다.

full-catalog calibration replay와 LOSO calibration replay를 scenario별로 비교해
어떤 stress family가 catalog coverage에 필수인지 요약하려면 coverage-gap
후처리를 실행한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/summarize_stereodistance_loso_coverage_gaps.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/loso_coverage_gap_summary
```

출력은 `loso_coverage_gap_scenarios.csv`,
`loso_coverage_gap_summary.json/txt`, `loso_coverage_gap_pass_counts.png`이다.
2026-06-16 결과에서 full gap scenario는 `intrinsic_scale_stress` 하나였다.
full catalog에서는 3/3 PASS, LOSO에서는 0/3 PASS, runtime LOSO에서도 0/3 PASS다.
나머지 scenario는 target family profile을 제거해도 calibration replay LOSO가
3/3 PASS였다. 따라서 다음 후보 탐색은 arbitrary single profile보다
baseline/focal scale mismatch를 식별하거나, 해당 regime에서 별도
calibration-time profile을 export하는 방향이 우선이다.

policy catalog runtime LOSO 결과를 coverage-gap summary에 직접 포함하려면
`--runtime-loso-dir`에
`runtime_measurement_guard_audit_policy_catalog_robust_family_loso`를 넘긴다. 현재
저장된 policy catalog runtime LOSO summary에서는 `intrinsic_scale_stress`가 여전히
0/3 PASS인 full-gap family이고, `bbox_outlier_stress`는 target-window error는
`disparity_noise_stress` fallback으로 10m 이내에 들어오지만 train-side guard 때문에
2/3 PASS로 남아 있다. 이는 runtime 선택 feature의 좌표 계약은 그대로
stereo-center RDF를 유지하면서, 평가용 truth/replay guard만 더 엄격해진 결과다.

coverage gap family가 runtime raw feature만으로 감지 가능한지 확인하려면
feature separability audit를 실행한다. 이 절차는 gap label은 offline
coverage-gap 결과에서 가져오지만, classifier 후보 feature는 raw RDF/disparity/image
center 집계값만 사용한다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_stereodistance_gap_feature_separability.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/gap_feature_separability_audit
```

출력은 `gap_feature_separability.csv`,
`gap_feature_separability_summary.json/txt`,
`gap_feature_separability_top_features.png`이다. 2026-06-16 결과에서
`intrinsic_scale_stress`는 `feature_raw_forward_mean_m >= 1145.212m` 기준으로
현재 synthetic sweep 18개 run을 100% 분리했다. effect size는 `5.301`이다.
`feature_raw_range_mean_m`, `feature_raw_forward_std_m`,
`feature_raw_range_std_m`도 100% 분리했다. 다만 raw forward/range 통계는
trajectory 설계에 민감하므로, 이 결과는 곧바로 일반 runtime detector로 확정하지
않고 “calibration required” guard 후보로 기록한다. 실제 비행에서는 동일 trajectory
coverage 또는 별도 trajectory-normalized scale feature가 필요하다.

runtime selector는 intrinsic-scale regime 후보를 더 물리적으로 보기 위해
`measured_disparity_px * raw_forward_m`를 nominal `f_px * baseline_m`과 비교하는
product-ratio 계열 feature도 기록한다. 이 feature는 stereo depth 식
`Z ~= fB/d`의 scale 일관성 proxy이며 runtime truth를 쓰지 않는다. 단순 평균
ratio는 15/18 수준이었지만, robust 통계인
`feature_disparity_forward_product_ratio_p10_to_nominal >= 1.003744`와
`feature_disparity_forward_product_ratio_median_to_nominal >= 1.003915`는 현재
synthetic sweep의 `intrinsic_scale_stress`를 18/18로 분리했다. raw forward/range
통계보다 effect size는 낮지만 trajectory 해석 의존성이 작아 intrinsic-scale
gap guard 후보로 더 적합하다.

이 신호를 실제 selector에 연결한
`intrinsic_scale_gap_transfer_risk` 정책도 비교한다. 이 정책은 product-ratio p10이
감사에서 정한 threshold를 넘고 `intrinsic_scale_stress` source profile이 catalog에
있으면 해당 profile을 우선 선택하고, 없으면 기존 transfer-risk fallback으로
돌아가며 `intrinsic_scale_gap_detected_but_source_unavailable`을 남긴다.

```bash
MPLCONFIGDIR=/tmp/matplotlib Simulation/.venv/bin/python Simulation/scripts/audit_runtime_measurement_guard_policies.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/scenario_profile_catalog_minimal_set.json \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_monotone_inverse_depth_runtime/runtime_measurement_guard_policy_audit \
  --candidate-source-modes all_catalog \
  --policies transfer_risk_fallback,intrinsic_scale_gap_transfer_risk,centroid_knn_veto_transfer_risk,knn_transfer_risk,nearest_centroid \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30 \
  --target-error-limit-m 10
```

2026-06-16 재실행 결과에서 일반 catalog 조건은 기존
`transfer_risk_fallback`과 새 product-ratio guard 정책이 모두 18/18 PASS,
validation RMSE mean/max `1.089/1.676m`, target max abs worst-case `9.681m`로
동일했다. 새 정책은 `intrinsic_scale_stress` 3개 run을 명시적으로
`intrinsic_scale_stress` profile로 보냈지만, 성능 이득은 없어 복잡도 penalty
때문에 최종 runtime 추천은 여전히 `transfer_risk_fallback`이다. LOSO 조건에서는
새 정책이 intrinsic gap을 감지하지만 source profile이 없어 14/18 PASS에 머물렀다.
따라서 product-ratio guard는 correction 자체보다 “해당 scale family profile이
catalog/calibration에 필요하다”는 runtime-safe 진단 신호로 해석한다.

참고 방법론과 현재 적용 가능 범위:

- Kabsch/Umeyama/orthogonal Procrustes 계열은 paired estimated RDF point와
  truth RDF point 사이의 전역 rotation/translation/scale을 닫힌 형태로 맞추는
  기준 방법이다. 현재 Sim(3), scale+rotation 후보가 이 계열이며, 보정 비행
  truth를 사용하는 offline calibration 단계에만 해당한다.
- Bundle adjustment 계열은 이미지 reprojection error를 직접 최소화해 camera
  pose/intrinsic/3D point를 동시에 조정하는 방법이다. 현재 synthetic CSV는
  좌/우 이미지 feature track이나 full bbox geometry가 아니라 중심/disparity와
  raw RDF만 보존하므로, 지금 pipeline에서는 residual disparity/RDF correction로
  근사한다. 향후 좌/우 image topic 또는 YOLO BBox topic을 저장하면 reprojection
  residual 기반 후보를 추가할 수 있다.
- Markerless stereo extrinsic calibration 계열은 자연 feature의 epipolar error로
  stereo extrinsic을 online/offline refinement하는 방향이다. 이 방법은 좌/우
  영상 feature correspondence가 필요하므로 현재 RTK synthetic mode의
  raw RDF/disparity-only CSV에는 바로 적용하지 않는다.
- LiDAR-camera targetless calibration, mutual-information registration, ICP/GMM
  point-set registration은 scene geometry 또는 dense point cloud가 있을 때
  유효한 비교군이다. 현재 문제는 target drone의 paired RTK truth와 stereo
  estimate가 있는 supervised correction 문제이므로, 우선순위는 낮지만 실제
  depth/point-cloud 센서가 추가되면 후보군으로 확장할 수 있다.

single profile selection audit와 blend audit를 둘 다 실행한 뒤에는 다음
후처리로 배포 관점의 최종 offline 추천을 만들 수 있다. 추천 strategy 이름에는
후보 범위가 포함된다. 예를 들어 deployment-ready set만 썼으면
`single_profile_calibration_replay_minimal_set`, 전체 catalog를 썼으면
`single_profile_calibration_replay_all_catalog`처럼 기록된다.

전체 후처리 pipeline을 한 번에 실행하려면 다음 wrapper를 사용한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_stereodistance_post_sweep_deployment_audit.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set/post_sweep_deployment_audit \
  --candidate-source-mode all_catalog
```

이 wrapper는 profile selection audit, blend audit, deployment audit strategy
summary를 순서대로 실행하고
`post_sweep_deployment_audit_manifest.json`에 최종 추천 전략/정책과 산출물
경로를 기록한다. 빠른 진단만 필요하면 `--skip-blend-audit`로 blend 탐색을
건너뛸 수 있다.

```bash
Simulation/.venv/bin/python Simulation/scripts/summarize_stereodistance_deployment_audits.py \
  --sweep-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set/deployment_audit_strategy_summary
```

출력은 `deployment_audit_strategy_candidates.csv`,
`deployment_audit_strategy_recommendation.json`,
`deployment_audit_strategy_summary.txt`이다. 이 요약은 validation RMSE
평균/최대, 1852m target-window error 평균/최대를 함께 점수화해 단일
calibration replay profile selection과 blend policy 중 어느 쪽을 offline
배포 후보로 둘지 비교한다. 같은 pass count에서 점수 차이가 기본 0.01m 이내면
과적합과 운용 복잡도를 피하기 위해 더 단순한 single-profile 전략을 우선한다.
여기서도 truth는 audit/selection 단계에서만 사용되며, 실제 runtime profile은
raw RDF/disparity와 고정 보정 파라미터만 사용한다.

post-sweep deployment audit의 추천을 새 보정 비행 CSV에 바로 적용해 active
StereoDistance runtime profile을 export하려면 다음 wrapper를 사용한다. 추천이
single profile이면 calibration replay selector를, blend policy이면 blend
explorer를 호출하고, 마지막에 promotion check로 schema/load/runtime input
계약을 확인한다. 이 단계에서도 CSV truth는 offline 선택/검증에만 쓰이며,
export된 profile은 raw stereo-center RDF와 disparity만 runtime 입력으로 쓴다.

```bash
Simulation/.venv/bin/python Simulation/scripts/export_stereodistance_profile_from_deployment_audit.py \
  --deployment-audit-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/post_sweep_deployment_audit/deployment_audit_strategy_summary/deployment_audit_strategy_recommendation.json \
  --catalog-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/scenario_profile_catalog/manifest.json \
  --minimal-set-json Simulation/results/rtk_stereo_robustness_sweep_minimal_set/scenario_profile_catalog_minimal_set.json \
  --input-csv Simulation/results/rtk_stereo_robustness_sweep_minimal_set/combined_sensor_stress/seed_20260614/calibration_samples.csv \
  --output-dir Simulation/results/rtk_stereo_robustness_sweep_minimal_set/active_profile_export_from_deployment_audit \
  --output-profile Simulation/results/rtk_stereo_robustness_sweep_minimal_set/active_runtime_profile_from_deployment_audit.json
```

출력은 `deployment_audit_profile_export_summary.json`,
`deployment_audit_profile_export_summary.txt`, `promotion_check/*`, 그리고
선택 경로별 `single_profile_export/*` 또는 `blend_profile_export/*`이다.
추천과 다른 scope/policy를 강제로 시험하려면 `--candidate-source-mode`와
`--selection-policy`를 명시한다.

pipeline의 train/validation split은 `--target-range-m`과
`--target-window-m`을 기준으로 target-window sample을 stratify한다.
target-window sample이 2개 이상 있으면 train과 validation 양쪽에 최소
1개 이상 남겨 1852m PASS 조건을 실제 validation set에서 평가할 수 있게
한다. target-window sample 자체가 부족한 입력 CSV에서는
`method_stability_gate.csv`가 `*_target_window_missing_in_some_seeds` 사유로
PASS를 막는다.

synthetic mode는 기본적으로 `--synthetic-target-oversample-count 12`를
사용해 1852m target-window 근방 sample을 추가로 배치한다. 이 값은
multi-seed wrapper에도 같은 이름의 옵션으로 전달할 수 있다. 목적은
보정법 점수를 유리하게 만드는 것이 아니라, 1852m PASS 조건이 validation
set에서 충분한 표본으로 평가되도록 하는 것이다. 실제 CSV mode에서는 입력
비행 데이터 자체가 target-window 관측을 충분히 포함해야 한다.

실제 보정 비행 CSV를 후처리하려면, `/fmu/out/vehicle_local_position`,
좌/우 카메라 image 또는 synthetic BBox 중심, YOLO BBox 결과를 먼저 명시적
축 필드가 있는 CSV로 저장한 뒤 실행한다.

라이브 ROS 2 토픽에서 calibration CSV를 직접 만들려면 다음 recorder를
사용한다. 이 recorder는 PX4 `VehicleLocalPosition.x/y/z`를 NED
`north/east/down`으로만 해석하고, StereoDistance
`/stereo/estimated_position`은 public FRD+range
`forward/right/down/range`로 해석한 뒤 CSV에는 named RDF/NED 필드만 쓴다.
Gazebo synthetic truth를 같이 기록해야 하는 경우에는
`--sim-truth-topic /ground_truth_position`을 추가한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/ros2_calibration_csv_recorder.py \
  --output-csv Simulation/results/calibration_flight/telemetry.csv \
  --truth-topic /fmu/out/vehicle_local_position \
  --estimated-topic /stereo/estimated_position \
  --detections-topic /stereo/detections \
  --disparity-topic /stereo/disparity \
  --hz 10
```

Gazebo/PX4/StereoDistance launch와 함께 calibration CSV를 기록하려면 launch
인자로 recorder를 켠다. 실제 PX4 토픽 대신 Gazebo synthetic truth
`/ground_truth_position`을 calibration truth로 기록하려면
`calibration_sim_truth_topic:=/ground_truth_position`을 지정한다.

평소 운용 스크립트를 쓸 때는 다음처럼 실행한다.

```bash
./run_operator_view.sh \
  --record-calibration-csv Simulation/results/calibration_flight/telemetry.csv \
  --calibration-sim-truth
```

기록 전에 필수 ROS 토픽이 떠 있는지 확인하려면 별도 터미널에서 readiness
check를 실행한다. PX4 truth 대신 Gazebo synthetic truth를 쓰는 경우
`--sim-truth-topic /ground_truth_position`을 지정한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/calibration_topic_readiness_check.py \
  --sim-truth-topic /ground_truth_position \
  --output-dir Simulation/results/calibration_flight/topic_readiness
```

```bash
ros2 launch stereo_ground_sim stereo_ground_sim.launch.py \
  record_calibration_csv:=true \
  calibration_csv_path:=results/calibration_flight/telemetry.csv \
  calibration_truth_topic:=/fmu/out/vehicle_local_position \
  calibration_sim_truth_topic:=/ground_truth_position
```

```bash
Simulation/.venv/bin/python Simulation/scripts/rtk_stereo_calibration_pipeline.py \
  --mode csv \
  --input-csv Simulation/results/calibration_flight/telemetry.csv \
  --output-dir Simulation/results/rtk_stereo_calibration_from_csv
```

CSV 품질을 먼저 확인하려면 preflight를 실행한다. 이 단계는 pipeline loader와
같은 named NED/RDF/FRD 계약으로 usable row 수, skip 사유, 1852m target
window 표본 수를 보고한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/calibration_csv_preflight.py \
  --input-csv Simulation/results/calibration_flight/telemetry.csv \
  --output-dir Simulation/results/calibration_flight/preflight \
  --target-range-m 1852 \
  --target-window-m 30 \
  --min-target-count 3
```

기록 CSV에서 후보 탐색, runtime profile export, offline replay, PASS gate용
`metrics.json`까지 한 번에 만들려면 다음 runner를 사용한다. 실제 CSV가 아직
없으면 `--mode synthetic`으로 같은 경로를 smoke test할 수 있다.
CSV mode에서는 runner가 위 preflight를 자동으로 실행하고 결과를
`preflight/`와 `metrics.json`에 포함한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/run_estimated_to_truth_calibration_exp.py \
  --mode csv \
  --input-csv Simulation/results/calibration_flight/telemetry.csv \
  --output-dir Simulation/results/estimated_to_truth_calibration_exp
```

운용 스크립트로 기록한 CSV를 바로 후처리하려면 wrapper를 사용할 수 있다.
CSV 경로를 생략하면 `results/calibration_flight` 아래 최신 CSV를 찾는다.

```bash
./run_estimated_to_truth_from_csv.sh \
  results/calibration_flight/telemetry.csv
```

후처리 결과의 runtime profile을 측정 실행에 적용하려면 다음 wrapper를
사용한다. profile 경로를 생략하면 `results/` 아래 최신
`sim3_runtime_profile.json`을 찾는다.

```bash
./run_operator_view_with_calibration_profile.sh
```

명시 profile을 쓰려면:

```bash
./run_operator_view_with_calibration_profile.sh \
  --profile-json results/estimated_to_truth_calibration_exp/sim3_runtime_profile.json
```

Runner 출력:

- pipeline 기본 산출물: `method_comparison.csv`, `validation_predictions.csv`,
  `target_1852m_stats.csv`, PNG 3종, `summary.txt`
- StereoDistance runtime profile: `sim3_runtime_profile.json`
- runtime profile replay: `profile_replay/profile_replay_*.csv|txt|png`
- StereoDistance promotion check:
  `promotion_check/promotion_check.json|txt|csv`
- PASS gate/audit 산출물: `metrics.json`, `promotion_summary.txt`,
  `result_audit.json`, `result_audit.txt`

CSV 입력은 최소한 다음 중 하나의 truth 계약과 raw RDF 또는 BBox/시차 계약을
가져야 한다.

- truth RDF: `truth_right_m`, `truth_down_m`, `truth_forward_m`
- recorded-style truth RDF aliases: `gt_right_m`, `gt_down_m`,
  `gt_forward_m`
- truth NED: `truth_north_m`, `truth_east_m`, `truth_down_ned_m`
- raw RDF: `raw_right_m`, `raw_down_m`, `raw_forward_m`
- recorded-style raw RDF aliases: `estimated_right_m`, `estimated_down_m`,
  `estimated_forward_m`
- 또는 triangulation 입력: `left_u_px`, `left_v_px`, `measured_disparity_px`
- `measured_disparity_px`가 없으면 `left_cx_px`와 `right_cx_px`의 차이를
  measured disparity로 사용할 수 있다.

비교하는 후보 보정법:

- `global_rotation`: `p_corr = R_corr * p_raw`
- `global_scale_rotation`: `p_corr = s * R_corr * p_raw`
- `sim3`: `p_corr = s * R_corr * p_raw + t`
- `diagonal_affine_rdf`, `identity_shrunk_diagonal_affine_rdf_lambda0p01/1`:
  RDF 각 축별 `p_corr_axis = gain_axis * p_raw_axis + bias_axis`만 fitting하는
  6자유도 후보. 목적은 bbox center와 target origin 격차 또는 축별 scale/bias가
  남는지 확인하는 것이며, shrink 후보는 gain을 identity에 묶어 과적합을
  줄인다. 출력은 `estimated_to_truth_affine_rdf` profile이고 PASS 기준은 다른
  후보와 동일하게 validation RMSE 개선, 1852m 오차, stability gate이다.
- `affine_rdf`, `ridge_affine_rdf`: RDF 벡터에서 `p_corr = A * p_raw + t`를
  직접 fitting하는 12자유도 affine 후보. Sim(3)보다 표현력이 크므로
  validation/1852m/stability gate를 통과하는지 확인해야 한다. StereoDistance
  runtime에서는 `estimated_to_truth_affine_rdf` profile로 적용할 수 있지만,
  과적합 위험 때문에 단일 run 성능이 아니라 multi-seed/robustness gate 통과를
  기준으로 승격한다.
- `affine_binned_residual_disparity`: affine RDF 보정 후 train set의
  disparity 구간별 forward residual median LUT를 만들고, runtime에서는
  measured disparity에 대해 선형 보간된 residual을 추가 적용한다.
- `affine_binned_residual_raw_forward`,
  `affine_binned_residual_raw_forward_bins11`: affine RDF 보정 후 train set의
  raw RDF forward 거리 구간별 forward residual median LUT를 만들고,
  runtime에서는 raw RDF forward 값으로 선형 보간된 residual을 추가 적용한다.
  disparity-key LUT가 focal/baseline scale stress에 민감한지 비교하기 위한
  후보이다.
- `affine_monotone_binned_residual_raw_forward`,
  `affine_monotone_binned_residual_raw_forward_bins11`: affine RDF 보정 후
  raw RDF forward 거리 구간별 residual median을 weighted PAVA isotonic
  regression으로 단조화한다. 거리 증가에 따라 보정 residual이 불안정하게
  뒤집히는 과적합을 줄이는 보수적 LUT 후보이다.
- `affine_range_normalized_binned_residual_raw_forward`,
  `affine_range_normalized_binned_residual_raw_forward_bins11`: affine RDF 보정
  후 forward residual 자체가 아니라 `residual / raw_forward` 비율을 구간별로
  fitting하고, runtime profile에는 raw RDF forward 거리별 meter residual LUT로
  저장한다. 거리 비례 scale/bias 성분이 클 때 raw-forward LUT보다 안정적인지
  비교하는 후보이다.
- `affine_inverse_depth_binned_residual_raw_forward`,
  `affine_inverse_depth_binned_residual_raw_forward_bins11`: affine RDF 보정 후
  `1 / truth_forward - 1 / affine_forward` inverse-depth delta를 구간별로
  fitting하고, runtime profile에는 raw RDF forward 거리별 meter residual LUT로
  저장한다. stereo의 `Z = fB / d` 관계에서 disparity scale/bias 오차가
  inverse depth 공간에서 더 단순해지는지 확인하는 후보이다.
- `affine_quadratic_residual_raw_forward_lut`: affine RDF 보정 후
  `residual = a * raw_forward^2 + b * raw_forward + c` 형태의 ridge polynomial을
  fitting하고, runtime profile에는 raw RDF forward 거리별 meter residual LUT로
  저장한다. 시차 bias로 인한 stereo depth 오차가 장거리에서 `Z^2`에 가깝게
  커지는 경우를 겨냥한 물리형 후보이다.
- `affine_quadratic_local_residual_raw_forward_lut`,
  `affine_quadratic_local_residual_raw_forward_lut_bins3`,
  `affine_quadratic_local_residual_raw_forward_lut_bins5`,
  `affine_quadratic_local_residual_raw_forward_lut_shrink025`,
  `affine_quadratic_local_residual_raw_forward_lut_shrink075`: affine RDF 보정 후
  2차 raw-forward residual로 큰 stereo depth bias를 먼저 설명하고, 남은
  residual-of-residual을 shrink된 구간별 median LUT로 더한다. runtime에는
  합쳐진 meter residual LUT 하나만 저장한다. 기본 이름은 local residual
  shrink `0.5`/7-bin이고, suffix 후보는 `0.25`, `0.75` shrink 강도와
  3/5/7-bin local LUT 복잡도를 grid-search한다.
- `affine_binned_residual_disparity_bins5/9/11`: 같은 affine+forward
  residual LUT 구조에서 bin 수를 grid-search하여 과소/과다 smoothing을
  validation 기준으로 비교한다.
- `affine_smoothed_binned_residual_disparity`,
  `affine_smoothed_binned_residual_disparity_bins11`: affine+forward residual
  LUT의 bin median을 global median 쪽으로 shrink하고 인접 bin smoothing을
  적용해 outlier/희소 bin 과적합을 줄이는 후보
- `affine_capped_binned_residual_disparity`,
  `affine_capped_binned_residual_disparity_bins11`: affine+forward residual
  LUT의 bin median을 train residual MAD와 absolute-residual quantile로
  winsorized cap 처리해 calibration stress 조건 전이에서 과도한 residual
  gain이 생기는지 비교하는 후보
- `affine_binned_rdf_residual_disparity`: affine RDF 보정 후 train set의
  disparity 구간별 residual median LUT를 RDF 각 축마다 만들고, runtime에서는
  measured disparity에 대해 축별 선형 보간 residual을 추가 적용한다.
- `trimmed_sim3`: 큰 residual sample을 제외한 뒤 Sim(3)를 다시 fitting하는
  consensus/least-trimmed 계열 강건 정합
- `ransac_sim3`: 여러 최소 subset에서 Sim(3) 가설을 만들고 consensus set으로
  다시 fitting하는 RANSAC 계열 강건 정합
- `huber_sim3`: residual norm에 Huber weight를 적용하는 iteratively
  reweighted Sim(3) 정합
- `tukey_sim3`: 큰 residual sample의 weight를 0에 가깝게 낮추는 Tukey
  biweight redescending M-estimator 기반 iteratively reweighted Sim(3) 정합
- `range_weighted_sim3`: 1852m 목표거리 근방 sample을 더 크게 반영하는
  weighted Sim(3) 정합
- `sim3_piecewise_raw_forward_bins3`,
  `sim3_piecewise_raw_forward_bins5`: raw RDF forward 거리 분위수 구간마다
  Sim(3) node를 따로 fitting하고, runtime에서는 raw_forward_m으로 인접 node의
  transformed RDF point를 선형 보간한다. 목적은 자세/origin/bbox-center 격차가
  거리 구간에 따라 달라질 때 단일 전역 Sim(3)보다 global estimated-to-truth
  오차를 줄일 수 있는지 확인하는 것이다. 입력 가정은 calibration-time PX4/RTK
  truth와 raw stereo-center RDF이고, runtime 출력 계약은 기존
  `estimated_to_truth_sim3` profile과 같은 RDF 보정값이다. PASS 판단은 다른
  후보와 동일하게 validation RMSE, 1852m target-window error, raw 대비 개선,
  모델 복잡도/안정성 gate를 기준으로 한다.
- `sim3_blended_piecewise_raw_forward_bins3_alpha025/050/075`,
  `sim3_blended_piecewise_raw_forward_bins5_alpha025/050/075`: 전역 Sim(3)
  출력과 raw_forward 구간별 Sim(3) 출력 RDF를 calibration-time에 정한 alpha로
  convex blend한다. 목적은 piecewise registration의 낮은 validation RMSE를
  유지하면서 구간별 자세/origin 보정 자유도를 줄여 실제 StereoDistance runtime
  profile로 더 안정적으로 배포 가능한지 확인하는 것이다. runtime 입력은 raw
  stereo-center RDF와 raw_forward_m뿐이며 PX4/RTK truth는 사용하지 않는다.
- `huber_range_weighted_sim3`: Huber 강건 가중과 목표거리 가중을 함께 적용한
  Sim(3) 정합
- `tukey_range_weighted_sim3`: Tukey biweight 강건 가중과 목표거리 가중을
  함께 적용한 Sim(3) 정합
- `soft_l1_sim3`, `cauchy_sim3`: `scipy.optimize.least_squares` robust loss로
  Sim(3) 파라미터를 직접 최적화하는 M-estimator 계열 정합
- `global_disparity_linear`: `d_corr = a * d_meas + b`
- `global_disparity_quadratic`: `d_corr = a2 * d_meas^2 + a1 * d_meas + b`
- `global_inverse_depth_linear`: `1 / forward_corr = a * (1 / raw_forward) + b`
  형태로 raw RDF ray를 재스케일한다. stereo의 장거리 depth error가 range보다
  inverse depth에서 더 선형일 수 있다는 기하 가정을 시험하는 저복잡도 후보이며,
  runtime에서는 truth 없이 raw stereo-center RDF만 사용한다.
- `inverse_depth_linear_then_sim3`: 위 inverse-depth ray 재스케일 뒤 Sim(3)로
  자세/origin/scale 오차를 정합한다. runtime profile에는 inverse-depth
  coefficients와 Sim(3) 파라미터만 저장되며, raw RDF와 고정 파라미터만 사용한다.
- `huber_global_inverse_depth_linear`, `huber_inverse_depth_linear_then_sim3`:
  `1/raw_forward -> 1/truth_forward` 선형 fit에 Huber IRLS를 적용해 일부
  보정 비행 sample의 disparity/bbox outlier 영향이 전체 inverse-depth 계수를
  끌고 가지 못하게 한다. 목적은 저복잡도 inverse-depth 후보의 장거리 안정성을
  유지하면서 calibration-time 이상치 민감도를 낮추는 것이다. 출력은 기존
  inverse-depth coefficients 또는 inverse-depth+Sim(3) profile이고, runtime
  입력은 raw stereo-center RDF와 고정 파라미터뿐이다.
- `target_weighted_global_inverse_depth_linear`,
  `target_weighted_inverse_depth_linear_then_sim3`: inverse-depth ray correction
  fitting에서 1852m 근방 train sample에 더 큰 가중치를 준다. target 가중치는
  calibration-time fitting에만 쓰이고, runtime profile은 raw RDF와 고정
  inverse-depth coefficients 및 선택적으로 Sim(3) 파라미터만 사용한다.
- `theil_sen_global_inverse_depth_linear`,
  `theil_sen_inverse_depth_linear_then_sim3`,
  `target_weighted_theil_sen_global_inverse_depth_linear`,
  `target_weighted_theil_sen_inverse_depth_linear_then_sim3`: inverse-depth
  선형 mapping을 least-squares 대신 Theil-Sen median-slope 추정으로 fitting한다.
  목적은 bbox/disparity outlier가 일부 섞인 보정 비행에서 장거리 inverse-depth
  계수가 한두 샘플에 끌리는지 확인하는 것이다. target-weighted 변형은 1852m
  근방 sample weight를 median-slope와 median residual 계산에 반영한다. 이 계열도
  calibration-time truth로 계수만 추정하고, runtime 입력은 raw stereo-center
  RDF(`right,down,forward`)와 고정 inverse-depth coefficients, 선택적 Sim(3)
  파라미터뿐이다.
- `target_weighted_huber_global_inverse_depth_linear`,
  `target_weighted_huber_inverse_depth_linear_then_sim3`: 1852m 근방 sample
  가중과 Huber IRLS를 함께 적용한다. 이 계열은 target-window 성능을 올리되
  sparse/outlier sample 하나에 profile이 과도하게 맞춰지는지를 validation split,
  1852m 근방 error, train/validation gap, 축별 RMSE regression으로 검증한다.
- `target_weighted_global_inverse_depth_linear_sigma060/250`,
  `target_weighted_inverse_depth_linear_then_sim3_sigma060/250`: target-window
  weighting의 거리 폭을 grid-search한다. 기본 target-weighted 후보는
  `sigma=120m`이고, `060`은 더 좁은 1852m 집중, `250`은 더 넓은 거리 안정성을
  시험한다. 선택은 validation RMSE, 1852m error, train/validation gap, 복잡도
  기준으로만 이루어진다.
- `target_shrunk_global_inverse_depth_linear_shrink025/040/050/060/075`,
  `target_shrunk_inverse_depth_linear_then_sim3_shrink025/040/050/060/075`: global
  inverse-depth coefficients와 target-weighted coefficients를 25/40/50/60/75% 비율로
  convex blend한다. 목적은 1852m target-window 개선과 validation 전체 안정성
  사이의 trade-off를 자동 탐색하는 것이다. shrink 값도 calibration-time
  후보 선택에만 쓰이며 runtime 입력 계약은 동일하다.
- `target_shrunk_huber_global_inverse_depth_linear_shrink025/040/050/060/075`,
  `target_shrunk_huber_inverse_depth_linear_then_sim3_shrink025/040/050/060/075`: global
  Huber inverse-depth coefficients와 target-weighted Huber coefficients를
  convex blend한다. 목적은 Huber의 이상치 억제 효과를 유지하면서 1852m
  target-window로 적당히 이동하는 지점을 자동 탐색하는 것이다. target-weighted
  Huber가 validation 전체나 1852m max error에서 과도하게 흔들릴 때, shrink
  후보가 더 안정적인 runtime profile로 선택될 수 있는지 평가한다.
- `disparity_linear_then_sim3`, `disparity_quadratic_then_sim3`: train set에서
  measured disparity를 1차/2차로 보정해 raw RDF ray를 먼저 재스케일하고,
  그 corrected ray를 PX4/RTK truth RDF에 Sim(3)로 정합한다. runtime에서는
  measured disparity와 raw RDF로 ray scale을 적용한 다음 고정 Sim(3)을
  사용하므로 truth를 요구하지 않는다.
- `disparity_linear_then_sim3_quadratic_residual_raw_forward_lut`,
  `disparity_quadratic_then_sim3_quadratic_residual_raw_forward_lut`,
  `disparity_linear_then_sim3_inverse_depth_residual_raw_forward_lut`,
  `disparity_quadratic_then_sim3_inverse_depth_residual_raw_forward_lut`: 위
  disparity-corrected ray + Sim(3) 후보 뒤에 raw RDF forward 기반 2차 residual
  LUT 또는 inverse-depth residual LUT를 추가한다. inverse-depth 변형은 stereo
  depth bias가 range보다 inverse depth에서 더 선형이라는 기하 가정에 맞춰
  `1 / truth_forward - 1 / base_forward` residual을 fitting한다. 목적은 disparity
  보정으로 validation RMSE를 낮추면서, 장거리 residual curve로 1852m
  target-window 안정성도 함께 개선되는지 확인하는 것이다.
- `target_weighted_disparity_linear_then_sim3`,
  `target_weighted_disparity_quadratic_then_sim3`,
  `target_weighted_disparity_linear_then_sim3_quadratic_residual_raw_forward_lut`,
  `target_weighted_disparity_quadratic_then_sim3_quadratic_residual_raw_forward_lut`,
  `target_weighted_disparity_linear_then_sim3_inverse_depth_residual_raw_forward_lut`,
  `target_weighted_disparity_quadratic_then_sim3_inverse_depth_residual_raw_forward_lut`,
  `target_weighted_disparity_linear_then_sim3_target_weighted_quadratic_residual_raw_forward_lut`,
  `target_weighted_disparity_quadratic_then_sim3_target_weighted_quadratic_residual_raw_forward_lut`,
  그리고 `_sigma040/160`, `_shrink025/050/075` 조합:
  disparity-corrected ray + Sim(3) 계열의 fitting에서 1852m 근방 train sample에
  더 큰 가중치를 준다. `_target_weighted_quadratic_residual_raw_forward_lut`
  변형은 Sim(3) 뒤 forward residual curve에도 별도 target-window 가중치를
  적용해 1852m worst-case error를 낮출 수 있는지 검증한다. residual weight
  sigma는 기본 80m이며 `_sigma040/160` 변형은 더 좁거나 넓은 target-window
  강조를 비교한다. `_shrink025/050/075` 변형은 해당 residual을 25%/50%/75%만
  적용해 validation RMSE 손실과 1852m worst-case 개선 사이의 trade-off를
  탐색한다. 이 가중치는 calibration-time fitting에만 쓰이며, runtime profile은
  measured disparity, raw RDF, fixed Sim(3)/LUT 파라미터만 사용한다.
- `sim3_residual_disparity`: Sim(3) 보정 후 train set에서 fitting한
  disparity 기반 forward residual을 추가 적용
- `sim3_residual_disparity_deg0`, `sim3_residual_disparity_deg1`: Sim(3)
  보정 후 평균 bias 또는 1차 disparity residual만 적용하는 보수적 저차 후보
- `sim3_target_weighted_residual_disparity_deg1`,
  `sim3_target_weighted_residual_disparity`: Sim(3) 보정 후 1852m 목표거리
  근방 sample을 더 크게 반영한 forward residual을 추가 적용
- `sim3_ridge_residual_disparity`: Sim(3) 보정 후 ridge-regularized polynomial
  forward residual을 추가 적용
- `sim3_binned_residual_disparity`: Sim(3) 보정 후 train set의 disparity
  구간별 forward residual median LUT를 만들고, runtime에서는 measured
  disparity에 대해 선형 보간된 residual을 추가 적용
- `sim3_binned_residual_raw_forward`,
  `sim3_binned_residual_raw_forward_bins11`: Sim(3) 보정 후 train set의 raw
  RDF forward 거리 구간별 forward residual median LUT를 만들고,
  runtime에서는 raw RDF forward 값으로 선형 보간된 residual을 추가 적용한다.
- `sim3_kernel_smoothed_residual_raw_forward_sigma060/120/250`: Sim(3) 보정 후
  raw RDF forward 거리와 forward residual의 관계를 Gaussian-kernel
  Nadaraya-Watson regression으로 smoothing한 뒤 quantile support LUT로
  저장한다. 목적은 binned median LUT의 bin 경계 민감도를 줄이고, 거리별
  residual curve가 매끄럽게 전이되는지 확인하는 것이다. `sigma`는 meter
  단위 bandwidth이며 calibration-time train set에만 쓰인다. runtime 출력
  계약은 기존 `forward_residual_raw_forward_lut`와 같고, raw RDF forward와
  fixed LUT만 사용한다. PASS 기준은 다른 후보와 동일하게 validation RMSE,
  1852m 오차, replay/promotion check이다.
- `sim3_monotone_binned_residual_raw_forward`,
  `sim3_monotone_binned_residual_raw_forward_bins11`: Sim(3) 보정 후 raw RDF
  forward residual LUT를 weighted PAVA isotonic regression으로 단조화한다.
  stereo depth calibration에서 거리별 residual curve가 완만해야 한다는
  안정성 가정을 둔 후보이며, validation/replay에서만 승격된다.
- `sim3_range_normalized_binned_residual_raw_forward`,
  `sim3_range_normalized_binned_residual_raw_forward_bins11`: Sim(3) 보정 후
  `residual / raw_forward` 비율을 구간별로 fitting하고 meter residual LUT로
  환산해 저장한다. range-proportional residual 가정이 실제 StereoDistance
  오차에 더 맞는지 validation/replay 기준으로 비교한다.
- `sim3_inverse_depth_binned_residual_raw_forward`,
  `sim3_inverse_depth_binned_residual_raw_forward_bins11`: Sim(3) 보정 후
  `1 / truth_forward - 1 / sim3_forward` inverse-depth delta를 구간별로
  fitting하고 meter residual LUT로 환산해 저장한다. runtime에서는 truth 없이
  raw RDF forward 값으로 LUT를 조회한다.
- `sim3_quadratic_residual_raw_forward_lut`: Sim(3) 보정 후 raw RDF forward에
  대한 2차 residual polynomial을 fitting하고 meter residual LUT로 저장한다.
  disparity bias가 거리 오차를 `Z^2` 형태로 키우는 stereo depth 근사에
  기반한 후보이며, runtime에서는 여전히 raw RDF와 fixed LUT만 사용한다.
- `sim3_quadratic_local_residual_raw_forward_lut`,
  `sim3_quadratic_local_residual_raw_forward_lut_bins3`,
  `sim3_quadratic_local_residual_raw_forward_lut_bins5`,
  `sim3_quadratic_local_residual_raw_forward_lut_shrink025`,
  `sim3_quadratic_local_residual_raw_forward_lut_shrink075`: Sim(3) 보정 후 2차
  raw-forward residual과 shrink된 local residual-of-residual LUT를 결합한다.
  목적은 물리형 quadratic 후보가 놓치는 거리대별 국소 bias를 제한적으로
  보정하는 것이다. 기본 이름은 local residual shrink `0.5`이고, suffix
  후보는 더 보수적인 `0.25`, 더 적극적인 `0.75`, 그리고 3/5/7-bin local LUT
  복잡도를 비교한다.
- `sim3_blended_binned_residual_disparity_raw_forward`,
  `sim3_blended_binned_residual_disparity_raw_forward_bins11` 및
  `_alpha025/075` 변형: Sim(3) 보정 후 같은 forward residual을 measured
  disparity LUT와 raw RDF forward LUT에 나눠 적용한다. 기본 후보는 50:50이고
  alpha 변형은 disparity-key 비중을 25%/75%로 바꿔 자동 탐색한다. 단일 key
  LUT가 특정 intrinsic/disparity stress에 과하게 결합되는지 확인하는 dual-key
  보수 후보이다.
- `sim3_agreement_gated_binned_residual_disparity_raw_forward`,
  `sim3_agreement_gated_binned_residual_disparity_raw_forward_bins11`: dual-key
  residual LUT를 쓰되, runtime에서 두 LUT가 예측한 residual 차이가 임계값을
  넘으면 residual 보정을 억제하고 Sim(3) 정합만 남긴다. truth 없이 raw RDF와
  measured disparity의 보정 일치성만으로 single-profile 전이 실패를 줄이는
  후보이다.
- `sim3_binned_residual_disparity_bins5/9/11`: 같은 Sim(3)+forward
  residual LUT 구조에서 bin 수를 grid-search하여 validation/1852m/안정성
  기준으로 선택
- `sim3_kernel_smoothed_binned_residual_disparity_sigma020/040/080` 및
  `_support5/7` 변형:
  measured disparity를 key로 Gaussian-kernel Nadaraya-Watson residual
  regression을 수행하고 runtime에는 같은 piecewise-linear disparity LUT로
  export한다. 목적은 hard bin 경계에서 residual이 튀는 현상을 줄이면서
  `sim3_binned_residual_disparity_bins*` 계열과 같은 StereoDistance runtime
  입력 계약을 유지하는 것이다. `_support5/7`은 support point 수를 줄여
  validation 성능과 모델 복잡도 사이의 trade-off를 탐색한다. PX4/RTK truth는
  fitting/evaluation에만 쓰고, runtime은 raw stereo-center RDF와 measured
  disparity만 사용한다.
- `sim3_local_linear_residual_raw_forward_sigma060/120/250`,
  `sim3_local_linear_residual_disparity_sigma020/040/080` 및 `_support5/7`
  변형: Sim(3) 보정 후 Forward residual을 Gaussian-kernel local-linear
  regression으로 fitting하고 runtime에는 기존 LUT profile로 export한다.
  목적은 Nadaraya-Watson local-constant smoothing이 거리/시차 범위 끝에서
  보일 수 있는 boundary bias를 줄이는 것이다. raw-forward 변형은 raw RDF
  forward meter를 key로 쓰고, disparity 변형은 measured disparity pixel을
  key로 쓴다. pass/fail은 validation range RMSE 개선, 1852m target window
  error, train/validation 안정성, model complexity로 판단한다. PX4/RTK truth는
  fitting/evaluation에만 쓰며 runtime 입력은 raw stereo-center RDF와 measured
  disparity 및 고정 LUT뿐이다.
- `sim3_spline_residual_raw_forward_smooth025/100/400`,
  `sim3_spline_residual_disparity_smooth025/100/400`: Sim(3) 보정 후 Forward
  residual을 regularized cubic smoothing-spline calibration curve로 fitting하고,
  runtime에는 support point 기반 piecewise-linear LUT로 export한다. 목적은
  binned LUT의 hard bin 경계와 sparse-bin overfit을 줄이면서, stereo depth
  residual이 거리/시차에 대해 smooth하게 변한다는 calibration curve 가정을
  검증하는 것이다. `smooth025/100/400`은 smoothing 강도를 grid-search한다.
  pass/fail은 다른 후보와 동일하게 validation range RMSE 개선, 1852m target
  window error, train/validation 안정성, model complexity로 판단한다. PX4/RTK
  truth는 fitting/evaluation에만 쓰며 runtime 입력은 raw stereo-center RDF와
  measured disparity 및 고정 LUT뿐이다.
- `affine_monotone_inverse_depth_binned_residual_raw_forward`,
  `affine_monotone_inverse_depth_binned_residual_raw_forward_bins11`,
  `sim3_monotone_inverse_depth_binned_residual_raw_forward`,
  `sim3_monotone_inverse_depth_binned_residual_raw_forward_bins11`: affine 또는
  Sim(3) estimated-to-truth 정합 뒤 남은 Forward residual을 stereo geometry에
  맞게 `1/truth_forward - 1/base_forward` inverse-depth delta로 fitting한 뒤,
  weighted PAVA isotonic regression으로 단조 제약을 걸고 meter residual LUT로
  다시 export한다. 목적은 일반 inverse-depth residual LUT의 sparse-bin
  과보정과 intrinsic-scale 전이 실패를 줄이는 것이다. pass/fail은 validation
  range RMSE 개선, 1852m target window error, RDF axis-wise RMSE regression,
  cross-seed/transfer 안정성, model complexity로 판단한다. PX4/RTK truth는
  fitting/evaluation에만 쓰며 runtime 입력은 raw stereo-center RDF와 고정
  LUT뿐이다.
- `sim3_smoothed_binned_residual_disparity`,
  `sim3_smoothed_binned_residual_disparity_bins11`: Sim(3)+forward residual
  LUT의 bin median을 global median 쪽으로 shrink하고 인접 bin smoothing을
  적용해 희소 calibration bin과 outlier에 대한 안정성을 비교
- `sim3_capped_binned_residual_disparity`,
  `sim3_capped_binned_residual_disparity_bins11`: Sim(3)+forward residual
  LUT의 bin median을 robust residual cap으로 제한해 single-profile 전이와
  1852m target-window 안정성이 개선되는지 비교
- `sim3_binned_rdf_residual_disparity`: Sim(3) 보정 후 train set의 disparity
  구간별 residual median LUT를 RDF 각 축마다 만들고, runtime에서는 measured
  disparity에 대해 축별 선형 보간 residual을 추가 적용
- `sim3_rdf_residual_disparity`: Sim(3) 보정 후 train set에서 fitting한
  disparity 기반 residual을 RDF 각 축에 추가 적용
- `sim3_target_weighted_rdf_residual_disparity`: Sim(3) 보정 후 1852m 목표거리
  근방 sample을 더 크게 반영한 residual을 RDF 각 축에 추가 적용
- `sim3_ridge_rdf_residual_disparity`: Sim(3) 보정 후 ridge-regularized
  polynomial residual을 RDF 각 축에 추가 적용
- `sim3_ridge_raw_rdf_residual_field_deg1_alpha10`,
  `sim3_ridge_raw_rdf_residual_field_deg1_alpha100_cap4m`,
  `sim3_ridge_raw_rdf_residual_field_deg2_alpha100_cap4m`,
  `sim3_ridge_raw_rdf_residual_field_deg2_alpha1000_cap4m`: Sim(3) 보정 후
  raw stereo-center RDF의 `right/forward`, `down/forward`, normalized
  forward feature로 ridge-regularized vector residual field를 fitting한다.
  목적은 bbox center와 target origin 격차처럼 영상 위치/거리 의존 residual이
  남는지 확인하는 것이다. runtime profile에는 fixed feature normalization과
  RDF residual coefficient matrix, 선택적 residual norm cap만 저장되며,
  runtime에서는 truth 없이 raw RDF만 사용한다.

출력은 `results/rtk_stereo_calibration_*` 아래에 저장된다.

- `method_comparison.csv`: 보정법별 train/validation RMSE, 1852m 근방 통계,
  모델 복잡도, 파라미터 JSON, `registration_model`, `residual_model`,
  `regularization_model`, `runtime_profile_family`, `selection_note`
- `validation_predictions.csv`: validation sample별 truth/prediction/error
- `target_1852m_stats.csv`: 1852m 근방 range error 통계
- `pareto_frontier.csv`, `pareto_frontier.json`: validation range RMSE,
  1852m 근방 절대 평균 오차, 모델 복잡도 기준 Pareto tradeoff
- `policy_recommendations.csv`, `policy_recommendations.json`: balanced,
  axis-balanced, target-specialist, lowest-complexity-pass 등 목적별 추천 후보
- `range_error_before_after.png`: raw와 최종 추천 보정법의 거리 오차 그래프
- `residual_disparity_by_distance.png`: 거리/시차별 residual disparity 그래프
- `rdf_axis_rmse.png`: RDF 축별 validation RMSE 그래프
- `pareto_frontier.png`: 보정 후보들의 validation/1852m/복잡도 tradeoff 그래프
- `summary.txt`: 각 보정법 성능 비교표와 최종 추천 보정법
- `method_parameters.json`, `run_config.json`: 재현용 설정과 보정 파라미터
- `sim3_runtime_profile.json`: 최종 추천이 Sim(3) 계열일 때 StereoDistance
  runtime에 바로 넣을 수 있는 `estimated_to_truth_sim3` JSON profile
- `runtime_profiles/`: 최종 추천 여부와 무관하게 StereoDistance runtime에
  적용 가능한 후보 profile 전체와 `manifest.json`
- `policy_profiles/`: 목적별 추천 후보의 StereoDistance runtime profile과
  `manifest.json`

강건/가중 Sim(3) 후보의 목적은 bbox center, origin 차이, 자세각 오차,
baseline/focal/disparity 오차가 섞여 만든 raw stereo RDF와 PX4/RTK truth
RDF 사이의 전역 estimated-to-truth 오차를 보정하는 것이다. 입력 가정은
train set에서만 PX4/RTK truth RDF와 raw RDF correspondence를 사용한다는
것이며, 출력 계약은 런타임에서 truth 없이 적용 가능한
`estimated_to_truth_sim3` profile(`scale`, `rotation_matrix_rdf`,
`translation_rdf_m`)이며, residual 후보는 named RDF axis별 disparity
polynomial 계수를 추가로 저장한다. disparity-then-Sim(3) 후보는 measured
disparity polynomial로 raw RDF ray를 먼저 scale한 뒤 같은 Sim(3) profile
계약으로 저장된다. disparity-only 후보는
`estimated_to_truth_disparity_poly` profile로 저장되며, runtime에서 measured
disparity를 보정한 뒤 raw RDF ray를 corrected forward/raw forward 비율로
스케일한다. RANSAC/trimmed 후보는 outlier consensus,
Huber/Tukey/robust-loss 후보는 robust M-estimation, range-weighted 후보는
weighted least-squares/Procrustes 목적식, target-weighted residual 후보는
목표거리 근방 가중 polynomial regression, ridge residual 후보는 정규화된
polynomial ridge regression, binned residual 후보는 quantile-binned median
calibration curve와 piecewise-linear interpolation을 기반으로 한다.
diagonal affine 후보는 RDF 축간 shear를 금지한 축별 gain+bias estimated-to-truth
보정으로, runtime에서는 truth 없이 raw RDF와 고정된 diagonal affine profile만
사용한다. identity-shrunk diagonal affine 후보는 gain을 1에 가깝게 묶어
validation/replay에서 안정성이 좋아지는지 확인한다.
raw-RDF residual field 후보는 thin residual vector field/regularized
regression 방식으로 Sim(3) 뒤에 남는 시야 위치 의존 bias를 설명할 수 있는지
비교하되, residual norm cap과 validation/replay 기준으로 과적합을 걸러낸다.
smoothed binned 후보는 empirical-Bayes shrinkage와 인접 bin smoothing,
capped binned 후보는 MAD/quantile 기반 winsorization으로 residual LUT
보정량을 제한한다. raw-forward binned 후보는 measured disparity 대신 raw
RDF forward 축을 LUT key로 사용해 disparity scale stress와의 결합을
비교한다. kernel-smoothed raw-forward 후보는 Gaussian kernel regression으로
거리별 residual curve를 smoothing해 bin 경계와 희소 sample에 대한 민감도를
줄이는지 비교한다. range-normalized raw-forward 후보는 residual 자체가 아니라
`residual / raw_forward` 비율을 먼저 fitting해 거리 비례 오차 성분이 더
안정적으로 전이되는지 확인한다. quadratic raw-forward 후보는 시차 bias의
1차 근사에서 거리 오차가 `Z^2`에 비례하는 성질을 반영해 장거리 bias를
더 적은 자유도로 설명할 수 있는지 확인한다. blended binned 후보는
disparity-key residual과 raw-forward-key residual을 grid-searched 비율로
함께 적용해 single-key 전이 실패가 줄어드는지 본다.
agreement-gated 후보는 두 key의 residual 예측이 크게 다를 때 residual 항을
끄거나 낮추는 runtime guard를 추가한다.
이 계열들은 모두 runtime에서는 truth 없이 measured disparity와 raw RDF만
사용한다.
Tukey biweight 후보는 bbox center 튐이나 correspondence outlier처럼 큰
잔차 sample이 일부 섞였을 때 해당 sample의 영향이 Huber보다 더 빨리
사라지는지 확인하기 위한 redescending M-estimator 후보이다. forward-only
LUT는 거리축 오차만 줄이는 보수적 후보이고, RDF-axis LUT는 bbox
center-origin 격차가 right/down 축에도 남는 경우를 탐색하는 확장 후보이다.
Pass 기준은 raw보다 validation range/position RMSE가 줄고, RDF 축별 RMSE가
후퇴하지 않으며, 1852m 근방 절대 평균 거리 오차가 허용치 안에 있고,
profile replay와 promotion check가 truth 입력 없이 통과하는 것이다.

최종 추천은 train RMSE가 아니라 validation range RMSE, 1852m 근방 절대 평균
오차, train/validation gap, 모델 복잡도를 함께 고려한다. 1852m 근방 절대
평균 거리 오차가 ±10m 안에 들어오고 raw보다 validation RMSE가 줄어드는
방법을 우선한다.

`estimated_to_truth_sim3` runtime profile은 내부 보정 벡터를 RDF
`[right, down, forward]`로 저장한다. 기존 StereoDistance position topic은
호환성을 위해 계속 FRD+range `[forward, right, down, range]` 순서로
발행된다. profile 적용 시 measurement runtime 입력은 raw stereo RDF와
measured disparity뿐이며, PX4/RTK truth나 true disparity는 사용하지 않는다.

보정 profile을 StereoDistance runtime에 적용하려면 pipeline 결과의
`sim3_runtime_profile.json`을 기존 `--bbox-adjustment` 경로에 넘긴다.

```bash
./run_operator_view.sh \
  --bbox-adjustment reports/experiments/EXP-20260613-010-sim3-disparity-global-calibration/output/sim3_runtime_profile.json
```

또는 launch 인자로 직접 지정할 수 있다.

```bash
ros2 launch stereo_ground_sim stereo_ground_sim.launch.py \
  enable_bbox_center_adjustment:=true \
  bbox_center_adjustment_path:=reports/experiments/EXP-20260613-010-sim3-disparity-global-calibration/output/sim3_runtime_profile.json
```

ROS/Gazebo를 띄우기 전, 기록 CSV에 대해 runtime profile과 같은 로직으로
offline replay를 할 수 있다. 이 검증은 CSV의 named RDF/NED/FRD 필드만 읽고,
runtime 입력으로 raw stereo RDF와 measured disparity만 사용한다.

```bash
Simulation/.venv/bin/python Simulation/scripts/stereodistance_profile_replay.py \
  --input-csv Simulation/reports/experiments/EXP-20260613-010-sim3-disparity-global-calibration/output/calibration_samples.csv \
  --profile-json Simulation/reports/experiments/EXP-20260613-010-sim3-disparity-global-calibration/output/sim3_runtime_profile.json \
  --output-dir Simulation/reports/experiments/EXP-20260613-010-sim3-disparity-global-calibration/profile_replay \
  --split validation \
  --target-range-m 1852 \
  --target-window-m 30
```

Replay 출력:

- `profile_replay_summary.csv`: raw/corrected range, position, RDF axis RMSE
- `profile_replay_predictions.csv`: row별 truth/raw/corrected RDF와 error
- `profile_replay_range_error.png`: truth range별 raw/corrected 거리 오차
- `profile_replay_summary.txt`: 사람이 읽는 요약

## 실행

```bash
./run_operator_view.sh
```

이 단일 명령이 build, Gazebo Sim 서버, ROS-Gazebo bridge, 거리 계산 노드, PX4 SITL, 단일 `rqt_gui` 운용 창을 모두 실행합니다. 커스텀 앱을 만들지 않고 표준 ROS 도구만 실행합니다.
기본 실행은 자동 이동을 켜지 않고, PX4가 실험 타겟 `x500_15m`에 attach되므로 QGroundControl이 MAVLink UDP 14550으로 같은 드론을 조종할 수 있습니다. 실행만으로 이륙-접근-착륙까지 시작하려면 `--auto-mission`을 붙입니다.
`x500_15m`는 카메라 전방 +Y 약 2000 m 지점에서 yaw -90도로 스폰되어, QGC/PX4에서 전진 명령을 주면 카메라 방향으로 접근합니다. 스테레오 베이스라인은 X축 방향입니다.
Gazebo 3D GUI에서는 카메라 뒤에서 전방 +Y를 바라보는 운용자 기준으로 왼쪽 카메라는 world X 음수 위치의 파란색 본체, 오른쪽 카메라는 world X 양수 위치의 빨간색 본체로 표시됩니다. 카메라 모델 원점은 실제 sensor optical center이며, 렌즈 visual의 앞면이 이 원점에 맞춰집니다.
`rqt_gui` 안에는 상단 좌/우 표준 Image View 2개, 가운데 기록 패널, 하단에 상하 3단으로 쌓인 Plot 3개가 로드됩니다. 기본 화면은 카메라와 그래프가 최대한 넓게 보이도록 파라미터 패널을 열지 않습니다.
Image View는 기본적으로 YOLO bbox overlay 토픽인 `/left_camera/detection_image`, `/right_camera/detection_image`를 봅니다. 추론 부하를 낮추고 원본 화면을 보려면 `--raw-view`를 붙여 `/left_camera/image`, `/right_camera/image`를 표시합니다. Plot 3개는 각각 카메라 중앙 기준 FRD축 실제값(`/ground_truth_position`), raw YOLO 추정값(`/stereo/estimated_position`), bbox 중심 보정 추정값(`/stereo/corrected_position`)을 비교합니다. 축 계약은 StereoDistance와 같은 비교 계약으로 `data[0]=Forward`, `data[1]=Right`, `data[2]=Down`입니다.
Gazebo 카메라 센서는 기본 30Hz(`CAMERA_UPDATE_RATE_HZ=30.0`)로 설정하고, 거리/오차 그래프 토픽도 기본 30Hz(`plot_publish_hz:=30.0`)로 발행합니다. 실제 거리 ground truth와 드론 위치 FRD는 Gazebo pose 스트림을 기본 15Hz(`actual_distance_read_hz:=15.0`)로 샘플링한 뒤 카메라 중앙 기준 `Forward/Right/Down`으로 변환합니다. `gz topic` pose 조회와 overlay 발행은 YOLO worker를 지연시킬 수 있습니다. 기본 실행은 Gazebo 3D GUI도 함께 엽니다(`gz_gui:=true`). 카메라/그래프 Hz를 우선해 headless로 돌리려면 `gz_gui:=false`를 붙입니다.
각 Plot의 시간축은 최근 30초 창으로 autoscroll되며, 값 축은 현재 보이는 데이터 범위에 맞춰 자동으로 다시 잡히도록 실행 시 rqt_plot 패치를 적용합니다. 시간 창 길이는 `--plot-window 30`처럼 바꿀 수 있습니다.
기본 YOLO 설정은 `config/yolo.env`에서 읽습니다. 기본 모델은 Ultralytics 기본 모델 이름 `yolov8n.pt`, class는 `any`, confidence는 `0.10`입니다. 모델 파일이 로컬에 없으면 Ultralytics가 런타임에 기본 위치로 내려받을 수 있습니다.
기본 실행은 YOLO 거리 계산을 `yolo_target_hz:=40.0`, `yolo_device:=cuda:0`, `yolo_imgsz:=1280`으로 요청합니다. 2 km 부근의 작은 드론 bbox를 양쪽 카메라에서 유지하고 YOLO stride 32 경고를 피하기 위해 카메라 가로 해상도와 같은 1280 입력 크기를 기본으로 사용합니다. 거리 노드는 `config/yolo.env`의 `YOLO_PYTHON` 또는 로컬 `.venv`, 없으면 `python3`로 실행합니다. CUDA가 실제로 잡히면 CUDA/FP16으로 좌/우 이미지를 batch 추론하고, CUDA가 잡히지 않으면 CPU로 fallback하더라도 YOLO 이미지 구독과 거리 계산을 계속 시도합니다. CPU fallback은 느릴 수 있으므로 필요하면 `--yolo-hz`나 `yolo_imgsz`를 낮춥니다.
YOLO bbox overlay 화면은 기본으로 켜집니다(`overlay_publish_hz:=20.0`). Overlay 이미지는 전용 `detection_overlay_node`가 raw HD 카메라와 `/stereo/detections` bbox 결과를 결합해 발행하므로, YOLO 거리 계산 루프와 이미지 그리기/rqt 표시 부하를 분리합니다. Overlay 노드는 최신 카메라 프레임을 캐시하고 좌/우 이미지를 교차 발행해 부하 피크를 줄입니다. 이 값은 목표 발행률이며, Gazebo/rqt/PX4 부하에 따라 실측 Hz는 낮아질 수 있습니다. YOLO 거리 계산에 자원을 우선 배정하려면 `--raw-view`를 붙여 overlay 발행을 끕니다.
YOLO 실제 추론 Hz는 `/stereo/yolo_update_hz`를 기준으로 확인합니다. `/target_distance`는 그래프 표시를 위해 마지막 거리값도 `plot_publish_hz`로 재발행하므로, 이 토픽 Hz가 곧 YOLO 추론 Hz를 의미하지는 않습니다.
실시간 기록은 앱 가운데의 Recorder 패널에서 `Start Lite`, `Start Full`, `Start Debug`, `Stop` 버튼으로 제어합니다. `Lite`는 Excel/MATLAB용 `telemetry.csv`, `Full`은 CSV와 좌/우 overlay MP4, `Debug`는 CSV/MP4와 재현용 rosbag을 함께 저장합니다. 새 `telemetry.csv`에는 `axis_contract=stereo_center_frd:forward_right_down`이 들어가며, 카메라 기준 실제 위치는 `drone_forward_m`, `drone_right_m`, `drone_down_m`, 카메라 기준 실제 3D 직선거리는 `ground_truth_m`으로 저장됩니다. 스테레오 3D 추정값은 `estimated_forward_m`, `estimated_right_m`, `estimated_down_m`, `estimated_range_m`과 raw YOLO alias `raw_forward_m/right_m/down_m`, bbox 보정 alias `corrected_forward_m/right_m/down_m`으로 저장합니다. 새 CSV는 공개 축 필드로 legacy `x/y/z`를 내보내지 않습니다. 명령 실행과 동시에 기록을 시작해야 하는 경우에는 `--record-lite`, `--record-full`, `--record-debug` 옵션을 사용할 수 있습니다.
저장된 debug rosbag을 같은 `rqt_gui` 창 안의 Bag 플러그인으로 확인하려면 Plot 대신 Bag 구성을 선택합니다.

```bash
./run_operator_view.sh --bag-view
```

이 환경의 ROS 2 Humble `rqt_bag`과 `rqt_plot`은 같은 perspective로 자동 로드하면 시작 중 충돌할 수 있어, 기본 관찰 구성은 카메라+그래프이고 `--bag-view`는 카메라+Bag 확인 구성입니다.

관찰과 저장을 한 번에 시작하려면:

```bash
./run_operator_view.sh --record-lite
```

YOLO `.pt`를 바로 지정하려면:

```bash
./run_operator_view.sh --yolo-model yolov8n.pt --yolo-class any --yolo-hz 20
```

파일 선택 창으로 `.pt`를 고르려면:

```bash
./run_operator_view.sh --select-yolo-model --yolo-class any --yolo-hz 20
```

YOLO bbox 보정 profile을 적용하려면:

```bash
./run_operator_view.sh --bbox-adjustment ../ref/Stereo_Camera_3D_Position_DKL/Cent_Bbox_Adj_Mtrx.csv
```

`--bbox-adjustment`는 기존 6x4 CSV bbox-center affine profile과 JSON profile을 모두 받을 수 있습니다.
JSON `origin_bbox_output_ridge` profile은 원본 bbox 좌표와 raw stereo FRD 추정값을 feature로 사용해 `/stereo/corrected_position`을 직접 보정합니다.
JSON `bounded_gain_forward_cell_guard` profile은 EXP009 계열 runtime policy로, 보정용 데이터에서 고정한 guard/gain/cell 정책을 사용해 ground truth 없이 `/stereo/corrected_position`을 발행합니다. 이 profile은 bbox anchor profile, right-camera rotation, `profile_family`, Down cell 정책을 함께 포함해야 합니다.
JSON `estimated_to_truth_sim3` profile은 보정 비행에서 학습한 Sim(3)
RDF 정합 파라미터를 raw stereo RDF에 적용해 ground truth 없이
`/stereo/corrected_position`을 발행합니다. 내부 계산은 RDF
`right/down/forward`이고, topic 출력은 기존 FRD+range
`forward/right/down/range` 순서를 유지합니다.
JSON `estimated_to_truth_affine_rdf` profile은 같은 RDF 계약에서
`p_corr = A * p_raw + t`를 적용합니다. 이 profile은 optional
`forward_residual_disparity_lut`, `forward_residual_raw_forward_lut`, 또는
`rdf_residual_disparity_lut`를 포함할 수 있습니다. disparity residual LUT는
보정 비행 truth로 fitting한 고정 LUT를 runtime measured disparity에
적용하고, raw-forward residual LUT는 runtime raw RDF forward 값에 적용합니다.
JSON
`estimated_to_truth_disparity_poly` profile은 measured disparity polynomial로
raw RDF ray scale을 보정합니다. 세 profile 모두 보정 비행 truth로 파라미터를
학습하지만 runtime 입력은 raw RDF와 measured disparity뿐입니다.
예시:

```bash
./run_operator_view.sh --bbox-adjustment reports/origin_bbox_output_profile_20260609_1415/origin_bbox_output_profile.json
```

실행 중 같은 `rqt_gui` 창에서 Dynamic Reconfigure로 `yolo_model_path`, `yolo_class_name`, `yolo_confidence`, `yolo_device`를 바꾸려면 `--params`를 붙여 실행합니다. `yolo_model_path`를 바꾸면 노드가 `.pt`를 다시 로드합니다.

```bash
./run_operator_view.sh --params
```

거리 계산 주기를 높이고 싶으면 `--yolo-hz` 또는 launch 인자 `yolo_target_hz`를 조정합니다. 이 값은 화면 표시 Hz가 아니라 YOLO 추론 Hz입니다. 그래프 토픽 발행 Hz는 `--plot-hz` 또는 `plot_publish_hz`로 조정합니다. Ground truth pose 재조회 Hz는 `--actual-hz` 또는 `actual_distance_read_hz`로 조정합니다.

```bash
./run_operator_view.sh yolo_target_hz:=40.0
./run_operator_view.sh --plot-hz 30
./run_operator_view.sh --actual-hz 15
./run_operator_view.sh --plot-window 30
```

CUDA/CPU 장치를 명시하거나, 부하가 부족하면 추론 Hz 또는 입력 크기를 더 줄일 수 있습니다.

```bash
./run_operator_view.sh --yolo-device cuda:0
./run_operator_view.sh --yolo-device cpu
./run_operator_view.sh --yolo-python python3
./run_operator_view.sh yolo_target_hz:=0.0
./run_operator_view.sh yolo_imgsz:=256
```

거리 노드의 Python 자체를 바꾸려면 `--yolo-python` 또는 launch 인자 `yolo_python:=/path/to/python`을 사용합니다. 기본 실행은 로컬 `.venv`를 우선 찾고, 없으면 `python3`를 사용합니다. 별도 site-packages를 써야 하면 `YOLO_PYTHONPATH=/path/to/site-packages ./run_operator_view.sh ...`처럼 지정합니다.

자동 접근은 QGC/PX4 제어 경로로 수행합니다. 기존 Gazebo `cmd_vel` 미션 옵션은 QGC 조종과 충돌하므로 사용하지 않습니다.
드론 스폰 위치와 PX4 자동 미션 기본값은 `config/drone_mission.env`에서 수정할 수 있습니다.

```bash
DRONE_SPAWN_X_M=0.0
DRONE_SPAWN_Y_M=2000.0
DRONE_SPAWN_Z_M=5.56
DRONE_SPAWN_YAW_DEG=-90.0

DRONE_TAKEOFF_ALTITUDE_M=10.0
DRONE_FORWARD_SPEED_MPS=15.0
DRONE_MOVE_DISTANCE_M=1900.0
DRONE_TARGET_DISTANCE_M=200.0
DRONE_APPROACH_TIMEOUT_S=600.0
DRONE_LAND_DURATION_S=10.0
DRONE_LAND_TIMEOUT_S=90.0
```

`DRONE_SPAWN_*` 값은 `./run_operator_view.sh`와 자세각 실험이 Gazebo 월드를 만들 때 적용합니다. `DRONE_TAKEOFF_ALTITUDE_M`, `DRONE_FORWARD_SPEED_MPS`, `DRONE_MOVE_DISTANCE_M`, `DRONE_TARGET_DISTANCE_M` 값은 자동 미션, waypoint 미션, 자세각 실험에서 공통 기본값으로 사용합니다. 명령줄 옵션과 launch 인자를 같이 주면 명령줄 값이 우선합니다.

Gazebo, rqt, PX4 실행 후 자동 이륙-접근-착륙까지 한 번에 시작하려면:

```bash
./run_operator_view.sh --auto-mission
```

기본 자동 임무는 PX4 SITL이 붙을 시간을 기다린 뒤 `./run_auto_mission.sh`를 실행합니다. 빠른 확인용 설정은:

```bash
./run_operator_view.sh --auto-mission --quick-mission
```

이미 `./run_operator_view.sh`로 Gazebo, PX4 SITL, QGC 연결이 떠 있는 상태에서 자동 이륙-접근-착륙만 실행하려면 별도 터미널에서 아래 명령을 실행합니다.

```bash
./run_auto_mission.sh
```

기본 동작은 `config/drone_mission.env` 기준으로 이륙, 호버, PX4 local `north` 음수 방향 접근, `/ground_truth`가 목표 거리 이하가 되면 호버, 착륙입니다. 현재 월드에서는 이 축이 Gazebo Y 거리를 줄이는 카메라 방향 접근입니다. `/ground_truth`를 읽지 못하면 fallback으로 `DRONE_MOVE_DISTANCE_M` local displacement까지 이동합니다. PX4 기본 `MPC_XY_CRUISE`/`MPC_XY_VEL_MAX` 제한 때문에 명령 속도보다 실제 접근 속도가 낮을 수 있습니다. 한 번만 목표 거리나 속도를 바꾸려면:

```bash
./run_auto_mission.sh --target-distance-m 200 --altitude-m 10 --speed-mps 15
```

이 명령은 Gazebo나 rqt 앱을 다시 띄우지 않고, 실행 중인 PX4에 MAVLink offboard 명령만 병렬로 보냅니다. 기본 연결 포트는 PX4 SITL offboard 포트인 `udpin:0.0.0.0:14540`입니다.
임무를 시작하지 않고 연결만 확인하려면:

```bash
./run_auto_mission.sh --check-connection
```

`./run_operator_view.sh`의 기본값은 `gz_gui:=true`, `launch_rviz:=false`입니다. 기본 실행에서 Gazebo 3D GUI와 rqt 카메라/그래프 창이 함께 뜹니다. RViz Marker/3D도 같이 보고 싶으면 아래처럼 켭니다.

```bash
./run_operator_view.sh launch_rviz:=true
```

인자를 넘길 수도 있습니다.

```bash
./run_operator_view.sh right_yaw_error_deg:=0.5
./run_operator_view.sh approach_duration_s:=20.0
```

`./run_x500_mission.sh`는 이전 Gazebo `cmd_vel` 방식이라 현재 QGC/PX4 모드에서는 비활성화되어 있습니다.

카메라/그래프 Hz를 우선해서 Gazebo 3D GUI를 끄려면:

```bash
./run_operator_view.sh gz_gui:=false
```

ROS/Gazebo 로그 디렉터리 권한 문제가 있으면:

```bash
export ROS_LOG_DIR=/tmp/ros2-launch-log
export GZ_CONFIG_HOME=/tmp/gz-config
```

운용 화면 가운데의 Recorder 패널에서 보통은 `Start Lite`를 눌러 CSV만 저장하고, 화면 확인용 영상까지 필요할 때는 `Start Full`을 사용합니다. 원본 ROS 토픽 재생/재처리까지 필요할 때만 `Start Debug`를 사용합니다.
명령 실행과 동시에 기록을 시작해야 하면 아래처럼 실행할 수도 있습니다.

```bash
./run_operator_view.sh --record-lite
```

명령형 기록의 저장 위치를 지정할 수도 있습니다.

```bash
./run_operator_view.sh --record-lite bags/yaw_0p25deg
```

## 주요 토픽

```text
/left_camera/image
/right_camera/image
/left_camera/detection_image
/right_camera/detection_image
/left_camera/camera_info
/right_camera/camera_info
/stereo/disparity
/ground_truth
/ground_truth_position
/stereo/estimated_position
/stereo/corrected_position
/stereo/bbox_adjustment
/stereo/distance_error
/stereo/distance_comparison
/stereo/distance_marker
/target_distance
```

거리 값 확인:

```bash
ros2 topic echo /target_distance
```

실제 거리와 측정 오차 확인:

```bash
ros2 topic echo /ground_truth
ros2 topic echo /ground_truth_position
ros2 topic echo /stereo/estimated_position
ros2 topic echo /stereo/corrected_position
ros2 topic echo /stereo/distance_error
```

한 토픽에서 같이 보려면:

```bash
ros2 topic echo /stereo/distance_comparison
```

`/stereo/distance_comparison` 배열 순서는 다음과 같습니다.

```text
[actual_m, measured_m, error_m, disparity_px]
```

`/ground_truth_position` 배열 순서는 다음과 같습니다.

```text
[forward_m, right_m, down_m, ground_truth_m]
```

여기서 배열은 StereoDistance와 같은 FRD 비교 계약입니다. `data[0]`은 전방, `data[1]`은 오른쪽, `data[2]`는 아래쪽입니다. `ground_truth_m`은 카메라 위치 기준 실제 3D 직선거리이며, `estimated_range_m` 비교용입니다. `/ground_truth` 토픽과 `/stereo/distance_comparison`의 `actual_m`은 카메라 전방축 실제 거리 `forward_m`을 발행합니다.
새 CSV에서는 `drone_forward_m/right_m/down_m`과 `estimated_forward_m/right_m/down_m`을 표준으로 사용하고 legacy `x/y/z` 공개 필드는 쓰지 않습니다. 과거 CSV에는 `drone_x_m/y_m/z_m`과 `estimated_x_m/y_m/z_m`가 남아 있을 수 있으므로, reader는 `axis_contract`를 확인해 `stereo_center_frd:forward_right_down`이면 legacy `x/y/z=forward/right/down`, 누락/구버전이면 변경 전 RDF인지 구분해야 합니다.

`/stereo/estimated_position` 배열 순서는 다음과 같습니다.

```text
[estimated_forward_m, estimated_right_m, estimated_down_m, estimated_range_m]
```

`/stereo/corrected_position` 배열 순서도 같습니다.

```text
[corrected_forward_m, corrected_right_m, corrected_down_m, corrected_range_m]
```

추정값도 같은 FRD 축 계약을 사용합니다. `/target_distance`는 raw `estimated_forward_m`을 발행합니다. CSV bbox-center affine 보정에서는 `/stereo/bbox_adjustment`가 `[left_corrected_cx, left_corrected_cy, right_corrected_cx, right_corrected_cy, left_du_px, left_dv_px, right_du_px, right_dv_px]`를 발행합니다. JSON output profile은 bbox 중심을 옮기지 않고 FRD 결과를 직접 보정하므로 `/stereo/corrected_position`만 비교하면 됩니다.

## 파라미터 확인/변경

```bash
ros2 param get /stereo_distance_node baseline_m
ros2 param get /stereo_distance_node horizontal_fov_rad
ros2 param get /stereo_distance_node approach_start_distance_m
ros2 param get /stereo_distance_node approach_end_distance_m
ros2 param get /stereo_distance_node approach_duration_s
ros2 param get /stereo_distance_node mission_profile
ros2 param get /stereo_distance_node takeoff_altitude_m
ros2 param get /stereo_distance_node fly_target_in_gazebo
ros2 param get /stereo_distance_node actual_distance_source
ros2 param get /stereo_distance_node yolo_model_path
ros2 param get /stereo_distance_node yolo_class_name
ros2 param get /stereo_distance_node yolo_confidence

ros2 param set /stereo_distance_node baseline_m 25.0
ros2 param set /stereo_distance_node horizontal_fov_rad 0.154997710698
ros2 param set /stereo_distance_node approach_duration_s 120.0
ros2 param set /stereo_distance_node yolo_model_path /path/to/drone.pt
ros2 param set /stereo_distance_node yolo_class_name drone
```

카메라 위치, 높이, 타겟 크기, 해상도, FPS는
`stereo_ground_sim/worlds/stereo_ground_drone.sdf`에서 수정합니다.
기본 ground는 `4000 m x 4000 m`이고 월드 기준 x=0..4000, y=-2000..2000 범위를 덮습니다. 실행 시 `stereo_ground_sim.launch.py`가 같은 범위에 100 m 간격 grid line visual을 생성하고, 카메라 방향으로 굵은 기준선을 추가합니다. 카메라 모델 원점은 sensor optical center이고, Gazebo GUI 확인용 본체와 렌즈 visual은 그 기준점 뒤쪽에 배치됩니다.

카메라 자세각은 자주 바꾸는 값이므로 `config/camera_attitude.env`에서 수정할 수 있습니다.

```bash
RIGHT_YAW_ERROR_DEG=0.25
```

파일을 저장한 뒤 `./run_operator_view.sh`를 다시 실행하면 적용됩니다. 명령줄 인자를 함께 주면 명령줄 인자가 우선합니다.

PX4 waypoint 경로도 같은 방식으로 `config/waypoints.env`에서 밖으로 뺄 수 있습니다.

```bash
WAYPOINTS_XY_ALT_M="-300,0,10;-600,0,10;-900,0,10"
WAYPOINT_SPEED_MPS=12.0
WAYPOINT_ACCEPTANCE_RADIUS_M=5.0
WAYPOINT_LAND_AFTER=true
WAYPOINT_RECORD_LITE=true
```

`WAYPOINTS_XY_ALT_M`의 각 항목은 `north,east,altitude_agl_m` 순서이며, 항목은 세미콜론으로 구분합니다. PX4 내부 `LOCAL_NED` 좌표계에서 north/east는 x/y이고, 고도는 사람이 보기 쉽게 양수 AGL 값으로 적습니다. 기본 waypoint는 `config/drone_mission.env`의 `DRONE_MOVE_DISTANCE_M`와 `DRONE_TAKEOFF_ALTITUDE_M`를 참조합니다. 파일을 수정한 뒤 `./run_operator_view.sh`로 Gazebo/PX4를 먼저 띄우고 별도 터미널에서 실행합니다.
현재 시뮬레이션 축 기준에서는 카메라 방향 접근이 `north` 음수 방향이고, `east`는 카메라 베이스라인과 같은 횡방향입니다. 예를 들어 시작 거리 2000 m에서 카메라까지 415 m 지점으로 가려면 local north는 `-(2000 - 415) = -1585`입니다.

```bash
./run_px4_waypoint_mission.sh
```

이 명령은 기본적으로 lite CSV도 함께 저장합니다. 저장 위치는 `bags/waypoint_YYYYmmdd_HHMMSS_lite`이고, `telemetry.csv`에는 실제 Forward/Right/Down/GroundTruth, 스테레오 3D 추정값, 거리/오차/YOLO bbox/confidence 같은 수치 로그가 기록됩니다. 저장 위치를 바꾸거나 끄려면:

```bash
./run_px4_waypoint_mission.sh --record-dir bags/waypoint_test_01
./run_px4_waypoint_mission.sh --no-record
```

한 번만 다른 경로를 쓰려면 명령줄 인자가 설정 파일을 대체합니다.

```bash
./run_px4_waypoint_mission.sh \
  --waypoint -300,0,10 \
  --waypoint -600,0,10 \
  --speed-mps 8
```

## 카메라 자세각 오차 실험

launch 인자로 좌/우 카메라 roll, pitch, yaw 오차를 도 단위로 줄 수 있습니다.
거리 계산 노드는 이상적인 평행 스테레오 조건에서 bbox 중심을 3D 위치로 복원하므로, RViz2와 토픽에서 자세각 오차가 3D 추정값과 거리 측정값에 주는 영향을 관찰할 수 있습니다.
단일 `rqt_gui` 운용 창에는 좌/우 원본 카메라 영상과 하단에 상하 3단으로 쌓인 Forward/Right/Down축 실제값-raw 추정값-corrected 추정값 비교 그래프가 표시됩니다.
그래프는 표준 `rqt_plot` 플러그인 3개로 나뉩니다. Forward Axis Plot은 `/ground_truth_position/data[0]`, `/stereo/estimated_position/data[0]`, `/stereo/corrected_position/data[0]`, Right Axis Plot은 각 topic의 `data[1]`, Down Axis Plot은 각 topic의 `data[2]`를 표시합니다. 런타임 패치는 각 plot 제목과 Y축 라벨에 `Forward`, `Right`, `Down` 이름을 명시합니다.
거리 측정은 왼쪽/오른쪽 영상에서 같은 YOLO class의 bbox 중심을 찾고, disparity와 영상 중심 오프셋으로 내부 RDF를 계산한 뒤 FRD topic 순서인 `estimated_forward_m`, `estimated_right_m`, `estimated_down_m`, `estimated_range_m`으로 발행합니다. bbox 중심 보정이 켜져 있으면 같은 순서로 `corrected_forward_m`, `corrected_right_m`, `corrected_down_m`, `corrected_range_m`도 발행합니다. 기존 거리 그래프와 `/target_distance`는 이 3D 복원 결과 중 raw 카메라 전방 거리인 `estimated_forward_m`을 사용합니다. `/ground_truth_position`의 `ground_truth_m`은 카메라 기준 FRD로 계산한 3D 직선거리이므로 `estimated_range_m` 및 `corrected_range_m`과 직접 비교합니다.
자동 접근 실험은 QGC mission 또는 PX4용 MAVLink/offboard 명령으로 수행합니다. 이전 `./run_x500_mission.sh`의 Gazebo `cmd_vel` 방식은 현재 QGC/PX4 제어와 충돌하므로 비활성화되어 있습니다.

자세각 오차 정량 실험은 아래 명령을 사용합니다. 기본 실행은 Gazebo 이미지와 YOLO 거리 계산을 포함한 반복 실험입니다.
실행 전에 이전 `attitude_error` Gazebo/ROS 프로세스가 남아 있으면 자동으로 종료한 뒤 새 실험을 시작합니다. 이 자동 정리를 막고 기존처럼 즉시 실패하게 하려면 `--no-stop-existing-sim`을 사용하고, 의도적으로 기존 시뮬레이션과 함께 실행하려면 `--allow-existing-sim`을 사용합니다.

```bash
./run_attitude_error_experiment.sh
```

기본 Gazebo/YOLO 실험은 오른쪽 카메라 roll/pitch/yaw 축을 대상으로 `+0.25도` 오차를 테스트합니다. 음수 오차도 포함하려면:

```bash
./run_attitude_error_experiment.sh --include-negative
```

각도 오차 범위는 `config/attitude_error.env`에서 조절합니다. 예를 들어 `-1.0도`부터 `+1.0도`까지 `0.25도` 간격으로 테스트하려면:

```bash
ATTITUDE_EXPERIMENT_ANGLE_MIN_DEG=-1.0
ATTITUDE_EXPERIMENT_ANGLE_MAX_DEG=1.0
ATTITUDE_EXPERIMENT_ANGLE_STEP_DEG=0.25
```

`0도` 케이스는 별도 `baseline`과 중복되므로 자동으로 제외됩니다.

축마다 다른 범위와 단계를 쓰려면 `AXIS:MIN:MAX:STEP` 형식의 `--axis-range`를 반복해서 줍니다.

```bash
./run_attitude_error_experiment.sh \
  --axis-range right_roll_error_deg:-5:5:0.5 \
  --axis-range right_pitch_error_deg:-2:2:0.25 \
  --axis-range right_yaw_error_deg:-10:10:1
```

설정 파일에서는 공백으로 구분합니다.

```bash
ATTITUDE_EXPERIMENT_AXIS_RANGES="right_roll_error_deg:-5:5:0.5 right_pitch_error_deg:-2:2:0.25 right_yaw_error_deg:-10:10:1"
```

축별 범위가 지정된 축은 전역 `ATTITUDE_EXPERIMENT_ANGLE_MIN_DEG`, `ATTITUDE_EXPERIMENT_ANGLE_MAX_DEG`, `ATTITUDE_EXPERIMENT_ANGLE_STEP_DEG` 대신 해당 축별 값을 사용합니다.

Gazebo/YOLO 실험 결과는 `reports/attitude_error_*/summary.csv`와 케이스별 CSV/log로 저장됩니다. `summary.csv`에는 케이스별 좌/우 roll/pitch/yaw, 마지막 실제 Forward/Right/Down/GroundTruth, 마지막 3D 추정값, baseline, 카메라 높이, FOV, 해상도 같은 카메라 모델 정보도 함께 들어가며, 공통 실행 조건은 `metadata.txt`에도 저장됩니다.

카메라 모델 기본값은 `config/camera_model.env`에서 공통으로 바꿉니다. YOLO 기본값은 `config/yolo.env`에서 공통으로 바꿉니다. 두 파일 값은 operator view와 자세각 실험에 같이 적용됩니다.

```bash
CAMERA_BASELINE_M=25.0
CAMERA_HEIGHT_M=5.0
CAMERA_HORIZONTAL_FOV_RAD=0.154997710698
CAMERA_IMAGE_WIDTH_PX=1280
CAMERA_IMAGE_HEIGHT_PX=720
CAMERA_UPDATE_RATE_HZ=60.0
CAMERA_CLIP_NEAR_M=0.1
CAMERA_CLIP_FAR_M=2500.0
```

```bash
YOLO_MODEL_PATH="yolov8n.pt"
YOLO_CLASS_NAME=any
YOLO_CONFIDENCE=0.10
YOLO_DEVICE=cuda:0
YOLO_TARGET_HZ=40.0
YOLO_IMGSZ=1280
YOLO_PYTHON=python3
```

자세각 실험 범위 기본값은 `config/attitude_error.env`에서 바꿀 수 있습니다. 각도 단위는 도(deg)입니다.

```bash
ATTITUDE_EXPERIMENT_AXES="right_roll_error_deg right_pitch_error_deg right_yaw_error_deg"
ATTITUDE_EXPERIMENT_AXIS_RANGES=""
ATTITUDE_EXPERIMENT_ANGLE_MIN_DEG=0.25
ATTITUDE_EXPERIMENT_ANGLE_MAX_DEG=0.25
ATTITUDE_EXPERIMENT_ANGLE_STEP_DEG=0.25
ATTITUDE_EXPERIMENT_INCLUDE_NEGATIVE=false
ATTITUDE_EXPERIMENT_QUICK=false
```

다른 설정 파일을 임시로 쓰려면 `CAMERA_MODEL_CONFIG=config/my_camera.env ./run_operator_view.sh`, `YOLO_CONFIG=config/my_yolo.env ./run_operator_view.sh`, 또는 `ATTITUDE_EXPERIMENT_CONFIG=config/my_attitude.env ./run_attitude_error_experiment.sh`처럼 실행합니다.
드론 스폰/미션 설정도 같은 방식으로 `DRONE_MISSION_CONFIG=config/my_drone_mission.env ./run_operator_view.sh`, `DRONE_MISSION_CONFIG=config/my_drone_mission.env ./run_auto_mission.sh`, `DRONE_MISSION_CONFIG=config/my_drone_mission.env ./run_px4_waypoint_mission.sh`, `DRONE_MISSION_CONFIG=config/my_drone_mission.env ./run_attitude_error_experiment.sh`처럼 바꿀 수 있습니다.

예: 오른쪽 카메라 yaw를 1도 틀기

```bash
./run_operator_view.sh right_yaw_error_deg:=1.0
```

예: 양쪽 카메라 pitch 오차를 다르게 주기

```bash
./run_operator_view.sh \
  left_pitch_error_deg:=0.5 \
  right_pitch_error_deg:=-0.5
```

사용 가능한 인자:

```text
baseline_m
camera_height_m
approach_start_distance_m
approach_end_distance_m
approach_duration_s
target_spawn_x_m
target_spawn_y_m
target_spawn_z_m
target_spawn_yaw_deg
mission_profile
fly_target_in_gazebo
actual_distance_source
actual_distance_read_hz
gz_gui
launch_rviz
takeoff_altitude_m
takeoff_duration_s
first_hover_duration_s
final_hover_duration_s
land_duration_s
target_center_z_m
left_roll_error_deg
left_pitch_error_deg
left_yaw_error_deg
right_roll_error_deg
right_pitch_error_deg
right_yaw_error_deg
yolo_model_path
yolo_class_name
yolo_confidence
yolo_device
yolo_python
plot_publish_hz
yolo_imgsz
```

## 드론 모델 선택

이 시뮬레이션의 주 목적은 카메라 자세각 오차가 거리 측정에 미치는 영향을 보는 것입니다.
그래서 기본 타겟은 2000 m 장거리에서도 영상 검출이 가능한 15 m 폭의 대형 쿼드콥터 형상으로 둡니다.

X500 또는 IRIS 모델로 시각 형상을 바꾸는 것은 가능하지만, 실제 모델 텍스처가 초록색 검출 조건과 맞지 않으면 거리 노드가 타겟을 놓칠 수 있습니다. 특히 2000 m 장거리에서는 실제 크기가 작으면 픽셀 수가 부족합니다. 이 경우에는 폭을 실험 조건에 맞게 스케일링하거나, 색상 검출 대신 YOLO/형상 기반 검출로 바꾸는 것이 좋습니다.
현재 거리 노드는 YOLO `.pt` 기반 검출을 기본으로 하므로, `.pt` 안에 `drone` 클래스가 있거나 `--yolo-class` / `yolo_class_name`을 실제 클래스명으로 맞춰야 합니다. 이 Python 환경에 `ultralytics`와 `torch`가 설치되어 있지 않으면 노드는 상태에 import 실패를 표시하고 거리를 계산하지 않습니다.

## 드론 깜빡임 완화

2000 m 장거리 카메라 영상에서는 얇은 텍스처 면이나 큰 배율의 얇은 프로펠러 메시가 픽셀 경계에서 깜빡이는 것처럼 보일 수 있습니다.
`x500_15m` 모델은 이 영향을 줄이기 위해 본체 위 로고 텍스처 plane을 제거하고, 네 개의 얇은 프로펠러 STL 시각 요소를 어두운 원통형 로터 디스크로 바꿨습니다.
본체, 암, 모터 형상은 유지하므로 타겟은 여전히 X500 기반 드론 형상입니다.
Z축 비교에서는 Gazebo pose 기반 ground truth와 YOLO bbox 중심 기반 추정값의 기준점을 맞추기 위해 대형화된 시각 본체/로터의 내부 Z 오프셋을 모델 원점 근처로 정렬했습니다. 다리 collision은 같은 기준에서 지면에 닿도록 아래쪽에 배치하고, 기본 `DRONE_SPAWN_Z_M=5.56`은 이 다리 collision 높이를 반영합니다.

## QGroundControl

`./run_operator_view.sh` 기본 실행은 PX4 SITL을 standalone Gazebo attach 모드로 시작합니다.
PX4는 이미 world 안에 있는 15 m급 `x500_15m` 타겟에 붙고, QGC는 PX4의 MAVLink UDP 14550 heartbeat를 받아 자동 연결됩니다. PX4 콘솔 출력은 기본적으로 버립니다. 디버깅 로그가 필요하면 `PX4_CONSOLE_LOG=/tmp/px4_qgc.log ./run_operator_view.sh`처럼 실행합니다.

PX4를 끄고 카메라/거리 계산만 실행하려면:

```bash
./run_operator_view.sh --no-px4
```

PX4/QGC 제어가 필요하면 `./run_operator_view.sh`를 기본값으로 다시 실행합니다.
Ground truth는 기본적으로 Gazebo pose 토픽(`/world/stereo_ground_drone/pose/info`)에서 `x500_15m`의 실제 pose를 읽고, 이를 카메라 기준 `Forward/Right/Down`으로 변환해 `/ground_truth_position`으로 발행합니다. `/ground_truth` 전방거리는 변환된 `forward_m`입니다. pose 토픽을 읽지 못하면 계획 거리값으로 fallback합니다.
반복 가능한 접근 실험이 필요하면 QGC mission 또는 PX4용 MAVLink/offboard 명령을 사용합니다.

## 기록 프리셋

```text
Lite  = telemetry.csv
Full  = telemetry.csv + left_overlay.mp4 + right_overlay.mp4
Debug = telemetry.csv + left_overlay.mp4 + right_overlay.mp4 + rosbag/
```

운용 중에는 직접 `ros2 bag record`를 실행하지 않아도 됩니다. 앱 가운데의 Recorder 패널에서 `Start Lite`, `Start Full`, `Start Debug`, `Stop` 버튼을 사용합니다. 실행과 동시에 기록을 시작하려면 `./run_operator_view.sh --record-lite`, `./run_operator_view.sh --record-full`, `./run_operator_view.sh --record-debug`을 사용할 수 있습니다. Debug rosbag은 MCAP 플러그인이 있으면 MCAP을 쓰고, 없으면 sqlite3로 자동 대체합니다. 저장된 bag을 단일 `rqt_gui` 창 안에서 열려면 `./run_operator_view.sh --bag-view`를 사용합니다.

## 변경 이력

- 2026-06-11: EXP009 `bounded_gain_forward_cell_guard` JSON profile을 StereoDistance runtime loader에서 지원하도록 추가했습니다. 보수 Down guard를 적용한 EXP013은 5-trial Simulation Runtime 검증 `PASS`이며, 아직 hardware/WIRIS 검증은 아닙니다.
- 2026-06-09: YOLO inference 기본 입력 크기를 `1280`으로 통일했습니다.
