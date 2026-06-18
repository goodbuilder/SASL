# 업데이트 기록

## v1.6.2 — 2026-06-17 — 앱 디렉터리 루트 이동

- 변경:
  - `StereoDistance` 앱을 `apps/` 밖의 repository 루트 `StereoDistance/`로 이동했다.
  - `run.sh`가 더 이상 `apps/StereoDistance/main.py`나 repo-root 상대 깊이에 의존하지 않고 앱 루트의 `main.py`를 실행하도록 바꿨다.
  - Simulation helper scripts가 `../../Simulation` 고정 경로 대신 `SIM_DIR`, `SASL_ROOT`, 상위 디렉터리 탐색 순서로 Simulation workspace를 찾도록 바꿨다.
  - 실행 권한에 의존하던 helper script 내부 호출을 `bash script.sh` 호출로 바꿨다.
  - 문서와 ignore 규칙을 새 앱 루트 기준으로 정리했다.
- 이유:
  - `StereoDistance`를 `apps/` 밖으로 분리해도 실행, Simulation profile 생성/평가, 설정/캡처 경로가 깨지지 않게 하기 위해서다.
- 확인:
  - `bash -n StereoDistance/run.sh StereoDistance/scripts/simulation_origin_profile_path.sh StereoDistance/scripts/simulation_origin_profile_generate.sh StereoDistance/scripts/simulation_origin_evaluate.sh`
  - `PYTHONPYCACHEPREFIX=/tmp/stereodistance_pycache python3 -m compileall -q StereoDistance/wiris_stereo_distance StereoDistance/scripts/simulation_origin_tools.py StereoDistance/main.py`
  - `QT_QPA_PLATFORM=offscreen PYTHONPATH=StereoDistance python3 -m pytest StereoDistance/tests` → 286 passed
  - `bash StereoDistance/scripts/simulation_origin_evaluate.sh --dry-run --validation-run --repeats 1 --output-dir /tmp/stereodistance_eval_after_move`
- 남은 위험:
  - `/home/moontaehyung/Capstone/...` 기본값은 장비/개발환경 기본값으로 남아 있으며, 필요 시 `PYTHON_BIN`, `PX4_WORKSPACE_SETUP`, `PX4_DIR`, `OPERATOR_RECORDER_PYTHON`으로 override해야 한다.

## v1.6.1 — 2026-06-09 — Reprojection Profile component gain 안정화

- 변경:
  - Origin-BBOX Reprojection Profile에 `correction_mode`를 추가하고 `full`, `disparity_only`, `component_gains` 적용 정책을 지원하게 했다.
  - 기본 적용을 raw `du/dv` 전체 적용이 아니라 `component_gains(disparity_gain=0.125, screen_u_gain=0.0, screen_v_gain=0.025)`로 바꿨고, mode가 없는 기존 Reprojection JSON도 이 보수 기본값으로 읽게 했다.
  - Simulation profile CLI와 random validation script에 correction mode/gain 옵션을 연결했다.
  - attitude-aware profile/validation trajectory 생성은 camera attitude error가 반영된 좌/우 실제 reprojection 공통 FOV 안에서 waypoint를 고르게 했다.
  - `docs/PROJECT.md`와 `docs/usage.md`에 conservative component gain 정책과 long-profile 검증 조건을 추가했다.
- 이유:
  - raw median reprojection residual을 그대로 적용하면 Forward/Range는 개선되지만 평균 screen-position shift가 Right/Down을 과보정할 수 있다. 좌우 horizontal separation 성분을 보수적으로 적용하고 screen-center 성분을 제한해야 5개 random validation 궤적에서 축별 RMSE regression을 피할 수 있었다.
- 확인:
  - `PYTHONDONTWRITEBYTECODE=1 QT_QPA_PLATFORM=offscreen python3 -m pytest -q tests/test_reprojection_profile.py tests/test_simulation_origin.py -p no:cacheprovider` → 16 passed
  - `python3 -m py_compile scripts/reprojection_profile_random_validation.py`
  - `source install/setup.bash; QT_QPA_PLATFORM=offscreen python3 scripts/reprojection_profile_random_validation.py --seed 20260625 --right-roll-error-deg 0.1 --right-pitch-error-deg 0.1 --right-yaw-error-deg 0.1 --profile-screen-u-fracs 0.29,0.50,0.71 --profile-screen-v-fracs 0.29,0.50,0.71 --profile-ranges-m 310,450,650,950,1300 --profile-altitude-min-m 0.0 --profile-altitude-max-m 30.0 --profile-waypoint-repeat 5 --profile-trial-duration-s 180 --trial-duration-s 35 --profile-min-cell-samples 6 --profile-lookup-policy exact` → `Final pass: yes`
- 남은 위험:
  - 고정 gain만으로는 짧은 profile 수집에서 mm 단위 Right/Down regression이 발생할 수 있었다. 최종 채택 profile은 attitude-aware long-profile 조건으로 cell median을 안정화한 뒤 별도 random validation 5개를 통과한 JSON만 사용해야 한다.

## v1.6.0 — 2026-06-09 — Origin-BBOX Reprojection Profile 추가

- 변경:
  - `wiris_stereo_distance/reprojection_profile.py`를 추가해 GT reprojection pixel residual `duL/dvL/duR/dvR` label 생성, screen grid + disparity/range bin + median profile, exact/nearest lookup, no-op fallback, JSON 저장/로드를 구현했다.
  - 측정 경로에서 Reprojection Profile JSON을 적용하면 보정 전 bbox center로 raw disparity/range를 구한 뒤 corrected bbox center를 만들고, 그 pixel로 stereo ray triangulation을 다시 수행하게 했다.
  - 기록 row에 left/right bbox center, width, height, confidence, `raw_disparity_px`, `raw_range_m`, corrected bbox center, `left/right_bbox_du/dv_px`, corrected `Right/Down/Forward/Range`, `profile_id`, fallback 정보를 추가했다.
  - 기존 `Cent_Bbox_Adj_Mtrx.csv` 전역 affine은 baseline 후보로 유지하고, 기존 3D m residual Bias/Origin-BBOX profile은 legacy 비교용으로 분리했다.
  - Simulation profile CLI에 `origin-bbox-reprojection` build/evaluate 경로를 추가했다.
  - `docs/PROJECT.md`와 `docs/usage.md`에 ref 방식과의 차이, Reprojection Profile 정의, 적용/fallback 조건, 검증 절차를 문서화했다.
- 이유:
  - 3D m-offset residual 보정은 stereo geometry를 다시 풀지 않아 주 보정 방식으로 부적절하므로, bbox center pixel residual을 보정한 뒤 triangulation을 재수행하는 구조가 필요했다.
- 확인:
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest -q tests/test_reprojection_profile.py tests/test_measurement_controller.py tests/test_simulation_origin.py` → 17 passed
  - `PYTHONPYCACHEPREFIX=/tmp/stereodistance_pycache python3 -m compileall -q wiris_stereo_distance scripts/simulation_origin_tools.py`
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest -q` → 273 passed
- 남은 위험:
  - 실제 5개 random overlap-safe 검증 궤적은 아직 실행하지 않았다. 새 profile JSON은 검증 궤적에서 축별 RMSE non-regression gate를 통과한 뒤 본 실험 main profile로 채택해야 한다.

## v1.5.41 — 2026-06-09 — 측정 탭 live range overlay 추가

- 변경:
  - 측정 탭 거리 그래프의 Range pane에 `YOLO Range`와 `Live Truth Range`를 함께 표시할 수 있게 했다.
  - 좌표 보정 profile과 실시간 UAM N/E/D telemetry가 모두 있을 때 `live_truth_range_m`, `live_range_error_m`, `live_range_error_basis`, `live_range_alignment_profile_id`를 거리 history point와 recording row에 남긴다.
  - 그래프 선택 상세에 Live Truth Range, Range Error, Profile 정보를 표시한다.
  - `docs/PROJECT.md`와 `docs/usage.md`에 실시간 그래프는 운용 참고용이고, 최종 평가는 ULog 기반 UAM 비교에서 수행한다는 구분을 추가했다.
- 이유:
  - 본 실험 중 운용자가 YOLO range와 live truth range를 같은 시간축에서 즉시 비교하되, 실시간 telemetry 지연/수신시각 오차를 최종 평가와 혼동하지 않게 하기 위해서다.
- 확인:
  - `python3 -m compileall wiris_stereo_distance tests/test_stereo.py`
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_stereo.py::test_distance_history_plot_tracks_xyz_and_distance_panels tests/test_stereo.py::test_bias_recording_saves_bbox_drone_raw_corrected_and_error_fields -q` → 2 passed

## v1.5.40 — 2026-06-09 — 본 실험 range 기준을 baseline 중앙점으로 고정

- 변경:
  - UAM 분석 row에 `estimated_baseline_center_distance_m`, `truth_baseline_center_distance_m`, `range_error_basis`를 추가했다.
  - 좌표 보정 profile 적용 시 `origin_offset_n/e/d`를 baseline 중앙점의 PX4 local N/E/D 위치로 보고, truth range를 `||truth_ned - origin_offset_ned||`로 계산하게 했다.
  - `range_error_m`은 profile 적용 후 `estimated_baseline_center_distance_m - truth_baseline_center_distance_m`가 되도록 정리했다.
  - UAM 비교 테이블에 YOLO Range, Truth Range, Range Err 컬럼을 추가했다.
  - `docs/PROJECT.md`에 본 실험 거리값은 baseline 중앙점에서 UAM까지의 range라고 명시했다.
- 이유:
  - 본 실험의 거리 평가는 local N/E/D 원점으로부터의 norm이 아니라 좌/우 카메라 baseline 중앙점에서 UAM까지의 거리여야 하기 때문이다.
- 확인:
  - `python3 -m compileall wiris_stereo_distance tests/test_uam_telemetry.py tests/test_stereo.py`
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_uam_telemetry.py tests/test_stereo.py::test_uam_alignment_estimate_applies_ned_origin_correction tests/test_stereo.py::test_uam_time_offset_estimate_applies_and_runs_analysis tests/test_stereo.py::test_uam_analysis_uses_recorded_truth_fallback tests/test_stereo.py::test_bias_recording_saves_bbox_drone_raw_corrected_and_error_fields -q` → 17 passed

## v1.5.39 — 2026-06-09 — 장착 보정 용어 정리

- 변경:
  - 런타임 UI, 파라미터 요약, CSV 메타데이터, 테스트, `docs/usage.md`, `UPDATE.md`에서 개별 장착 보정 용어를 제거했다.
  - CSV 메타데이터 이름을 장착 가정 중심의 `mount_assumption_*`로 바꿨다.
  - 앱 화면에는 `nominal shared rig`와 baseline/Pixhawk 자세 사용 가정만 표시하게 했다.
  - 상세 결정 문장은 `docs/PROJECT.md`의 측정 리그 장착 단순화 결정에만 남겼다.
- 이유:
  - 개별 장착 보정은 현 장비 조건에서 앱 기능으로 다루지 않기로 했으므로, UI와 기록 필드에 해당 보정처럼 보이는 이름이 남으면 운용자가 다시 설정해야 하는 값으로 오해할 수 있기 때문이다.
- 확인:
  - `python3 -m compileall wiris_stereo_distance tests/test_triangulation.py tests/test_settings.py tests/test_stereo.py tests/test_measurement_controller.py`
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_settings.py tests/test_stereo.py::test_pixhawk_translation_uses_nominal_shared_rig_without_mount_correction_ui tests/test_stereo.py::test_bias_recording_saves_bbox_drone_raw_corrected_and_error_fields -q` → 8 passed
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_triangulation.py tests/test_settings.py tests/test_measurement_controller.py tests/test_stereo.py -q` → 161 passed

## v1.5.38 — 2026-06-09 — 장착 보정 기능 제거

- 변경:
  - 장착 모델을 `nominal-shared-rig-v1`로 고정하고, 개별 장착 보정값을 앱 보정 변수에서 제거했다.
  - GUI의 `외부 파라미터 / Baseline`에서 개별 장착 보정 입력 행을 삭제하고, 동일 리그 장착 가정 상태만 표시하게 했다.
  - 설정 파일 로드/저장에서 개별 장착 보정 필드를 제거하고, 기존 설정 JSON에 남은 해당 키는 무시하게 했다.
  - `triangulation.py`와 `measurement_controller.py`에서 per-camera 장착 보정 적용 경로를 제거하고, nominal mount와 Pixhawk attitude 기반 변환만 유지했다.
  - 거리/외부 파라미터 CSV에는 개별 장착 보정 수치 대신 `mount_assumption_model=nominal-shared-rig-v1`와 장착 가정을 남기도록 했다.
  - `docs/PROJECT.md`, `docs/usage.md`, 기본 `stereo_distance_settings.json`을 새 장착 가정에 맞춰 갱신했다.
- 이유:
  - 좌우 WIRIS 리그는 알루미늄 가공 케이스로 축을 맞추고, 각 Optical-center와 GNSS antenna의 상대 배치가 동일하므로 baseline 차분에는 공통 offset이 남지 않는다고 보기 때문이다.
- 확인:
  - `python3 -m compileall wiris_stereo_distance tests/test_triangulation.py tests/test_settings.py tests/test_stereo.py tests/test_measurement_controller.py`
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_triangulation.py tests/test_settings.py tests/test_measurement_controller.py tests/test_stereo.py::test_pixhawk_translation_scales_to_measurement_baseline tests/test_stereo.py::test_pixhawk_translation_uses_nominal_shared_rig_without_mount_correction_ui tests/test_stereo.py::test_camera_parameters_tab_shows_intrinsics_and_live_extrinsics tests/test_stereo.py::test_bias_recording_saves_bbox_drone_raw_corrected_and_error_fields tests/test_stereo.py::test_rtk_alignment_view_separates_antenna_and_camera_baseline -q`
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_triangulation.py tests/test_settings.py tests/test_measurement_controller.py tests/test_stereo.py -q` → 161 passed

## v1.5.37 — 2026-06-09 — RPY 잔여 오차 보정 포기 범위 명시

- 변경:
  - `docs/PROJECT.md`에 Roll/Pitch/Yaw가 만드는 장거리 bbox 중심 편향과 stereo depth 증폭 오차를 origin profile의 보정 목표에서 제외한다고 명시했다.
  - Pixhawk attitude는 좌표 변환과 baseline 방향 변환에 계속 사용하되, 남는 RPY 민감도는 기록/분석/한계 보고 대상으로 처리한다고 정리했다.
- 이유:
  - 25 m baseline, 8배줌, 1852 m급 장거리에서는 작은 상대 RPY 오차가 거리 오차로 크게 증폭되며, bbox 중심 기반 profile로 이를 안정적으로 제거할 수 없기 때문이다.
- 확인:
  - 문서 변경만 수행했다. 이후 v1.5.38에서 개별 장착 보정 기능을 제거했다.

## v1.5.36 — 2026-06-09 — 본 실험 8배줌 조건 명시

- 변경:
  - `docs/PROJECT.md`에 본 실험 카메라 조건을 WIRIS visible `zoom index 8 / 8.0x`로 명시했다.
  - 최종 분석에는 명목 `fx` 추정값이 아니라 해당 줌에서 체커보드로 만든 `intrinsics_zoom_08.json`을 적용해야 한다고 정리했다.
  - 앱의 명목 `fx≈8242 px` 기준으로 1852 m 거리의 15 m급 UAM은 약 67 px 폭으로 보인다는 계산을 보정 가능성 판단용으로 추가했다.
- 이유:
  - 8배줌에서는 장거리 검출 가능성 판단이 1배줌과 크게 달라지지만, 줌별 intrinsics가 없으면 최종 거리/좌표 오차의 근거로 사용할 수 없기 때문이다.
- 확인:
  - 현재 저장된 intrinsics 파일은 `intrinsics_zoom_01.json`뿐이며, `intrinsics_zoom_08.json`은 아직 없음을 확인했다.

## v1.5.35 — 2026-06-09 — UAM 폭 기반 bbox sanity check 제외

- 변경:
  - `docs/PROJECT.md`에 15 m급 UAM 가로폭 정보를 bbox 폭 기반 sanity check나 거리 정확도 gate로 사용하지 않는다는 제약을 명시했다.
  - 최종 정확도 평가는 UAM 명목 폭이 아니라 Pixhawk/GNSS body 원점의 local N/E/D와 시간 매칭된 YOLO 3D 추정값의 오차로 수행한다고 정리했다.
- 이유:
  - 본 실험 전 드론 테스트는 UAM과 표적 폭이 다르고, 실제 UAM에서도 YOLO bbox가 기체 전체 가로폭을 항상 포함한다고 보장할 수 없기 때문이다.
- 확인:
  - 문서 변경만 수행했다. 앱 코드에는 15 m 폭 기반 sanity check 로직이 존재하지 않음을 `rg`로 확인했다.

## v1.5.34 — 2026-06-09 — Raw NED 그래프 식별 정보 추가

- 변경:
  - 공용 telemetry graph에 우측 상단 compact legend를 추가해 series 색상과 축 이름을 표시하게 했다.
  - 그래프 hover 시 가까운 point의 series, `t` 시간, 값을 tooltip으로 표시하게 했다.
  - x축 하단에 시작/끝 `t` 시간을 표시하게 했다.
  - `Raw NED`는 N/E/D 순서로 legend와 색상이 고정되도록 series ordering을 추가했다.
- 이유:
  - ULog Raw N/E/D 확인 단계에서 어떤 색이 어떤 축인지 즉시 알아야 하고, 특정 지점의 시간값을 분석 매칭과 함께 확인해야 하기 때문이다.
- 확인:
  - Raw NED plot 테스트에 series label, 순서, hover text, time bounds 검증을 추가했다.
- 남은 위험:
  - 실제 마우스 hover의 화면 위치감은 Qt offscreen 테스트로는 픽셀 단위 확인하지 못했다.

## v1.5.33 — 2026-06-09 — 좌표 계약 고정

- 변경:
  - `wiris_stereo_distance/coordinate_contract.py`를 추가해 world/body/camera/truth/measurement frame과 axis contract를 단일 코드 계약으로 고정했다.
  - `gui.py`, `uam_telemetry.py`, `triangulation.py`의 중복 축 상수 정의를 제거하고 새 계약 파일을 import하도록 정리했다.
  - `docs/COORDINATE_CONTRACT.md`를 추가해 `Right/Down/Forward`, `North/East/Down`, legacy CSV field, `mixed-frame`, `estimated_north/east/down_m` 최종 비교 규칙을 문서화했다.
  - 문서 목차와 프로젝트 문서에서 좌표 계약 문서를 참조하도록 했다.
- 이유:
  - UAM 최종 정확도 평가에서 축 이름이나 CSV `x/y/z` 호환 필드를 잘못 해석하면 결과가 무의미해지므로, 코드와 문서가 같은 계약을 공유해야 하기 때문이다.
- 확인:
  - 좌표 계약 공유 테스트를 추가했다.
- 남은 위험:
  - 외부 분석 스크립트가 legacy `x_m/y_m/z_m`을 직접 읽는 경우에는 새 계약 문서를 기준으로 해석을 갱신해야 한다.

## v1.5.32 — 2026-06-09 — UAM 좌표 보정 profile 적용

- 변경:
  - UAM 분석에 `camera Right/Down/Forward -> PX4 local North/East/Down` rigid transform profile을 추가했다.
  - `저장·분석 > UAM 비교`에 `좌표 보정 자동 추정`, `보정 저장`, `보정 불러오기`, `보정 해제`를 추가했다.
  - 자동 추정은 현재 매칭된 YOLO row와 ULog/live/recorded truth row에서 회전행렬과 `origin_offset_n/e/d`를 계산하며, 3개 미만 또는 직선형 퇴화 샘플은 거부한다.
  - profile이 적용되면 분석 row에 `estimated_north/east/down_m`, `error_north/east/down_m`, `position_error_m`, `alignment_profile_id`를 남기고, 오차 그래프는 N/E/D 최종 위치 오차를 우선 표시한다.
  - profile JSON은 세션 metadata에도 같이 기록해 어떤 보정으로 `uam_analysis.csv`가 만들어졌는지 추적할 수 있게 했다.
- 이유:
  - 본 실험에서는 UAM 위치 자체와 무관하게 YOLO 추정점을 UAM 내부 Pixhawk의 PX4 local N/E/D와 같은 좌표계에서 비교해야 하며, 보정 세션에서 얻은 축 일치화와 origin offset을 저장해 재사용해야 하기 때문이다.
- 확인:
  - `python3 -m compileall wiris_stereo_distance tests/test_uam_telemetry.py tests/test_stereo.py`
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_uam_telemetry.py::test_estimate_alignment_profile_maps_camera_points_to_truth_ned tests/test_uam_telemetry.py::test_uam_alignment_profile_round_trips_json_payload tests/test_stereo.py::test_uam_alignment_estimate_applies_ned_origin_correction tests/test_stereo.py::test_uam_alignment_profile_save_and_load -q`
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_uam_telemetry.py tests/test_stereo.py::test_uam_time_offset_estimate_applies_and_runs_analysis tests/test_stereo.py::test_uam_analysis_uses_recorded_truth_fallback tests/test_stereo.py::test_uam_truth_plot_refreshes_raw_plot tests/test_stereo.py::test_visible_buttons_do_not_overlap_at_compact_width -q`
- 남은 위험:
  - 데이터 기반 rigid transform은 같은 장착 조건에서만 재사용해야 한다. 카메라, 렌즈 zoom/intrinsics, Pixhawk/GNSS 위치, UAM 장착 자세가 바뀌면 새 profile을 만들어야 한다.

## v1.5.31 — 2026-06-09 — 중복 UI 진입점 정리

- 변경:
  - YOLO 탭의 `고급 설정 저장`과 측정 `계산 설정`의 `계산 설정 저장` 버튼을 제거하고, 설정 저장 진입점을 `저장·분석 > 전체 설정 저장` 하나로 통합했다.
  - `고급 > Bias 보정`의 legacy `drone_x/y/z` watch 등록 폼을 제거하고, `연결 > UAM`에서 수신된 UAM 중심 위치 상태만 표시하도록 정리했다.
  - `저장·분석 > UAM 비교`의 ULog 버튼을 `ULog 선택/로드`와 `현재 경로 다시 로드`로 구분해 파일 선택 즉시 로드와 수동 경로 재로드 역할을 분리했다.
  - legacy `drone_x/y/z` 값은 새 UI 등록 경로에서는 제거했지만, 과거 기록 row와 bias profile 호환을 위해 분석 fallback으로만 유지했다.
- 이유:
  - 최종 워크플로우에서 설정 저장은 전역 저장 하나면 충분하고, UAM ground truth 수신은 `연결 > UAM`이 단일 표준 진입점이어야 하기 때문이다.
- 확인:
  - `python3 -m compileall wiris_stereo_distance tests/test_stereo.py tests/test_uam_telemetry.py`
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_stereo.py::test_yolo_advanced_fields_have_visible_names tests/test_stereo.py::test_settings_save_entry_is_global_only tests/test_stereo.py::test_drone_position_watch_updates_latest_drone_position tests/test_stereo.py::test_uam_mavlink_watch_updates_live_buffer tests/test_stereo.py::test_camera_parameters_tab_shows_intrinsics_and_live_extrinsics tests/test_stereo.py::test_visible_buttons_do_not_overlap_at_compact_width -q`
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_uam_telemetry.py -q`
- 남은 위험:
  - legacy CSV/row fallback은 호환을 위해 남아 있다. 신규 실험에서는 `연결 > UAM`과 ULog/live buffer source를 우선 사용해야 한다.

## v1.5.30 — 2026-06-09 — 좌표 표시를 의미 이름으로 전환

- 변경:
  - 카메라 측정값 표시를 `X/Y/Z` 또는 `X/Right` 대신 `Right/Down/Forward`로 바꿨다.
  - UAM/PX4 기준 위치 설명은 `N/E/D` 약어와 함께 `North/East/Down` 의미 이름을 우선 사용하도록 정리했다.
  - 거리 히스토리, UAM 비교 표, triangulation 품질, RTK 정렬 상세, bias profile 표, tooltip, 문서의 현재형 설명을 의미 이름 기준으로 갱신했다.
  - CSV 호환 필드 `x_m/y_m/z_m`, `raw_x/y/z_m`, `error_x/y/z_m`는 유지하되 UI에서는 의미 이름 alias를 우선 표시한다.
- 이유:
  - 축 글자 `X/Y/Z`만으로는 camera optical frame, body FRD, PX4 local NED를 혼동하기 쉽기 때문이다. 실험 보고와 GUI에서는 방향의 의미가 드러나는 이름이 더 안전하다.
- 확인:
  - 문자열 기대 테스트를 의미 이름 기준으로 갱신했다.

## v1.5.29 — 2026-06-09 — UAM 중심 기준과 고급 탭 재배치

- 변경:
  - `연결` 세부탭은 `WIRIS`, `Pixhawk`, `UAM` 주 연결 경로만 남기고 `WIRIS SDK`를 `고급`으로 이동했다.
  - `측정` 세부탭은 `실시간`, `계산 설정`, `삼각측량 품질`만 남기고 `중심점 보정` 기능을 `고급 > Bias 보정`으로 이동했다.
  - `고급` 세부탭을 `WIRIS SDK`, `필드 감시`, `Bias 보정`으로 정리했다.
  - `연결 > UAM`에 GPS 안테나와 Pixhawk를 UAM/드론 중앙에 둔 body origin N/E/D 기준임을 표시했다.
- 이유:
  - UAM ground truth의 주 경로는 중앙 장착 Pixhawk/GNSS local position이므로 `연결 > UAM`에서 단일하게 관리하고, 저수준 SDK 명령/범용 field watch/bias profile은 주 측정 워크플로우와 분리해야 하기 때문이다.
- 확인:
  - 탭 구조 테스트를 새 배치로 갱신했다.
- 남은 위험:
  - `고급 > Bias 보정`의 legacy `drone_x/y/z` 감시는 이전 CSV/프로파일 워크플로우 호환용이다. 신규 실험에서는 `연결 > UAM`을 ground truth 텔레메트리의 표준 진입점으로 사용해야 한다.

## v1.5.28 — 2026-06-09 — 카메라 측정 X/Y/Z와 Truth N/E/D 계약 고정

- 변경:
  - YOLO 측정 `x_m/y_m/z_m`의 원점을 왼쪽 카메라에서 좌/우 optical center 중점으로 옮겨 `stereo_center_opencv_optical` 기준으로 기록하게 했다.
  - 카메라 측정 축 계약을 `x/right,y/down,z/forward`로 명시하고, 중앙선 표적의 `estimated_x`가 0에 가까워지도록 raw left point에서 baseline/2를 보정한다.
  - UAM/PX4 Truth raw data는 UI/분석 CSV에서 `truth_north_m`, `truth_east_m`, `truth_down_m`으로 표시하고 Raw 그래프를 N/E/D 기준으로 바꿨다.
  - 기존 CSV 호환을 위해 `truth_x/y/z`, `drone_x/y/z`, `raw_x/y/z` 필드는 유지하되, 의미가 명확한 alias와 frame metadata를 추가했다.
  - YOLO camera X/Y/Z와 Truth N/E/D의 축별 차이는 `mixed-frame` 검토값으로 표시하고, 최종 축별 오차는 origin 보정과 카메라-기체 외부파라미터 변환 후 NED에서 판단하도록 문서화했다.
- 이유:
  - Simulation의 stereo center optical frame과 앱의 측정 기준을 맞추고, PX4 local position을 X/Y/Z로 오해해 카메라 좌표와 직접 비교하는 오류를 막기 위해서다.
- 확인:
  - stereo-center helper 테스트를 추가해 중앙선 표적에서 center-frame X가 0이 되는지 검증했다.
  - UAM 비교/plot 테스트를 새 N/E/D field와 frame metadata 기준으로 갱신했다.
- 남은 위험:
  - 현재 축별 error 그래프는 변환 전 mixed-frame 검토값이다. 최종 보고용 축별 정확도는 YOLO 측정점을 PX4 local NED로 변환하는 origin 보정/외부파라미터 적용 결과로 산출해야 한다.

## v1.5.27 — 2026-06-09 — UAM UTC 매칭 fallback 적용

- 변경:
  - ULog 로더가 `vehicle_gps_position.time_utc_usec` 또는 `sensor_gps.time_utc_usec`를 사용해 PX4 local timestamp를 UTC Unix 시간으로 매핑하도록 했다.
  - ULog local position timestamp는 `timestamp_sample`이 유효하면 우선 사용하고, 없거나 0이면 기존 `timestamp`로 fallback한다.
  - UAM 비교는 `bbox_mid_utc_unix_s`와 ULog UTC 매핑을 먼저 사용하고, 실패하면 기존 timestamp/time offset 매칭, 마지막으로 기록 row의 `drone_utc_unix_s` 또는 `drone_timestamp_s` fallback을 사용한다.
  - UAM 비교 테이블에 사용된 time basis를 표시하고, 분석 row/detail/CSV에 UTC timestamp와 basis 필드를 남긴다.
  - `docs/PROJECT.md`와 `docs/usage.md`에 UTC-first time-basis fallback 순서를 명시했다.
- 이유:
  - RTP packet 자체에는 UTC가 없기 때문에 RTCP Sender Report로 기록된 bbox UTC와 PX4 ULog의 GNSS UTC를 우선 매칭하고, RTCP/UTC가 없는 현장에서는 기존 time offset 및 실시간/기록 fallback으로 분석 가능성을 유지해야 하기 때문이다.
- 확인:
  - UTC 보간, ULog UTC 매핑, timestamp fallback, 기록 row UTC fallback, UTC 기반 offset 추정 테스트를 추가했다.
- 남은 위험:
  - ULog UTC 매핑 정확도는 GNSS UTC topic 품질과 PX4 timestamp_sample 유효성에 의존한다.
  - UTC fallback은 시간축만 맞춘다. YOLO camera-frame X/Y/Z와 PX4 local position X/Y/Z를 직접 비교하려면 별도의 origin 보정과 camera-body 외부파라미터 변환이 계속 필요하다.

## v1.5.26 — 2026-06-08 — 파라미터와 저장·분석 세부탭 분리

- 변경:
  - `파라미터` 탭을 `내부`, `외부/기하` 세부탭으로 나눠 내부 파라미터 확인과 live 외부 기하 확인을 분리했다.
  - `저장·분석` 탭을 `세션 저장`, `UAM 비교`, `기록 샘플` 세부탭으로 나눠 저장, ground truth 비교, 기록 row 검토를 분리했다.
  - 기존 위젯과 데이터 흐름은 유지하고 부모 탭만 바꿔 버튼/상태/테이블 동작이 그대로 이어지게 했다.
  - `docs/PROJECT.md`와 `docs/usage.md`에 새 세부탭 기준 워크플로우를 반영했다.
- 이유:
  - `저장·분석`은 세션 저장, UAM post-flight 분석, row inspection이 한 화면에 섞이면서 버튼을 찾기 어려워졌고, `파라미터`는 intrinsics와 외부 기하 상태가 서로 다른 확인 단계이기 때문이다.
- 확인:
  - 탭 구조 테스트와 UAM 분석 GUI 테스트를 갱신했다.

## v1.5.25 — 2026-06-08 — UAM 분석 워크플로 버튼 완성

- 변경:
  - `연결 > UAM`에 `UAM 텔레메트리 해제`를 추가해 Pixhawk ROS2/MAVLink 연결은 유지하면서 `uam_x/y/z` 감시만 해제할 수 있게 했다.
  - `저장·분석 > UAM Ground Truth 비교`에 `시간 offset 추정`, `오차 그래프`, `Raw XYZ` 버튼을 명시적으로 배치했다.
  - ULog 또는 실시간 UAM ground truth Raw X/Y/Z 시계열을 분석 탭에서 바로 시각화하고, 비교 실행 후에는 오차 그래프로 자동 전환하게 했다.
  - offset 추정은 ULog/live buffer와 YOLO row를 비교해 후보값을 적용한 뒤 즉시 비교를 재실행하도록 했다.
  - `docs/PROJECT.md`와 `docs/usage.md`에 Raw 확인, offset 후보 검토, source별 결과 분리 원칙을 보강했다.
- 이유:
  - 최종 실험 워크플로우와 GUI 버튼을 1:1로 맞추고, ULog 선택이 어려운 경우에도 실시간 fallback 기반 비교를 수행할 수 있어야 하기 때문이다.
- 확인:
  - UAM offset 추정 로직, UAM telemetry 해제, Raw XYZ 시각화, UAM 비교 fallback GUI 테스트를 보강했다.
- 남은 위험:
  - 실제 ULog parsing은 `pyulog` 설치와 PX4 topic 구성에 의존한다.
  - 자동 offset은 후보 추정값이므로 최종 보고 전 matched count, timestamp delta, 오차 그래프 검토가 필요하다.

## v1.5.24 — 2026-06-08 — UAM 텔레메트리 fallback 및 ULog 비교 분석 반영

- 변경:
  - `연결 > UAM` 세부탭을 추가해 UAM/드론 ground truth 위치를 ROS2 `vehicle_local_position` 또는 MAVLink `LOCAL_POSITION_NED`로 실시간 수신하도록 했다.
  - 실시간 `uam_x/y/z` 수신값을 기존 중심점 보정의 `drone_x/y/z` 상태와 같은 최신 기체 중심 위치로 합쳐, bias profile 기록과 분석 fallback에서 재사용하게 했다.
  - `저장·분석` 탭에 ULog 선택/로드, 최대 시간차, time offset, YOLO-vs-ground-truth 비교 테이블, 오차 그래프를 추가했다.
  - 분석 source 우선순위를 ULog, 실시간 UAM 텔레메트리 버퍼, 기록 row 내장 `drone_x/y/z` fallback 순서로 고정했다.
  - 세션 저장 시 분석 결과가 있으면 `uam_analysis.csv`를 추가 저장하고 metadata에 ULog/live sample count와 비교 설정을 남기도록 했다.
  - `docs/PROJECT.md`와 `docs/usage.md`에 UAM 최종 실험 ground truth 방법, ULog 우선 분석, 실시간 fallback 운용을 명시했다.
- 이유:
  - 최종 UAM 15 m 폭 표적 실험은 드론 평가와 UAM 적용 모두에서 같은 기준 위치 획득/분석 방법을 써야 하며, ROS2가 막힌 현장에서도 MAVLink 실시간 위치 fallback으로 비교 가능성을 유지해야 하기 때문이다.
- 확인:
  - 순수 UAM 비교 로직, 설정 roundtrip, 세션 저장, GUI UAM watch/분석 흐름 테스트를 추가했다.
- 남은 위험:
  - 실제 PX4 `.ulg` 로드는 `pyulog` 설치와 ULog topic 구성에 의존한다.
  - 실시간 fallback은 앱 수신 시각 기준이므로 최종 정확도 평가는 ULog 기반 결과와 분리해 보고해야 한다.
  - 실제 ROS2/MAVLink 장비 수신은 하드웨어 smoke check가 필요하다.

## v1.5.23 — 2026-06-08 — 카메라-기체 외부파라미터 앱 반영

- 변경:
  - 카메라별 개별 장착 보정 모델을 추가해 계산 경로에 반영했다.
  - 장착 offset은 각 Pixhawk body-to-world 자세로 world frame에 회전한 뒤 body 원점 baseline과 합성하고, left camera frame translation으로 변환한다.
  - 3D ray triangulation의 좌/우 camera ray 변환에도 장착 자세 보정을 적용했다.
  - GUI 외부 파라미터 설정 영역에 좌/우 개별 장착 보정 입력을 추가하고 설정 파일 저장/로드에 포함했다.
  - 파라미터 탭, 현재 적용값 요약, 거리 기록 CSV row에 적용 모델, 상태, 개별 장착 보정 수치를 남긴다.
  - `docs/PROJECT.md`의 앱 반영 상태를 구현 완료 범위와 남은 추정 UI 범위로 갱신했다.
- 이유:
  - UAM/드론 yaw, pitch, roll 움직임에서 body frame에 고정된 카메라 optical center 위치가 달라지므로, RTK/Pixhawk 기반 translation과 ray 방향 계산에 카메라-기체 외부파라미터가 직접 반영되어야 하기 때문이다.
- 확인:
  - `python3 -m compileall wiris_stereo_distance tests/test_triangulation.py tests/test_settings.py tests/test_stereo.py`
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_triangulation.py tests/test_measurement_controller.py tests/test_settings.py tests/test_stereo.py::test_pixhawk_translation_scales_to_measurement_baseline tests/test_stereo.py::test_pixhawk_translation_applies_yaw_rotated_mount_offset tests/test_stereo.py::test_bias_recording_saves_bbox_drone_raw_corrected_and_error_fields -q`
- 남은 위험:
  - 보정값 추정 UI, 보정 생성 timestamp, residual 요약, 보정 JSON import/export는 아직 구현하지 않았다.
  - `tests/test_stereo.py` 전체 실행은 기존으로 보이는 `vehicle_magnetometer` sensor slot 기대값과 distance label stylesheet 기대값 2건에서 실패했다.

## v1.5.22 — 2026-06-08 — 장착 offset yaw 회전 효과 명시

- 변경:
  - `docs/PROJECT.md`의 카메라-기체 외부파라미터 보정 결정에 장착 offset world 위치 계산식을 추가했다.
  - `p_camera_world(t) = p_body_origin_world(t) + R_world_body(t) * t_body_camera`를 명시했다.
  - 수평 yaw만 있는 경우 `x_fwd/y_right/z_down` 장착 offset이 N/E/D로 어떻게 회전하는지 예시 수식을 추가했다.
  - 앱 반영 순서에 yaw/pitch/roll에 따른 카메라 world 위치 계산 함수를 포함했다.
- 이유:
  - 드론/UAM yaw 움직임이 body frame에 고정된 장착 offset을 world frame에서 회전시키므로, 이를 무시하면 카메라 optical center 위치와 거리 계산에 체계 오차가 생기기 때문이다.
- 확인:
  - `docs/PROJECT.md`와 `UPDATE.md` 내용을 재확인했다.
- 남은 위험:
  - 아직 실제 장착 offset 회전 계산은 코드에 구현하지 않았다.

## v1.5.21 — 2026-06-08 — 카메라-기체 외부파라미터 보정 계획 확정

- 변경:
  - `docs/PROJECT.md`의 좌표계 보정 섹션을 `카메라-기체 외부파라미터 보정 결정`으로 확장했다.
  - 개별 장착 보정, nominal mount profile, 적용 수식, 보정 입력, 지상 보정 절차, 검증 gate, 저장 항목, 앱 반영 순서를 명시했다.
  - `현재 결합 지점과 분리 계획`에 개별 장착 보정 설정 및 계산 반영 항목을 추가했다.
- 이유:
  - 앱 구현 전에 좌표계 결정만으로는 부족한 보정 범위, 정량 검증 기준, 저장 스키마, 구현 순서를 하나의 계획으로 확정해야 하기 때문이다.
- 확인:
  - `docs/PROJECT.md`와 `UPDATE.md` 내용을 재확인했다.
- 남은 위험:
  - 아직 실제 개별 장착 보정 설정 저장, 계산 적용, 추정 UI는 구현하지 않았다.

## v1.5.20 — 2026-06-08 — 카메라-기체 좌표계 보정 결정 명시

- 변경:
  - `docs/PROJECT.md`에 `카메라-기체 좌표계 보정 결정` 섹션을 추가했다.
  - PX4 world NED, body FRD, 카메라 optical frame 기준을 명시했다.
  - 전방 감시 카메라는 camera +Z를 body +Z가 아니라 body +X에 대응시키고, camera +Y를 body +Z에 대응시키는 현재 mount profile을 프로젝트 결정으로 정리했다.
  - 실제 장착 오차는 각 카메라별 정적 mount bias로 추정하고, 비행 중 Pixhawk attitude는 동적 자세로만 사용한다는 보정 방법을 적었다.
- 이유:
  - 카메라 z축과 드론/UAM z축을 같은 축으로 맞춘다는 오해를 막고, 현재 triangulation 코드와 PX4/ROS 좌표계 관례에 맞는 실험 방법을 문서화하기 위해서다.
- 확인:
  - `docs/PROJECT.md`와 `UPDATE.md` 내용을 재확인했다.
- 남은 위험:
  - 문서 결정만 수행했으며 mount bias를 설정 파일에 저장하거나 UI에서 추정하는 기능은 아직 구현하지 않았다.

## v1.5.19 — 2026-06-08 — 프로젝트 목적과 방법론 보강

- 변경:
  - `docs/PROJECT.md`에 `프로젝트 목적 상세` 섹션을 추가했다.
  - `docs/PROJECT.md`에 `프로젝트 방법론` 섹션을 추가해 입력 검증, 단계별 운용, source 추적, 3D ray triangulation 중심, MAVLink 복구/fallback, 세션 단위 검증 저장 원칙을 정리했다.
- 이유:
  - 문서가 운용 조건과 방법뿐 아니라 프로젝트가 왜 존재하고 어떤 원칙으로 측정 신뢰도를 확보하는지 설명해야 하기 때문이다.
- 확인:
  - `docs/PROJECT.md`와 `UPDATE.md` 내용을 재확인했다.
- 남은 위험:
  - 문서 보강만 수행했으며 실제 기능 동작은 변경하지 않았다.

## v1.5.18 — 2026-06-08 — 프로젝트 문서 통합

- 변경:
  - `docs/architecture.md`의 구조 개요, 주요 모듈, 측정 흐름, 결합 지점, 분리 방향을 `docs/PROJECT.md`로 통합했다.
  - `docs/field-workflow.md`의 현장 운용 순서, 연결 확인 기준, 저장 기준을 `docs/PROJECT.md`로 통합했다.
  - `docs/refactor-roadmap.md`의 분리 우선순위와 리팩터링 원칙을 `docs/PROJECT.md`의 `현재 결합 지점과 분리 계획`으로 흡수했다.
  - 통합 후 중복 문서인 `docs/architecture.md`, `docs/field-workflow.md`, `docs/refactor-roadmap.md`를 삭제했다.
  - `docs/README.md`, `docs/usage.md`, `docs/AGENTS.md`의 문서 목록을 새 구조에 맞게 정리했다.
- 이유:
  - 프로젝트 목적, 구조, 현장 워크플로, 분리 계획이 여러 문서에 흩어져 같은 의도를 다르게 설명할 수 있었기 때문이다.
- 확인:
  - 삭제 대상 문서의 핵심 내용이 `docs/PROJECT.md`에 흡수됐는지 확인하고, 현재 문서 목록에서 삭제된 파일 참조를 점검했다.
- 남은 위험:
  - 오래된 변경 이력 항목에는 당시 추가됐던 문서명이 과거 기록으로 남아 있다.

## v1.5.17 — 2026-06-08 — Pixhawk ROS2/MAVLink 연결 의도 명시

- 변경:
  - `docs/PROJECT.md`에 `Pixhawk 연결 의도` 섹션을 추가했다.
  - ROS2를 주 경로로 살리되, 실패 시 MAVLink로 Pixhawk 쪽 설정값을 전달해 ROS2 복구를 시도하고 가능한 telemetry fallback을 유지한다는 의도를 명시했다.
  - MAVLink fallback이 대체하지 못하는 ROS2 전용 `vehicle_magnetometer.device_id`, namespace 기반 topic 식별, custom uORB field, 구조화된 parameter service 한계를 함께 적었다.
- 이유:
  - 최근 ROS2 실패 시 MAVLink 복구/fallback 자동화 변경의 사용자 의도와 운용 원칙을 프로젝트 개요 문서에 명확히 남기기 위해서다.
- 확인:
  - `docs/PROJECT.md`와 `UPDATE.md` 내용을 재확인했다.
- 남은 위험:
  - 문서 의도 명시만 수행했으며 실제 연결 동작은 변경하지 않았다.

## v1.5.16 — 2026-06-08 — ROS2 실패 시 MAVLink 복구/fallback 자동화

- 변경:
  - ROS2 Pixhawk 세션 시작이 실패하면 설정된 MAVLink 연결을 자동으로 시작하도록 했다.
  - MAVLink 연결 후 선택된 `MAVLink stream 설정`과 `ROS2 client 시작` 옵션에 따라 Pixhawk 쪽 MAVLink stream/uXRCE-DDS client 설정을 전송하도록 연결 흐름을 재사용했다.
  - MAVLink가 `uxrce_dds_client` 시작 명령을 전송하면 잠시 후 ROS2 연결을 한 번 재시도하고, 재시도 실패 시 MAVLink telemetry fallback을 유지하도록 했다.
  - ROS2와 MAVLink 세션이 복구 과정에서 공존할 수 있도록 기존 상호 해제 동작을 제거했다.
  - `docs/PROJECT.md`와 `docs/usage.md`에 ROS2 복구 경로와 MAVLink fallback 한계를 기록했다.
- 이유:
  - ROS2/uXRCE 연결이 막힌 현장 상황에서 MAVLink로 Pixhawk 설정값을 전달해 ROS2를 복구하고, 복구 전까지 가능한 자세/GPS/MAG telemetry를 유지해야 하기 때문이다.
- 확인:
  - ROS2 시작 실패 시 MAVLink 복구 호출과 uXRCE 설정 전달을 확인하는 회귀 테스트를 추가했다.
- 남은 위험:
  - 실제 ROS2 복구 성공은 Pixhawk MAVLink heartbeat, PX4 shell serial control, 네트워크, MicroXRCEAgent, DDS 설정 상태에 의존한다.
  - MAVLink fallback은 ROS2 전용 `vehicle_magnetometer.device_id`, namespace 기반 topic 식별, arbitrary uORB custom field를 완전히 대체하지 못한다.

## v1.5.15 — 2026-06-08 — 프로젝트 문서 최신 구조 반영

- 변경:
  - `docs/PROJECT.md`의 목적, 현재 범위, 핵심 목표를 현재 WIRIS/Pixhawk/YOLO 운용 흐름 기준으로 보강했다.
  - 실제 `wiris_stereo_distance` 패키지에 맞춰 `theme.py`, `photo_clock.py`, `sync.py`, `attitude_bias.py`, `px4_mag_manager.py`, `xrce_agent.py`, `wiris_zoom.py`, `measurement_widgets.py` 설명을 추가했다.
  - 주요 워크플로와 현재 주의점에 preview/YOLO 분리, ROS2/MAVLink 실제 수신 확인, 앱 루트 기준 설정 경로를 반영했다.
- 이유:
  - `PROJECT.md`가 최근 모듈 분리와 Pixhawk 연결 진단 흐름을 충분히 설명하지 못해 프로젝트 개요 문서로서 최신성이 떨어졌기 때문이다.
- 확인:
  - 현재 파일 목록과 세부 문서를 대조하고, 문서 참조 파일 존재 여부를 확인했다.
- 남은 위험:
  - 문서 정리만 수행했으며 실제 하드웨어 동작은 변경하지 않았다.

## v1.5.14 — 2026-06-08 — 보정/시간 저장 기본 위치 앱 내부 고정

- 변경:
  - `stereo_distance_settings.json`이 없으면 앱 루트 안에 기본 설정 파일을 자동 생성하도록 했다.
  - 보정 intrinsics 상대경로와 시간탭 FTP 다운로드 상대경로를 shell 현재 디렉터리가 아니라 앱 루트 기준으로 해석하도록 바꿨다.
  - 시간탭 FTP user/password/port/remote dir/download dir/대기 시간을 설정 JSON에 저장하고 재시작 시 복원하도록 했다.
  - 기본 시간탭 다운로드 폴더를 앱 루트의 `clock_captures/`로 고정했다.
- 이유:
  - 앱을 상위 디렉터리에서 실행하면 기존 `Path.cwd()` 기준 때문에 보정값이나 시간탭 다운로드 파일이 `StereoDistance` 밖에 생길 수 있었기 때문이다.
- 확인:
  - 설정 파일 생성/roundtrip, 앱 루트 상대경로 저장, 시간탭 기본 경로 테스트와 `compileall`로 확인했다.
- 남은 위험:
  - 이미 사용자가 절대경로로 지정한 보정/시간 저장 위치는 그대로 존중한다.

## v1.5.13 — 2026-06-08 — Pixhawk 작업 순서 UI 재배치

- 변경:
  - `연결 > Pixhawk` 섹션을 `0 공통 준비`, `1 탐색 후보 / USB 할당`, `2 Pixhawk 1/2 사전 설정`, `3 연결 실행` 순서로 재배치했다.
  - USB/ROS2 후보 탐색과 USB 대상 선택을 같은 후보 그룹에 묶고, Pixhawk 1/2는 각각 독립 패널에서 MAVLink URL, baudrate, ETH IP, namespace, PX4 작업을 관리하도록 했다.
  - Pixhawk UI 겹침 회귀 테스트를 새 그룹 구조 기준으로 갱신했다.
- 이유:
  - USB 직접 연결은 최초 설정/복구 로직이면서 MAVLink/ROS2 연결 전 사전 로직이므로, 물리 연결 후 작업 순서가 UI에서도 그대로 보여야 하기 때문이다.
- 확인:
  - Pixhawk UI 테스트와 `compileall`로 확인했다.
- 남은 위험:
  - `MAVLink 연결`과 `ROS2 연결` 실행 버튼은 아직 전체 연결 단위다. 좌/우 단독 연결 실행까지 분리하려면 연결 관리자 수명주기를 추가로 나눠야 한다.

## v1.5.12 — 2026-06-08 — Pixhawk 연결 진단 스크립트 추가

- 변경:
  - `scripts/diagnose_pixhawk_links.py`를 추가했다.
  - 진단 스크립트가 설정값, 호스트 IP/route, UDP 포트, USB MAVLink 후보, MAVLink heartbeat, ROS2 publisher/sample, MicroXRCEAgent 로그, ARP scan 결과를 한 번에 출력하도록 했다.
  - `docs/usage.md`에 진단 실행 명령과 `result=OK` / `result=NOT_CONNECTED` 판정 기준을 추가했다.
- 이유:
  - 구버전은 ROS2 topic 이름만 보고 연결된 것처럼 보일 수 있었고, 현장에서는 실제 publisher/sample와 MAVLink heartbeat를 구분해서 봐야 하기 때문이다.
  - Pixhawk가 스위치에 물리 연결되어 있어도 Ethernet IP, MAVLink UDP, uXRCE client가 살아 있지 않으면 앱에서 연결 성공으로 만들 수 없다.
- 확인:
  - `python3 -m py_compile scripts/diagnose_pixhawk_links.py`로 스크립트 문법을 확인했다.
  - `./scripts/diagnose_pixhawk_links.py --timeout 3` 실행 결과 현재 상태가 `result=NOT_CONNECTED`임을 확인했다.
  - 진단 결과: `MAVLink Heartbeat` left/right 모두 timeout, ROS2 `/uav_0`/`/uav_1` 주요 PX4 topic 모두 `publishers=0`, `MicroXRCEAgent`는 UDP 8888에서 실행 중이나 client 접속 로그 없음, ARP scan에서 `10.0.0.231/232` 카메라만 응답.
- 남은 위험:
  - 이 변경은 원인 가시화이며 실제 Pixhawk endpoint를 생성하지는 않는다. 실제 성공에는 PX4 쪽 Ethernet/uXRCE/MAVLink 실행 또는 USB MAVLink 접속 경로가 필요하다.

## v1.5.11 — 2026-06-08 — Pixhawk 구버전 연결 판정 차이 확인

- 변경:
  - `ETH 저장`의 `extras.txt` 작성 방식을 구버전 PixhawkUorbConsole처럼 `echo >` / `echo >>` 기반으로 바꿨다.
  - PX4 NSH 환경에서 `grep -qxF` 지원 여부에 의존하지 않도록 했다.
- 이유:
  - 실제 장비 설정 명령은 PX4 shell에서 실행되므로, Linux 사용자 공간 명령에 가까운 `grep` 의존을 줄이는 편이 안전하기 때문이다.
  - 구버전의 ROS2 연결은 ROS graph의 topic 이름만 보고 연결 성공처럼 표시할 수 있는데, 현재 환경에서는 실제 PX4 publisher 없이도 `/fmu/out` topic 이름 12개가 보이는 상태임을 확인했다.
- 확인:
  - 현 버전 `discover_pixhawk_topics(current_domain_id=0, timeout_s=1.0, extra_domain_ids=(0,))`는 publisher가 없어 `None`을 반환했다.
  - 구버전 discovery 조건을 그대로 재현하면 같은 환경에서 `/fmu/out` topic 이름 12개를 발견했다.
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_stereo.py::test_persistent_pixhawk_eth_command_uses_side_ip_and_host tests/test_stereo.py::test_mavlink_start_ros2_client_sends_px4_shell_command tests/test_stereo.py::test_mavlink_start_ros2_client_retries_after_first_heartbeat tests/test_settings.py -q`로 관련 테스트 7개 통과를 확인했다.
  - `python3 -m compileall main.py wiris_stereo_distance`로 Python 문법 상태를 확인했다.
- 남은 위험:
  - Pixhawk가 실제 네트워크 endpoint로 보이지 않는 상태는 계속된다. 현재 환경에서는 구버전도 UDP MAVLink heartbeat를 받지 못하고, 현 버전도 ROS2 publisher를 찾지 못한다.

## v1.5.10 — 2026-06-08 — Pixhawk MAVLink/uXRCE 설정 경로 보강

- 변경:
  - MAVLink로 Pixhawk-side `uxrce_dds_client`를 시작할 때 기존 client를 먼저 `stop`하고, 선택된 namespace(`uav_0`, `uav_1`)를 `-n`으로 전달하도록 했다.
  - UDP `udpin` 연결에서 첫 heartbeat 전 PX4 shell 명령이 실패할 수 있어, MAVLink stream/uXRCE 설정 명령을 보관했다가 첫 heartbeat 후 재시도하도록 했다.
  - `ETH 저장` 명령에 구버전 PixhawkUorbConsole의 `UXRCE_DDS_CFG`, `UXRCE_DDS_KEY`, `UXRCE_DDS_DOM_ID`, `UXRCE_DDS_AG_IP`, `UXRCE_DDS_PRT` 설정과 `extras.txt` uXRCE 재시작 라인을 반영했다.
  - Pixhawk Host IP 기본값과 현재 설정을 구버전/현장 NIC 기준인 `10.41.10.1`로 맞췄다.
  - `docs/usage.md`에 ROS2 실제 연결 판단 기준과 MAVLink uXRCE namespace 전송 동작을 기록했다.
- 이유:
  - 백업 구버전의 DDS setup은 namespace와 DDS key/domain/agent IP를 명시했지만, 현 버전은 MAVLink로 `uxrce_dds_client start`를 보낼 때 namespace가 빠져 ROS2 topic이 기대 namespace로 나오지 않을 수 있었기 때문이다.
  - UDP MAVLink는 heartbeat 전 peer 주소를 모를 수 있으므로 연결 직후 shell 명령만으로는 설정 전송 성공을 보장하기 어렵다.
- 확인:
  - 구버전 `PX4ShellClient`도 현재 `udpin:0.0.0.0:14550/14551`에서는 heartbeat timeout임을 확인했다.
  - 현재 네트워크에서 `ros2 topic info -v /uav_0/fmu/out/vehicle_attitude`, `/uav_1/fmu/out/vehicle_attitude`의 publisher count가 0이고, `ros2 topic hz`가 샘플을 받지 못하는 것을 확인했다.
  - `arp-scan`에서 `10.0.0.231/232` 카메라만 응답하고 `10.41.10.0/24`, `192.168.0.0/24`에서 Pixhawk IP 응답이 없음을 확인했다.
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_settings.py tests/test_stereo.py::test_mavlink_start_ros2_client_sends_px4_shell_command tests/test_stereo.py::test_mavlink_start_ros2_client_retries_after_first_heartbeat tests/test_stereo.py::test_mavlink_configure_streams_sends_px4_shell_commands tests/test_stereo.py::test_persistent_pixhawk_eth_command_uses_side_ip_and_host tests/test_stereo.py::test_mavlink_connection_routes_sensor_signals_to_alignment -q`로 관련 회귀 테스트 9개 통과를 확인했다.
  - `python3 -m compileall main.py wiris_stereo_distance`로 Python 문법 상태를 확인했다.
- 남은 위험:
  - 현재 장비 네트워크에서는 Pixhawk IP와 MAVLink UDP heartbeat가 보이지 않아 실제 MAVLink/ROS2 연결 성공까지는 확인되지 않았다. Pixhawk Ethernet이 PX4에서 `ifup` 및 IP 설정된 상태이거나, USB/기존 MAVLink 경로로 한 번 접속해 `ETH 저장` 명령을 적용해야 한다.

## v1.5.9 — 2026-06-08 — MAG 관리 대기 문구 의미 정정

- 변경:
  - `PX4 Magnetometer / RM3100 관리` 대기 상태 문구를 `프로토콜 수신 대기`로 바꿨다.
  - MAG 관리 버튼 tooltip과 차단 로그를 `MAVLink/ROS2 MAG 수신 없음` 기준으로 정리했다.
- 이유:
  - Pixhawk가 카메라와 같은 스위치에 물리적으로 연결되어 있어도, 앱이 MAVLink/ROS2로 실제 MAG device_id를 받기 전에는 관리 내용을 실제 상태로 표시할 수 없기 때문이다.
- 확인:
  - MAG 관리 UI 회귀 테스트와 `compileall`로 확인했다.
- 남은 위험:
  - 이 변경은 표시 문구 정정이며, 실제 MAG 내용 표시 조건은 `vehicle_magnetometer` device_id 수신 또는 MAVLink connection 존재 기준을 유지한다.

## v1.5.8 — 2026-06-08 — Pixhawk 미연결 MAG 관리 내용 숨김

- 변경:
  - `PX4 Magnetometer / RM3100 관리` 패널이 Pixhawk MAG device_id 수신 전에는 테이블 내용을 표시하지 않도록 했다.
  - 미연결/수신 대기 상태에서는 MAG 관리 새로고침, Yaw 기준 적용, 저장, 재부팅 버튼을 비활성화한다.
  - Pixhawk ROS2/MAVLink 연결 해제 시 이전 MAG selected device_id 상태를 지우고 관리 패널을 빈 상태로 되돌린다.
- 이유:
  - 실제 Pixhawk가 연결되지 않았는데도 초기 mock 파라미터가 관리 그룹에 남아 실제 장비 상태처럼 보일 수 있었기 때문이다.
- 확인:
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_stereo.py::test_px4_mag_manager_stays_empty_until_pixhawk_mag_source tests/test_stereo.py::test_px4_mag_manager_tracks_actual_selected_mag_per_pixhawk tests/test_stereo.py::test_rm3100_yaw_source_selection_sets_px4_primary_mag tests/test_stereo.py::test_rm3100_yaw_source_selection_rejects_missing_device_id -q`로 MAG 관리 UI와 Yaw 기준 선택 회귀 테스트를 확인했다.
  - `python3 -m compileall main.py wiris_stereo_distance`로 Python 문법 상태를 확인했다.
- 남은 위험:
  - ROS2 세션만 시작되고 `vehicle_magnetometer` device_id가 아직 들어오지 않은 동안에는 관리 테이블이 계속 숨겨진다. 실제 표시에는 MAG selected device_id 수신 또는 MAVLink 연결 정보가 필요하다.

## v1.5.7 — 2026-06-08 — Pixhawk 탐색 후보 그룹 겹침 방지

- 변경:
  - `Pixhawk 탐색 후보` 그룹의 내부 여백과 행 간격을 정리하고, 전역 UI 크기 보정 뒤 그룹 최소 높이를 실제 `sizeHint` 이상으로 보장했다.
  - 자동 탐색 버튼, 후보 테이블, P1/P2 적용 버튼 행이 실제 geometry 기준으로 서로 겹치지 않는지 확인하는 UI 회귀 테스트를 추가했다.
- 이유:
  - 연결 탭의 Pixhawk 영역이 압축되면 후보 그룹 높이가 내용보다 작아져 자동 탐색 버튼, 테이블, 적용 버튼이 세로로 겹칠 수 있었기 때문이다.
- 확인:
  - compact 크기 오프스크린 렌더에서 후보 그룹 높이와 행 bounds를 확인했다.
  - Pixhawk UI 레이아웃 테스트와 `compileall`로 확인했다.
- 남은 위험:
  - 매우 작은 창에서는 Pixhawk 설정/후보 그룹의 최소 높이가 커져 로그 영역이 줄어들 수 있다.

## v1.5.6 — 2026-06-08 — 줌 명령 템플릿 펼침 겹침 방지

- 변경:
  - `줌 명령 템플릿` 그룹이 펼쳐질 때 라벨/입력 행의 실제 `sizeHint` 높이를 최소 높이로 예약하도록 했다.
  - 템플릿 그룹 내부 여백과 행 간격을 정리하고, 펼친 상태에서 4개 행이 geometry 기준으로 겹치지 않는지 확인하는 UI 회귀 테스트를 추가했다.
- 이유:
  - 기본 접힘 상태에서는 문제가 없지만, 펼치면 상위 보정 탭 레이아웃이 템플릿 그룹을 내용보다 낮게 압축해 글자가 세로로 겹칠 수 있었기 때문이다.
- 확인:
  - compact 크기 오프스크린 렌더에서 템플릿 그룹 높이와 행 bounds를 확인했다.
  - 보정탭 줌 컨트롤 UI 테스트와 `compileall`로 확인했다.
- 남은 위험:
  - 다른 데스크톱 테마에서 폰트 높이가 달라도 `sizeHint` 기반으로 다시 계산되지만, 매우 작은 창에서는 상위 스크롤 영역 사용이 필요할 수 있다.

## v1.5.5 — 2026-06-08 — Pixhawk 설정 그룹 글자 겹침 방지

- 변경:
  - `픽스호크 1/2 설정` 그룹의 행 간격과 내부 여백을 늘리고, 전역 버튼/입력칸 크기 보정 뒤 그룹 최소 높이를 실제 `sizeHint` 이상으로 보장했다.
  - Pixhawk 설정 그룹의 라벨/입력칸/작업 버튼 행이 실제 geometry 기준으로 서로 겹치지 않는지 확인하는 UI 회귀 테스트를 추가했다.
- 이유:
  - 연결 탭의 splitter가 Pixhawk 설정 영역을 압축하면 그룹 높이가 내용보다 작아져 행과 버튼 글자가 세로로 겹칠 수 있었기 때문이다.
- 확인:
  - compact 크기 오프스크린 렌더에서 그룹 높이와 행 bounds를 확인했다.
  - Pixhawk UI 레이아웃 테스트와 `compileall`로 확인했다.
- 남은 위험:
  - 실제 데스크톱 테마의 폰트 metrics가 크게 다르면 최소 높이가 더 커질 수 있으나, 현재는 Qt `sizeHint` 기반으로 추적된다.

## v1.5.4 — 2026-06-08 — 보정 이미지 단일 버튼 독립 캡처

- 변경:
  - 보정탭 캡처 버튼을 `캡처` 하나로 유지하면서 좌/우 중 체커보드가 검출된 카메라 프레임만 저장하도록 바꿨다.
  - 한쪽에서만 체커보드가 보이면 해당 쪽 내부 파라미터 보정 이미지로만 저장하고, 검출 실패 쪽 후보 파일은 삭제한다.
  - 보정 캡처 안내 문구와 `docs/usage.md`의 쌍 캡처 설명을 내부 보정용 독립 캡처 기준으로 갱신했다.
- 이유:
  - stereo R/T를 체커보드로 계산하지 않으므로, 내부 파라미터 보정 이미지는 좌/우가 동시에 체커보드를 볼 필요가 없기 때문이다.
- 확인:
  - 좌측만 체커보드 검출되는 캡처 테스트와 보정탭 UI 테스트를 실행했다.
  - `python3 -m compileall main.py wiris_stereo_distance`로 코드 문법 상태를 확인했다.
- 남은 위험:
  - OpenCV가 없는 환경에서는 캡처 시 체커보드 검출 여부를 판정할 수 없어 현재 프레임을 저장하고, 실제 보정 실행은 OpenCV 복구 후 가능하다.

## v1.5.3 — 2026-06-08 — 체커보드 stereo R/T 경로 제거

- 변경:
  - 체커보드 이미지 쌍에서 stereo R/T를 계산하던 OpenCV `stereoCalibrate` 경로를 제거했다.
  - 보정 결과 JSON은 좌/우 내부 파라미터만 저장하고, 기존 JSON의 `extrinsics` 키는 로드 시 무시하도록 정리했다.
  - 거리 계산 외부 파라미터 출처에서 `체커보드 보정 R/T` 모드를 제거하고 `Pixhawk 자세 + RTK 위치`만 남겼다.
- 이유:
  - stereo R/T는 캘리브레이션 보드로 진행하지 않고 Pixhawk 자세와 RTK translation을 기준으로 운용하기 위해서다.
- 확인:
  - 보정/측정 컨트롤러/GUI 테스트를 새 Pixhawk+RTK 경로 기준으로 갱신하고 실행했다.
  - `python3 -m compileall main.py wiris_stereo_distance`로 코드 문법 상태를 확인했다.
- 남은 위험:
  - 기존 저장 설정에 `triangulation_extrinsics_source=calibration`이 있어도 로드 시 `pixhawk`로 정규화된다.

## v1.5.2 — 2026-06-08 — 체커보드 기본 내부 코너 수정

- 변경:
  - 보정탭 체커보드 기본값을 내부 열 13, 내부 행 8, 격자 크기 40.0 mm로 맞췄다.
- 이유:
  - 실제 보정 체커보드의 내부 코너 수와 GUI 기본값을 일치시키기 위해서다.
- 확인:
  - 보정탭 기본값 위젯 테스트와 `compileall`로 확인했다.
- 남은 위험:
  - 기존에 저장된 사용자 설정 파일이 별도로 이 값을 저장하고 있다면 실행 시 해당 저장값이 우선될 수 있다.

## v1.5.1 — 2026-06-08 — 보정 미리보기 비동기화

- 변경:
  - 보정탭 체커보드 preview의 `findChessboardCorners` 실행을 GUI 메인 스레드에서 전용 worker thread로 분리했다.
  - 보정 preview는 먼저 `detecting` overlay를 즉시 표시하고, 최신 요청 결과만 받아 최종 corner overlay를 갱신하도록 바꿨다.
  - 창 resize에서는 체커보드 재검출을 하지 않고 기존 overlay 이미지만 다시 스케일하도록 변경했다.
  - `docs/usage.md`에 비동기 보정 preview 동작을 반영했다.
- 이유:
  - RTSP 수신 중 보정탭 진입 또는 resize 때 OpenCV 체커보드 탐색이 Qt 이벤트 루프를 막아 `main.py is not responding`처럼 보이는 문제를 줄이기 위해서다.
- 확인:
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_stereo.py::test_checkerboard_preview_refresh_queues_detection_worker tests/test_stereo.py::test_checkerboard_zoom_controls_are_above_calibration_and_collapsible -q`로 보정탭 관련 위젯 테스트를 확인했다.
  - `python3 -m compileall main.py wiris_stereo_distance`로 코드 문법 상태를 확인했다.
- 남은 위험:
  - 실제 카메라 수신 중 보정탭 진입 응답성은 현장 장비에서 추가 확인이 필요하다.
  - live overlay는 빠른 preview 경로이므로, 저장/보정 실행의 full-resolution 검출 결과와 일시적으로 다르게 보일 수 있다.

## v1.5.0 — 2026-06-08 — requirements 파일 구조 정리

- 변경:
  - `requirements.txt`를 `requirements/runtime.txt`로 이동했다.
  - `requirements-calibration.txt`를 `requirements/calibration.txt`로 이동했다.
  - `scripts/bootstrap_calibration_deps.sh`가 새 `requirements/calibration.txt` 경로를 사용하도록 수정했다.
  - `docs/usage.md`, `docs/testing.md`, `docs/README.md`에 새 requirements 경로를 반영했다.
- 이유:
  - 루트 파일 수를 줄이고, 기본 실행 의존성과 보정 전용 의존성을 같은 디렉터리 아래에서 명확히 관리하기 위해서다.
- 확인:
  - `bash -n scripts/bootstrap_calibration_deps.sh run.sh`로 shell 문법을 확인했다.
  - `python3 -m compileall main.py wiris_stereo_distance`로 코드 문법 상태가 유지되는지 확인했다.
  - `find requirements -maxdepth 1 -type f -print`로 이동된 파일을 확인했다.
- 남은 위험:
  - 외부 문서나 사용자가 예전 `requirements.txt` 경로로 직접 설치하던 습관이 있으면 새 경로를 사용해야 한다.

## v1.4.0 — 2026-06-08 — 루트 Markdown 문서 docs 이동

- 변경:
  - 루트 `README.md`를 `docs/usage.md`로 이동했다.
  - 루트 `PROJECT.md`, `DESIGN.md`, `DESIGN-ibm.md`, `AGENTS.md`를 `docs/`로 이동했다.
  - `docs/README.md`, `docs/usage.md`, `docs/PROJECT.md`, `docs/AGENTS.md`의 문서 역할 설명을 이동 후 구조에 맞게 갱신했다.
- 이유:
  - 루트에는 작성 중심 변경 이력인 `UPDATE.md`만 남기고, 읽을 문서는 모두 `docs/`에서 관리하기 위해서다.
- 확인:
  - `find . -maxdepth 1 -name '*.md' -print`와 `find docs -maxdepth 1 -name '*.md' -print`로 루트에는 `UPDATE.md`만 남고 관리 문서가 `docs/` 아래에 있는지 확인했다.
  - `python3 -m compileall main.py wiris_stereo_distance`로 코드 문법 상태가 유지되는지 확인했다.
- 남은 위험:
  - `AGENTS.md`가 루트에서 `docs/`로 이동했으므로, 도구가 루트 `AGENTS.md`만 자동 인식하는 환경에서는 해당 지침을 자동 적용하지 않을 수 있다.
  - 일부 외부 도구는 루트 `README.md`를 기본 표시 문서로 기대할 수 있다.

## v1.3.0 — 2026-06-08 — docs 세부 문서 구조 추가

- 변경:
  - `docs/README.md`를 추가해 세부 문서 목차를 만들었다.
  - `docs/architecture.md`, `docs/field-workflow.md`, `docs/testing.md`, `docs/refactor-roadmap.md`를 추가했다.
  - `README.md`, `PROJECT.md`, `AGENTS.md`에 `docs/` 역할과 세부 문서 링크를 반영했다.
  - `PROJECT.md`의 중복된 현재 주의점 문장을 정리했다.
- 이유:
  - 루트 문서는 빠른 안내와 기준만 담당하고, 긴 아키텍처/운용/테스트/리팩터링 내용은 `docs/`에서 관리하기 위해서다.
- 확인:
  - `find docs -maxdepth 1 -type f -name '*.md' -print`로 문서 생성 상태를 확인했다.
  - `python3 -m compileall main.py wiris_stereo_distance`로 코드 문법 상태가 유지되는지 확인했다.
- 남은 위험:
  - `README.md`의 기존 상세 운용 매뉴얼은 아직 `docs/field-workflow.md`로 완전히 분리되지 않았다.
  - `docs/` 문서는 초기 뼈대이므로 실제 현장 절차 변화에 맞춰 계속 보강해야 한다.

## v1.2.0 — 2026-06-08 — IBM/Carbon 디자인 1차 적용

- 변경:
  - `DESIGN.md`를 추가해 프로젝트용 UI/스타일 기준을 한글로 정리했다.
  - `wiris_stereo_distance/theme.py`를 추가해 IBM/Carbon 계열 색상 토큰을 분리했다.
  - PyQt 전역 스타일, 버튼, 입력 필드, 상태칩, 거리/센서 상태 라벨을 밝은 Carbon 계열로 변경했다.
  - 거리 그래프, 삼각측량 품질 그래프, Pixhawk/RTK 그래프, 카메라 오버레이 색상을 새 테마 토큰에 맞췄다.
  - `README.md`, `PROJECT.md`, `AGENTS.md`에서 `DESIGN.md`와 `DESIGN-ibm.md` 역할을 정리했다.
- 이유:
  - 앱의 디자인 기준을 실제 코드와 연결하고, 어두운 slate 테마에서 IBM/Carbon 계열의 구조적 운용 대시보드로 이동하기 위해서다.
  - 스타일 색상 하드코딩을 줄이고 이후 디자인 수정을 한 곳에서 시작할 수 있게 하기 위해서다.
- 확인:
  - `python3 -m compileall main.py wiris_stereo_distance`로 Python 문법 컴파일을 확인했다.
  - 기존 어두운 테마 핵심 색상 문자열이 주요 UI 코드에 남지 않았는지 검색했다.
  - `QT_QPA_PLATFORM=offscreen python3 -m pytest tests/test_stereo.py::test_session_buttons_toggle_labels_and_roles tests/test_stereo.py::test_camera_network_settings_are_single_compact_row tests/test_stereo.py::test_measurement_settings_save_button_triggers_current_settings_save -q`로 대표 PyQt 위젯 테스트 3개 통과를 확인했다.
  - `python3 -m pytest tests/test_measurement_controller.py tests/test_triangulation.py tests/test_settings.py tests/test_recording_session.py -q`로 관련 로직 테스트 15개 통과를 확인했다.
  - 오프스크린 Qt 렌더링으로 `/tmp/stereo_design_check.png`를 생성해 밝은 IBM/Carbon 스타일 적용 상태를 확인했다.
- 남은 위험:
  - 실제 하드웨어 연결 화면과 전체 탭을 눈으로 확인한 것은 아니다.
  - 일부 세부 위젯의 여백과 행 높이는 화면 캡처 기반으로 추가 조정이 필요할 수 있다.
  - 전체 `tests/test_stereo.py`는 현재 로컬 `stereo_distance_settings.json`의 `rtsp_transport=tcp` 값 때문에 UDP 기본값을 기대하는 테스트에서 실패했고, 이후 오프스크린 Qt 이벤트 처리 중 세그폴트가 발생해 끝까지 완료하지 못했다.

## v1.1.0 — 2026-06-08 — README와 PROJECT 문서 정리

- 변경:
  - `README.md` 상단에 한글 빠른 실행 안내, 문서 역할, 기본 현장 순서를 추가했다.
  - `PROJECT.md`를 추가해 프로젝트 목적, 시스템 구성, 주요 워크플로, 설계 방향, 디자인 방향을 정리했다.
- 이유:
  - 처음 보는 사람은 `README.md`에서 바로 실행과 문서 위치를 확인하고, 프로젝트 맥락은 `PROJECT.md`에서 따로 읽을 수 있게 하기 위해서다.
  - 기존 상세 운용 매뉴얼 내용을 유지하면서 문서별 역할을 분리하기 위해서다.
- 확인:
  - `README.md`, `PROJECT.md`, `UPDATE.md` 내용을 읽어 생성 및 반영 상태를 확인했다.
- 남은 위험:
  - 별도 표준 `DESIGN.md` 파일은 아직 없고, 현재는 `DESIGN-ibm.md`가 디자인 기준 역할을 한다.
  - `README.md`의 기존 상세 운용 매뉴얼은 여전히 대부분 영어로 작성되어 있다.

## v1.0.0 — 2026-06-08 — 문서 체계 시작

- 변경:
  - `AGENTS.md`를 추가해 프로젝트 작업 지침과 업데이트 기록 규칙을 명시했다.
  - `UPDATE.md`를 추가해 변경 이력을 한글로 기록할 위치를 만들었다.
- 이유:
  - 사용법, 프로젝트 맥락, 디자인 기준, 변경 이력, 에이전트 작업 규칙을 문서별로 분리하기 위해서다.
  - 이후 작업에서 의미 있는 변경 사항을 같은 형식으로 남기기 위해서다.
- 확인:
  - `AGENTS.md`와 `UPDATE.md`를 프로젝트 루트에 생성했다.
- 남은 위험:
  - `PROJECT.md`와 `DESIGN.md`는 아직 별도 표준 문서로 정리되지 않았다.
  - 기존 PyQt 스타일 코드는 아직 IBM/Carbon 계열 디자인 기준으로 정리되지 않았다.
