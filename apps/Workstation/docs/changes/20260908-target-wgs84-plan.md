# Target WGS84 저장 수정 계획 및 사전 검토

작성일: 2026-09-08
상태: 아래는 승인 시점의 계획이며, 구현 결과는 문서 끝의 실행 결과를 따른다.

## 1. 합의된 요구와 범위

- 영상 삼각측량으로 계산한 target의 위치를 저장 데이터에 추가한다.
- 필드: `target_latitude_deg`, `target_longitude_deg`,
  `target_altitude_ellipsoid_m`, `target_altitude_msl_m`.
- latitude/longitude 단위는 degree, altitude 단위는 metre다.
- 사용자의 가정: Pixhawk 기준점과 카메라 Optical Center는 일치한다.
- 기존 Forward/Right/Down/Range와 기존 측정 방식의 계산·판정은 유지한다.
- 새로운 화면이나 장치 명령은 이 계획의 범위에 포함하지 않는다.
- 사용자용 결과 열에는 위 네 필드를 추가한다. 별도 source/status 열을
  반복해서 추가하지 않고, 변환의 입력·가정·실패 사유는 기존 증거 구조에 둔다.

추가로 명시해야 할 가정: Pixhawk 기준점과 Optical Center가 같다는 말은
GNSS 안테나까지 같은 점이라는 의미는 아니다. 현재 앱은 GNSS 위치를
카메라 위치의 대용값으로 사용한다. 본 계획은 그 근사를 그대로 이용하며,
GNSS 안테나 offset까지 보정되었다고 주장하지 않는다. 위치 일치만으로
장착 회전도 같아지지는 않으므로 기존 camera/body 회전 계약을 사용한다.

## 2. 좌표 계약

| 값 | 원점 / 축 | 단위 |
| --- | --- | --- |
| `source_point_left_rdf_m` | 왼쪽 광학 중심 / Right, Down, Forward | m |
| 기존 `point_m` | baseline 중앙 / Forward, Right, Down | m |
| camera/body 변환 | Optical RDF → PX4 body FRD | 회전 |
| absolute camera rotation | 왼쪽 Optical RDF → world NED | 회전 |
| local world target | 명시한 지리 기준점 / North, East, Down | m |
| ECEF | 지구 중심 / ECEF X,Y,Z | m |
| target 지리좌표 | WGS84 latitude, longitude, ellipsoid height | deg, deg, m |
| target MSL | 명시된 해수면 고도 기준 | m |

가장 단순한 계산 입력은 기존 결과에 보존된 `source_point_left_rdf_m`이다.
baseline-center 값을 body FRD라고 간주하여 quaternion을 곱하지 않는다.

```text
p_target_ned = C_left_ned + R_ned_from_left_optical * p_target_left_rdf
p_target_ecef = origin_ecef + R_ecef_from_ned * p_target_ned
target latitude/longitude/ellipsoid height = ECEF 역변환
```

Pixhawk live 방식은 P1 GNSS를 지리 원점으로 둘 경우 `C_left_ned = 0`이다.
방법별 실제 geometry quaternion을 사용한다. 특히 baseline-yaw 방식에서
선택한 yaw를 raw Pixhawk yaw로 바꾸어 월드 위치를 계산하면 안 된다.

상대 R,t만으로는 절대 월드 방향이 정해지지 않는다. Mountain/Cloud 및
프로파일 방식은 그 계산에 사용된 왼쪽 absolute pose와 지리 원점의 관계를
검출 pair 입력에 동결한다. 이 관계가 없는 프로파일은 기존 거리 결과를
유지하고 target 지리좌표만 null로 둔다. 임의로 최신 Pixhawk 자세를 붙이지 않는다.

## 3. 고도 결정

- WGS84/ECEF 계산에는 원점의 유효한 타원체고를 사용한다.
- MSL을 타원체고처럼 ECEF 변환에 넣지 않는다.
- 정확한 target MSL에는 target 위치의 지오이드 높이가 필요하다.
- 이번 기본 계획은 지오이드 모델이 없으면 `target_altitude_msl_m = null`이다.
  네 필드는 모두 존재하되, 계산 불가능한 값은 0으로 채우지 않는다.
- P1의 타원체고−MSL 차이를 target에도 적용하는 방식은 별도 근사 선택이다.
  이 근사를 채택한다면 기존 증거/설명에 일정 지오이드 가정을 명시해야 한다.
- 원점 타원체고가 없을 때 MSL을 대입해 lat/lon을 조용히 생성하지 않는다.
  MSL-only 근사 경로가 필요하면 별도 계약과 테스트로 다룬다.

## 4. 구현 순서와 수정 위치

| 순서 | 위치 | 수정 내용 |
| --- | --- | --- |
| 1 | 신규 `workstation_core/target_coordinates.py` | 유효성 검사, Optical RDF → NED → ECEF → 지리좌표의 순수 계산 함수. UI/live 상태 접근 금지 |
| 2 | `workstation_core/measurement_attempt.py` | 필요한 absolute world anchor를 optional 입력 snapshot으로 보존. 결과 lat/lon/alt는 이 입력 객체에 넣지 않음. 구형 payload hash 유지 |
| 3 | `measurement_view_controller.py`의 방법별 geometry/attempt 생성부 | 상대 pose를 R,t로 축약하기 전에 실제 absolute pose와 지리 원점 관계를 동결. pair 시점의 GNSS/attitude만 사용 |
| 4 | `workstation_core/method_evaluation.py`의 `evaluate_method_request()` / `_geometry_result_payload()` | geometry 계산 직후 동결 입력으로 target 계산. 결과 payload에 네 필드 보존. 변환 불가가 기존 거리 결과를 실패시키지 않도록 분리 |
| 5 | `workstation_core/method_result_evidence.py`의 `derive_method_result()` | 저장된 target 필드를 공통 derived 결과로 전달. renderer에서 재계산하지 않음 |
| 6 | `workstation_core/measurement_record.py` / `measurement_journal.py` | `recorded_geometry` 및 outcome journal에 target 보존. 읽기·재생성 시 현재 상태로 덮어쓰지 않음 |
| 7 | `measurement_view_controller.py`의 `_method_history_payload()`, `_method_result_row()`, `_bundle_flat_fields()` | 대표 결과 전달, 방법별 결과 행, 방법별 평탄화에 네 필드 연결 |
| 8 | `recording_controller.py`의 `_commit_distance_history()`, `_compact_primary_retry_row()`, `_measurement_result_evidence()` | 일반 저장·캐시 유실 재시도·canonical 결과 모두 동일한 target 값 유지. preview 경로도 공통 계산 경로 이용 |
| 9 | `workstation_core/recording_session.py` | 결과 열, compact projection, review projection에 네 필드 추가 |
| 10 | `workstation_core/session_parquet.py` | measurement 결과의 nullable float64 컬럼 추가. geometry comparison은 해당 결과를 실제 수록하는 경로에만 추가 |
| 11 | `workstation_core/measurement_workbook.py` | 방법별 [A], 90_Flat, 방법 비교, 91_Exact에 필드 연결. degree는 소수점 8~9자리 표시, 내부 float 보존 |
| 12 | `workstation_core/measurement_reproduce.py` / `measurement_xlsx_read.py` | workbook에 동결한 원점·회전·높이 근거로 별도 target 재현 지원. 기존 FRD 재현 API 호환 유지 |
| 13 | `README.md`, `docs/RECORDING_SCHEMA.md` | 필드 의미, 높이 기준, null 조건, 위치·장착 가정, 구형 파일 호환 명시 |

`measurement_csv_mirror.py`는 고정 envelope 안에 payload를 저장하므로
네 필드를 새로운 최상위 CSV 열로 무조건 추가하지 않는다. 해당 payload가
JSONL과 동일하게 보존되는지 검증한다. 일반 compact CSV는 별도로 열을 추가한다.

신규 모듈에서 기존 `gnss_profile.py`의 WGS84/ECEF 함수를 재사용할지는
의존성 방향을 확인해 결정한다. 지형 전용 모듈 의존성을 불필요하게 추가하지 않는다.

## 5. 사전 검토 결과

| 발견 사항 | 이전 제안의 위험 | 계획 반영 |
| --- | --- | --- |
| baseline-center Down은 world Down이 아님 | FRD에 body quaternion을 직접 적용하면 축 오류 | left optical RDF 원본 점을 이용 |
| 상대 R,t와 absolute pose가 다른 정보 | 모든 방법을 raw Pixhawk 자세로 변환하면 잘못된 위치 | 방법별 world anchor 동결 |
| 현재 attempt는 일부 상대 pose의 world anchor를 보존하지 않음 | 결과 필드만 추가하면 재현 불가능 | optional 입력 snapshot 추가 허용 |
| Pixhawk/Optical Center 일치는 GNSS 안테나 일치와 다름 | 위치 가정을 과도하게 확장 | 기존 GNSS 원점 근사 명시 |
| 두 고도 기준이 별개 | 타원체고를 MSL로 오표기 | 지오이드 근거 없으면 MSL null |
| `recorded_geometry`가 journal 재생성의 근거 | exporter마다 계산하면 결과가 달라짐 | 최초 결과 고정 후 전파 |
| 캐시 유실용 별도 primary 저장 경로 존재 | 정상 저장만 필드를 갖게 됨 | retry 경로에도 동일 필드 |
| canonical payload/hash를 엄격히 검사 | 새 기본 필드가 구형 hash 변경 | optional 기본값 생략 및 버전별 읽기 검증 |

검토 판정: 조건부 구현 가능. 네 필드를 모든 결과에 무조건 채우는 계획은
부적절하며, world anchor와 높이 근거가 있는 결과만 계산하도록 수정했다.
MSL을 항상 숫자로 채우는 요구는 아직 합의되지 않았으므로 기본값은 null이다.

## 6. 검증 slice와 완료 조건

각 slice는 직전 의존 slice의 테스트가 PASS한 뒤 진행한다.
아래 상태는 실행 결과가 아니라 계획 상태다.

| ID | 근거 / 입력 → 출력 | 검증 및 독립 기준 | 상태 |
| --- | --- | --- | --- |
| S1 계약 | 사용자 필드명·위치 가정 + 기존 좌표 코드 | 축 순서/단위/원점/높이/방법별 anchor의 코드·문서 대조 | PASS (사전 검토) |
| S2 수학 | left RDF 점 + absolute pose + 지리원점 → 네 target 필드 | ECEF 축의 손계산 fixture와 독립 변환 라이브러리 비교. 0/90도 회전, 양·음 Down, 날짜변경선·극 부근·비유한 입력. 목표 오차 ≤1 mm ECEF 및 높이, 각도 ≤1e-8 deg | NOT_RUN |
| S3 입력 고정 | pair 및 방법 pose → world anchor snapshot | live/Pixhawk-yaw90/relative profile 비교. 실행 중 latest 자세 변경에도 동일 결과. 원점/회전 누락 시 null. 구형 hash 유지 | NOT_RUN |
| S4 tilt 불변 | 같은 world target을 서로 다른 camera pitch로 관측 → 같은 WGS84 | 독립적으로 만든 월드 점으로 각 카메라의 RDF 입력을 구성. 같은 pixel을 고정하는 잘못된 불변 시험은 하지 않음 | NOT_RUN |
| S5 결과 저장 | outcome → primary/방법별 JSONL·CSV·Parquet | 정상·중복·부분 실패·캐시 유실 retry에서 값과 pair/method identity 보존, 기존 FRD 그대로 | NOT_RUN |
| S6 재생성 | 저장 journal → workbook → 읽기/재현 | 재생성 전후 target 수치 일치. 새 필드 없는 구형 파일은 빈 셀. 91_Exact 및 동결 변환 근거 확인 | NOT_RUN |
| S7 통합 | 실제 형식 fixture → worker → 모든 저장 artifact | 재계산 원점/시간 불일치와 다른 방식의 quaternion 혼용 방지. 기존 focused regression + 프로젝트 필수 검사 | NOT_RUN |

수치 테스트의 독립 기준은 구현 단계에서 공식 좌표 정의와 별도 구현으로
확인한다. 같은 함수의 왕복만으로 정확하다고 판정하지 않는다.

실행 대상: 신규 target 좌표 단위 테스트, 기존 `test_method_evaluation.py`,
`test_measurement_methods.py`, `test_measurement_journal.py`,
`test_measurement_workbook.py`, recording/retry 및 Parquet 관련 테스트.
그 뒤 프로젝트 지침의 회귀·문서·경로 검사를 진행한다.

## 7. 실행 범위와 현재 상태

이번 작업은 계획 문서만 작성한다. 앱 코드, 기존 측정 파일, Pixhawk 설정을
수정하지 않는다. 작업 트리에는 이미 많은 사용자 변경이 있으므로 구현 전에
겹치는 파일의 최신 diff를 다시 확인하고 기존 변경을 보존해야 한다.

Coordinate contract checked: Optical RDF / body FRD / baseline-center FRD /
world NED / ECEF / WGS84 / MSL; 코드 경로 사전 검토, 구현 테스트 NOT_RUN.

## 8. 구현 및 검증 결과

- `target_coordinates.py`에 계산을 구현하고 worker, 대표 결과/재시도,
  방법별 결과, JSONL/CSV, compact projection, Parquet, XLSX에 연결했다.
- 계획 이후 최신 코드에 이미 있는 `measurement_pose_evidence_json`을 활용했다.
  별도 MeasurementAttempt 필드는 추가하지 않았다. 실제 candidate pose를
  동결하고 P1 GNSS를 왼쪽 optical origin proxy로 명시하여 사용한다.
- 순수 relative 프로파일은 world anchor가 없으면 좌표를 생성하지 않는다.
- 지오이드 모델은 도입하지 않았으므로 target MSL은 null로 보존된다.
- 기존 FRD/거리 계산 및 기존 canonical 입력의 기본 형태는 유지한다.
- `reproduce_target_coordinates_exact()`는 저장된 target 점을 재사용하지 않고
  exact K,D,R,T,u,v로 삼각측량한 뒤 world anchor로 지리좌표를 재현한다.
- 검증 스킬의 slice 절차에 따라 축/부호, pitch 불변, geometry yaw 우선순위,
  상대 pose의 anchor 누락, 결측 고도, 날짜변경선·극, journal 재생성,
  primary retry, JSONL/CSV/Parquet/XLSX 보존을 확인했다.

Workstation 디렉터리에서 실행:

```sh
PYTHONPATH=..:../../common/python ../../.venv/bin/python -m pytest -q
# 2077 passed, 2 skipped (213.62s)
PYTHONPATH=..:../../common/python ../../.venv/bin/python -m pytest -q tests/test_target_coordinates.py tests/test_target_coordinate_storage.py
# 15 passed (1.03s); 전체 테스트 실행 후 추가한 두 검증 포함
```

`git diff --check` PASS. 저장소 루트의 `tools/check_portable_paths.py`는
기존 Pixhawk 문서/endpoint와 과거 변경 기록의 절대 경로 26건으로 FAIL.
이번 기능의 변경 파일에서는 위반이 없음을 별도 검사한다.
실제 장치 좌표 정확도 실험이나 실행 중 앱 재시작은 수행하지 않았다.

Coordinate contract checked: left optical RDF → method world NED → ECEF →
WGS84; 기존 baseline-center FRD 유지, MSL null; 단위·부호·tilt·저장·재현 테스트 PASS.

## 9. 후속 요청: 측정 중 좌표 표시

사용자의 측정 중 조회 요청으로 표시 범위를 확장했다. `Measurement > Result`
바로 아래 `Target 위치 · WGS84`에 네 필드를 배치했다. 기존 EngineeringValue를
재사용하고 각 값을 클릭하면 Inspector에서 단위·좌표 기준을 확인할 수 있다.

저장 공통 함수 `target_coordinates_from_attempt()`의 결과를 history 생성 시
한 번 계산하여 DistanceHistoryPoint와 저장 행에 함께 전달한다. 이 공통 함수의
기존 level-coordinate 출력은 보존한다. 화면은 최신 history 항목의 geographic
필드만 표시하며 최신 GNSS/attitude를 따로 조회하거나 재계산하지 않는다.
마지막 결과의 경과시간이 표시되고 새 결과가 결측이면 이전 좌표를 지우며,
history가 비면 전체 값을 비운다. 위경도 9자리, 고도 3자리로 표시한다.

검증: UI·recording·기존 화면 관련 99개 PASS, method/storage/workbook 및 새 UI
관련 120개 PASS(두 묶음은 일부 중복). 신규 `test_target_coordinates_ui.py`는
값 갱신, 결측·구형 값, 오래된 결과의 age, Inspector click, recording-owner의
동일 frozen 좌표 전달을 확인한다. 실행 명령은 Workstation 기준:

```sh
PYTHONPATH=..:../../common/python ../../.venv/bin/python -m pytest -q tests/test_target_coordinates_ui.py tests/test_target_coordinate_storage.py tests/test_workstation_ui.py tests/test_measurement_package_ui.py tests/test_recording_raw_artifacts.py tests/test_workstation_contracts.py
PYTHONPATH=..:../../common/python ../../.venv/bin/python -m pytest -q tests/test_method_evaluation.py tests/test_measurement_data_store.py tests/test_measurement_workbook.py tests/test_target_coordinates_ui.py tests/test_target_coordinate_storage.py
```

700×180 offscreen 렌더링에서 -89/-179도 9자리와 음수 고도의 잘림 없음을
확인했다. 이는 Qt 테스트용 표시 검증이며, 실행 중 사용자 앱을 조작하거나
재시작한 것은 아니다. `README.md`와 `docs/ui/03_WORKSPACE_LAYOUT.md` 갱신.
Coordinate contract checked: WGS84 deg / ellipsoid·MSL m, 기존 FRD 표시와 분리.
