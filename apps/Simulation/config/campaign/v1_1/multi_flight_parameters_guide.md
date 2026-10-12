# 다중 비행 파라미터 입력 안내

현재 실행 기본본은 `multi_flight_parameters_v1_1_numeric_v3.xlsx`입니다. 실제 비행 결과가 아닌 실행 조건표입니다. 기존 `numeric_v2.xlsx`는 보존하며, 실행기는 v2와 v3를 모두 읽습니다. 입력 값에 수식은 없습니다.

## 공통값과 run별 값

`Runs` 탭의 행이 비행 수를 정합니다. `Run sequence number`는 1–1000 사이에서 중복 없이 지정하고, 숫자가 작은 run부터 실행합니다. 중간 run을 지워도 나머지 번호를 다시 매길 필요가 없습니다. 새 run은 사용하지 않은 번호로 추가합니다. `Defaults` 탭에는 각 숫자 파라미터의 공통 기본값이 한 번만 있습니다. 나머지 항목별 탭에는 기본값과 다른 run의 값만 적습니다. 그 탭에서 같은 `Run sequence number`가 run을 가리킵니다. 값이 빈 칸이면 `Defaults` 값을 사용하고, 숫자 `0`은 빈 칸이 아니라 명시한 값입니다.

공통 파라미터를 run별로 바꾸려면 `Defaults` 값은 그대로 두고 해당 항목 탭에 sequence 행을 추가해 바꿀 값만 씁니다. 다시 모두 공통으로 만들려면 `Defaults` 값을 바꾸고 그 항목의 run별 값을 지웁니다. 파라미터를 다른 탭으로 옮기지 않습니다. 새 run을 추가할 때는 `Runs`에 새 sequence 행만 추가하고, 예외 조건이 있을 때만 해당 항목 탭에 override 행을 추가합니다. run마다 별도 worksheet를 만들 필요가 없습니다. 숫자 중간에 새 run을 끼워 넣으려면 비어 있는 sequence를 사용하고, 빈 번호가 없으면 실행 순서에 맞게 해당 sequence와 override 참조를 함께 조정합니다.

현재 계획에서 `Path`에는 직선/활형/S자 및 600m 시작 경로의 예외값이 있고, `RenderedCameraPose`에는 G1에 해당하는 좌우 heading의 예외값이 있습니다. 나머지 비행·카메라·기록 조건은 `Defaults`에서 공유됩니다. `Runs`의 반복 번호와 보정/최종평가 플래그는 run마다 직접 적습니다.

탭은 `Runs`, `Path`, `Motion`, `FrameGate`, `Entry`, `CameraGeometry`, `RenderedCameraPose`, `CameraModel`, `CameraPX4Mount`, `MagnetometerLeft`, `GyroscopeLeft`, `GPS`, `Vehicle`, `Capture`, `CaptureTiming`, `ImageObservation`, `MagnetometerRight`, `GyroscopeRight`, `Defaults`입니다. Defaults와 항목별 override를 사람이 편집하고, 실행기는 이 값들을 합친 최종 run 조건을 각 결과에 기록합니다.

## 경로와 시간

세계 좌표는 착륙장 기준 ENU(동쪽, 북쪽, 위쪽)입니다. `s`는 0에서 1까지의 경로 매개변수이며 시간 그 자체가 아닙니다.

```text
N(s) = N_start + (N_end - N_start) * s
E(s) = A_bow * sin(pi*s)^2 + A_S * sin(2*pi*s)^3
U(s) = N(s) * tan(alpha)
```

현재 run에서는 bow와 S 진폭을 동시에 0이 아닌 값으로 쓰지 않습니다. 따라서 두 진폭이 모두 0이면 직선, bow 진폭만 있으면 활형, S 진폭만 있으면 단일 S자 경로입니다. `alpha=20°`이고 N과 U는 모두 착륙장 원점 기준 좌표입니다. 높이는 북쪽 거리와 함께 계산되므로 시작점이 패드 높이 0m라는 뜻은 아닙니다.

순항 속도 5m/s는 3차원 경로 호길이 기준입니다. 시작·종료 속도 램프는 각각 5초 반코사인이고, 중간 정지는 0초입니다. 시간별 `s(t)`는 누적 3D 호길이에 속도 프로필을 적용해 역산하므로 `s`가 시간에 따라 일정하게 증가하지 않습니다. 기준 경로 시간은 `T=L/v+Tr`이며 `L`은 3D 경로 길이, `v`는 순항 속도, `Tr`은 램프 시간입니다. 월드 ENU 경로 참조는 50Hz, PX4 위치·속도·yaw setpoint는 진입·과학 구간·종료 유지 구간에서 20Hz입니다.

경로 시작점 진입은 순간이동하지 않습니다. PX4 위치 setpoint를 현재 기체 위치에서 `s=0` 방향으로 최대 5m씩 전진시키며 실제 경로는 PX4 동역학을 따릅니다. 1m 시작 창에 들어간 뒤 readiness 제한시간 120초가 적용됩니다. 위치 오차 0.5m 이하, 속도 0.3m/s 이하, heading 오차 3° 이하를 5초 연속 유지하고 표본 간격은 0.2초 이하여야 과학 구간을 시작합니다. 과학 시간 0과 과학 캡처는 이 gate를 통과한 뒤 시작합니다. 과학 구간에는 두 속도 램프가 포함되고, 종료점에서 5초 유지한 뒤 안전한 SITL 착륙에 최대 90초를 허용합니다.

## 좌표계, 카메라와 센서

차량 heading은 월드 북쪽에서 동쪽으로 시계 방향으로 잰 각도입니다. 180°는 남쪽입니다. 이 값을 PX4 NED yaw로 바꾸는 식은 `atan2(sin(psi*pi/180), cos(psi*pi/180))`입니다. Gazebo spawn yaw −90°는 시뮬레이터의 초기 자세값이며 비행 중 명령 heading과 별개입니다. 실행 전 실제 spawn 방향을 월드 ENU와 대조합니다.

PX4 local NED 원점은 spawn 설정에서 추정하지 않습니다. 정지 상태의 동기 관측 20개로 월드 ENU 원점을 계산하고, 같은 시뮬레이션 epoch 및 안정된 추정기 reset 카운터를 요구합니다. ENU/NED 축이 맞는지 확인하며 어긋나면 run을 막습니다. 정렬 확인 후 위치는 `[N−N0, E−E0, U0−U]`, 속도는 `[Ndot, Edot, −Udot]`로 변환하고, 실제 원점과 확인 결과를 run 기록에 저장합니다.

스테레오 광학 중심은 기체에 붙지 않고 세계 ENU에 고정됩니다. 좌측은 `(-12.5, 0, 0.1)m`, 우측은 `(12.5, 0, 0.1)m`이며 기준선은 25m입니다. 광학 좌표는 RDF(+x 오른쪽, +y 아래, +z 전방)입니다. 두 카메라의 물리 렌더 자세는 roll 0°, pitch/elevation +20°이고 기본 월드 heading은 0°입니다. 31–39번 run은 좌측 +0.5°, 우측 −0.5°입니다. `R_WC=Rz_world(-psi)*R_WC_base`이며 `R_WC_base=R_ENU_RDF0*Rx(alpha)*Rz(phi)`, `R_ENU_RDF0`의 열은 `[+East, -Up, +North]`입니다. +20° pitch는 광학 전방축을 위로 올립니다.

`CameraPX4Mount`는 이 Gazebo 광학 자세와 별도의 PX4 카메라 센서 body-FRD 장착각입니다. 양쪽 모두 roll 0°, pitch 8°, yaw 0°입니다. PX4 body attitude를 렌더된 광학 자세로 대신 읽지 않습니다. GPS의 `SIM_GPS_USED=10`은 위성 10개 사용 설정이고 on/off 값이 아닙니다. 기준선 삼각측량과 좌우 GPS 입력은 켜져 있으며, 최대 입력 나이는 0.2초입니다.

## 기록 조건과 범위

캡처 탭의 GT vehicle-state logging rate(최소 50Hz, 선호 100Hz)는 50Hz 명령 경로 참조와 별도입니다. 카메라/YOLO 요청 rate는 각각 20Hz이며 실제 속도는 따로 측정합니다. 노출 시각은 Gazebo simulation clock을 쓰고, 각 이미지에 simulation epoch, 수신 시각, YOLO 완료 시각, 원본 frame ID를 따로 기록합니다. GT 위치는 선형 보간, quaternion은 최단경로 SLERP를 쓰고 외삽하지 않습니다. 보간 구간 한도는 0.04초, stereo 노출 차이는 0.001초입니다. 각 pair는 서로 다른 원본 좌우 프레임을 쓰며 frame을 재사용하지 않습니다. 비매칭·미검출 프레임도 보존하고, bbox가 없으면 null, bbox 좌표는 letterbox 역변환 후 원본 이미지 좌표로 기록합니다. 원본 이미지는 무손실로 저장하며 오버레이만으로 대체하지 않습니다.

현재 workbook 숫자와 환경값의 관계는 다음과 같습니다.

- `Simulation.env`의 기본 optical pitch 8°는 `G0.env`/`G1.env` 프로필에서 20°로 덮어씌워집니다. 표에는 물리 렌더에 적용되는 20°를 적었고, PX4 body-FRD mount는 별도로 8°입니다.
- `CameraModel` 원거리 clipping은 700m입니다. 600m에서 시작하는 직선 경로의 optical-axis 깊이는 약 638m여서 기본 `Simulation.env` 값 500m로는 시작 구간이 렌더되지 않습니다. 실행기는 각 run의 700m를 임시 시나리오 설정에 적용합니다.
- `Vehicle`의 spawn yaw 검증은 SITL 시작 직후 Gazebo 월드 ENU 자세 쿼터니언에서 계산한 yaw와 표의 시뮬레이터 spawn yaw를 비교합니다. 오차가 2°보다 크면 시작 gate 전에 run을 막고, 비교값을 `coordinate_frame_check.json`에 저장합니다.
- 카메라 baseline, 높이, 영상 크기, FOV, clipping, 자력계/자이로 noise·bias, GPS baseline 설정과 satellite count, 기체 spawn, YOLO 요청 rate·크기·confidence는 run별 임시 시나리오 설정이나 오프라인 처리 인수로 적용됩니다. NMS IoU는 표의 숫자를 적용합니다.
- 구형 `SWEEP_*`, `AUTO_*`, `WAYPOINT_*`는 이번 연속 경로 계획의 입력이 아니어서 넣지 않았습니다. 모델 파일명, 토픽, 장치, 출력 경로 등 문자열·실행 설정은 `Simulation.env`나 모델 manifest에 남습니다. 오프라인 RTK/IMU 합성오차와 보정 알고리즘 설정도 비행 입력이 아니므로 실험 규격/분석 쪽에 둡니다.
- `EnvironmentContract`에는 현재 Gazebo world 이름, source/install SDF 경로와 SHA-256, 조명·배경 고정 방식, 날씨와 바람 선언을 둡니다. preflight가 source와 설치본 해시를 비교하고, world 이름·대기 종류·wind 설정을 확인한 뒤 실제 SDF 조명 정보를 기록합니다. 현재 SDF에는 adiabatic atmosphere가 있고 명시적인 wind element/plugin은 없습니다. 기존 39회 실행의 환경 상태를 소급 증명하는 자료로 사용하지 않습니다.
- 비교 후에도 `OVERLAY_PUBLISH_HZ`, `CAM_L/R_PX4_INSTANCE`, `MEASUREMENT_QUEUE_CAPACITY`는 run 표에 추가하지 않았습니다. overlay 주기는 선택 표시 기능이고, PX4 instance 번호는 모델명·namespace·topic과 함께 관리해야 하는 연결 설정이며, queue capacity는 현재 캠페인 실행 경로에서 직접 읽는 항목이 아닙니다. `STEREO_V1_1_MC_TRIALS`는 비행 횟수가 아니라 오프라인 분석 반복 수입니다. XLSX의 `OfflinePlan.trial_count`는 인수인계 계획값이고, `xlsx_*`는 오프라인 분석을 실행하지 않습니다. 고정 protocol의 `resume`, `all_results`, `reprocess`가 실제 분석을 시작할 때만 `STEREO_V1_1_MC_TRIALS`를 읽습니다.
- `RunManifest`가 run ID, 초기 attempt ID, 순서와 요구 seed를 소유합니다. `Campaign.seed_policy=manifest_required`이면 각 run의 `seed_requested`를 적용하고, 기술 재시도는 같은 seed로 새 attempt ID를 사용합니다.

RQT에서 `STEREO_V1_1_ACTION=xlsx_plan`은 실제 비행 없이 표와 환경 계약을 검증하고 순서를 보여줍니다. `xlsx_all`은 preflight 후 XLSX의 모든 행을 순서대로 비행하고, 각 행의 최종 파라미터를 `workbook_parameters.json`에, 각 값이 `campaign_fixed`인지 `run_variable`인지 `workbook_parameter_sources.json`에 저장합니다. `run_manifest.json`에는 입력 식별과 workbook hash가 들어갑니다. 실행 결과 상태는 `run_status.json`, `runner_result.json`, `qc_report.json`에서 확인합니다. `xlsx_resume`은 출력 폴더에 고정된 워크북을 읽어 미완료 행부터 이어갑니다. 각 run 결과는 기존 `recordings/stereo_v1_1/<output-id>/원본_실행별/` 구조에 저장됩니다. 지원되지 않는 값 조합은 비행 전에 오류로 막습니다. 최종 논문 분석기는 아직 고정된 protocol manifest의 ID·역할 분할을 요구하므로, 임의로 구성한 XLSX campaign을 자동 분석하지는 않습니다.
