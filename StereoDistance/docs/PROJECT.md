# StereoDistance 프로젝트

## 목적

StereoDistance는 좌/우 WIRIS RTSP 영상, YOLO 검출, Pixhawk 자세/RTK 데이터를 결합해 현장에서 객체 거리와 3D 위치를 측정하기 위한 PyQt5 데스크톱 운용 도구다.

이 프로젝트는 Pixhawk 자체 보정을 QGC에서 먼저 완료했다는 전제를 둔다. 앱은 그 이후 현장 연결, WIRIS 시간 확인, WIRIS 내부 파라미터 보정, RTK/RM3100/자세 정렬, YOLO 측정, 세션 저장과 분석을 한 흐름으로 묶는다.

## 프로젝트 목적 상세

이 프로젝트의 목적은 단순히 두 카메라 영상에서 거리를 계산하는 것이 아니다. 현장에서 장비 연결, 시간 동기, 카메라 보정, Pixhawk 자세/RTK 상태, YOLO 검출 품질이 모두 얽혀 있을 때 운용자가 어떤 조건에서 측정값을 신뢰할 수 있는지 판단하게 만드는 것이다.

따라서 StereoDistance는 측정 결과만 보여주는 앱이 아니라, 측정 전제 조건을 확인하고 부족한 조건을 드러내며, 가능한 복구 경로를 제공하는 현장 검증 도구를 목표로 한다.

핵심 판단 기준은 다음이다.

- 좌/우 영상이 실제로 서로 다른 stereo source인지 확인한다.
- 카메라 내부 파라미터와 baseline이 현재 측정 조건과 맞는지 확인한다.
- Pixhawk 자세, RTK/GPS, RM3100/MAG 데이터가 실제로 수신되고 충분히 최신인지 확인한다.
- YOLO 검출 쌍이 시간, 클래스, geometry 기준을 만족하는지 확인한다.
- 기록된 거리 샘플이 어떤 입력과 상태에서 만들어졌는지 세션 단위로 추적 가능하게 남긴다.

## 프로젝트 방법론

이 프로젝트는 `측정값 생성`보다 `측정값을 신뢰할 조건을 만들고 검증하는 과정`을 우선한다.

1. 입력 검증 우선
   - RTSP 포트 open, ROS2 topic 이름, MicroXRCEAgent 실행 여부 같은 간접 신호만으로 성공을 판단하지 않는다.
   - 첫 프레임, publisher/sample, MAVLink heartbeat, 실제 MAG/attitude/GPS 수신처럼 계산에 쓰이는 신호를 직접 확인한다.

2. 단계별 운용
   - WIRIS 연결, 시간 확인, 내부 보정, 파라미터 확인, RTK/MAG/자세 정렬, YOLO 설정, 측정, 저장을 분리된 단계로 둔다.
   - 각 단계는 다음 단계의 전제 조건을 확인하는 역할을 가진다.

3. 출처가 명확한 계산
   - 내부 파라미터, baseline, Pixhawk 자세, GPS/RTK, YOLO bbox의 출처와 age를 가능한 한 표시하고 저장한다.
   - ROS2와 MAVLink처럼 같은 종류의 값을 줄 수 있는 경로도 source를 구분해 기록한다.

4. 3D ray triangulation 중심
   - 기본 측정 방법은 카메라 내부 파라미터와 Pixhawk/RTK 외부 파라미터를 사용하는 3D ray triangulation이다.
   - 단순 disparity 값은 진단 지표로만 취급하고, 필요한 입력이 없을 때 임의의 거리값을 기록하지 않는다.

5. 복구 가능한 현장 운용
   - ROS2/uXRCE가 막히면 MAVLink로 Pixhawk 쪽 설정을 전달해 ROS2 복구를 시도한다.
   - 복구 전까지는 MAVLink telemetry fallback으로 가능한 자세/GPS/MAG raw 값을 유지하되, 대체할 수 없는 ROS2 전용 정보는 제한 상태로 표시한다.

6. 검증 가능한 저장
   - 측정 CSV, 외부 파라미터, MAG 상태, 영상 파일, 설정을 같은 세션 단위로 묶는다.
   - 사후 분석에서 측정값 자체뿐 아니라 당시 입력 조건과 품질 상태를 함께 확인할 수 있게 한다.

7. Ground truth 우선순위
   - 최종 실험은 가로폭 15 m급 UAM을 안정적으로 검출하고 거리를 재는 것이며, 드론 평가와 UAM 적용이 같은 절차로 가능해야 한다.
   - 15 m급이라는 정보는 최종 표적 규모 설명일 뿐, bbox 폭 기반 sanity check나 거리 정확도 gate로 사용하지 않는다. 본 실험 전 드론 테스트에서는 표적 폭이 다르고, 실제 UAM에서도 YOLO bbox가 전체 가로폭을 항상 포함한다고 보장할 수 없기 때문이다.
   - 본 실험 카메라 조건은 WIRIS visible 8배줌, 즉 앱의 명목 `zoom index 8 / 8.0x`를 기준으로 잡는다. 다만 최종 분석에는 명목 `fx` 추정값이 아니라 해당 줌에서 체커보드로 만든 `intrinsics_zoom_08.json`을 적용해야 한다.
   - 기준 위치는 GNSS/Pixhawk를 기체 중심에 배치한 body 원점 기준으로 잡고, 비행 후 PX4 ULog를 우선 사용해 기록된 YOLO 측정 row와 시간축을 맞춰 비교한다. 따라서 최종 비교를 하려면 `측정` 탭에서 YOLO/depth 측정뿐 아니라 `세션 기록 시작`으로 거리 row를 반드시 남겨야 한다.
   - ULog가 없거나 아직 선택되지 않은 상태에서는 비행 중 수신한 UAM/드론 텔레메트리 버퍼를 fallback으로 사용하고, 마지막으로 거리 기록 row에 저장된 `drone_x/y/z`를 사용한다.
   - 비행 후 분석에서는 ULog Raw N/E/D 시계열을 먼저 확인하고, `time_offset_s` 자동 추정 후보를 적용한 뒤 matched count, timestamp delta, 오차 그래프를 검토한다.
   - 이 우선순위는 분석 source에 남겨야 하며, ULog와 실시간 fallback 또는 기록 row fallback을 섞어 최종 정확도 지표를 계산하지 않는다. ULog/live 샘플이 있는 분석에서 시간축 밖 row가 있으면 기록 row fallback으로 채우지 않고 unmatched row로 보고한다.

## 현재 범위

- QGC를 대체하지 않는다. Pixhawk 보정과 최종 파라미터 저장은 QGC 또는 PX4 shell 흐름을 전제로 한다.
- WIRIS 카메라 연결, RTSP 프레임 수신 확인, WIRIS SDK 명령, still image EXIF 시간 비교를 앱 안에서 다룬다.
- Pixhawk는 ROS2/uXRCE와 MAVLink 경로를 모두 지원하며, 현장에서는 실제 publisher/sample, heartbeat, MAG device_id 수신 여부를 분리해서 확인한다.
- ROS2 연결이 막히면 MAVLink를 복구 경로로 사용해 Pixhawk 쪽 MAVLink stream과 `uxrce_dds_client` 설정을 전송하고, ROS2 재시도 전까지 MAVLink telemetry fallback을 유지한다.
- UAM/드론 ground truth 텔레메트리는 `연결 > UAM`에서 별도로 연결한다. 정상 경로는 ROS2 `vehicle_local_position`이고, ROS2가 막히면 MAVLink `LOCAL_POSITION_NED` 또는 GPS/global-derived 위치 메시지를 실시간 fallback 버퍼로 유지한다.
- 측정은 내부 파라미터, Pixhawk 자세, RTK/baseline, 좌/우 YOLO 검출 매칭이 준비됐을 때 3D ray triangulation 결과를 기록하는 것을 목표로 한다.
- 하드웨어 연결 성공 여부는 자동 테스트만으로 증명할 수 없으므로, 진단 스크립트와 현장 smoke check를 함께 사용한다.

## Pixhawk 연결 의도

Pixhawk 연결의 우선 의도는 ROS2/uXRCE 경로를 살리는 것이다. ROS2는 PX4 uORB 데이터를 namespace와 topic 기준으로 구조화해서 받을 수 있으므로, 정상 연결 상태에서는 자세, GPS, MAG, custom field 감시의 주 경로로 둔다.

다만 현장에서는 ROS2 graph, MicroXRCEAgent, Pixhawk 쪽 `uxrce_dds_client`, Ethernet 설정 중 하나만 어긋나도 ROS2 sample이 들어오지 않을 수 있다. 이때 앱은 사용자가 수동으로 QGC나 PX4 shell을 오가며 복구하기 전에, 이미 연결 가능한 MAVLink를 사용해 Pixhawk에 필요한 설정값을 전달해야 한다.

따라서 ROS2 연결 실패 시 MAVLink는 두 역할을 가진다.

- 복구 경로: Pixhawk 쪽 MAVLink stream 설정과 MAVLink `PARAM_SET`/readback 기반 `UXRCE_DDS_*` 설정으로 ROS2가 다시 붙을 조건을 만든다. Shell은 고급/진단 기능이며, Shell이 응답하지 않는 장비에서는 `uxrce_dds_client` namespace/재시작은 PX4 재부팅 또는 별도 차량 설정에 의존한다.
- 대체 경로: ROS2가 끝내 붙지 않아도 MAVLink `ATTITUDE`, GPS, GNSS relative, `SCALED_IMU*`로 가능한 자세/GPS/MAG raw telemetry를 계속 받아 정렬과 측정 입력을 최대한 유지한다.

이 fallback은 ROS2를 완전히 대체하는 기능이 아니다. ROS2 전용 `vehicle_magnetometer.device_id`, namespace 기반 topic 식별, arbitrary `/fmu/out/...` custom field, 구조화된 parameter service는 MAVLink만으로 보장하지 않는다. 앱은 가능한 값은 MAVLink source로 명확히 표시하고, 대체할 수 없는 값은 대기/제한 상태로 남겨야 한다.

## UAM Ground Truth 분석 방법

좌표계와 축 이름의 기준 계약은 `docs/COORDINATE_CONTRACT.md`와 `wiris_stereo_distance/coordinate_contract.py`에 고정한다. 이 섹션은 해당 계약을 UAM ground truth 분석 절차에 적용한 설명이다.

UAM 거리 정확도 평가는 YOLO 3D 측정 row와 Pixhawk/GNSS 기준 위치 row를 1:1로 시간 매칭해 수행한다. 기준 위치는 기체 중심에 배치한 Pixhawk/GNSS body 원점을 사용한다. 앱 모델은 WIRIS/Pixhawk/GNSS가 같은 알루미늄 리그에서 축과 위치가 충분히 공유된다고 보고, 별도의 카메라-기체 보어사이트/레버암 보정 없이 stereo baseline 중앙점을 YOLO range 원점으로 사용한다.

수집 source와 profile 생성 source는 분리한다. StereoDistance는 가능한 경우
PX4 local position, global position, GPS position을 모두 원본 source로
수집/저장한다. Profile 생성 또는 UAM 비교 실행 단계에서는 사용할 truth
source를 명시적으로 하나 선택한다. 선택하지 않은 source는 같은 세션의
진단/재처리 후보로 남기며, 같은 profile 학습 안에서 local/global/GPS truth를
섞지 않는다.

분석 source 기본 우선순위는 다음으로 고정한다.

1. PX4 `.ulg` 파일의 `vehicle_local_position`, `vehicle_odometry`, `vehicle_visual_odometry` 중 사용 가능한 local position.
2. PX4 `.ulg` 파일의 `vehicle_global_position`, `vehicle_gps_position`, `sensor_gps`를 선택 origin 기준 local N/E/D로 변환한 source.
3. `연결 > UAM`에서 ROS2 또는 MAVLink로 실시간 수신한 local/global/GPS 텔레메트리 버퍼.
4. 거리 기록 row에 저장된 `drone_x/y/z`와 `drone_timestamp_s` fallback.

비행 후 분석에서는 ULog를 먼저 선택하고, Raw N/E/D 시계열을 확인한 뒤 필요한 경우 YOLO timestamp에 더할 `time_offset_s`를 조정해 기준 위치와 측정 row의 시간축을 맞춘다. 앱은 PX4 ULog의 `vehicle_gps_position.time_utc_usec` 또는 `sensor_gps.time_utc_usec`를 사용해 PX4 boot/local timestamp를 UTC Unix 시간으로 매핑한다. local position topic에 `timestamp_sample`이 있으면 이를 우선 사용하고, 유효하지 않으면 `timestamp`를 사용한다.

Truth raw data의 축 이름은 PX4 관례를 따라 `N/E/D`로 고정한다. ULog의 `vehicle_local_position.x/y/z`, MAVLink `LOCAL_POSITION_NED.x/y/z`, ROS2 PX4 local position의 `x/y/z`는 앱 내부 호환 필드로 읽을 수 있지만, UI와 분석 CSV에서는 각각 `truth_north_m`, `truth_east_m`, `truth_down_m`으로 해석한다.

Truth source priority is local NED first, GPS/global fallback second. ULog
analysis first uses `vehicle_local_position`, `vehicle_odometry`, or
`vehicle_visual_odometry`. If no usable local NED topic exists, or if the user
selects a GPS/global profile source, the loader may convert
`vehicle_global_position`, `vehicle_gps_position`, or `sensor_gps`
`lat/lon/alt` to local NED using an explicit reference origin. The reference
priority is PX4 local-position `ref_lat/ref_lon/ref_alt`, then `home_position`,
then the first valid GPS/global sample. GPS/global fallback rows must be labeled
as derived truth and must keep raw latitude/longitude/altitude plus reference
metadata; first-sample origin is a fallback/diagnostic basis, not stronger
evidence than PX4 local NED.

카메라 내부 계산의 YOLO 3D 좌표는 `Right/Down/Forward`로 유지하되, Simulation과 최종 축별 비교 표시는 `Forward/Right/Down` 순서로 맞춘다. 공통 비교 계약은 `stereo_center_frd:forward_right_down`이다. 원점은 왼쪽 카메라가 아니라 좌/우 optical center의 중점인 `stereo_center_opencv_optical`로 둔다. 따라서 카메라 중앙선에 대상이 있으면 `Right`는 0에 가까워야 하고, `Forward`가 전방 거리 축이다. 왼쪽 카메라 원점 기준 삼각측량 값은 `raw_left_x/y/z_m` 진단 필드로만 남긴다.

UAM의 명목 가로폭은 YOLO 거리값의 자동 검산 입력으로 쓰지 않는다. bbox가 실제 기체 전체 폭을 잡는지, 날개/동체 일부만 잡는지, 회전과 가림에 따라 어느 범위까지 박스에 포함되는지 확정할 수 없으므로, 최종 정확도는 bbox 폭 모델이 아니라 Pixhawk/GNSS body 원점의 local N/E/D와 시간 매칭된 YOLO 3D 추정값의 오차로 평가한다.

WIRIS visible 8배줌에서 앱의 명목 추정 `fx`는 약 8242 px다. 이 값만 놓고 보면 1852 m 거리의 15 m급 UAM은 영상에서 약 67 px 폭으로 보인다. 이는 장거리 검출과 coarse bbox 추적에는 의미가 있지만, 마커/keypoint/3D pose 기반으로 Pixhawk body origin을 안정 복원하기에는 여전히 제한적이다. 이 계산은 보정 가능성 판단용이며, `intrinsics_zoom_08.json`이 없으면 최종 오차 산출의 근거로 사용하지 않는다.

YOLO row와 Truth row는 시간축으로 1:1 매칭한다. 단, `stereo_center_opencv_optical`의 camera frame과 PX4 local `North/East/Down`은 같은 좌표계가 아니므로 축별 오차를 최종 정확도 지표로 바로 사용하지 않는다. 카메라 기준 축별 비교가 필요할 때는 origin/alignment profile의 역변환으로 `truth_ned`를 `truth_frd_forward/right/down_m`로 바꾼 뒤 `measurement_frd_forward/right/down_m`과 비교한다. PX4 local frame 보고가 필요할 때는 YOLO 측정점을 PX4 local NED에 변환한 `estimated_north/east/down_m`과 `truth_north/east/down_m`을 비교한다. 변환 전 앱이 표시하는 축별 차이는 `mixed-frame` 검토값으로 취급한다.

Roll/Pitch/Yaw가 만드는 장거리 bbox 중심 편향과 stereo depth 증폭 오차는 origin profile의 보정 목표에서 제외한다. 즉 앱은 RPY별 residual correction profile을 만들어 이 오차를 제거하려고 하지 않는다. Pixhawk attitude는 좌표 변환과 baseline 방향 변환에 계속 사용하지만, 남는 RPY 민감도는 `relative_roll/pitch/yaw`, attitude age, 거리 구간, 화면 위치와 함께 기록하고 최종 보고서의 한계 또는 제외 조건으로 처리한다.

좌표 보정 profile은 저장 대상이다. 보정 세션에서는 UAM/드론을 화면의 여러 위치로 이동시켜 UTC/time-offset으로 매칭된 샘플을 확보하고, 선택한 truth source 하나로 `좌표 보정 자동 추정`을 실행한다. Local source는 `camera Right/Down/Forward -> PX4 local N/E/D` 회전행렬과 `origin_offset_n/e/d`를 추정한다. Global/GPS source는 먼저 선택 origin 기준 local N/E/D로 변환한 뒤 같은 추정 과정을 사용한다. Profile ID와 JSON에는 `truth_source_type`, 원본 source frame, origin kind, time basis를 기록한다. 본 실험에서는 같은 카메라/UAM/Pixhawk 장착 조건일 때 불러와 적용한다. profile 적용 후 분석 row는 NED 보고용 `estimated_north/east/down_m`, `error_north/east/down_m`과 FRD 축별 보고용 `measurement_frd_forward/right/down_m`, `truth_frd_forward/right/down_m`, `error_frd_forward/right/down_m`, 그리고 `position_error_m`, `alignment_profile_id`를 남긴다. 최소 3개 이상의 비공선 matched sample이 필요하며, 한 직선 이동만으로 만든 profile은 퇴화로 취급한다.

보정 세션과 본 실험 세션은 분리한다. 보정 세션의 목적은 화면 위치별 matched sample을 확보해 alignment profile을 만들고 저장하는 것이다. 본 실험 세션의 목적은 저장된 profile을 불러온 상태에서 새 YOLO 기록 row와 새 ULog 또는 offline CSV truth source를 비교해 최종 정확도 row를 만드는 것이다. Primary PASS/정확도 증거는 offline 재처리 가능한 ULog/CSV source와 저장 profile을 기준으로 한다. ULog가 없어서 live/recorded-row fallback을 쓴 결과는 `truth_source`를 명시한 fallback 결과로 분리 보고하며, primary PASS 증거와 합치지 않는다. Profile이 없으면 해당 결과는 백업/진단 비교로 남기고 profile 기반 최종 정확도 지표로 취급하지 않는다.

본 실험의 거리값은 baseline 중앙점에서 UAM까지의 range로 정의한다. 모든 `Range`/`Distance` 표시는 이 기준점을 전제로 하므로 UI와 새 CSV 출력 이름에서는 `baseline`을 생략한다. YOLO가 직접 산출한 `distance_m` 또는 분석 row의 표준 `estimated_range_m`은 `stereo_center_opencv_optical` 원점, 즉 좌/우 baseline 중앙점 기준 거리다. profile이 적용된 분석에서는 `origin_offset_n/e/d`를 baseline 중앙점의 PX4 local N/E/D 위치로 보고, ground truth 기준 range를 `truth_range_m = ||truth_ned - origin_offset_ned||`로 계산한다. 따라서 `range_error_m`은 `estimated_range_m - truth_range_m`로 해석한다.

측정 탭의 실시간 거리 그래프는 평가 비행 중 운용 참고용으로 `YOLO Range`와 `Telemetry Range`를 함께 보여준다. 두 값 모두 stereo baseline 중앙점 기준 거리다. `YOLO Range`는 YOLO/stereo 추정 거리이고, `Telemetry Range`는 좌표 보정 profile의 `origin_offset_n/e/d`를 baseline 중앙점의 N/E/D 위치로 사용해 실시간 UAM N/E/D telemetry로 계산한 기준 거리다. 선택 상세와 기록 row에는 `YOLO-Telemetry error`도 남긴다. 최종 정확도 평가는 여전히 `저장·분석 > UAM 비교`에서 ULog 기반 시간 매칭과 profile 적용 후 계산한 결과를 사용한다.

시간 매칭 fallback 순서는 다음으로 고정한다.

1. YOLO 기록 row의 `bbox_mid_utc_unix_s`와 ULog UTC 매핑값을 UTC 기준으로 1:1 매칭한다.
2. UTC가 없거나 `최대 Δt` 안에 맞는 샘플이 없으면 기존처럼 `bbox_mid_timestamp_s`/`timestamp_s + time_offset_s`를 ULog 또는 실시간 버퍼 timestamp와 매칭한다.
3. ULog/live 샘플이 없으면 기록 row에 내장된 `drone_utc_unix_s`를 먼저 사용하고, 없으면 `drone_timestamp_s` 또는 `drone_age_s`를 사용한다.

앱의 `시간 offset 추정`은 UTC 매칭이 가능한 경우 UTC 기준 후보를 우선 찾고, UTC가 없으면 기존 timestamp 기준 후보를 찾는다. 이는 최종값 확정이 아니라 검토 가능한 후보값이다. 운용자는 matched count, 평균 timestamp delta, 사용된 time basis, 오차 그래프를 확인한 뒤 최종 분석에 사용할 offset을 결정한다.

ULog가 없을 때만 실시간 버퍼 또는 기록 row fallback을 사용한다. 분석 결과는 row별 YOLO `Forward/Right/Down`, 보정 profile 적용 시 `estimated North/East/Down`, 기준 `North/East/Down`, frame 정보, 축별 오차, 거리/위치 오차, 매칭 시간 차이를 표시하고 `uam_analysis.csv`로 세션에 저장한다. 분석 화면은 오차 그래프와 Raw NED 그래프를 전환할 수 있어야 한다.

실시간 fallback은 운용 중 손실을 줄이기 위한 백업이지만, ULog가 없거나 사용할 수 없는 세션에서도 최종 보고용 지표 계산에 사용할 수 있다. 단, ULog 기반 결과와 live/recorded-row fallback 기반 결과는 `truth_source`별로 분리해 보고해야 하며 서로 섞어 하나의 대표 지표로 합치지 않는다.

Simulation에서 Origin Profile을 만들 때는 ROS2 topic을 직접 사용한다. `/ground_truth_position`과 `/stereo/estimated_position`은 모두 StereoDistance와 같은 FRD 비교 계약 `stereo_center_frd:forward_right_down`으로 해석한다. 이 Simulation-origin profile은 Gazebo/Simulation truth를 학습 입력으로 쓰는 oracle-style baseline이다. 반대로 `local_ned`, `global_position`, `gps_position` 3개 truth source를 비교하는 실험에서는 Gazebo true pose를 profile 학습에 쓰지 않고, 세 profile을 같은 validation trajectory에서 평가하는 독립 oracle로만 사용한다. profile/평가 CSV는 GUI 세션 기록과 맞춘 `measurement_frd_forward/right/down`, `truth_frd_forward/right/down`, `estimated_range_m`, `yolo_range_m`, `origin_range_m`을 표준 field로 사용한다. RDF 호환 필드는 내부 계산과 과거 CSV 호환용으로 유지한다. PX4 waypoint 입력만 mission API 계약 때문에 `north/east/altitude`를 유지한다. profile 생성 경로는 `camera_model.env`의 8x FOV, 25 m baseline, 15 m target width, `drone_mission.env`의 spawn range를 사용해 stereo overlap 안에서 lateral/altitude가 변하는 비공선 경로로 만든다.

Origin-BBOX Reprojection Profile이 최종 주 방식이다. 이 방식은 ref `Stereo_Camera_3D_Position_DKL`처럼 bbox center pixel을 먼저 보정한 뒤 stereo 3D를 다시 계산한다. ref의 전역 1차 affine은 baseline 후보로만 남기고, 주 방식은 화면 위치와 raw disparity/range bin별 image-space residual profile이다. 학습 label은 `duL = uL_gt - uL_yolo`, `dvL = vL_gt - vL_yolo`, `duR = uR_gt - uR_yolo`, `dvR = vR_gt - vR_yolo`다. GT pixel은 Simulation truth 또는 ULog/Pixhawk N/E/D truth를 `stereo_center_opencv_optical`의 `Right/Down/Forward`로 변환한 뒤, 현재 intrinsics/baseline/extrinsics로 좌/우 이미지에 재투영해 만든다.

Reprojection Profile의 lookup feature는 보정 전 좌/우 bbox center의 평균 화면 위치 `u/v`와 `raw_disparity_px` 또는 `raw_range_m`이다. 기본 profile은 `raw_disparity_px`를 사용하므로 검증 비행 중 외부 거리 truth가 없어도 적용 가능하다. 모델은 screen grid + disparity/range bin + robust median residual이며, 기본 lookup은 exact cell이다. 적용은 raw median `du/dv`를 그대로 쓰지 않고 기본 `component_gains` 정책을 사용한다. 기본 gain은 `disparity_gain=0.125`, `screen_u_gain=0.0`, `screen_v_gain=0.025`로, 좌우 horizontal separation 성분을 보수적으로 열어 Forward/Range를 안정화하고 평균 화면 위치 성분은 과보정 위험 때문에 거의 차단한다. `full` 적용은 baseline/진단 후보이며, `disparity_only`는 screen center 성분을 완전히 막는 보수 후보로만 둔다. `output_policy=triangulated`는 corrected bbox center를 재삼각측량한 `Right/Down/Forward`를 그대로 쓰는 진단/비교 정책이다. 기본 `output_policy=range_only_forward_fusion`은 corrected bbox center와 재삼각측량 결과를 계속 기록하되, 최종 corrected 출력은 raw `Right/Down`을 보존하고 corrected `Forward`만 사용한다. 이 정책은 lateral 축을 개선했다고 주장하지 않고 lateral regression을 막는 보수 적용이다. `nearest` 보간/대체는 JSON 옵션으로만 사용하고, 기본 운용에서는 셀 없음, 학습 범위 밖, bbox pair 불완전, disparity 불량, range-bin profile에서 raw range 없음이면 bbox center를 움직이지 않는 no-op fallback을 남긴다. 기록 row에는 raw bbox center/width/height/confidence, `raw_disparity_px`, `raw_range_m`, corrected bbox center, `left/right_bbox_du/dv_px`, corrected `Right/Down/Forward/Range`, `profile_id`, `profile_mode`, `profile_output_policy`, `profile_fallback_level`, `profile_fallback_reason`을 저장한다.

검증은 보정용 궤적과 검증용 궤적을 분리한다. 보정용 궤적은 detect image의 좌/중/우, 상/중/하와 여러 disparity/range bin을 의도적으로 채워야 한다. 자세 오차가 있는 검증에서는 좌/우 카메라 attitude error를 반영한 실제 reprojection 공통 FOV 안에서 spawn/waypoint를 생성해야 하며, profile cell median이 안정되도록 각 screen/depth cell에 충분히 오래 머무른다. 2026-06-09 right camera Roll/Pitch/Yaw `+0.1deg` 조건의 여러 report/seed 검증에서는 corrected bbox center를 완전 재삼각측량하는 정책이 일부 validation 궤적에서 Right/Down regression을 만들었다. 현재 앱 기본 보수 정책은 `range_only_forward_fusion`이며, 6개 report set의 30개 validation trial에서 Forward/Position RMSE는 개선되고 Right/Down은 raw와 동일하게 보존되어 non-regression을 만족했다. 추가로 `origin_bbox_reprojection_validation_attitude_aware_component_gains_longprofile_right_att_0p1_20260609_seed20260625` report는 `origin-bbox-reprojection-grid-v1 + component_gains` 조건에서 5개 validation trial 모두 `pass=yes`를 기록했고, trial별 최소 개선은 Right `+0.000065 m`, Down `+0.002199 m`, Forward `+3.436333 m`였다. 이는 저장된 Simulation truth를 쓰는 oracle-style 검증 PASS 증거이며, hardware truth 없이 런타임에서 보장된다는 뜻은 아니다. 검증은 서로 다른 random overlap-safe 궤적 5개로 수행하고, 첫 검증 궤적부터 Forward/Right/Down 중 하나라도 corrected RMSE가 raw RMSE보다 증가하면 실패로 본다. 성공 조건은 5개 검증 궤적 모두에서 축별 RMSE non-regression이며, 평균 Position/Range RMSE 개선은 별도 권장 지표다. scalar range error는 보고하되 축별 fail gate와 분리한다.

bbox center가 아닌 후보도 baseline 실험으로 남긴다. `bbox_non_center_method_sweep.py`는 top/bottom/edge/corner/cross-edge/learned-origin anchor와 축별 gain을, `bbox_feature_residual_offline_validation.py`는 bbox size/shape/disparity 기반 residual을 검증한다. 2026-06-09의 0.1도 자세 오차 CSV-only 검증에서는 non-center bbox-only 후보가 모든 report와 validation trial에서 RDF strict improvement를 만족하지 못했다. 전역 non-regression 통과 후보는 gain이 0으로 수렴한 no-op뿐이었고, image/silhouette/template/keypoint 후보는 저장된 image/ROI가 없어 별도 수집 후 검증해야 한다.

Legacy Origin-BBOX 3D residual Profile은 비교용 baseline이다. 이 방식은 bbox 중심의 정규화 화면 위치 `u/v`와 YOLO range를 bin으로 나누고, 각 셀에서 `truth Right/Down/Forward - measured Right/Down/Forward` m residual을 학습한다. m-offset residual은 stereo geometry를 다시 풀지 않으므로 정석 보정이 아니며, 최종 주 방식으로 사용하지 않는다. 기본 생성 정책은 보수적으로 둔다. local regression 후보(`axis_range`, `axis_poly`, `axis_screen_poly`)는 기본 비활성화하고, constant gain 후보만 validation telemetry로 비교한다. 각 축은 validation RMSE가 최소 0.02 m 이상 줄어드는 경우에만 저장하며, 장거리 random 검증에서 불안정했던 Forward 축은 기본 비활성화한다. Right처럼 raw 오차가 작아 mm 단위로 개선/악화가 뒤집히는 축도 이 gate를 통과하지 못하면 원본을 유지한다.

Origin-BBOX Profile의 적용 범위는 보수적으로 제한한다. 검증으로 선택된 `axis_gains`는 같은 `range/u/v` 셀에 정확히 들어온 row에만 적용한다. `same_range_nearest`나 `nearest` fallback 셀에서는 축별 보정을 외삽하지 않고 no-op으로 처리해 `*_axis_noop` fallback level을 남긴다. 검증 샘플이 부족한 셀도 축별 보정을 적용하지 않는다. 성공 판정은 서로 다른 seed의 랜덤 overlap-safe 궤적 5개에서 수행한다. 각 궤적은 25 m baseline, 8배줌 FOV, 15 m target width 조건의 stereo 겹침 영역 안에서 생성해야 하며, 첫 궤적부터 Forward/Right/Down 중 하나라도 corrected RMSE가 raw RMSE보다 증가하면 즉시 실패로 처리하고 profile 생성 방법을 바꾼다. scalar range error는 별도 지표이며, 현재 Origin-BBOX 축 보정 성공 조건에 포함하지 않는다.

`고급 > Bias 보정`은 실시간 운용 표시와 legacy 후보정 검토용이다. 최종 UAM 비교는 Bias profile 적용 후의 표시/기록값이 아니라 raw stereo-center 기록을 우선 사용하고, alignment profile로 FRD/NED 비교 필드를 만든 결과만 최종 축별 평가로 본다.

## 측정 리그 장착 단순화 결정

측정 리그는 알루미늄 가공 케이스로 WIRIS와 측정용 Pixhawk/GNSS의 축 방향을 맞춘다. 따라서 본 프로젝트의 앱 모델에서는 보어사이트와 레버암을 무시하고, 별도 입력/계산/보정을 제공하지 않는다. 모델 이름은 `nominal-shared-rig-v1`로 둔다.

이 프로젝트의 기준 좌표계는 PX4 관례에 맞춰 world는 NED, 기체 body는 FRD로 둔다. 즉 body +X는 전방, +Y는 우측, +Z는 아래 방향이다. 카메라는 OpenCV/ROS optical frame으로 해석한다. 즉 camera +X는 영상 우측, +Y는 영상 아래, +Z는 렌즈가 바라보는 전방 깊이 방향이다.

전방 감시 카메라의 명목 mount profile은 현재 코드와 동일하게 `front_camera_z_to_body_x_x_to_body_y`로 고정한다.

- camera +X right -> body +Y right
- camera +Y down -> body +Z down
- camera +Z forward -> body +X forward

따라서 25 m baseline은 GNSS antenna 간 baseline이자 WIRIS optical-center 간 effective stereo baseline으로 취급한다. 앱은 `Baseline` 입력값 또는 RTK/GNSS baseline을 그대로 stereo translation의 scale로 사용한다.

측정 계산 순서는 `pixel -> camera optical ray -> body FRD ray -> world NED ray`다. 픽셀과 내부 파라미터로 optical ray를 만들고, 고정 mount rotation을 거쳐 body FRD에 올린 뒤, Pixhawk attitude quaternion으로 world NED에 놓는다. 단순히 축 이름을 맞추기 위해 임의로 축을 교환하지 않는다.

본 실험에서 확인해야 할 조건은 다음이다.

- 좌/우 리그의 WIRIS, Pixhawk, GNSS antenna 장착 위치와 방향이 동일하다.
- 좌/우 WIRIS optical axis가 알루미늄 케이스 기준으로 같은 방향을 본다.
- GNSS antenna 간 25 m baseline이 실제 설치 baseline과 일치한다.
- WIRIS visible 8배줌에서 `intrinsics_zoom_08.json`을 만들어 적용한다.
- 남는 장착 제작 오차는 별도 보정하지 않고 최종 오차에 포함한다.

현재 앱 반영 상태는 다음과 같다.

1. nominal mount profile과 Pixhawk attitude 기반 ray 변환은 유지한다.
2. 개별 장착 보정 UI, 설정 파일 저장 필드, CSV 개별 수치 기록은 제거한다.
3. 세션 row에는 `mount_assumption_model=nominal-shared-rig-v1`와 장착 가정만 남긴다.
4. `triangulation.py`는 per-camera 장착 보정을 적용하지 않고, 좌/우 동일 리그 가정으로 baseline을 계산한다.

현재 범위에서 제외하는 것은 명확히 둔다. 좌/우 리그의 상대 배치가 서로 다르거나, 카메라 축 정렬이 케이스 기준에서 크게 틀어졌거나, 짐벌처럼 비행 중 camera-body 관계가 변하거나, zoom에 따라 optical center가 크게 이동하거나, 열화상/가시광 센서의 optical center가 서로 다른 경우는 별도 보정 모델이 필요하다. 이런 조건에서는 `nominal-shared-rig-v1`로 최종 정확도를 보장하지 않는다.

## 핵심 목표

- 좌/우 WIRIS RTSP 스트림을 연결하고, 포트 open이 아니라 실제 첫 프레임 수신까지 확인한다.
- 카메라 preview와 YOLO/depth 측정을 분리해 스트림 상태 확인과 실제 측정을 구분한다.
- WIRIS still image EXIF 시간 차이를 확인해 좌/우 카메라 시간 오차를 파악한다.
- 체커보드 캡처와 줌별 intrinsics 저장/적용으로 WIRIS 내부 파라미터를 관리한다.
- YOLO 검출 결과를 좌/우 카메라에서 시간, 클래스, geometry gate 기준으로 매칭한다.
- 카메라 내부 파라미터, Pixhawk 자세, GPS baseline을 사용해 3D ray triangulation을 수행한다.
- 측정 결과, 외부 파라미터, Pixhawk MAG 상태, 영상 파일, 설정을 세션 단위로 저장한다.
- 현장 운용자가 연결 상태와 측정 품질을 빠르게 판단할 수 있는 UI를 제공한다.

## 시스템 구성

StereoDistance는 PyQt5 GUI가 현장 워크플로를 조율하고, 계산/스트리밍/저장 로직은 `wiris_stereo_distance` 패키지 안의 모듈들이 담당하는 구조다.

현재는 `gui.py`가 많은 상태와 제어 흐름을 가지고 있으므로 완전한 MVC 구조는 아니다. 다만 측정 계산, 삼각측량, 네트워크 탐색, 설정, 저장 같은 핵심 기능은 별도 모듈로 분리되어 있다.

### 진입점과 GUI

- `main.py`: 앱 진입점.
- `wiris_stereo_distance/gui.py`: PyQt5 메인 GUI와 현장 워크플로 오케스트레이션.
- `wiris_stereo_distance/theme.py`: IBM/Carbon 계열 색상 토큰과 공통 스타일 기준.
- `wiris_stereo_distance/camera_pane.py`: 좌/우 카메라 프리뷰, 스트림 워커 연결, bbox 표시.
- `wiris_stereo_distance/measurement_widgets.py`: 거리 기록 그래프와 삼각측량 품질 표시 위젯.

### 스트림과 측정 계산

- `wiris_stereo_distance/yolo_stream.py`: RTSP 수신, 디코딩, YOLO 추론, 프레임/검출 이벤트.
- `wiris_stereo_distance/measurement_controller.py`: 좌/우 검출 매칭과 3D 측정 계산 연결.
- `wiris_stereo_distance/reprojection_profile.py`: Origin-BBOX Reprojection Profile의 GT pixel residual label 생성, grid/median profile, lookup/fallback, JSON 저장/로드.
- `wiris_stereo_distance/stereo.py`: 검출 observation, stereo match, disparity 기반 진단 계산.
- `wiris_stereo_distance/triangulation.py`: ray triangulation과 좌표계 변환.
- `wiris_stereo_distance/uam_telemetry.py`: PX4 ULog local position 로드, 실시간 UAM telemetry fallback 샘플, YOLO row와 ground truth row의 시간 매칭/오차 계산, post-flight time offset 후보 추정.
- `wiris_stereo_distance/sync.py`: timestamp buffer, 좌/우 sample 매칭, sync 품질 분류.
- `wiris_stereo_distance/attitude_bias.py`: 좌/우 Pixhawk 상대 자세와 bias capture 계산.

### 보정, 센서, 장비 제어

- `wiris_stereo_distance/calibration.py`: 체커보드 기반 카메라 보정.
- `wiris_stereo_distance/photo_clock.py`: WIRIS still image EXIF/GPS timestamp 읽기와 좌/우 시간 차이 비교.
- `wiris_stereo_distance/pixhawk_attitude.py`: Pixhawk ROS2/MAVLink 자세와 센서 구독.
- `wiris_stereo_distance/px4_mag_manager.py`: RM3100/PX4 magnetometer 파라미터 상태 계산.
- `wiris_stereo_distance/xrce_agent.py`: MicroXRCEAgent 실행 여부 확인과 시작 보조.
- `wiris_stereo_distance/rtk_geometry.py`: GPS/RTK baseline 계산.
- 장착 모델은 `nominal-shared-rig-v1`로 고정하고, 개별 장착 보정 기능은 앱 범위에서 제외한다.
- `wiris_stereo_distance/wiris_zoom.py`: WIRIS SDK TCP 명령, zoom, Ethernet mode, IP 관련 명령 구성.
- `wiris_stereo_distance/network.py`: 카메라 네트워크 탐색, NIC/RTSP/WIRIS TCP 후보 확인.

### 저장과 설정

- `wiris_stereo_distance/recording_session.py`: 세션 폴더와 CSV/JSON 저장.
- `wiris_stereo_distance/settings.py`: 설정 로드/저장.

## 측정 데이터 흐름

1. 좌/우 WIRIS RTSP 스트림을 연결한다.
2. `YoloStreamWorker`가 프레임과 검출 결과를 GUI로 전달한다.
3. `CameraPane`이 최신 프레임과 bbox 상태를 보관한다.
4. `gui.py`가 좌/우 검출 후보와 Pixhawk/RTK 상태를 모은다.
5. `measurement_controller.py`가 stereo match와 triangulation 결과를 계산한다.
6. 결과는 거리 상태, 그래프, 세션 기록 테이블에 반영된다.
7. 저장 시 `recording_session.py`가 CSV/JSON/영상 경로를 세션 단위로 정리한다.

## 주요 워크플로

1. QGC에서 Pixhawk 보정 완료.
2. `./run.sh`로 앱 실행.
3. `연결` 탭에서 WIRIS, RTSP, Pixhawk, UAM ground truth telemetry를 확인하고, 수동 SDK/필드 감시는 `고급` 탭에서 수행한다.
4. `시간` 탭에서 WIRIS 사진 EXIF 시간 차이 확인.
5. `보정` 탭에서 체커보드 캡처와 내부 파라미터 보정.
6. `파라미터` 탭의 `내부`, `외부/기하` 세부탭에서 현재 내부/외부 파라미터 확인.
7. `정렬` 탭에서 Baseline, MAG, Pixhawk 자세 상태 확인.
8. `YOLO` 탭에서 모델, 장치, 스트림 설정 조정.
9. `측정` 탭에서 `세션 기록 시작`을 누른 뒤 YOLO/depth 측정을 시작한다. Pixhawk 부팅 전 또는 ULog 시간 범위 밖의 YOLO row는 기록될 수 있지만 최종 UAM 비교에서는 matched row에 포함되지 않고 unmatched로 보고된다.
10. 보정 세션에서는 `저장·분석 > UAM 비교`에서 ULog Raw N/E/D 확인, time offset 후보 추정/검토, `좌표 보정 자동 추정`, `보정 저장`을 수행한다.
11. 본 실험 세션에서는 `보정 불러오기`, 새 ULog 선택/로드 또는 실시간 telemetry fallback 확인, time offset 후보 추정/검토, `UAM 비교 실행`을 수행한다. 최종 정확도 지표는 매칭된 row 전체에서 계산하되 ULog/live/recorded-row source별로 분리해 보고한다.
12. `세션 저장`에서 거리 기록, UAM 분석 CSV, 설정과 메타데이터를 저장한다.
13. 필요하면 `고급 > WIRIS SDK` 또는 `고급 > 필드 감시`에서 저수준 명령/ROS2/MAVLink field를 직접 확인한다.

## 연결 확인 기준

- RTSP는 포트 open만으로 충분하지 않다. `영상 수신 확인`에서 첫 프레임 수신까지 확인한다.
- 카메라 preview는 YOLO 없이 스트림 상태를 확인하는 용도다.
- YOLO 측정은 실제 거리 측정이 필요할 때만 시작한다.
- Pixhawk ROS2와 MAVLink는 별도 경로지만 복구 과정에서는 함께 사용할 수 있다.
- ROS2는 topic 이름만으로 연결 성공으로 보지 않는다. publisher와 sample 수신까지 확인한다.
- MAVLink는 heartbeat와 필요한 메시지 수신을 확인한다.

## 저장 기준

- 유효한 3D triangulation 결과가 있을 때 거리 샘플을 기록한다.
- 세션 저장은 측정 CSV, 외부 파라미터 CSV, Pixhawk MAG 상태, 영상 파일을 같은 세션 폴더에 묶는 것을 목표로 한다.

## 설계 방향

기능과 디자인은 점진적으로 분리한다.

- 계산 로직은 `measurement_controller.py`, `triangulation.py`, `calibration.py`처럼 테스트 가능한 모듈에 둔다.
- GUI는 화면 구성, 사용자 입력, 상태 표시, 워크플로 연결에 집중한다.
- 카메라/Pixhawk/저장 같은 장비 제어 흐름은 장기적으로 service/controller 계층으로 분리한다.
- 스타일과 디자인 토큰은 Python 코드 안에 흩어두지 않고 별도 디자인 기준 문서와 테마 코드로 이동한다.
- 측정 경로는 3D ray triangulation을 우선하며, 단순 disparity 값은 진단 지표로만 취급한다.
- hardware I/O와 UI orchestration은 남아 있는 결합 지점이므로 큰 수정 전에 관련 테스트나 현장 smoke check 기준을 먼저 정한다.
- Pixhawk 연결은 ROS2를 선호하되, 실패 시 MAVLink로 uXRCE 설정을 밀어 넣고 가능한 자세/GPS/MAG raw telemetry를 계속 사용한다. 단, ROS2 전용 device_id, namespace, custom uORB field는 fallback 대상이 아니다.

## 현재 결합 지점과 분리 계획

- 스타일 토큰 분리
  - 완료: `wiris_stereo_distance/theme.py`에 IBM/Carbon 색상 토큰을 추가했다.
  - 다음: 전역 QSS 생성 함수를 별도 모듈로 분리한다.
- 그래프 위젯 분리
  - 현재: 일부 그래프는 `measurement_widgets.py`, 일부 Pixhawk/RTK 그래프는 `gui.py`에 있다.
  - 다음: Pixhawk/RTK 그래프를 별도 widgets 모듈로 이동한다.
- 카메라 service 분리
  - 현재: `gui.py`가 stream 시작/중지와 UI 상태를 함께 처리한다.
  - 다음: URL 검증, stream kwargs, start/stop orchestration을 작은 service로 분리한다.
- Pixhawk service 분리
  - 현재: ROS2/MAVLink 연결, 복구, fallback, UI 업데이트가 GUI에 강하게 묶여 있다.
  - 다음: 연결 상태, 후보 탐색, 복구 명령, telemetry source 선택, 로그 이벤트를 service/controller 경계로 나눈다.
- 세션 recording controller 분리
  - 현재: 기록 상태와 table 갱신, 저장 호출이 GUI에 있다.
  - 다음: 기록 row 생성과 저장 orchestration을 GUI 밖으로 이동한다.
- 측정 리그 장착 모델
  - 현재: nominal `camera_mount_profile`과 `nominal-shared-rig-v1` 장착 가정을 유지하고, 좌/우 동일 리그 조건에서 baseline을 사용한다.
  - 다음: 좌/우 리그가 동일하지 않은 새 하드웨어 조건이 생길 때만 별도 보정 모델을 재검토한다.

분리 작업은 한 번에 대규모 재작성하지 않는다. 동작을 잠그는 테스트를 먼저 확인하고, 새 의존성은 추가하지 않으며, 각 단계는 `UPDATE.md`에 한글로 기록한다.

## 디자인 방향

현재 디자인 기준은 `DESIGN.md`를 우선 참고한다. `DESIGN-ibm.md`는 IBM/Carbon 계열 원본 레퍼런스로 둔다.

적용 방향은 IBM/Carbon 계열의 구조적인 운용 대시보드다.

- 차분한 흰색/회색/차콜 기반.
- IBM Blue 계열의 단일 강조색.
- 얇은 구분선과 낮은 장식성.
- 상태, 입력, 버튼, 로그, 그래프가 빠르게 스캔되는 정보 밀도.

Apple식 마케팅/갤러리 레이아웃은 이 앱의 목적과 맞지 않으므로 보조 참고로만 사용한다.

## 문서 역할

- `usage.md`: 실행 방법, 상세 운용 방법, 입력/출력, 제한 사항.
- `PROJECT.md`: 프로젝트 목적, 시스템 구성, 핵심 워크플로, 측정 흐름, 설계 방향, 분리 계획.
- `DESIGN.md`: 프로젝트 UI/스타일 기준.
- `DESIGN-ibm.md`: IBM/Carbon 계열 원본 디자인 레퍼런스와 토큰.
- `../UPDATE.md`: 변경 이력과 인수인계 로그.
- `AGENTS.md`: 코딩 에이전트 작업 규칙.
- `README.md`: 문서 목차.

## 세부 문서

- `testing.md`: 테스트와 검증 방법.

## 현재 주의점

- `gui.py`는 아직 화면 구성, 스타일, 장비 제어, 저장 흐름을 많이 함께 가지고 있다.
- 일부 그래프와 Pixhawk/RTK UI 흐름도 아직 `gui.py`에 남아 있다.
- 기존 PyQt stylesheet는 1차로 IBM/Carbon 기준을 적용했지만, 세부 화면 밀도와 레이아웃은 추가 조정이 필요할 수 있다.
- `stereo_distance_settings.json`은 앱 루트 기준 설정 파일이며, 보정/시간 저장 상대경로도 앱 루트 기준으로 해석한다.
- RTSP 포트 open, ROS2 topic 이름 존재, MicroXRCEAgent 실행만으로 실제 측정 준비 완료를 의미하지 않는다. 첫 프레임, publisher/sample, MAVLink heartbeat, MAG/attitude 수신을 별도로 확인해야 한다.
- 하드웨어 연결이 필요한 검증은 자동 테스트만으로 충분히 확인할 수 없다.
