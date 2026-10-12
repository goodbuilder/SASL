# 탐색·연결·저장 계약

후보 필드: discovery_generation,candidate_id,transport,endpoint,peer IP/port 또는 canonical device,baud,SYS/COMP,heartbeat_count,observed_monotonic_s,identity_state,busy_owner,error. candidate_id는 GUI 행 번호가 아니다. 취소/재탐색/UAM 변경 뒤 늦게 도착한 결과는 generation으로 거부한다. 유효기간 기본30초를 제안하며 연결 때 실제 heartbeat를 다시 확인한다.

UDP 탐색은 사용자 선택 수신 포트의 passive listen이다. 기본 목록은14570과14550–14559를 제안하며 중복 제거/1..65535/최대16개를 검증한다. 전체 서브넷 스캔·브로드캐스트 송신·네트워크 설정 변경은 없다. MAVLink 정상 parser로 CRC/메시지 구조를 확인한다. 유효 heartbeat는 발신자가 주장하는 식별 정보이며 암호학적 기체 인증으로 표현하지 않는다. 서명 정보가 있다고 무검증 상태를 authenticated로 승격하지 않는다.

기존 owner의 활성 UDP 포트는 탐색에서 제외하고 점유자를 표시한다. 다른 프로세스 점유는 독점 bind 실패로 표시한다. SO_REUSEADDR/REUSEPORT로 같은 포트 수신을 가로채지 않는다. 부분 bind 성공은 PARTIAL과 포트별 원인을 반환한다. 취소·예외에도 열린 소켓을 finally에서 정리한다.

USB는 기존 장치 열거와 /dev/serial/by-id 우선 정규화를 재사용한다. 같은 장치 별칭을 중복 probe하지 않는다. GCS heartbeat, stream request, PARAM, COMMAND, RTCM, logger 송신은 금지한다. 기존 USB probe의 heartbeat_send가 없는 별도 수신 adapter를 사용한다. 소유권 잠금+OS 독점 열기를 적용하며, 열기 자체의 DTR/RTS·장치 reset 영향은 별도 확인한다. probe가 즉시 취소되지 않는 transport는 미지원/시간초과로 표시한다.

네트워크 후보의 peer IP/port와 수신 endpoint를 구분한다. SYS/COMP만 같은 다른 peer를 동일 기체로 받아들이지 않는다. discovery→connect 사이 점유권 handoff는 먼저 probe를 종료하고 connection owner가 다시 독점 획득한다. 수신 adapter에서 peer 검증이 불가능하면 해당 경로를 검증 완료로 표시하지 않고 제한 사유를 낸다. 다중 송신 포트에서는 제한시간 내 선택한 peer+IDs의 heartbeat를 찾고 다른 source는 계수해 제외한다. 신원 검증된 source의 ATTITUDE/GPS만 기존 record pipeline에 전달한다.

snapshot 추가는 수신 host monotonic 기준 heartbeat/GPS1/GPS2/ATTITUDE age, validity, GPS fix/RTK와 refusal/error를 포함한다. 관측마다 실제 출처와 clock을 유지한다. GPS age3초, heartbeat age3초의 STALE 기준을 초기 제안하고 기존 runtime timeout과 정합성을 시험한다. 시각 기준 없는 데이터는 UNKNOWN이다. 데이터 freshness가 나빠져도 기존 Measurement의 시작/지속 정책에 새 gate를 추가하지 않는다.

설정은 명시적 Save로 platform,transport,endpoint,baud,optional expected IDs,수신 포트 목록만 원자 저장한다. 후보 목록·LIVE/verified 상태·관찰 timestamp는 저장하지 않는다. 저장 실패는 기존 값을 보존한다. /dev 장치는 machine-local 계약으로 명시하며 이동한 PC에서 없으면 미연결 상태로 표시한다. 임의 checkout/home 경로를 설정에 저장하지 않는다.

좌표 검토: target record는 WGS84 latitude/longitude(deg), ellipsoid/MSL 고도(m)를 구별한다. world NED=[north,east,down], PX4 body FRD=[forward,right,down] 자세와 GNSS yaw/COG를 별개로 취급한다. 카메라 optical RDF=[right,down,forward], left/right camera 및 optical baseline-center FRD, 별도 GNSS midpoint frame으로 자동 변환하지 않는다. target raw source는 P1/P2 또는 calibration truth가 아니다. 신규 UI는 좌표 대신 identity/age/fix 중심으로 표시해 암묵적 좌표변환을 만들지 않는다.
