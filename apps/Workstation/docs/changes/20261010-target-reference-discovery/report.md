# Target reference 탐색·연결 개선 결과

승인된 소프트웨어 구현을 완료했다. Data → Status → Target reference에서 Drone을 선택하면 Find Network / Find USB / Cancel, 후보표, Use selected / Connect target / Stop target / Save를 사용할 수 있다.

## 사용 순서

1. Drone 선택 후 UDP 수신 포트 또는 USB/Radio baud를 확인한다.
2. Find Network 또는 Find USB로 후보를 찾는다. 네트워크는 이미 PC로 도착하는 MAVLink를 수신하며, 기체 측 송신 주소를 설정하거나 전체 네트워크를 스캔하지 않는다.
3. 주소·SYS/COMP·최근 heartbeat를 확인하고 Use selected를 누른다. 이 단계는 연결 초안만 채운다.
4. Connect target으로 선택 기체를 재확인한다. 후보는30초 후 만료되고 UDP peer를 함께 확인한다.
5. heartbeat/GPS1/GPS2/ATTITUDE 상태와 수신 나이를 확인한다. heartbeat만 있으면 GPS는 UNKNOWN으로 표시된다. 데이터가3초를 넘으면 STALE이다.
6. Stop target으로 연결을 종료하고, 필요한 경우 Save로 연결 선호값을 저장한다. 재시작 때 설정은 복원되지만 연결은 자동으로 시작되지 않는다.

UAM은 Set UAM deferred로 외부 사후 RTK 대기를 표시하며 실시간 worker/socket을 만들지 않는다. 수동 endpoint 입력도 지원한다. 신원이 확인되지 않은 USB 장치는 NO HEARTBEAT로 표시하며 자동으로 확인된 후보가 되지 않는다.

## 실제 변경

- 독립 TargetReferenceView와 TargetDiscoveryController에 화면·탐색 상태·취소·후보 만료·비동기 Stop을 배치했다.
- UDP/USB 수신 경로는 MAVLink CRC 검증, peer/IDs 확인, bounded packet/candidate 처리와 독점 자원 획득을 수행한다. Target 경로에는 MAVLink 송신 API가 없다.
- 기존 Radio의 serial lease key를 Target·Pixhawk USB probe·Base GNSS worker/probe가 공유해 동시에 같은 직렬 장치를 여는 경쟁을 줄였다. 실제Target transport에는 OS tty exclusion도 적용한다.
- TargetReferenceSettings는 독립적인 nested 설정이다. 원자 저장 실패 시 이전 파일과 메모리 설정을 보존한다. /dev 경로는 명시적인 machine-local 장치 설정이다.
- 비동기 Stop의 Measurement 상태 통지는 GUI thread의 QObject relay로 복귀한다. resource inventory와 앱 종료에 탐색·Stop·수신 thread가 포함된다.
- 기존 Target 원시 좌표 필드와 P1/P2·Telemetry 역할 구분을 유지했다. 연결 lifecycle metadata에 expected/verified peer만 추가했다.

## 검증

- 최종 Workstation + 공통 좌표 회귀: **2266 passed,2 skipped,1 warning,266.14s,exit0**. 세션 종료까지 완료했다.
- Target/Pixhawk/GNSS/RTCM/settings 경계 그룹: **178 passed**. 전체 시험과 중복되므로 합산하지 않는다.
- 실제 UDP loopback/PTY, 별도 프로세스 UDP 점유, 고정 heartbeat/손상CRC, 같은IDs의 다른peer, 송신0, busy장치, 취소/만료/중복, 저장 실패, GUI thread 실행을 확인했다.
- 두 개의 새 프로세스로 저장→종료→UI 재시작을 확인했다. 연결 정보 복원, 자동 연결0, callback 오류0이다.
- 1366×900/1600×980과100/150% 배율4조합의20개 화면을 생성했다. 각 조합 내 상태 변화에도 Target 영역 크기가 동일했고, 예상 밖 callback0, 종료 활성자원0이었다. 화면 예시는 evidence/target-views 아래에 있다.
- 최종 source inventory437파일은 시험 시작 후 내용이 바뀌지 않았다. 로그와 hash는 evidence/final-acceptance.json에 연결했다.

중간 전체 실행에서 기존 최소 settings adapter의 새 필드 부재로1개 실패가 발생했다. 기본 설정 fallback으로 수정하고 기존 실패시험과 전체 시험을 다시 통과했다. 상세 이력은 verification.md에 보존한다.

## 한계와 인계

실장비를 탐색하거나 장치 설정을 변경하지 않았다. 실제 USB/Radio의 무송신 heartbeat 수신과 serial open의 DTR/RTS 영향, 현장 멀티 NIC 동작은 별도 확인 대상이다. 소프트웨어 lease/OS exclusion은 이를 준수하는 앱 경계 및 지원 OS 동작에 기반하며, 이미 열린 비협조적 외부 serial descriptor까지 보편적으로 통제한다는 주장은 하지 않는다.

NVENC 사용 불가와 optional pyproj 부재로2개 시험을 건너뛰었으며 NumPy/SciPy 버전 경고와 offscreen GL 제한이 있다. 이 체크아웃의 공식 실행환경(.venv/필수패키지/기본 모델 경로)은 앞선 실행 점검과 동일하게 준비가 필요하다. 지금 떠 있는 이전 확인용 앱에는 새 코드가 자동 반영되지 않아 재실행해야 한다.

변경 소스·시험·도구·문서는 모두 저장소 안에 있으며 미커밋 상태다. 기존 RCA/갓 모듈 분리 변경과 별도 Simulation 작업을 보존했다. 실제 장비 대상으로 자동 탐색을 실행하는 것은 이번 소프트웨어 검증에 포함되지 않는다.

Coordinate contract checked: Target WGS84·NED·body FRD, camera optical RDF·baseline-center FRD, 기존 좌표/기록 회귀시험; 좌표 변환식 변경 없음.
