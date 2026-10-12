# Impact and ownership

| 경계 | 이전 | 이후 | 증거 |
|---|---|---|---|
| snapshot | frozen shell + mutable nested JSON | recursive immutable owned values + detached export | mutation/hash/spawn |
| run | panel shared state | Qt-free single state owner + compatibility projection | state/late generation/controller/panel |
| workers | actor owns processes, no user recovery | same actor + observable state and queued restart | real child recovery, peer/queue identity |
| storage | JSONL authoritative, CSV/XLSX derived | 같은 schema/values; new recovery failures use existing FAILED reason | journal/workbook reproduction |
| UI | Data Status pending count | compact method status/explicit recovery in existing style | click, busy, stable selection/geometry |
| CI | deleted file target | whole Workstation directory | collect and full run |
| portability | unclassified historical references | exact content-hash historical records | active-path negative tests |

공유 자원: 기존 카메라, telemetry, recording 소유자와 worker 수/큐 한도/CPU-GPU 배치는 유지한다. 추가 프로세스 상시 실행은 없다. 구조 변경은 Measurement Run 소유권과 해당 adapter까지로 제한한다.

좌표 검토: world NED; PX4 body FRD quaternion wxyz; optical RDF pixel u/right,v/down; canonical X_right=R X_left+T; C=-R^T T; public baseline-center FRD metres; GNSS local-level center는 optical center와 별개; WGS84 deg/ellipsoid/MSL distinct. serialization 보강은 순서/단위/부호/원점을 바꾸지 않는다.
