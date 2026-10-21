# 분석 시간 단축: 계획과 검증 기록

2026-10-18. 사용자 요청: 진행 중 분석 감시, 개선 기준점 확보, 완료 확인 후 계획 진행.
추가 요청: risk-plan / auto-subagents / verify-check 적용.

## 범위와 불변조건

현재 분석과 기존 후속 검증은 그대로 실행한다. 원본·기존 결과·코드·설정·worker 수를 변경하지 않는다.
39개 슬롯, 100 MC, 표본 선택, 실패 분류, V0 동결 규칙을 유지한다.
공유 캐시/재개/병렬 실행은 이미 있으므로 중복 구현하지 않는다.
실제 후보 코드는 프로파일 근거가 생긴 뒤 선택한다. 이 기록은 속도 개선 완료 증거가 아니다.

## 검증 단계

| ID | 관측 목표 / 선행조건 | 검증·통과 기준 | 실패·복귀 | 현재 상태 |
|---|---|---|---|---|
| V-01 | 현재 작업 식별과 기준점 | 실행 argv/analysis ID 확인, 기존 30초 자원 로그의 연속 표본과 진행 로그 대조 | PID 소멸만으로 성공 판정 금지 | PASS: PID 370079, MC100/workers8, 기존 감시기 확인 |
| V-02 | 분석 종료 / V-01 | 종료 receipt exit=0, ANALYSIS_COMPLETE 로그, 최신 pointer ID, summary/checksums 존재 | 실패면 후보 실행 보류하고 receipt 기록 | NOT_RUN: 분석 중 |
| V-03 | 기존 후속 감사 완료 / V-02 | 기존 post_analysis_verification_001 성공 receipt와 auditor PASS; 입력·코드 동일성 | 감사 실패/코드 변경 시 중단, 원본 보존 | NOT_RUN |
| V-04 | 대표 입력 기준점 / V-03 | G0 P0/P1/P4, G1 P4 각 trial0의 all_data/matched200, A/B/C×RPY_IMU/YAW_ONLY. 기존 selected_pair_ids 그대로, 센서/RTK/설정 snapshot, C에 RTK 금지 | 누락·재구성 불일치 시 중단 | NOT_RUN |
| V-05 | 병렬도 실측 / V-04 | 동일 8개 작업, BLAS1, 4/8 workers 각 3회 교차 측정; 모든 결과 canonical hash 동일. 중앙시간/최대 관측 RSS/개별 시간 기록 | 차이·오류·timeout 시 후보 격리. 기존8 유지 | NOT_RUN |
| V-06 | 후보 선정·검증 / V-05 | 프로파일 근거로 최소 변경. 별도 후보에서 결과·표본·rank·실패·공분산 검증 및 해당 테스트 | 결과 의미 변화 시 적용 중지. 기존 구현 유지 | NOT_RUN |
| V-07 | 사용자 원래 흐름 / V-06 | 별도 출력의 짧은 실제 분석→저장→재개→감사, 미완료/손상/잠금 충돌 경로. 최종100회 기준 유지 | 영향 범위만 수정·재검증 | NOT_RUN |

V-05의 3회, 잠정 채택 기준인 중앙시간 10% 감소는 공학적 비교 조건이며 사용자 합의 SLA가 아니다.
단일 표본 프로파일은 함수별 병목 가설만 제공하며 전체 캠페인 절감률로 환산하지 않는다.
계측 전 예열·순서 편향과 대표성 한계를 보고한다. 4→8→8→4→4→8 순서로 비교한다.

## 위험 및 결정 연결

- D-01 / R-01 / G-01(V-02~04): 검증된 결과 재사용 vs 재계산. 기본은 해시·완료 검증 후 재사용. 오염 신호는 입력/결과 identity 불일치. 즉시 보류하고 해당 범위 재계산으로 전환.
- D-02 / R-02 / G-02(V-05): workers8 유지 vs 조정. CPU 포화는 관측, 최적 worker 수는 미확인. 동일 입력 반복 측정 후만 선택. 악화·결과 차이는 기존8로 복귀.
- D-03 / R-03 / G-03(V-06~07): solver 불변 RTK 공분산 준비의 중복 제거 vs 현상 유지. residual 반복마다 준비하는 코드는 확인했지만 영향도는 가설. 프로파일에서 비중이 확인될 때만 확장. 최적화 대상은 world ENU 위치/공분산, optical RDF 영상 관측; 회전/단위/RTK·truth 분리 불변.
- 담당: Codex 조정자, 소스 조사 에이전트는 읽기 전용. 미확정 구현 선택은 proposed, ADR 아직 해당 없음. 구현 선택 시 기존 ADR 경로에 기록한다.
- update-architecture: NOT_REQUESTED. 현재 소스/계약 변경 없음. 공유 상태·수명주기 변경이 필요하면 영향 검토 후 별도 기록.

## 근거 및 상태 이력 (추가 기록)

- 2026-10-18: root/apps AGENTS, risk-plan/분기 규칙, auto-subagents, verify-check/최소변경·라우팅 지침 확인.
- 기존 evidence: apps/Simulation/recordings/stereo_v1_1/paper1_v1_2_001/source_evidence/full_analysis_execution_002/{command.json,resource_monitor.jsonl,analysis.log,result.json}.
- 기존 감사: 같은 캠페인의 source_evidence/post_analysis_verification_001/{plan.json,events.jsonl,result.json,analysis_results_audit.json}.
- 읽기 전용 프로세스 확인: 주 프로세스 PID370079, worker8개 각각 약98% CPU, family RSS 약1.2GB. CPU 계산 병목 가능성, 디스크 병목 배제까지 입증한 것은 아님.
- 로그 CALIBRATION 표시는 task 제출 이벤트(최대16개 선행 가능)이며 완료율이 아니다. 앞선 '13번째 진입' 설명은 완료율로 해석하면 안 된다.
- 독립 조사: analysis_execution.py 기존 bounded spawn/BLAS 제한/lock; analyze_campaign.py 기존 재사용/버전 identity; calibration_solver.py 공분산 반복 준비 후보 확인. 아직 테스트/후보 실행 안 함.

자동 후속 계측은 V-02와 V-03 PASS 후에만 시작한다. 소스 변경·감사 실패·출력 잠금 충돌 시 HOLD/STOP을 기록하며 실행 중인 타 작업에 신호를 보내지 않는다.

- 독립 검토에서 6작업으로 8 workers를 비교할 수 없는 문제, profiler의 timeout/thread 환경 문제, 완료 코드 identity 대조 누락을 발견하여 수정했다. 비교 입력은 8작업으로 확장하고 profiler를 제한된 별도 프로세스로 이동했다. 최종 검토에서 실행 차단 결함 없음. 단, 작은 배치 결과는 전체 처리량을 보장하지 않는다.
- 종료·감사 gate의 독립 fixture 검증: 완료 receipt 없음, 분석 실패, summary만 존재, 감사 대기, 최종 checksum 누락, 오래된 pointer/감사 receipt, auditor FAIL, 두 단계 성공 및 JSONL 부분쓰기 경로를 검사한다. 실제 분석 종료 검증과 구분한다.
- 이후 자동 상태 전이와 계측 결과는 이 폴더의 events.jsonl / result.json에 추가 기록한다. baseline.json은 원본 자원 이력에서 재생성한 요약이다.

- 실행 확인: 프로젝트 .venv의 unittest 10/10 PASS(0.035s). 대기 감독기 exec session 44511 시작; WAITING_FOR_ANALYSIS_AND_EXISTING_AUDIT 기록과 baseline.json 생성 확인. V-02~07은 아직 실제 PASS 아님.
