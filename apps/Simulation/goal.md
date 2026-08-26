# 재부팅 후 실행할 Goal: YOLO 15 Hz + 수직 화면 이동 제거

## 2026-08-26 사용자 최신 계약(아래 기존 수직경로 계약보다 우선)

- 실험의 주 목적은 각 waypoint hold에서 오른쪽 카메라 optical
  roll/pitch/yaw 오차가 거리 오차에 미치는 영향을 비교하는 것이다.
- 주 지표는 `nominal_pose_range_m - gt_range_m`(추정-값 Gazebo GT)이며,
  각 자세오차 case의 hold median에서 같은 경로 baseline hold median을 뺀 값을
  `attitude_induced_error_delta_m`으로 기록한다.
- 경로 pitch가 정확히 20°인지는 현재 실험의 합격 조건이 아니다. waypoint는
  모든 hold에서 양안 유효 거리 sample을 얻을 수 있도록 추후 수정할 수 있다.
- 자세오차 비교 전에 모든 case가 동일 waypoint 파일, guidance mode,
  모델/SDF hash, hold 시간을 사용해야 한다.

### 확정된 원인과 해결 증거

- E01 ramp-only: command speed p95 `2.000 m/s`에도 실측 수평
  `11.54~11.62 m/s`, 수직 `5.12~5.13 m/s`, 구간 이탈 `7.21~8.23 m`.
- E02 position-only final target: velocity feed-forward를 제거해도 수평
  `11.49~11.59 m/s`, 수직 `5.12~5.14 m/s`, 구간 이탈 `7.25~7.51 m`.
- 결론: 먼 final-position setpoint의 PX4 position P항이 전역 속도 한계를
  지배한 것이 원인이다.
- E03 `velocity_only`: 실측 수평 p95 `1.95~1.97 m/s`, 수직
  `0.63~0.68 m/s`, 구간 이탈 `0.14~0.24 m`, hold 위치 p95
  `0.39~0.56 m`. 이 mode를 attitude sweep 기본으로 선정했다.
- PASS smoke:
  `recordings/vertical-solved-attitude-smoke-20260826_attempt01`
  (center waypoint 1~6, baseline + right pitch 0.2°, 2/2 case PASS).
- 결과 표:
  `waypoint_attitude_distance_errors.csv`; waypoint 1~6의 pitch 0.2° baseline 대비
  median delta는 `-1.534,-0.903,-0.828,-0.597,-0.250,-0.073 m`.

### 남은 작업

1. 현재 화면 밖인 waypoint 7/8을 포함해 최종 측정 waypoint 집합을
   모든 attitude case에서 양안 가시로 확정한다.
2. 확정된 동일 경로에서 center/zigzag와 roll/pitch/yaw `0.2,0.5,1.0°`
   전체 sweep을 실행한다.
3. 진단 smoke에서 낮춘 unique camera-source `10 Hz` gate를 최종 `15 Hz`로
   회복하고, 좌/우 모두 PASS를 입증한다.

## 2026-08-26 20-run 자세오차 측정 실행 조건

이 campaign은 **최종 camera-source 15 Hz qualification이 아니라**, 현재 검증된
6-waypoint/10 Hz camera-source gate에서 자세오차별 거리 측정값을 수집하는 20-run이다.
YOLO update 최소 15 Hz와 inference p95 66.7 ms 이하는 그대로 유지한다.

### 고정 실험 계약

- source commit: `c439799` (`feat(simulation): stabilize attitude sweep guidance and evidence`)
- trajectory: `center,zigzag`
- 각 trajectory별 case:
  - baseline 1회
  - right optical roll `0.2,0.5,1.0°`
  - right optical pitch `0.2,0.5,1.0°`
  - right optical yaw `0.2,0.5,1.0°`
- 총 case: `2 × (1 + 9) = 20`
- repetition: 각 non-zero 조건 1회
- 실행 순서: `--ordered`
- guidance: `velocity_only`
- 요청 속도: `2.0 m/s`, horizontal ramp `1.0 s`, vertical ramp `2.5 s`
- waypoint hold: 5초, 분석 window는 마지막 3초
- hold 안정성:
  - 3D position-error p95 `<= 1 m`
  - `abs(roll), abs(pitch) <= 15°`
- camera/YOLO/overlay 요청률: 각각 `20 Hz`
- 진단용 unique camera-source/overlay 최소 gate: `10 Hz`
- YOLO 최소 update median: `15 Hz`
- YOLO inference p95 최대: `66.7 ms`
- 모델/SDF와 protected geometry hash는 모든 20 case에서 동일해야 한다.

20-run에서는 현재 양안 거리 sample이 검증된 다음 6개 world RFU waypoint만 사용한다.
기본 8개 중 화면 밖인 7/8번은 포함하지 않는다.

```text
R,F,U
0,415,154
0,365,137
0,315,121
0,265,104
0,215,88
0,165,71
```

zigzag는 같은 Front/Up을 유지하고 기존 40% stereo-overlap 규칙으로 Right만 생성한다.

### 실행 전 확인

1. 다른 `run_operator_view`, PX4, Gazebo, rqt/RViz session이 없어야 한다.
2. `nvidia-smi`가 RTX 4050과 driver를 정상 출력해야 한다.
3. `.venv/bin/python`에서 `torch.cuda.is_available() == True`이고 CUDA tensor 생성이
   성공해야 한다.
4. 최소 약 `5 GB`의 여유 공간을 확보한다. 2-case smoke 실측은 약 `430 MB`였고,
   20 case 예상량은 약 `4.3 GB`다. 2026-08-26 확인 당시 여유 공간은 약 `1.1 TB`였다.
5. 예상 총 실행 시간은 약 `2시간`이다. GUI/RViz/rqt는 각 case에서 표시되고 case
   종료 시 다음 case를 위해 다시 시작된다.
6. `--skip-colcon-build --skip-px4-build`는 commit `c439799`의 검증된 build/install을
   그대로 재사용할 때만 사용한다. source/build가 달라졌으면 두 옵션을 제거한다.

### 실행 명령

output directory는 반드시 존재하지 않는 새 경로를 사용한다.

```bash
env SWEEP_MIN_OVERLAY_HZ=10 \
  ./run_auto_attitude_sweep.sh \
    --axes roll,pitch,yaw \
    --values-deg 0.2,0.5,1.0 \
    --repetitions 1 \
    --trajectories center,zigzag \
    --waypoint-count 6 \
    --waypoints-rfu-m '0,415,154;0,365,137;0,315,121;0,265,104;0,215,88;0,165,71' \
    --ordered \
    --keep-going \
    --guidance-mode velocity_only \
    --camera-hz 20 \
    --yolo-hz 20 \
    --overlay-hz 20 \
    --skip-colcon-build \
    --skip-px4-build \
    --output-dir "recordings/attitude-20run-$(date +%Y%m%d_%H%M%S)" \
    -- \
    --ramp-s 1.0 \
    --vertical-ramp-s 2.5
```

동일 조건의 plan-only 검증은 2026-08-26에
`trajectories=2 conditions=9 cases=20`으로 PASS했다.

### 완료 판정 및 산출물

다음을 모두 만족해야 20-run 측정 campaign을 PASS로 보고한다.

- `sweep_manifest.tsv`: 20행 모두 `PASS`
- `SWEEP_FINALIZED_VALID` 존재, `SWEEP_FINALIZED_INVALID` 부재
- `sweep_validation.json`: `valid=true`, `case_count=20`, `passed_case_count=20`
- `waypoint_attitude_distance_errors.csv`: `20 × 6 = 120` data rows
- trajectory별 baseline 1개와 non-zero comparison 9개, 전체 comparison 18개
- 모든 case의 `guidance_mode=velocity_only`
- 모든 case의 waypoint hold-analysis index가 `[1,2,3,4,5,6]`
- 모든 case의 operator `progress.json`이 `state=finalized`
- 좌/우 MP4가 양수 frame/duration이고 recorder/ffprobe frame count가 일치
- queue overflow `0`, shutdown drain `SUCCESS`
- 실제 CUDA FP16 backend log 존재

중간 case가 실패해도 `--keep-going`으로 나머지를 수집한다. 실패 artifact를 삭제하거나
같은 output directory에 재실행하지 말고, 원인을 보고한 뒤 새 directory에서 retry한다.
이 20-run 결과를 camera-source 15 Hz 최종 qualification으로 표현하지 않는다.

### 2026-08-26 attempt 1 중단 증거와 재개 조건

- 보존 output:
  `recordings/attitude-20run-20260826_131752`
- manifest:
  - center baseline `FAIL`
  - center roll 0.2° `FAIL`(중단 요청으로 hold 미완료)
  - center roll 0.5° 시작 직후 campaign 중단; manifest row 없음
- baseline 실측:
  - YOLO median `9.842 Hz` (`>=15 Hz` 실패)
  - inference p95 `120.041 ms` (`<=66.7 ms` 실패)
  - left/right unique camera-source `5.224/5.145 Hz` (`>=10 Hz` 실패)
  - waypoint 5 finite nominal/GT sample `0`
- 원인: Simulation 외부의 `wiris_stereo_distance.legacy_gui` PID `396894`가
  multiprocessing GPU worker PID `399790/399792`를 동시 실행했다. 두 worker는
  각 CPU 약 100%와 NVIDIA compute memory를 사용해 Simulation YOLO/Gazebo/recorder와
  경합했다. Workstation 자체는 이전 PASS smoke와 공존했으므로 중단 대상이
  아니며, StereoWIRIS legacy camera/YOLO worker만 중지해야 한다.
- 조치: 무효 20-run을 계속하지 않고 Simulation campaign만 안전하게 중단했다.
  사용자의 Workstation/StereoWIRIS 프로세스는 자동으로 종료하지 않았다.
- retry 전 필수 확인:
  1. StereoWIRIS에서 camera/YOLO stream을 중지하거나 legacy GUI를 종료한다.
  2. `nvidia-smi --query-compute-apps` 결과에 PID `399790/399792` 또는 대체 WIRIS
     inference worker가 없어야 한다.
  3. Simulation/PX4/Gazebo/rqt/RViz 프로세스가 없음을 확인한다.
  4. 기존 attempt 1 directory를 재사용하지 말고 새
     `recordings/attitude-20run-<timestamp>` directory에서 20 case 전체를 처음부터 실행한다.
  5. 첫 baseline 종료 직후 YOLO median/inference p95/camera-source rate/6 waypoint
     sample count를 확인하고 PASS일 때만 나머지 19 case를 계속한다.

### 2026-08-26 attempt 2/3 추가 진단과 재개 조건

attempt 1 후 StereoWIRIS GPU worker를 중지하고 두 번 더 새 directory에서
baseline을 실행했지만, 모두 inference p95 gate 하나만 실패했다.

- attempt 2: `recordings/attitude-20run-20260826_133238`
  - baseline YOLO median `15.367 Hz` PASS
  - inference p95 `97.902 ms` FAIL
  - left/right unique camera-source `10.046/11.038 Hz` PASS
  - waypoint 1~6 finite distance sample 모두 존재
  - StereoWIRIS legacy GUI가 GPU worker는 없었지만 CPU 약 50%를 사용했다.
- attempt 3: `recordings/attitude-20run-20260826_134142`
  - baseline YOLO median `16.803 Hz` PASS
  - inference p95 `98.516 ms` FAIL
  - left/right unique camera-source `12.168/11.758 Hz` PASS
  - waypoint 1~6 finite distance sample 모두 존재
  - StereoWIRIS legacy GUI/worker가 모두 종료된 후에도 inference p95만 실패했다.
- 독립 batch=2 GPU benchmark:
  `recordings/attitude20-preflight-benchmark-20260826_135000.json`
  - accepted `true`
  - pair throughput `159.525 Hz`
  - wall p95 `7.057 ms`
  - CUDA FP16 / RTX 4050 / 동일 model SHA-256

결론: GPU/model 자체가 아니라 Simulation의 Gazebo/PX4/rqt/RViz/recorder와 desktop
백그라운드 앱이 동시 실행될 때의 CPU scheduling/thermal 경합이 남은 원인이다.
실패 run의 90~180초 구간에 inference p95가 `148~158 ms`로 급증했다가
다시 회복했고, CPU package는 실행 중 `95°C`, 종료 30초 후에도 `80°C`였다.
종료 후 주요 백그라운드 부하는 Orca 약 46%, Workstation 약 25%, Firefox 및
content process 합계 약 100% CPU였다.

다음 retry 전에는 다음을 추가로 만족해야 한다.

1. StereoWIRIS legacy GUI/worker는 계속 종료 상태여야 한다.
2. Workstation과 Firefox를 종료한다. 화면 읽기 기능을 사용하지 않는다면 Orca도
   중지한다. 이 프로세스는 사용자 확인 없이 자동 종료하지 않는다.
3. 2026-08-26 사용자 지정에 따라 Simulation 시작 전 CPU package 온도는
   `90°C` 이하를 합격 기준으로 사용하고,
   load average가 충분히 낮아졌는지 확인한다.
4. 독립 benchmark p95 `<= 66.7 ms`를 다시 PASS한 뒤 새 directory에서 시작한다.
5. 첫 baseline이 YOLO median/inference p95/camera-source/6-waypoint sample gate를 모두
   PASS해야 나머지 19 case를 계속한다.

## 2026-08-26 terminal handoff checkpoint

이 checkpoint는 Codex terminal을 종료한 뒤 다음 session에서 같은 20-run을 이어가기
위해 작성했다.

- checkpoint 시간: `2026-08-26T15:02:57+09:00`
- Simulation source HEAD: `c439799`
- `apps/Simulation` source diff: `goal.md` 문서만 modified; Simulation 코드는
  commit `c439799` 그대로이다.
- 기존 Goal thread: `01a0392f-0487-7d60-8312-89399340cb79`
- Goal status: `blocked`(완료 아님). 다음 session에서 사용자가 재개하면 이 문서의
  최신 계약으로 Goal을 다시 시작/재개한다.
- 현재 background 상태:
  - Simulation/PX4/Gazebo/rqt/RViz/waypoint mission 프로세스 없음
  - NVIDIA compute-app 없음
  - StereoWIRIS legacy GUI PID `501423`은 idle 상태이고 GPU worker는 없음
  - CPU package `75°C`; 사용자 최신 시작 기준 `<=90°C` PASS
- 독립 GPU preflight PASS:
  - `recordings/attitude20-preflight-benchmark-20260826_140500.json`
  - accepted `true`, CUDA FP16, batch=2 p95 `9.167 ms`
- 보존된 무효/중단 campaign(삭제/재사용 금지):
  - `recordings/attitude-20run-20260826_131752`
  - `recordings/attitude-20run-20260826_133238`
  - `recordings/attitude-20run-20260826_134142`
  - `recordings/attitude-20run-20260826_141011`
- attempt 4 (`..._141011`)은 baseline 실행 중 StereoWIRIS GPU worker와 별도
  `pytest -q tests`가 동시 생성되어 즉시 중단했다. 합격 case는 없다.

다음 session에 보낼 재개 요청:

> `apps/Simulation/goal.md를 전체 읽고 terminal handoff checkpoint부터 /goal을 새로 생성하거나 재개해서 20-run 실험 데이터를 완성해줘.`

재개 시 최우선 작업:

1. StereoWIRIS legacy GUI가 GPU multiprocessing worker를 다시 생성하지 않는지 30초 이상
   관찰한다.
2. 별도 `pytest -q tests` 또는 다른 CPU/GPU 고부하 task가 없음을 확인한다.
3. CPU package `<=90°C`, CUDA tensor PASS, NVIDIA compute-app 없음을 확인한다.
4. 독립 benchmark p95 `<=66.7 ms`를 다시 확인한다.
5. 기존 attempt directory를 재사용하지 말고 새 timestamp directory에서 baseline을
   실행한다.
6. baseline의 YOLO median `>=15 Hz`, inference p95 `<=66.7 ms`, left/right unique
   camera-source `>=10 Hz`, waypoint 1~6 finite sample, hold/guidance/finalization이 모두
   PASS일 때만 나머지 19 case를 계속한다.

완료 기준은 위 `2026-08-26 20-run 자세오차 측정 실행 조건`의 20/20 PASS,
120-row CSV, 18 comparisons, velocity-only/hold/finalization/CUDA 증거를 그대로 유지한다.

이 문서는 `/home/moontaehyung/바탕화면/SASL-20260821/apps/Simulation`에서
재부팅 직후 새 Goal의 objective로 사용한다.

## 재개 명령

재부팅 후 같은 작업공간에서 Codex에 다음과 같이 요청한다.

> `goal.md를 완전히 읽고, 그 내용을 objective로 Goal을 생성한 뒤 완료될 때까지 실행해줘.`

Goal을 생성하기 전에 아래의 **재부팅 확인**을 먼저 수행한다. Goal 기능에는 예약 실행
기능이 없으므로 재부팅 전에는 새 Goal을 시작하지 않는다.

## 재부팅 확인

이 문서를 작성할 때의 고장 난 부팅 상태는 다음과 같다.

- boot ID: `cf7f65ce-656a-4a9c-882b-05506b0333ed`
- boot 시작: `2026-08-24 20:46:06`
- 커널 오류: NVIDIA `Xid 154`, `Node Reboot Required`
- 증상: `cuInit` 오류 999, `torch.cuda.is_available() == False`

재개 시 다음을 확인한다.

1. `/proc/sys/kernel/random/boot_id`가 위 ID와 달라야 한다.
2. 현재 boot의 커널 로그에 새로운 Xid 31/154 또는 UVM fatal 오류가 없어야 한다.
3. `.venv/bin/python`에서 `torch.cuda.is_available()`가 `True`여야 한다.
4. `torch.zeros(1, device="cuda:0")`가 성공하고 RTX 4050 이름이 출력돼야 한다.

boot ID가 그대로이거나 CUDA tensor가 실패하면 시뮬레이션을 시작하지 말고 재부팅이
필요하다는 사실을 보고한다.

## 최종 objective

먼저 YOLO GPU/runtime 문제를 해결하여 실제 YOLO 처리율과 좌·우 overlay MP4를 지속
15 Hz 이상으로 만든다. 그 결과가 실측 PASS한 뒤에만, 카메라 clip 밖 staging 없이
고정 pitch 20도 경로에서 드론의 큰 수직 화면 이동을 제거한다. 두 단계의 모든 합격
조건을 center와 zigzag GUI smoke에서 동시에 입증할 때까지 계속한다.

기존 accepted 결과와 사용자 변경을 보존한다. commit, push, 기존 결과 삭제를 하지
않는다. 작업 범위는 가능한 한 `apps/Simulation` 안으로 제한한다.

## 반드시 보존할 기존 결과

- archive:
  `recordings/accepted-20-runs-20260825_063251.tar.zst`
- archive SHA-256:
  `a68e0b614351028d2f615e54e8142301912422c2c978f0130041076cd613540f`
- 원본 target SDF SHA-256:
  `7bee1569ed5c73cda938c66b022c11c0d17da3ccdf0ec89533ea90c1b11ea180`
- 보호 geometry SHA-256:
  `21cc2d3987ce38aaa751c010b61ebb95ac9ce2110bb254bad97d789dc7764f71`

작업트리는 이미 dirty이며 다른 앱의 변경은 사용자 소유다. 관련 없는 파일을 되돌리거나
정리하지 않는다.

## 현재까지 준비된 YOLO 변경

다음 구현은 이미 작업트리에 있으므로 먼저 현재 내용을 검토하고 재사용한다.

- `.venv`는 system/user Python과 분리되어 있고
  `include-system-site-packages = false`다.
- 설치 조합:
  - Python 3.10.12
  - torch `2.12.0+cu132`
  - torchvision `0.27.0+cu132`
  - Ultralytics `8.4.127`
  - OpenCV `4.9.0`
- `scripts/ensure_yolo_runtime.sh`
  - 정확한 CUDA wheel/version 검사
  - 실제 CUDA tensor 검사
  - CPU/system-Python silent fallback 차단
  - stdlib `venv`가 없을 때 설치된 `virtualenv` fallback
- `scripts/benchmark_yolo_pair.py`
  - 실제 좌·우 MP4 프레임을 batch=2로 넣는 독립 benchmark
  - 실제 CUDA FP16 backend, throughput, wall-time p95 JSON 증거
- `run_operator_view.sh`
  - 격리 runtime만 사용
  - `--overlay-hz` 지원
  - runtime 준비 실패 시 fail-closed
- `run_auto_attitude_sweep.sh`
  - camera/YOLO/overlay rate를 각각 전달
  - `--baseline-only` 지원
  - 새 run에 performance acceptance contract 기록
- `operator_recorder.py`
  - MP4 callback wall rate와 고유 원본 camera timestamp rate 기록
- `validate_attitude_sweep_output.py`
  - warmup 이후 YOLO median/p95
  - recorder callback rate
  - 고유 camera source rate
  - ffprobe frame count/actual average FPS
  - 실제 CUDA FP16 backend log
  - queue overflow와 shutdown drain 판정

관련 focused tests는 마지막 확인에서 68개가 통과했다. 전체 suite에서는 처음
`1431 passed, 98 skipped, 3 failed`였고, 3개 실패의 원인이 runtime 성공 보고의 잘못된
소유 경계임을 수정한 후 해당 실패 3개를 포함한 focused tests 23개가 통과했다. 전체
suite는 최종 단계에서 반드시 다시 실행한다.

## 1단계: YOLO GPU와 15 Hz 실측

### 1. 격리 runtime 검증

시스템/user Python 패키지를 제거하거나 덮어쓰지 않는다. 먼저 다음과 동등한 검증을
수행한다.

```bash
env \
  YOLO_TORCH_BACKEND=cuda \
  YOLO_TORCH_CUDA_INDEX_URL=https://download.pytorch.org/whl/cu132 \
  YOLO_TORCH_VERSION=2.12.0 \
  YOLO_TORCHVISION_VERSION=0.27.0 \
  YOLO_TORCHAUDIO_VERSION= \
  YOLO_ULTRALYTICS_VERSION=8.4.127 \
  YOLO_VENV_ISOLATED=true \
  bash scripts/ensure_yolo_runtime.sh \
    --venv-dir "$PWD/.venv" \
    --preferred-python /usr/bin/python3 \
    --isolated
```

성공 조건은 version 문자열뿐 아니라 실제 CUDA tensor 생성이다. torchaudio는 YOLO가
사용하지 않고 공식 cu132 인덱스에 matching wheel이 없으므로 설치하지 않는다.

### 2. 실제 stereo pair benchmark

모델과 기존 성공 run의 실제 MP4를 사용한다.

```bash
MODEL=/home/moontaehyung/바탕화면/SASL-20260821/.local/legacy/Simulation/yolov8n_uav.pt
SOURCE=recordings/goal-attitude-sweep-20260825_025133/case-center-001_Lr0p20y0_Rr0p20y0_repetition00/operator-recording
OUT=recordings/goal-yolo-vertical-$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUT"

.venv/bin/python scripts/benchmark_yolo_pair.py \
  --model "$MODEL" \
  --left-video "$SOURCE/left_overlay.mp4" \
  --right-video "$SOURCE/right_overlay.mp4" \
  --output "$OUT/yolo_pair_benchmark.json" \
  --device cuda:0 \
  --imgsz 736 \
  --confidence 0.10 \
  --class-name UAV \
  --warmup-iterations 10 \
  --iterations 60 \
  --minimum-pair-update-hz 15 \
  --maximum-p95-ms 66.7
```

`accepted=true`, 실제 `backend_device=cuda:*`, `backend_precision=fp16`, p95 66.7 ms
이하가 필요하다. 실패할 때만 CUDA와 호환되는 TensorRT를 조사하고 batch=2 FP16
engine을 사용한다. imgsz 736, 모델, confidence 0.10은 검출 증거 없이 낮추지 않는다.

### 3. GUI 단계 검증

GUI/RViz/rqt를 보이는 상태로 center baseline을 5 Hz, 10 Hz, 마지막 15 Hz 이상 순서로
검증한다. 단계별 output directory는 서로 다르게 만든다. 진단 단계의 최소 실측률은
각 target의 약 90%로 두고, 최종 단계는 camera/YOLO/overlay target을 20 Hz로 두어
실측 15 Hz gate를 판정한다.

예시:

```bash
# 5 Hz 진단
SWEEP_MIN_YOLO_UPDATE_HZ=4.5 \
SWEEP_MIN_OVERLAY_HZ=4.5 \
SWEEP_MAX_YOLO_INFERENCE_P95_MS=200 \
./run_auto_attitude_sweep.sh \
  --baseline-only --trajectories center --ordered \
  --camera-hz 5 --yolo-hz 5 --overlay-hz 5 \
  --output-dir "recordings/goal-yolo-stage-05-$(date +%Y%m%d_%H%M%S)"

# 10 Hz 진단
SWEEP_MIN_YOLO_UPDATE_HZ=9 \
SWEEP_MIN_OVERLAY_HZ=9 \
SWEEP_MAX_YOLO_INFERENCE_P95_MS=100 \
./run_auto_attitude_sweep.sh \
  --baseline-only --trajectories center --ordered \
  --camera-hz 10 --yolo-hz 10 --overlay-hz 10 \
  --output-dir "recordings/goal-yolo-stage-10-$(date +%Y%m%d_%H%M%S)"

# 최종 15 Hz 이상 gate: 20 Hz로 요청하고 실측 최소 15 Hz 판정
./run_auto_attitude_sweep.sh \
  --baseline-only --trajectories center --ordered \
  --camera-hz 20 --yolo-hz 20 --overlay-hz 20 \
  --output-dir "recordings/goal-yolo-stage-final-$(date +%Y%m%d_%H%M%S)"
```

각 단계에서 실패 지점을 GPU inference, scheduler, camera input, overlay publisher,
MP4 writer로 분리한다. 최종 YOLO PASS 조건은 모두 충족해야 한다.

- warmup 제외 `yolo_update_hz` median >= 15 Hz
- full pair inference p95 <= 66.7 ms
- 좌·우 고유 camera-source observed rate >= 15 Hz
- 좌·우 overlay callback observed rate >= 15 Hz
- 좌·우 MP4 ffprobe actual average rate >= 15 Hz
- MP4 positive duration, 정상 video stream, recorder/ffprobe frame count 일치
- `measurement_queue_overflow_count == 0`
- `shutdown_drain_status == SUCCESS`
- operator log가 실제 CUDA FP16 backend를 증명

이 단계가 PASS하기 전에는 수직 경로 문제를 수정하거나 합격으로 간주하지 않는다.

## 2단계: 큰 수직 화면 이동 제거

### 고정 계약

- 모델: `x500_2m`
- 전진 속도: 2 m/s
- 카메라 baseline/FOV 유지
- 좌·우 기본 optical attitude: roll 0°, pitch 20°, yaw 0°
- gimbal 사용 금지
- camera clip 밖 이륙/staging 금지
- 초기 이륙이 화면에 보이는 것은 허용
- 합격 분석 구간: 첫 필수 waypoint settle 이후부터 마지막 양안 가시 waypoint까지
- 필수 Front: `415,365,315,265,215,165,115,65` m
- 각 필수 waypoint에서 5초 hold, 총 8개
- zigzag Right는 기존 40% stereo-overlap 규칙 유지

center Up은 임의 반올림값이 아니라 `U = F * tan(20°)`로 생성한다.

```text
R,F,U
0,415,151.048
0,365,132.849
0,315,114.651
0,265,96.452
0,215,78.254
0,165,60.055
0,115,41.857
0,65,23.658
```

### RFU → PX4 고도 변환 수정

현재 generator/mission 경계가 configured spawn world Z=6 m를 PX4 local home으로
가정하여 실제 world altitude가 약 5 m 낮아지는 문제가 있다. 과거 smoke에서 관찰한
고정 offset을 하드코딩하지 않는다.

1. PX4 연결 직후 안정된 local NED 위치와 Gazebo world pose를 같은 bootstrap 구간에서
   수집한다.
2. 실제 `world_up = home_world_up - local_down` 관계를 계산한다.
3. 요청한 world RFU Up을 해당 run의 PX4 local Down setpoint로 변환한다.
4. 변환 source, bootstrap timestamp/sample count, home world Up을 event/evidence에 기록한다.
5. 각 필수 waypoint의 실제 Gazebo world Up과 요청 RFU Up 오차 p95가 1 m 이하여야 한다.

필요하면 8개 필수 hold 사이에 무정지 pass-through waypoint와 속도 ramp를 추가할 수
있다. 필수 Front/Up과 hold는 바꾸지 않는다. 비행이 불안정하면 크기 geometry를 제외한
target SDF 동역학 값을 조정할 수 있지만, 변경 전후 SDF/protected geometry hash와 이유를
기록한다.

### 수직 화면 이동 validator

실측 artifact에서 다음을 계산하고 validator/report에 저장한다.

- 각 필수 hold의 좌·우 bbox center-y median
- 첫 hold부터 마지막 양안 가시 hold까지 hold median peak-to-peak
- 20° boresight 예상 경로에 대한 bbox 세로 residual p95
- 요청 world Up 대비 실제 world Up 오차 p95
- 기존 waypoint position-error p95
- 최대 절대 roll/pitch

center와 zigzag 모두 다음을 만족해야 한다.

- dual bbox `cy` hold-median peak-to-peak <= 40 px
- 세로 경로 residual p95 <= 40 px
- world Up 오차 p95 <= 1 m
- waypoint position error p95 <= 1 m
- `abs(roll) <= 15°`, `abs(pitch) <= 15°`
- 동시에 YOLO/overlay/MP4 최종 15 Hz gate PASS

최종 smoke는 다음과 같이 baseline 두 run만 실행할 수 있다.

```bash
./run_auto_attitude_sweep.sh \
  --baseline-only --trajectories center,zigzag --ordered \
  --camera-hz 20 --yolo-hz 20 --overlay-hz 20 \
  --output-dir "recordings/goal-final-center-zigzag-$(date +%Y%m%d_%H%M%S)"
```

## 구현 및 검증 규칙

- 실제 작업트리를 source of truth로 다시 검사한다.
- 기존 accepted archive와 unrelated dirty 변경을 보존한다.
- 파일 수정은 작은 slice로 나누고 focused test를 바로 실행한다.
- shell syntax와 Python compile 검사를 수행한다.
- 마지막에 `/usr/bin/python3 -m pytest -q` 전체 suite를 실행한다.
- README는 이미 사용자가 `README.md`로 확정했다.
- 최종 동작이 확정되면 다음 문서를 모두 갱신한다.
  - `README.md`
  - `docs/RQT_OPERATOR_GUIDE.md`
- commit/push는 하지 않는다.

## 완료 보고에 포함할 증거

- `.venv` Python/torch/torchvision/Ultralytics/CUDA 버전
- 실제 GPU 이름과 PyTorch 또는 TensorRT backend/precision
- benchmark JSON 경로와 pair throughput/inference p95
- 5/10/final GUI stage artifact 경로
- final center/zigzag artifact 경로
- center/zigzag 각각의 YOLO median, inference p95
- 좌·우 camera unique rate, overlay callback rate, MP4 ffprobe FPS/duration/frame count
- queue overflow count와 shutdown drain 상태
- PX4 home/world bootstrap 값
- 8개 요청/실제 world Up과 p95 오차
- bbox vertical peak-to-peak와 residual p95
- position/attitude stability 결과
- focused tests 및 전체 pytest 결과

모든 항목이 실제 artifact로 입증될 때만 Goal을 `complete`로 표시한다.
