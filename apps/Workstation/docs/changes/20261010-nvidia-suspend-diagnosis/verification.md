# NVIDIA 절전 복귀 / CPU fallback 조사 — 2026-10-10

범위: 읽기 전용 호스트 진단. 앱, 드라이버, 절전 설정을 변경하지 않았으며 절전/재부팅을 실행하지 않았다.
방법: 동일 boot ID의 kernel journal monotonic timestamp로 resume와 Xid 154를 대조하고,
실제 module parameter를 실행 중인 드라이버 버전의 NVIDIA 공개 소스와 비교했다.
목적: 현재 업데이트 오류와 반복되는 절전 복귀 오류를 분리한다. 좌표/측정 ground truth는 입력이 아니다.
출력: 확인된 사실, 원인 가설, 미실행 검증을 구분한다. 해결 판정은 설정 적용 후 실제 재현 시험 통과를 요구한다.

Coordinate contract checked: 진단 문서만 추가. NED/ENU/body FRD/optical RDF,
stereo-center/left/right-camera, GPS/ULog 좌표 및 배열 축, CSV, UI 계약 변경 없음.

## S1 / 현재 드라이버 일치 여부 — FAIL (원인 확인)

- Source of truth: `/proc/driver/nvidia/version`, `modinfo -F version nvidia`, host `nvidia-smi`, apt history.
- Hardware evidence: 로드된 모듈 595.91.07, 설치된 모듈/라이브러리 595.99.02.
- `nvidia-smi`는 샌드박스 밖에서도 `Driver/library version mismatch`를 반환.
- apt history: 10월 10일 06:15:16~06:17:29, `/usr/bin/unattended-upgrade`가 NVIDIA 595.91.07을 595.99.02로 업데이트.
- 독립 증거: kernel API mismatch 로그와 apt 패키지 이력이 일치한다.
- 현재 부팅에서 조사 시점까지 Xid 31/154는 발견되지 않았다. 현재 실패를 과거 fatal resume와 동일시하지 않는다.
- 다음: 재부팅 후 로드/설치 버전 일치 확인. 지금 CUDA 실패로 절전 설정 효과를 시험할 수 없다.

## S2 / 반복되는 절전 복귀 오류 — FAIL (반복 장애 확인)

- Source of truth: 보존된 전체 kernel journal (`journalctl _TRANSPORT=kernel --no-pager -o json`).
- `_BOOT_ID`별 가장 최근 `PM: suspend exit`와 Xid 154의 `__MONOTONIC_TIMESTAMP` 차이를 계산.
- 13건 모두 resume 후 0.885~3.163초 안에 `Node Reboot Required` 발생.
- 초 단위 차이: 3.162979, 1.918001, 1.608047, 1.950920, 2.270868, 1.604967,
  0.885003, 1.807052, 1.561703, 2.043177, 1.609128, 1.898894, 1.695975.
- 독립 경계 증거: Xid 31 `FAULT_PDE ACCESS_TYPE_VIRT_READ`, UVM global fatal error 0x60,
  Xid 154가 앱 로그가 아닌 kernel에서 기록됨. 다수 첫 fault의 process name은 `modprobe`.
- 최근 사례: 10월 7일 10:03:56 resume, 10:03:57 NVIDIA resume service 성공,
  10:03:58 modprobe Xid 31 / UVM fatal / Xid 154. 당시 로드 버전은 595.91.07.
- 벽시계/boot 목록에 시각 불일치가 있어 인과 순서는 같은 boot의 monotonic 시간을 기준으로 판단했다.
- 한계: 보존된 로그 범위의 상관관계다. 모든 절전이 실패하지 않으며 앱 비관여나 하드웨어 결함 부재를 증명하지 않는다.

## S3 / 절전 메모리 보존 설정 — FAIL (유력한 설정 원인)

- Source of truth: 현재 `/proc/driver/nvidia/params`, `/sys/power/mem_sleep`, systemd units,
  실행 버전 595.91.07 공개 소스. 현재 값이 과거 모든 실패 당시에도 같았는지는 미확인.
- 실제 값: `PreserveVideoMemoryAllocations: 2`, `UseKernelSuspendNotifiers: 0`,
  `EnableS0ixPowerManagement: 0`; sleep mode `[s2idle] deep`.
- [osinit.c](https://github.com/NVIDIA/open-gpu-kernel-modules/blob/595.91.07/src/nvidia/arch/nvalloc/unix/src/osinit.c#L947):
  mode 2는 `preserve_vidmem_allocations = os_supports_kernel_suspend_notifiers()`로 결정.
- [os-interface.c](https://github.com/NVIDIA/open-gpu-kernel-modules/blob/595.91.07/kernel-open/nvidia/os-interface.c#L2808):
  그 함수는 `NVreg_UseKernelSuspendNotifiers == 1`을 반환한다. 따라서 현재 조합은 전체 VRAM 보존을 활성화하지 않는다.
- 독립 oracle: [NVIDIA power-management 설명](https://download.nvidia.com/XFree86/Linux-x86_64/595.91.07/README/powermanagement.html).
  전체 메모리 보존 부재가 복귀 후 오류를 만들 수 있다고 설명한다.
- nvidia-suspend/resume/hibernate 모두 enabled. 조사한 resume 로그는 서비스 완료 성공.
  서비스 활성화만으로 전체 메모리 보존이 활성화되는 것은 아니다.
- `/tmp` backing filesystem은 ext4, 여유 약 58 GiB. 현재 공간 부족 증거는 없다.
- 추론: 절전 중 GPU 메모리/페이지 테이블 보존·복원이 깨지는 경로가 최우선 원인 후보.
  실제 메모리 손실을 직접 관측하거나 설정 변경으로 인과관계를 확정한 것은 아니다.
- 다른 후보: s2idle 복귀 타이밍. NVIDIA 문서에 deep 우회가 있으나 이 장비에서 효과는 미검증.

## S4 / 교정 및 절전 재현 — NOT_RUN

아래는 제안이며 시스템에 적용하지 않았다. 현재 드라이버 불일치가 먼저 해소돼야 한다.

1. 새 드라이버로 재부팅하고 NVML, CUDA 초기화, 실제 NVDEC 프레임 출력을 확인한다.
2. 재부팅 후 parameter를 다시 읽어 동일 조합이 유지되는지 확인한다.
3. 유지된다면 기존 systemd 절전 hook 경로를 유지하며 `NVreg_PreserveVideoMemoryAllocations=1`을
   명시하는 설정을 첫 단일 변경 후보로 삼는다. 설정/부팅 반영과 복귀 검증이 필요하다.
4. s2idle에서 앱 종료 상태와 양쪽 Preview 실행 상태를 각각 3회 이상 시험한다.
   매회 NVML, `cuInit(0)`, 양쪽 실제 `nvh264dec` 프레임, 신규 Xid/UVM 오류를 기록한다.
5. Pass: 절전 전후 동일한 드라이버 버전, CUDA/NVDEC 정상, 새 Xid/UVM fatal과 CPU fallback 없음.
   Fail: 한 번이라도 새 fatal 오류 또는 GPU context 생성 실패. 3회 통과는 장기 신뢰성 보증은 아니다.
6. 보존 설정 후에도 재발하면 `deep`만 별도로 바꿔 같은 시험을 한다. 두 설정을 동시에 바꾸지 않는다.

추가 운영 문제: 자동 업데이트가 작업 중 NVIDIA 사용자 공간과 로드 모듈 버전을 분리시켰다.
드라이버 업데이트를 작업 종료 및 계획 재부팅과 묶는 운영 정책을 검토하되, 보안 업데이트 전체를 끄지 않는다.

코드/시스템 설정 변경이 없으므로 앱 회귀 테스트는 실행하지 않았다. 조사 완료와 장애 해결 완료는 구분한다.
