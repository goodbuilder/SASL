# Workstation 시험 환경 정리 결과

Python 의존성 문제는 해결했다. NVIDIA NVENC는 설치된 드라이버 활성화를 위한 재부팅과 후속 실인코딩 검증이 남아 있다. 전체 작업 완료로 표시하지 않는다.

| 항목 | 현재 결과 |
|---|---|
| NumPy/SciPy 호환 경고 | 프로젝트 .venv에서 NumPy1.24.4 + SciPy1.8.0, 경고를 오류로 취급한 import/선형대수 smoke 통과 |
| pyproj 부재 | pyproj3.7.1 + rasterio1.4.4 설치, 지형·공통좌표25개 통과 |
| 의존성 완전성 | 누락 cffi 보완, pip check: No broken requirements found |
| 전체 시험 | 2268 passed,1 skipped,0 warnings,234.56s,exit0 |
| NVENC | 실제 FFmpeg 초기화 실패, 재부팅 후 검증 필요 |

기존 missing-terrain-dependency 시험은 설치 여부 때문에 건너뛰는 대신 rasterio/pyproj 부재를 각각 주입해 검증한다. 그래서 이전2268개 case에서2269개 case로 늘었다. NVENC 시험은 실제 녹화와 같은160×64로 probe하며, 실패를 숨기거나 소프트웨어 인코더로 대체하지 않는다.

## 설치와 재현

Ubuntu22.04/Python3.10, repository root에서:

```bash
tools/setup_workstation_test_env.sh
apps/Workstation/tools/run_isolated_tests.sh -q -ra --tb=short
```

설치는 저장소의 .venv에 한정했다. .venv는 의도적으로 Git에서 무시되며 재생성한다. 필수 파일은 requirements/optional-terrain.txt, requirements/constraints-workstation-py310.txt, requirements/dev.txt와 setup script다. 전역 사용자 Python/ROS 패키지를 시험에 혼합하지 않도록 runner에서 user-site/PYTHONPATH를 차단한다. 누락된 가상환경을 시스템 Python으로 자동 대체하지 않는다. setup 재실행 전후 local package freeze hash는 동일했다.

README 변경: apps/Workstation/README.md의 Reproducible validation environment에 설치·실행·GPU 검증 절차를 추가했다. 현재 ARCHITECTURE의 runner 설명도 갱신했다. 앱의 좌표 변환식과 측정/기록 코드는 변경하지 않았다.

## GPU에서 남은 작업

호스트 GPU는 RTX4050 Laptop GPU이다. 현재 로드된 NVIDIA kernel module은595.91.07이고 user-space libraries와 설치된 DKMS module은595.99.02이다. 현재 kernel6.8.0-138-generic에 대응하는 새 모듈이 설치돼 있고 initramfs도 새로 생성돼 있다. SecureBoot는 비활성 상태다. 실제 NVENC 실패는 호스트와 네트워크 격리 환경 모두에서 재현했다.

[Ubuntu 공식 안내](https://ubuntu.com/server/docs/how-to/graphics/install-nvidia-drivers/#driver-library-version-mismatch-error)는 이 업데이트 후 버전 불일치를 재부팅으로 맞추도록 설명한다. 그래픽 모듈은 현재 데스크톱이 사용하고 있으므로 작업 중 강제 unload하지 않았다. 재부팅은 앱과 실행 중 작업을 종료하므로 사용자의 명시적인 진행 확인이 필요하다.

재부팅 후 repository root에서:

```bash
python3 tools/check_workstation_nvenc.py
```

이 명령은 nvidia-smi를 확인하고 기존 실제 NVENC lossless-frame/MP4 finalization 시험을 격리 runner로 실행한다. pytest가 skipped로 끝나도 실패를 반환하므로 GPU 정상화 판정을 대체할 수 없다. 성공 결과가 나오기 전에는 NVENC가 해결됐다고 주장하지 않는다.

이 환경 정리는 모델 가중치 provisioning이나 실장비 카메라 연결 시험을 포함하지 않는다. 기존 미커밋 앱/Simulation 변경은 보존했다. 상세 로그와 package/source inventory는 evidence/에 보존한다.

Coordinate contract checked: terrain WGS84 lon/lat/ellipsoid-height→ECEF→NED[north,east,down],25 coordinate tests; 변환식 변경 없음.
