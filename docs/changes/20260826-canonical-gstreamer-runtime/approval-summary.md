# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

두 운영 앱은 루트 공식 venv와 검증된 GStreamer runtime만 사용한다. 카메라/측정/저장/GNSS/
MAVLink 데이터 계약과 UI 배치는 유지한다.

## Operator workflow before and after

여러 Python fallback과 launch-time 설치 대신 manifest대로 수동 설치·공식 venv 생성·doctor
확인 후 실행한다.

## Success, delay, rejection, failure, overload, and recovery

Auto는 NVIDIA가 없으면 software로 정상 운용하고 경고한다. NVIDIA 선택은 capability가 없으면
즉시 거부한다. Auto의 실행 중 NVIDIA 장애만 안전한 MP4 종료 후 software로 전환한다.

## Data and compatibility effects

settings 값과 저장 파일은 그대로다. 앱별/Simulation venv를 삭제하지 않지만 선택하지 않는다.

## Performance priorities and allowed degradation

worker/queue/rate는 유지한다. Auto에서만 GPU decode 불가 시 CPU decode degradation이 허용된다.

## Visual and interaction changes, including dynamic-layout stability

새 UI는 없고 기존 status/log 문구만 명확해진다. geometry/focus/scroll은 유지한다.

## How acceptance will be demonstrated

doctor, launcher/direct-entry, capability/decoder, camera rejection/fallback, lifecycle/recording
regression tests와 실제 좌우 RTSP 수동 절차로 검증한다.

## Cost, risks, assumptions, rollback, and field limits

Ubuntu 22.04/Python 3.10/ROS Humble/NVIDIA 대상이다. 현재 PC는 `python3.10-venv`가 없어
deployment doctor가 실패해야 하며 실제 hardware decode는 자동 test로 증명하지 않는다.

## Exact scope awaiting approval

Approved scope: `canonical-gstreamer-runtime` plan supplied by the user. Implementation may proceed.
