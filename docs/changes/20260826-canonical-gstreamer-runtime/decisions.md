# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-08-26 — Approved runtime contract

- 저장소 루트 `.venv`만 공식 운영 interpreter이다.
- 공식 생성 명령은 `python3 -m venv --system-site-packages .venv`이며 수동 `.pth`는 미지원이다.
- GStreamer/PyGObject는 Ubuntu system dependency이고 앱 실행 중 설치하지 않는다.
- `avdec_h264`는 필수, NVIDIA는 optional acceleration이다.
- Auto만 software degradation을 허용하고 명시적 NVIDIA는 fail-closed한다.
- 요청에 포함된 구현 계획이 이 packet의 명시적 승인이다.

## 2026-08-26 — NVIDIA runtime health remains observable

Explicit NVIDIA mode keeps the existing first-frame/stall health deadlines, but a failure emits
an Auto recovery instruction and never changes the decoder. Auto alone converts the same health
failure into the existing MP4-safe software transition. Existing RTSP reconnect ownership is not
changed.
