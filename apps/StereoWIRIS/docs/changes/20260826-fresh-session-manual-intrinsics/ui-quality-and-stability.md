# UI Quality and Stability

시각 배치는 바꾸지 않는다. startup event-loop에서 두 자동 작업을 제거하므로 dialog/focus 이동은
추가되지 않는다. 빈 session 상태와 not-loaded intrinsics 상태가 기존 label contract로 표시되는지
offscreen Qt와 실제 재시작 screenshot으로 확인한다.
