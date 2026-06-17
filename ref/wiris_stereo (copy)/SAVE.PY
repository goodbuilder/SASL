import cv2

# ---- 1) 스트림 주소 설정 ----
stream_url = "rtsp://10.0.0.232:8554/live"


# ---- 2) 스트림 받아오기 ----
cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)

if not cap.isOpened():
    print("Stream open failed")
    exit()

# ---- 3) 프레임 크기 / FPS 자동 읽기 ----
width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps    = cap.get(cv2.CAP_PROP_FPS)

print("Stream info:", width, height, fps)

# ---- 4) VideoWriter 준비 ----
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
writer = cv2.VideoWriter("stream_saved.mp4", fourcc, fps, (width, height))

# ---- 5) 저장 루프 ----
while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame read failed")
        break

    writer.write(frame)
    cv2.imshow("stream", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ---- 6) 종료 ----
cap.release()
writer.release()
cv2.destroyAllWindows()
