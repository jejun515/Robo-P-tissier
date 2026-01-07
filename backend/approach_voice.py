import time
import cv2
import pygame

# =========================
# 설정값 (여기만 만져도 됨)
# =========================
CAM_INDEX = 0
AUDIO_PATH = "welcome.wav"
FACE_WIDTH_TRIGGER = 300       # 실행 중 +/- 로 조절
RESET_MARGIN = 100             # 가까움 상태 해제(리셋) 마진(히스테리시스)
HOLD_OFF_SEC = 2.0             # 진입 트리거 후 최소 재생 간격(안전용)
SHOW_DEBUG = True

CASCADE_PATH = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"

def main():
    global FACE_WIDTH_TRIGGER

    # 오디오 초기화
    pygame.mixer.init()
    sound = pygame.mixer.Sound(AUDIO_PATH)

    # 카메라 열기
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"카메라를 열 수 없습니다. CAM_INDEX={CAM_INDEX} 값을 바꿔보세요.")

    # 얼굴 검출기 로드
    face_cascade = cv2.CascadeClassifier(CASCADE_PATH)
    if face_cascade.empty():
        raise RuntimeError("Haar Cascade 로드 실패 (OpenCV 설치 확인).")

    last_play_time = 0.0
    is_near = False  # ✅ '가까움 상태' 플래그(진입할 때만 1회 재생)

    print("[INFO] 시작! 창에서 q를 누르면 종료됩니다.")
    print("[INFO] +/- 키로 FACE_WIDTH_TRIGGER 값을 실시간 조절할 수 있게 해놨어요.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] 프레임을 읽지 못했습니다.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(60, 60)
        )

        target = None
        if len(faces) > 0:
            target = max(faces, key=lambda r: r[2] * r[3])  # (x,y,w,h)

        now = time.time()
        triggered_this_frame = False

        if target is not None:
            x, y, w, h = target

            # ✅ 현재 프레임에서 "가까움" 판정
            near_now = (w >= FACE_WIDTH_TRIGGER)

            # ✅ 멀리->가까움으로 "진입"하는 순간에만 1회 재생
            if (not is_near) and near_now and (now - last_play_time) >= HOLD_OFF_SEC:
                sound.play()
                last_play_time = now
                is_near = True
                triggered_this_frame = True

            # ✅ 가까운 상태에서 충분히 멀어지면 리셋(히스테리시스)
            #    (경계에서 흔들리며 연속 재생되는 걸 방지)
            if is_near and (w <= FACE_WIDTH_TRIGGER - RESET_MARGIN):
                is_near = False

            if SHOW_DEBUG:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"face_w={w}px / trig={FACE_WIDTH_TRIGGER}px",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(frame, f"state={'NEAR' if is_near else 'FAR'} reset@<{FACE_WIDTH_TRIGGER-RESET_MARGIN}px",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                if triggered_this_frame:
                    cv2.putText(frame, "PLAY!", (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        else:
            # 얼굴이 사라지면 가까움 상태 해제(다시 접근 시 재생되게)
            is_near = False
            if SHOW_DEBUG:
                cv2.putText(frame, "No face", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(frame, f"trig={FACE_WIDTH_TRIGGER}px",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("Approach Voice (q=quit, +/-=threshold)", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key in (ord('+'), ord('=')):
            FACE_WIDTH_TRIGGER += 10
            print(f"[TUNE] FACE_WIDTH_TRIGGER = {FACE_WIDTH_TRIGGER}")
        elif key in (ord('-'), ord('_')):
            FACE_WIDTH_TRIGGER = max(60, FACE_WIDTH_TRIGGER - 10)
            print(f"[TUNE] FACE_WIDTH_TRIGGER = {FACE_WIDTH_TRIGGER}")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
