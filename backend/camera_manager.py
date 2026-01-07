import cv2
import time
import os
import threading
from firebase_admin import storage

class TimeLapseRecorder:
    def __init__(self, output_dir="./videos"):
        self.output_dir = output_dir
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        self.is_recording = False
        self.cap = None
        self.out = None
        self.thread = None
        self.frame_interval = 0.2  # 0.2초마다 1프레임

    def start(self, order_id, camera_index=0):
        """녹화 시작"""
        if self.is_recording: return

        # ✅ [변경 1] 확장자를 .webm으로 변경 (웹 친화적)
        self.filename = f"{self.output_dir}/cake_{order_id}.webm"
        
        print(f"📷 [Camera] {camera_index}번 카메라 연결 시도 중...")
        self.cap = cv2.VideoCapture(camera_index)
        
        if not self.cap.isOpened():
            print(f"❌ [Error] {camera_index}번 카메라를 열 수 없습니다!")
            return

        # ✅ [변경 2] 리눅스 호환성이 좋고 웹 재생이 잘 되는 VP80 코덱 사용
        fourcc = cv2.VideoWriter_fourcc(*'VP80')
        
        self.out = cv2.VideoWriter(self.filename, fourcc, 20.0, (640, 480))
        
        if not self.out.isOpened():
            print("❌ [Critical] 비디오 작성기 초기화 실패! (코덱 문제)")
            self.cap.release()
            return

        self.is_recording = True
        print(f"🎥 [Camera] 녹화 시작 (주문 #{order_id}, 카메라 #{camera_index})")

        self.thread = threading.Thread(target=self._record_loop)
        self.thread.start()

    def _record_loop(self):
        last_time = 0
        while self.is_recording and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret: break
            
            current_time = time.time()
            if current_time - last_time > self.frame_interval:
                frame = cv2.resize(frame, (640, 480))
                self.out.write(frame)
                last_time = current_time
            
            time.sleep(0.01)

    def stop_and_upload(self, order_id):
        if not self.is_recording: return None

        print("🎥 [Camera] 녹화 종료... 저장 중...")
        self.is_recording = False
        if self.thread: self.thread.join()
        
        if self.cap: self.cap.release()
        if self.out: self.out.release()
        
        return self._upload_to_firebase(order_id, self.filename)

    def _upload_to_firebase(self, order_id, local_path):
        try:
            # 파일이 진짜 생성됐는지 확인
            if not os.path.exists(local_path):
                print(f"❌ [Error] 파일이 생성되지 않았습니다: {local_path}")
                return None

            print(f"☁️ [Storage] 업로드 시작: {local_path}")
            bucket = storage.bucket() 
            
            # ✅ [변경 3] 저장될 파일명도 .webm으로 변경
            blob = bucket.blob(f"timelapse/cake_{order_id}.webm")
            
            # ✅ [변경 4] 메타데이터도 webm으로 설정
            blob.content_type = 'video/webm'
            
            blob.upload_from_filename(local_path)
            blob.make_public()
            print(f"✅ [Storage] 업로드 완료: {blob.public_url}")
            return blob.public_url
        except Exception as e:
            print(f"❌ [Storage Error] {e}")
            return None