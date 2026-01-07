import os
import time
import uuid
import shutil
import base64
import numpy as np
import cv2
import firebase_admin
from firebase_admin import credentials, db, storage
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
import uvicorn
import requests
import urllib.parse # URL 인코딩용
import io
from openai import OpenAI
from fastapi.responses import StreamingResponse
from fastapi.responses import JSONResponse
import json
from fastapi import FastAPI, UploadFile, File, Form, Body, HTTPException
import hmac
import hashlib
import time
import uuid
import requests
import json
from apscheduler.schedulers.background import BackgroundScheduler
import cv_utils
from typing import List, Optional, Union
import threading # 스레드 사용을 위해 필수
import pygame    # 오디오 재생용\
from datetime import datetime, timedelta
from camera_manager import TimeLapseRecorder

OPENAI_API_KEY = ""
client = OpenAI(api_key=OPENAI_API_KEY)

CAM_INDEX = 0
AUDIO_PATH = "welcome.wav"  # backend 폴더 안에 이 파일이 있어야 합니다.
FACE_WIDTH_TRIGGER = 200    # 이 크기보다 얼굴이 커지면(가까워지면) 인사
RESET_MARGIN = 100          # 인사 후 이만큼 멀어져야 재장전
HOLD_OFF_SEC = 20.0          # 연속 재생 방지 대기 시간

# 전역 변수 (스레드 간 공유)
output_frame = None         # 웹으로 보낼 현재 화면 프레임
frame_lock = threading.Lock() # 프레임 충돌 방지용 락
trigger_value = FACE_WIDTH_TRIGGER # 실시간 조절용 변수


# ✅ [추가] 녹화기 초기화
recorder = TimeLapseRecorder()
current_recording_order_id = None

# ============================
# ✅ [1. 설정] 환경 설정
# ============================
UPLOAD_DIR = "./saved_drawings"
if not os.path.exists(UPLOAD_DIR):
    os.makedirs(UPLOAD_DIR)

CRED_PATH = "/home/dell/rokey_robot_arm/backend/rokey-ad6ec-firebase-adminsdk-fbsvc-848d4f8c3c.json"
DB_URL = "https://rokey-ad6ec-default-rtdb.asia-southeast1.firebasedatabase.app"

# ✅ Gemini API 키 (REST API 직접 호출용)
GEMINI_API_KEY = "AIzaSyCXQ0yXSVjdT4menmFIa57BXr4GmS5v0co"

# 로봇 작업 설정
CAKE_DIAMETER_MM = 300.0       
ROBOT_CENTER_X = 0.0           
ROBOT_CENTER_Y = 0.0           
ROBOT_CENTER_Z = -155.0        
FLIP_Y_AXIS = True             

# 이미지 처리 옵션
TARGET_SIZE_PX = 480

# ============================
# ✅ [2. 초기화] Firebase & FastAPI
# ============================
print("\n🔍 [System Diagnostic] Starting Firebase Connection Check...")
if not os.path.exists(CRED_PATH):
    print(f"❌ [CRITICAL] Key file NOT FOUND at: {CRED_PATH}")
else:
    print(f"✅ Key file found.")

if not firebase_admin._apps:
    try:
        cred = credentials.Certificate(CRED_PATH)
        firebase_admin.initialize_app(cred, {
            'databaseURL': DB_URL,
            # 👇 이미지에 나온 주소를 여기에 넣으세요 (gs:// 제외)
            'storageBucket': 'rokey-ad6ec.firebasestorage.app' 
        })
        print("🔥 Firebase (DB + Storage) Connected Successfully!")
    except Exception as e:
        print(f"❌ [CRITICAL] Firebase Init Failed: {e}")
app = FastAPI()

origins = [
    "http://localhost:5173",
    "http://127.0.0.1:5173",
    "*" 
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.mount("/images", StaticFiles(directory=UPLOAD_DIR), name="images")

# ==========================================
# 📩 [NEW] CoolSMS (Solapi) 문자 발송 함수
# ==========================================
def send_sms(to_number, text):
    # ⚠️ 이미지에서 추출한 키 적용
    api_key = "NCS0FF3FNARVSWOO"
    api_secret = "JFDOQYVJYMEFZ4JWZXQJ5Q69SZAP6YES"
    
    # ⚠️ [중요] 쿨에스엠에스 사이트에서 '발신번호 등록'을 해야 전송됩니다.
    # 등록된 번호를 아래에 적어주세요. (테스트용으로 본인 번호 등록 필수)
    from_number = "01054576826" 

    # Solapi 인증 헤더 생성
    date = datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'
    salt = str(uuid.uuid4().hex)
    combined = date + salt
    signature = hmac.new(api_secret.encode(), combined.encode(), hashlib.sha256).hexdigest()
    
    headers = {
        "Authorization": f"HMAC-SHA256 apiKey={api_key}, date={date}, salt={salt}, signature={signature}",
        "Content-Type": "application/json"
    }
    
    url = "https://api.solapi.com/messages/v4/send"
    
    payload = {
        "message": {
            "to": to_number,
            "from": from_number,
            "text": text
        }
    }
    
    try:
        res = requests.post(url, headers=headers, json=payload)
        if res.status_code == 200:
            print(f"📨 [SMS 전송 성공] To: {to_number}")
        else:
            print(f"❌ [SMS 전송 실패] {res.text}")
    except Exception as e:
        print(f"❌ [SMS 에러] {e}")

# ==========================================
# ⏰ [NEW] 스케줄러 작업: 픽업 10분 전 알림
# ==========================================
def check_pickup_alerts():
    # print("⏰ [Scheduler] 픽업 임박 주문 스캔 중...")
    
    try:
        orders_ref = db.reference('orders')
        all_orders = orders_ref.get()

        if not all_orders: return

        now = datetime.now()
        
        for key, order in all_orders.items():
            # 이미 완료/취소되었거나 알림 보낸 주문 패스
            if order.get('status') in ['done', 'cancelled', 'reset']:
                continue
            if order.get('alert_sent') is True:
                continue
            
            # 전화번호 없으면 패스
            user_phone = order.get('phone')
            if not user_phone:
                continue

            # 픽업 시간 파싱
            pickup_dt = None
            
            # 1. 현장 픽업 (주문 시간 + 15분)
            if order.get('pickup_type') == 'onsite':
                ts = order.get('timestamp')
                # 문자열/숫자 처리를 안전하게
                try:
                    if isinstance(ts, str):
                        start_dt = datetime.strptime(ts, "%Y-%m-%d %H:%M:%S")
                    else:
                        start_dt = datetime.fromtimestamp(ts)
                    pickup_dt = start_dt + timedelta(minutes=15)
                except:
                    continue

            # 2. 예약 픽업
            else:
                p_time = order.get('pickup_time')
                if p_time and p_time != "now":
                    try: pickup_dt = datetime.strptime(p_time, "%Y-%m-%d %H:%M")
                    except: continue
            
            if not pickup_dt: continue

            # 남은 시간 계산
            diff = pickup_dt - now
            minutes_left = diff.total_seconds() / 60

            # "10분 ~ 0분" 사이 남았을 때 발송
            if 0 < minutes_left <= 15:
                msg = f"[ROKEY] 고객님, 주문하신 케이크 픽업 15분 전입니다! 🎂\n픽업시간: {pickup_dt.strftime('%H:%M')}"
                
                # 문자 발송 실행
                send_sms(user_phone, msg)
                
                # 중복 발송 방지 업데이트
                orders_ref.child(key).update({"alert_sent": True})
                print(f"✅ 알림 처리 완료: {key}")

    except Exception as e:
        print(f"⚠️ 스케줄러 에러: {e}")


# ============================
# ✅ [3. 유틸리티 함수]
# ============================

def upload_image_to_storage(local_file_path, destination_blob_name):
    """
    로컬 이미지를 Firebase Storage에 업로드하고, 공개 다운로드 URL을 반환합니다.
    """
    try:
        bucket = storage.bucket()
        blob = bucket.blob(destination_blob_name)
        
        # 파일 업로드
        blob.upload_from_filename(local_file_path)
        
        # 공개 액세스 허용 (이미지 다운로드를 위해 필요)
        blob.make_public()
        
        print(f"☁️ [Storage] 업로드 성공: {blob.public_url}")
        return blob.public_url
    except Exception as e:
        print(f"❌ [Storage] 업로드 실패: {e}")
        return None

def normalize_points_for_web(points_px: np.ndarray, W: int, H: int) -> np.ndarray:
    cx, cy = W / 2.0, H / 2.0
    u = (points_px[:, 0] - cx) / (W / 2.0)
    v = (points_px[:, 1] - cy) / (H / 2.0)
    return np.stack([u, v], axis=1).astype(np.float32)

def normalized_to_robot_xyz(uv: np.ndarray, diameter_mm: float) -> np.ndarray:
    R = diameter_mm / 2.0
    X = ROBOT_CENTER_X + uv[:, 0] * R
    Y = ROBOT_CENTER_Y + uv[:, 1] * R
    if FLIP_Y_AXIS: Y = -Y
    Z = np.full((len(uv),), float(ROBOT_CENTER_Z), dtype=np.float32)
    return np.stack([X, Y, Z], axis=1)

def upload_to_firebase(design_id, name, robot_paths,image_url=None):
    try:
        json_paths = []
        for path in robot_paths:
            stroke = []
            for point in path:
                stroke.append({
                    "x": round(float(point[0]), 3),
                    "y": round(float(point[1]), 3),
                    "z": round(float(point[2]), 3)
                })
            json_paths.append(stroke)

        data = {
            "name": name,
            "type": "custom",
            "paths": json_paths,
            "image_url": image_url,  # 👈 DB에 이미지 주소 저장
            "updated_at": int(time.time() * 1000)
        }
        ref = db.reference(f'custom_designs/{design_id}')
        ref.set(data)
        print(f"🔥 [Firebase] Uploaded design: custom_designs/{design_id}")
        return True
    except Exception as e:
        print(f"❌ [Firebase] Upload Failed: {e}")
        return False

def run_face_detection():
    global output_frame, trigger_value

    # 1. 오디오 초기화
    try:
        pygame.mixer.init()
        sound = pygame.mixer.Sound(AUDIO_PATH)
        print("🔊 [Audio] welcome.wav 로드 성공")
    except Exception as e:
        print(f"⚠️ [Audio] 오디오 초기화 실패 (파일 경로 확인): {e}")
        sound = None

    # 2. 카메라 & 얼굴 인식기 초기화
    cap = cv2.VideoCapture(CAM_INDEX)
    # Haar Cascade 파일 로드 (OpenCV 내장)
    cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    face_cascade = cv2.CascadeClassifier(cascade_path)

    if not cap.isOpened():
        print("❌ [Camera] 카메라를 열 수 없습니다.")
        return

    is_near = False
    last_play_time = 0.0

    print("👀 [Vision] 얼굴 인식 백그라운드 스레드 시작")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # 그레이스케일 변환 (인식용)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 얼굴 검출
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60))

        # 가장 큰 얼굴 찾기
        target = None
        if len(faces) > 0:
            target = max(faces, key=lambda r: r[2] * r[3]) # w * h 가 가장 큰 얼굴

        now = time.time()
        
        # --- 로직 판정 ---
        if target is not None:
            x, y, w, h = target
            
            # 현재 거리 판정 (가까움: True / 멂: False)
            near_now = (w >= trigger_value)

            # 진입 시점 (멀리 -> 가까움) & 쿨타임 지남
            if (not is_near) and near_now and (now - last_play_time >= HOLD_OFF_SEC):
                if sound: 
                    sound.play()
                    print(f"👋 [Welcome] 손님 감지! (얼굴 크기: {w}px)")
                last_play_time = now
                is_near = True # 상태 변경
            
            # 이탈 시점 (히스테리시스 적용: 기준값보다 확실히 작아져야 리셋)
            if is_near and (w <= trigger_value - RESET_MARGIN):
                is_near = False

            # --- 화면에 그리기 (디버그용) ---
            color = (0, 255, 0) if not is_near else (0, 255, 255) # 평소 초록, 인사중 노랑
            cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
            cv2.putText(frame, f"Width: {w} / Trig: {trigger_value}", (x, y-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        else:
            # 얼굴 없으면 상태 리셋 (다시 나타나면 바로 판정 가능하게 할지는 선택사항)
            # 여기서는 얼굴이 사라지면 '멀어짐'으로 간주
            is_near = False

        # --- 웹 송출을 위해 전역 변수에 저장 ---
        with frame_lock:
            output_frame = frame.copy()
        
        # CPU 점유율 조절
        time.sleep(0.03) 

# main.py 내부의 generate_frames 함수를 이걸로 교체하세요

def generate_frames():
    global output_frame
    
    # 1. 빈 화면 (검은색) 미리 생성 (480x640 크기)
    blank_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    _, encoded_blank = cv2.imencode(".jpg", blank_frame)
    blank_bytes = encoded_blank.tobytes()

    while True:
        frame_to_send = None
        
        with frame_lock:
            if output_frame is None:
                # 2. 카메라가 아직 안 켜졌으면 '검은 화면'을 전송
                yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
                       blank_bytes + b'\r\n')
                time.sleep(0.1) # 서버 과부하 방지
                continue
            
            # 3. 정상 프레임이 있으면 복사해서 사용
            frame_to_send = output_frame.copy()

        # 인코딩 (Lock 밖에서 수행하여 성능 향상)
        try:
            flag, encodedImage = cv2.imencode(".jpg", frame_to_send)
            if not flag:
                continue
            
            # 4. 실제 이미지 전송
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
                   bytearray(encodedImage) + b'\r\n')
        except Exception as e:
            print(f"Frame encoding error: {e}")
            continue
# main.py의 process_image_file 함수 내 시각화 로직 수정
def process_image_file(file_path, save_debug_path=None):
    img = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
    TARGET_SIZE_PX = 480
    
    # 1. 전처리 및 경로 추출 (v2.2 로직)
    binary = cv_utils.preprocess_image_smart(img, TARGET_SIZE_PX)
    raw_paths = cv_utils.get_combined_paths(binary, min_area=15)
    
    processed = []
    for p in raw_paths:
        p = cv_utils.resample_polyline(p, 2.0)
        p = cv_utils.smooth_polyline(p, 2)
        if len(p) > 1: processed.append(p)
            
    unique = cv_utils.remove_overlapping_contours(processed, TARGET_SIZE_PX, TARGET_SIZE_PX)
    connected = cv_utils.connect_broken_contours_smart(unique, max_gap=10.0, force_gap=5.0)
    final_paths = cv_utils.sort_contours_nearest(connected)

    # 🔍 [시각화 로직 강화]
    if save_debug_path:
        # 캔버스 배경 (어두운 회색으로 설정하여 흰색/유색 선이 잘 보이게 함)
        vis = np.full((TARGET_SIZE_PX, TARGET_SIZE_PX, 3), 40, dtype=np.uint8) 
        
        for i, p in enumerate(final_paths):
            # 획마다 다른 색상 부여 (고대비 색상 리스트)
            colors = [(255, 100, 100), (100, 255, 100), (100, 100, 255), (255, 255, 100), (255, 100, 255), (100, 255, 255)]
            color = colors[i % len(colors)]
            
            pts_int = p.astype(np.int32)
            
            # 1. 선 그리기 (두께 2)
            cv2.polylines(vis, [pts_int], False, color, 2, cv2.LINE_AA)
            
            # 2. 시작점(초록)과 끝점(빨강) 표시
            cv2.circle(vis, tuple(pts_int[0]), 5, (0, 255, 0), -1)  # 시작
            cv2.circle(vis, tuple(pts_int[-1]), 3, (0, 0, 255), -1) # 끝
            
            # 3. 획 번호 표시 (시작점 근처에 크게)
            text_pos = (pts_int[0][0] + 10, pts_int[0][1] - 10)
            cv2.putText(vis, f"#{i+1}", text_pos, cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 255, 255), 1)

        cv2.imwrite(save_debug_path, vis)
        print(f"✅ Debug image saved with {len(final_paths)} strokes.")

    # (이하 정규화 및 리턴 로직 동일...)
    normalized_paths = []
    for poly in final_paths:
        cx, cy = TARGET_SIZE_PX / 2.0, TARGET_SIZE_PX / 2.0
        u = (poly[:, 0] - cx) / (TARGET_SIZE_PX / 2.0)
        v = (poly[:, 1] - cy) / (TARGET_SIZE_PX / 2.0)
        norm = np.stack([u, v], axis=1).astype(np.float32)
        normalized_paths.append(norm.tolist())
    return normalized_paths

# ==========================================
# 🕒 [NEW] 시간 슬롯 중복 검사 헬퍼 함수
# ==========================================
def find_next_available_time(start_dt: datetime):
    """
    start_dt부터 시작해서 15분 작업이 가능한 가장 빠른 시간을 찾습니다.
    """
    orders_ref = db.reference('orders')
    all_orders = orders_ref.get()
    
    # 정렬된 사용 중인 시간대 리스트 만들기 [(start, end), ...]
    busy_slots = []
    if all_orders:
        for _, order in all_orders.items():
            if order.get('status') in ['done', 'reset', 'cancelled']:
                continue
            
            # 시작 시간 파싱
            s_time = None
            if order.get('pickup_type') == 'onsite':
                ts = order.get('timestamp')
                if isinstance(ts, (int, float)): s_time = datetime.fromtimestamp(ts)
                elif isinstance(ts, str): 
                    try: s_time = datetime.strptime(ts, "%Y-%m-%d %H:%M:%S")
                    except: pass
            else:
                pt = order.get('pickup_time')
                if pt and pt != "now":
                    try: s_time = datetime.strptime(pt, "%Y-%m-%d %H:%M")
                    except: pass
            
            if s_time:
                e_time = s_time + timedelta(minutes=15)
                busy_slots.append((s_time, e_time))
    
    # 시간 순 정렬
    busy_slots.sort(key=lambda x: x[0])
    
    # 빈 공간 찾기 (Candidate Check)
    candidate = start_dt
    
    while True:
        candidate_end = candidate + timedelta(minutes=15)
        conflict = False
        
        for (s, e) in busy_slots:
            # 겹침 판정: (요청시작 < 기존종료) AND (요청종료 > 기존시작)
            if candidate < e and candidate_end > s:
                # 겹치면, 해당 작업이 끝나는 시간으로 점프해서 다시 검사
                # (조금의 여유를 위해 1초 추가하거나 그대로 사용)
                candidate = e 
                conflict = True
                break # 루프 다시 시작 (새 candidate로 처음부터 검사)
        
        if not conflict:
            return candidate

def validate_time_slot(new_type: str, new_time_str: str):
    # 1. 시간 파싱 로직 개선: 'onsite'라도 시간이 명시되면 그 시간을 따름
    if new_time_str == "now":
        target_start = datetime.now()
    else:
        try:
            target_start = datetime.strptime(new_time_str, "%Y-%m-%d %H:%M")
        except ValueError:
            return False, "날짜 형식이 올바르지 않습니다."

    target_end = target_start + timedelta(minutes=15)

    # 2. 겹침 확인 로직 (기존과 동일하지만, 위에서 만든 busy_slots 로직 재사용 가능)
    # 여기선 간단히 기존 로직 유지하되 find_next_available_time 활용
    
    # (단순 확인을 위해 find 함수 내부 로직과 유사하게 다시 조회)
    orders_ref = db.reference('orders')
    all_orders = orders_ref.get()
    
    if not all_orders: return True, "예약 가능"

    for _, order in all_orders.items():
        if order.get('status') in ['done', 'reset', 'cancelled']: continue
        
        existing_start = None
        if order.get('pickup_type') == 'onsite':
            ts = order.get('timestamp')
            if isinstance(ts, (int, float)): existing_start = datetime.fromtimestamp(ts)
            elif isinstance(ts, str): 
                try: existing_start = datetime.strptime(ts, "%Y-%m-%d %H:%M:%S")
                except: pass
        else:
            pt = order.get('pickup_time')
            if pt and pt != "now":
                try: existing_start = datetime.strptime(pt, "%Y-%m-%d %H:%M")
                except: pass
        
        if existing_start:
            existing_end = existing_start + timedelta(minutes=15)
            if target_start < existing_end and target_end > existing_start:
                return False, f"작업 중입니다. ({existing_start.strftime('%H:%M')}~{existing_end.strftime('%H:%M')})"

    return True, "예약 가능"

# ============================
# ✅ [4. 데이터 모델]
# ============================
# 백엔드의 Pydantic 모델 (FastAPI 기준)
class CakeOrder(BaseModel):  # Order에서 CakeOrder로 변경
    size: str
    design: str
    syrup: str
    powder: str
    toppings: List[int] = []  # 명시적 리스트 타입 지정
    topping_count: int = 0          
    type: str
    design_id: Optional[Union[int, str]] = None
    pickup_type: str = "onsite"  # 기본값 'onsite'
    pickup_time: str = "now"     # 기본값 'now'
    phone: str = None  # 👈 추가됨 (전화번호)

class PathData(BaseModel):
    paths: list[list[list[float]]] 
    size: str 
    cloud_url: Optional[str] = None  # 👈 필드 추가

class PromptRequest(BaseModel):
    prompt: str

def get_next_order_id():
    try:
        counter_ref = db.reference('meta/last_order_id')
        def increment_counter(current_val):
            if current_val is None: return 1 
            try: return int(current_val) + 1
            except: return 1
        new_id = counter_ref.transaction(increment_counter)
        return new_id
    except Exception as e:
        print(f"❌ ID Generation Error: {e}")
        return int(datetime.now().timestamp())

# ============================
# ✅ [5. API 엔드포인트]
# ============================

@app.get("/")
def read_root():
    return {"message": "ROKEY Robot System Backend is Running!"}

@app.post("/api/check_availability")
def check_availability(request: dict):
    target_time = request.get("time")
    target_type = "onsite" if target_time == "now" else "reservation"
    
    is_available, message = validate_time_slot(target_type, target_time)
    
    response = {
        "available": is_available,
        "message": message
    }

    # ❌ 만약 불가능하다면, 가장 빠른 시간 계산해서 알려줌
    if not is_available:
        base_time = datetime.now()
        next_slot = find_next_available_time(base_time)
        
        # 포맷팅 (YYYY-MM-DD HH:mm)
        response["recommended_time"] = next_slot.strftime("%Y-%m-%d %H:%M")
        
        # 읽기 쉬운 메시지 추가 (예: 14:30)
        response["rec_msg"] = next_slot.strftime("%H:%M")

    return response

@app.post("/api/analyze_image")
async def analyze_image(file: UploadFile = File(...)):
    try:
        custom_id = str(uuid.uuid4())[:8]
        file_ext = file.filename.split(".")[-1]
        saved_filename = f"{custom_id}.{file_ext}"
        debug_filename = f"debug_{custom_id}.png" # 시각화용 파일명
        
        file_path = os.path.join(UPLOAD_DIR, saved_filename)
        debug_path = os.path.join(UPLOAD_DIR, debug_filename)

        with open(file_path, "wb") as buffer:
            shutil.copyfileobj(file.file, buffer)

        cloud_url = upload_image_to_storage(file_path, f"user_uploads/{saved_filename}")
        print(f"\n🔗 [다운로드 URL 확인]: {cloud_url}\n")
        # 1. 이미지 로드 및 분석 (이미 구현된 process_image_file 사용)
        # 단, 시각화 이미지를 얻기 위해 내부 로직을 조금 노출하거나 별도 함수화 필요
        normalized_paths = process_image_file(file_path, save_debug_path=debug_path)

        return {
            "status": "success",
            "paths": normalized_paths, 
            "image_url": f"/images/{saved_filename}",
            "cloud_url": cloud_url,                   # 👈 로봇이 다운받을 스토리지 주소
            "debug_url": f"/images/{debug_filename}" # 브라우저에서 확인할 분석 결과 이미지
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

# --- 5-2. [NEW] AI 이미지 생성 (REST API 직접 호출 방식) ---
@app.post("/api/generate_image")
async def generate_image(request: PromptRequest):
    """
    REST API를 직접 호출하여 이미지 생성 (SDK 불필요)
    Gemini API 실패 시 Pollinations로 자동 Fallback
    한글 프롬프트 지원 및 초간단 스타일 최적화
    """
    
    # 🤖 [개선] 영문 프롬프트로 변환하되, 극도로 단순한 스타일 강조
    # 한글 입력을 영어로 번역 (간단한 매핑, 필요시 확장 가능)
    prompt_translation = {
        "유령": "ghost",
        "고양이": "cat",
        "강아지": "dog",
        "토끼": "rabbit",
        "곰": "bear",
        "새": "bird",
        "물고기": "fish",
        "별": "star",
        "하트": "heart",
        "꽃": "flower",
        "나무": "tree",
        "집": "house",
        "차": "car",
        "비행기": "airplane",
        "로봇": "robot"
    }
    
    # 한글이면 영어로 변환, 아니면 그대로 사용
    english_prompt = prompt_translation.get(request.prompt.strip(), request.prompt)
    
    cv_optimized_prompt = (
        f"Ultra minimalist line art drawing of a {english_prompt}. "
        "CRITICAL REQUIREMENTS: "
        "- Single thick black outline (5-8px width) forming ONE continuous closed loop"
        "- Blob-like organic shape with smooth curves, absolutely NO sharp corners"
        "- Maximum 5 visual elements total (e.g., body outline + 2 eyes + 2 small details)"
        "- Eyes MUST be simple perfect circles or dots, positioned asymmetrically for cuteness"
        "- Pure white background (#FFFFFF), zero textures, zero patterns, zero shading"
        "- Cartoon style similar to 'doodle art' or 'kawaii minimalism'"
        "- NO realistic anatomy, NO complex details, NO multiple strokes"
        "- Think: child's first drawing, maximum simplicity"
        "- Style reference: LINE Friends characters, simple emoji designs"
    )
    
    # Pollinations용 Fallback 프롬프트 (영문)
    fallback_prompt_en = (
        f"simple thick black line drawing of {english_prompt}, "
        f"minimalist cartoon style, blob shape, two circle eyes, "
        f"white background, one continuous outline, kawaii doodle art"
    )

    try:
        print(f"🎨 Generating image with Gemini (REST) for prompt: {request.prompt}")
        
        PROJECT_ID = "157620141318" 
        REGION = "us-central1"
        url = f"https://{REGION}-aiplatform.googleapis.com/v1/projects/{PROJECT_ID}/locations/{REGION}/publishers/google/models/imagen-3.0-generate-001:predict"

        headers = {
            "Content-Type": "application/json"
        }
        
        payload = {
            "instances": [
                {"prompt": cv_optimized_prompt}
            ],
            "parameters": {
                "sampleCount": 1,
                "aspectRatio": "1:1",
                # 더 단순한 이미지를 위한 추가 파라미터
                "guidanceScale": 7.5  # 프롬프트 준수도 조정
            }
        }

        response = requests.post(url, headers=headers, json=payload, timeout=30)
        
        if response.status_code == 200:
            result = response.json()
            if "predictions" in result and len(result["predictions"]) > 0:
                b64_data = result["predictions"][0]["bytesBase64Encoded"]
                image_data = base64.b64decode(b64_data)
                
                custom_id = str(uuid.uuid4())[:8]
                saved_filename = f"gen_{custom_id}.png"
                file_path = os.path.join(UPLOAD_DIR, saved_filename)
                
                with open(file_path, "wb") as f:
                    f.write(image_data)
                    
                print(f"✅ Gemini (REST) Image saved to {file_path}")
            else:
                raise Exception("No image data in response")
        else:
            raise Exception(f"Gemini API returned status {response.status_code}")

    except Exception as e:
        print(f"ℹ️ Gemini API Unavailable ({e}), switching to Pollinations AI...")
        
        try:
            # Fallback: Pollinations AI - 영문 프롬프트로 극단적 단순화
            safe_prompt = urllib.parse.quote(fallback_prompt_en)
            image_url = (
                f"https://image.pollinations.ai/prompt/{safe_prompt}"
                f"?width=512&height=512&nologo=true"
                f"&model=flux"  # 더 나은 품질의 모델
                f"&enhance=false"  # 과도한 디테일 방지
            )
            
            resp = requests.get(image_url, timeout=30)
            if resp.status_code != 200:
                raise HTTPException(status_code=500, detail="Image generation failed completely")
                
            custom_id = str(uuid.uuid4())[:8]
            saved_filename = f"gen_{custom_id}.png"
            file_path = os.path.join(UPLOAD_DIR, saved_filename)
            with open(file_path, "wb") as f:
                f.write(resp.content)
            print(f"✅ Pollinations AI Image saved to {file_path}")
            
        except Exception as fallback_error:
            return {"status": "error", "message": str(fallback_error)}

    # 이미지 처리 및 경로 변환
    try:
        normalized_paths = process_image_file(file_path)
        return {
            "status": "success",
            "paths": normalized_paths,
            "image_url": f"/images/{saved_filename}",
            "message": "AI generated image processed successfully"
        }
    except Exception as e:
        return {"status": "error", "message": f"Processing failed: {str(e)}"}   

# --- 5-3. 최종 경로 저장 ---
@app.post("/api/save_custom_paths")
async def save_custom_paths(data: PathData):
    try:
        custom_id = str(uuid.uuid4())[:8]
        
        # 케이크 사이즈에 따른 지름 설정
        diameter = 240.0 # 기본값 (1호)
        
        if data.size == "2호":
            diameter = 300.0
            print(f"🛠️ [Size Check] 2호 선택됨 -> 지름 300mm 적용")
        elif data.size == "1호":
            diameter = 240.0
            print(f"🛠️ [Size Check] 1호 선택됨 -> 지름 240mm 적용")
        else:
            print(f"⚠️ [Size Check] 알 수 없는 사이즈 '{data.size}' -> 기본값 240mm 적용")
        
        robot_paths_3d = []
        
        for stroke in data.paths:
            uv = np.array(stroke, dtype=np.float32)
            robot_xyz = normalized_to_robot_xyz(uv, diameter)
            robot_paths_3d.append(robot_xyz)

        design_name = f"Custom-{custom_id}"
        success = upload_to_firebase(custom_id, design_name, robot_paths_3d, data.cloud_url)        
        if success:
            return {"status": "success", "design_id": custom_id}
        else:
            return {"status": "error", "message": "DB Save Failed"}
            
    except Exception as e:
        print(e)
        return {"status": "error", "message": str(e)}

# --- 5-4. 주문 생성 (한글 타입 대응 및 좌표 연결 수정) ---
@app.post("/api/order")
def create_order(order: CakeOrder):
    print("------------------------------------------------")
    print(f"📝 [주문 접수] 타입: {order.type}, ID: {order.design_id}")
    is_ok, msg = validate_time_slot(order.pickup_type, order.pickup_time)
    try:
        order_id = get_next_order_id()
        order_data = order.dict()
        order_data['order_id'] = order_id
        
        # 현재 시간 객체
        now_dt = datetime.now()
        
        order_data['timestamp'] = now_dt.strftime("%Y-%m-%d %H:%M:%S")
        order_data['status'] = 'pending'
        
        # ✅ [추가됨] 현장 픽업이고 시간이 'now'라면 -> 현재 시간으로 변경해서 저장
        if order_data['pickup_type'] == 'onsite' and order_data['pickup_time'] == 'now':
            order_data['pickup_time'] = now_dt.strftime("%Y-%m-%d %H:%M")
        
        p_type = "현장 픽업" if order.pickup_type == "onsite" else "예약 픽업"
        p_time = order.pickup_time if order.pickup_type == "reservation" else "즉시"
        print(f"   📅 [픽업 정보] {p_type} ({p_time})")
       
        target_design_id = order.design_id
        db_path = "custom_designs" 

        # 2️⃣ 음성 주문(AI_VOICE_ORDER)일 경우 키워드 매핑 처리
        if order.type == "AI_VOICE_ORDER":
            # .strip()으로 공백 제거, 인식률을 높이기 위해 필요 시 필터링 추가
            clean_keyword = order.design.strip() if order.design else ""
            
            # 매핑 테이블 (필요시 '강아지', '멍멍이' 등 유사어도 추가 가능)
            keyword_map = {
                "개": 3, 
                "가나디": 4
            }
            
            matched_id = keyword_map.get(clean_keyword)
            
            if matched_id:
                target_design_id = matched_id
                order_data['design_id'] = matched_id
                print(f"   🎯 [AI 매칭] 키워드 '{clean_keyword}' -> ID {matched_id} 할당")
            else:
                print(f"   ⚠️ [AI 매칭 실패] '{clean_keyword}'와 일치하는 키워드가 map에 없음")

        # 3️⃣ [핵심 해결책] 타입에 상관없이 design_id가 있으면 Firebase에서 좌표(paths)를 가져와 연결
        if target_design_id:
            try:
                # Firebase DB에서 해당 도안 데이터 조회
                design_ref = db.reference(f'{db_path}/{target_design_id}')
                design_val = design_ref.get()
                
                if design_val and 'paths' in design_val:
                    # 주문 데이터 하위에 'drawing_path'라는 이름으로 좌표 리스트 삽입
                    order_data['drawing_path'] = design_val['paths']
                    print(f"   ✅ 좌표 연결 성공: {db_path}/{target_design_id}")
                else:
                    print(f"   ⚠️ DB에 좌표 데이터가 없습니다: {db_path}/{target_design_id}")
            except Exception as link_error:
                print(f"   ❌ 좌표 연결 중 오류: {link_error}")

        # 4️⃣ 최종 주문 데이터를 Firebase 'orders' 노드에 저장
        ref = db.reference('orders')
        ref.child(str(order_id)).set(order_data)
        
        print(f"🔥 [성공] 주문 #{order_id}번이 Firebase에 저장되었습니다.")
        
    except Exception as e:
        print(f"❌ [실패] 주문 저장 에러: {e}")
        return {"status": "error", "message": str(e)}
    
    print("------------------------------------------------")
    return {
        "status": "success",
        "message": f"주문번호 {order_id}번 접수 완료",
        "order_id": order_id
    }

# --- 5-5. 음성 상담원 엔드포인트 ---
# =====================================================================
# 🚨 [수정] 환각(MBC 뉴스 등) 방지 필터가 적용된 음성 상담원 함수
# =====================================================================
# main.py 의 기존 voice_counselor 함수를 아래 코드로 덮어씌우세요.

# =====================================================================
# 🗣️ [UPGRADE] AI 음성 상담원 (풀 코스: 주문 -> 픽업 -> 결제)
# =====================================================================
@app.post("/api/voice_counselor")
async def voice_counselor(
    file: UploadFile = File(...), 
    current_context: str = Form(None)
):
    try:
        # 1. STT (음성 -> 텍스트)
        audio_bytes = await file.read()
        audio_file = io.BytesIO(audio_bytes)
        audio_file.name = "input.wav"
        
        transcript = client.audio.transcriptions.create(
            model="whisper-1", 
            file=audio_file,
            language="ko",
            temperature=0.0 
        )
        user_text = transcript.text.strip()
        print(f"🗣️ 사용자: {user_text}")

        # 환각 필터링
        hallucination_triggers = ["MBC 뉴스", "자막 제작", "시청해 주셔서", "구독과 좋아요"]
        if any(t in user_text for t in hallucination_triggers) or len(user_text) < 2:
            return JSONResponse({
                "status": "success", "action": "chat",
                "message": "죄송해요, 잘 못 들었어요. 다시 말씀해 주시겠어요?",
                "order_data": json.loads(current_context) if current_context else {},
                "audio_base64": None
            })

        # 2. 컨텍스트 로드 및 기본값 설정
        default_context = {
            "size": None, "design_keyword": None, 
            "syrup": None, "powder": None, 
            "toppings": [], "topping_count": 0,
            "asked_fields": [],
            # 👇 새로 추가된 필드들 (상태 관리용)
            "pickup_type": None,    # 'onsite' or 'reservation'
            "pickup_time": None,    # 'now' or 'YYYY-MM-DD HH:MM'
            "phone": None,          # '01012345678'
            "payment_method": None, # 'card', 'kakao', 'naver'
            "step": "cake"          # 현재 단계: cake -> pickup -> contact -> payment -> done
        }
        
        try:
            incoming = json.loads(current_context) if current_context else {}
            context = {**default_context, **incoming}
        except:
            context = default_context

        # 현재 시간 (AI에게 시간 개념 주입)
        now_str = datetime.now().strftime("%Y-%m-%d %H:%M")
        
        # 3. GPT-4o 도구 정의 (시간 확인용)
        tools = [
            {
                "type": "function",
                "function": {
                    "name": "check_availability_tool",
                    "description": "Check availability. If busy, returns recommendation.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "type": {"type": "string", "enum": ["onsite", "reservation"]},
                            "time_str": {"type": "string", "description": "For onsite: 'now', For reservation: 'YYYY-MM-DD HH:MM'"}
                        },
                        "required": ["type", "time_str"]
                    }
                }
            }
        ]

        # 4. 시스템 프롬프트 (단계별 진행 가이드)
        system_prompt = f"""
        당신은 '달콤 케이크'의 스마트한 AI 점원입니다.
        현재 시각: {now_str}
        현재 주문 상태: {json.dumps(context, ensure_ascii=False)}

        [진행 단계 (step)]
        1. **cake**: 사이즈(1호/2호), 도안, 시럽, 파우더, 토핑(총 8개)을 확정합니다.
           - 토핑이 8개가 아니면 채우도록 유도하세요.
           - 정보가 다 차면 step을 'pickup'으로 변경하고 픽업 방식을 물어보세요.

        2. **pickup**: 현장(onsite) / 예약(reservation)을 묻고 시간을 정합니다.
           - 사용자가 시간을 말하면 반드시 `check_availability_tool` 도구를 사용하세요.
           - 결과가 "가능"이면 pickup_time을 저장하고 step을 'contact'로 넘기세요.
           - "불가능"이면 추천 시간을 안내하고 다시 물어보세요.

        3. **contact**: 전화번호를 물어봅니다. (010...)
           - 유효한 번호면 step을 'payment'로 넘기세요.

        4. **payment**: 결제 수단(카드/카카오페이/네이버페이)을 물어봅니다.
           - 확정되면 step을 'done', action을 'confirm_order'로 설정하세요.
        5. 도안은 개, 가나디 2개중 선택할 수 있게 유도하세요.

        [응답 JSON 형식]
        {{
          "ai_message": "사용자에게 할 말",
          "updated_context": {{ ...모든 필드 업데이트... }},
          "action": "chat" 또는 "confirm_order"
        }}
        """

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_text}
        ]

        # 5. LLM 1차 호출 (도구 사용 여부 확인)
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=messages,
            tools=tools,
            tool_choice="auto",
            response_format={"type": "json_object"} 
        )

        assistant_msg = response.choices[0].message
        
        # 6. 도구 호출 처리 (DB 시간 확인 로직)
        if assistant_msg.tool_calls:
            tool_call = assistant_msg.tool_calls[0]
            if tool_call.function.name == "check_availability_tool":
                args = json.loads(tool_call.function.arguments)
                print(f"🔧 AI 도구 호출: {args}")

                # 내부 로직 호출
                is_avail, msg = validate_time_slot(args['type'], args['time_str'])
                
                tool_output = "Available"
                if not is_avail:
                    # 불가능하면 추천 시간 계산
                    base_time = datetime.now() if args['time_str'] == 'now' else datetime.strptime(args['time_str'], "%Y-%m-%d %H:%M")
                    next_slot = find_next_available_time(base_time)
                    rec_str = next_slot.strftime("%H:%M")
                    tool_output = f"Busy. Recommendation: {rec_str}"
                
                # 결과 메시지 추가 후 2차 호출
                messages.append(assistant_msg)
                messages.append({
                    "role": "tool",
                    "tool_call_id": tool_call.id,
                    "content": tool_output
                })
                
                # 2차 호출 (최종 답변 생성)
                response = client.chat.completions.create(
                    model="gpt-4o",
                    messages=messages,
                    response_format={"type": "json_object"}
                )

        # 7. 최종 응답 파싱
        final_content = response.choices[0].message.content
        res_json = json.loads(final_content)
        
        ai_message = res_json.get("ai_message", "네, 알겠습니다.")
        new_context = res_json.get("updated_context", context)
        
        # step이 done이면 주문 확정 액션 전송
        action = "confirm_order" if new_context.get("step") == "done" else "chat"

        print("📝 [AI Context Update Log]")
        updated_count = 0
        for key, new_val in new_context.items():
            old_val = context.get(key)
            
            # 값이 새로 생겼거나 변경된 경우 (리스트인 경우 내용 비교)
            if str(new_val) != str(old_val):
                # None -> 값 채워짐, 혹은 값 변경됨
                if new_val is not None and new_val != "" and new_val != []:
                    print(f"   ✨ {key.upper()}: {old_val} ➡️  \033[92m{new_val}\033[0m") # 초록색 강조
                    updated_count += 1
        
        if updated_count == 0:
            print("   (변경된 정보 없음)")
        print(f"🤖 [AI Message]: {ai_message}")
        print("-" * 50)


        # 8. 음성 합성 (TTS)
        speech_response = client.audio.speech.create(model="tts-1", voice="nova", input=ai_message)
        audio_base64 = base64.b64encode(speech_response.content).decode('utf-8')

        return JSONResponse({
            "status": "success",
            "action": action,
            "message": ai_message,
            "order_data": new_context,
            "audio_base64": audio_base64
        })

    except Exception as e:
        print(f"❌ Error: {e}")
        return JSONResponse({"status": "error", "message": "오류가 발생했습니다."})
    
@app.get("/video_feed")
def video_feed():
    """웹 브라우저 <img> 태그용 MJPEG 스트리밍"""
    return StreamingResponse(generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.post("/api/update_trigger")
def update_trigger(value: int = Body(..., embed=True)):
    """React에서 감지 거리 조절용"""
    global trigger_value
    trigger_value = value
    print(f"🔧 [Setting] 얼굴 감지 기준 변경됨: {trigger_value}px")
    return {"status": "success", "current_trigger": trigger_value}

# main.py

@app.post("/api/reserved_times")
def get_reserved_times(request: dict):
    """
    특정 날짜(date_str)에 이미 예약된 시간 목록을 반환합니다.
    (안전장치 추가 버전: 형식이 잘못된 데이터가 있어도 서버가 죽지 않음)
    """
    target_date = request.get("date")
    
    if not target_date:
        return {"status": "error", "message": "날짜 정보가 필요합니다."}

    reserved_slots = []
    
    # DB에서 주문 데이터 조회
    orders_ref = db.reference('orders')
    all_orders = orders_ref.get()

    if not all_orders:
        return {"status": "success", "reserved_times": []}

    for key, order in all_orders.items(): # key도 같이 받아서 문제있는 데이터 확인용으로 씀
        # 취소되거나 완료된 주문은 제외
        if order.get('status') in ['cancelled', 'reset']:
            continue

        # 1. 예약 주문(reservation)인 경우
        if order.get('pickup_type') == 'reservation':
            p_time = order.get('pickup_time')
            
            # 🛠️ [수정 포인트] 데이터가 있고, 해당 날짜로 시작하는지 확인
            if p_time and p_time.startswith(target_date):
                parts = p_time.split(' ')
                
                # 🚨 [핵심 수정] 잘라낸 개수가 2개 이상인지 확인 (날짜 + 시간)
                if len(parts) >= 2:
                    time_part = parts[1] # "16:15"
                    reserved_slots.append(time_part)
                else:
                    # 형식이 이상하면 그냥 넘어가고 로그만 찍음 (서버 다운 방지)
                    print(f"⚠️ [Data Skip] 시간 형식 오류 (Order {key}): {p_time}")

        # 2. 현장 주문(onsite)인 경우
        elif order.get('pickup_type') == 'onsite':
            ts = order.get('timestamp')
            if ts:
                try:
                    if isinstance(ts, str):
                        dt = datetime.strptime(ts, "%Y-%m-%d %H:%M:%S")
                    else:
                        dt = datetime.fromtimestamp(ts)
                    
                    if dt.strftime("%Y-%m-%d") == target_date:
                        reserved_slots.append(dt.strftime("%H:%M"))
                except:
                    pass

    # 중복 제거 및 정렬
    reserved_slots = sorted(list(set(reserved_slots)))
    
    print(f"📅 [{target_date}] 예약된 시간: {reserved_slots}")
    return {"status": "success", "reserved_times": reserved_slots}

@app.on_event("startup")
async def startup_event():
    print("🚀 [Startup] Starting Face Detection Thread...")
    t = threading.Thread(target=run_face_detection, daemon=True)
    t.start()

    # 👇 스케줄러 시작 코드 추가
    print("⏰ [Startup] Starting SMS Scheduler...")
    scheduler = BackgroundScheduler()
    scheduler.add_job(check_pickup_alerts, 'interval', minutes=1) # 1분마다 실행
    scheduler.start()
if __name__ == "__main__":
    # ✅ 2. 그 다음 서버 실행
    uvicorn.run(app, host="0.0.0.0", port=8000)