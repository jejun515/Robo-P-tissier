import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import time
import sys

# --- Firebase 설정 (main.py와 동일) ---
CRED_PATH = "/home/dell/rokey_robot_arm/backend/rokey-ad6ec-firebase-adminsdk-fbsvc-848d4f8c3c.json"
DB_URL = "https://rokey-ad6ec-default-rtdb.asia-southeast1.firebasedatabase.app"

# Firebase 앱 초기화
if not firebase_admin._apps:
    cred = credentials.Certificate(CRED_PATH)
    firebase_admin.initialize_app(cred, {
        'databaseURL': DB_URL
    })
    print("🔥 [Monitor] Firebase Connected Successfully!")

def handle_order(event):
    """
    DB에 변경사항이 생길 때마다 호출되는 함수
    event.event_type: 'put' (데이터 쓰기/수정), 'patch' 등
    event.path: 변경된 경로 (예: '/')
    event.data: 변경된 실제 데이터
    """
    
    # 데이터가 없으면 무시
    if event.data is None:
        return

    # event.path가 '/'이면 전체 데이터가 로드된 것 (초기 실행 시)
    # 실제 실시간 추가는 보통 구체적인 path를 가집니다.
    # 여기서는 간단히 새로 들어온 데이터(딕셔너리)를 확인합니다.
    
    print(f"\n🔔 [Update Detected] Type: {event.event_type}")
    
    # 데이터가 딕셔너리 형태라면 (주문 객체)
    if isinstance(event.data, dict):
        # 만약 한 번에 여러 개가 들어오거나, 초기 로딩인 경우
        # event.data가 { "order_id1": {...}, "order_id2": {...} } 형태일 수 있음
        # 혹은 단일 주문 추가시 event.path가 "/order_id"이고 data가 내용일 수 있음
        
        # 간단한 출력 로직
        print(f"📦 Data: {event.data}")
        
        # 여기서 로봇 팔을 움직이는 함수를 호출하면 됩니다.
        # robot_controller.start_process(event.data)

    else:
        print(f"📄 Data: {event.data}")

print("👀 Listening for new orders on 'orders' node... (Press CTRL+C to stop)")

# 'orders' 경로를 구독(Listen)합니다.
ref = db.reference('orders')
# listen 함수는 백그라운드 스레드에서 동작하며 변경사항이 있을 때 handle_order를 실행합니다.
ref.listen(handle_order)

# 메인 스레드가 죽지 않도록 무한 루프
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n🛑 Monitor Stopped.")
    sys.exit(0)