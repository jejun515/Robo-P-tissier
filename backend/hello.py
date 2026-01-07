import firebase_admin
from firebase_admin import credentials, db
import os

# ============================
# ⚙️ 설정 (main.py와 동일)
# ============================
CRED_PATH = "/home/dell/rokey_robot_arm/backend/rokey-ad6ec-firebase-adminsdk-fbsvc-848d4f8c3c.json"
DB_URL = "https://rokey-ad6ec-default-rtdb.asia-southeast1.firebasedatabase.app"

def clean_database():
    print("🚀 [DB Management] Starting to clean 'orders' node...")

    # 1. 인증 확인
    if not os.path.exists(CRED_PATH):
        print(f"❌ Error: Credentials file not found at {CRED_PATH}")
        return

    # 2. 파이어베이스 초기화
    try:
        if not firebase_admin._apps:
            cred = credentials.Certificate(CRED_PATH)
            firebase_admin.initialize_app(cred, {'databaseURL': DB_URL})
            print("✅ Firebase Connected.")
    except Exception as e:
        print(f"❌ Connection Failed: {e}")
        return

    # 3. 데이터 삭제 수행
    try:
        ref = db.reference('orders')
        
        # 데이터가 존재하는지 먼저 확인 (선택 사항)
        snapshot = ref.get()
        if snapshot is None:
            print("ℹ️ 'orders' node is already empty.")
        else:
            # 삭제 실행
            ref.delete()
            print("🔥 SUCCESS: All data in 'orders' has been deleted.")
            
            # 메타데이터(주문 번호 카운터)도 초기화하고 싶다면 아래 주석을 해제하세요.
            # db.reference('meta/last_order_id').set(0)
            # print("🔢 Metadata counter has been reset to 0.")

    except Exception as e:
        print(f"❌ Delete Operation Failed: {e}")

if __name__ == "__main__":
    # 실행 전 마지막 확인
    confirm = input("⚠️  Are you sure you want to delete ALL orders? (y/n): ")
    if confirm.lower() == 'y':
        clean_database()
    else:
        print("❌ Operation cancelled by user.")