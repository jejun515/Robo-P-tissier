import time
import datetime
import uuid
import hmac
import hashlib
import requests
import platform

# ==========================================
# ✅ 사용자 설정 (여기만 수정하세요)
# ==========================================
# 제공해주신 키를 적용했습니다.
API_KEY = "NCS0FF3FNARVSWOO"
API_SECRET = "JFDOQYVJYMEFZ4JWZXQJ5Q69SZAP6YES"

# ⚠️ [중요] 쿨에스엠에스 사이트에서 '발신번호 등록'을 마친 번호를 적어야 합니다.
# 등록되지 않은 번호를 넣으면 "SenderIdMismatch" 에러가 납니다.
FROM_NUMBER = "01054576826"  

# 문자를 받을 본인 휴대폰 번호 (하이픈 없이 입력)
TO_NUMBER = "01040599928" 
# ==========================================

def get_iso_datetime():
    utc_offset_sec = time.altzone if time.localtime().tm_isdst else time.timezone
    utc_offset = datetime.timedelta(seconds=-utc_offset_sec)
    return datetime.datetime.now().replace(tzinfo=datetime.timezone(offset=utc_offset)).isoformat()

def get_headers(api_key, api_secret):
    date = get_iso_datetime()
    salt = str(uuid.uuid4().hex)
    combined = date + salt
    signature = hmac.new(api_secret.encode(), combined.encode(), hashlib.sha256).hexdigest()

    return {
        "Authorization": f"HMAC-SHA256 apiKey={api_key}, date={date}, salt={salt}, signature={signature}",
        "Content-Type": "application/json"
    }

def send_test_message():
    url = "https://api.solapi.com/messages/v4/send"
    headers = get_headers(API_KEY, API_SECRET)
    
    data = {
        "message": {
            "to": TO_NUMBER,
            "from": FROM_NUMBER,
            "text": "[ROKEY 테스트] 인증번호는 [1234] 입니다. 성공! 🎉"
        }
    }

    print(f"🚀 발송 시도 중... (To: {TO_NUMBER})")

    try:
        response = requests.post(url, headers=headers, json=data)
        result = response.json()
        
        print(f"📡 응답 코드: {response.status_code}")
        
        if response.status_code == 200:
            print("✅ 문자 발송 성공!")
            print(f"📜 메시지 ID: {result.get('messageId')}")
            print(f"💰 잔액 차감됨 (약 20원)")
        else:
            print("❌ 발송 실패")
            print(f"에러 코드: {result.get('errorCode')}")
            print(f"에러 메시지: {result.get('errorMessage')}")

    except Exception as e:
        print(f"❌ 연결 오류: {e}")

if __name__ == "__main__":
    send_test_message()