import os
import time
import sounddevice as sd
import numpy as np
from scipy.io.wavfile import write
from openai import OpenAI
from pydub import AudioSegment
from pydub.playback import play
import io

# [주의] 노출된 API 키는 꼭 삭제(Revoke)하고 새 키를 발급받아 사용하세요!
client = OpenAI(api_key="")

# 48000Hz가 리눅스 내장 마이크 표준입니다.
FS = 48000  
DURATION = 10  
FILENAME = "input_test.wav"

def record_audio():
    print(f"\n🎤 듣고 있습니다... (지금 크게 말씀하세요!)")
    
    try:
        # device=None으로 두면 아까 설정창에서 선택한 '기본 마이크'를 자동으로 씁니다.
        recording = sd.rec(int(DURATION * FS), samplerate=FS, channels=1, dtype='int16', device=None)
        sd.wait()
        
        max_vol = np.abs(recording).max()
        print(f"✅ 녹음 완료 (신호 강도: {max_vol})")
        
        if max_vol < 500: # 최소 500~1000은 넘어야 목소리가 들리는 상태입니다.
            print("⚠️ 아직 소리가 너무 작습니다! 설정에서 'Internal Microphone'을 선택했는지 확인하세요.")
            
        write(FILENAME, FS, recording)
    except Exception as e:
        print(f"❌ 녹음 에러: {e}")

def process_voice_ai():
    try:
        # [Step 1] STT: Whisper 호출 시 한국어 설정을 강제합니다.
        with open(FILENAME, "rb") as audio_file:
            transcript = client.audio.transcriptions.create(
                model="whisper-1", 
                file=audio_file,
                language="ko" # 한국어 인식률 향상
            )
        user_text = transcript.text
        print(f"👤 나: {user_text}")

        if not user_text.strip() or user_text == ". .":
            print("⚠️ 인식이 안 되었습니다. 마이크 설정을 다시 확인해 주세요.")
            return

        # [Step 2] GPT-4o 상담
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": "친절한 케이크 가게 사장님입니다. 다정하게 2-3문장으로 답하세요."},
                {"role": "user", "content": user_text}
            ]
        )
        ai_text = response.choices[0].message.content
        print(f"🍰 사장님: {ai_text}")

        # [Step 3] TTS: 음성 생성 및 재생
        speech_response = client.audio.speech.create(
            model="tts-1",
            voice="nova",
            input=ai_text
        )
        
        audio_data = io.BytesIO(speech_response.content)
        audio_segment = AudioSegment.from_file(audio_data, format="mp3")
        
        print("🔊 사장님 목소리 재생 중...")
        play(audio_segment)

    except Exception as e:
        print(f"❌ 에러 발생: {e}")

if __name__ == "__main__":
    print("=== 드디어 마이크 잡는 날! 케이크 상담원 테스트 ===")
    while True:
        input("\n[Enter] 키를 누르고 녹음을 시작하세요.")
        record_audio()
        process_voice_ai()