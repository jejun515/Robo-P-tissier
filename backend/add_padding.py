import cv2
import numpy as np
import os
import tkinter as tk
from tkinter import filedialog

# ==========================================
# ⚙️ 물리적 크기 설정 (Physical Configuration)
# ==========================================
# 실제 로봇/케이크 환경에 맞춘 설정입니다.
CAKE_DIAMETER_MM = 300.0   # 케이크 작업 영역 지름 (mm)
FIXED_MARGIN_MM = 50.0    # 상하좌우에 줄 고정 여백 크기 (mm)
TARGET_SIZE_PX = 480       # 결과 이미지 픽셀 해상도

def resize_with_padding(image_path, output_path, scale_ratio, target_size=480):
    """
    이미지에 여백을 추가하여 리사이즈하는 함수
    
    Args:
        image_path (str): 원본 이미지 경로
        output_path (str): 저장할 경로
        scale_ratio (float): 전체 캔버스 대비 이미지 크기 비율 (0.0 ~ 1.0)
        target_size (int): 결과 이미지의 가로/세로 크기 (픽셀)
    """
    # 1. 이미지 읽기
    if not os.path.exists(image_path):
        print(f"❌ 오류: 파일을 찾을 수 없습니다 -> {image_path}")
        return

    # 한글 경로 처리 등을 위해 numpy로 읽어서 decode
    img_array = np.fromfile(image_path, np.uint8)
    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    
    if img is None:
        print("❌ 오류: 이미지를 불러올 수 없습니다.")
        return

    h, w = img.shape[:2]
    
    # 2. 흰색 캔버스 생성 (정사각형)
    if target_size is None:
        target_size = max(h, w)
    
    canvas = np.full((target_size, target_size, 3), 255, dtype=np.uint8)
    
    # 3. 리사이즈 계산
    effective_size = int(target_size * scale_ratio)
    
    # 원본 비율 유지하면서 effective_size 안에 들어오도록 스케일 조정
    scale = effective_size / max(h, w)
    new_w = int(w * scale)
    new_h = int(h * scale)
    
    # 이미지 리사이즈
    resized_img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
    
    # 4. 중앙 정렬
    x_off = (target_size - new_w) // 2
    y_off = (target_size - new_h) // 2
    
    # 캔버스 중앙에 이미지 붙여넣기
    canvas[y_off:y_off+new_h, x_off:x_off+new_w] = resized_img
    
    # 5. 결과 저장
    is_success, buffer = cv2.imencode(".png", canvas)
    if is_success:
        with open(output_path, "wb") as f:
            f.write(buffer)
        print(f"\n✅ 변환 성공!")
        print(f"📂 저장 위치: {output_path}")
        print(f"📏 물리적 계산: 케이크 {CAKE_DIAMETER_MM}mm 중 상하좌우 여백 {FIXED_MARGIN_MM}mm 제외")
        print(f"📐 적용 비율: {scale_ratio*100:.1f}% (실제 그림 영역 약 {int(CAKE_DIAMETER_MM * scale_ratio)}mm)")
    else:
        print("❌ 저장 실패")

if __name__ == "__main__":
    print("="*50)
    print("   🎨 이미지 여백 자동 추가 도구 (Robot Safe Zone)")
    print(f"   ⚙️ 설정: 케이크 {CAKE_DIAMETER_MM}mm / 상하좌우 여백 {FIXED_MARGIN_MM}mm")
    print("="*50)
    
    # 1. 파일 선택 (GUI 창 열기)
    print("\n📂 이미지 파일을 선택하세요...")
    root = tk.Tk()
    root.withdraw() # TKinter 메인 윈도우 숨기기
    
    # 창이 맨 앞으로 오게 설정
    root.lift()
    root.attributes('-topmost',True)
    root.after_idle(root.attributes,'-topmost',False)
    
    img_path_input = filedialog.askopenfilename(
        title="변환할 이미지 선택",
        filetypes=[("Image files", "*.jpg *.jpeg *.png *.bmp *.webp")]
    )

    if not img_path_input:
        print("⚠️  파일이 선택되지 않았습니다. 종료합니다.")
        exit()
        
    print(f"✅ 선택된 파일: {img_path_input}")

    # 2. 비율 자동 계산
    # 전체 지름에서 양쪽 여백(상+하 또는 좌+우)을 뺀 크기가 이미지가 들어갈 공간
    # 예: 300mm - (100mm * 2) = 100mm (이미지 영역)
    safe_zone_mm = CAKE_DIAMETER_MM - (FIXED_MARGIN_MM * 2)
    
    if safe_zone_mm <= 0:
        print(f"\n❌ [오류] 여백({FIXED_MARGIN_MM}mm x 2)이 케이크 크기({CAKE_DIAMETER_MM}mm)보다 큽니다!")
        exit()
        
    scale_ratio = safe_zone_mm / CAKE_DIAMETER_MM
    print(f"\n🔄 자동 계산된 비율: {scale_ratio:.4f} (안전 영역 {safe_zone_mm}mm)")

    # 3. 저장 경로 생성
    dir_name = os.path.dirname(img_path_input)
    file_name = os.path.basename(img_path_input)
    name_only, _ = os.path.splitext(file_name)
    
    output_filename = f"padded_{name_only}.png"
    output_path = os.path.join(dir_name, output_filename)

    # 4. 실행
    resize_with_padding(img_path_input, output_path, scale_ratio, target_size=TARGET_SIZE_PX)