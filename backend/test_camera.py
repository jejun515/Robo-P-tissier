import cv2

def list_available_cameras(max_check=10):
    """0번부터 max_check번까지 카메라를 확인하여 사용 가능한 리스트 반환"""
    print("\n🔍 연결된 카메라를 검색하고 있습니다... (잠시만 기다려주세요)")
    available_cameras = []
    
    for i in range(max_check):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            # 실제로 프레임이 읽히는지 확인 (더 정확함)
            ret, _ = cap.read()
            if ret:
                print(f"   ✅ 카메라 #{i}: 사용 가능")
                available_cameras.append(i)
            else:
                print(f"   ⚠️ 카메라 #{i}: 감지는 되나 화면이 안 나옴")
            cap.release()
    
    if not available_cameras:
        print("❌ 사용 가능한 카메라를 찾지 못했습니다.")
    
    return available_cameras

def run_camera(cam_index):
    """선택한 카메라 실행"""
    print(f"\n🎥 {cam_index}번 카메라를 시작합니다...")
    print("👉 종료하려면 화면을 클릭하고 'q' 키를 누르세요.")
    
    cap = cv2.VideoCapture(cam_index)
    
    if not cap.isOpened():
        print(f"❌ {cam_index}번 카메라를 열 수 없습니다.")
        return

    # 해상도 출력
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"   - 해상도: {w}x{h}")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 프레임을 읽을 수 없습니다.")
            break
            
        cv2.imshow(f'Camera #{cam_index}', frame)
        
        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()
    print("👋 종료되었습니다.")

if __name__ == "__main__":
    # 1. 카메라 목록 찾기
    cameras = list_available_cameras()
    
    if cameras:
        # 2. 사용자 입력 받기
        while True:
            try:
                print("\n" + "="*30)
                user_input = input(f"📸 사용할 카메라 번호를 입력하세요 {cameras}: ")
                selected_idx = int(user_input)
                
                if selected_idx in cameras:
                    # 3. 선택한 카메라 실행
                    run_camera(selected_idx)
                    break
                else:
                    print("❌ 목록에 있는 번호를 입력해주세요.")
            except ValueError:
                print("❌ 숫자를 입력해주세요.")