#!/usr/bin/env python3
import rclpy
import DR_init
import time
import threading
import firebase_admin
from firebase_admin import credentials, db
from rclpy.executors import MultiThreadedExecutor

# ============================
# 설정 구간
# ============================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight2" 
ROBOT_TCP = "GripperDA_v2"

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# Firebase 설정
FIREBASE_JSON = "/home/jejun/cobot1_ws/src/doosan-robot2/dsr_rokey2/dsr_rokey2/rokey-ad6ec-firebase-adminsdk-fbsvc-848d4f8c3c.json"
DATABASE_URL =  "https://rokey-ad6ec-default-rtdb.asia-southeast1.firebasedatabase.app"

# 전역 변수
task_lock = threading.Lock() # 작업 중복 실행 방지용 락

# ============================
# 1. 유틸리티 함수
# ============================

def get_latest_pending_order(orders_data):
    """
    딕셔너리 형태의 주문 데이터에서 
    1. status가 'pending'이고
    2. type이 '기본도안' (ID 3,4) 또는 '커스텀 도안'인
    주문 중 '가장 최신(키 값이 큰)' 주문의 Key와 Value를 반환합니다.
    """
    if not orders_data:
        return None, None

    pending_orders = []

    for key, val in orders_data.items():
        if not isinstance(val, dict): continue
        
        status = val.get('status')
        o_type = val.get('type')
        d_id = str(val.get('design_id'))

        # 조건 확인
        is_basic = (status == 'pending' and o_type == '기본도안' and d_id in ['3', '4'])
        is_custom = (status == 'pending' and o_type == '커스텀도안')

        if is_basic or is_custom:
            # 키를 정수로 변환하여 리스트에 추가 (정렬을 위해)
            try:
                int_key = int(key)
                pending_orders.append((int_key, key, val))
            except ValueError:
                continue

    # 대기 중인 주문이 없으면 종료
    if not pending_orders:
        return None, None

    # 키(int_key)를 기준으로 내림차순 정렬 (가장 큰 값이 0번 인덱스)
    pending_orders.sort(key=lambda x: x[0], reverse=True)

    # 가장 최신 주문 반환
    latest_key_str = pending_orders[0][1]
    latest_val = pending_orders[0][2]
    
    return latest_key_str, latest_val

# ============================
# 2. 로봇 동작 함수들
# ============================

def execute_cake_pickup():
    """
    케이크를 픽업대로 이동시키는 함수
    """
    from DSR_ROBOT2 import (
        posx, movel, set_digital_output, wait,
        set_singular_handling, set_velj, set_accj, set_velx, set_accx,
        DR_AVOID, ON, OFF, DR_MV_MOD_ABS, DR_MV_RA_DUPLICATE
    )

    def grip_open():
        print("Gripper Open (Pickup)")
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

    def grip_close():
        print("Gripper Close (Pickup)")
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1)

    print("🍰 [Pickup] 케이크 픽업대로 이동 시작")

    # 설정
    set_singular_handling(DR_AVOID)
    set_velj(100.0)
    set_accj(500.0)
    set_velx(150.0, 72.375)
    set_accx(500.0, 289.5)

    # 동작 시퀀스
    grip_open()
    
    # 접근
    movel(posx(0.00, 0.00, -50.00, 39.02, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(7.93, -250.39, -113.51, 35.27, 90.97, 5.45), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(7.93, -250.39, -312.36, 35.27, 90.97, 5.45), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(31.15, -164.25, -312.36, 47.15, 89.13, 5.45), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    
    # 잡기
    grip_close()
    
    # 이동
    movel(posx(37.75, -170.33, -187.51, 45.26, 85.70, 15.32), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(313.29, -503.05, -250.60, 149.61, -87.60, -174.77), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(313.29, -503.05, -307.60, 149.61, -87.60, -174.77), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    
    # 놓기
    grip_open()
    
    # 복귀
    movel(posx(260.29, -503.05, -307.58, 149.65, -80.06, -176.41), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(260.29, -503.05, -207.58, 149.65, -80.06, -176.41), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(0.00, 0.00, -50.00, 39.02, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    print("✅ [Pickup] 케이크 픽업 완료")

def execute_powder():
    """
    Powder 작업 실행 함수
    """
    from DSR_ROBOT2 import (
        posx, movel, set_digital_output, wait,
        set_singular_handling, set_velj, set_accj, set_velx, set_accx,
        DR_AVOID, ON, OFF, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE,
        move_periodic
    )

    def grip_open():
        print("Gripper Open (Powder)")
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

    def grip_close():
        print("Gripper Close (Powder)")
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1)

    print("🧂 [Powder] 파우더 프로세스 시작")

    set_singular_handling(DR_AVOID)
    set_velj(100.0)
    set_accj(500.0)
    set_velx(250.0, 80.625) 
    set_accx(1000.0, 322.5)

    grip_open()
    
    movel(posx(0.00, 0.00, -79.07, 68.89, 98.88, -89.62), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(249.06, 339.09, -79.06, 68.89, 98.88, -89.62), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(275.85, 401.24, -71.07, 67.75, 95.65, -89.23), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    
    grip_close()
    
    movel(posx(275.85, 401.24, 0.00, 67.75, 95.65, -89.23), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(0.00, 0.00, 100.00, 39.02, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    
    movel(posx(75.00, 0.00, 100.00, 39.02, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    
    movel(posx(75.00, -75.00, 100.00, 39.02, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    
    movel(posx(0.00, -75.00, 100.00, 39.02, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    
    movel(posx(-75.00, -75.00, 100.00, 39.02, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    
    movel(posx(-75.00, 0.00, 100.00, 39.02, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    
    movel(posx(-75.00, 75.00, 100.00, 39.02, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    
    movel(posx(0.00, 75.00, 100.00, 39.02, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    
    movel(posx(75.00, 75.00, 100.00, 39.02, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    
    movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
    movel(posx(275.86, 401.23, -0.03, 67.75, 95.65, -89.23), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(275.85, 401.23, -68.05, 67.75, 95.65, -89.23), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    
    grip_open()

    print("✅ [Powder] 파우더 프로세스 완료")

def execute_custom_design(line_data):
    """
    커스텀 도안 실행 함수
    """
    from DSR_ROBOT2 import (
        posx, movel, movesx,
        set_digital_output, wait,
        set_singular_handling,
        set_velj, set_accj, set_velx, set_accx,
        task_compliance_ctrl, release_compliance_ctrl,
        set_stiffnessx, set_desired_force, release_force,
        DR_AVOID, ON, OFF,
        DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_FC_MOD_ABS, DR_MV_RA_DUPLICATE
    )

    def grip_open():
        print("Gripper Open")
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

    def grip_close():
        print("Gripper Close")
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1)

    print("🎨 [Custom Design] 커스텀 도안 드로잉 시작")

    if not line_data:
        print("⚠️ [Custom Design] 그릴 데이터(line_data)가 없습니다.")
        return

    # 설정 값
    FIXED_RX, FIXED_RY, FIXED_RZ = 39.03, 89.35, -85.59
    Z_HOP_HEIGHT = 50.0 

    set_singular_handling(DR_AVOID)
    set_velj(30.0); set_accj(50.0)
    set_velx(65.0, 33.0); set_accx(250.0, 130.0)

    grip_open()
    movel(posx(156.51, 421.10, -82.00, 79.23, 89.05, -87.16), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    grip_close()
    movel(posx(156.51, 421.10, -30.00, 79.23, 89.05, -87.16), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    for line in line_data:
        if not line: continue
        first_pt = line[0]
        p0_x = float(first_pt.get('x', 0)) if isinstance(first_pt, dict) else float(first_pt[0])
        p0_y = float(first_pt.get('y', 0)) if isinstance(first_pt, dict) else float(first_pt[1])
        p0_z = float(first_pt.get('z', 0)) if isinstance(first_pt, dict) else float(first_pt[2])

        movel(posx(0, 0, Z_HOP_HEIGHT, 0, 0, 0), radius=0, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        movel(posx(p0_x, p0_y, p0_z + Z_HOP_HEIGHT, FIXED_RX, FIXED_RY, FIXED_RZ), radius=0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(p0_x, p0_y, p0_z, FIXED_RX, FIXED_RY, FIXED_RZ), radius=0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

        task_compliance_ctrl()
        set_stiffnessx([3000.00, 3000.00, 2.50, 200.00, 200.00, 200.00], time=0.0)
        set_desired_force([0.00, 0.00, -4.70, 0.00, 0.00, 0.00], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)

        for pt in line[1:]:
            pt_x = float(pt.get('x', 0)) if isinstance(pt, dict) else float(pt[0])
            pt_y = float(pt.get('y', 0)) if isinstance(pt, dict) else float(pt[1])
            pt_z = float(pt.get('z', 0)) if isinstance(pt, dict) else float(pt[2])
            movel(posx(pt_x, pt_y, pt_z, FIXED_RX, FIXED_RY, FIXED_RZ), radius=1.5, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

        release_compliance_ctrl()
        release_force(time=0.0)
        movel(posx(0, 0, Z_HOP_HEIGHT, 0, 0, 0), radius=0, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)

    movel(posx(156.51, 421.11, -30.03, 79.23, 89.05, -87.16), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(156.51, 421.10, -80.00, 79.23, 89.05, -87.16), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    grip_open()
    print("✅ [Custom Design] 작업 완료")

def execute_design_3():
    """
    Design ID 3 (기본도안)
    """
    from DSR_ROBOT2 import (
        posx, movel, movec, movesx, set_digital_output, wait,
        set_singular_handling, set_velj, set_accj, set_velx, set_accx,
        task_compliance_ctrl, release_compliance_ctrl, set_stiffnessx,
        set_desired_force, release_force, DR_AVOID, ON, OFF,
        DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_ORI_FIXED, DR_MV_ORI_TEACH,
        DR_FC_MOD_ABS, DR_MV_RA_DUPLICATE
    )

    def grip_open():
        set_digital_output(1, ON); set_digital_output(2, OFF); wait(0.5)
    def grip_close():
        set_digital_output(1, OFF); set_digital_output(2, ON); wait(0.5)

    print("🎨 [Design 3] 드로잉 프로세스 시작")
    set_singular_handling(DR_AVOID)
    set_velj(30.0); set_accj(50.0)
    set_velx(65.0, 33.0); set_accx(250.0, 130.0)

    grip_open()
    movel(posx(156.51, 421.10, -82.00, 79.23, 89.05, -87.16), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    grip_close()
    
    # ... (기존 좌표 유지) ...
    movel(posx(156.51, 421.10, -30.00, 79.23, 89.05, -87.16), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-50.00, -30.00, -29.98, 39.02, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-50.00, -30.02, -155.00, 39.02, 89.35, -85.60), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    task_compliance_ctrl()
    set_stiffnessx([3000.00, 3000.00, 2.50, 200.00, 200.00, 200.00], time=0.0)
    set_desired_force([0.00, 0.00, -4.80, 0.00, 0.00, 0.00], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)

    movesx([
        posx(-49.99, -30.03, -155.00, 39.02, 89.35, -85.60), 
        posx(-58.50, -25.50, -154.99, 39.02, 89.35, -85.59), 
        posx(-67.00, -3.50, -154.96, 39.03, 89.35, -85.59), 
        posx(-71.50, 20.00, -154.96, 39.03, 89.35, -85.59), 
        posx(-70.00, 35.00, -154.98, 39.03, 89.35, -85.59), 
        posx(-64.00, 39.00, -154.98, 39.03, 89.35, -85.59), 
        posx(-70.00, 35.00, -154.98, 39.03, 89.35, -85.59), 
        posx(-79.00, 34.00, -154.98, 39.03, 89.35, -85.59), 
        posx(-87.50, 35.00, -154.98, 39.03, 89.35, -85.59), 
        posx(-92.50, 41.00, -154.98, 39.03, 89.35, -85.59), 
        posx(-93.00, 49.00, -154.98, 39.03, 89.35, -85.59), 
        posx(-77.50, 66.00, -154.98, 39.03, 89.35, -85.59), 
        posx(-55.00, 72.00, -154.98, 39.03, 89.35, -85.59), 
        posx(-39.00, 79.00, -154.98, 39.03, 89.35, -85.59), 
        posx(-22.00, 82.50, -154.98, 39.03, 89.35, -85.59), 
        posx(-1.50, 83.50, -154.98, 39.03, 89.35, -85.59)
    ], ref=101)

    movesx([
        posx(0.00, 83.00, -154.96, 39.03, 89.35, -85.59), 
        posx(12.50, 81.80, -154.96, 39.03, 89.35, -85.59), 
        posx(24.60, 84.70, -154.96, 39.03, 89.35, -85.59), 
        posx(35.30, 86.00, -154.96, 39.03, 89.35, -85.59), 
        posx(46.20, 83.90, -154.96, 39.03, 89.35, -85.59), 
        posx(57.10, 76.70, -154.96, 39.03, 89.35, -85.59), 
        posx(63.50, 63.60, -154.96, 39.03, 89.35, -85.59), 
        posx(57.20, 55.00, -154.96, 39.03, 89.35, -85.59), 
        posx(41.50, 53.00, -154.96, 39.03, 89.35, -85.59), 
        posx(28.00, 57.10, -154.96, 39.03, 89.35, -85.59), 
        posx(41.50, 53.00, -154.96, 39.03, 89.35, -85.59), 
        posx(49.40, 27.00, -154.96, 39.03, 89.35, -85.59), 
        posx(52.00, 14.70, -154.96, 39.03, 89.35, -85.59), 
        posx(51.00, 0.00, -154.96, 39.03, 89.35, -85.59), 
        posx(43.50, -14.40, -154.96, 39.03, 89.35, -85.59), 
        posx(52.20, -28.40, -154.96, 39.03, 89.35, -85.59), 
        posx(58.00, -41.80, -154.96, 39.03, 89.35, -85.59), 
        posx(60.50, -59.50, -154.96, 39.03, 89.35, -85.59), 
        posx(58.50, -72.00, -154.96, 39.03, 89.35, -85.59), 
        posx(54.40, -79.00, -154.96, 39.03, 89.35, -85.59), 
        posx(45.00, -84.50, -154.96, 39.03, 89.35, -85.59), 
        posx(33.40, -86.10, -154.96, 39.03, 89.35, -85.59), 
        posx(16.70, -83.20, -154.96, 39.03, 89.35, -85.59), 
        posx(18.70, -77.60, -154.96, 39.03, 89.35, -85.59), 
        posx(19.40, -72.60, -154.96, 39.03, 89.35, -85.59), 
        posx(18.70, -77.60, -154.96, 39.03, 89.35, -85.59), 
        posx(16.70, -83.20, -154.96, 39.03, 89.35, -85.59), 
        posx(10.00, -86.00, -154.96, 39.03, 89.35, -85.59), 
        posx(3.20, -82.60, -154.96, 39.03, 89.35, -85.59), 
        posx(3.20, -74.00, -154.96, 39.03, 89.35, -85.59), 
        posx(3.20, -82.60, -154.96, 39.03, 89.35, -85.59), 
        posx(-6.90, -86.01, -154.91, 39.03, 89.35, -85.60), 
        posx(-13.50, -83.20, -154.93, 39.03, 89.35, -85.59), 
        posx(-15.50, -77.58, -154.96, 39.03, 89.35, -85.59), 
        posx(-16.20, -72.56, -154.94, 39.03, 89.35, -85.59), 
        posx(-15.50, -77.61, -154.95, 39.03, 89.35, -85.59), 
        posx(-13.50, -83.22, -154.95, 39.03, 89.35, -85.59), 
        posx(-23.51, -89.08, -154.98, 39.02, 89.35, -85.59), 
        posx(-34.99, -89.00, -154.96, 39.03, 89.35, -85.59)
    ], ref=101)

    movesx([
        posx(-45.00, -87.50, -154.96, 39.03, 89.35, -85.59), 
        posx(-55.00, -83.50, -155.00, 39.03, 89.35, -85.59), 
        posx(-63.00, -76.50, -155.00, 39.03, 89.35, -85.59), 
        posx(-66.00, -68.50, -155.00, 39.03, 89.35, -85.59), 
        posx(-64.00, -60.50, -155.00, 39.03, 89.35, -85.59), 
        posx(-61.00, -53.00, -155.00, 39.03, 89.35, -85.59), 
        posx(-55.00, -41.00, -155.00, 39.03, 89.35, -85.59), 
        posx(-50.00, -30.00, -155.00, 39.03, 89.35, -85.59), 
        posx(-35.00, -34.00, -155.00, 39.03, 89.35, -85.59), 
        posx(-22.50, -33.50, -155.00, 39.03, 89.35, -85.59), 
        posx(0.00, -32.90, -155.00, 39.03, 89.35, -85.59), 
        posx(29.50, -22.70, -155.00, 39.03, 89.35, -85.59), 
        posx(43.50, -14.40, -155.00, 39.03, 89.35, -85.59), 
        posx(45.00, -12.50, -154.96, 39.03, 89.35, -85.60)
    ], ref=101)

    wait(0.50)
    release_compliance_ctrl()
    release_force(time=0.0)

    movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-64.00, -62.49, -100.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-64.00, -62.48, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    task_compliance_ctrl()
    set_stiffnessx([3000.00, 3000.00, 2.50, 200.00, 200.00, 200.00], time=0.0)
    set_desired_force([0.00, 0.00, -4.50, 0.00, 0.00, 0.00], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)

    movesx([
        posx(-64.00, -62.50, -155.00, 39.03, 89.35, -85.59), 
        posx(-72.00, -60.50, -154.00, 39.03, 89.35, -85.59), 
        posx(-82.50, -58.50, -155.00, 39.03, 89.35, -85.59), 
        posx(-90.00, -55.50, -155.00, 39.03, 89.35, -85.59), 
        posx(-92.00, -61.50, -155.00, 39.03, 89.35, -85.59), 
        posx(-89.00, -68.00, -155.00, 39.03, 89.35, -85.59), 
        posx(-82.00, -73.00, -155.00, 39.03, 89.35, -85.59), 
        posx(-74.00, -72.00, -155.00, 39.03, 89.35, -85.59), 
        posx(-70.00, -76.50, -155.00, 39.03, 89.35, -85.59), 
        posx(-63.00, -76.50, -155.00, 39.03, 89.35, -85.59)
    ], ref=101)

    release_compliance_ctrl()
    release_force(time=0.0)

    movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-43.00, 40.00, -100.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-43.00, 40.00, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    task_compliance_ctrl()
    set_stiffnessx([3000.00, 3000.00, 2.50, 200.00, 200.00, 200.00], time=0.0)
    set_desired_force([0.00, 0.00, -4.50, 0.00, 0.00, 0.00], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)

    movel(posx(-43.00, 40.01, -155.01, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movec(posx(-38.24, 31.75, -155.00, 39.03, 89.35, -85.59), posx(-47.76, 31.75, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE, ori=DR_MV_ORI_FIXED)
    movec(posx(-39.24, 32.75, -155.00, 39.03, 89.35, -85.59), posx(-46.76, 32.75, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE, ori=DR_MV_ORI_FIXED)
    movec(posx(-40.24, 33.75, -155.00, 39.03, 89.35, -85.59), posx(-45.76, 33.75, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE, ori=DR_MV_ORI_FIXED)
    movec(posx(-41.24, 34.75, -155.00, 39.03, 89.35, -85.59), posx(-44.76, 34.75, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE, ori=DR_MV_ORI_FIXED)
    movec(posx(-42.24, 35.76, -155.01, 39.03, 89.35, -85.59), posx(-43.78, 35.77, -155.01, 39.03, 89.35, -85.59), radius=0.00, ref=101, angle=[0.00, 0.00], ra=DR_MV_RA_DUPLICATE, ori=DR_MV_ORI_TEACH)

    release_compliance_ctrl()
    release_force(time=0.0)

    movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
    movel(posx(17.00, 47.30, -100.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(17.00, 47.30, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    task_compliance_ctrl()
    set_stiffnessx([3000.00, 3000.00, 2.50, 200.00, 200.00, 200.00], time=0.0)
    set_desired_force([0.00, 0.00, -4.50, 0.00, 0.00, 0.00], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)

    movel(posx(17.00, 47.30, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movec(posx(21.76, 39.05, -155.00, 39.03, 89.35, -85.59), posx(12.24, 39.05, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE, ori=DR_MV_ORI_FIXED)
    movec(posx(20.76, 40.05, -155.00, 39.03, 89.35, -85.59), posx(13.24, 40.05, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE, ori=DR_MV_ORI_FIXED)
    movec(posx(19.76, 41.05, -155.00, 39.03, 89.35, -85.59), posx(14.24, 41.05, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE, ori=DR_MV_ORI_FIXED)
    movec(posx(18.76, 42.05, -155.00, 39.03, 89.35, -85.59), posx(15.24, 42.05, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE, ori=DR_MV_ORI_FIXED)
    movec(posx(17.76, 43.05, -155.00, 39.03, 89.35, -85.59), posx(16.24, 43.05, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE, ori=DR_MV_ORI_FIXED)

    release_compliance_ctrl()
    release_force(time=0.0)

    movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-9.10, -5.00, -100.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-9.10, -5.00, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    task_compliance_ctrl()
    set_stiffnessx([3000.00, 3000.00, 2.50, 200.00, 200.00, 200.00], time=0.0)
    set_desired_force([0.00, 0.00, -4.50, 0.00, 0.00, 0.00], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)

    movel(posx(-2.00, -4.00, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    
    wait(0.50)

    release_compliance_ctrl()
    release_force(time=0.0)

    movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-15.00, -11.00, -100.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-15.00, -11.00, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    task_compliance_ctrl()
    set_stiffnessx([3000.00, 3000.00, 2.50, 200.00, 200.00, 200.00], time=0.0)
    set_desired_force([0.00, 0.00, -4.50, 0.00, 0.00, 0.00], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)

    movesx([
        posx(-15.00, -11.00, -155.00, 39.03, 89.35, -85.59), 
        posx(-12.00, -14.30, -155.00, 39.03, 89.35, -85.59), 
        posx(-9.10, -15.50, -155.00, 39.03, 89.35, -85.59), 
        posx(-6.40, -14.50, -155.00, 39.03, 89.35, -85.59), 
        posx(-5.00, -12.50, -154.97, 39.04, 89.35, -85.60), 
        posx(-2.00, -14.00, -155.00, 39.03, 89.35, -85.59), 
        posx(0.00, -14.50, -155.00, 39.03, 89.35, -85.59), 
        posx(2.60, -12.70, -155.00, 39.03, 89.35, -85.59), 
        posx(6.00, -11.00, -155.00, 39.03, 89.35, -85.59)
    ], ref=101)

    release_compliance_ctrl()
    release_force(time=0.0)

    movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
    movel(posx(156.51, 421.11, -30.03, 79.23, 89.05, -87.16), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(156.51, 421.10, -80.00, 79.23, 89.05, -87.16), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    
    grip_open()

    print("✅ [Design 3] 드로잉 프로세스 완료")

def execute_design_4():
    """
    Design ID 4 (기본도안)
    """
    from DSR_ROBOT2 import (
        posx, movej, movel, movec, movesx,
        set_user_cart_coord, DR_BASE,
        set_digital_output, wait,
        set_singular_handling,
        set_velj, set_accj, set_velx, set_accx,
        task_compliance_ctrl, release_compliance_ctrl,
        set_stiffnessx,
        set_desired_force, release_force,
        DR_AVOID, ON, OFF,
        DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_ORI_FIXED, DR_FC_MOD_ABS,
        DR_MV_RA_DUPLICATE
    )

    def grip_open():
        set_digital_output(1, ON); set_digital_output(2, OFF); wait(0.5)
    def grip_close():
        set_digital_output(1, OFF); set_digital_output(2, ON); wait(0.5)

    print("🎨 [Design 4] 드로잉 프로세스 시작")
    set_singular_handling(DR_AVOID)
    set_velj(50.0); set_accj(80.0)
    set_velx(125.0, 80.625); set_accx(500.0, 322.5)

    grip_open()
    movel(posx(156.51, 421.10, -82.00, 79.23, 89.05, -87.16), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    grip_close()
    
    movel(posx(156.51, 421.10, -30.00, 79.23, 89.05, -87.16), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-27.51, -20.59, -29.98, 39.02, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-27.50, -20.50, -153.00, 39.03, 89.35, -85.58), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    task_compliance_ctrl()
    set_stiffnessx([3000.00, 3000.00, 2.50, 200.00, 200.00, 200.00], time=0.0)
    set_desired_force([0.00, 0.00, -4.50, 0.00, 0.00, 0.00], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)

    movesx([
        posx(-27.50, -20.50, -154.00, 39.03, 89.35, -85.58),
        posx(-32.20, -13.50, -154.96, 39.03, 89.35, -85.59),
        posx(-33.50, 23.70, -154.96, 39.03, 89.35, -85.59),
        posx(-30.80, 33.10, -154.96, 39.03, 89.35, -85.59),
        posx(-42.90, 37.00, -154.96, 39.03, 89.35, -85.59),
        posx(-41.90, 50.00, -154.96, 39.03, 89.35, -85.59),
        posx(-34.10, 52.30, -154.96, 39.03, 89.35, -85.59),
        posx(-26.30, 51.10, -154.96, 39.03, 89.35, -85.59),
        posx(-20.20, 46.00, -154.98, 39.03, 89.35, -85.59),
        posx(-15.60, 51.00, -154.98, 39.03, 89.35, -85.59),
        posx(-8.30, 54.20, -154.98, 39.03, 89.35, -85.59),
        posx(-1.10, 54.40, -154.98, 39.03, 89.35, -85.59),
        posx(7.80, 53.10, -154.98, 39.03, 89.35, -85.59),
        posx(23.20, 48.00, -154.98, 39.03, 89.35, -85.59),
        posx(29.70, 50.80, -154.98, 39.03, 89.35, -85.59),
        posx(45.30, 43.20, -154.98, 39.03, 89.35, -85.59),
        posx(41.20, 34.50, -154.98, 39.03, 89.35, -85.59),
        posx(30.00, 29.80, -154.98, 39.03, 89.35, -85.59),
        posx(33.20, 8.40, -154.98, 39.03, 89.35, -85.59),
        posx(28.20, -19.00, -154.98, 39.03, 89.35, -85.59),
        posx(39.70, -20.20, -154.98, 39.03, 89.35, -85.59),
        posx(45.00, -25.80, -154.98, 39.03, 89.35, -85.59),
        posx(42.90, -31.80, -154.98, 39.03, 89.35, -85.59),
        posx(32.10, -33.50, -154.98, 39.03, 89.35, -85.59),
        posx(31.60, -60.00, -154.98, 39.03, 89.35, -85.59),
        posx(20.50, -71.00, -154.98, 39.03, 89.35, -85.59),
        posx(8.00, -73.50, -154.98, 39.03, 89.35, -85.59),
        posx(8.00, -85.00, -154.98, 39.03, 89.35, -85.59),
        posx(4.00, -88.80, -154.98, 39.03, 89.35, -85.59),
        posx(-0.80, -84.00, -154.98, 39.03, 89.35, -85.59),
        posx(-0.80, -79.20, -154.98, 39.03, 89.35, -85.59),
        posx(-0.81, -84.01, -154.97, 39.03, 89.35, -85.59),
        posx(-7.00, -90.00, -154.98, 39.03, 89.35, -85.59),
        posx(-10.60, -78.00, -154.98, 39.03, 89.35, -85.59),
        posx(-19.60, -74.00, -154.98, 39.03, 89.35, -85.59),
        posx(-26.20, -79.80, -154.98, 39.03, 89.35, -85.59),
        posx(-37.00, -79.40, -154.98, 39.03, 89.35, -85.59),
        posx(-41.20, -71.00, -154.98, 39.03, 89.35, -85.59),
        posx(-25.00, -69.00, -154.98, 39.03, 89.35, -85.59),
        posx(-29.50, -53.00, -154.98, 39.03, 89.35, -85.59),
        posx(-31.80, -35.20, -154.98, 39.03, 89.35, -85.59),
        posx(-44.70, -28.50, -154.98, 39.03, 89.35, -85.59),
        posx(-42.80, -23.00, -154.98, 39.03, 89.35, -85.59),
        posx(-27.50, -20.50, -154.98, 39.03, 89.35, -85.59),
        posx(0.00, -25.00, -154.98, 39.03, 89.35, -85.59),
        posx(28.20, -19.00, -154.98, 39.03, 89.35, -85.59)
    ], ref=101)

    release_compliance_ctrl()
    release_force(time=0.0)

    movel(posx(536.20, -173.00, 300.02, 39.03, 89.35, -85.59), radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    movel(posx(20.20, 25.50, -99.98, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(20.21, 25.50, -155.00, 39.03, 89.35, -85.60), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    task_compliance_ctrl()
    set_stiffnessx([3000.00, 3000.00, 2.50, 200.00, 200.00, 200.00], time=0.0)
    set_desired_force([0.00, 0.00, -3.50, 0.00, 0.00, 0.00], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)

    movec(posx(12.70, 21.17, -155.00, 39.02, 89.35, -85.59), posx(12.70, 29.83, -155.00, 39.02, 89.35, -85.59), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
    movec(posx(13.70, 22.17, -155.00, 39.02, 89.35, -85.59), posx(13.70, 28.83, -155.00, 39.02, 89.35, -85.59), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
    movec(posx(14.70, 23.17, -155.00, 39.02, 89.35, -85.59), posx(14.70, 27.83, -155.00, 39.02, 89.35, -85.59), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
    movec(posx(15.70, 24.17, -155.00, 39.02, 89.35, -85.59), posx(15.70, 26.83, -155.00, 39.02, 89.35, -85.59), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)

    release_compliance_ctrl()
    release_force(time=0.0)

    movel(posx(15.00, 31.00, -99.98, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-23.20, 25.50, -99.98, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-23.21, 25.50, -155.00, 39.03, 89.35, -85.60), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    task_compliance_ctrl()
    set_stiffnessx([3000.00, 3000.00, 2.50, 200.00, 200.00, 200.00], time=0.0)
    set_desired_force([0.00, 0.00, -3.50, 0.00, 0.00, 0.00], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)

    movec(posx(-15.70, 29.83, -155.01, 39.03, 89.35, -85.59), posx(-15.70, 21.17, -155.00, 39.03, 89.35, -85.59), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
    movec(posx(-16.70, 28.83, -155.01, 39.03, 89.35, -85.59), posx(-16.70, 22.17, -155.00, 39.03, 89.35, -85.59), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
    movec(posx(-17.70, 27.83, -155.01, 39.03, 89.35, -85.59), posx(-17.70, 23.17, -155.00, 39.03, 89.35, -85.59), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
    movec(posx(-18.70, 26.83, -155.01, 39.03, 89.35, -85.59), posx(-18.70, 24.17, -155.00, 39.03, 89.35, -85.59), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)

    release_compliance_ctrl()
    release_force(time=0.0)

    movel(posx(15.00, 31.00, -99.98, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(0.00, -8.00, -99.98, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(0.00, -8.00, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    task_compliance_ctrl()
    set_stiffnessx([3000.00, 3000.00, 2.50, 200.00, 200.00, 200.00], time=0.0)
    set_desired_force([0.00, 0.00, -4.00, 0.00, 0.00, 0.00], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)

    movel(posx(0.00, -8.00, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    
    release_compliance_ctrl()
    release_force(time=0.0)

    movel(posx(0.01, -7.95, -99.99, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-10.00, -15.00, -99.98, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-10.00, -15.00, -155.00, 39.03, 89.35, -85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    task_compliance_ctrl()
    set_stiffnessx([3000.00, 3000.00, 2.50, 200.00, 200.00, 200.00], time=0.0)
    set_desired_force([0.00, 0.00, -4.00, 0.00, 0.00, 0.00], [0, 0, 1, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS)

    movel(posx(-10.00, -14.99, -154.96, 39.03, 89.35, -85.60), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movec(posx(0.00, -13.50, -154.96, 39.03, 89.35, -85.60), posx(-6.30, -5.58, -154.96, 39.03, 89.35, -85.60), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[130.00, 0.00], ra=DR_MV_RA_DUPLICATE)

    wait(0.50)

    movel(posx(0.00, -13.51, -154.96, 39.03, 89.35, -85.60), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movec(posx(10.00, -15.00, -154.96, 39.03, 89.35, -85.60), posx(6.30, -5.58, -155.00, 39.03, 89.35, -85.60), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[150.00, 0.00], ra=DR_MV_RA_DUPLICATE)

    release_compliance_ctrl()
    release_force(time=0.0)

    movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
    movel(posx(156.51, 421.11, -30.03, 79.23, 89.05, -87.16), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(156.51, 421.10, -80.00, 79.23, 89.05, -87.16), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    grip_open()
    print("✅ [Design 4] 드로잉 프로세스 완료")


# ============================
# 3. 로봇 작업 실행 매니저
# ============================

def perform_task(order_key, order_data):
    """
    주문 정보를 분석하여 적절한 드로잉 함수를 실행하고,
    'powder' 주문이 존재하면 후처리로 실행합니다.
    """
    from DSR_ROBOT2 import set_tool, set_tcp

    if not task_lock.acquire(blocking=False):
        print(f"⚠️ 이미 다른 작업이 진행 중입니다. 주문 {order_key} 대기.")
        return

    try:
        print(f"\n🚀 [START] 주문 {order_key} 작업 시작...")
        # 1. 상태 업데이트
        db.reference(f"orders/{order_key}").update({"status": "processing"})

        # 2. 로봇 설정
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)

        # 3. 도안 그리기 실행
        order_type = order_data.get('type')
        design_id = str(order_data.get('design_id'))
        line_data = order_data.get('drawing_path', [])

        if order_type == '기본도안' and design_id == '4':
            execute_design_4()
        elif order_type == '기본도안' and design_id == '3':
            execute_design_3()
        elif order_type == '커스텀도안':
            execute_custom_design(line_data)
        else:
            print(f"⚠️ 알 수 없는 도안입니다: Type={order_type}, ID={design_id}")
            pass

        # 4. Powder 로직
        try:
            print("🔎 Powder 주문 확인 중...")
            all_orders = db.reference('orders').get()
            need_powder = False
            
            if all_orders:
                for k, v in all_orders.items():
                    if not isinstance(v, dict): continue
                    
                    # Powder 체크 (사용자 로직 유지)
                    powder_val = str(v.get('powder')).lower()
                    if powder_val == "choco_powder" or powder_val == "sugar_powder":
                        need_powder = True
                        print(f"🧂 Powder 주문 감지됨 (Key: {k}, Type: {powder_val})")
                        break
            
            if need_powder:
                execute_powder()
            else:
                print("ℹ️ Powder 주문 없음.")
                
        except Exception as e_powder:
            print(f"⚠️ Powder 확인 중 오류 발생: {e_powder}")

        # 5. 케이크 픽업대로 이동 (항상 실행)
        try:
            execute_cake_pickup()
        except Exception as e_pickup:
            print(f"⚠️ 픽업 이동 중 오류 발생: {e_pickup}")

        # 6. 최종 완료 처리
        print(f"✅ [DONE] 주문 {order_key} 최종 완료 처리.")
        db.reference(f"orders/{order_key}").update({"status": "done"})

    except Exception as e:
        print(f"❌ [ERROR] 작업 중 오류 발생: {e}")
        db.reference(f"orders/{order_key}").update({"status": "failed"})
        
    finally:
        task_lock.release()

def initialize_robot():
    """로봇 초기 연결 및 기본 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp
    try:
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        print(f"✅ Robot Initialized: {ROBOT_ID}, {ROBOT_TOOL}, {ROBOT_TCP}")
    except Exception as e:
        print(f"❌ Robot Initialization Failed: {e}")

# ============================
# 4. Firebase 리스너
# ============================

def order_listener(event):
    if event.data is None:
        return

    orders_to_check = {}
    
    if event.path == "/":
        orders_to_check = event.data
    else:
        try:
            orders_to_check = db.reference('orders').get()
        except:
            return

    target_key, target_val = get_latest_pending_order(orders_to_check)

    if target_key:
        print(f"📥 [ORDER FOUND] 최신 주문 감지: Key={target_key}, Status=pending")
        perform_task(target_key, target_val)
    else:
        pass

# ============================
# 5. 메인 함수
# ============================

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("robot_main_controller", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        initialize_robot()

        cred = credentials.Certificate(FIREBASE_JSON)
        firebase_admin.initialize_app(cred, {
            'databaseURL': DATABASE_URL
        })
        print("✅ Firebase Initialized.")

        ref = db.reference("orders")
        print("🔥 Firebase Listening Started... (Waiting for orders)")
        ref.listen(order_listener)

        while rclpy.ok():
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
    except Exception as e:
        print(f"❌ Critical Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()