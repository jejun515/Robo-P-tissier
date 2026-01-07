#!/usr/bin/env python3
import rclpy
import DR_init
import time
import threading
import sys
import os
import queue
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor 

# ROS Messages & Services
from od_msg.srv import SrvDepthPosition
from dsr_msgs2.srv import (
    GetRobotState, SetRobotMode, SetRobotControl, 
    SetSafeStopResetType, DrlPause, DrlResume
)

from .onrobot import RG
import firebase_admin
from firebase_admin import credentials, db

# ============================
# [NEW] 터미널 색상 클래스
# ============================
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# ============================
# 설정 구간
# ============================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight" 
ROBOT_TCP = "GripperDA_v1"
GRIPPER_NAME = 'rg2'
TOOLCHARGER_IP = '192.168.1.1'
TOOLCHARGER_PORT = '502'

OBJECT_DICT = {1: "strawberry", 2: "blueberry", 3: "mango"}
PLACE_POSES = [
    [0, 130, -285, 84.34, 176.65, 85.22],    # Slot 1
    [100, 100, -285, 84.34, 176.65, 85.22],  # Slot 2
    [130, 0, -285, 84.34, 176.65, 85.22],    # Slot 3
    [100, -80, -285, 84.34, 176.65, 85.22],  # Slot 4
    [0, -120, -285, 84.34, 176.65, 85.22],   # Slot 5
    [-70, -80, -285, 84.34, 176.65, 85.22],  # Slot 6
    [-120, 0, -285, 84.34, 176.65, 85.22],   # Slot 7
    [-70, 100, -285, 84.34, 176.65, 85.22],  # Slot 8
]

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

FIREBASE_JSON = "/home/rokey/ros2_ws/src/doosan-robot2/dsr_rokey/pick_and_place_text/pick_and_place_text/rokey-ad6ec-firebase-adminsdk-fbsvc-848d4f8c3c.json"
DATABASE_URL =  "https://rokey-ad6ec-default-rtdb.asia-southeast1.firebasedatabase.app"

g_node = None                
vision_client = None         
order_queue = queue.Queue()
g_robot_error = False 
g_last_topping_source = None
class RobotStopException(Exception):
    pass

def check_safety():
    global g_robot_error
    
    # 1. 에러가 없으면 그냥 리턴
    if not g_robot_error:
        return

    # 2. 에러 발생 시 로직 진입
    print(f"{Colors.RED}⛔ [STOP] 로봇 에러 감지! 안전 조치 실행 중...{Colors.ENDC}")
    
    try:
        # DB에서 현재 상태 확인
        current_status = db.reference("order_status").get()
        
        # [CASE 1] 시럽 작업 중일 때 -> 흐르지 않게 닫기 (기존 유지)
        if current_status == "syrup":
            print(f"{Colors.YELLOW}🩸 [SAFETY] 시럽 흘림 방지 (Target: 780)...{Colors.ENDC}")
            if gripper:
                gripper.move_gripper(780) 
                time.sleep(0.5)
        
        # [CASE 2] 토핑 작업 중일 때 -> 움직이지 말고 그대로 멈춰라! (수정됨)
        elif current_status in ["topping_pick", "topping_place"]:
            gripper.move_gripper(200) 
            time.sleep(0.5)
            # 기존에는 여기서 move_gripper(200) 같은게 있었을 겁니다.
            # 그것 때문에 그리퍼가 움직인 것이니, 여기서는 아무것도 하지 않게 비워둡니다.
            print(f"{Colors.YELLOW}🛡️ [SAFETY] 토핑 파지 중. 낙하 방지를 위해 그리퍼 상태를 유지합니다.{Colors.ENDC}")
            pass 
            
        # [CASE 3] 그 외 상태
        else:
            print(f"{Colors.YELLOW}🛡️ [SAFETY] 현재 상태({current_status})는 별도 안전 동작이 필요 없습니다.{Colors.ENDC}")

    except Exception as e:
        print(f"⚠️ [WARNING] 비상 그리퍼 동작 실패: {e}")
    
    # 마지막에 에러 던지기 (필수)
    raise RobotStopException("Robot Error Detected")

# def wait_safe(seconds):
#     start_time = time.time()
#     while (time.time() - start_time) < seconds:
#         check_safety()
#         time.sleep(0.05)

def update_process_status(status_text):
    """
    Firebase의 'order_status' 노드를 업데이트합니다.
    예: order_status: "topping" / "powder" / "syrup"
    """
    try:
        db.reference("order_status").set(status_text)
        print(f"📡 [DB Update] 현재 공정 상태 업데이트: {Colors.CYAN}{status_text}{Colors.ENDC}")
    except Exception as e:
        print(f"⚠️ [DB Error] 상태 업데이트 실패: {e}")

# ============================
# [UPDATED] 로봇 상태 모니터링 & 복구 클래스
# ============================
class RobotStateMonitor(Node):
    def __init__(self):
        super().__init__('robot_state_monitor', namespace=ROBOT_ID)
        
        self.cli_state = self.create_client(GetRobotState, f'/{ROBOT_ID}/system/get_robot_state')
        self.cli_control = self.create_client(SetRobotControl, f'/{ROBOT_ID}/system/set_robot_control')
        self.cli_mode = self.create_client(SetRobotMode, f'/{ROBOT_ID}/system/set_robot_mode')
        self.cli_reset_type = self.create_client(SetSafeStopResetType, f'/{ROBOT_ID}/system/set_safe_stop_reset_type')
        self.cli_pause = self.create_client(DrlPause, f'/{ROBOT_ID}/drl/drl_pause')
        self.cli_resume = self.create_client(DrlResume, f'/{ROBOT_ID}/drl/drl_resume')

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.current_state = None
        self.last_state = -1 
        self._is_requesting = False 
        self.is_recovering = False

        self.RESET_TYPE_MODE = 1 

        self.STATE_MAP = {
            0: "INITIALIZING", 1: "STANDBY", 2: "MOVING", 3: "SAFE_OFF",
            4: "TEACHING", 5: "SAFE_STOP", 6: "EMERGENCY_STOP", 7: "HOMING",
            8: "RECOVERY", 9: "SAFE_STOP2", 10: "SAFE_OFF2", 11: "RESERVED"
        }
        self.ERROR_STATES = [3, 5, 6, 9, 10]

        self.RECOVERY_MAP = {
            3: [[3]],          # Safe Off -> Servo On
            5: [[2]],          # Safe Stop -> Reset
            9: [[4, 7]],       # Safe Stop2 -> Recovery Mode(4) -> Reset(7)
            10: [[5, 7], [6, 7]]
        }
        self.CMD_TEXT = {
            2: "RESET", 3: "SERVO_ON", 4: "RECOVERY_SAFE_STOP",
            5: "RECOVERY_SAFE_OFF", 6: "RECOVERY_BACKDRIVE", 7: "RESET_RECOVERY"
        }

        self.get_logger().info("🛡️ Robot Monitor & Recovery System Started")

    def timer_callback(self):
        if not self.cli_state.service_is_ready(): return
        if self._is_requesting: return

        self._is_requesting = True
        req = GetRobotState.Request()
        future = self.cli_state.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        global g_robot_error
        try:
            res = future.result()
            self.current_state = res.robot_state
            
            if not self.is_recovering:
                if self.current_state != self.last_state:
                    state_str = self.STATE_MAP.get(self.current_state, f"UNKNOWN({self.current_state})")
                    
                    if self.current_state in self.ERROR_STATES:
                        self.get_logger().error(f"🚨 [STATE CHANGE] 에러 발생: {state_str}")
                        g_robot_error = True
                    elif self.current_state == 2:
                        self.get_logger().info(f"▶️ [STATE CHANGE] 동작 중: {state_str}")
                    elif self.current_state == 1:
                        self.get_logger().info(f"✅ [STATE CHANGE] 대기 중: {state_str}")
                    
                    self.last_state = self.current_state
        except: pass
        finally: self._is_requesting = False

    def _call_sync(self, client, req, timeout=2.0):
        if not client.wait_for_service(timeout_sec=1.0): return None
        future = client.call_async(req)
        start = time.time()
        while not future.done():
            if time.time() - start > timeout: return None
            time.sleep(0.05)
        return future.result()

    def recover_to_standby(self):
        global g_robot_error
        self.is_recovering = True
        print(f"\n{Colors.YELLOW}🔄 [Recovery] 복구 시퀀스 시작... (Reset Type: {self.RESET_TYPE_MODE}){Colors.ENDC}")
        
        try:
            req_reset = SetSafeStopResetType.Request()
            req_reset.reset_type = self.RESET_TYPE_MODE
            self._call_sync(self.cli_reset_type, req_reset)

            current_state = self.current_state
            strategies = self.RECOVERY_MAP.get(current_state, [])
            
            if not strategies:
                if current_state in self.ERROR_STATES:
                    strategies = [[2, 3]] # Default
                else:
                    print(f"{Colors.GREEN}✅ 에러 상태가 아닙니다 ({current_state}).{Colors.ENDC}")
                    g_robot_error = False
                    self.is_recovering = False
                    return True

            print(f"   📋 상태 {current_state}에 대한 {len(strategies)}가지 전략이 감지되었습니다.")

            strategy_success = False
            for idx, strategy in enumerate(strategies):
                print(f"   🚀 [Plan {idx+1}] 시도 중: 명령 순서 {strategy}")
                plan_ok = True 
                
                for cmd in strategy:
                    cmd_name = self.CMD_TEXT.get(cmd, str(cmd))
                    print(f"      👉 명령 전송: {cmd} ({cmd_name})... ", end="", flush=True)
                    
                    req_ctrl = SetRobotControl.Request()
                    req_ctrl.robot_control = cmd
                    res = self._call_sync(self.cli_control, req_ctrl)
                    
                    if res and res.success:
                        print(f"{Colors.GREEN}성공{Colors.ENDC}")
                        time.sleep(1.5) 
                    else:
                        print(f"{Colors.RED}실패{Colors.ENDC}")
                        plan_ok = False
                        break 
                
                if plan_ok:
                    print(f"   ✨ [Plan {idx+1}] 명령 전송 완료. 상태 확인 중...")
                    time.sleep(2.0)
                    
                    req_st = GetRobotState.Request()
                    res_st = self._call_sync(self.cli_state, req_st)
                    final_state = res_st.robot_state if res_st else self.current_state
                    
                    if final_state == 1:
                        print(f"{Colors.GREEN}✅ [Recovery] Standby 상태 복귀 성공!{Colors.ENDC}")
                        strategy_success = True
                        break 
                    elif final_state not in self.ERROR_STATES and final_state > 0:
                        print(f"{Colors.GREEN}✅ [Recovery] 정상 상태({final_state}) 복귀 성공!{Colors.ENDC}")
                        strategy_success = True
                        break
                    else:
                        print(f"   ⚠️ [Plan {idx+1}] 여전히 에러 상태({final_state})입니다. 다음 전략...")

            if strategy_success:
                print(f"{Colors.GREEN}▶️ [Resume] 작업을 재개합니다.{Colors.ENDC}")
                self._call_sync(self.cli_resume, DrlResume.Request())
                g_robot_error = False
                return True
            else:
                print(f"{Colors.RED}❌ [Final] 모든 복구 전략 실패.{Colors.ENDC}")
                return False

        except Exception as e:
            print(f"{Colors.RED}❌ [Recovery] 예외 발생: {e}{Colors.ENDC}")
            return False
        finally:
            self.is_recovering = False

# ============================
# [NEW] 복구 후 클린업 동작
# ============================
def execute_post_recovery_motion():
    """
    복구 성공 후: 시럽 반납 -> 그리퍼 열기 -> 홈 이동
    """
    from DSR_ROBOT2 import movel, posx, DR_MV_MOD_ABS, DR_MV_RA_DUPLICATE,set_velj,set_accj,set_velx,set_accx
    set_velj(10.0); set_accj(30.0)
    set_velx(60.0, 33.0); set_accx(200.0, 130.0)
    
    print(f"\n{Colors.CYAN}🧹 [Cleanup] 시럽 반납 및 홈 위치로 이동 중...{Colors.ENDC}")
    try:
        # 1. 시럽 반납 위치로 이동
        movel(posx(432.26, 194.39, 0, 23.35, 90, 88.97), ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(432.26, 194.39, -139, 23.35, 90, 88.97), ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        
        # 2. 그리퍼 열기
        print(f"{Colors.CYAN}   👉 그리퍼 오픈 (반납){Colors.ENDC}")
        grip_open()
        
        # 3. 빠져나오기
        movel(posx(374.26, 166.74, -139, 23.35, 90, 88.97), ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        
        # 4. 홈 위치로 이동
        print(f"{Colors.CYAN}   👉 홈 위치로 이동{Colors.ENDC}")
        movel(posx(0,0,0,22.75, 90, 90.62), vel=150, acc=400, radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        
        print(f"{Colors.GREEN}✨ [Cleanup] 초기화 완료.{Colors.ENDC}")
        
    except Exception as e:
        print(f"{Colors.RED}❌ [Cleanup Error] 이동 중 문제 발생: {e}{Colors.ENDC}")


def execute_powder_recovery():
    """
    파우더 작업 중 에러 발생 시, 파우더 통을 원위치에 반납하고 안전하게 빠져나오는 동작
    마지막에 홈 위치로 복귀
    """
    from DSR_ROBOT2 import movel, posx, DR_MV_MOD_ABS, DR_MV_RA_DUPLICATE
    
    print(f"{Colors.BLUE}🔄 [Recovery] 파우더 통 반납(Reset) 동작을 수행합니다...{Colors.ENDC}")

    # 1. 접근 (파우더 거치대 상단)
    movel(posx(273.38, 428.64, 0, 64.87, 90, 90), vel=100, acc=200, radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    # 2. 하강 (거치 위치로 진입)
    movel(posx(273.38, 428.64, -137.51, 64.87, 90, 90), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    
    # 3. 반납 (그리퍼 열기)
    print(f"{Colors.BLUE}🔓 그리퍼 오픈 (파우더 통 놓기){Colors.ENDC}")
    grip_open() 
    time.sleep(1.0) # 그리퍼가 완전히 열릴 때까지 잠시 대기

    # 4. 퇴피 (옆으로 빠지기 - 안전 거리 확보)
    movel(posx(246.49, 381.59, -137, 65.36, 90, 90), vel=100, acc=200, radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    print(f"{Colors.GREEN}✅ [Recovery] 파우더 반납 완료.{Colors.ENDC}")

    # 5. 홈 위치 복귀 [추가됨]
    print(f"{Colors.BLUE}🏠 홈 위치로 복귀합니다...{Colors.ENDC}")
    movel(posx(0, 0, 0, 22.75, 90, 90.62), vel=150, acc=400, radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    # 6. 최종 완료 메시지 [추가됨]
    print(f"{Colors.GREEN}✨ [Cleanup] 초기화 완료.{Colors.ENDC}")

def execute_topping_recovery():
    """
    토핑을 들고 있다가 에러 발생 시, 원래 집었던 위치로 되돌려 놓고 홈으로 복귀
    """
    from DSR_ROBOT2 import movel, posx, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE
    
    global g_last_topping_source
    
    print(f"{Colors.BLUE}🔄 [Recovery] 토핑 원위치 반납 동작을 수행합니다...{Colors.ENDC}")

    VELOCITY, ACC = 50.0, 100.0  # 복구는 천천히 안전하게

    # 1. 현재 위치에서 수직 상승 (안전하게 빠져나오기)
    # 케이크 바로 위에서 멈췄을 수도 있으므로 일단 위로 10cm 듭니다.
    print("⬆️ 안전 높이로 상승 중...")
    movel(posx(0, 0, 50, 0, 0, 0), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL, ref=101, ra=DR_MV_RA_DUPLICATE)

    # 2. 원래 집었던 위치(Source)가 기억되어 있는지 확인
    if g_last_topping_source:
        # 2-1. 반납 위치의 상공(Approach) 계산
        return_pos = list(g_last_topping_source).copy()
        return_approach = list(return_pos).copy()
        return_point_approach = list(return_pos).copy()

        return_approach[1] -= 10.0
        return_approach[2] += 100.0
        return_point_approach[1] -= 10.0
        return_point_approach[2]+=300.0 # 10cm 위
        movel(return_point_approach, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        # 2-2. 반납 위치 상공으로 이동
        print(f"🔙 원래 위치({return_pos}) 상공으로 이동...")
        movel(return_approach, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        
        # 2-3. 하강 (내려놓기 위치)
        # pick 할 때 target_pos[2] 보정을 했으므로, 안전하게 살짝 위(5mm)에 놓거나 원래 높이로 갑니다.
        # 여기선 안전하게 원래 좌표로 갑니다.
        movel(return_pos, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        
        # 3. 그리퍼 오픈 (토핑 놓기)
        print(f"{Colors.BLUE}🔓 그리퍼 오픈 (토핑 반납){Colors.ENDC}")
        # grip_fruit_open() 이 있다면 사용, 없다면 grip_open() 사용
        try:
            # 만약 별도의 open 함수가 없다면 grip_open() 사용
            grip_fruit_open()
        except:
            pass
        time.sleep(1.0)
        
        # 4. 다시 상승 (퇴피)
        movel(return_approach, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        
    else:
        print(f"{Colors.RED}⚠️ [Warning] 돌아갈 위치 정보가 없습니다. 현재 위치에서 그리퍼만 엽니다.{Colors.ENDC}")
        grip_open()
        time.sleep(1.0)

    # 5. 홈 위치 복귀
    print(f"{Colors.BLUE}🏠 홈 위치로 복귀합니다...{Colors.ENDC}")
    movel(posx(0, 0, 0, 22.75, 90, 90.62), vel=150, acc=400, radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    
    print(f"{Colors.GREEN}✨ [Cleanup] 토핑 단계 초기화 완료.{Colors.ENDC}")

# ============================
# [수정됨] Firebase 기반 복구 처리 함수
# ============================
def handle_recovery_process(monitor_node):
    """
    에러 발생 시 Firebase 'robot_reset' 노드의 'reset' 값을 대기하여 복구 시도
    복구 성공 시 'order_status'에 따라 후처리 동작 분기
    """
    global g_robot_error
    
    print(f"\n{Colors.RED}🛑 [System Error] 로봇 에러 상태입니다. Firebase 'robot_reset' 신호를 대기합니다...{Colors.ENDC}")
    
    # Firebase Reference
    reset_ref = db.reference("robot_reset")
    status_ref = db.reference("order_status")  # 상태를 확인하기 위한 레퍼런스
    
    while g_robot_error and rclpy.ok():
        try:
            # 1. Firebase polling (1초 간격)
            cmd = reset_ref.get()
            
            if cmd == "reset":
                print(f"\n{Colors.YELLOW}📩 [Firebase] 'reset' 명령 감지! 복구를 시작합니다.{Colors.ENDC}")
                
                # 2. 로봇 하드웨어/소프트웨어 복구 시도
                success = monitor_node.recover_to_standby()
                
                if success:
                    # 3. 현재 작업 상태(order_status) 확인
                    current_status = status_ref.get()
                    print(f"{Colors.BLUE}🚀 복구 성공! 현재 상태({current_status})에 따른 후처리를 시작합니다...{Colors.ENDC}")

                    # 4. 상태별 후처리 동작 분기 (확장성을 고려한 구조)
                    if current_status == "syrup":
                        # 시럽 작업 중이었을 때: 시럽 반납 -> 홈
                        execute_post_recovery_motion()
                        print(f"{Colors.GREEN}✅ [Post-Process] 시럽 반납 동작 완료.{Colors.ENDC}")
                        
                    elif current_status == "powder":
                        # [UPDATE] 방금 만든 파우더 복구 함수 호출
                        execute_powder_recovery()
                        print(f"{Colors.GREEN}✅ [Post-Process] 파우더 반납 동작 완료.{Colors.ENDC}")
                        
                    elif current_status == "topping_place":
                        # [UPDATE] 토핑 복구 함수 연결
                        execute_topping_recovery()
                        print(f"{Colors.GREEN}✅ [Post-Process] 토핑 반납 동작 완료.{Colors.ENDC}")

                    else:
                        # 상태가 없거나 알 수 없는 경우 (기본 홈 이동 등)
                        print(f"{Colors.RED}⚠️ 기본 대기 위치로 이동합니다.{Colors.ENDC}")
                        from DSR_ROBOT2 import movel, posx, DR_MV_MOD_ABS, DR_MV_RA_DUPLICATE
                        movel(posx(0, 0, 0, 22.75, 90, 90.62), vel=150, acc=400, radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                    
                    # 5. Firebase 리셋 상태 완료 처리
                    reset_ref.set("done")
                    print(f"{Colors.GREEN}✅ [Firebase] robot_reset -> 'done' 업데이트 완료.{Colors.ENDC}")
                    
                    break
                else:
                    print(f"{Colors.RED}⚠️ 복구 실패! 다시 'reset' 신호를 기다립니다.{Colors.ENDC}")
            
            time.sleep(1.0)
            
        except Exception as e:
            print(f"{Colors.RED}❌ [Error] 복구 프로세스 중 예외 발생: {e}{Colors.ENDC}")
            time.sleep(1.0)

# ============================
# 1. 유틸리티 함수 (기존)
# ============================
def get_latest_pending_order(orders_data):
    if not orders_data: return None, None
    pending_orders = []
    for key, val in orders_data.items():
        if not isinstance(val, dict): continue
        status = val.get('status')
        o_type = val.get('type')
        d_id = str(val.get('design_id'))
        is_basic = (status == 'pending' and o_type in ['기본도안', 'AI_VOICE_ORDER'] and d_id in ['3', '4'])
        is_custom = (status == 'pending' and o_type == '커스텀도안')
        if is_basic or is_custom:
            try:
                int_key = int(key)
                pending_orders.append((int_key, key, val))
            except ValueError: continue
    if not pending_orders: return None, None
    pending_orders.sort(key=lambda x: x[0], reverse=True)
    return pending_orders[0][1], pending_orders[0][2]

def _get_rotation_matrix(x, y, z, rx, ry, rz):
    R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T

def convert_camera_to_robot(camera_xyz):
    from DSR_ROBOT2 import get_current_posx
    try:
        package_path = get_package_share_directory("pick_and_place_text")
        gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
        if not os.path.exists(gripper2cam_path):
            print(f"❌ [Error] Calibration file not found")
            return None
        gripper2cam = np.load(gripper2cam_path)
        cam_point = np.append(np.array(camera_xyz), 1)
        curr_pos = get_current_posx()[0]
        x, y, z, rx, ry, rz = curr_pos
        base2gripper = _get_rotation_matrix(x, y, z, rx, ry, rz)
        base_point = base2gripper @ gripper2cam @ cam_point
        return list(base_point[:3]) + curr_pos[3:]
    except Exception as e:
        print(f"❌ [Error] TF failed: {e}")
        return None

def get_vision_target_pos(target_name):
    global g_node, vision_client
    if g_node is None or vision_client is None: return None
        
    req = SrvDepthPosition.Request()
    req.target = target_name
    print(f"🔍 [Vision] Searching for {target_name}...")
    
    if not vision_client.wait_for_service(timeout_sec=2.0):
        print("⚠️ [Vision] Service not ready.")
        return None

    check_safety()
    future = vision_client.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)
    
    if future.result() is not None:
        pos = future.result().depth_position.tolist()
        if sum(pos) == 0:
            print(f"⚠️ [Vision] Not found (0,0,0).")
            return None
        print(f"✅ [Vision] Found at {pos}")
        return pos
    else:
        return None

# ============================
# 3. Gripper Wrappers
# ============================
def grip_open():
    from DSR_ROBOT2 import wait
    check_safety()
    gripper.move_gripper(950)
    wait(1.00)

def grip_close():
    from DSR_ROBOT2 import wait,set_digital_output,OFF,ON
    check_safety()
    gripper.move_gripper(100)
    wait(1.00)

def grip_close_max():
    from DSR_ROBOT2 import wait,set_digital_output,OFF,ON
    check_safety()
    gripper.move_gripper(10)
    wait(1.00)

def grip_fruit_open():
    from DSR_ROBOT2 import wait
    gripper.move_gripper(450); 
    wait(1.0)

def grip_fruit_close():
    from DSR_ROBOT2 import wait
    check_safety()
    gripper.move_gripper(200)
    wait(1.0)

def push_gripper():
    from DSR_ROBOT2 import wait
    check_safety()
    gripper.move_gripper(300)
    wait(1.0)

def custom_gripper():
    from DSR_ROBOT2 import wait
    check_safety()
    gripper.move_gripper(300)
    wait(0.1)
    
def syrup_gripper():
    from DSR_ROBOT2 import wait
    check_safety()
    gripper.move_gripper(780)
    wait(0.5)

# ============================
# 4. Robot Tasks
# ============================
def execute_topping_pick(target_pos):
    from DSR_ROBOT2 import movel, mwait, posx, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE
    
    # [추가됨] 나중에 복구할 때를 대비해, 집으러 가는 위치를 전역 변수에 저장
    global g_last_topping_source

    update_process_status("topping_pick") 
    check_safety()
    
    VELOCITY, ACC = 60.0, 200.0 
    approach_pos = list(target_pos).copy()
    target_pos[1] -= 10.0
    approach_pos[1] -= 10.0
    approach_pos[2] += 100.0
    approach_pos[2] = max(approach_pos[2], 50.0)
    target_pos[2] = max(target_pos[2], 2.0)
    g_last_topping_source = list(target_pos).copy()
    movel(approach_pos, vel=VELOCITY, acc=ACC); mwait()
    check_safety()
    movel(target_pos, vel=VELOCITY, acc=ACC); mwait()
    check_safety()
    grip_close_max()
    movel(posx(0, 0, 200, 0, 0, 0), vel=VELOCITY, acc=ACC, radius=0.0, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); mwait()

def execute_topping_place(place_pose_list):
    from DSR_ROBOT2 import movel, mwait, posx, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE
    update_process_status("topping_place") 
    check_safety()
    VELOCITY, ACC = 60.0, 200.0
    pre_place_pose = list(place_pose_list).copy()
    pre_place_pose[2] += 80.0 
    check_safety()
    movel(pre_place_pose, vel=VELOCITY, acc=ACC, radius=0.0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    check_safety()
    movel(place_pose_list, vel=VELOCITY, acc=ACC, radius=0.0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE); mwait()
    check_safety()
    grip_fruit_open()
    check_safety()
    movel(posx(0, 0, 80, 0, 0, 0), vel=VELOCITY, acc=ACC,radius=0.0, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); mwait()

def execute_toppings_process(order_data):
    # [NEW] DB 상태 업데이트: topping
    update_process_status("topping_setup") 

    from DSR_ROBOT2 import posx, movel, DR_MV_MOD_ABS, DR_MV_RA_DUPLICATE, wait
    check_safety()
    print("🍓 [Toppings] 시작")
    VELOCITY, ACC = 60, 200
    movel(posx(0, 0, 0, 22.75, 90.0, 90.62), radius=0.0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    check_safety()
    grip_fruit_open()
    movel(posx(-93.15, 374.85, -157, 84.34, 176.65, 85.22), vel=VELOCITY, acc=ACC, radius=0.0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE); wait(1.0)
    check_safety()

    toppings_data = order_data.get('toppings')
    if not toppings_data: return
    topping_list = []
    if isinstance(toppings_data, list): topping_list = [x for x in toppings_data if isinstance(x, int)]
    elif isinstance(toppings_data, dict):
        try: topping_list = [toppings_data[k] for k in sorted(toppings_data.keys(), key=lambda k: int(k)) if isinstance(toppings_data[k], int)]
        except: topping_list = [v for v in toppings_data.values() if isinstance(v, int)]

    for i, topping_id in enumerate(topping_list):
        check_safety()
        if i >= len(PLACE_POSES): break
        target_name = OBJECT_DICT.get(topping_id)
        if not target_name: continue
        
        cam_pos = get_vision_target_pos(target_name)
        if not cam_pos: continue
        robot_pos = convert_camera_to_robot(cam_pos)
        if not robot_pos: continue
        robot_pos[2] = max(robot_pos[2] - 5, 2)
        
        execute_topping_pick(robot_pos)
        check_safety()
        execute_topping_place(PLACE_POSES[i])
        check_safety()
        movel(posx(-93.15, 374.85, -157, 84.34, 176.65, 85.22), vel=VELOCITY, acc=ACC, radius=0.0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        time.sleep(0.5)

def execute_powder():
    update_process_status("powder")
    """
    Powder 작업 실행 함수
    """
    from DSR_ROBOT2 import (
        posx, movel, set_singular_handling, set_velj, set_accj, set_velx, set_accx,
        DR_AVOID, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE,
        move_periodic
    )

    print("🧂 [Powder] 파우더 프로세스 시작")

    set_singular_handling(DR_AVOID)
    set_velj(10.0); set_accj(30.0)
    set_velx(80.0, 33.0); set_accx(300.0, 130.0)
    check_safety()
    print("👉 [DEBUG] Powder: 1. 도구 잡으러 이동")
    grip_open()
    movel(posx(0,0,0,22.75, 90, 90.62), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(246.49, 381.59, -137, 65.36, 90, 90), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(273.38, 428.64, -137.51, 64.87, 90, 90), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    check_safety()
    grip_close()
    
    print("👉 [DEBUG] Powder: 2. 도구 들고 작업 위치로 이동")
    movel(posx(273.38, 428.64, 0, 64.87, 90, 90), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(0.00, 0.00, 100.00, 39.02, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    check_safety()
    print("👉 [DEBUG] Powder: 3. 뿌리기 동작 시작 (중앙)")
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    check_safety()
    print("👉 [DEBUG] Powder: 4. 뿌리기 동작 (우측)")
    movel(posx(75.00, 0.00, 100.00, 39.02, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    check_safety()
    print("👉 [DEBUG] Powder: 5. 뿌리기 동작 (우하단)")
    movel(posx(75.00, -75.00, 100.00, 39.02, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    check_safety()
    print("👉 [DEBUG] Powder: 6. 뿌리기 동작 (하단)")
    movel(posx(0.00, -75.00, 100.00, 39.02, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    check_safety()
    print("👉 [DEBUG] Powder: 7. 뿌리기 동작 (좌하단)")
    movel(posx(-75.00, -75.00, 100.00, 39.02, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    check_safety()
    print("👉 [DEBUG] Powder: 8. 뿌리기 동작 (좌측)")
    movel(posx(-75.00, 0.00, 100.00, 39.02, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    check_safety()
    print("👉 [DEBUG] Powder: 9. 뿌리기 동작 (좌상단)")
    movel(posx(-75.00, 75.00, 100.00, 39.02, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    check_safety()
    print("👉 [DEBUG] Powder: 10. 뿌리기 동작 (상단)")
    movel(posx(0.00, 75.00, 100.00, 39.02, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    check_safety()
    print("👉 [DEBUG] Powder: 11. 뿌리기 동작 (우상단)")
    movel(posx(75.00, 75.00, 100.00, 39.02, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    move_periodic(amp=[0.00, 0.00, 50.00, 0.00, 0.00, 0.00], period=[0.00, 0.00, 2.00, 0.00, 0.00, 0.00], atime=1.50, repeat=2, ref=101)
    check_safety()
    print("👉 [DEBUG] Powder: 12. 도구 반납")
    movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
    movel(posx(273.38, 428.64, 0, 64.87, 90, 90), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(273.38, 428.64, -137.51, 64.87, 90, 90), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    
    grip_open()

    movel(posx(246.49, 381.59, -137, 65.36, 90, 90), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    print("✅ [Powder] 파우더 프로세스 완료")

def execute_custom_design(line_data):
    update_process_status("syrup")
    """
    [핵심 수정] 사용자 요청 로직 이식 완료
    - Firebase DB 조회 제거 (line_data 인자 사용)
    - 로봇 동작 및 변수만 남김
    """
    from DSR_ROBOT2 import (
        posx, movel, movesx,
        set_velj, set_accj, wait, set_velx, set_accx,
        DR_MV_MOD_ABS, DR_MV_RA_DUPLICATE
    )

    if not line_data:
        print("⚠️ drawing_path 데이터가 비어있습니다. 커스텀 도안 스킵.")
        return

    print("🎨 [Custom Design] 커스텀 도안 드로잉 시작")

    # --- 설정 값 ---
    Z_DRAW = -173.0  
    Z_LIFT = -165.0  
    
    VEL_DRAW = 65.0   
    ACC_DRAW = 800.0
    VEL_MOVE = 150.0  
    ACC_MOVE = 400.0

    ORI_RX, ORI_RY, ORI_RZ = 21.59, 98.00, 90.93
    USER_COORD = 101 # 사용자 좌표계

    set_velj(10)
    set_accj(30)
    check_safety()
    # 1. 시작 위치 이동 및 초기화
    grip_open()
    movel(posx(374.26, 166.74, -139, 23.35, 90, 88.97),  vel=VEL_MOVE, acc=ACC_MOVE,radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(432.26, 194.39, -139, 23.35, 90, 88.97),  vel=VEL_MOVE, acc=ACC_MOVE,radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    check_safety()
    syrup_gripper()
    movel(posx(432.26, 194.39, 0, 23.35, 90, 88.97),  vel=VEL_MOVE, acc=ACC_MOVE,radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    check_safety()
    # 2. 데이터 파싱 (List or Dict)
    strokes = []
    if isinstance(line_data, list):
        strokes = line_data
    elif isinstance(line_data, dict):
        for k in sorted(line_data.keys(), key=lambda x: int(x)):
            strokes.append(line_data[k])

    total_strokes = len(strokes)
    print(f"🖌️ 총 {total_strokes}개의 획(Stroke)을 그립니다.")

    # 3. 그리기 루프
    for i, stroke_points in enumerate(strokes):
        if not stroke_points: 
            continue 
        
        print(f"  [{i+1}/{total_strokes}] 번째 획 그리기 준비...")

        start_pt = stroke_points[0]
        target_x = float(start_pt.get('x', 0))
        target_y = float(start_pt.get('y', 0))
        
        p_draw = posx(target_x+35, target_y+18, Z_DRAW, ORI_RX, ORI_RY, ORI_RZ)

        # 시작점으로 이동
        movel(p_draw, vel=150, acc=600, ref=USER_COORD)
        check_safety()
        # Spline 경로 생성
        path_list_posx = []
        for pt in stroke_points:
            px = float(pt.get('x', 0))
            py = float(pt.get('y', 0))
            path_list_posx.append(posx(px+35, py+18, Z_DRAW, ORI_RX, ORI_RY, ORI_RZ))
        
        # 100개씩 잘라서 실행 (메모리/버퍼 관리)
        MAX_POINTS = 100
        total_pts = len(path_list_posx)

        if total_pts > 1:
            for idx in range(0, total_pts, MAX_POINTS):
                chunk = path_list_posx[idx : idx + MAX_POINTS]
                
                if len(chunk) > 1:
                    check_safety()
                    custom_gripper() # 짜기 시작
                    movesx(chunk, vel=VEL_DRAW, acc=ACC_DRAW, time=0, mod=DR_MV_MOD_ABS, ref=USER_COORD)
                    syrup_gripper() # 멈춤
                elif len(chunk) == 1:
                    wait(0.01)
        elif total_pts == 1:
            wait(0.1)

        # 획 종료 및 들어올리기
        syrup_gripper()
        
        last_pt = stroke_points[-1]
        last_x = float(last_pt.get('x', 0))
        last_y = float(last_pt.get('y', 0))
        
        p_end_lift = posx(last_x+35, last_y+18, Z_LIFT, ORI_RX, ORI_RY, ORI_RZ)
        movel(p_end_lift, vel=VEL_MOVE, acc=ACC_MOVE, ref=USER_COORD)
    
    print("🏠 원위치로 복귀 중...")
    movel(posx(432.26, 194.39, 0, 23.35, 90, 88.97), vel=VEL_MOVE, acc=ACC_MOVE, ref=USER_COORD, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(432.26, 194.39, -139, 23.35, 90, 88.97), vel=VEL_MOVE, acc=ACC_MOVE, ref=USER_COORD, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    grip_open()
    movel(posx(374.26, 166.74, -139, 23.35, 90, 88.97), vel=VEL_MOVE, acc=ACC_MOVE, ref=USER_COORD, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    
    print("\n✨ 모든 작업 완료.")

def execute_design_3():
        update_process_status("syrup")
        from DSR_ROBOT2 import (
            posx, movel, movec, movesx, set_singular_handling, set_velj, set_accj, set_velx, set_accx,
            DR_AVOID, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE, wait
        )
        print("🎨 [Design 3] 시작")
        set_singular_handling(DR_AVOID)
        set_velj(10.0); set_accj(30.0)
        set_velx(60.0, 33.0); set_accx(200.0, 130.0)
        change_x = 35
        change_y = 18
        check_safety()
        #시럽 픽업
        grip_open()
        movel(posx(374.26, 166.74, -139, 23.35, 90, 88.97),radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(432.26, 194.39, -139, 23.35, 90, 88.97),radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        syrup_gripper()
        movel(posx(432.26, 194.39, 0, 23.35, 90, 88.97),radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        # 시작점
        # movel(posx(-50.00, -30.00, -29.98, 39.02, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(-50.00+change_x, -30.02+change_y, -173.0, 21.59, 98.00, 90.93), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        print("👉 [DEBUG] D3: 3. 첫 번째 윤곽 드로잉 (힘 제어 ON)")
        push_gripper()
        movesx([
            posx(-49.99+change_x, -30.03+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-58.50+change_x, -25.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-67.00+change_x, -3.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-71.50+change_x, 20.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-70.00+change_x, 35.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-64.00+change_x, 39.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-70.00+change_x, 35.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-79.00+change_x, 34.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-87.50+change_x, 35.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-92.50+change_x, 41.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-93.00+change_x, 49.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-77.50+change_x, 66.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-55.00+change_x, 72.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-39.00+change_x, 79.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-22.00+change_x, 82.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-1.50+change_x, 83.50+change_y, -173.0 , 21.59, 98.00, 90.93),
            posx(0.00+change_x, 83.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(12.50+change_x, 81.80+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(24.60+change_x, 84.70+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(35.30+change_x, 86.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(46.20+change_x, 83.90+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(57.10+change_x, 76.70+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(63.50+change_x, 63.60+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(57.20+change_x, 55.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(41.50+change_x, 53.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(28.00+change_x, 57.10+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(41.50+change_x, 53.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(49.40+change_x, 27.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(52.00+change_x, 14.70+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(51.00+change_x, 0.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(43.50+change_x, -14.40+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(52.20+change_x, -28.40+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(58.00+change_x, -41.80+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(60.50+change_x, -59.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(58.50+change_x, -72.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(54.40+change_x, -79.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(45.00+change_x, -84.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(33.40+change_x, -86.10+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(16.70+change_x, -83.20+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(18.70+change_x, -77.60+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(19.40+change_x, -72.60+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(18.70+change_x, -77.60+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(16.70+change_x, -83.20+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(10.00+change_x, -86.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(3.20+change_x, -82.60+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(3.20+change_x, -74.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(3.20+change_x, -82.60+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-6.90+change_x, -86.01+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-13.50+change_x, -83.20+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-15.50+change_x, -77.58+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-16.20+change_x, -72.56+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-15.50+change_x, -77.61+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-13.50+change_x, -83.22+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-23.51+change_x, -89.08+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-34.99+change_x, -89.00+change_y, -173.0 , 21.59, 98.00, 90.93),
            posx(-45.00+change_x, -87.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-55.00+change_x, -83.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-63.00+change_x, -76.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-66.00+change_x, -68.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-64.00+change_x, -60.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-61.00+change_x, -53.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-55.00+change_x, -41.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-50.00+change_x, -30.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-35.00+change_x, -34.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-22.50+change_x, -33.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(0.00+change_x, -32.90+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(29.50+change_x, -22.70+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(43.50+change_x, -14.40+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(45.00+change_x, -12.50+change_y, -173.0 , 21.59, 98.00, 90.93)
        ], ref=101)
        check_safety()
        syrup_gripper()
        print("👉 [DEBUG] D3: 6. 첫 번째 드로잉 완료 (힘 제어 OFF)")
        wait(0.50)

        print("👉 [DEBUG] D3: 7. 내부 디테일 드로잉 1 시작")
        # movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        # movel(posx(-64.00, -62.49, -100.00, 39.03, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(-64.00+change_x, -62.48+change_y, -173.0 , 21.59, 98.00, 90.93), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        push_gripper()
        check_safety()
        movesx([
            posx(-64.00+change_x, -62.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-72.00+change_x, -60.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-82.50+change_x, -58.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-90.00+change_x, -55.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-92.00+change_x, -61.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-89.00+change_x, -68.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-82.00+change_x, -73.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-74.00+change_x, -72.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-70.00+change_x, -76.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-63.00+change_x, -76.50+change_y, -173.0 , 21.59, 98.00, 90.93)
        ], ref=101)
        check_safety()
        syrup_gripper()
        print("👉 [DEBUG] D3: 8. 내부 디테일 드로잉 2 (원형)")
        # movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        # movel(posx(-43.00, 40.00, -100.00, 39.03, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(-43.00+change_x, 40.00+change_y, -173.0 , 21.59, 98.00, 90.93), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        push_gripper()
        check_safety()
        movel(posx(-43.00+change_x, 40.01+change_y, -173.0 , 21.59, 98.00, 90.93), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        movec(posx(-38.24+change_x, 31.75+change_y, -173.0 , 21.59, 98.00, 90.93), posx(-47.76+change_x, 31.75+change_y, -173.0 , 21.59, 98.00, 90.93), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        check_safety()
        movec(posx(-39.24+change_x, 32.75+change_y, -173.0 , 21.59, 98.00, 90.93), posx(-46.76+change_x, 32.75+change_y, -173.0 , 21.59, 98.00, 90.93), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        check_safety()
        movec(posx(-40.24+change_x, 33.75+change_y, -173.0 , 21.59, 98.00, 90.93), posx(-45.76+change_x, 33.75+change_y, -173.0 , 21.59, 98.00, 90.93), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        check_safety()
        movec(posx(-41.24+change_x, 34.75+change_y, -173.0 , 21.59, 98.00, 90.93), posx(-44.76+change_x, 34.75+change_y, -173.0 , 21.59, 98.00, 90.93), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        check_safety()
        syrup_gripper()
        
        print("👉 [DEBUG] D3: 9. 내부 디테일 드로잉 3 (원형)")
        # movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        # movel(posx(17.00, 47.30, -100.00, 39.03, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(17.00+change_x, 47.30+change_y, -173.0 , 21.59, 98.00, 90.93), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        push_gripper()

        movel(posx(17.00+change_x, 47.30+change_y, -173.0 , 21.59, 98.00, 90.93), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movec(posx(21.76+change_x, 39.05+change_y, -173.0 , 21.59, 98.00, 90.93), posx(12.24+change_x, 39.05+change_y, -173.0 , 21.59, 98.00, 90.93), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        movec(posx(20.76+change_x, 40.05+change_y, -173.0 , 21.59, 98.00, 90.93), posx(13.24+change_x, 40.05+change_y, -173.0 , 21.59, 98.00, 90.93), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        movec(posx(19.76+change_x, 41.05+change_y, -173.0 , 21.59, 98.00, 90.93), posx(14.24+change_x, 41.05+change_y, -173.0 , 21.59, 98.00, 90.93), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        movec(posx(18.76+change_x, 42.05+change_y, -173.0 , 21.59, 98.00, 90.93), posx(15.24+change_x, 42.05+change_y, -173.0 , 21.59, 98.00, 90.93), radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        check_safety()
        syrup_gripper()

        print("👉 [DEBUG] D3: 10. 눈 그리기 1")
        # movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        # movel(posx(-9.10, -5.00, -100.00, 39.03, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        movel(posx(-9.10+change_x, -5.00+change_y, -173.0 , 21.59, 98.00, 90.93), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        push_gripper()
        check_safety()
        movel(posx(-2.00+change_x, -4.00+change_y, -173.0 , 21.59, 98.00, 90.93), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        syrup_gripper()
        wait(0.50)

        print("👉 [DEBUG] D3: 11. 입 그리기")
        # movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        # movel(posx(-15.00, -11.00, -100.00, 39.03, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(-15.00+change_x, -11.00+change_y, -173.0 , 21.59, 98.00, 90.93), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        push_gripper()
        movesx([
            posx(-15.00+change_x, -11.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-12.00+change_x, -14.30+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-9.10+change_x, -15.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-6.40+change_x, -14.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-5.00+change_x, -12.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(-2.00+change_x, -14.00+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(0.00+change_x, -14.50+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(2.60+change_x, -12.70+change_y, -173.0 , 21.59, 98.00, 90.93), 
            posx(6.00+change_x, -11.00+change_y, -173.0 , 21.59, 98.00, 90.93)
        ], ref=101)
        check_safety()
        syrup_gripper()
        print("🏠 원위치로 복귀 중...")
        movel(posx(432.26, 194.39, 0, 23.35, 90, 88.97),ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(432.26, 194.39, -139, 23.35, 90, 88.97), ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        grip_open()
        movel(posx(374.26, 166.74, -139, 23.35, 90, 88.97), ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

def execute_design_4():
        update_process_status("syrup")
        from DSR_ROBOT2 import (
            posx, movel, movec, movesx, set_singular_handling, set_velj, set_accj, set_velx, set_accx,
            DR_AVOID, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE, wait
        )
        print("🎨 [Design 4] 시작")
        set_singular_handling(DR_AVOID)
        set_velj(10.0); set_accj(30.0)
        set_velx(65.0, 33.0); set_accx(250.0, 130.0)
        check_safety()
        #시럽 픽업
        grip_open()
        movel(posx(374.26, 166.74, -139, 23.35, 90, 88.97),radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(432.26, 194.39, -139, 23.35, 90, 88.97),radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        syrup_gripper()
        movel(posx(432.26, 194.39, 0, 23.35, 90, 88.97),radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

        # self.grip_open()
        # movel(posx(156.51, 421.10, -82.00, 79.23, 89.05, -87.16), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        # self.grip_close()
        check_safety()
        # movel(posx(156.51, 421.10, -30.00, 79.23, 89.05, -87.16), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(-27.51, -20.59, -29.98, 39.02, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(-27.50, -20.50, -153.00, 39.03, 89.35, 85.58), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        push_gripper()
        check_safety()
        # 메인 윤곽 (Split 1)
        movesx([
            posx(-27.50, -20.50, -154.00, 39.03, 89.35, 85.58),
            posx(-32.20, -13.50, -154.96, 39.03, 89.35, 85.59),
            posx(-33.50, 23.70, -154.96, 39.03, 89.35, 85.59),
            posx(-30.80, 33.10, -154.96, 39.03, 89.35, 85.59),
            posx(-42.90, 37.00, -154.96, 39.03, 89.35, 85.59),
            posx(-41.90, 50.00, -154.96, 39.03, 89.35, 85.59),
            posx(-34.10, 52.30, -154.96, 39.03, 89.35, 85.59),
            posx(-26.30, 51.10, -154.96, 39.03, 89.35, 85.59),
            posx(-20.20, 46.00, -154.98, 39.03, 89.35, 85.59),
            posx(-15.60, 51.00, -154.98, 39.03, 89.35, 85.59),
            posx(-8.30, 54.20, -154.98, 39.03, 89.35, 85.59),
            posx(-1.10, 54.40, -154.98, 39.03, 89.35, 85.59),
            posx(7.80, 53.10, -154.98, 39.03, 89.35, 85.59),
            posx(23.20, 48.00, -154.98, 39.03, 89.35, 85.59),
            posx(29.70, 50.80, -154.98, 39.03, 89.35, 85.59),
            posx(45.30, 43.20, -154.98, 39.03, 89.35, 85.59),
            posx(41.20, 34.50, -154.98, 39.03, 89.35, 85.59),
            posx(30.00, 29.80, -154.98, 39.03, 89.35, 85.59),
            posx(33.20, 8.40, -154.98, 39.03, 89.35, 85.59),
            posx(28.20, -19.00, -154.98, 39.03, 89.35, 85.59),
            posx(39.70, -20.20, -154.98, 39.03, 89.35, 85.59),
            posx(45.00, -25.80, -154.98, 39.03, 89.35, 85.59),
            posx(42.90, -31.80, -154.98, 39.03, 89.35, 85.59),
            posx(32.10, -33.50, -154.98, 39.03, 89.35, 85.59),
            posx(31.60, -60.00, -154.98, 39.03, 89.35, 85.59),
            posx(20.50, -71.00, -154.98, 39.03, 89.35, 85.59),
            posx(8.00, -73.50, -154.98, 39.03, 89.35, 85.59),
            posx(8.00, -85.00, -154.98, 39.03, 89.35, 85.59),
            posx(4.00, -88.80, -154.98, 39.03, 89.35, 85.59),
            posx(-0.80, -84.00, -154.98, 39.03, 89.35, 85.59),
            posx(-0.80, -79.20, -154.98, 39.03, 89.35, 85.59),
            posx(-0.81, -84.01, -154.97, 39.03, 89.35, 85.59),
            posx(-7.00, -90.00, -154.98, 39.03, 89.35, 85.59),
            posx(-10.60, -78.00, -154.98, 39.03, 89.35, 85.59),
            posx(-19.60, -74.00, -154.98, 39.03, 89.35, 85.59),
            posx(-26.20, -79.80, -154.98, 39.03, 89.35, 85.59),
            posx(-37.00, -79.40, -154.98, 39.03, 89.35, 85.59),
            posx(-41.20, -71.00, -154.98, 39.03, 89.35, 85.59),
            posx(-25.00, -69.00, -154.98, 39.03, 89.35, 85.59),
            posx(-29.50, -53.00, -154.98, 39.03, 89.35, 85.59),
            posx(-31.80, -35.20, -154.98, 39.03, 89.35, 85.59),
            posx(-44.70, -28.50, -154.98, 39.03, 89.35, 85.59),
            posx(-42.80, -23.00, -154.98, 39.03, 89.35, 85.59),
            posx(-27.50, -20.50, -154.98, 39.03, 89.35, 85.59),
            posx(0.00, -25.00, -154.98, 39.03, 89.35, 85.59),
            posx(28.20, -19.00, -154.98, 39.03, 89.35, 85.59)
    ], ref=101)
        syrup_gripper()
        check_safety()
        print("👉 [DEBUG] D4: 4. 메인 윤곽 완료 (힘 제어 OFF)")
        movel(posx(536.20, -173.00, 300.02, 39.03, 89.35, 85.59), radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        print("👉 [DEBUG] D4: 5. 오른쪽 눈 드로잉 (원형)")
        movel(posx(20.20, 25.50, -99.98, 39.03, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(20.21, 25.50, -155.00, 39.03, 89.35, 85.60), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        push_gripper()
        movec(posx(12.70, 21.17, -155.00, 39.02, 89.35, 85.59), posx(12.70, 29.83, -155.00, 39.02, 89.35, 85.59), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        movec(posx(13.70, 22.17, -155.00, 39.02, 89.35, 85.59), posx(13.70, 28.83, -155.00, 39.02, 89.35, 85.59), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        movec(posx(14.70, 23.17, -155.00, 39.02, 89.35, 85.59), posx(14.70, 27.83, -155.00, 39.02, 89.35, 85.59), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        movec(posx(15.70, 24.17, -155.00, 39.02, 89.35, 85.59), posx(15.70, 26.83, -155.00, 39.02, 89.35, 85.59), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        syrup_gripper()
        check_safety()
        print("👉 [DEBUG] D4: 6. 왼쪽 눈 드로잉 (원형)")
        movel(posx(15.00, 31.00, -99.98, 39.03, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(-23.20, 25.50, -99.98, 39.03, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(-23.21, 25.50, -155.00, 39.03, 89.35, 85.60), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        push_gripper()
        movec(posx(-15.70, 29.83, -155.01, 39.03, 89.35, 85.59), posx(-15.70, 21.17, -155.00, 39.03, 89.35, 85.59), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        movec(posx(-16.70, 28.83, -155.01, 39.03, 89.35, 85.59), posx(-16.70, 22.17, -155.00, 39.03, 89.35, 85.59), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        movec(posx(-17.70, 27.83, -155.01, 39.03, 89.35, 85.59), posx(-17.70, 23.17, -155.00, 39.03, 89.35, 85.59), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        movec(posx(-18.70, 26.83, -155.01, 39.03, 89.35, 85.59), posx(-18.70, 24.17, -155.00, 39.03, 89.35, 85.59), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[360.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        syrup_gripper()
        check_safety()
        print("👉 [DEBUG] D4: 7. 입 그리기 1")
        movel(posx(15.00, 31.00, -99.98, 39.03, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(0.00, -8.00, -99.98, 39.03, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(0.00, -8.00, -155.00, 39.03, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        push_gripper()
        movel(posx(0.00, -8.00, -155.00, 39.03, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        syrup_gripper()
        check_safety()
        print("👉 [DEBUG] D4: 8. 입 그리기 2 (곡선)")
        movel(posx(0.01, -7.95, -99.99, 39.03, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(-10.00, -15.00, -99.98, 39.03, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(-10.00, -15.00, -155.00, 39.03, 89.35, 85.59), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        push_gripper()
        movel(posx(-10.00, -14.99, -154.96, 39.03, 89.35, 85.60), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movec(posx(0.00, -13.50, -154.96, 39.03, 89.35, 85.60), posx(-6.30, -5.58, -154.96, 39.03, 89.35, 85.60), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[130.00, 0.00], ra=DR_MV_RA_DUPLICATE)

        wait(0.50)
        check_safety()
        movel(posx(0.00, -13.51, -154.96, 39.03, 89.35, 85.60), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movec(posx(10.00, -15.00, -154.96, 39.03, 89.35, 85.60), posx(6.30, -5.58, -155.00, 39.03, 89.35, 85.60), vel=[50.00, 64.13], acc=[100.00, 256.50], radius=0.00, ref=101, angle=[150.00, 0.00], ra=DR_MV_RA_DUPLICATE)
        syrup_gripper()
        check_safety()
        print("👉 [DEBUG] D4: 9. 도구 반납")
        movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=101, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        # movel(posx(156.51, 421.11, -30.03, 79.23, 89.05, -87.16), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        # movel(posx(156.51, 421.10, -80.00, 79.23, 89.05, -87.16), radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE
        print("🏠 원위치로 복귀 중...")
        check_safety()
        movel(posx(432.26, 194.39, 0, 23.35, 90, 88.97),ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        movel(posx(432.26, 194.39, -139, 23.35, 90, 88.97), ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        check_safety()
        grip_open()
        movel(posx(374.26, 166.74, -139, 23.35, 90, 88.97), ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

def execute_cake_pickup():
    update_process_status("pickup")
    """
    케이크를 픽업대로 이동시키는 함수 (좌표 테스트용)
    """
    from DSR_ROBOT2 import (
        posx, movel, set_singular_handling,
        set_velj, set_accj, set_velx, set_accx,
        DR_AVOID, DR_MV_MOD_ABS, DR_MV_RA_DUPLICATE
    )

    print("[Pickup] 케이크 픽업대로 이동 시작")

    # 설정
    set_singular_handling(DR_AVOID)
    set_velj(10.0); set_accj(30.0)
    set_velx(150.0, 72.375)
    set_accx(500.0, 289.5)
    check_safety()
    # 동작 시퀀스
    print("[DEBUG] 1) 초기 위치 및 접근")
    grip_open()
    check_safety()
    movel(posx(0, 0, 0, 22.75, 90.0, 90.62), radius=0.0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(7.93, -250.39, -180.51, 35.27, 90.97, 5.45), radius=0.0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(7.93, -250.39, -379.36, 35.27, 90.97, 5.45), radius=0.0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(31.15, -164.25, -379.36, 47.15, 89.13, 5.45), radius=0.0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    print("[DEBUG] 2) 케이크 잡기")
    grip_close()
    check_safety()
    print("[DEBUG] 3) 픽업대로 이동")
    movel(posx(33.13, -162.19, -287.56, 45.98, 86.47, 11.4), radius=0.0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(44.81, -325.83, -261.62, 21.19, 83.93, 12.23), radius=0.0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(128.59, -590.83, -249.38, 157.97, -84.76, -165.99), radius=0.0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(128.59, -590.83, -383.0, 157.97, -84.76, -165.99), radius=0.0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    check_safety()
    print("[DEBUG] 4) 케이크 놓기")
    grip_open()
    check_safety()
    print("[DEBUG] 5) 원위치 복귀")
    movel(posx(68.43, -615.76, -365.25, 157.82, -85.2, -170.38), radius=0.0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(68.43, -615.76, -200, 157.82, -85.2, 170.38), radius=0.0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(0, 0, 0, 22.75, 90.0, 90.62), radius=0.0, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    print("[OK] 케이크 픽업 완료")

# ============================
# Main Executor (수정됨)
# ============================
def perform_task(monitor_node, order_key, order_data):
    from DSR_ROBOT2 import set_tool, set_tcp, movel, posx, DR_MV_MOD_ABS, DR_MV_RA_DUPLICATE
    
    global g_robot_error
    if g_robot_error:
        print(f"⛔ [SKIP] 현재 로봇 에러 상태입니다. 주문 {order_key}를 보류합니다.")
        return

    try:
        print(f"\n🚀 [START] 주문 {order_key} 처리 시작...")
        db.reference(f"orders/{order_key}").update({"status": "processing"})
        update_process_status("start")
        check_safety()

        set_tool(ROBOT_TOOL); set_tcp(ROBOT_TCP)
        movel(posx(0,0,0,22.75, 90, 90.62), vel=150, acc=400, radius=0.00, ref=101, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

        o_type = order_data.get('type')
        d_id = str(order_data.get('design_id'))
        line_data = order_data.get('drawing_path', [])
        last_status = db.reference("order_status").get()
        if o_type in ['기본도안', 'AI_VOICE_ORDER'] and d_id == '4': execute_design_4()
        elif o_type in ['기본도안', 'AI_VOICE_ORDER'] and d_id == '3': execute_design_3()
        elif o_type == '커스텀도안': execute_custom_design(line_data)

        try:
            current_powder = str(order_data.get('powder')).lower()
        
        # 2. 파우더 주문이 있고, AND '이미 토핑이나 픽업 단계로 넘어간 게 아니라면' 실행
        # 즉, last_status가 'topping'이나 'pickup'이면 이미 파우더는 끝난 것이므로 건너뜀
            if current_powder in ["choco_powder", "sugar_powder"]:
                
                if last_status in ["topping", "pickup", "done"]:
                    print(f"⏩ [SKIP] 파우더 단계는 이미 완료되었습니다. (현재 상태: {last_status})")
                else:
                    # 실행하기 직전에 상태를 'powder'로 업데이트
                    execute_powder() 
                
        except RobotStopException:
            raise # 에러 밖으로 던지기 (필수)
        except Exception as e:
            print(f"⚠️ 파우더 오류: {e}")
            pass

        try:
                # 3. 토핑도 마찬가지로 이미 픽업 단계라면 건너뛸 수 있음
                if last_status in ["pickup", "done"]:
                    print(f"⏩ [SKIP] 토핑 단계는 이미 완료되었습니다.")
                else: # 상태 업데이트
                    execute_toppings_process(order_data)
                    
        except RobotStopException:
                raise
        except Exception as e: 
                print(f"⚠️ 토핑 오류: {e}")
                    
        except RobotStopException:
            raise # 에러 밖으로 던지기 (필수)
        except Exception as e:
            print(f"⚠️ 파우더 오류: {e}")
            pass

        try: execute_cake_pickup()
        except RobotStopException: raise
        except Exception as e: print(f"⚠️ 픽업 이동 오류: {e}")

        print(f"✅ [DONE] 주문 {order_key} 완료.")
        db.reference(f"orders/{order_key}").update({"status": "done"})
        update_process_status("done")

    # [NEW] 로봇 에러 발생 시 처리
    except RobotStopException:
        print(f"{Colors.RED}🛑 [ABORT] 로봇 에러로 작업이 중단되었습니다.{Colors.ENDC}")
        db.reference(f"orders/{order_key}").update({"status": "error"})
        
        # [핵심] 복구 모드 진입
        handle_recovery_process(monitor_node)

    except Exception as e:
        print(f"❌ [ERROR] 작업 실패: {e}")
        db.reference(f"orders/{order_key}").update({"status": "failed"})

def order_listener(event):
    if event.data is None: return
    try:
        if event.path == "/": orders_to_check = event.data
        else: orders_to_check = db.reference('orders').get()
    except: return
    t_key, t_val = get_latest_pending_order(orders_to_check)
    if t_key:
        print(f"📥 [QUEUE] 주문 감지됨: {t_key}")
        order_queue.put((t_key, t_val))

# ============================
# MAIN
# ============================
def monitor_thread_func(monitor_node):
    executor = SingleThreadedExecutor()
    executor.add_node(monitor_node)
    try:
        executor.spin()
    except Exception as e:
        print(f"Monitor Thread Error: {e}")
    finally:
        executor.shutdown()

def main(args=None):
    global g_node, vision_client
    rclpy.init(args=args)
    
    main_node = rclpy.create_node("robot_main_controller", namespace=ROBOT_ID)
    DR_init.__dsr__node = main_node
    g_node = main_node
    vision_client = main_node.create_client(SrvDepthPosition, "/get_3d_position")

    monitor_node = RobotStateMonitor()
    monitor_thread = threading.Thread(target=monitor_thread_func, args=(monitor_node,), daemon=True)
    monitor_thread.start()

    try:
        from DSR_ROBOT2 import set_tool, set_tcp, set_velx, set_velj
        try: set_tool(ROBOT_TOOL); set_tcp(ROBOT_TCP); set_velj(30.0); set_velx(100.0, 30.0)
        except: pass

        cred = credentials.Certificate(FIREBASE_JSON)
        firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
        db.reference("orders").listen(order_listener)
        
        print("🔥 시스템 준비 완료. (Main: Logic / Thread: Monitoring)")

        while rclpy.ok():
            if not order_queue.empty():
                key, val = order_queue.get()
                # [수정] 복구 함수 호출을 위해 monitor_node 전달
                perform_task(monitor_node, key, val) 
            
            rclpy.spin_once(main_node, timeout_sec=0.1)

    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
    except Exception as e:
        print(f"❌ Critical Error: {e}")
    finally:
        # 종료 처리
        main_node.destroy_node()
        monitor_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()