import sys
import os
import time
import glob
import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, db

# ------------------------------------------------------------------
# 🛠️ [경로 설정] 라이브러리 경로 최우선 순위 등록
# ------------------------------------------------------------------
print("🔍 라이브러리 경로 우선순위 재설정 중...")

# 사용자 PC의 진짜 파일 경로
real_path = '/home/dell/cobot1_ws/install/dsr_msgs2/local/lib/python3.10/dist-packages'

if os.path.exists(real_path):
    if real_path not in sys.path:
        sys.path.insert(0, real_path)
        print(f"✅ [System] 경로 등록 완료: {real_path}")
else:
    print("⚠️ [경고] 지정된 경로가 없습니다. 빌드 상태를 다시 확인해주세요.")

# ------------------------------------------------------------------
# ✅ 로봇 설정
ROBOT_ID = "dsr01"
FIREBASE_KEY = "/home/dell/rokey_robot_arm/backend/rokey-ad6ec-firebase-adminsdk-fbsvc-848d4f8c3c.json"

# ROS2 메시지 임포트 (수정됨: Movej -> MoveJoint)
try:
    from dsr_msgs2.srv import MoveJoint  # 👈 여기가 바뀌었습니다!
    print("📚 [System] ROS2 두산 로봇 메시지(MoveJoint) 로드 성공! 🎉")
except ImportError as e:
    print(f"\n❌ [오류] '{e.name}' 모듈을 불러올 수 없습니다.")
    exit()

class RobotBridge(Node):
    def __init__(self):
        super().__init__('firebase_ros_bridge')
        
        # 👈 서비스 이름도 'move_joint'로 변경될 가능성이 높습니다.
        self.cli_move_joint = self.create_client(MoveJoint, f'/{ROBOT_ID}/motion/move_joint')
        
        print("⏳ ROS2 서비스 연결 대기 중... (터미널 1의 ros2 launch가 켜져 있어야 함)")
        while not self.cli_move_joint.wait_for_service(timeout_sec=2.0):
            print(f'   ... /{ROBOT_ID}/motion/move_joint 서비스를 찾는 중 ...')
        print("✅ [System] ROS2 로봇 연결 완료! (명령 대기 중)")

    def move_joint(self, joints):
        # 👈 요청 객체 생성도 MoveJoint.Request()로 변경
        req = MoveJoint.Request()
        req.pos = [float(x) for x in joints]
        req.vel = 10.0
        req.acc = 20.0
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        req.blendType = 0
        req.syncType = 0
        
        print(f"📤 로봇 전송 -> {req.pos}")
        future = self.cli_move_joint.call_async(req)

def main():
    rclpy.init()
    bridge = RobotBridge()
    
    if not firebase_admin._apps:
        cred = credentials.Certificate(FIREBASE_KEY)
        firebase_admin.initialize_app(cred, {
            'databaseURL': 'https://rokey-ad6ec-default-rtdb.asia-southeast1.firebasedatabase.app'
        })
    
    def on_joint_change(event):
        if not event.data: return
        try:
            target_joints = event.data
            if isinstance(target_joints, list):
                print(f"🦾 [Web->ROS] 이동 명령: {target_joints}")
                bridge.move_joint(target_joints)
        except Exception as e:
            print(f"❌ 오류: {e}")

    print("🌐 웹 제어 서버 시작! (ROS2 모드)")
    db.reference('robot_control/joints').listen(on_joint_change)

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print("\n🛑 서버 종료")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()