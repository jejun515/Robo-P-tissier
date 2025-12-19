import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from sensor_msgs.msg import JointState
from dsr_msgs2.srv import GetRobotState

import firebase_admin
from firebase_admin import credentials, db

import tf2_ros
import time
import math

# ================================
# 🔥 Firebase 설정
# ================================
FIREBASE_KEY_PATH = "/home/jejun/cobot1_ws/src/doosan-robot2/dsr_rokey2/dsr_rokey2/rokey-ad6ec-firebase-adminsdk-fbsvc-848d4f8c3c.json"
FIREBASE_DB_URL = "https://rokey-ad6ec-default-rtdb.asia-southeast1.firebasedatabase.app"
FIREBASE_DB_PATH = "/robots/dsr01/snapshot"


class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # -------------------------------
        # Firebase 초기화
        # -------------------------------
        cred = credentials.Certificate(FIREBASE_KEY_PATH)
        firebase_admin.initialize_app(cred, {'databaseURL': FIREBASE_DB_URL})
        self.db_ref = db.reference(FIREBASE_DB_PATH)

        # -------------------------------
        # 서비스 클라이언트
        # -------------------------------
        self.cli = self.create_client(GetRobotState, '/dsr01/system/get_robot_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /dsr01/system/get_robot_state service...')

        # -------------------------------
        # joint_states 구독
        # -------------------------------
        self.latest_joint_state = None
        self.create_subscription(
            JointState,
            '/dsr01/joint_states',
            self.joint_state_cb,
            10
        )

        # -------------------------------
        # TF listener
        # -------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -------------------------------
        # 퍼블리셔
        # -------------------------------
        self.pub_state = self.create_publisher(Int32, '/dsr01/robot_state', 10)
        self.pub_state_text = self.create_publisher(String, '/dsr01/robot_state_text', 10)

        self.last_state = None
        self.in_flight = False

        # -------------------------------
        # 타이머 (5Hz)
        # -------------------------------
        self.create_timer(0.2, self.tick)

        self.get_logger().info("✅ Robot snapshot → Firebase (state dedup ON)")

    # =================================================
    # joint_states callback
    # =================================================
    def joint_state_cb(self, msg: JointState):
        self.latest_joint_state = msg

    # =================================================
    # 서비스 polling
    # =================================================
    def tick(self):
        if self.in_flight:
            return
        self.in_flight = True
        future = self.cli.call_async(GetRobotState.Request())
        future.add_done_callback(self.on_response)

    def on_response(self, future):
        self.in_flight = False
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return

        state_val = int(res.robot_state)
        state_text = self.state_to_text(state_val)

        # =================================================
        # joint_states → deg (항상)
        # =================================================
        joint_data = {}
        if self.latest_joint_state:
            for name, pos in zip(
                self.latest_joint_state.name,
                self.latest_joint_state.position
            ):
                joint_data[name] = round(math.degrees(pos), 3)

        # =================================================
        # TF → TCP pose (항상)
        # =================================================
        tcp_pose = {}
        try:
            tf = self.tf_buffer.lookup_transform(
                'link_1',   # base (실제 루트)
                'link_6',   # TCP
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )



            t = tf.transform.translation
            q = tf.transform.rotation
            roll, pitch, yaw = self.quat_to_rpy(q.x, q.y, q.z, q.w)

            tcp_pose = {
                "x": round(t.x * 1000, 2),
                "y": round(t.y * 1000, 2),
                "z": round(t.z * 1000, 2),
                "roll":  round(math.degrees(roll), 2),
                "pitch": round(math.degrees(pitch), 2),
                "yaw":   round(math.degrees(yaw), 2),
            }
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

        # =================================================
        # Firebase 업로드 (항상)
        # =================================================
        self.db_ref.set({
            "robot_state": {
                "value": state_val,
                "text": state_text
            },
            "joint_states": joint_data,
            "tcp_pose": tcp_pose,
            "timestamp": time.time()
        })

        # =================================================
        # robot_state 변경 시에만 publish
        # =================================================
        if state_val != self.last_state:
            self.last_state = state_val
            self.pub_state.publish(Int32(data=state_val))
            self.pub_state_text.publish(String(data=state_text))
            self.get_logger().info(f"🔁 Robot state changed → {state_text}")

    # =================================================
    # utils
    # =================================================
    def state_to_text(self, s: int) -> str:
        return {
            0:  'STATE_INITIALIZING',
            1:  'STATE_STANDBY',
            2:  'STATE_MOVING',
            3:  'STATE_SAFE_OFF',
            4:  'STATE_TEACHING',
            5:  'STATE_SAFE_STOP',
            6:  'STATE_EMERGENCY_STOP',
            7:  'STATE_HOMMING',
            8:  'STATE_RECOVERY',
            9:  'STATE_SAFE_STOP2',
            10: 'STATE_SAFE_OFF2',
        }.get(s, f'UNKNOWN_STATE_{s}')

    def quat_to_rpy(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


def main():
    rclpy.init()
    node = RobotStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
