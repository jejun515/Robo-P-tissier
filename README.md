# 🤖 Robot State Publisher: Real-time Digital Twin Sync
두산 로봇(Doosan Robotics)의 물리적 상태 정보를 실시간으로 수집하여 클라우드(Firebase)와 동기화하는 미들웨어 모듈입니다.

# 📖 Project Overview
이 프로젝트는 물리 로봇과 웹/모바일 클라이언트를 연결하는 Bridge Node입니다.
ROS 2 네트워크 상의 로봇 관절 정보(Joint State)와 TCP 좌표(TF)를 5Hz 주기로 수집 및 가공하여, 원격지에서도 로봇의 현재 상태를 딜레이 없이 모니터링할 수 있도록 Digital Twin 데이터를 생성합니다.

# 🛠 Tech Stack
- Languages & Environment

<img src="https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white"> <img src="https://img.shields.io/badge/ROS 2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white"> <img src="https://img.shields.io/badge/Ubuntu-22.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white">

- Core Libraries & Cloud

<img src="https://img.shields.io/badge/Firebase-FFCA28?style=for-the-badge&logo=firebase&logoColor=black"> <img src="https://img.shields.io/badge/TF2_ROS-Geometry-blue?style=for-the-badge&logo=ros&logoColor=white"> <img src="https://img.shields.io/badge/DSR_Msgs2-Doosan-C70025?style=for-the-badge&logo=robot&logoColor=white">

## ⚙️ Synchronization Logic
로봇 데이터가 클라우드로 업로드되기까지의 핵심 로직입니다.
### 1. State Acquisition (상태 수집)
다양한 소스로부터 비동기적으로 데이터를 수집하여 하나의 스냅샷을 구성합니다.
* Service Polling: get_robot_state 서비스를 호출하여 로봇 컨트롤러의 운용 상태(Manual, Auto, E-Stop 등)를 확인.
* Topic Subscription: /dsr01/joint_states를 구독하여 6개 축의 실시간 관절 각도(Radian) 수집.
### 2. Kinematics Computing (기구학 계산)
단순한 관절 각도 정보를 넘어, 직관적인 3차원 공간 좌표를 계산합니다.
* TF Lookup: TF 트리 상에서 link_1(Base) 대비 link_6(End-Effector)의 변환 행렬 조회.
* Quaternion to Euler: 기계적인 회전 정보(Quaternion)를 사람이 이해하기 쉬운 Roll, Pitch, Yaw로 변환.
### 3. Data Processing (데이터 가공)
* Unit Conversion: 내부 연산용 Radian 값을 UI 표출용 Degree 단위로 변환.
* State Deduplication: 네트워크 대역폭 절약을 위해 로봇의 상태(State Code)가 변경된 순간에만 로컬 토픽(Int32) 발행.
### 4. Cloud Upload (클라우드 동기화)
* Realtime Database: 가공된 전체 데이터를 JSON 객체로 직렬화하여 Firebase의 지정된 경로(/robots/dsr01/snapshot)에 set() 연산 수행 (Update Rate: 5Hz).
## 💻 Code Snippet (Example)
``` 
def on_response(self, future):
    """
    서비스 응답과 TF 데이터를 결합하여 Firebase에 업로드하는 핵심 콜백
    """
    # 1. TF Listener를 통한 TCP 좌표 계산 (Base -> Flange)
    try:
        tf = self.tf_buffer.lookup_transform('link_1', 'link_6', ...)
        t, q = tf.transform.translation, tf.transform.rotation
        
        # Quaternion -> Euler (Roll, Pitch, Yaw) 변환
        roll, pitch, yaw = self.quat_to_rpy(q.x, q.y, q.z, q.w)
        
        tcp_pose = {
            "x": round(t.x * 1000, 2), # m -> mm 변환
            "y": round(t.y * 1000, 2),
            "z": round(t.z * 1000, 2),
            "roll":  round(math.degrees(roll), 2),
            "pitch": round(math.degrees(pitch), 2),
            "yaw":   round(math.degrees(yaw), 2),
        }
    except Exception as e:
        self.get_logger().warn(f"TF lookup failed: {e}")

    # 2. Firebase Realtime Database 업로드
    self.db_ref.set({
        "robot_state": {"value": state_val, "text": state_text},
        "joint_states": joint_data, # Degree 변환된 관절 각도
        "tcp_pose": tcp_pose,       # 계산된 공간 좌표
        "timestamp": time.time()
    })
```
### 📊 Data Structure Visualization
이 노드가 생성하여 관리하는 실시간 데이터 구조입니다. 프론트엔드(React) 및 관제 시스템에서 이 데이터를 구독하여 시각화합니다.

### 🔥 Firebase Realtime DB Structure
| Field | Description | Type | Update Frequency |
| :--- | :--- | :--- | :--- |
| **robot_state** | 로봇의 현재 운용 상태 (코드 및 텍스트) | `Object` | Change Event |
| **joint_states** | 1축~6축의 관절 각도 (Degree) | `Array/Map` | 5Hz (Real-time) |
| **tcp_pose** | 3차원 위치(XYZ) 및 자세(RPY) | `Object` | 5Hz (Real-time) |

### 💾 Snapshot Example (JSON)
실제 DB에 저장되는 데이터 포맷입니다.
JSON
```
{
  "robot_state": {
    "value": 1,
    "text": "STATE_STANDBY"
  },
  "joint_states": {
    "joint_1": 0.05, "joint_2": 15.21, "joint_3": -89.55,
    "joint_4": 0.00, "joint_5": 90.10, "joint_6": 12.45
  },
  "tcp_pose": {
    "x": 450.50, "y": 100.20, "z": 500.00,
    "roll": 180.00, "pitch": 0.00, "yaw": 90.00
  },
  "timestamp": 1704781234.567
}
```
    Data Flow: Robot Controller → ROS 2 Node → Firebase → Web Client
### 🚀 Future Improvements
* Security: Firebase Admin SDK 키 관리 방식을 환경 변수(Environment Variable) 기반으로 고도화.
* Optimization: 데이터 변화량이 임계값 이하일 경우 업로드를 스킵하는 Adaptive Sync 로직 추가 예정.
