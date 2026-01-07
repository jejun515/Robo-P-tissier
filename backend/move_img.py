import rclpy
import DR_init
import numpy as np
import os

# ============================
# 로봇 설정
# ============================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

ROBOT_TOOL = "Tool Weight2"
ROBOT_TCP  = "GripperDA_v2"

VELOCITY = 60
ACC = 60

CSV_POINTS = "robot_path_points.csv"
CSV_BREAKS = "contour_lengths.csv"

Z_HOP = 40.0
DRAW_Z_OFFSET = 2.0
# 드로잉 시 고정 자세
TCP_RX =  55
TCP_RY = 141
TCP_RZ = 100
# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

CSV_DIR = "/home/jejun/Desktop/Test"   # ✅ CSV가 저장된 폴더로 수정
CSV_POINTS = os.path.join(CSV_DIR, "robot_path_points.csv")
CSV_BREAKS = os.path.join(CSV_DIR, "contour_lengths.csv")

# ============================
# CSV 경로 설정
# ============================
def assert_csv_exists():
    print("[DEBUG] CWD:", os.getcwd())
    print("[DEBUG] CSV_POINTS:", CSV_POINTS)
    print("[DEBUG] CSV_BREAKS:", CSV_BREAKS)

    if not os.path.exists(CSV_POINTS):
        raise FileNotFoundError(f"CSV_POINTS not found: {CSV_POINTS}")
    if not os.path.exists(CSV_BREAKS):
        raise FileNotFoundError(f"CSV_BREAKS not found: {CSV_BREAKS}")


# ============================
# CSV 로드
# ============================
def load_paths(points_csv, breaks_csv):
    pts = np.loadtxt(points_csv, delimiter=",", skiprows=1)
    br  = np.loadtxt(breaks_csv, delimiter=",", skiprows=1).astype(int)

    paths = []
    idx = 0
    for n in br:
        n = int(n)
        if n <= 0:
            continue
        seg = pts[idx:idx+n]
        idx += n
        if len(seg) >= 2:
            paths.append(seg)

    return paths


# ============================
# 로봇 초기화
# ============================
def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, posx, set_user_cart_coord, DR_BASE

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    print("===== Robot Initialized =====")
    print(f"ROBOT_ID   : {ROBOT_ID}")
    print(f"MODEL      : {ROBOT_MODEL}")
    print(f"TOOL       : {ROBOT_TOOL}")
    print(f"TCP        : {ROBOT_TCP}")
    print(f"VELOCITY   : {VELOCITY}")
    print(f"ACC        : {ACC}")
    print("=============================")

    # global user
    # user_coord = posx(508,-154,400,0,0,0)
    # user = set_user_cart_coord(user_coord,DR_BASE)



# ============================
# 메인 작업
# ============================
def perform_task():
    from DSR_ROBOT2 import (posx, movej, movel,set_user_cart_coord,DR_BASE,release_compliance_ctrl,release_force,
        check_force_condition,
        task_compliance_ctrl,
        set_desired_force,
        set_ref_coord,
        movej,
        movel,wait,
        DR_FC_MOD_REL,
        DR_AXIS_Z,fkin)
    
    # ============================
    # ✅ USER 좌표계 호환 처리
    # ============================
    # 1) DR_USER 상수 있으면 사용
    # 2) 없으면 ref 후보를 순서대로 시도: 1, "user"
    # 3) set_ref_coord 있으면 호출 시도
    # user_coord = posx(508,-154,400,0,0,0)
    # user = set_user_cart_coord(,DR_BASE)

    user_coord = posx(508,-154,400,-0,-0,-44)
    user = set_user_cart_coord(user_coord,DR_BASE)

    # go_home = posx(0,0,0,0,0,0)
    # movel(go_home,vel=VELOCITY,acc=ACC,ref=user)

    assert_csv_exists()
    # ============================
    # 경로 로드
    # ============================
    paths = load_paths(CSV_POINTS, CSV_BREAKS)
    print(f"[INFO] Loaded paths={len(paths)}")

    # 시작 자세
    
    for i, path in enumerate(paths):
        print(f"[PATH {i}] points={len(path)}")

        p0 = path[0].copy()
        p0[2] += DRAW_Z_OFFSET

        p0_up = p0.copy()
        p0_up[2] += Z_HOP

        # ---------- (A) 펜업 접근 ----------
        p = posx([p0_up[0], p0_up[1], p0_up[2], TCP_RX, TCP_RY, TCP_RZ])
        movel(p, vel=VELOCITY, acc=ACC,ref=user)

        # ---------- (B) 펜다운 ----------
        p = posx([p0[0], p0[1], p0[2], TCP_RX, TCP_RY, TCP_RZ])
        movel(p, vel=VELOCITY, acc=ACC,ref=user)

        # ---------- (C) 경로 이동 ----------
        task_compliance_ctrl(stx=[3000, 3000, 2.5, 200, 200, 200])
        wait(0.5) # 안정화 대기(필수)
        set_desired_force(fd=[0, 0, -4.5, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        for pt in path[1:]:
            x, y, z = float(pt[0]), float(pt[1]), float(pt[2] + DRAW_Z_OFFSET)
            p = posx([x, y, z, TCP_RX, TCP_RY, TCP_RZ])
            movel(p, vel=VELOCITY, acc=ACC,ref=user,radius=1)
        release_force()
        release_compliance_ctrl()
        # ---------- (D) 펜업 ----------
        pend = path[-1].copy()
        pend[2] += DRAW_Z_OFFSET + Z_HOP

        p = posx([pend[0], pend[1], pend[2], TCP_RX, TCP_RY, TCP_RZ])
        movel(p, vel=VELOCITY, acc=ACC,ref=user)
    print("[DONE] all paths executed")


# ============================
# ROS2 엔트리
# ============================
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("draw_from_csv_userframe", namespace=ROBOT_ID)

    DR_init.__dsr__node = node
    try:
        initialize_robot()
        perform_task()
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
