#!/usr/bin/env python3
import cv2
import numpy as np
import os
import firebase_admin
from firebase_admin import credentials, db

# ============================
# 1. 설정 (Configuration)
# ============================
IMAGE_PATH = "/home/dell/rokey_robot_arm/frontend/basic_print/custom/2.png"
OUTPUT_DIR = "/home/dell/rokey_robot_arm/frontend/basic_print/custom/output"
CSV_POINTS = os.path.join(OUTPUT_DIR, "robot_path_points.csv")
CSV_BREAKS = os.path.join(OUTPUT_DIR, "contour_lengths.csv")
os.makedirs(OUTPUT_DIR, exist_ok=True)

# [🔥 Firebase 설정]
FIREBASE_KEY_PATH = "/home/dell/rokey_robot_arm/backend/rokey-ad6ec-firebase-adminsdk-fbsvc-848d4f8c3c.json"
FIREBASE_DB_URL = "https://rokey-ad6ec-default-rtdb.asia-southeast1.firebasedatabase.app"
FIREBASE_NODE = "test_drawing"  # DB 내 저장될 노드 이름

# 로봇 기준 파라미터
CAKE_DIAMETER_MM = 300.0
ROBOT_CENTER_X = 0.0
ROBOT_CENTER_Y = 0.0
ROBOT_CENTER_Z = -155.0
FLIP_Y_AXIS = True

# 이미지 전처리 파라미터
TARGET_SIZE_PX = 480       

# [⭐ 중복선 제거 설정]
DEDUP_THICKNESS_PX = 6     
DEDUP_OVERLAP_RATIO = 0.6  

# 좌표 보정 파라미터
APPROX_EPSILON = 1.0       
SMOOTHING_ITERATIONS = 2   
RESAMPLE_STEP_PX = 2.0     
MIN_PERIMETER = 40         

# 끊긴 선 연결 설정
MAX_CONNECTION_GAP_PX = 15.0 
FORCE_CONNECT_GAP_PX = 5.0

# 시각화
SHOW_PREVIEW = True

# ============================
# 2. Firebase 유틸리티
# ============================
def init_firebase():
    """Firebase 앱 초기화 (중복 초기화 방지)"""
    if not firebase_admin._apps:
        try:
            cred = credentials.Certificate(FIREBASE_KEY_PATH)
            firebase_admin.initialize_app(cred, {
                'databaseURL': FIREBASE_DB_URL
            })
            print("🔥 Firebase 연결 성공!")
        except Exception as e:
            print(f"❌ Firebase 연결 실패: {e}")
            return False
    return True

def upload_paths_to_firebase(paths_xyz):
    """
    Numpy 좌표 데이터를 Firebase에 업로드하기 좋은 JSON 형태로 변환하여 전송
    구조: [ 
        [{x:1, y:2, z:3}, {x:4, y:5, z:6}, ...],  # 획 1
        [{x:7, y:8, z:9}, ...],                   # 획 2
        ...
    ]
    """
    formatted_data = []
    
    for path in paths_xyz:
        stroke_points = []
        for point in path:
            # numpy float32 -> native python float 변환 필수 (JSON 직렬화 위해)
            stroke_points.append({
                "x": float(point[0]),
                "y": float(point[1]),
                "z": float(point[2])
            })
        formatted_data.append(stroke_points)
    
    try:
        ref = db.reference(FIREBASE_NODE)
        ref.set(formatted_data) # 기존 데이터 덮어쓰기 (set)
        print(f"🔥 Firebase 업로드 완료! (총 {len(formatted_data)}개의 획)")
    except Exception as e:
        print(f"❌ 데이터 업로드 실패: {e}")

# ============================
# 3. 이미지 처리 유틸리티 (Math)
# ============================
def resample_polyline(pts, step):
    if len(pts) < 2: return pts
    diffs = pts[1:] - pts[:-1]
    seglens = np.sqrt((diffs**2).sum(axis=1))
    total = seglens.sum()
    if total < 1e-6: return pts

    n = max(2, int(total / step) + 1)
    dist = np.linspace(0, total, n)
    cum = np.concatenate([[0], np.cumsum(seglens)])
    out = []
    j = 0
    for d in dist:
        while j < len(cum)-2 and d > cum[j+1]: j += 1
        d0, d1 = cum[j], cum[j+1]
        t = (d - d0) / max(d1 - d0, 1e-9)
        p = pts[j] * (1 - t) + pts[j+1] * t
        out.append(p)
    return np.array(out, dtype=np.float32)

def smooth_polyline(points, iterations=1):
    if len(points) < 3 or iterations <= 0: return points
    smoothed = points.copy()
    for _ in range(iterations):
        temp = smoothed.copy()
        temp[1:-1] = 0.25 * smoothed[:-2] + 0.5 * smoothed[1:-1] + 0.25 * smoothed[2:]
        smoothed = temp
    return smoothed

def dist2(a, b):
    return (a[0]-b[0])**2 + (a[1]-b[1])**2

# ============================
# 4. 중복 제거 및 연결 알고리즘
# ============================
def remove_overlapping_contours(contours, w, h, thickness=5, overlap_thresh=0.6):
    if not contours: return []
    sorted_contours = sorted(contours, key=lambda x: cv2.arcLength(x.reshape(-1,2).astype(np.float32), False), reverse=True)
    kept_contours = []
    occupancy_map = np.zeros((h, w), dtype=np.uint8)
    
    for cnt in sorted_contours:
        pts = cnt.reshape(-1, 2).astype(np.int32)
        pts[:, 0] = np.clip(pts[:, 0], 0, w-1)
        pts[:, 1] = np.clip(pts[:, 1], 0, h-1)
        
        overlap_count = 0
        total_points = len(pts)
        if total_points == 0: continue
        
        pixel_values = occupancy_map[pts[:, 1], pts[:, 0]]
        overlap_count = np.count_nonzero(pixel_values)
        overlap_ratio = overlap_count / total_points
        
        if overlap_ratio < overlap_thresh:
            kept_contours.append(cnt)
            cv2.polylines(occupancy_map, [pts], False, 255, thickness=thickness)
    return kept_contours

def connect_broken_contours_smart(contours, max_gap, force_gap):
    if not contours: return []
    pool = [c.reshape(-1, 2) for c in contours]
    merged = []
    
    while pool:
        current = list(pool.pop(0))
        while True:
            head, tail = current[0], current[-1]
            best_idx, best_mode, min_dist = -1, "", max_gap
            
            for i, other in enumerate(pool):
                o_head, o_tail = other[0], other[-1]
                d_th = np.linalg.norm(tail - o_head)
                d_tt = np.linalg.norm(tail - o_tail)
                d_ht = np.linalg.norm(head - o_tail)
                d_hh = np.linalg.norm(head - o_head)
                
                if d_th < min_dist: min_dist, best_idx, best_mode = d_th, i, 'th'
                if d_tt < min_dist: min_dist, best_idx, best_mode = d_tt, i, 'tt'
                if d_ht < min_dist: min_dist, best_idx, best_mode = d_ht, i, 'ht'
                if d_hh < min_dist: min_dist, best_idx, best_mode = d_hh, i, 'hh'
            
            if best_idx != -1:
                match = list(pool.pop(best_idx))
                if best_mode == 'th': current.extend(match)
                elif best_mode == 'tt': current.extend(match[::-1])
                elif best_mode == 'ht': current = match + current
                elif best_mode == 'hh': current = match[::-1] + current
            else:
                break
        merged.append(np.array(current, dtype=np.float32))
    return merged

def sort_contours_nearest(contours):
    if not contours: return []
    lengths = [len(c) for c in contours]
    start_idx = np.argmax(lengths)
    ordered = [contours[start_idx]]
    remaining = [c for i, c in enumerate(contours) if i != start_idx]
    current_pt = ordered[-1][-1]
    
    while remaining:
        best_idx, best_dist, reverse_flag = -1, float('inf'), False
        for i, cnt in enumerate(remaining):
            d_start = dist2(current_pt, cnt[0])
            d_end = dist2(current_pt, cnt[-1])
            if d_start < best_dist: best_dist, best_idx, reverse_flag = d_start, i, False
            if d_end < best_dist: best_dist, best_idx, reverse_flag = d_end, i, True
        
        nxt = remaining.pop(best_idx)
        if reverse_flag: nxt = nxt[::-1]
        ordered.append(nxt)
        current_pt = ordered[-1][-1]
    return ordered

# ============================
# 5. 전처리 및 스켈레톤
# ============================
def preprocess_image_smart(img_gray, target_size):
    h, w = img_gray.shape
    scale = target_size / max(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(img_gray, (new_w, new_h), interpolation=cv2.INTER_AREA)
    canvas = np.full((target_size, target_size), 255, dtype=np.uint8)
    x_off, y_off = (target_size - new_w)//2, (target_size - new_h)//2
    canvas[y_off:y_off+new_h, x_off:x_off+new_w] = resized
    _, binary = cv2.threshold(canvas, 127, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
    return binary

def get_skeleton(binary_img):
    if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
        try:
            return cv2.ximgproc.thinning(binary_img, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
        except: pass
    
    skel = np.zeros_like(binary_img)
    temp = binary_img.copy()
    cross = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
    while cv2.countNonZero(temp) > 0:
        eroded = cv2.erode(temp, cross)
        opened = cv2.dilate(eroded, cross)
        subset = cv2.subtract(temp, opened)
        skel = cv2.bitwise_or(skel, subset)
        temp = eroded
    return skel

def normalize_uv(pts, w, h):
    uv = np.zeros_like(pts, dtype=np.float32)
    uv[:,0] = (pts[:,0] - w/2) / (w/2)
    uv[:,1] = (pts[:,1] - h/2) / (h/2)
    if FLIP_Y_AXIS: uv[:,1] *= -1
    return uv

def uv_to_robot(uv):
    R = CAKE_DIAMETER_MM / 2
    X = ROBOT_CENTER_X + uv[:,0] * R
    Y = ROBOT_CENTER_Y + uv[:,1] * R
    Z = np.full(len(uv), ROBOT_CENTER_Z)
    return np.stack([X,Y,Z], axis=1)

# ============================
# 6. 메인 실행
# ============================
def main():
    # Firebase 초기화
    init_firebase()

    img = cv2.imread(IMAGE_PATH, cv2.IMREAD_GRAYSCALE)
    if img is None: 
        print(f"❌ 이미지를 찾을 수 없습니다: {IMAGE_PATH}")
        return

    # 1. 전처리 (Resize + Threshold)
    proc_img = preprocess_image_smart(img, TARGET_SIZE_PX)
    H, W = proc_img.shape

    # 2. 스켈레톤 (뼈대 추출)
    skel = get_skeleton(proc_img)

    # 3. 기본 컨투어 추출 및 보정
    contours, _ = cv2.findContours(skel, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    raw_contours = []
    for cnt in contours:
        if cv2.arcLength(cnt, False) < MIN_PERIMETER: continue
        pts = cnt.reshape(-1, 2).astype(np.float32)
        pts = cv2.approxPolyDP(pts, APPROX_EPSILON, False).reshape(-1, 2)
        pts = resample_polyline(pts, RESAMPLE_STEP_PX)
        pts = smooth_polyline(pts, SMOOTHING_ITERATIONS)
        pts = resample_polyline(pts, RESAMPLE_STEP_PX)
        if len(pts) > 1: raw_contours.append(pts)
    
    print(f"🔹 1차 추출 개수: {len(raw_contours)}")

    # 4. 중복선 제거
    unique_contours = remove_overlapping_contours(
        raw_contours, W, H, 
        thickness=DEDUP_THICKNESS_PX, 
        overlap_thresh=DEDUP_OVERLAP_RATIO
    )
    print(f"🔹 중복 제거 후 개수: {len(unique_contours)}")

    # 5. 끊긴 선 연결
    connected_contours = connect_broken_contours_smart(
        unique_contours, 
        max_gap=MAX_CONNECTION_GAP_PX, 
        force_gap=FORCE_CONNECT_GAP_PX
    )
    print(f"🔹 연결 후 개수: {len(connected_contours)}")

    # 6. 정렬
    final_contours = sort_contours_nearest(connected_contours)
    print(f"🔹 최종 경로 개수: {len(final_contours)}")

    # 7. 데이터 변환 및 시각화
    paths_xyz = []
    lengths = []
    vis = cv2.cvtColor(proc_img, cv2.COLOR_GRAY2BGR)
    vis = 255 - vis # 배경 흰색 반전

    for i, pts in enumerate(final_contours):
        uv = normalize_uv(pts, W, H)
        xyz = uv_to_robot(uv)
        paths_xyz.append(xyz)
        lengths.append(len(xyz))

        # Draw
        pts_int = pts.astype(np.int32)
        color = ((i*60)%180, (i*140)%180, 200)
        cv2.polylines(vis, [pts_int], False, color, 2, cv2.LINE_AA)
        cv2.putText(vis, str(i), tuple(pts_int[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)

    # [🔥 Firebase 업로드]
    if paths_xyz:
        print("🚀 Firebase 업로드 시작...")
        upload_paths_to_firebase(paths_xyz)
    else:
        print("⚠️ 업로드할 경로 데이터가 없습니다.")

    # Save CSV (Backup)
    all_pts = np.vstack(paths_xyz) if paths_xyz else np.zeros((0,3))
    np.savetxt(CSV_POINTS, all_pts, delimiter=",", header="X,Y,Z", comments="", fmt="%.3f")
    np.savetxt(CSV_BREAKS, np.array(lengths), delimiter=",", header="length", comments="", fmt="%d")
    
    preview_file = os.path.join(OUTPUT_DIR, "preview_final.png")
    cv2.imwrite(preview_file, vis)

    print(f"✅ 로컬 저장 완료! 미리보기: {preview_file}")
    if SHOW_PREVIEW:
        cv2.imshow("Final Result", vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()