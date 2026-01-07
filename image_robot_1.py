import cv2
import numpy as np
import os
import tkinter as tk
from tkinter import filedialog
import json

# ============================
# 1. 수학 및 유틸리티 함수 (Math Utils)
# ============================
def resample_polyline(pts, step):
    """경로를 일정한 간격(step)으로 재샘플링합니다."""
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
    """경로를 부드럽게 만듭니다 (Smoothing)."""
    if len(points) < 3 or iterations <= 0: return points
    smoothed = points.copy()
    for _ in range(iterations):
        temp = smoothed.copy()
        temp[1:-1] = 0.25 * smoothed[:-2] + 0.5 * smoothed[1:-1] + 0.25 * smoothed[2:]
        smoothed = temp
    return smoothed

def dist2(a, b):
    """두 점 사이의 거리 제곱을 반환합니다."""
    return (a[0]-b[0])**2 + (a[1]-b[1])**2

# ============================
# 2. 알고리즘 핵심 로직 (Core Algorithms)
# ============================
def remove_overlapping_contours(contours, w, h, thickness=3, overlap_thresh=0.7):
    """중복된 경로를 제거합니다."""
    if not contours: return []
    sorted_contours = sorted(contours, key=lambda x: cv2.arcLength(x.reshape(-1,2).astype(np.float32), False), reverse=True)
    kept_contours = []
    occupancy_map = np.zeros((h, w), dtype=np.uint8)
    for cnt in sorted_contours:
        pts = cnt.reshape(-1, 2).astype(np.int32)
        pts[:, 0] = np.clip(pts[:, 0], 0, w-1)
        pts[:, 1] = np.clip(pts[:, 1], 0, h-1)
        pixel_values = occupancy_map[pts[:, 1], pts[:, 0]]
        overlap_count = np.count_nonzero(pixel_values)
        overlap_ratio = overlap_count / len(pts)
        if overlap_ratio < overlap_thresh:
            kept_contours.append(cnt)
            cv2.polylines(occupancy_map, [pts], False, 255, thickness=thickness)
    return kept_contours

def connect_broken_contours_smart(contours, max_gap):
    """끊어진 선들을 지능적으로 연결합니다."""
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
    """로봇 이동 경로를 최소화하도록 정렬합니다."""
    if not contours: return []
    ordered = [contours.pop(0)]
    current_pt = ordered[-1][-1]
    while contours:
        best_idx, best_dist, reverse_flag = -1, float('inf'), False
        for i, cnt in enumerate(contours):
            d_start = dist2(current_pt, cnt[0])
            d_end = dist2(current_pt, cnt[-1])
            if d_start < best_dist: best_dist, best_idx, reverse_flag = d_start, i, False
            if d_end < best_dist: best_dist, best_idx, reverse_flag = d_end, i, True
        nxt = contours.pop(best_idx)
        if reverse_flag: nxt = nxt[::-1]
        ordered.append(nxt)
        current_pt = ordered[-1][-1]
    return ordered

# ============================
# 3. 이미지 전처리 및 고급 검출
# ============================
def preprocess_image_smart(img_gray, target_size, padding_ratio=0.8):
    """이미지 리사이즈 및 이진화."""
    h, w = img_gray.shape
    scale = (target_size * padding_ratio) / max(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(img_gray, (new_w, new_h), interpolation=cv2.INTER_AREA)
    canvas = np.full((target_size, target_size), 255, dtype=np.uint8)
    x_off, y_off = (target_size - new_w)//2, (target_size - new_h)//2
    canvas[y_off:y_off+new_h, x_off:x_off+new_w] = resized
    _, binary = cv2.threshold(canvas, 127, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    return binary

def get_combined_paths(binary_img, min_area=15, max_eye_area=1000):
    """외곽선 및 세선화(뼈대)를 조합하여 경로 추출."""
    final_paths = []
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary_img, connectivity=8)
    mask_for_skeleton = np.zeros_like(binary_img)

    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < min_area: continue 
        if area <= max_eye_area:
            component_mask = (labels == i).astype(np.uint8) * 255
            cnts, _ = cv2.findContours(component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                pts = c.reshape(-1, 2).astype(np.float32)
                pts = np.vstack([pts, pts[0]]) 
                final_paths.append(pts)
        else:
            mask_for_skeleton[labels == i] = 255

    if cv2.countNonZero(mask_for_skeleton) > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4))
        mask_for_skeleton = cv2.dilate(mask_for_skeleton, kernel, iterations=1)
        mask_for_skeleton = cv2.morphologyEx(mask_for_skeleton, cv2.MORPH_CLOSE, kernel)
        
        if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
            skel = cv2.ximgproc.thinning(mask_for_skeleton, thinningType=cv2.ximgproc.THINNING_GUOHALL)
        else:
            skel = np.zeros_like(mask_for_skeleton)
            temp = mask_for_skeleton.copy()
            element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
            while True:
                eroded = cv2.erode(temp, element)
                temp_open = cv2.dilate(eroded, element)
                subset = cv2.subtract(temp, temp_open)
                skel = cv2.bitwise_or(skel, subset)
                temp = eroded.copy()
                if cv2.countNonZero(temp) == 0: break
        
        skel_cnts, _ = cv2.findContours(skel, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        for c in skel_cnts:
            if cv2.arcLength(c, False) > 15: 
                final_paths.append(c.reshape(-1, 2).astype(np.float32))

    return final_paths

# ============================
# 4. 저장 함수 (좌표 변환 및 CSV 분리 저장)
# ============================
def transform_and_save_csv(paths, base_output_name, img_size, real_size_mm, z_height=-150):
    """
    경로 데이터를 로봇 좌표계로 변환 후 두 개의 CSV 파일로 저장합니다.
    1. _points.csv: X,Y,Z (중심 원점, mm 단위)
    2. _lengths.csv: 각 경로(획)의 점 개수
    """
    paths_xyz = []
    lengths_log = []
    
    # 캔버스 중심 좌표
    cx, cy = img_size / 2, img_size / 2
    # 픽셀 -> mm 스케일 (이미지 너비 = 실제 크기)
    scale_factor = real_size_mm / img_size 

    for path in paths:
        # path: (N, 2) 픽셀 좌표
        if len(path) == 0: continue
        
        # 1. 중심 원점으로 이동
        pts_centered = path - [cx, cy]
        
        # 2. mm 단위 스케일링
        pts_mm = pts_centered * scale_factor
        
        # 3. Y축 반전 (이미지: 아래가 +, 로봇: 위가 +)
        pts_mm[:, 1] *= -1
        
        # 4. Z축 추가
        n_points = len(pts_mm)
        z_vals = np.full((n_points, 1), z_height, dtype=np.float32)
        xyz = np.hstack([pts_mm, z_vals])
        
        paths_xyz.append(xyz)
        lengths_log.append(n_points)

    if not paths_xyz:
        print("[경고] 저장할 경로가 없습니다.")
        return

    all_pts = np.vstack(paths_xyz)
    
    csv_points = base_output_name + "_points.csv"
    csv_lengths = base_output_name + "_lengths.csv"
    
    # 포맷: 소수점 3자리까지 저장
    np.savetxt(csv_points, all_pts, delimiter=",", header="X,Y,Z", comments="", fmt="%.3f")
    np.savetxt(csv_lengths, np.array(lengths_log), delimiter=",", header="length", comments="", fmt="%d")
    
    print(f"[알림] 좌표 변환 및 저장이 완료되었습니다. (크기: {real_size_mm}mm)")
    print(f" - 좌표 파일: {csv_points}")
    print(f" - 길이 파일: {csv_lengths}")

# ============================
# 5. 메인 실행
# ============================
def main():
    print("🚀 로봇 드로잉 경로 추출 및 저장기 시작...")
    root = tk.Tk()
    root.withdraw()
    root.attributes('-topmost', True)
    
    file_path = filedialog.askopenfilename(title="이미지 선택", filetypes=[("Images", "*.jpg *.png *.jpeg *.webp")])
    if not file_path: return
    
    img = cv2.imdecode(np.fromfile(file_path, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if img is None: return
    
    # [설정] 이미지 처리 크기 및 실제 출력 크기
    TARGET_SIZE = 640
    REAL_DIAMETER_MM = 300.0  # 실제 작업 영역 크기 (예: 케이크 지름)
    
    binary = preprocess_image_smart(img, TARGET_SIZE)
    
    raw_paths = get_combined_paths(binary, min_area=15, max_eye_area=1500)
    
    processed = []
    for p in raw_paths:
        p = resample_polyline(p, 2.0)
        p = smooth_polyline(p, 2)
        if len(p) > 1: processed.append(p)
        
    unique = remove_overlapping_contours(processed, TARGET_SIZE, TARGET_SIZE, thickness=3, overlap_thresh=0.6)
    connected = connect_broken_contours_smart(unique, max_gap=10.0)
    final = sort_contours_nearest(connected)
    
    # --- 좌표 변환 및 저장 ---
    base_name = os.path.splitext(file_path)[0]
    # 여기서 이미지 크기(TARGET_SIZE)와 실제 크기(REAL_DIAMETER_MM)를 전달합니다.
    transform_and_save_csv(final, base_name, TARGET_SIZE, REAL_DIAMETER_MM, z_height=-155)
    
    # --- 시각화 (확인용은 픽셀 좌표 그대로 표시) ---
    vis = np.full((TARGET_SIZE, TARGET_SIZE, 3), 255, dtype=np.uint8)
    for i, p in enumerate(final):
        hue = int(170 * (i / max(len(final), 1)))
        color = cv2.cvtColor(np.uint8([[[hue, 255, 200]]]), cv2.COLOR_HSV2BGR)[0][0].tolist()
        cv2.polylines(vis, [p.astype(np.int32)], False, color, 2, cv2.LINE_AA)
        cv2.circle(vis, tuple(p.astype(np.int32)[0]), 3, (0,0,255), -1)
    
    # 중심점 표시 (빨간 십자가)
    cx, cy = TARGET_SIZE // 2, TARGET_SIZE // 2
    cv2.drawMarker(vis, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
    
    cv2.imshow("Detection Result", np.hstack([cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR), vis]))
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()