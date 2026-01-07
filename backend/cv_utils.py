import cv2
import numpy as np

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
    """가중 평균을 이용하여 경로의 꺾임을 부드럽게 만듭니다."""
    if len(points) < 3 or iterations <= 0: return points
    smoothed = points.copy()
    for _ in range(iterations):
        temp = smoothed.copy()
        temp[1:-1] = 0.25 * smoothed[:-2] + 0.5 * smoothed[1:-1] + 0.25 * smoothed[2:]
        smoothed = temp
    return smoothed

def dist2(a, b):
    """두 점 사이의 거리의 제곱을 반환합니다."""
    return (a[0]-b[0])**2 + (a[1]-b[1])**2

# ============================
# 2. 경로 최적화 알고리즘 (Core Drawing Algorithms)
# ============================

def remove_overlapping_contours(contours, w, h, thickness=3, overlap_thresh=0.6):
    """점유 맵을 사용하여 중복된 경로를 제거합니다."""
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

def connect_broken_contours_smart(contours, max_gap, force_gap=5.0):
    """끊어진 경로들을 지능적으로 연결합니다."""
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
    """Greedy 정렬로 로봇의 이동 효율을 최적화합니다."""
    if not contours: return []
    ordered = [contours.pop(0)]
    while contours:
        current_pt = ordered[-1][-1]
        best_idx, best_dist, reverse_flag = -1, float('inf'), False
        
        for i, cnt in enumerate(contours):
            d_start = dist2(current_pt, cnt[0])
            d_end = dist2(current_pt, cnt[-1])
            
            if d_start < best_dist:
                best_dist, best_idx, reverse_flag = d_start, i, False
            if d_end < best_dist:
                best_dist, best_idx, reverse_flag = d_end, i, True
        
        nxt = contours.pop(best_idx)
        if reverse_flag: nxt = nxt[::-1]
        ordered.append(nxt)
    
    return ordered

# ============================
# 3. 이미지 분석 핵심 로직 (Detection Pipeline)
# ============================

def preprocess_image_smart(img_gray, target_size, padding_ratio=0.8):
    """이미지 전처리 및 이진화."""
    h, w = img_gray.shape
    scale = (target_size * padding_ratio) / max(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    
    resized = cv2.resize(img_gray, (new_w, new_h), interpolation=cv2.INTER_AREA)
    canvas = np.full((target_size, target_size), 255, dtype=np.uint8)
    
    x_off, y_off = (target_size - new_w)//2, (target_size - new_h)//2
    canvas[y_off:y_off+new_h, x_off:x_off+new_w] = resized
    
    _, binary = cv2.threshold(canvas, 127, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    kernel = np.ones((3,3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    return binary

def get_combined_paths(binary_img, min_area=15, max_eye_area=1500):
    """외곽선과 뼈대 추출을 조합하여 드로잉 경로를 생성합니다."""
    final_paths = []
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary_img, connectivity=8)
    mask_for_skeleton = np.zeros_like(binary_img)

    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < min_area: continue 
        
        # 작은 객체는 외곽선 추출 (테스트 코드와 동일하게 1500 이하 기준)
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