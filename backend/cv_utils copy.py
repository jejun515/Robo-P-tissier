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
def remove_overlapping_contours(contours, w, h, thickness=5, overlap_thresh=0.6):
    """겹치는(중복된) 경로를 제거합니다."""
    if not contours: return []
    # 길이가 긴 순서대로 정렬
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
        
        # 이미 그려진 영역과 얼마나 겹치는지 확인
        pixel_values = occupancy_map[pts[:, 1], pts[:, 0]]
        overlap_count = np.count_nonzero(pixel_values)
        overlap_ratio = overlap_count / total_points
        
        # 겹치는 비율이 낮을 때만 추가
        if overlap_ratio < overlap_thresh:
            kept_contours.append(cnt)
            # 지도에 현재 경로 표시 (두께를 주어 주변 영역 선점)
            cv2.polylines(occupancy_map, [pts], False, 255, thickness=thickness)
    return kept_contours

def connect_broken_contours_smart(contours, max_gap, force_gap):
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
                # 4가지 연결 경우의 수 (Head-Head, Head-Tail, Tail-Head, Tail-Tail)
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
                # 방향에 맞춰 연결
                if best_mode == 'th': current.extend(match)
                elif best_mode == 'tt': current.extend(match[::-1])
                elif best_mode == 'ht': current = match + current
                elif best_mode == 'hh': current = match[::-1] + current
            else:
                break
        merged.append(np.array(current, dtype=np.float32))
    return merged

def sort_contours_nearest(contours):
    """로봇 이동 경로를 최소화하도록 그리는 순서를 정렬합니다."""
    if not contours: return []
    lengths = [len(c) for c in contours]
    start_idx = np.argmax(lengths) # 가장 긴 선부터 시작
    ordered = [contours[start_idx]]
    remaining = [c for i, c in enumerate(contours) if i != start_idx]
    current_pt = ordered[-1][-1] # 현재 붓의 위치
    
    while remaining:
        best_idx, best_dist, reverse_flag = -1, float('inf'), False
        for i, cnt in enumerate(remaining):
            d_start = dist2(current_pt, cnt[0])
            d_end = dist2(current_pt, cnt[-1])
            # 시작점이나 끝점 중 더 가까운 곳을 찾음
            if d_start < best_dist: best_dist, best_idx, reverse_flag = d_start, i, False
            if d_end < best_dist: best_dist, best_idx, reverse_flag = d_end, i, True
        
        nxt = remaining.pop(best_idx)
        if reverse_flag: nxt = nxt[::-1] # 필요하면 선의 방향 뒤집기
        ordered.append(nxt)
        current_pt = ordered[-1][-1]
    return ordered

# ============================
# 3. 이미지 전처리 함수 (Image Preprocessing)
# ============================
def preprocess_image_smart(img_gray, target_size, padding_ratio=0.8):
    """
    이미지를 리사이즈하고 이진화(Binary) 처리합니다.
    
    Args:
        img_gray: 입력 흑백 이미지
        target_size: 캔버스 크기 (정사각형)
        padding_ratio: 이미지가 캔버스 내에서 차지할 비율 (기본 0.8 = 80%)
                       값이 작을수록 여백이 커져 그림이 케이크 안쪽으로 안전하게 들어옵니다.
    """
    h, w = img_gray.shape
    
    # 1. 스케일 계산 (여백 비율 적용)
    # 원본 비율 유지하면서 target_size * padding_ratio 크기에 맞춤
    effective_size = target_size * padding_ratio
    scale = effective_size / max(h, w)
    
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(img_gray, (new_w, new_h), interpolation=cv2.INTER_AREA)
    
    # 2. 캔버스 중앙 정렬 (흰색 배경)
    canvas = np.full((target_size, target_size), 255, dtype=np.uint8)
    x_off, y_off = (target_size - new_w)//2, (target_size - new_h)//2
    canvas[y_off:y_off+new_h, x_off:x_off+new_w] = resized
    
    # 3. 이진화 및 노이즈 제거
    # THRESH_BINARY_INV: 글씨(검정)를 흰색(Foreground)으로 반전 (스켈레톤 추출 위해)
    _, binary = cv2.threshold(canvas, 127, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    
    # 작은 노이즈 제거 (Close 연산)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
    
    return binary

def get_skeleton(binary_img):
    """이진화된 이미지에서 뼈대(Skeleton)를 추출합니다."""
    # OpenCV 확장 모듈이 있으면 사용 (더 빠름)
    if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
        try:
            return cv2.ximgproc.thinning(binary_img, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
        except: pass
    
    # 없으면 기본 모폴로지 연산으로 수행
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