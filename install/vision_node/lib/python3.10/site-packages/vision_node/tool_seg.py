# -*- coding: utf-8 -*-
"""
RealSense + Ultralytics YOLO (Seg/Det 자동 폴백) + 2D OBB(Roll) + Depth
- 라벨 정규화 매칭(공백/하이픈/언더스코어/대소문자 무시)
- 작은 물체 탐지 개선: imgsz, conf/iou 완화
- 학습 파이프라인과 일치하는 색공간 선택(BGR/RGB)
- seg 마스크 없을 때 bbox로 폴백
- OBB 각도 안정화(지수평활)
- 손잡이/날 오인 방지: 폭 + distance transform 반경 + 뾰족도 가중합
- 손잡이/날 중앙점 + 끝점(각 2개) 시각화 = 총 6점
"""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import re
import time
import math
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# =======================
# 사용자 설정
# =======================
WEIGHTS = "/home/wt/Tool_Sample/tool_seg/best.pt"  # seg 또는 det 모두 허용
DEVICE  = "0"

# 탐지/입력 해상도 파라미터 (작은 공구용 권장값)
CONF_TH = 0.28     # 0.25~0.3 사이 권장
IOU_TH  = 0.45     # 0.45 권장
IMG_SIZE = 960     # 896~1024 권장

# 훈련 시 사용한 색공간에 맞추기 (훈련이 BGR이었다면 False, RGB였으면 True)
USE_RGB_INPUT = False

# 표시/후처리
LABEL_FILTER = None
DRAW_MASK_ALPHA = 0.4
FONT = cv2.FONT_HERSHEY_SIMPLEX
EXCLUDE_CLASS_IDS = []
COLOR_W, COLOR_H, COLOR_FPS = 640, 480, 30
DEPTH_W, DEPTH_H, DEPTH_FPS = 640, 480, 30
TARGET_LABELS = {'nipper', 'vernier_calipers', 'wire_cutter', 'wire_stripper'}
SHOW_WINDOW = True
FALLBACK_SAVE_EVERY_N = 15
FALLBACK_SAVE_DIR = "/tmp/tool_pca_frames"

# 깊이 폴백(중앙 패치) 커널
DEPTH_KERNEL = 5

# OBB 각도 평활화
SMOOTHING_ALPHA = 0.2

# 손잡이/날 시각화
DRAW_HANDLE_TIP = True
DRAW_ENDPOINTS  = True   # 끝부분(손잡이 끝/날 끝) 점까지 표시

# 오인 방지 가중치
WIDTH_W   = 0.55  # 폭(직교폭) 가중
RADIUS_W  = 0.35  # distance transform 반경 가중
POINTY_W  = 0.10  # 뾰족도(길이/폭) 가중
WIDTH_MARGIN_RATIO = 0.15  # 폭 차이가 이 비율 이상이면 폭으로 즉결

# =======================
# 유틸 함수
# =======================
def norm_label(s: str) -> str:
    """라벨 비교를 위한 정규화(공백/하이픈/언더스코어 제거, 소문자)"""
    return re.sub(r'[\s\-_]+', '', s.lower())

def apply_mask_overlay(overlay_bgr, mask, alpha=0.4, color=(0, 255, 255)):
    H, W = overlay_bgr.shape[:2]
    if mask.dtype != np.uint8:
        mask = (mask > 0.5).astype(np.uint8)
    if mask.shape[0] != H or mask.shape[1] != W:
        mask = cv2.resize(mask, (W, H), interpolation=cv2.INTER_NEAREST)
    out = overlay_bgr.copy()
    color_img = np.zeros_like(out, dtype=np.uint8)
    color_img[:] = color
    m = mask.astype(bool)
    out[m] = ((1.0 - alpha) * out[m] + alpha * color_img[m]).astype(np.uint8)
    return out

def median_depth_meters_from_center(depth_frame, cx, cy, k=5, depth_scale=0.001):
    depth_image = np.asanyarray(depth_frame.get_data())
    H, W = depth_image.shape[:2]
    x1, x2 = max(0, cx - k // 2), min(W - 1, cx + k // 2)
    y1, y2 = max(0, cy - k // 2), min(H - 1, cy + k // 2)
    patch = depth_image[y1:y2 + 1, x1:x2 + 1]
    if patch.size == 0: return None
    valid = patch[patch > 0]
    if valid.size == 0: return None
    return float(np.median(valid)) * depth_scale

def median_depth_meters_from_mask(depth_frame, mask, depth_scale=0.001):
    depth_image = np.asanyarray(depth_frame.get_data())
    H_d, W_d = depth_image.shape[:2]
    if mask.shape[0] != H_d or mask.shape[1] != W_d:
        mask = cv2.resize(mask.astype(np.uint8), (W_d, H_d), interpolation=cv2.INTER_NEAREST)
    m = mask.astype(bool)
    if not np.any(m): return None
    valid = depth_image[m]
    valid = valid[valid > 0]
    if valid.size == 0: return None
    return float(np.median(valid)) * depth_scale

# ---------- 손잡이/날 판별(폭 + 반경 + 뾰족도) ----------
def _band_metrics_with_dt(mask_bin, band_mask, centered, pts, u_major, v_minor):
    """
    한 끝단 밴드의 지표:
      - width_v : v축 폭
      - area    : 픽셀 수
      - radius_med : distance transform 반경의 중앙값(두께)
      - pointiness : (u 길이)/(v 폭) (클수록 뾰족)
    """
    if not np.any(band_mask):
        return dict(valid=False)

    pts_b = pts[band_mask]       # (Nb,2) 이미지 좌표
    ctr_b = centered[band_mask]  # (Nb,2) 평균 원점 기준

    v_b   = ctr_b @ v_minor
    width_v = float(v_b.max() - v_b.min())
    area    = float(pts_b.shape[0])

    # mask_bin 해상도와 pts 좌표계가 동일해야 함 (이미 동일)
    # 노이즈 완화: 살짝 열기-닫기
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    mask_smooth = cv2.morphologyEx(mask_bin, cv2.MORPH_OPEN, k, iterations=1)
    dt = cv2.distanceTransform(mask_smooth, cv2.DIST_L2, 3).astype(np.float32)

    yy = pts_b[:,1].astype(np.int32).clip(0, mask_bin.shape[0]-1)
    xx = pts_b[:,0].astype(np.int32).clip(0, mask_bin.shape[1]-1)
    radii = dt[yy, xx]
    radius_med = float(np.median(radii)) if radii.size else 0.0

    u_b = ctr_b @ u_major
    u_len = float(u_b.max() - u_b.min()) if u_b.size else 0.0
    pointiness = (u_len + 1e-6) / (width_v + 1e-6)

    return dict(valid=True, width_v=width_v, area=area, radius_med=radius_med, pointiness=pointiness)

def choose_handle_tip(mask_bin, mask_head, mask_tail, centered, pts, u_major, v_minor,
                      width_w=WIDTH_W, radius_w=RADIUS_W, pointy_w=POINTY_W, width_margin_ratio=WIDTH_MARGIN_RATIO):
    """
    손잡이/날 결정:
      1) 폭 차가 충분히 크면 폭으로 즉시 결정
      2) 아니면 (폭 + 반경 + 뾰족도)의 가중합으로 결정
    반환: band_handle_mask, band_tip_mask, decision_conf(0~1)
    """
    mh = _band_metrics_with_dt(mask_bin, mask_head, centered, pts, u_major, v_minor)
    mt = _band_metrics_with_dt(mask_bin, mask_tail, centered, pts, u_major, v_minor)

    if not mh["valid"] or not mt["valid"]:
        # 하나가 비정상 → 폭 기준으로라도 결정
        if mh.get("width_v",0) >= mt.get("width_v",0):
            return mask_head, mask_tail, 0.5
        else:
            return mask_tail, mask_head, 0.5

    wh, wt = mh["width_v"], mt["width_v"]
    big = max(wh, wt) + 1e-6
    width_gap_ratio = abs(wh - wt) / big

    # 1) 폭 차가 충분하면 즉결
    if wh > wt * (1.0 + width_margin_ratio):
        return mask_head, mask_tail, min(1.0, 0.6 + 0.4*width_gap_ratio)
    if wt > wh * (1.0 + width_margin_ratio):
        return mask_tail, mask_head, min(1.0, 0.6 + 0.4*width_gap_ratio)

    # 2) 가중합 점수
    width_score  = (wh - wt) / big  # 양수면 head가 넓음 → head=손잡이
    rh, rt       = mh["radius_med"], mt["radius_med"]
    radius_score = (rh - rt) / (max(rh, rt) + 1e-6)  # 양수면 head가 두꺼움 → head=손잡이
    ph, pt       = mh["pointiness"], mt["pointiness"]
    pointy_score = (pt - ph)  # 양수면 tail이 더 뾰족 → tail=날

    combined = width_w*width_score + radius_w*radius_score + pointy_w*pointy_score

    if combined >= 0:
        conf = min(1.0, 0.5 + 0.5*abs(combined))
        return mask_head, mask_tail, conf
    else:
        conf = min(1.0, 0.5 + 0.5*abs(combined))
        return mask_tail, mask_head, conf

# ---------- OBB & 손잡이/날(중앙점+끝점2) ----------
def obb_handle_tip_from_mask(mask, out_size_hw, end_band_ratio=0.18, min_pts=30):
    """
    세그 마스크로 OBB 계산하되, x축을 '손잡이 → 날' 방향으로 강제 정렬.
    - 각 끝단 밴드에서 v축 최상/최하 끝점 2개를 잡고 그 '중점'을 중앙점으로 사용
    - 손잡이/날 판별은 (폭+반경+뾰족도) 가중합으로 안정화
    반환:
      corners(int32,4x2), (len_x, len_y), angle_deg, center(float2),
      handle_ctr(float2), tip_ctr(float2),
      (h_end_a, h_end_b, t_end_a, t_end_b), decide_conf
    """
    H, W = out_size_hw
    # 바이너리화 & 해상도 보정
    if mask.dtype != np.uint8:
        mask_bin = (mask > 0.5).astype(np.uint8)
    else:
        mask_bin = mask
    if mask_bin.shape[:2] != (H, W):
        mask_bin = cv2.resize(mask_bin, (W, H), interpolation=cv2.INTER_NEAREST)

    ys, xs = np.where(mask_bin > 0)
    if xs.size < min_pts:
        return None, None, None, None, None, None, (None, None, None, None), 0.0

    pts = np.stack([xs.astype(np.float32), ys.astype(np.float32)], axis=1)
    mean = pts.mean(axis=0)
    centered = pts - mean

    # PCA 축
    cov = np.cov(centered, rowvar=False)
    vals, vecs = np.linalg.eigh(cov)
    order = np.argsort(vals)[::-1]
    u_major = vecs[:, order[0]]
    v_minor = vecs[:, order[1]]
    u_major /= (np.linalg.norm(u_major) + 1e-9)
    v_minor /= (np.linalg.norm(v_minor) + 1e-9)

    # 주축으로 양 끝 밴드
    proj_u = centered @ u_major
    t_min, t_max = proj_u.min(), proj_u.max()
    L = (t_max - t_min)
    if L < 1e-6:
        return None, None, None, None, None, None, (None, None, None, None), 0.0
    band = end_band_ratio * L
    mask_head = proj_u <= (t_min + band)  # 앞쪽
    mask_tail = proj_u >= (t_max - band)  # 뒤쪽

    # 손잡이/날 결정(폭+반경+뾰족도)
    band_handle_mask, band_tip_mask, decide_conf = choose_handle_tip(
        mask_bin, mask_head, mask_tail, centered, pts, u_major, v_minor
    )

    # 각 밴드에서 v축 최상/최하 끝점 2개 → 중점(=중앙점)
    def band_endpoints_and_center(band_mask):
        if not np.any(band_mask):
            return None, None, None
        pts_b = pts[band_mask]      # in image coords
        ctr_b = centered[band_mask] # centered
        v_b   = ctr_b @ v_minor
        i_min = int(np.argmin(v_b))
        i_max = int(np.argmax(v_b))
        p_min = pts_b[i_min]        # 한쪽 끝
        p_max = pts_b[i_max]        # 반대 끝
        center_mid = (p_min + p_max) * 0.5
        return p_min, p_max, center_mid

    h_end_a, h_end_b, handle_ctr = band_endpoints_and_center(band_handle_mask)
    t_end_a, t_end_b, tip_ctr    = band_endpoints_and_center(band_tip_mask)

    if handle_ctr is None or tip_ctr is None:
        return None, None, None, None, None, None, (None, None, None, None), decide_conf

    # x축: 손잡이 중앙 → 날 중앙
    xdir = (tip_ctr - handle_ctr).astype(np.float32)
    n = float(np.linalg.norm(xdir))
    if n < 1e-6:
        # 비상: 주축 방향 사용 (손잡이/날 평균 위치로 부호 보정)
        xdir = u_major.copy()
        mu_h = float(np.mean(proj_u[band_handle_mask])) if np.any(band_handle_mask) else 0.0
        mu_t = float(np.mean(proj_u[band_tip_mask]))    if np.any(band_tip_mask) else 0.0
        if mu_t < mu_h:
            xdir = -xdir
    else:
        xdir /= n

    ydir = np.array([-xdir[1], xdir[0]], dtype=np.float32)
    ydir /= (np.linalg.norm(ydir) + 1e-9)

    # OBB
    R = np.stack([xdir, ydir], axis=1)
    proj_xy = (pts - mean) @ R
    mins = proj_xy.min(axis=0); maxs = proj_xy.max(axis=0)
    c_local = (mins + maxs) * 0.5
    center  = mean + (R @ c_local)
    a = (maxs[0] - mins[0]) * 0.5
    b = (maxs[1] - mins[1]) * 0.5
    c = center
    corners = np.stack([
        c + (+a)*xdir + (+b)*ydir,
        c + (+a)*xdir + (-b)*ydir,
        c + (-a)*xdir + (-b)*ydir,
        c + (-a)*xdir + (+b)*ydir
    ], axis=0).astype(np.int32)

    # 각도: 기준축 b=(0,-1) 기준, 시계방향 +
    b_up = np.array([0.0, -1.0], dtype=np.float32)
    dot = float(b_up @ xdir)
    det = float(b_up[0]*xdir[1] - b_up[1]*xdir[0])
    angle_rad_ccw = math.atan2(det, dot)
    angle_deg = -math.degrees(angle_rad_ccw)  # CW +
    if angle_deg > 180: angle_deg -= 360
    elif angle_deg < -180: angle_deg += 360

    len_x = 2*a; len_y = 2*b
    return corners, (len_x, len_y), angle_deg, center, handle_ctr, tip_ctr, (h_end_a, h_end_b, t_end_a, t_end_b), decide_conf

def draw_obb(overlay, corners, color=(0, 180, 255), thickness=2):
    cv2.polylines(overlay, [corners], isClosed=True, color=color, thickness=thickness)

def draw_handle_tip_viz(overlay, mask, handle_ctr, tip_ctr, end_points_tuple,
                        draw_endpoints=True,
                        color_center_handle=(0,200,255),
                        color_center_tip=(0,50,255),
                        color_arrow=(0,255,0),
                        color_end_handle=(255,200,0),
                        color_end_tip=(255,0,100)):
    """
    - 중앙점(손잡이/날) 2개 + 끝점 4개(손잡이 2, 날 2) 시각화 → 총 6점
    """
    if handle_ctr is None or tip_ctr is None:
        return overlay

    hc = tuple(np.int32(handle_ctr))
    tc = tuple(np.int32(tip_ctr))

    # 중앙점
    cv2.circle(overlay, hc, 5, color_center_handle, -1)
    cv2.circle(overlay, tc, 5, color_center_tip, -1)

    # 방향 화살표(중앙점 기준)
    cv2.arrowedLine(overlay, hc, tc, color_arrow, 2, tipLength=0.25)

    if draw_endpoints and end_points_tuple is not None:
        h_end_a, h_end_b, t_end_a, t_end_b = end_points_tuple
        if h_end_a is not None:
            h_end_a = tuple(np.int32(h_end_a))
            h_end_b = tuple(np.int32(h_end_b))
            cv2.circle(overlay, h_end_a, 6, color_end_handle, -1)
            cv2.circle(overlay, h_end_b, 6, color_end_handle, -1)
            cv2.putText(overlay, "HANDLE A", h_end_a, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_end_handle, 2, cv2.LINE_AA)
            cv2.putText(overlay, "HANDLE B", h_end_b, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_end_handle, 2, cv2.LINE_AA)
        if t_end_a is not None:
            t_end_a = tuple(np.int32(t_end_a))
            t_end_b = tuple(np.int32(t_end_b))
            cv2.circle(overlay, t_end_a, 6, color_end_tip, -1)
            cv2.circle(overlay, t_end_b, 6, color_end_tip, -1)
            cv2.putText(overlay, "TIP A", t_end_a, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_end_tip, 2, cv2.LINE_AA)
            cv2.putText(overlay, "TIP B", t_end_b, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_end_tip, 2, cv2.LINE_AA)

    return overlay

# =======================
# 메인
# =======================
def main():
    # ── 모델 로드
    model = YOLO(WEIGHTS)
    names = model.names
    print(f"[INFO] 모델 클래스({len(names)}): {names}")

    # ── 라벨 정규화 매칭
    target_norm = {norm_label(t) for t in TARGET_LABELS}
    names_norm = {i: norm_label(n) for i, n in names.items()}
    allowed_ids = [i for i, n in names_norm.items() if n in target_norm]
    print("[DEBUG] matched class ids =", allowed_ids, "=>", [names[i] for i in allowed_ids] if allowed_ids else [])
    if len(allowed_ids) == 0 or len(allowed_ids) < max(1, len(TARGET_LABELS)//2):
        print("[WARN] 라벨 매칭이 충분하지 않습니다. classes 필터를 비활성화합니다.")
        allowed_ids = None

    # LABEL_FILTER도 정규화 비교
    label_filter_norm = None
    if LABEL_FILTER:
        label_filter_norm = {norm_label(x) for x in LABEL_FILTER}

    # ── RealSense 파이프라인 구성
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, COLOR_FPS)
    config.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, DEPTH_FPS)
    profile = pipeline.start(config)

    # 깊이 관련
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    align = rs.align(rs.stream.color)
    spat_filter = rs.spatial_filter()
    temp_filter = rs.temporal_filter()
    hole_filling = rs.hole_filling_filter(1)

    # FPS/표시
    fps_time = time.time()
    frame_count = 0
    fps = None

    # 각도 평활화
    smoothed_angle = None

    show_window = SHOW_WINDOW
    if not show_window:
        os.makedirs(FALLBACK_SAVE_DIR, exist_ok=True)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # 깊이 필터 체인
            depth_frame = spat_filter.process(depth_frame)
            depth_frame = temp_filter.process(depth_frame)
            depth_frame = hole_filling.process(depth_frame)

            # 컬러 프레임
            color = np.asanyarray(color_frame.get_data())
            overlay = color.copy()

            # 입력 색공간 결정
            inp = cv2.cvtColor(color, cv2.COLOR_BGR2RGB) if USE_RGB_INPUT else color

            # 추론
            results = model(
                inp,
                conf=CONF_TH,
                iou=IOU_TH,
                device=DEVICE,
                classes=allowed_ids if allowed_ids else None,
                imgsz=IMG_SIZE,
                verbose=False
            )

            # 결과 해석
            if results and len(results) > 0:
                r = results[0]

                if getattr(r, "boxes", None) is not None and r.boxes is not None:
                    boxes = r.boxes.xyxy.cpu().numpy()
                    clses = r.boxes.cls.cpu().numpy().astype(int)
                    confs = r.boxes.conf.cpu().numpy()
                else:
                    boxes, clses, confs = np.zeros((0, 4)), np.zeros((0,), dtype=int), np.zeros((0,))

                if getattr(r, "masks", None) is not None and r.masks is not None:
                    masks_np = r.masks.data.cpu().numpy()
                else:
                    masks_np = None

                print(f"[DBG] boxes={len(boxes)} masks={'None' if masks_np is None else masks_np.shape}")

                N = len(boxes)
                for i in range(N):
                    c = int(clses[i])
                    if c in EXCLUDE_CLASS_IDS:
                        continue

                    x1, y1, x2, y2 = boxes[i]
                    x1i, y1i, x2i, y2i = map(int, [x1, y1, x2, y2])

                    # class name 안전 조회
                    if isinstance(names, dict):
                        cls_name = names.get(c, str(c))
                    else:
                        try:
                            cls_name = names[c]
                        except Exception:
                            cls_name = str(c)

                    conf = float(confs[i])

                    # LABEL_FILTER 적용 시 정규화 이름 비교
                    if label_filter_norm:
                        if norm_label(cls_name) not in label_filter_norm:
                            continue

                    # ── seg 마스크가 있을 때
                    if masks_np is not None and i < masks_np.shape[0]:
                        mask = masks_np[i]
                        overlay = apply_mask_overlay(overlay, mask, alpha=DRAW_MASK_ALPHA, color=(0, 255, 255))

                        dist_m = median_depth_meters_from_mask(depth_frame, mask, depth_scale)

                        # OBB + 손잡이/날 중앙점 & 끝점2
                        (corners, (w_len, h_len), angle_deg, center,
                         handle_ctr, tip_ctr, end_points_tuple, decide_conf) = obb_handle_tip_from_mask(
                            mask, (overlay.shape[0], overlay.shape[1])
                        )

                        if corners is not None:
                            # 프레임간 각도 평활화 (단일 대상 기준)
                            if smoothed_angle is None:
                                smoothed_angle = angle_deg
                            else:
                                diff = angle_deg - smoothed_angle
                                if diff > 180: diff -= 360
                                elif diff < -180: diff += 360
                                smoothed_angle += SMOOTHING_ALPHA * diff
                                if smoothed_angle > 180: smoothed_angle -= 360
                                elif smoothed_angle < -180: smoothed_angle += 360

                            # OBB
                            draw_obb(overlay, corners, color=(0, 180, 255), thickness=2)

                            # 손잡이/날 중앙점 + 끝부분(각 2개) 시각화
                            if DRAW_HANDLE_TIP:
                                overlay = draw_handle_tip_viz(
                                    overlay, mask, handle_ctr, tip_ctr, end_points_tuple,
                                    draw_endpoints=DRAW_ENDPOINTS
                                )

                            # 라벨(HUD)
                            label = f"{cls_name} {conf:.2f} | Roll: {smoothed_angle:.1f}°"
                            if dist_m is not None:
                                label += f" | Depth: {dist_m:.2f}m"
                            label += f" | H/T conf: {decide_conf:.2f}"
                            (tw, th), _ = cv2.getTextSize(label, FONT, 0.6, 2)
                            y_text = max(int(center[1]), th + 8)
                            x_text = int(center[0])
                            cv2.rectangle(overlay, (x_text, y_text - th - 6),
                                          (x_text + tw + 6, y_text), (0, 180, 255), -1)
                            cv2.putText(overlay, label, (x_text + 3, y_text - 4),
                                        FONT, 0.6, (0, 0, 0), 2, cv2.LINE_AA)
                        else:
                            # 마스크 기반 OBB 실패 시 bbox 표시
                            cv2.rectangle(overlay, (x1i, y1i), (x2i, y2i), (0, 255, 0), 2)
                            cv2.putText(overlay, f"{cls_name} {conf:.2f}",
                                        (x1i, max(0, y1i-5)), FONT, 0.6, (0,255,0), 2, cv2.LINE_AA)

                    else:
                        # ── det-only 또는 마스크 없음 → bbox 폴백
                        cv2.rectangle(overlay, (x1i, y1i), (x2i, y2i), (0, 255, 0), 2)
                        # 중앙 깊이 추정(폴백)
                        cx, cy = int((x1i + x2i)/2), int((y1i + y2i)/2)
                        dist_m = median_depth_meters_from_center(depth_frame, cx, cy, k=DEPTH_KERNEL, depth_scale=depth_scale)
                        label = f"{cls_name} {conf:.2f}"
                        if dist_m is not None:
                            label += f" | Depth: {dist_m:.2f}m"
                        cv2.putText(overlay, label, (x1i, max(0, y1i-5)), FONT, 0.6, (0,255,0), 2, cv2.LINE_AA)

            # FPS 계산
            now = time.time()
            frame_count += 1
            if (now - fps_time) > 1:
                fps = frame_count / (now - fps_time)
                frame_count = 0
                fps_time = now

            if fps is not None:
                txt = f"FPS: {fps:.1f}"
                cv2.putText(overlay, txt, (12, 28), FONT, 0.8, (50, 50, 255), 2, cv2.LINE_AA)

            # 표시/저장
            if show_window:
                try:
                    cv2.imshow("RealSense YOLO (2D Roll & Depth)", overlay)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                except Exception as e:
                    print(f"[WARN] imshow 실패, 헤드리스로 전환합니다: {e}")
                    show_window = False
                    os.makedirs(FALLBACK_SAVE_DIR, exist_ok=True)
            else:
                if (fps is None) or (int(time.time() * 10) % FALLBACK_SAVE_EVERY_N == 0):
                    fp = f"{FALLBACK_SAVE_DIR}/frame_{int(time.time()*1000)}.jpg"
                    cv2.imwrite(fp, overlay)

    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        if show_window:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    print("ROS2 humble is activated!")
    main()