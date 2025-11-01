# -*- coding: utf-8 -*-
"""
ROS2 Vision Node (통합판)
- 구독:  /info/string/obj_name (std_msgs/String)
- 발행:  /info/array/target_obj_array (std_msgs/Float32MultiArray)  # [x, y, z, roll_deg]
좌표계: RealSense color optical frame (x+우, y+하, z+전)
"""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
import re
import cv2
import time
import math
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from .utils import ARUCO_DICT, aruco_display

WEIGHTS = "/home/pc/soomac_ws/src/zeus_vision/tool_best.pt"
DEVICE = "0"
CONF_TH = 0.28
IOU_TH  = 0.45
IMG_SIZE = 960
USE_RGB_INPUT = False

# 표시/후처리
DRAW_MASK_ALPHA = 0.4
COLOR_W, COLOR_H, COLOR_FPS = 640, 480, 30
DEPTH_W, DEPTH_H, DEPTH_FPS = 640, 480, 30
TARGET_LABELS = {"nipper", "vernier_calipers", "wire_cutter", "wire_stripper"}
SHOW_WINDOW = True

SHOW_BAND_PREVIEW = True
DEPTH_KERNEL = 5
SMOOTHING_ALPHA = 0.2
DRAW_HANDLE_TIP = True
DRAW_ENDPOINTS  = True

WIDTH_W   = 0.55
RADIUS_W  = 0.35
POINTY_W  = 0.10
WIDTH_MARGIN_RATIO = 0.15

FONT = cv2.FONT_HERSHEY_SIMPLEX

# ---------- 유틸 ----------
def norm_label(s: str) -> str:
    return re.sub(r"[\s\-_]+", "", s.lower())

def draw_label_box(img, x, y, text, font=FONT, font_scale=0.4, thickness=1,
                   bg=(0,180,255), fg=(0,0,0)):
    (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)
    H, W = img.shape[:2]
    x = int(max(0, min(x, W - tw - 8)))
    y = int(max(th + 6, min(y, H - 6)))
    cv2.rectangle(img, (x, y - th - 6), (x + tw + 6, y), bg, -1)
    cv2.putText(img, text, (x + 3, y - 4), font, font_scale, fg, thickness, cv2.LINE_AA)

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
    if patch.size == 0:
        return None
    valid = patch[patch > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid)) * depth_scale

def median_depth_meters_from_mask(depth_frame, mask, depth_scale=0.001):
    depth_image = np.asanyarray(depth_frame.get_data())
    H_d, W_d = depth_image.shape[:2]
    if mask.shape[0] != H_d or mask.shape[1] != W_d:
        mask = cv2.resize(mask.astype(np.uint8), (W_d, H_d), interpolation=cv2.INTER_NEAREST)
    m = mask.astype(bool)
    if not np.any(m):
        return None
    valid = depth_image[m]
    valid = valid[valid > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid)) * depth_scale

def mask_area_m2_from_depth(depth_frame, mask, depth_scale, fx, fy):
    """per-pixel area ≈ Z^2/(fx*fy) 합산."""
    if depth_frame is None:
        return None
    depth = np.asanyarray(depth_frame.get_data())
    H, W = depth.shape[:2]
    if mask.dtype != np.uint8:
        mask = (mask > 0.5).astype(np.uint8)
    if mask.shape[0] != H or mask.shape[1] != W:
        mask = cv2.resize(mask, (W, H), interpolation=cv2.INTER_NEAREST)
    m = mask.astype(bool)
    if not np.any(m):
        return None
    z = depth[m].astype(np.float32) * depth_scale
    z = z[z > 0]
    if z.size == 0:
        return None
    per_pix_area = (z * z) / (float(fx) * float(fy))
    return float(per_pix_area.sum())

# ---------- 밴드/OBB ----------
def _band_metrics_with_dt(mask_bin, band_mask, centered, pts, u_major, v_minor):
    if not np.any(band_mask):
        return dict(valid=False)
    pts_b = pts[band_mask]
    ctr_b = centered[band_mask]
    v_b   = ctr_b @ v_minor
    width_v = float(v_b.max() - v_b.min())
    area    = float(pts_b.shape[0])
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
                      width_w=WIDTH_W, radius_w=RADIUS_W, pointy_w=POINTY_W,
                      width_margin_ratio=WIDTH_MARGIN_RATIO,
                      force_width_only=False):
    mh = _band_metrics_with_dt(mask_bin, mask_head, centered, pts, u_major, v_minor)
    mt = _band_metrics_with_dt(mask_bin, mask_tail, centered, pts, u_major, v_minor)
    if not mh["valid"] or not mt["valid"]:
        if mh.get("width_v",0) >= mt.get("width_v",0): return mask_head, mask_tail, 0.5
        else: return mask_tail, mask_head, 0.5
    wh, wt = mh["width_v"], mt["width_v"]
    big = max(wh, wt) + 1e-6
    width_gap_ratio = abs(wh - wt) / big
    if force_width_only:
        if wh >= wt: return mask_tail, mask_head, min(1.0, 0.6 + 0.4*width_gap_ratio)
        else:        return mask_head, mask_tail, min(1.0, 0.6 + 0.4*width_gap_ratio)
    if wh > wt * (1.0 + width_margin_ratio):
        return mask_head, mask_tail, min(1.0, 0.6 + 0.4*width_gap_ratio)
    if wt > wh * (1.0 + width_margin_ratio):
        return mask_tail, mask_head, min(1.0, 0.6 + 0.4*width_gap_ratio)
    width_score  = (wh - wt) / big
    rh, rt       = mh["radius_med"], mt["radius_med"]
    radius_score = (rh - rt) / (max(rh, rt) + 1e-6)
    ph, pt       = mh["pointiness"], mt["pointiness"]
    pointy_score = (pt - ph)
    combined = width_w*width_score + radius_w*radius_score + pointy_w*pointy_score
    conf = min(1.0, 0.5 + 0.5*abs(combined))
    return (mask_head, mask_tail, conf) if combined >= 0 else (mask_tail, mask_head, conf)

def obb_handle_tip_from_mask(mask, out_size_hw, end_band_ratio=0.18, min_pts=30, force_width_only=False):
    H, W = out_size_hw
    mask_bin = (mask > 0.5).astype(np.uint8) if mask.dtype != np.uint8 else mask
    if mask_bin.shape[:2] != (H, W):
        mask_bin = cv2.resize(mask_bin, (W, H), interpolation=cv2.INTER_NEAREST)
    contours, _ = cv2.findContours(mask_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return None, None, None, None, None, None, (None, None, None, None), 0.0, None
    main_contour = max(contours, key=cv2.contourArea)
    if main_contour.shape[0] < min_pts:
        return None, None, None, None, None, None, (None, None, None, None), 0.0, None
    hull = cv2.convexHull(main_contour)
    rect = cv2.minAreaRect(hull)
    box = cv2.boxPoints(rect)
    edge1 = box[1] - box[0]; edge2 = box[2] - box[1]
    u_major = edge1 if np.linalg.norm(edge1) > np.linalg.norm(edge2) else edge2
    u_major = u_major / (np.linalg.norm(u_major) + 1e-9)
    v_minor = np.array([-u_major[1], u_major[0]], dtype=np.float32)
    ys, xs = np.where(mask_bin > 0)
    pts = np.stack([xs.astype(np.float32), ys.astype(np.float32)], axis=1)
    mean = pts.mean(axis=0); centered = pts - mean
    proj_u = centered @ u_major
    u_min, u_max = proj_u.min(), proj_u.max()
    L = u_max - u_min
    if L < 1e-6:
        return None, None, None, None, None, None, (None, None, None, None), 0.0, None
    proj_v = centered @ v_minor
    ar = L / (proj_v.max() - proj_v.min() + 1e-6)
    band = (end_band_ratio if ar >= 1.25 else max(end_band_ratio, 0.25)) * L
    mask_head = proj_u <= (u_min + band)
    mask_tail = proj_u >= (u_max - band)

    # [CHANGED-1] 픽셀 y-평균 규칙으로 head/tail 1차 교정
    if np.any(mask_head) and np.any(mask_tail):
        head_y_mean = float(pts[mask_head][:, 1].mean())
        tail_y_mean = float(pts[mask_tail][:, 1].mean())
        if head_y_mean < tail_y_mean:
            mask_head, mask_tail = mask_tail, mask_head
    # [CHANGED-1 END]

    band_handle_mask, band_tip_mask, decide_conf = choose_handle_tip(
        mask_bin, mask_head, mask_tail, centered, pts, u_major, v_minor, force_width_only=force_width_only
    )

    def _endpoints_and_center(band_mask):
        if not np.any(band_mask): return None, None, None
        pts_b = pts[band_mask]
        ctr_b = centered[band_mask]
        v_b = ctr_b @ v_minor
        p_min = pts_b[int(np.argmin(v_b))]
        p_max = pts_b[int(np.argmax(v_b))]
        center_mid = (p_min + p_max) * 0.5
        return p_min, p_max, center_mid

    h_end_a, h_end_b, handle_ctr = _endpoints_and_center(band_handle_mask)
    t_end_a, t_end_b, tip_ctr    = _endpoints_and_center(band_tip_mask)

    # [CHANGED-2] 최종 handle/tip 픽셀 규칙 강제: handle가 tip보다 '아래'(y가 큼)여야 함
    if handle_ctr is not None and tip_ctr is not None:
        # 수평에 가까워 Δy가 작으면 규칙을 약하게 하고 싶다면 임계값(예: 3~5px) 추가 가능
        if handle_ctr[1] < tip_ctr[1]:
            band_handle_mask, band_tip_mask = band_tip_mask, band_handle_mask
            handle_ctr, tip_ctr = tip_ctr, handle_ctr
            h_end_a, h_end_b, t_end_a, t_end_b = t_end_a, t_end_b, h_end_a, h_end_b
    # [CHANGED-2 END]

    def _create_band_img(band_mask):
        img = np.zeros((H, W), dtype=np.uint8)
        if np.any(band_mask):
            pts_b = np.round(pts[band_mask]).astype(np.int32)
            pts_b[:,0] = np.clip(pts_b[:,0], 0, W-1)
            pts_b[:,1] = np.clip(pts_b[:,1], 0, H-1)
            img[pts_b[:,1], pts_b[:,0]] = 1
        return img

    debug = dict(
        band_head_img=_create_band_img(mask_head),
        band_tail_img=_create_band_img(mask_tail),
        u_major=u_major.copy(), v_minor=v_minor.copy(), mean=mean.copy(),
        t_min=u_min, t_max=u_max,
        head_ctr=(pts[mask_head].mean(axis=0) if np.any(mask_head) else None),
        tail_ctr=(pts[mask_tail].mean(axis=0) if np.any(mask_tail) else None),
        chosen_axis="major_from_hull", decide_conf=decide_conf,
        sep=np.linalg.norm(tip_ctr - handle_ctr) if handle_ctr is not None and tip_ctr is not None else 0.0,
        ar=ar
    )
    if handle_ctr is None or tip_ctr is None:
        return None, None, None, None, None, None, (None, None, None, None), decide_conf, debug

    xdir = (tip_ctr - handle_ctr).astype(np.float32)
    n = np.linalg.norm(xdir)
    xdir = (xdir / n) if n >= 1e-6 else u_major.copy()
    ydir = np.array([-xdir[1], xdir[0]], dtype=np.float32)
    R = np.stack([xdir, ydir], axis=1)
    proj_xy = (pts - mean) @ R
    mins = proj_xy.min(axis=0); maxs = proj_xy.max(axis=0)
    c_local = (mins + maxs) * 0.5
    center = mean + (R @ c_local)
    a = (maxs[0] - mins[0]) * 0.5; b = (maxs[1] - mins[1]) * 0.5
    corners = np.stack([
        center + a*xdir + b*ydir,
        center + a*xdir - b*ydir,
        center - a*xdir - b*ydir,
        center - a*xdir + b*ydir
    ], axis=0).astype(np.int32)
    b_up = np.array([0.0, -1.0], dtype=np.float32)
    dot = b_up @ xdir
    det = b_up[0]*xdir[1] - b_up[1]*xdir[0]
    angle_rad_ccw = math.atan2(det, dot)
    angle_deg = -math.degrees(angle_rad_ccw)
    if angle_deg > 180: angle_deg -= 360
    elif angle_deg < -180: angle_deg += 360
    len_x = float(maxs[0] - mins[0]); len_y = float(maxs[1] - mins[1])
    return corners, (len_x, len_y), angle_deg, center, handle_ctr, tip_ctr, \
           (h_end_a, h_end_b, t_end_a, t_end_b), decide_conf, debug

def draw_obb(overlay, corners, color=(0, 180, 255), thickness=2):
    cv2.polylines(overlay, [corners], isClosed=True, color=color, thickness=thickness)

def draw_handle_tip_viz(overlay, handle_ctr, tip_ctr, end_points_tuple,
                        draw_endpoints=True,
                        color_center_handle=(0,200,255),
                        color_center_tip=(0,50,255),
                        color_arrow=(0,255,0),
                        color_end_handle=(255,200,0),
                        color_end_tip=(255,0,100)):
    if handle_ctr is None or tip_ctr is None:
        return overlay
    hc = tuple(np.int32(handle_ctr)); tc = tuple(np.int32(tip_ctr))
    cv2.circle(overlay, hc, 5, color_center_handle, -1)
    cv2.circle(overlay, tc, 5, color_center_tip, -1)
    cv2.arrowedLine(overlay, hc, tc, color_arrow, 2, tipLength=0.25)
    if draw_endpoints and end_points_tuple is not None:
        h_end_a, h_end_b, t_end_a, t_end_b = end_points_tuple
        for p in [h_end_a, h_end_b]:
            if p is not None:
                cv2.circle(overlay, tuple(np.int32(p)), 6, color_end_handle, -1)
        for p in [t_end_a, t_end_b]:
            if p is not None:
                cv2.circle(overlay, tuple(np.int32(p)), 6, color_end_tip, -1)
    return overlay

# ---------- ROS2 노드 ----------
class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.detection_pub = self.create_publisher(Float32MultiArray, '/zeus/array/tool_pos', 10)
        self.target_tool_sub = self.create_subscription(String, '/zeus/string/tool_info', self.target_tool_callback, 10)

        self.target_tool = None
        self.detection_active = False
        self.frame_id = 'camera_color_optical_frame'
        self.timer = self.create_timer(0.1, self.vision_callback)
        self.setup_vision_system()
        self.get_logger().info('Vision Node 시작됨. 제어단에서 도구 요청 대기...')
        self.smoothed_angle = None

        self.aruco_type = "DICT_6X6_1000"
        self.marker_length = 0.029
        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[self.aruco_type])
        aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        self.last_area_m2 = None  # 상단 고정 표기를 위한 캐시

    def setup_vision_system(self):
        self.model = YOLO(WEIGHTS)
        self.names = self.model.names
        self.get_logger().info(f"모델 클래스({len(self.names)}): {self.names}")
        target_norm = {norm_label(t) for t in TARGET_LABELS}
        names_norm = {i: norm_label(n) for i, n in self.names.items()}
        self.allowed_ids = [i for i, n in names_norm.items() if n in target_norm]
        self.get_logger().info(f"매칭된 클래스 ID: {self.allowed_ids}")

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, COLOR_FPS)
        config.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, DEPTH_FPS)
        profile = self.pipeline.start(config)
        color_sensor = profile.get_device().first_color_sensor()

        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        self.align = rs.align(rs.stream.color)
        self.spat_filter = rs.spatial_filter()
        self.temp_filter = rs.temporal_filter()
        self.hole_filling = rs.hole_filling_filter(1)

        color_stream = profile.get_stream(rs.stream.color)
        self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

        # 캘리브레이션한 K, D 값을 사용
        self.K = np.array([
            [609.59370966, 0.0,          327.77961006],
            [ 0.0,       610.16182704, 244.8987311],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        
        self.D = np.array([
            [ 2.98079773e-02,  7.71843130e-01,  1.12771351e-03,  1.91769037e-03, -2.86200282e+00]
        ], dtype=np.float32)

        self.size_tolerance = 0.2
        self.get_logger().info("RealSense 카메라 초기화 완료")

    def pixel_to_3d_point(self, pixel_x, pixel_y, depth_m):
        if depth_m is None or depth_m <= 0:
            return None
        # 캘리브레이션된 K 행렬의 값을 직접 사용
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        ppx = self.K[0, 2]
        ppy = self.K[1, 2]
        
        x = (pixel_x - ppx) * depth_m / fx
        y = (pixel_y - ppy) * depth_m / fy
        z = depth_m
        return [float(x), float(y), float(z)]

    def vision_callback(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
            aligned = self.align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not color_frame:
                return
            
            color = np.asanyarray(color_frame.get_data())
            
            # [수정 1] 왜곡 보정을 먼저 수행
            undistort_img = cv2.undistort(color, self.K, self.D)
            
            # [수정 2] 오버레이(시각화) 기반 이미지를 왜곡 보정된 이미지로 변경
            overlay = undistort_img.copy() 

            # [수정 3] YOLO 입력 이미지를 왜곡 보정된 이미지로 변경
            if USE_RGB_INPUT:
                inp_model = cv2.cvtColor(undistort_img, cv2.COLOR_BGR2RGB)
            else:
                inp_model = undistort_img.copy() # 원본 수정을 피하기 위해 .copy()

            if self.target_tool == "M3" and self.detection_active:
                corners, ids, rejected = self.detector.detectMarkers(undistort_img)
                if ids is not None:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.marker_length, self.K, self.D
                    )
                    for i, mid in enumerate(ids.flatten()):
                        #ID 변경 할 부분
                        if mid != int(1):
                            continue
                        pts = corners[i].reshape(4,2)
                        edge1 = np.linalg.norm(pts[0]-pts[1])
                        edge2 = np.linalg.norm(pts[1]-pts[2])
                        avg_pix = (edge1+edge2)/2.0
                        Z = float(tvecs[i][0][2])
                        if Z <= 0: 
                            continue
                        fx = float(self.K[0,0])
                        expected_pix = fx * self.marker_length / Z
                        if abs(avg_pix-expected_pix)/max(expected_pix,1e-6) > self.size_tolerance:
                            continue
                        R,_ = cv2.Rodrigues(rvecs[i][0])
                        x, y, z = map(float, tvecs[i][0])
                        yaw_rad = math.atan2(R[1,0], R[0,0])
                        yaw_deg = math.degrees(yaw_rad)
                        if 0<= yaw_deg < 180: yaw_deg -= 90
                        elif -180 < yaw_deg < 0: yaw_deg += 90
                        cv2.drawFrameAxes(undistort_img, self.K, self.D, rvecs[i][0], tvecs[i][0], 0.03)
                        out = Float32MultiArray()
                        out.data = [x * 1000, y * 1000, z * 1000, float(yaw_deg)]
                        self.detection_pub.publish(out)
                        self.detection_active = False
                        current_corners = [corners[i]]
                        current_ids = ids[i:i+1]
                        disp = aruco_display(current_corners, current_ids, rejected, undistort_img)
                        overlay = cv2.addWeighted(overlay, 0.5, disp, 0.5, 0.0)
                        break

            else:
                # [수정 4] YOLO 모델 입력을 inp_model로 변경
                results = self.model(
                    inp_model, conf=CONF_TH, iou=IOU_TH, device=DEVICE,
                    classes=self.allowed_ids if self.allowed_ids else None,
                    imgsz=IMG_SIZE, verbose=False,
                ) if self.detection_active else []
                found_payload = None
                area_m2 = None

                if results and len(results) > 0:
                    r = results[0]
                    boxes = r.boxes.xyxy.cpu().numpy() if getattr(r, "boxes", None) and r.boxes is not None else np.zeros((0,4))
                    clses = r.boxes.cls.cpu().numpy().astype(int) if getattr(r, "boxes", None) and r.boxes is not None else np.zeros((0,), dtype=int)
                    confs = r.boxes.conf.cpu().numpy() if getattr(r, "boxes", None) and r.boxes is not None else np.zeros((0,))
                    masks_np = r.masks.data.cpu().numpy() if getattr(r, "masks", None) and r.masks is not None else None

                    for i in range(len(boxes)):
                        c = int(clses[i])
                        x1, y1, x2, y2 = boxes[i]
                        x1i, y1i, x2i, y2i = map(int, [x1, y1, x2, y2])
                        if isinstance(self.names, dict):
                            cls_name = self.names.get(c, str(c))
                        else:
                            try: cls_name = self.names[c]
                            except Exception: cls_name = str(c)
                        if self.target_tool and norm_label(cls_name) != norm_label(self.target_tool):
                            continue
                        conf = float(confs[i])
                        depth_m = None # [Z값 로직 복원] depth_m 변수 초기화
                        roll_deg = 0.0
                        decide_conf = 0.0
                        handle_ctr = None
                        tip_ctr = None

                        if masks_np is not None and i < masks_np.shape[0]:
                            mask = masks_np[i]
                            overlay = apply_mask_overlay(overlay, mask, alpha=DRAW_MASK_ALPHA, color=(0,255,255))

                            if depth_frame is not None:
                                area_m2 = mask_area_m2_from_depth(depth_frame, mask, self.depth_scale, self.K[0,0], self.K[1,1])
                                self.last_area_m2 = area_m2

                            # [Z값 로직 복원] 마스크 전체의 중앙 깊이를 계산
                            depth_m = median_depth_meters_from_mask(depth_frame, mask, self.depth_scale) if depth_frame is not None else None
                            
                            is_vc = norm_label(cls_name) == norm_label("vernier_calipers")
                            (corners, (w_len, h_len), angle_deg, center_px_obb,
                             handle_ctr, tip_ctr, end_points_tuple, decide_conf, debug) = obb_handle_tip_from_mask(
                                mask, (overlay.shape[0], overlay.shape[1]), force_width_only=is_vc
                            )

                            if SHOW_BAND_PREVIEW and debug is not None:
                                # ... (디버그 시각화) ...
                                head_img = debug["band_head_img"]; tail_img = debug["band_tail_img"]
                                overlay = apply_mask_overlay(overlay, head_img, alpha=0.35, color=(255,0,255))
                                overlay = apply_mask_overlay(overlay, tail_img, alpha=0.35, color=(0,128,255))
                                if debug.get("head_ctr") is not None:
                                    hx, hy = int(debug["head_ctr"][0]), int(debug["head_ctr"][1])
                                    cv2.putText(overlay, "HEAD_CAND", (hx-20, max(hy-8, 15)), FONT, 0.5, (255,0,255), 2, cv2.LINE_AA)
                                if debug.get("tail_ctr") is not None:
                                    tx, ty = int(debug["tail_ctr"][0]), int(debug["tail_ctr"][1])
                                    cv2.putText(overlay, "TAIL_CAND", (tx-20, max(ty-8, 15)), FONT, 0.5, (0,128,255), 2, cv2.LINE_AA)
                                u_major = debug["u_major"]; mean = debug["mean"]
                                tmin = debug["t_min"]; tmax = debug["t_max"]
                                axis_len = 0.5 * (tmax - tmin + 1e-6)
                                p1 = (mean + u_major * (-axis_len)).astype(np.int32)
                                p2 = (mean + u_major * (+axis_len)).astype(np.int32)
                                cv2.line(overlay, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (255,255,0), 2)
                                cv2.circle(overlay, (int(mean[0]), int(mean[1])), 3, (255,255,0), -1)

                            if corners is not None:
                                if self.smoothed_angle is None:
                                    self.smoothed_angle = angle_deg
                                else:
                                    diff = angle_deg - self.smoothed_angle
                                    if diff > 180: diff -= 360
                                    elif diff < -180: diff += 360
                                    self.smoothed_angle += SMOOTHING_ALPHA * diff
                                    if self.smoothed_angle > 180: self.smoothed_angle -= 360
                                    elif self.smoothed_angle < -180: self.smoothed_angle += 360
                                roll_deg = float(self.smoothed_angle)
                                draw_obb(overlay, corners, color=(0,180,255), thickness=2)
                                if DRAW_HANDLE_TIP:
                                    overlay = draw_handle_tip_viz(overlay, handle_ctr, tip_ctr, end_points_tuple, draw_endpoints=DRAW_ENDPOINTS)
                            else:
                                cv2.rectangle(overlay, (x1i, y1i), (x2i, y2i), (0,255,0), 2)
                                # [Z값 로직 복원] OBB 실패 시 BBox 중심 깊이 계산
                                cx, cy = int((x1i+x2i)/2), int((y1i+y2i)/2)
                                depth_m = median_depth_meters_from_center(depth_frame, cx, cy, k=DEPTH_KERNEL, depth_scale=self.depth_scale) if depth_frame is not None else None

                        else:
                            cv2.rectangle(overlay, (x1i, y1i), (x2i, y2i), (0,255,0), 2)
                            # [Z값 로직 복원] 마스크 없을 시 BBox 중심 깊이 계산
                            cx, cy = int((x1i+x2i)/2), int((y1i+y2i)/2)
                            depth_m = median_depth_meters_from_center(depth_frame, cx, cy, k=DEPTH_KERNEL, depth_scale=self.depth_scale) if depth_frame is not None else None

                        # 퍼블리시 픽셀 (X, Y) 확정
                        if masks_np is not None and i < (masks_np.shape[0] if masks_np is not None else 0) and handle_ctr is not None and tip_ctr is not None:
                            if norm_label(cls_name) == norm_label("wire_cutter"):
                                center_px = (handle_ctr[0]*0.37 + tip_ctr[0]*0.63, handle_ctr[1]*0.37 + tip_ctr[1]*0.63)
                            elif norm_label(cls_name) == norm_label("nipper"):
                                center_px = (handle_ctr[0]*0.32 + tip_ctr[0]*0.68, handle_ctr[1]*0.32 + tip_ctr[1]*0.68)
                            else:
                                center_px = ((handle_ctr[0]*0.515+tip_ctr[0]*0.485), (handle_ctr[1]*0.515+tip_ctr[1]*0.485))
                        else:
                            center_px = ((x1 + x2) * 0.5, (y1 + y2) * 0.5)

                        cx_i, cy_i = int(center_px[0]), int(center_px[1])
                        cv2.drawMarker(overlay, (cx_i, cy_i), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=14, thickness=2)
                        cv2.putText(overlay, "PUB", (cx_i + 10, max(15, cy_i - 10)), FONT, 0.5, (0,255,0), 2, cv2.LINE_AA)

                        # [Z값 로직 복원] 3D 변환:
                        # (X, Y)는 center_px를 사용하지만,
                        # (Z)는 위에서 계산한 depth_m (마스크 전체 또는 BBox 중심)을 사용
                        center_3d = None
                        if depth_m is not None:
                            center_3d = self.pixel_to_3d_point(center_px[0], center_px[1], 0.39)

                        label = f"{cls_name} {conf:.2f}"
                        if depth_m is not None: label += f" | Depth: {depth_m:.2f}m"
                        if self.smoothed_angle is not None: label += f" | Roll: {self.smoothed_angle:.1f}°"
                        if decide_conf: label += f" | H/T: {decide_conf:.2f}"
                        if area_m2 is not None: label += f" | Area: {area_m2:.4f} m^2"

                        draw_label_box(overlay, cx_i + 16, cy_i - 16, label)

                        if center_3d is not None:
                            payload = {"class_name": cls_name, "confidence": conf, "position": center_3d, "roll_deg": float(roll_deg)}
                            found_payload = payload
                            break

                if found_payload is not None:
                    self.publish_detections(found_payload)
                    self.detection_active = False
                    self.target_tool = None

            if self.last_area_m2 is not None:
                draw_label_box(overlay, 10, 30, f"Area: {self.last_area_m2:.4f} m^2")

            if SHOW_WINDOW:
                status_text = f"Detection: {'ACTIVE' if self.detection_active else 'INACTIVE'}"
                target_text = f"Target: {self.target_tool if self.target_tool else 'None'}"
                draw_label_box(overlay, 10, 60, status_text, bg=(40, 40, 40), fg=(255,255,255))
                draw_label_box(overlay, 10, 90, target_text, bg=(40, 40, 40), fg=(255,255,255))
                try:
                    # [수정] 최종 시각화는 왜곡 보정된 overlay 이미지를 사용
                    cv2.imshow("Tool Detection (seg+obb)", overlay)
                    cv2.waitKey(1)
                except Exception as e:
                    self.get_logger().warn(f"imshow 실패: {e}")

        except Exception as e:
            self.get_logger().error(f"Vision callback 오류: {e}")

    def publish_detections(self, det: dict):
        msg = Float32MultiArray()
        x, y, z = det["position"]
        roll = float(det.get("roll_deg", 0.0))
        msg.data = [float(x)*1000, float(y)*1000, float(z)*1000, roll]
        self.detection_pub.publish(msg)
        self.get_logger().info(f"탐지 전송: {det['class_name']} | pos=({x:.3f},{y:.3f},{z:.3f}) m | roll={roll:.1f}° | conf={det['confidence']:.2f}")

    def target_tool_callback(self, msg: String):
        tool_name = msg.data.strip()
        if tool_name and tool_name.lower() not in {"stop", "none"}:
            self.target_tool = tool_name
            self.detection_active = True
            self.get_logger().info(f"도구 탐지 요청 수신: {tool_name}")
            self.smoothed_angle = None
        else:
            self.target_tool = None
            self.detection_active = False
            self.get_logger().info("도구 탐지 중단")

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("중단됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    print("ROS2 humble is activated!")
    main()