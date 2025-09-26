# -*- coding: utf-8 -*-

import os
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import warnings; warnings.filterwarnings("ignore")
import math

# ===== 시각화/클러스터링 스위치 =====
USE_MPL = False          # Matplotlib 실시간 3D 시각화 비활성화
USE_DBSCAN_3D = False    # Open3D DBSCAN 비활성화

try:
    import open3d as o3d
    O3D_OK = True
except ImportError:
    O3D_OK = False

try:
    from scipy.spatial.transform import Rotation as SciRot
    SCIPY_OK = True
except ImportError:
    SCIPY_OK = False

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String

# =========================
# 설정
# =========================
WEIGHTS    = "/home/pc/Downloads/best_jjin_mak.pt"
DEVICE     = "0"

CONF_DET   = 0.28
CONF_PUB   = 0.50
IOU_TH     = 0.45
IMG_SIZE   = 640

COLOR_W, COLOR_H, COLOR_FPS = 640, 480, 30
DEPTH_W, DEPTH_H, DEPTH_FPS = 640, 480, 30
FONT = cv2.FONT_HERSHEY_SIMPLEX

# 포인트클라우드/평면 분할
PCL_STRIDE = 3
DEPTH_P_LOW, DEPTH_P_HIGH = 10, 90
PLANE_DIST_BASE, PLANE_RANSAC_N, PLANE_ITERS = 0.01, 3, 1000
STAT_NB_NEIGHBORS, STAT_STD_RATIO = 20, 2.0
RAD_RADIUS, RAD_MIN_POINTS = 0.03, 10

# 제약/시각화
DEAD_ZONE_DEG = 3.0
MIN_AREA_PX   = 800
INLIER_RATIO_TH = 0.30
DBSCAN_EPS_M  = 0.015
DBSCAN_MINPTS = 60

STAB_ANGLE_DEG = 1.0
STAB_MIN_FRAMES = 20
STAB_CENTER_JUMP_PX = 60
PUBLISH_COOLDOWN_FRAMES = 10

# 블록 실제 크기 [m]
BLOCK_LEN_M = 0.075
BLOCK_WID_M = 0.025

# 선택 규칙 파라미터
SAME_LAYER_EPS_M   = 0.003      # 같은 층(z) 판정 오차
WIDTH_THRESHOLD_MM = 24.0       # 폭(mm) 기준: 작으면 length면으로 간주

# =========================
# 유틸
# =========================
def apply_mask_overlay(bgr, mask, color=(0,255,255), alpha=0.4):
    H, W = bgr.shape[:2]
    out = bgr.copy()
    if mask.dtype != np.uint8:
        mask = (mask > 0.5).astype(np.uint8)
    if mask.shape[:2] != (H, W):
        mask = cv2.resize(mask, (W, H), interpolation=cv2.INTER_NEAREST)
    color_overlay = np.zeros_like(out); color_overlay[:] = color
    m = mask.astype(bool)
    out[m] = cv2.addWeighted(out, 1 - alpha, color_overlay, alpha, 0)[m]
    return out

def connected_components_keep(mask_bin, min_area_px=MIN_AREA_PX, keep_largest=True, center_bias=None):
    m = (mask_bin > 0).astype(np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel, iterations=1)
    num, labels, stats, cents = cv2.connectedComponentsWithStats(m, connectivity=8)
    if num <= 1:
        return m
    comps = []
    for i in range(1, num):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < min_area_px:
            continue
        score = area
        if center_bias is not None:
            cx, cy = center_bias
            cx_i, cy_i = cents[i]
            d2 = (cx_i - cx)**2 + (cy_i - cy)**2
            score = area / (1.0 + 0.001 * d2)
        comps.append((score, i))
    if not comps:
        return np.zeros_like(m, dtype=np.uint8)
    comps.sort(reverse=True)
    keep_ids = [comps[0][1]] if keep_largest else [i for _, i in comps]
    out = np.zeros_like(m, dtype=np.uint8)
    for i in keep_ids:
        out[labels == i] = 1
    out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, kernel, iterations=1)
    return out

def keep_largest_component(mask_bin):
    m = (mask_bin > 0).astype(np.uint8)
    num, labels, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    if num <= 1:
        return m
    areas = stats[1:, cv2.CC_STAT_AREA]
    i = np.argmax(areas) + 1
    return (labels == i).astype(np.uint8)

def get_points_from_mask(mask_bin, depth_frame, intr, depth_scale):
    depth = np.asanyarray(depth_frame.get_data())
    H, W = depth.shape[:2]
    if mask_bin.shape[:2] != (H, W):
        mask_bin = cv2.resize(mask_bin, (W, H), interpolation=cv2.INTER_NEAREST)
    ys, xs = np.where(mask_bin > 0)
    if ys.size == 0:
        return None, None, None
    xs_ds, ys_ds = xs[::PCL_STRIDE], ys[::PCL_STRIDE]
    zs = depth[ys_ds, xs_ds].astype(np.float32) * depth_scale
    valid = (zs > 1e-6)
    if not np.any(valid):
        return None, None, None
    xs_ds, ys_ds, zs = xs_ds[valid], ys_ds[valid], zs[valid]
    X = (xs_ds - intr.ppx) * zs / intr.fx
    Y = (ys_ds - intr.ppy) * zs / intr.fy
    pts = np.stack([X, Y, zs], axis=1)
    if pts.shape[0] == 0:
        return None, None, None
    z = pts[:, 2]
    lo, hi = np.percentile(z, [DEPTH_P_LOW, DEPTH_P_HIGH])
    v2 = (z >= lo) & (z <= hi)
    return pts[v2], xs_ds[v2], ys_ds[v2]

# =========================
# [VIS] 투영/마커
# =========================
def project_point_to_pixel(P, intr):
    X, Y, Z = float(P[0]), float(P[1]), float(P[2])
    if Z <= 1e-9 or not np.isfinite(Z):
        return None
    u = intr.fx * (X / Z) + intr.ppx
    v = intr.fy * (Y / Z) + intr.ppy
    return (int(round(u)), int(round(v)))

def draw_cross(img, pt, color=(0,0,255), size=6, thickness=2):
    if pt is None: return
    u, v = pt
    h, w = img.shape[:2]
    if not (0 <= u < w and 0 <= v < h): return
    cv2.line(img, (u-size, v), (u+size, v), color, thickness, cv2.LINE_AA)
    cv2.line(img, (u, v-size), (u, v+size), color, thickness, cv2.LINE_AA)

# =========================
# 평면/좌표계 유틸
# =========================
def segment_plane_and_axes(pts):
    if not O3D_OK or pts is None or pts.shape[0] < 50:
        return [None] * 6
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=STAT_NB_NEIGHBORS, std_ratio=STAT_STD_RATIO)
    pcd, _ = pcd.remove_radius_outlier(nb_points=RAD_MIN_POINTS, radius=RAD_RADIUS)
    if len(pcd.points) < 50:
        return [None] * 6

    plane_model, inliers_idx = pcd.segment_plane(distance_threshold=PLANE_DIST_BASE,
                                                 ransac_n=PLANE_RANSAC_N,
                                                 num_iterations=PLANE_ITERS)
    if len(inliers_idx) < 50:
        return [None] * 6
    inlier_cloud = pcd.select_by_index(inliers_idx)

    a, b, c, d = plane_model
    z_axis = np.array([a, b, c], dtype=np.float32)
    if z_axis[2] < 0: z_axis = -z_axis
    z_axis = z_axis / (np.linalg.norm(z_axis) + 1e-9)

    origin = inlier_cloud.get_center()

    _, cov = inlier_cloud.compute_mean_and_covariance()
    evals, evecs = np.linalg.eigh(cov)
    axes = evecs.T[np.argsort(evals)]
    x_axis = axes[2]

    ex = np.array([1.0, 0.0, 0.0], dtype=np.float32)
    r_x = ex - z_axis * float(ex @ z_axis)
    if np.linalg.norm(r_x) > 1e-9 and float(x_axis @ r_x) < 0:
        x_axis = -x_axis
    x_axis = x_axis / (np.linalg.norm(x_axis) + 1e-9)
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / (np.linalg.norm(y_axis) + 1e-9)

    return x_axis.astype(np.float32), y_axis.astype(np.float32), z_axis.astype(np.float32), \
           origin.astype(np.float32), np.asarray(inlier_cloud.points, dtype=np.float32), (a,b,c,d)

def plane_inlier_mask_from_model(mask_bin, depth_frame, intr, depth_scale, plane_model, dist_th=PLANE_DIST_BASE*1.5):
    if plane_model is None:
        return np.zeros_like(mask_bin, dtype=np.uint8)
    a, b, c, d = plane_model
    depth = np.asanyarray(depth_frame.get_data())
    H, W = depth.shape[:2]
    if mask_bin.shape[:2] != (H, W):
        mask_bin = cv2.resize(mask_bin, (W, H), interpolation=cv2.INTER_NEAREST)
    ys, xs = np.where(mask_bin > 0)
    if ys.size == 0:
        return np.zeros((H, W), dtype=np.uint8)
    zs = depth[ys, xs].astype(np.float32) * depth_scale
    valid = (zs > 1e-6)
    if not np.any(valid):
        return np.zeros((H, W), dtype=np.uint8)
    xs, ys, zs = xs[valid], ys[valid], zs[valid]
    X = (xs - intr.ppx) * zs / intr.fx
    Y = (ys - intr.ppy) * zs / intr.fy
    numer = np.abs(a*X + b*Y + c*zs + d)
    denom = (a*a + b*b + c*c) ** 0.5 + 1e-9
    dist = numer / denom
    inliers = (dist <= dist_th)
    out = np.zeros((H, W), dtype=np.uint8)
    out[ys[inliers], xs[inliers]] = 1
    return out

def largest_cluster_dbscan(inlier_points, eps=DBSCAN_EPS_M, min_points=DBSCAN_MINPTS):
    return None  # 비활성화

def ray_plane_intersect(cx, cy, intr, plane_model):
    if plane_model is None:
        return None
    a,b,c,d = plane_model
    vx = (cx - intr.ppx) / intr.fx
    vy = (cy - intr.ppy) / intr.fy
    vz = 1.0
    denom = a*vx + b*vy + c*vz
    if abs(denom) < 1e-9:
        return None
    t = -d / denom
    if t <= 0:
        return None
    return np.array([t*vx, t*vy, t*vz], dtype=np.float32)

def _largest_contour(mask_bin):
    cnts, _ = cv2.findContours(mask_bin.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts: return None
    return max(cnts, key=cv2.contourArea)

def _rasterize_rotated_rect(shape_hw, rect):
    box = cv2.boxPoints(rect).astype(np.int32)
    mask = np.zeros(shape_hw, dtype=np.uint8)
    cv2.fillPoly(mask, [box], 1)
    return mask, box

def refine_center_minarearect_with_size(mask_bin, depth_frame, intr, depth_scale, L_m, W_m,
                                        plane_model, x_axis, y_axis):
    H, W = mask_bin.shape[:2]
    cnt = _largest_contour(mask_bin)
    if cnt is None or cv2.contourArea(cnt) < 10:
        return None, None, None, None, None, None, None

    rect = cv2.minAreaRect(cnt)  # ((cx,cy),(w0,h0),angle)
    (cx, cy), (w0, h0), ang = rect
    rect_orig_box_pts = cv2.boxPoints(rect).astype(np.int32)

    ys, xs = np.where(mask_bin > 0)
    if xs.size == 0:
        return None, None, None, None, None, None, None

    depth = np.asanyarray(depth_frame.get_data())
    z_vals = depth[ys, xs].astype(np.float32) * depth_scale
    z_vals = z_vals[z_vals > 1e-6]
    if z_vals.size == 0:
        return None, None, None, None, None, None, None
    z_med = float(np.median(z_vals))

    scale_x = float(np.sqrt(x_axis[0]**2 + x_axis[1]**2)) if x_axis is not None else 1.0
    scale_y = float(np.sqrt(y_axis[0]**2 + y_axis[1]**2)) if y_axis is not None else 1.0

    fx, fy = intr.fx, intr.fy
    w_exp1 = (L_m * fx / max(z_med, 1e-6)) * scale_x
    h_exp1 = (W_m * fy / max(z_med, 1e-6)) * scale_y
    w_exp2 = (W_m * fx / max(z_med, 1e-6)) * scale_x
    h_exp2 = (L_m * fy / max(z_med, 1e-6)) * scale_y

    err1 = abs(w0 - w_exp1)/(w_exp1+1e-6) + abs(h0 - h_exp1)/(h_exp1+1e-6)
    err2 = abs(w0 - w_exp2)/(w_exp2+1e-6) + abs(h0 - h_exp2)/(h_exp2+1e-6)
    if err2 < err1:
        w_snap, h_snap = w_exp2, h_exp2
    else:
        w_snap, h_snap = w_exp1, h_exp1

    rect_snap = ((cx, cy), (w_snap, h_snap), ang)
    mask_rect, rect_snap_box_pts = _rasterize_rotated_rect((H, W), rect_snap)
    roi = (mask_bin.astype(np.uint8) & mask_rect.astype(np.uint8))

    pts, _, _ = get_points_from_mask(roi, depth_frame, intr, depth_scale)
    if pts is None or pts.shape[0] < 20:
        pt = ray_plane_intersect(cx, cy, intr, plane_model)
        if pt is not None:
            return pt.astype(np.float32), (cx, cy), z_med, rect_snap, rect_snap_box_pts, rect_orig_box_pts, "A-ray-plane"
        cx_i, cy_i = int(round(cx)), int(round(cy))
        x0, x1 = max(0, cx_i-2), min(W-1, cx_i+2)
        y0, y1 = max(0, cy_i-2), min(H-1, cy_i+2)
        patch = depth[y0:y1+1, x0:x1+1].astype(np.float32) * depth_scale
        patch = patch[patch > 1e-6]
        if patch.size == 0:
            return None, None, None, rect_snap, rect_snap_box_pts, rect_orig_box_pts, None
        zc = float(np.median(patch))
        X = (cx - intr.ppx) * zc / intr.fx
        Y = (cy - intr.ppy) * zc / intr.fy
        refined_origin = np.array([X, Y, zc], dtype=np.float32)
        return refined_origin, (cx, cy), z_med, rect_snap, rect_snap_box_pts, rect_orig_box_pts, "A-depth-median"

    refined_origin = pts.mean(axis=0).astype(np.float32)
    return refined_origin, (cx, cy), z_med, rect_snap, rect_snap_box_pts, rect_orig_box_pts, "B-ROI-mean"

def plane_center_from_points(points_xyz, plane_origin, x_axis, y_axis):
    if points_xyz is None or len(points_xyz) == 0:
        return None
    p_rel = points_xyz - plane_origin.reshape(1,3)
    x_coords = p_rel @ x_axis.reshape(3,1)
    y_coords = p_rel @ y_axis.reshape(3,1)
    mx = float(x_coords.mean()); my = float(y_coords.mean())
    center3d = plane_origin + mx * x_axis + my * y_axis
    return center3d.astype(np.float32)

def detect_area(points_xyz, plane_origin, x_axis, y_axis):
    """평면좌표로 투영했을 때의 최소 폭(mm) 근사"""
    if points_xyz is None or len(points_xyz) < 2:
        return None
    P  = points_xyz.astype(np.float32, copy=False)
    o  = plane_origin.reshape(1, 3).astype(np.float32, copy=False)
    xa = x_axis.reshape(3, 1).astype(np.float32, copy=False)
    ya = y_axis.reshape(3, 1).astype(np.float32, copy=False)
    rel = P - o
    x_coords = (rel @ xa).reshape(-1)
    y_coords = (rel @ ya).reshape(-1)
    range_x = float(x_coords.max() - x_coords.min())
    range_y = float(y_coords.max() - y_coords.min())
    width_m = min(range_x, range_y)
    return width_m * 1000.0

def classify_by_min_width_threshold(min_width_mm, threshold_mm=WIDTH_THRESHOLD_MM):
    if min_width_mm is None:
        return "uncertain"
    return "length_area" if float(min_width_mm) < float(threshold_mm) else "width_area"

def _xy_norm(x, y):
    return math.sqrt(x*x + y*y)

def select_candidate_block1(candidates, preferred_label, img_w, img_h):
    if not candidates or not preferred_label:
        return None
    same_label = []
    for c in candidates:
        if c.get('origin') is None:
            continue
        if str(c.get('label', '')) != str(preferred_label):
            continue
        same_label.append(c)
    if len(same_label) == 0:
        return None
    if len(same_label) == 1:
        return same_label[0]

    cx0 = (img_w - 1) * 0.5
    cy0 = (img_h - 1) * 0.5
    best = None
    best_key = None  # (center_dist, z, l1_xy)

    for v in same_label:
        cx = v.get('cx', None)
        cy = v.get('cy', None)
        if cx is None or cy is None:
            center_dist = float('inf')
        else:
            dx = float(cx) - cx0
            dy = float(cy) - cy0
            center_dist = math.hypot(dx, dy)

        ox, oy, oz = map(float, v['origin'])
        l1_xy = abs(ox) + abs(oy)
        key = (center_dist, oz, l1_xy)
        if (best is None) or (key < best_key):
            best = v
            best_key = key
    return best

# =========================
# Matplotlib 시각화 스텁 (비활성화)
# =========================
class MPLLiveOne:
    def __init__(self): pass
    def update(self, raw_pose, final_pose, inliers): return

# =========================
# ROS2 노드
# =========================
class BlockPosePublisher(Node):
    def __init__(self):
        super().__init__('block_pose_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/zeus/array/block_pose', 10)
        self.pose_pubisher = self.create_publisher(Float32MultiArray, '/zeus/rpy/block_pose', 10)
        self.block_pubisher = self.create_publisher(String, '/zeus/string/block_color', 10)
        self.subscriber = self.create_subscription(String, '/zeus/string/block_order', self.listener_callback, 10)
        self.detect_signal = ""
        self.cls = ""  # block1에서 선호 라벨로 사용

        self.model = YOLO(WEIGHTS)
        self.get_logger().info(f"YOLO Model Loaded. Classes: {self.model.names}")

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, COLOR_FPS)
        config.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, DEPTH_FPS)
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        self.mpl = MPLLiveOne()

        self.last_quat = None
        self.last_center = None
        self.stable_count = 0
        self.cooldown = 0

    def listener_callback(self, msg):
        self.get_logger().info(f'Received order: {msg.data}')
        self.detect_signal = msg.data

    def publish_block(self, label):
        msg = String()
        msg.data = label
        self.block_pubisher.publish(msg)
        self.get_logger().info(f'Published Block: {msg.data}')

# =========================
# 메인 루프
# =========================
def main(args=None):
    if not (O3D_OK and SCIPY_OK):
        print("[ERROR] Open3D와 SciPy가 모두 필요합니다.")
        return

    rclpy.init(args=args)
    node = BlockPosePublisher()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            mode = (node.detect_signal or "").strip().lower()

            try:
                frames = node.align.process(node.pipeline.wait_for_frames(timeout_ms=1000))
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not color_frame or not depth_frame:
                    continue
            except RuntimeError:
                node.get_logger().warning("Timeout waiting for frames from RealSense camera.")
                continue

            color_image = np.asanyarray(color_frame.get_data())
            overlay = color_image.copy()
            plane_vis = (color_image * 0.3).astype(np.uint8)
            plane_accum_mask = np.zeros((COLOR_H, COLOR_W), dtype=np.uint8)

            res = node.model(color_image, conf=CONF_DET, iou=IOU_TH, device=DEVICE, imgsz=IMG_SIZE, verbose=False)
            candidates = []

            if res and res[0].boxes is not None and len(res[0].boxes) > 0:
                r = res[0]
                has_masks = (r.masks is not None) and (r.masks.data is not None) and (len(r.masks.data) == len(r.boxes))
                confs = r.boxes.conf.detach().cpu().numpy().reshape(-1)
                xyxy  = r.boxes.xyxy.detach().cpu().numpy()
                clss  = r.boxes.cls.detach().cpu().numpy().astype(int) if r.boxes.cls is not None else np.zeros_like(confs, dtype=int)
                names = getattr(r, "names", None)

                idxs = np.where(confs >= CONF_PUB)[0]
                for i in idxs:
                    conf_i = float(confs[i])
                    x1, y1, x2, y2 = xyxy[i].astype(int)
                    label = names[clss[i]] if (names is not None and clss[i] < len(names)) else f"id{clss[i]}"

                    # 2D 마스크
                    if has_masks:
                        mask = r.masks.data[i].detach().cpu().numpy()
                        mask_bin = (mask > 0.5).astype(np.uint8)
                    else:
                        mask_bin = np.zeros((COLOR_H, COLOR_W), dtype=np.uint8)
                        mask_bin[max(y1,0):min(y2,COLOR_H), max(x1,0):min(x2,COLOR_W)] = 1

                    ys0, xs0 = np.where(mask_bin > 0)
                    if xs0.size > 0 and ys0.size > 0:
                        cx0, cy0 = int(xs0.mean()), int(ys0.mean())
                    else:
                        cx0, cy0 = int((x1 + x2)//2), int((y1 + y2)//2)

                    mask_bin = connected_components_keep(mask_bin,
                                                         min_area_px=MIN_AREA_PX,
                                                         keep_largest=True,
                                                         center_bias=(cx0, cy0))

                    ys, xs = np.where(mask_bin > 0)
                    if xs.size == 0:
                        continue
                    cx, cy = int(xs.mean()), int(ys.mean())

                    overlay = apply_mask_overlay(overlay, mask_bin, color=(0,255,255), alpha=0.35)
                    cv2.rectangle(overlay, (x1,y1), (x2,y2), (0,0,0), 2)
                    txt = f"{label} {conf_i:.2f}"
                    cv2.putText(overlay, txt, (x1, max(0,y1-8)), FONT, 0.6, (255,255,255), 3, cv2.LINE_AA)
                    cv2.putText(overlay, txt, (x1, max(0,y1-8)), FONT, 0.6, (0,0,0), 1, cv2.LINE_AA)

                    pts, _, _ = get_points_from_mask(mask_bin, depth_frame, node.intr, node.depth_scale)
                    x_axis, y_axis, z_axis, origin_m, inliers, plane_model = segment_plane_and_axes(pts)
                    if origin_m is None:
                        continue

                    plane_inlier_mask = plane_inlier_mask_from_model(mask_bin, depth_frame, node.intr, node.depth_scale, plane_model)
                    plane_inlier_mask = keep_largest_component(plane_inlier_mask)
                    ratio = float(plane_inlier_mask.sum()) / (float(mask_bin.sum()) + 1e-9)
                    if ratio < INLIER_RATIO_TH:
                        continue

                    if plane_inlier_mask.any():
                        plane_accum_mask = np.clip(plane_accum_mask + plane_inlier_mask, 0, 1)

                    refined_origin, rect_center_px, z_med, rect_snap, rect_snap_box_pts, rect_orig_box_pts, origin_src = \
                        refine_center_minarearect_with_size(
                            plane_inlier_mask, depth_frame, node.intr, node.depth_scale,
                            BLOCK_LEN_M, BLOCK_WID_M,
                            plane_model, x_axis, y_axis
                        )

                    if rect_orig_box_pts is not None:
                        cv2.polylines(overlay, [rect_orig_box_pts], True, (255,255,255), 1, cv2.LINE_AA)
                    if rect_snap_box_pts is not None:
                        cv2.polylines(overlay, [rect_snap_box_pts], True, (0,255,255), 2, cv2.LINE_AA)
                    if rect_center_px is not None:
                        cv2.circle(overlay, (int(rect_center_px[0]), int(rect_center_px[1])), 3, (0,255,255), -1, cv2.LINE_AA)

                    if refined_origin is not None:
                        origin_m = refined_origin
                    else:
                        origin_src = origin_src or "-"

                    # 후보 폭(mm) 계산 및 face 분류 저장
                    min_width_mm = detect_area(inliers, origin_m, x_axis, y_axis)
                    face = classify_by_min_width_threshold(min_width_mm, WIDTH_THRESHOLD_MM)

                    origin_px = project_point_to_pixel(origin_m, node.intr)
                    draw_cross(overlay, origin_px, color=(255,255,0), size=5, thickness=2)
                    if origin_px is not None:
                        cv2.putText(overlay, f"{label[:6]}:{origin_src or '-'}",
                                    (max(0, origin_px[0]-40), max(12, origin_px[1]-10)),
                                    FONT, 0.45, (0,0,0), 3, cv2.LINE_AA)
                        cv2.putText(overlay, f"{label[:6]}:{origin_src or '-'}",
                                    (max(0, origin_px[0]-40), max(12, origin_px[1]-10)),
                                    FONT, 0.45, (255,255,0), 1, cv2.LINE_AA)

                    candidates.append(dict(
                        label=label,
                        conf=conf_i, box=(x1,y1,x2,y2),
                        origin=origin_m, axes=(x_axis, y_axis, z_axis),
                        inliers=inliers, plane_model=plane_model, cx=cx, cy=cy,
                        src=origin_src or "-",
                        face=face,               # 'length_area' / 'width_area' / 'uncertain'
                        width_mm=min_width_mm    # 참고용
                    ))

            if plane_accum_mask.any():
                plane_vis = apply_mask_overlay(plane_vis, plane_accum_mask, color=(0,255,0), alpha=0.8)

            # -------------------------
            # 모드별 후보 선택 & 퍼블리시
            # -------------------------
            chosen = None

            if mode == 'block0':
                # 1) length_area 제외
                valid = []
                for c in candidates:
                    if c.get('origin') is None:
                        continue
                    if c.get('face', 'uncertain') == 'length_area':
                        continue
                    valid.append(c)

                # 2) 가장 낮은 z의 층 선택(± SAME_LAYER_EPS_M)
                if valid:
                    min_z_val = min([float(v['origin'][2]) for v in valid])
                    same_layer = [v for v in valid if abs(float(v['origin'][2]) - min_z_val) <= SAME_LAYER_EPS_M]

                    # 3) (x,y) 거리 최소 → 동률이면 z 더 작은 것 → 그래도 동률이면 |x|+|y| 작은 것
                    def key_block0(v):
                        ox, oy, oz = map(float, v['origin'])
                        dxy = _xy_norm(ox, oy)
                        l1  = abs(ox) + abs(oy)
                        return (dxy, oz, l1)
                    chosen = min(same_layer, key=key_block0) if same_layer else None

                # --- block0: R=I, t=[x_mm, y_mm, 0] 로 1회 발행 ---
                if chosen is not None:
                    x_axis, y_axis, z_axis = chosen['axes']   # (미사용)
                    origin_m = chosen['origin']
                    cx, cy = chosen['cx'], chosen['cy']

                    # block1 선호 라벨로 저장
                    node.cls = chosen['label']

                    final_px = project_point_to_pixel(origin_m, node.intr)
                    draw_cross(overlay, final_px, color=(0,0,255), size=7, thickness=2)
                    if final_px is not None:
                        cv2.putText(overlay, f"CHOSEN z={origin_m[2]:.3f}m src={chosen['src']}",
                                    (max(0, final_px[0]-60), min(COLOR_H-5, final_px[1]+18)),
                                    FONT, 0.5, (0,0,0), 3, cv2.LINE_AA)
                        cv2.putText(overlay, f"CHOSEN z={origin_m[2]:.3f}m src={chosen['src']}",
                                    (max(0, final_px[0]-60), min(COLOR_H-5, final_px[1]+18)),
                                    FONT, 0.5, (0,0,255), 1, cv2.LINE_AA)

                    x_mm = float(origin_m[0] * 1000.0)
                    y_mm = float(origin_m[1] * 1000.0)
                    final_matrix = np.eye(4, dtype=np.float64)
                    final_matrix[:3, :3] = np.eye(3, dtype=np.float64)
                    final_matrix[:3, 3]  = [x_mm, y_mm, 0.0]

                    # 안정화(센터 점프만)
                    if node.last_center is not None:
                        center_jump = math.hypot(cx - node.last_center[0], cy - node.last_center[1])
                    else:
                        center_jump = 0.0
                    same_target = (center_jump <= STAB_CENTER_JUMP_PX)
                    node.stable_count = node.stable_count + 1 if same_target else 0
                    node.cooldown = max(0, node.cooldown - 1)
                    node.last_center = (cx, cy)

                    if node.detect_signal != "":
                        # 4x4 행렬 퍼블리시
                        msg = Float32MultiArray()
                        rows, cols = 4, 4
                        msg.layout.dim.append(MultiArrayDimension(label='rows', size=rows, stride=cols))
                        msg.layout.dim.append(MultiArrayDimension(label='cols', size=cols, stride=1))
                        msg.layout.data_offset = 0
                        msg.data = final_matrix.flatten().astype(np.float32).tolist()
                        node.publisher_.publish(msg)

                        # 블록 라벨 알림
                        node.publish_block(chosen['label'])

                        node.cooldown = PUBLISH_COOLDOWN_FRAMES
                        node.stable_count = 0
                        node.detect_signal = ""  # 1회 발행 후 게이트 닫기

                    cv2.putText(
                        overlay,
                        f"BLOCK0 XY: {origin_m[0]:.3f},{origin_m[1]:.3f} m | jump:{center_jump:.1f}px | stab:{node.stable_count} | cd:{node.cooldown}",
                        (max(0, cx-220), max(20, cy-20)),
                        FONT, 0.5, (255,255,255), 4, cv2.LINE_AA
                    )
                    cv2.putText(
                        overlay,
                        f"BLOCK0 XY: {origin_m[0]:.3f},{origin_m[1]:.3f} m | jump:{center_jump:.1f}px | stab:{node.stable_count} | cd:{node.cooldown}",
                        (max(0, cx-220), max(20, cy-20)),
                        FONT, 0.5, (0,0,0), 2, cv2.LINE_AA
                    )

                # block0에서는 자세 추정/Matplotlib 갱신 없음
                mpl_raw_pose = None
                final_pose   = None
                mpl_inliers  = None

            elif mode == 'block1':
                # 선호 라벨 우선 후보 선택 → 없으면 중앙가까움 기준 백업
                preferred = node.cls
                chosen = select_candidate_block1(candidates, preferred_label=preferred, img_w=COLOR_W, img_h=COLOR_H)
                if chosen is None:
                    pool = [c for c in candidates if c.get('origin') is not None]
                    if pool:
                        cx0 = (COLOR_W - 1) * 0.5
                        cy0 = (COLOR_H - 1) * 0.5
                        def key(v):
                            cx, cy = v.get('cx'), v.get('cy')
                            center_dist = float('inf') if (cx is None or cy is None) else math.hypot(cx - cx0, cy - cy0)
                            ox, oy, oz = map(float, v['origin'])
                            l1_xy = abs(ox) + abs(oy)
                            return (center_dist, oz, l1_xy)
                        chosen = min(pool, key=key)

                mpl_raw_pose = final_pose = None
                mpl_inliers = None

                if chosen is not None:
                    x_axis, y_axis, z_axis = chosen['axes']
                    origin_m = chosen['origin']
                    cx, cy = chosen['cx'], chosen['cy']

                    final_px = project_point_to_pixel(origin_m, node.intr)
                    draw_cross(overlay, final_px, color=(0,0,255), size=7, thickness=2)
                    if final_px is not None:
                        cv2.putText(overlay, f"CHOSEN z={origin_m[2]:.3f}m src={chosen['src']}",
                                    (max(0, final_px[0]-60), min(COLOR_H-5, final_px[1]+18)),
                                    FONT, 0.5, (0,0,0), 3, cv2.LINE_AA)
                        cv2.putText(overlay, f"CHOSEN z={origin_m[2]:.3f}m src={chosen['src']}",
                                    (max(0, final_px[0]-60), min(COLOR_H-5, final_px[1]+18)),
                                    FONT, 0.5, (0,0,255), 1, cv2.LINE_AA)

                    if all(v is not None for v in [x_axis, y_axis, z_axis, origin_m]):
                        raw_rot_matrix = np.stack([x_axis, y_axis, z_axis], axis=1)
                        mpl_raw_pose = (origin_m, raw_rot_matrix, None, None)
                        mpl_inliers  = chosen['inliers']

                        raw_quat = SciRot.from_matrix(raw_rot_matrix).as_quat()
                        raw_rotation = SciRot.from_quat(raw_quat)
                        ypr = raw_rotation.as_euler('zyx', degrees=True)
                        yaw, pitch, roll = float(ypr[0]), float(ypr[1]), float(ypr[2])

                        # dead-zone 적용
                        is_roll_high  = abs(roll)  >= DEAD_ZONE_DEG
                        is_pitch_high = abs(pitch) >= DEAD_ZONE_DEG
                        if is_roll_high and is_pitch_high:
                            if abs(roll) >= abs(pitch): pitch = 0.0
                            else: roll = 0.0
                        else:
                            if not is_pitch_high: pitch = 0.0
                            if not is_roll_high:  roll  = 0.0

                        final_rpy = (roll, pitch, yaw)
                        final_rotation   = SciRot.from_euler('zyx', [yaw, pitch, roll], degrees=True)
                        final_rot_matrix = final_rotation.as_matrix()
                        final_quat       = final_rotation.as_quat()

                        origin_mm = origin_m * 1000.0
                        final_matrix = np.eye(4, dtype=np.float64)
                        final_matrix[:3, :3] = final_rot_matrix
                        final_matrix[:3, 3]  = origin_mm

                        # 안정화(센터+자세)
                        if node.last_center is not None:
                            dx = cx - node.last_center[0]
                            dy = cy - node.last_center[1]
                            center_jump = (dx*dx + dy*dy) ** 0.5
                        else:
                            center_jump = 0.0
                        same_target = (center_jump <= STAB_CENTER_JUMP_PX)

                        def quat_delta_deg(q1, q2):
                            if q1 is None or q2 is None:
                                return np.inf
                            dot = float(np.dot(q1, q2))
                            if dot < 0.0: dot = -dot
                            dot = np.clip(dot, -1.0, 1.0)
                            return 2.0 * np.degrees(np.arccos(dot))

                        dtheta = quat_delta_deg(node.last_quat, final_quat)

                        if not same_target:
                            node.stable_count = 0
                            node.cooldown = max(0, node.cooldown - 1)
                        else:
                            if dtheta < STAB_ANGLE_DEG:
                                node.stable_count += 1
                            else:
                                node.stable_count = 0
                            node.cooldown = max(0, node.cooldown - 1)

                        node.last_quat = final_quat
                        node.last_center = (cx, cy)

                        if node.detect_signal != "":
                            # 4x4 행렬 퍼블리시
                            msg = Float32MultiArray()
                            rows, cols = 4, 4
                            msg.layout.dim.append(MultiArrayDimension(label='rows', size=rows, stride=cols))
                            msg.layout.dim.append(MultiArrayDimension(label='cols', size=cols, stride=1))
                            msg.layout.data_offset = 0
                            msg.data = final_matrix.flatten().astype(np.float32).tolist()
                            node.publisher_.publish(msg)

                            # RPY 퍼블리시
                            msg2 = Float32MultiArray()
                            msg2.data = [roll, pitch, yaw]
                            node.pose_pubisher.publish(msg2)
                            node.get_logger().info(f'Published Pose: {msg2.data}')

                            node.get_logger().info(
                                f"[STABLE PUBLISH] z={origin_m[2]:.3f} m | Δθ={dtheta:.3f}° | R:{roll:.1f} P:{pitch:.1f} Y:{yaw:.1f}"
                            )
                            node.publish_block(chosen['label'])
                            node.cooldown = PUBLISH_COOLDOWN_FRAMES
                            node.stable_count = 0
                            node.detect_signal = ""

                        cv2.putText(
                            overlay,
                            f"NEAREST  R:{roll:.1f} P:{pitch:.1f} Y:{yaw:.1f} | dθ:{(dtheta if np.isfinite(dtheta) else 999):.3f}° "
                            f"| stab:{node.stable_count}/{STAB_MIN_FRAMES} | cd:{node.cooldown}",
                            (max(0, cx-180), max(20, cy-20)),
                            FONT, 0.55, (255,255,255), 4, cv2.LINE_AA
                        )
                        cv2.putText(
                            overlay,
                            f"NEAREST  R:{roll:.1f} P:{pitch:.1f} Y:{yaw:.1f} | dθ:{(dtheta if np.isfinite(dtheta) else 999):.3f}° "
                            f"| stab:{node.stable_count}/{STAB_MIN_FRAMES} | cd:{node.cooldown}",
                            (max(0, cx-180), max(20, cy-20)),
                            FONT, 0.55, (0,0,0), 2, cv2.LINE_AA
                        )

                        final_pose = (origin_m, final_rot_matrix, final_rpy, final_quat)

            else:
                # 기본 모드: 카메라 원점에서 가장 가까운 블록 표시만
                if candidates:
                    def dist(p): return math.sqrt(p[0]**2 + p[1]**2 + p[2]**2)
                    chosen = min((c for c in candidates if c['origin'] is not None),
                                 key=lambda c: dist(c['origin']), default=None)
                mpl_raw_pose = None
                final_pose   = None
                mpl_inliers  = None
                if chosen is not None:
                    origin_m = chosen['origin']
                    final_px = project_point_to_pixel(origin_m, node.intr)
                    draw_cross(overlay, final_px, color=(0,0,255), size=7, thickness=2)
                    if final_px is not None:
                        cv2.putText(overlay, f"CHOSEN z={origin_m[2]:.3f}m src={chosen['src']}",
                                    (max(0, final_px[0]-60), min(COLOR_H-5, final_px[1]+18)),
                                    FONT, 0.5, (0,0,0), 3, cv2.LINE_AA)
                        cv2.putText(overlay, f"CHOSEN z={origin_m[2]:.3f}m src={chosen['src']}",
                                    (max(0, final_px[0]-60), min(COLOR_H-5, final_px[1]+18)),
                                    FONT, 0.5, (0,0,255), 1, cv2.LINE_AA)

            # Matplotlib(비활성) 업데이트 호출
            node.mpl.update(mpl_raw_pose, final_pose, mpl_inliers)

            cv2.imshow("Detections (mask+box + centers)", overlay)
            cv2.imshow("PlaneFiltered (all blocks)", plane_vis)
            cv2.waitKey(1)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) received. Shutting down...")
    finally:
        node.pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()