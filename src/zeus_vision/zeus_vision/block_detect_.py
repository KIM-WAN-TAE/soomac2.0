# -*- coding: utf-8 -*-

import os
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import warnings; warnings.filterwarnings("ignore")
import math
import time

# ===== 시각화/클러스터링 스위치 =====
USE_MPL = False
USE_DBSCAN_3D = False

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
WEIGHTS    = "/home/pc/soomac_ws/src/zeus_vision/best_no_bri_roboflow.pt"
DEVICE     = "0"

CONF_DET   = 0.28
CONF_PUB   = 0.70
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
SAME_LAYER_EPS_M   = 0.003
WIDTH_THRESHOLD_MM = 24.0
AREA_THRESHOLD_MM2 = 1750.0  # mm^2

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

# ---- RealSense 왜곡 포함 역투영 ----
def deproject_points_from_mask_rs(mask_bin, depth_frame, intr_full, depth_scale, stride=PCL_STRIDE):
    depth = np.asanyarray(depth_frame.get_data())
    H, W = depth.shape[:2]
    if mask_bin.shape[:2] != (H, W):
        mask_bin = cv2.resize(mask_bin, (W, H), interpolation=cv2.INTER_NEAREST)

    ys, xs = np.where(mask_bin > 0)
    if ys.size == 0:
        return None, None, None

    xs = xs[::stride]
    ys = ys[::stride]
    z  = depth[ys, xs].astype(np.float32) * depth_scale
    ok = z > 1e-6
    if not np.any(ok):
        return None, None, None
    xs, ys, z = xs[ok], ys[ok], z[ok]

    pts = np.empty((xs.size, 3), dtype=np.float32)
    for i, (u, v, zz) in enumerate(zip(xs, ys, z)):
        X, Y, Z = rs.rs2_deproject_pixel_to_point(intr_full, [float(u), float(v)], float(zz))
        pts[i] = (X, Y, Z)
    return pts, xs, ys

# =========================
# [VIS] 투영/마커 (왜곡 포함)
# =========================
def project_point_to_pixel_rs(P, intr_full):
    if not np.isfinite(P[2]) or P[2] <= 1e-9:
        return None
    u, v = rs.rs2_project_point_to_pixel(intr_full, [float(P[0]), float(P[1]), float(P[2])])
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

def plane_inlier_mask_from_model(mask_bin, depth_frame, intr_full, depth_scale,
                                 plane_model, dist_th=PLANE_DIST_BASE*1.5, stride=1):
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

    ys = ys[::max(1, stride)]
    xs = xs[::max(1, stride)]
    zs = depth[ys, xs].astype(np.float32) * depth_scale
    valid = zs > 1e-6
    if not np.any(valid):
        return np.zeros((H, W), dtype=np.uint8)
    ys, xs, zs = ys[valid], xs[valid], zs[valid]

    pts = np.empty((zs.size, 3), dtype=np.float32)
    for i, (u, v, z) in enumerate(zip(xs, ys, zs)):
        X, Y, Z = rs.rs2_deproject_pixel_to_point(intr_full, [float(u), float(v)], float(z))
        pts[i] = (X, Y, Z)

    numer = np.abs(a*pts[:,0] + b*pts[:,1] + c*pts[:,2] + d)
    denom = (a*a + b*b + c*c) ** 0.5 + 1e-9
    dist = numer / denom
    inliers = dist <= dist_th

    out = np.zeros((H, W), dtype=np.uint8)
    out[ys[inliers], xs[inliers]] = 1
    return out

def ray_plane_intersect(cx, cy, intr_full, plane_model):
    if plane_model is None:
        return None
    a,b,c,d = plane_model
    vx, vy, vz = rs.rs2_deproject_pixel_to_point(intr_full, [float(cx), float(cy)], 1.0)
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

# ---- 2D 중심 → 3D 변환(왜곡 포함 교차 우선) ----
def get_3d_center_from_2d_pixel(rect_center_px, depth_frame, intr_full_rs, depth_scale, plane_model):
    cx, cy = rect_center_px
    pt_3d = ray_plane_intersect(cx, cy, intr_full_rs, plane_model)
    if pt_3d is not None:
        return pt_3d.astype(np.float32), "A-ray-plane"

    H, W = depth_frame.get_height(), depth_frame.get_width()
    cx_i, cy_i = int(round(cx)), int(round(cy))
    x0, x1 = max(0, cx_i - 2), min(W - 1, cx_i + 2)
    y0, y1 = max(0, cy_i - 2), min(H - 1, cy_i + 2)
    depth = np.asanyarray(depth_frame.get_data())
    patch = depth[y0:y1+1, x0:x1+1].astype(np.float32) * depth_scale
    patch = patch[patch > 1e-6]
    if patch.size > 0:
        z_med = float(np.median(patch))
        X, Y, Z = rs.rs2_deproject_pixel_to_point(intr_full_rs, [float(cx), float(cy)], float(z_med))
        return np.array([X, Y, Z], dtype=np.float32), "B-depth-median"
    return None, None

# ---- 중심 정제 ----
def refine_center_minarearect_with_size(mask_bin, depth_frame, intr_full_rs, depth_scale, L_m, W_m,
                                        plane_model, x_axis, y_axis):
    H, W = mask_bin.shape[:2]
    cnt = _largest_contour(mask_bin)
    if cnt is None or cv2.contourArea(cnt) < 10:
        return [None] * 8

    rect = cv2.minAreaRect(cnt)
    (cx, cy), (w0, h0), ang = rect
    rect_orig_box_pts = cv2.boxPoints(rect).astype(np.int32)

    depth = np.asanyarray(depth_frame.get_data())
    ys, xs = np.where(mask_bin > 0)
    if xs.size == 0:
        return [None] * 8
    z_vals = depth[ys, xs].astype(np.float32) * depth_scale
    z_vals = z_vals[z_vals > 1e-6]
    if z_vals.size == 0:
        return [None] * 8
    z_med = float(np.median(z_vals))

    scale_x = float(np.sqrt(x_axis[0]**2 + x_axis[1]**2)) if x_axis is not None else 1.0
    scale_y = float(np.sqrt(y_axis[0]**2 + y_axis[1]**2)) if y_axis is not None else 1.0

    fx = intr_full_rs.fx; fy = intr_full_rs.fy
    w_exp1 = (L_m * fx / max(z_med, 1e-6)) * scale_x
    h_exp1 = (W_m * fy / max(z_med, 1e-6)) * scale_y
    w_exp2 = (W_m * fx / max(z_med, 1e-6)) * scale_x
    h_exp2 = (L_m * fy / max(z_med, 1e-6)) * scale_y

    err1 = abs(w0 - w_exp1)/(w_exp1+1e-6) + abs(h0 - h_exp1)/(h_exp1+1e-6)
    err2 = abs(w0 - w_exp2)/(w_exp2+1e-6) + abs(h0 - h_exp2)/(h_exp2+1e-6)
    w_snap, h_snap = (w_exp2, h_exp2) if err2 < err1 else (w_exp1, h_exp1)

    rect_snap = ((cx, cy), (w_snap, h_snap), ang)
    _, rect_snap_box_pts = _rasterize_rotated_rect((H, W), rect_snap)

    refined_origin, origin_src = get_3d_center_from_2d_pixel(
        (cx, cy), depth_frame, intr_full_rs, depth_scale, plane_model
    )
    return refined_origin, (cx, cy), z_med, rect_snap, rect_snap_box_pts, rect_orig_box_pts, origin_src, None

def plane_center_from_points(points_xyz, plane_origin, x_axis, y_axis):
    if points_xyz is None or len(points_xyz) == 0:
        return None
    rel = points_xyz - plane_origin.reshape(1, 3)
    Xp = (rel @ x_axis.reshape(3, 1)).reshape(-1)
    Yp = (rel @ y_axis.reshape(3, 1)).reshape(-1)
    mx = float(np.median(Xp))
    my = float(np.median(Yp))
    return (plane_origin + mx * x_axis + my * y_axis).astype(np.float32)

def detect_area(points_xyz, plane_origin, x_axis, y_axis):
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

def _polygon_area_2d(points_xy):
    if points_xy is None or len(points_xy) < 3:
        return 0.0
    x = points_xy[:, 0]
    y = points_xy[:, 1]
    s = float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
    return abs(s) * 0.5

def area_mm2_from_inliers(inliers_xyz, plane_origin, x_axis, y_axis):
    if inliers_xyz is None or len(inliers_xyz) < 3:
        return None
    if any(v is None for v in [plane_origin, x_axis, y_axis]):
        return None
    P = inliers_xyz.astype(np.float32, copy=False)
    o = plane_origin.reshape(1, 3).astype(np.float32, copy=False)
    xa = x_axis.reshape(3, 1).astype(np.float32, copy=False)
    ya = y_axis.reshape(3, 1).astype(np.float32, copy=False)
    rel = P - o
    X = (rel @ xa).reshape(-1) * 1000.0
    Y = (rel @ ya).reshape(-1) * 1000.0
    pts2 = np.stack([X, Y], axis=1).astype(np.float32)
    hull = cv2.convexHull(pts2, returnPoints=True)
    hull = hull.reshape(-1, 2)
    area_mm2 = _polygon_area_2d(hull)
    return float(area_mm2)

def block_pass_filter(width_mm, area_mm2, width_thr_mm, area_thr_mm2):
    if width_mm is None or area_mm2 is None:
        return False
    if float(width_mm) < float(width_thr_mm):
        return False
    if float(area_mm2) < float(area_thr_mm2):
        return False
    return True

# =========================
# Matplotlib 시각화 스텁
# =========================
class MPLLiveOne:
    def __init__(self): pass
    def update(self, raw_pose, final_pose, inliers): return

# =========================
# HSV 색상 보정
# =========================
COLOR_NORMALIZE_SET = {"red", "pink", "purple", "green"}

def refine_label_by_hsv_mean(initial_label: str, mask_bin: np.ndarray, hsv_img: np.ndarray):
    if initial_label is None:
        return None, None, None, None
    lab = str(initial_label).lower()
    if lab not in COLOR_NORMALIZE_SET:
        return initial_label, None, None, None

    ys, xs = np.where(mask_bin > 0)
    if ys.size == 0:
        return initial_label, None, None, None

    hsv_vals = hsv_img[ys, xs].astype(np.float32)
    Hm = float(np.mean(hsv_vals[:, 0]))
    Sm = float(np.mean(hsv_vals[:, 1]))
    Vm = float(np.mean(hsv_vals[:, 2]))

    if lab in ("red", "pink"):
        lab = "pink" if Sm < 170.0 else "red"
    elif lab in ("purple", "green"):
        lab = "purple" if Hm > 100.0 else "green"

    return lab, Hm, Sm, Vm

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
        self.cls = ""

        self.model = YOLO(WEIGHTS)
        self.get_logger().info(f"YOLO Model Loaded. Classes: {self.model.names}")

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, COLOR_FPS)
        config.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, DEPTH_FPS)

        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

        self.color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.intr_full = self.color_stream.get_intrinsics()  # rs.intrinsics

        class SimpleIntr:
            __slots__ = ("fx","fy","ppx","ppy")
            def __init__(self, fx, fy, ppx, ppy):
                self.fx, self.fy, self.ppx, self.ppy = float(fx), float(fy), float(ppx), float(ppy)
        self.intr = SimpleIntr(self.intr_full.fx, self.intr_full.fy, self.intr_full.ppx, self.intr_full.ppy)

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
            hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

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
                    label_raw = names[clss[i]] if (names is not None and clss[i] < len(names)) else f"id{clss[i]}"

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

                    # HSV 라벨 보정
                    label_norm, Hm, Sm, Vm = refine_label_by_hsv_mean(label_raw, mask_bin, hsv_image)
                    label = label_norm if label_norm is not None else label_raw

                    overlay = apply_mask_overlay(overlay, mask_bin, color=(0,255,255), alpha=0.35)
                    cv2.rectangle(overlay, (x1,y1), (x2,y2), (0,0,0), 2)
                    txt = f"{label} {conf_i:.2f}"
                    cv2.putText(overlay, txt, (x1, max(0,y1-8)), FONT, 0.6, (255,255,255), 3, cv2.LINE_AA)
                    cv2.putText(overlay, txt, (x1, max(0,y1-8)), FONT, 0.6, (0,0,0), 1, cv2.LINE_AA)
                    if Hm is not None:
                        cv2.putText(overlay, f"H:{Hm:.1f} S:{Sm:.1f}",
                                    (x1, min(COLOR_H-5, y2+18)), FONT, 0.5, (0,0,0), 3, cv2.LINE_AA)
                        cv2.putText(overlay, f"H:{Hm:.1f} S:{Sm:.1f}",
                                    (x1, min(COLOR_H-5, y2+18)), FONT, 0.5, (0,255,255), 1, cv2.LINE_AA)

                    # ===== 3D 처리: 왜곡 포함 역투영 =====
                    pts_mask, _, _ = deproject_points_from_mask_rs(mask_bin, depth_frame, node.intr_full, node.depth_scale, stride=PCL_STRIDE)
                    x_axis, y_axis, z_axis, origin_m, inliers, plane_model = segment_plane_and_axes(pts_mask)
                    if origin_m is None:
                        continue

                    plane_inlier_mask = plane_inlier_mask_from_model(mask_bin, depth_frame, node.intr_full, node.depth_scale, plane_model, stride=1)
                    plane_inlier_mask = keep_largest_component(plane_inlier_mask)
                    ratio = float(plane_inlier_mask.sum()) / (float(mask_bin.sum()) + 1e-9)
                    if ratio < INLIER_RATIO_TH:
                        continue

                    if plane_inlier_mask.any():
                        plane_accum_mask = np.clip(plane_accum_mask + plane_inlier_mask, 0, 1)

                    # 스냅 크기 추정 + 2D중심→3D
                    refined_origin, rect_center_px, z_med, rect_snap, rect_snap_box_pts, rect_orig_box_pts, origin_src, roi_pts = \
                        refine_center_minarearect_with_size(
                            plane_inlier_mask, depth_frame, node.intr_full, node.depth_scale,
                            BLOCK_LEN_M, BLOCK_WID_M, plane_model, x_axis, y_axis
                        )

                    if rect_orig_box_pts is not None:
                        cv2.polylines(overlay, [rect_orig_box_pts], True, (255,255,255), 1, cv2.LINE_AA)
                    if rect_snap_box_pts is not None:
                        cv2.polylines(overlay, [rect_snap_box_pts], True, (0,255,255), 2, cv2.LINE_AA)
                    if rect_center_px is not None:
                        cv2.circle(overlay, (int(rect_center_px[0]), int(rect_center_px[1])), 3, (0,255,255), -1, cv2.LINE_AA)

                    # 평면 좌표계 중앙값으로 보정 시도
                    if roi_pts is not None and roi_pts.shape[0] >= 10:
                        po = plane_center_from_points(roi_pts, origin_m, x_axis, y_axis)
                        if po is not None:
                            refined_origin = po
                            origin_src = (origin_src or "") + "+PlaneMedian"

                    if refined_origin is not None:
                        origin_m = refined_origin

                    origin_px = project_point_to_pixel_rs(origin_m, node.intr_full)
                    draw_cross(overlay, origin_px, color=(255,255,0), size=5, thickness=2)
                    if origin_px is not None:
                        cv2.putText(overlay, f"{label[:6]}:{origin_src or '-'}",
                                    (max(0, origin_px[0]-40), max(12, origin_px[1]-10)),
                                    FONT, 0.45, (0,0,0), 3, cv2.LINE_AA)
                        cv2.putText(overlay, f"{label[:6]}:{origin_src or '-'}",
                                    (max(0, origin_px[0]-40), max(12, origin_px[1]-10)),
                                    FONT, 0.45, (255,255,0), 1, cv2.LINE_AA)

                    # ===== 면적(mm^2) & pitch(도)
                    pitch_deg = None
                    if all(v is not None for v in [x_axis, y_axis, z_axis]):
                        raw_rot_matrix = np.stack([x_axis, y_axis, z_axis], axis=1)
                        ypr = SciRot.from_matrix(raw_rot_matrix).as_euler('zyx', degrees=True)
                        pitch_deg = float(ypr[1])

                    area_mm2 = None
                    pts_for_area = inliers  # roi_pts가 None이므로 inliers 사용
                    area_mm2 = area_mm2_from_inliers(pts_for_area, origin_m, x_axis, y_axis)
                    if area_mm2 is None:
                        area_mm2 = 0.0

                    area_txt = f"area_mm2:{int(round(area_mm2))}"
                    cv2.putText(overlay, area_txt, (x1, max(0, y1 - 24)),
                                FONT, 0.6, (255,255,255), 3, cv2.LINE_AA)
                    cv2.putText(overlay, area_txt, (x1, max(0, y1 - 24)),
                                FONT, 0.6, (0,0,255), 1, cv2.LINE_AA)

                    candidates.append(dict(
                        label=label,
                        conf=conf_i, box=(x1,y1,x2,y2),
                        origin=origin_m, axes=(x_axis, y_axis, z_axis),
                        inliers=inliers, plane_model=plane_model, cx=cx, cy=cy,
                        src=origin_src or "-",
                        hsv=(Hm, Sm, Vm),
                        area_mm2=float(area_mm2),
                        pitch_deg=pitch_deg
                    ))

            if plane_accum_mask.any():
                plane_vis = apply_mask_overlay(plane_vis, plane_accum_mask, color=(0,255,0), alpha=0.8)

            # -------------------------
            # 선택 & 퍼블리시
            # -------------------------
            chosen = None
            pre_time = time.time()
            if mode == 'block':
                CLASS_ORDER = ['blue', 'green', 'pink', 'purple', 'red', 'yellow']
                AREA_TH_MM2  = float(AREA_THRESHOLD_MM2)

                def xy_dist_cam(c):
                    o = c.get('origin', None)
                    if o is None or not np.isfinite(o[0]) or not np.isfinite(o[1]):
                        return float('inf')
                    return math.hypot(float(o[0]), float(o[1]))

                def area_mm2_of(c):
                    return float(c.get('area_mm2', 0.0))

                for cls in CLASS_ORDER:
                    cand_cls = [c for c in candidates if str(c.get('label', '')).lower() == cls]
                    cand_sel = [c for c in cand_cls if area_mm2_of(c) >= AREA_TH_MM2]
                    if not cand_sel:
                        continue
                    chosen = min(cand_sel, key=xy_dist_cam)
                    break

                if chosen is not None:
                    x_axis, y_axis, z_axis = chosen['axes']
                    origin_m = chosen['origin']
                    label = chosen.get('label', None)
                    cx, cy = chosen['cx'], chosen['cy']

                    final_px = project_point_to_pixel_rs(origin_m, node.intr_full)
                    draw_cross(overlay, final_px, color=(0, 0, 255), size=7, thickness=2)

                    if all(v is not None for v in [x_axis, y_axis, z_axis]):
                        raw_rot_matrix = np.stack([x_axis, y_axis, z_axis], axis=1)
                        raw_quat = SciRot.from_matrix(raw_rot_matrix).as_quat()
                        raw_rotation = SciRot.from_quat(raw_quat)
                        ypr = raw_rotation.as_euler('zyx', degrees=True)
                        yaw, pitch, roll = float(ypr[0]), float(ypr[1]), float(ypr[2])

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
                    else:
                        final_rot_matrix = np.eye(3, dtype=np.float64)
                        final_quat       = SciRot.from_matrix(final_rot_matrix).as_quat()
                        final_rpy        = (0.0, 0.0, 0.0)

                    origin_mm = origin_m * 1000.0
                    final_matrix = np.eye(4, dtype=np.float64)
                    final_matrix[:3, :3] = final_rot_matrix
                    final_matrix[:3, 3]  = origin_mm

                    if node.last_center is not None:
                        dx = cx - node.last_center[0]
                        dy = cy - node.last_center[1]
                        center_jump = (dx*dx + dy*dy) ** 0.5
                    else:
                        center_jump = 0.0

                    def quat_delta_deg(q1, q2):
                        if q1 is None or q2 is None:
                            return np.inf
                        dot = float(np.dot(q1, q2))
                        if dot < 0.0: dot = -dot
                        dot = np.clip(dot, -1.0, 1.0)
                        return 2.0 * np.degrees(np.arccos(dot))

                    dtheta = quat_delta_deg(node.last_quat, final_quat)
                    same_target = (center_jump <= STAB_CENTER_JUMP_PX)

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
                        msg = Float32MultiArray()
                        rows, cols = 4, 4
                        msg.layout.dim.append(MultiArrayDimension(label='rows', size=rows, stride=cols))
                        msg.layout.dim.append(MultiArrayDimension(label='cols', size=cols, stride=1))
                        msg.layout.data_offset = 0
                        msg.data = final_matrix.flatten().astype(np.float32).tolist()
                        node.publisher_.publish(msg)

                        msg2 = Float32MultiArray()
                        roll, pitch, yaw = final_rpy
                        msg2.data = [roll, pitch, yaw]
                        node.pose_pubisher.publish(msg2)
                        node.get_logger().info(f'Published Pose: {msg2.data}')

                        node.publish_block(label if label is not None else "")

                        node.cooldown = PUBLISH_COOLDOWN_FRAMES
                        node.stable_count = 0
                        node.detect_signal = ""

                    if final_px is not None:
                        info1 = "CHOSEN (class→area_mm2≥thr→min XY)"
                        cv2.putText(overlay, info1,
                                    (max(0, final_px[0]-160), min(COLOR_H-8, final_px[1]+18)),
                                    FONT, 0.50, (255,255,255), 3, cv2.LINE_AA)
                        cv2.putText(overlay, info1,
                                    (max(0, final_px[0]-160), min(COLOR_H-8, final_px[1]+18)),
                                    FONT, 0.50, (0,0,255), 1, cv2.LINE_AA)
                    print(time.time() - pre_time)

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
