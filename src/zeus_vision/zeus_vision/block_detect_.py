# -*- coding: utf-8 -*-
import os
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import warnings; warnings.filterwarnings("ignore")
import math, time

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

# ---------- 원본 K, D (네가 구한 값) ----------
CAMERA_MATRIX = np.array([
    [609.59370966, 0.0,          327.77961006],
    [ 0.0,       610.16182704, 244.8987311],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

DIST_COEFFS = np.array([
    [ 2.98079773e-02,  7.71843130e-01,  1.12771351e-03,  1.91769037e-03, -2.86200282e+00]
], dtype=np.float32)

# ---------- 설정 ----------
WEIGHTS     = "/home/pc/Downloads/best_50cm_test_val.pt"
DEVICE      = "0"

CONF_DET    = 0.28
CONF_PUB    = 0.70
IOU_TH      = 0.45
IMG_SIZE    = 640

COLOR_W, COLOR_H, COLOR_FPS = 640, 480, 30
DEPTH_W, DEPTH_H, DEPTH_FPS = 640, 480, 30
FONT = cv2.FONT_HERSHEY_SIMPLEX

PCL_STRIDE = 3

####평면 추정 파라미터####
PLANE_DIST_BASE, PLANE_RANSAC_N, PLANE_ITERS = 0.003, 3, 1000
STAT_NB_NEIGHBORS, STAT_STD_RATIO = 20, 2.0
RAD_RADIUS, RAD_MIN_POINTS = 0.03, 10

DEAD_ZONE_DEG = 3.0
MIN_AREA_PX   = 800
INLIER_RATIO_TH = 0.30

STAB_ANGLE_DEG = 1.0
STAB_CENTER_JUMP_PX = 60
PUBLISH_COOLDOWN_FRAMES = 10

# --- 이너 코어 설정 추가 ---
# 마스크의 가장자리에서 최대 거리의 몇 % 안쪽까지를 코어로 사용할지 결정 (10%)
INNER_CORE_RATIO = 0.0

# 블록 크기 [m]
BLOCK_LEN_M = 0.075
BLOCK_WID_M = 0.025

AREA_THRESHOLD_MM2 = 1650.0  # mm^2
EE_OFFSET_MM = np.array([29.0, 67.0, 0.0], dtype=np.float32)
EE_OFFSET_M  = EE_OFFSET_MM / 1000.0  # m 단위 변환

# <<< ADD: 좌표 일치 판정 임계값 (카메라 좌표계, m) >>>
NEAR_SAME_THRESH_XY_M = 0.03   # XY 평면 거리 3 cm
NEAR_SAME_THRESH_Z_M  = 0.01   # Z 축 높이 차 1 cm

CAP_DIR = os.path.expanduser("/home/pc/soomac_ws/src/zeus_vision/zeus_vision/capture_images")
os.makedirs(CAP_DIR, exist_ok=True)

# ---------- 유틸 ----------
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
    if num <= 1: return m
    comps = []
    for i in range(1, num):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < min_area_px: continue
        score = area
        if center_bias is not None:
            cx, cy = center_bias
            cx_i, cy_i = cents[i]
            d2 = (cx_i - cx)**2 + (cy_i - cy)**2
            score = area / (1.0 + 0.001 * d2)
        comps.append((score, i))
    if not comps: return np.zeros_like(m, dtype=np.uint8)
    comps.sort(reverse=True)
    keep_ids = [comps[0][1]] if keep_largest else [i for _, i in comps]
    out = np.zeros_like(m, dtype=np.uint8)
    for i in keep_ids: out[labels == i] = 1
    out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, kernel, iterations=1)
    return out

def keep_largest_component(mask_bin):
    m = (mask_bin > 0).astype(np.uint8)
    num, labels, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    if num <= 1: return m
    areas = stats[1:, cv2.CC_STAT_AREA]
    i = np.argmax(areas) + 1
    return (labels == i).astype(np.uint8)

# 💡 --- 이너 코어 마스크 생성 함수 추가 ---
def create_inner_core_mask(mask_bin, ratio=INNER_CORE_RATIO):
    """
    거리 변환(Distance Transform)을 사용하여 마스크의 안정적인 내부 코어를 추출합니다.
    :param mask_bin: 입력 이진 마스크 (uint8)
    :param ratio: 가장자리로부터의 최대 거리 대비 몇 %까지를 코어로 인정할지
    :return: 내부 코어만 남은 이진 마스크 (uint8)
    """
    if mask_bin is None or not mask_bin.any():
        return np.zeros_like(mask_bin, dtype=np.uint8)

    # 0이 아닌 픽셀에서 가장 가까운 0인 픽셀까지의 거리를 계산
    dist_map = cv2.distanceTransform(mask_bin, cv2.DIST_L2, 5)

    # 최대 거리 값 찾기 (마스크의 가장 두꺼운 부분)
    _, max_val, _, _ = cv2.minMaxLoc(dist_map)

    if max_val <= 0:
        return mask_bin # 매우 얇은 마스크의 경우 원본 반환

    # 최대 거리의 일정 비율을 임계값으로 설정
    threshold = max_val * ratio
    _, core_mask = cv2.threshold(dist_map, threshold, 255, cv2.THRESH_BINARY)
    
    return core_mask.astype(np.uint8)
# ----------------------------------------

# ---------- 핀홀(보정 좌표계, D=0) 기반 변환 ----------
def deproject_points_from_mask_pinhole(mask_bin, depth_undist, intr, depth_scale, stride=PCL_STRIDE):
    fx, fy, cx, cy = intr
    H, W = depth_undist.shape[:2]
    if mask_bin.shape[:2] != (H, W):
        mask_bin = cv2.resize(mask_bin, (W, H), interpolation=cv2.INTER_NEAREST)
    ys, xs = np.where(mask_bin > 0)
    if ys.size == 0: return None, None, None
    xs = xs[::stride]; ys = ys[::stride]
    z_m = depth_undist[ys, xs].astype(np.float32) * depth_scale
    ok = z_m > 1e-6
    if not np.any(ok): return None, None, None
    xs, ys, z_m = xs[ok], ys[ok], z_m[ok]
    x_cam = (xs - cx) * z_m / fx
    y_cam = (ys - cy) * z_m / fy
    pts = np.stack([x_cam, y_cam, z_m], axis=-1)
    return pts, xs, ys

def project_point_to_pixel_pinhole(P, intr):
    fx, fy, cx, cy = intr
    X, Y, Z = P
    if not np.isfinite(Z) or Z <= 1e-9: return None
    u = (X * fx / Z) + cx
    v = (Y * fy / Z) + cy
    return (int(round(u)), int(round(v)))

def draw_cross(img, pt, color=(0,0,255), size=6, thickness=2):
    if pt is None: return
    u, v = pt
    h, w = img.shape[:2]
    if not (0 <= u < w and 0 <= v < h): return
    cv2.line(img, (u-size, v), (u+size, v), color, thickness, cv2.LINE_AA)
    cv2.line(img, (u, v-size), (u, v+size), color, thickness, cv2.LINE_AA)

def euclid_dist_ee(c):
    """EE 기준 3D 유클리디언 거리 계산"""
    o = c.get('origin', None)
    if o is None or not np.all(np.isfinite(o)):
        return float('inf')
    p_ee = o - EE_OFFSET_M  # EE 중심으로 원점 이동
    return float(np.linalg.norm(p_ee))

def xy_dist_cam_ee(c):
    """Z 무시. 카메라좌표계에서 EE 오프셋과 후보의 XY 거리만 비교"""
    o = c.get('origin', None)
    if o is None or not np.all(np.isfinite(o)):
        return float('inf')
    dx = float(o[0] - EE_OFFSET_M[0])
    dy = float(o[1] - EE_OFFSET_M[1])
    return float(math.hypot(dx, dy))

# <<< ADD: XY+Z 동시 판정 유틸 >>>
def is_near_xy_and_z(p: np.ndarray, q: np.ndarray,
                     th_xy: float = NEAR_SAME_THRESH_XY_M,
                     th_z: float  = NEAR_SAME_THRESH_Z_M) -> bool:
    """
    같은 큐브 판정: XY 평면 거리 <= th_xy AND |ΔZ| <= th_z
    p, q: 카메라 좌표계 3D (m)
    """
    if p is None or q is None:
        return False
    if not (np.all(np.isfinite(p)) and np.all(np.isfinite(q))):
        return False
    dx, dy, dz = float(p[0] - q[0]), float(p[1] - q[1]), float(p[2] - q[2])
    xy_ok = math.hypot(dx, dy) <= float(th_xy)
    z_ok  = abs(dz)            <= float(th_z)
    return xy_ok and z_ok

def is_near_any_xy_and_z(p: np.ndarray, lst: list, label: str = None,
                         th_xy: float = NEAR_SAME_THRESH_XY_M,
                         th_z: float  = NEAR_SAME_THRESH_Z_M) -> bool:
    """
    p가 lst 내 어떤 점과도 (XY<=th_xy) AND (|ΔZ|<=th_z)를 만족하면 True.
    label 지정 시 같은 라벨(색상)만 비교.
    lst 원소 예: {'pos': np.ndarray(shape=(3,)), 'label': str}
    """
    if p is None or not lst:
        return False
    for item in lst:
        pos = item.get('pos', None)
        lab = item.get('label', None)
        if (label is None) or (lab == label):
            if is_near_xy_and_z(p, pos, th_xy=th_xy, th_z=th_z):
                return True
    return False

# ---------- 평면/좌표계 ----------
def segment_plane_and_axes(pts):
    if not O3D_OK or pts is None or pts.shape[0] < 50: return [None]*6
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=STAT_NB_NEIGHBORS, std_ratio=STAT_STD_RATIO)
    pcd, _ = pcd.remove_radius_outlier(nb_points=RAD_MIN_POINTS, radius=RAD_RADIUS)
    if len(pcd.points) < 50: return [None]*6
    plane_model, inliers_idx = pcd.segment_plane(distance_threshold=PLANE_DIST_BASE,
                                                 ransac_n=PLANE_RANSAC_N,
                                                 num_iterations=PLANE_ITERS)
    if len(inliers_idx) < 50: return [None]*6
    inlier_cloud = pcd.select_by_index(inliers_idx)

    a,b,c,d = plane_model
    z_axis = np.array([a,b,c], np.float32)
    if z_axis[2] < 0: z_axis = -z_axis
    z_axis /= (np.linalg.norm(z_axis) + 1e-9)

    origin = inlier_cloud.get_center()
    _, cov = inlier_cloud.compute_mean_and_covariance()
    evals, evecs = np.linalg.eigh(cov)
    axes = evecs.T[np.argsort(evals)]
    x_axis = axes[2]
    ex = np.array([1.0,0.0,0.0], np.float32)
    r_x = ex - z_axis * float(ex @ z_axis)
    if np.linalg.norm(r_x) > 1e-9 and float(x_axis @ r_x) < 0: x_axis = -x_axis
    x_axis = x_axis / (np.linalg.norm(x_axis) + 1e-9)
    y_axis = np.cross(z_axis, x_axis); y_axis /= (np.linalg.norm(y_axis)+1e-9)
    return x_axis.astype(np.float32), y_axis.astype(np.float32), z_axis.astype(np.float32), \
           np.asarray(origin, np.float32), np.asarray(inlier_cloud.points, np.float32), (a,b,c,d)

def plane_inlier_mask_from_model_pinhole(mask_bin, depth_undist, intr, depth_scale, plane_model,
                                         dist_th=PLANE_DIST_BASE*1.5, stride=1):
    if plane_model is None: return np.zeros_like(mask_bin, dtype=np.uint8)
    fx, fy, cx, cy = intr
    a,b,c,d = plane_model
    H, W = depth_undist.shape[:2]
    if mask_bin.shape[:2] != (H,W):
        mask_bin = cv2.resize(mask_bin, (W,H), interpolation=cv2.INTER_NEAREST)
    ys, xs = np.where(mask_bin > 0)
    if ys.size == 0: return np.zeros((H,W), dtype=np.uint8)
    ys = ys[::max(1,stride)]; xs = xs[::max(1,stride)]
    zs = depth_undist[ys, xs].astype(np.float32) * depth_scale
    valid = zs > 1e-6
    if not np.any(valid): return np.zeros((H,W), dtype=np.uint8)
    ys, xs, zs = ys[valid], xs[valid], zs[valid]
    x_cam = (xs - cx) * zs / fx
    y_cam = (ys - cy) * zs / fy
    pts = np.stack([x_cam, y_cam, zs], axis=-1)
    numer = np.abs(a*pts[:,0] + b*pts[:,1] + c*pts[:,2] + d)
    denom = (a*a + b*b + c*c)**0.5 + 1e-9
    dist = numer / denom
    inliers = dist <= dist_th
    out = np.zeros((H,W), dtype=np.uint8)
    out[ys[inliers], xs[inliers]] = 1
    return out

def ray_plane_intersect_pinhole(u, v, intr, plane_model):
    if plane_model is None: return None
    fx, fy, cx, cy = intr
    a,b,c,d = plane_model
    vx = (u - cx) / fx
    vy = (v - cy) / fy
    vz = 1.0
    denom = a*vx + b*vy + c*vz
    if abs(denom) < 1e-9: return None
    t = -d / denom
    if t <= 0: return None
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

def get_3d_center_from_2d_pixel_pinhole(rect_center_px, depth_undist, intr, depth_scale, plane_model):
    u, v = rect_center_px
    pt_3d = ray_plane_intersect_pinhole(u, v, intr, plane_model)
    if pt_3d is not None: return pt_3d.astype(np.float32), "A-ray-plane"
    H, W = depth_undist.shape[:2]
    u_i, v_i = int(round(u)), int(round(v))
    x0, x1 = max(0, u_i-2), min(W-1, u_i+2)
    y0, y1 = max(0, v_i-2), min(H-1, v_i+2)
    patch = depth_undist[y0:y1+1, x0:x1+1].astype(np.float32) * depth_scale
    patch = patch[patch > 1e-6]
    if patch.size > 0:
        fx, fy, cx, cy = intr
        z_med = float(np.mean(patch))
        X = (u - cx) * z_med / fx
        Y = (v - cy) * z_med / fy
        Z = z_med
        return np.array([X, Y, Z], dtype=np.float32), "B-depth-median"
    return None, None

def refine_center_minarearect(mask_bin, depth_undist, intr, depth_scale, plane_model):
    if mask_bin is None or not mask_bin.any():
        return None, None, None, None, None, None, None, None

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    processed_mask = cv2.morphologyEx(mask_bin.astype(np.uint8), cv2.MORPH_CLOSE, kernel, iterations=2)
    
    cnts, _ = cv2.findContours(processed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not cnts:
        return None, None, None, None, None, None, None, None

    cnt = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(cnt) < MIN_AREA_PX:
        return None, None, None, None, None, None, None, None

    hull = cv2.convexHull(cnt)
    rect = cv2.minAreaRect(hull) 
    (cx, cy), _, _ = rect
    rect_orig_box_pts = cv2.boxPoints(rect).astype(np.int32)

    refined_origin, origin_src = get_3d_center_from_2d_pixel_pinhole(
        (cx, cy), depth_undist, intr, depth_scale, plane_model
    )
    ys, xs = np.where(processed_mask > 0)
    z_vals = depth_undist[ys, xs].astype(np.float32) * depth_scale
    z_vals = z_vals[z_vals > 1e-6]
    z_med = float(np.median(z_vals)) if z_vals.size else None
    return refined_origin, (cx, cy), z_med, None, None, rect_orig_box_pts, origin_src, None

# ---------- HSV 라벨 보정 ----------
COLOR_NORMALIZE_SET = {"red", "pink", "purple", "green"}
def refine_label_by_hsv_mean(initial_label: str, mask_bin: np.ndarray, hsv_img: np.ndarray):
    if initial_label is None: return None, None, None, None
    lab = str(initial_label).lower()
    if lab not in COLOR_NORMALIZE_SET: return initial_label, None, None, None
    ys, xs = np.where(mask_bin > 0)
    if ys.size == 0: return initial_label, None, None, None
    hsv_vals = hsv_img[ys, xs].astype(np.float32)
    Hm = float(np.mean(hsv_vals[:,0])); Sm = float(np.mean(hsv_vals[:,1])); Vm = float(np.mean(hsv_vals[:,2]))
    if lab in ("red","pink"):    lab = "pink" if Sm < 170.0 else "red"
    elif lab in ("purple","green"): lab = "purple" if Hm > 100.0 else "green"
    return lab, Hm, Sm, Vm

# =================== BB 면적/사이즈 유틸 =========================
def area_mm2_from_rect_on_plane(rect_box_pts, intr, plane_model):
    if rect_box_pts is None or plane_model is None: return 0.0
    corners_3d = []
    for (u, v) in rect_box_pts.astype(np.float32):
        p = ray_plane_intersect_pinhole(float(u), float(v), intr, plane_model)
        if p is None or not np.all(np.isfinite(p)): return 0.0
        corners_3d.append(p)
    if len(corners_3d) != 4: return 0.0
    p0, p1, p2, p3 = [np.asarray(q, dtype=np.float32) for q in corners_3d]
    a1 = 0.5 * np.linalg.norm(np.cross(p1 - p0, p2 - p0))
    a2 = 0.5 * np.linalg.norm(np.cross(p3 - p0, p2 - p0))
    area_m2 = float(a1 + a2)
    return area_m2 * 1e6

def size_mm_from_rect_on_plane(rect_box_pts, intr, plane_model):
    if rect_box_pts is None or plane_model is None: return 0.0, 0.0
    corners_3d = []
    # print("Rect Box Points (px):", rect_box_pts)
    for (u, v) in rect_box_pts.astype(np.float32):
        p = ray_plane_intersect_pinhole(float(u), float(v), intr, plane_model)
        if p is None or not np.all(np.isfinite(p)): return 0.0, 0.0
        corners_3d.append(p.astype(np.float32))
    if len(corners_3d) != 4: return 0.0, 0.0
    p0, p1, p2, p3 = corners_3d
    #print(f"3D Box Corners (m): {p0}, {p1}, {p2}, {p3}")
    e01 = float(np.linalg.norm(p1 - p0)) * 1000.0
    e12 = float(np.linalg.norm(p2 - p1)) * 1000.0
    w_mm = min(e01, e12)
    h_mm = max(e01, e12)
    return w_mm, h_mm

def draw_axes(img, origin_px, length=60, thickness=2):
    """이미지 상의 origin_px (u,v)에 2D X/Y 축을 그린다."""
    if origin_px is None: 
        return
    u, v = int(origin_px[0]), int(origin_px[1])
    h, w = img.shape[:2]
    # X축: 좌/우
    pt_x1 = (max(0, u - length), v)
    pt_x2 = (min(w-1, u + length), v)
    cv2.arrowedLine(img, (u, v), pt_x2, (0,255,0), thickness, tipLength=0.15)     # +X (초록)
    cv2.arrowedLine(img, (u, v), pt_x1, (0,155,0), thickness, tipLength=0.15)     # -X
    # Y축: 상/하 (이미지 좌표계 기준)
    pt_y1 = (u, max(0, v - length))
    pt_y2 = (u, min(h-1, v + length))
    cv2.arrowedLine(img, (u, v), pt_y1, (255,0,255), thickness, tipLength=0.15)   # +Y (보라, 위쪽)
    cv2.arrowedLine(img, (u, v), pt_y2, (155,0,155), thickness, tipLength=0.15)   # -Y
    cv2.circle(img, (u, v), 4, (0,0,0), -1, cv2.LINE_AA)
    cv2.circle(img, (u, v), 2, (255,255,255), -1, cv2.LINE_AA)

# =================================================================

# ---------- ROS2 ----------
class BlockPosePublisher(Node):
    def __init__(self):
        super().__init__('block_pose_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/zeus/array/block_pose', 10)
        self.pose_pubisher = self.create_publisher(Float32MultiArray, '/zeus/rpy/block_pose', 10)
        self.block_pubisher = self.create_publisher(String, '/zeus/string/block_color', 10)
        self.subscriber = self.create_subscription(String, '/zeus/string/block_order', self.listener_callback, 10)
        self.detect_signal = ""

        self.model = YOLO(WEIGHTS)
        self.get_logger().info(f"YOLO Model Loaded. Classes: {self.model.names}")

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, COLOR_FPS)
        config.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, DEPTH_FPS)
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        
        self.dec_filter  = rs.decimation_filter()     # 선택
        self.spa_filter  = rs.spatial_filter()
        self.tmp_filter  = rs.temporal_filter()
        self.hole_fill   = rs.hole_filling_filter(2)  # 0~2
        K = CAMERA_MATRIX; D = DIST_COEFFS

        self.blue_num = 0
        self.green_num = 0
        self.pink_num = 0
        self.purple_num = 0
        self.red_num = 0
        self.yellow_num = 0

        self.class_order = ['blue', 'green', 'pink', 'purple', 'red', 'yellow']
        # color_vsp  = profile.get_stream(rs.stream.color).as_video_stream_profile()
        # color_intr = color_vsp.get_intrinsics()  # width, height, ppx, ppy, fx, fy, model, coeffs

        # # K, D 생성
        # K = np.array([[color_intr.fx, 0.0,            color_intr.ppx],
        #             [0.0,            color_intr.fy, color_intr.ppy],
        #             [0.0,            0.0,           1.0         ]], dtype=np.float32)

        # # 왜곡모델: 대부분 BrownConrady(= rs.distortion.brown_conrady)
        # # coeffs 길이는 5 또는 그 이상일 수 있음. undistort는 k1..k3, p1, p2만 사용.
        # if color_intr.model == rs.distortion.none:
        #     D = np.zeros((1,5), dtype=np.float32)
        # else:
        #     D = np.array([list(color_intr.coeffs[:5])], dtype=np.float32)

        self.K_rect, _ = cv2.getOptimalNewCameraMatrix(K, D, (COLOR_W, COLOR_H), 0, (COLOR_W, COLOR_H))
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            K, D, None, self.K_rect, (COLOR_W, COLOR_H), cv2.CV_32FC1
        )
        fx, fy, cx, cy = self.K_rect[0,0], self.K_rect[1,1], self.K_rect[0,2], self.K_rect[1,2]
        self.rect_intr = (fx, fy, cx, cy)
        self.get_logger().info(f"[Rectified K] fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")

        # --- EE 전용 UI 창 준비 ---
        try:
            cv2.namedWindow("EE Distance View", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("EE Distance View", 480, 360)
        except Exception:
            pass  # GUI 미지원 환경 대비
        self.ee_idle_canvas = np.full((240, 360, 3), 30, dtype=np.uint8)
        self.last_plane_depth_m = 0.5  # 평면 Z 기본값(미탐지 시 0.5m 가정)

        # ------------------------

        self.last_quat = None
        self.last_center = None
        self.stable_count = 0
        self.cooldown = 0

        self.last_block1_color = None

        # <<< ADD: 좌표 기반 배제 로직 상태 >>>
        self.last_block1_coord_m = None   # np.ndarray shape=(3,)
        self.last_block1_label   = None   # str
        self.avoid_points        = []     # [{'pos': np.ndarray(3,), 'label': str}, ...]

    def listener_callback(self, msg):
        self.detect_signal = msg.data

    def publish_block(self, label):
        msg = String(); msg.data = label
        self.block_pubisher.publish(msg)

# ---------- 메인 ----------
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
                frameset = node.pipeline.wait_for_frames(timeout_ms=1000)
                aligned_frames = node.align.process(frameset)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if not color_frame or not depth_frame: 
                    continue
            except RuntimeError:
                continue

            color_dist = np.asanyarray(color_frame.get_data())
            depth_dist = np.asanyarray(depth_frame.get_data())

            color = cv2.remap(color_dist, node.map1, node.map2, interpolation=cv2.INTER_LINEAR)
            depth = cv2.remap(depth_dist, node.map1, node.map2, interpolation=cv2.INTER_NEAREST)
            #depth_frame = node.dec_filter.process(depth_frame)
            depth_frame = node.spa_filter.process(depth_frame)
            #depth_frame = node.tmp_filter.process(depth_frame)
            #depth_frame = node.hole_fill.process(depth_frame)
            depth_dist  = np.asanyarray(depth_frame.get_data())
            
            
            hsv_image = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
            overlay = color.copy()
            plane_vis = (color * 0.3).astype(np.uint8)
            plane_accum_mask = np.zeros((COLOR_H, COLOR_W), dtype=np.uint8)
            inner_core_accum_mask = np.zeros((COLOR_H, COLOR_W), dtype=np.uint8)

            res = None
            if node.detect_signal == 'block1' or node.detect_signal == 'block2':
                print(f"[INFO] Detect Order: {node.class_order}")
                res = node.model(color, conf=CONF_DET, iou=IOU_TH, device=DEVICE, imgsz=IMG_SIZE, verbose=False)
            # ==========================
            candidates = []
            chosen = None

            if res and res[0].boxes is not None and len(res[0].boxes) > 0:
                r = res[0]
                has_masks = (r.masks is not None) and (r.masks.data is not None) and (len(r.masks.data) == len(r.boxes))
                confs = r.boxes.conf.detach().cpu().numpy().reshape(-1)
                xyxy  = r.boxes.xyxy.detach().cpu().numpy()
                clss  = r.boxes.cls.detach().cpu().numpy().astype(int) if r.boxes.cls is not None else np.zeros_like(confs, dtype=int)
                names = getattr(r, "names", None)
                
                # 1) confidence 1차 컷
                valid_idx = np.where(confs >= CONF_PUB)[0]

                # 이름→id 매핑(안전)
                name2id = {}
                if names is not None:
                    for i, nm in enumerate(names):
                        name2id[str(nm).lower()] = i

                # 2) 클래스 우선순위부터 적용 → 해당 클래스에 속하는 디텍션만 전처리 실행
                mode = (node.detect_signal or "").strip().lower()
                CLASS_ORDER = node.class_order.copy()

                if mode == 'block2' and node.last_block1_color:
                    try:
                        # 리스트에 있다면 제거 후 맨 앞에 삽입(중복 방지)
                        CLASS_ORDER.remove(node.last_block1_color)
                        CLASS_ORDER.insert(0, node.last_block1_color)
                    except ValueError:
                        # last_block1_color가 현재 class_order에 없으면(이미 quota로 제거된 경우 등) 무시
                        pass

                for cls_name in CLASS_ORDER:
                    cls_id = None
                    if names is not None and cls_name in name2id:
                        cls_id = name2id[cls_name]
                    else:
                        # names가 없거나 매핑 실패 시, 문자열 비교로 보정
                        # (YOLO가 라벨명을 안 담고 있을 일은 드묾)
                        pass

                    # 현재 우선순위 클래스에 해당하는 디텍션 인덱스만 추출
                    idx_cls = []
                    for i in valid_idx:
                        if cls_id is not None:
                            if clss[i] == cls_id:
                                idx_cls.append(i)
                        else:
                            # names 매핑 실패 대비: 라벨 문자열 비교 (가급적 도달하지 않음)
                            lab = names[clss[i]] if (names is not None and clss[i] < len(names)) else f"id{clss[i]}"
                            if str(lab).lower() == cls_name:
                                idx_cls.append(i)

                    if not idx_cls:
                        continue  # 이 클래스에 후보 없음 → 다음 클래스

                    # 이 클래스 후보들만 기존 "블럭 전처리" 그대로 수행
                    candidates_cls = []
                    for i in idx_cls:
                        conf_i = float(confs[i])
                        x1, y1, x2, y2 = xyxy[i].astype(int)
                        label_raw = names[clss[i]] if (names is not None and clss[i] < len(names)) else f"id{clss[i]}"
                        #print(f"[DEBUG] Processing Detection: Label={label_raw}, Conf={conf_i:.3f}, BBox=({x1},{y1},{x2},{y2})")
                        # (전처리 1) 마스크 확보 (원본 로직 유지)
                        if has_masks:
                            mask = r.masks.data[i].detach().cpu().numpy()
                            mask_bin = (mask > 0.5).astype(np.uint8)
                        else:
                            mask_bin = np.zeros((COLOR_H, COLOR_W), dtype=np.uint8)
                            mask_bin[max(y1,0):min(y2,COLOR_H), max(x1,0):min(x2,COLOR_W)] = 1

                        # (전처리 2) 연결요소 정리 (원본 유지)
                        ys0, xs0 = np.where(mask_bin > 0)
                        cx0, cy0 = (int(xs0.mean()), int(ys0.mean())) if xs0.size>0 else (int((x1+x2)//2), int((y1+y2)//2))
                        mask_bin = connected_components_keep(mask_bin, min_area_px=MIN_AREA_PX,
                                                             keep_largest=True, center_bias=(cx0, cy0))
                        if not mask_bin.any():
                            continue

                        # (전처리 3) HSV 라벨 보정 (원본 유지)
                        label_norm, Hm, Sm, Vm = refine_label_by_hsv_mean(label_raw, mask_bin, hsv_image)

    
                        label = label_norm if label_norm is not None else label_raw
                        
                        if label not in node.class_order:
                            #print(f"[WARNING] Refined label '{label}' not in class order list.")
                            continue
                        
                        # 시각화(원본 유지)
                        overlay = apply_mask_overlay(overlay, mask_bin, color=(0,255,255), alpha=0.35)
                        cv2.rectangle(overlay, (x1,y1), (x2,y2), (0,0,0), 2)

                        # (전처리 4) 평면 추정 및 인라이어 검사 (원본 유지)
                        pts_mask, _, _ = deproject_points_from_mask_pinhole(mask_bin, depth, node.rect_intr, node.depth_scale, stride=PCL_STRIDE)
                        x_axis, y_axis, z_axis, origin_m, inliers, plane_model = segment_plane_and_axes(pts_mask)
                        if origin_m is None:
                            continue

                        ASSUMED_DISTANCE_M = np.mean(inliers[:, 2]) if inliers is not None and inliers.shape[0] > 0 else 0.5
                        # node.last_plane_depth_m = float(ASSUMED_DISTANCE_M)  # 최근 평면 깊이(카메라 Z) 업데이트
                        plane_model_flat = (0.0, 0.0, 1.0, -ASSUMED_DISTANCE_M)

                        plane_inlier_mask = plane_inlier_mask_from_model_pinhole(mask_bin, depth, node.rect_intr, node.depth_scale, plane_model, stride=1)
                        plane_inlier_mask = keep_largest_component(plane_inlier_mask)
                        ratio = float(plane_inlier_mask.sum()) / (float(mask_bin.sum()) + 1e-9)
                        if ratio < INLIER_RATIO_TH:
                            continue

                        if plane_inlier_mask.any():
                            plane_accum_mask = np.clip(plane_accum_mask + plane_inlier_mask, 0, 1)

                        # (전처리 5) inner core (원본 유지)
                        inner_core_mask = create_inner_core_mask(plane_inlier_mask, ratio=INNER_CORE_RATIO)
                        if inner_core_mask.any():
                            inner_core_accum_mask = np.clip(inner_core_accum_mask + inner_core_mask, 0, 1)
                            pts_core, _, _ = deproject_points_from_mask_pinhole(inner_core_mask, depth, node.rect_intr, node.depth_scale, stride=PCL_STRIDE)
                            x_axis_stable, y_axis_stable, z_axis_stable, _, _, _ = segment_plane_and_axes(pts_core)
                            if x_axis_stable is not None:
                                x_axis, y_axis, z_axis = x_axis_stable, y_axis_stable, z_axis_stable

                        # (전처리 6) minAreaRect 및 중심/박스 코너 (원본 유지)
                        refined_origin, rect_center_px, z_med, _, _, rect_orig_box_pts, origin_src, _ = \
                            refine_center_minarearect(
                                inner_core_mask, depth, node.rect_intr, node.depth_scale, plane_model
                            )

                        if rect_orig_box_pts is not None:
                            cv2.polylines(overlay, [rect_orig_box_pts], True, (255,255,255), 1, cv2.LINE_AA)
                        if rect_center_px is not None:
                            cv2.circle(overlay, (int(rect_center_px[0]), int(rect_center_px[1])), 3, (0,255,255), -1, cv2.LINE_AA)

                        if refined_origin is not None:
                            origin_m = refined_origin

                        origin_px = project_point_to_pixel_pinhole(origin_m, node.rect_intr)
                        draw_cross(overlay, origin_px, color=(255,255,0), size=5, thickness=2)

                        # (전처리 7) 면적/사이즈 계산 및 컷(원본 유지)
                        area_mm2 = area_mm2_from_rect_on_plane(rect_orig_box_pts, node.rect_intr, plane_model)
                        area_txt = f"area_mm2:{int(round(area_mm2))}"
                        cv2.putText(overlay, area_txt, (x1, max(0, y1 - 24)), FONT, 0.6, (255,255,255), 3, cv2.LINE_AA)
                        cv2.putText(overlay, area_txt, (x1, max(0, y1 - 24)), FONT, 0.6, (0,0,255), 1, cv2.LINE_AA)

                        w_mm, h_mm = size_mm_from_rect_on_plane(rect_orig_box_pts, node.rect_intr, plane_model)
                        w_flat_mm, h_flat_mm = size_mm_from_rect_on_plane(rect_orig_box_pts, node.rect_intr, plane_model_flat)

                
                        print(f"w_flat_mm={w_flat_mm:.1f} h_mm={h_flat_mm:.1f}")
                        if not (w_flat_mm >= 20.0 and h_mm >= 70.0):
                            # (디버깅 시각화 유지)
                            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 255), 2)
                            rejection_text = f"REJECTED: w_flat_mm={w_flat_mm:.1f} h_mm={h_mm:.1f}"
                            cv2.putText(overlay, rejection_text, (x1, y2 + 15), FONT, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                            continue
                        
                        # 후보 등록(원본 필드 유지)
                        candidates_cls.append(dict(
                            label=(label if label is not None else label_raw),
                            conf=conf_i, box=(x1,y1,x2,y2),
                            origin=origin_m, axes=(x_axis, y_axis, z_axis),
                            inliers=inliers, plane_model=plane_model, cx=cx0, cy=cy0,
                            src=origin_src or "-", hsv=(Hm, Sm, Vm),
                            area_mm2=float(area_mm2),
                            rect_center_px=(None if rect_center_px is None else (float(rect_center_px[0]), float(rect_center_px[1])))
                        ))

                    # 이 클래스에서 면적 하한 통과만 추려 최종 선택
                    #cand_sel = [c for c in candidates_cls if area_mm2_of(c) >= AREA_THRESHOLD_MM2]
                    # <<< REPLACE: 후보 선택 로직 (배제 리스트 + XY&Z 동시 판정 + 2·3순위 선택) >>>
                    # 1) 1차 후보 집합
                    cand_sel = [c for c in candidates_cls]

                    # 2) 영구 배제 리스트(avoid_points) 기반 제거 (같은 라벨 & XY<=3cm & |ΔZ|<=1cm 인 것 제외)
                    cand_sel = [
                        c for c in cand_sel
                        if not is_near_any_xy_and_z(
                            np.asarray(c.get('origin', None), dtype=np.float32),
                            node.avoid_points,
                            label=(c.get('label', None)),
                            th_xy=NEAR_SAME_THRESH_XY_M,
                            th_z=NEAR_SAME_THRESH_Z_M
                        )
                    ]

                    if cand_sel:
                        if mode == 'block1':
                            # 3) 직전 block1 대상과 "같은 큐브"(XY<=3cm & |ΔZ|<=1cm & 같은 라벨) 후보만 제외 → 2·3순위 허용
                            def is_same_as_last(c):
                                if (node.last_block1_coord_m is None) or (node.last_block1_label is None):
                                    return False
                                cur_o   = c.get('origin', None)
                                cur_lab = c.get('label', None)
                                if (cur_o is None) or (cur_lab is None):
                                    return False
                                if cur_lab != node.last_block1_label:
                                    return False
                                return is_near_xy_and_z(
                                    np.asarray(cur_o, dtype=np.float32),
                                    node.last_block1_coord_m,
                                    th_xy=NEAR_SAME_THRESH_XY_M,
                                    th_z=NEAR_SAME_THRESH_Z_M
                                )

                            cand_sel2 = [c for c in cand_sel if not is_same_as_last(c)]

                            # 4) 모두 제외되면(= 이 클래스에서 선택 불가) → best를 영구 배제 리스트에 등록하고 다음 클래스로 이동
                            if not cand_sel2:
                                def xy_dist_cam_tmp(c):
                                    o = c.get('origin', None)
                                    if o is None or not np.all(np.isfinite(o)): return float('inf')
                                    return math.hypot(float(o[0]), float(o[1]))
                                best_tmp = min(cand_sel, key=xy_dist_cam_tmp)
                                cur_o    = np.asarray(best_tmp.get('origin', None), dtype=np.float32)
                                cur_lab  = best_tmp.get('label', None)
                                if (cur_o is not None) and (cur_lab is not None):
                                    if not is_near_any_xy_and_z(cur_o, node.avoid_points,
                                                                label=cur_lab,
                                                                th_xy=NEAR_SAME_THRESH_XY_M,
                                                                th_z=NEAR_SAME_THRESH_Z_M):
                                        node.avoid_points.append({'pos': cur_o.copy(), 'label': cur_lab})
                                continue  # 이 클래스 스킵 → 다음 우선순위 클래스

                            # 5) 후보가 남으면(= 2번째, 3번째 …) 기존 기준으로 최단 XY 거리 선택
                            def xy_dist_cam(c):
                                o = c.get('origin', None)
                                if o is None or not np.all(np.isfinite(o)): return float('inf')
                                return math.hypot(float(o[0]), float(o[1]))
                            chosen = min(cand_sel2, key=xy_dist_cam)

                        elif mode == 'block2':
                            # block2는 EE 기준 최소 거리 유지
                            chosen = min(cand_sel, key=xy_dist_cam_ee)

                        # 클래스 하나에서 선택 끝 → 바깥 루프 종료
                        candidates = cand_sel  # (옵션) 디버깅용 참조
                        break

            # 누적 시각화(원본 유지)
            if plane_accum_mask.any():
                plane_vis = apply_mask_overlay(plane_vis, plane_accum_mask, color=(0,255,0), alpha=0.6)
            if inner_core_accum_mask.any():
                plane_vis = apply_mask_overlay(plane_vis, inner_core_accum_mask, color=(0,0,255), alpha=0.8)

            # 이후 chosen 처리(원본 로직 그대로 유지)
            if chosen is not None:
                x_axis, y_axis, z_axis = chosen['axes']
                origin_m = chosen['origin']
                label = chosen.get('label', None)

                if mode == 'block1':
                    if label == "blue":   
                        node.blue_num  += 1

                    elif label == "green":  
                        node.green_num += 1

                    elif label == "pink":   
                        node.pink_num  += 1

                    elif label == "purple": 
                        node.purple_num+= 1

                    elif label == "red":    
                        node.red_num   += 1

                    elif label == "yellow": 
                        node.yellow_num+= 1

                elif mode == 'block2':
                    if node.blue_num >= 6:
                        node.class_order = [c for c in node.class_order if c != "blue"]
 
                    if node.green_num >= 6:
                            node.class_order = [c for c in node.class_order if c != "green"] 
                    
                    if node.pink_num >= 6:
                            node.class_order = [c for c in node.class_order if c != "pink"]
                    
                    if node.purple_num >= 6:
                            node.class_order = [c for c in node.class_order if c != "purple"]

                    if node.red_num >= 2:
                            node.class_order = [c for c in node.class_order if c != "red"]
                    
                    if node.yellow_num >= 5:
                            node.class_order = [c for c in node.class_order if c != "yellow"] 
                    
                    print(f"blue: {node.blue_num}, green: {node.green_num}, pink: {node.pink_num}, purple: {node.purple_num}, red: {node.red_num}, yellow: {node.yellow_num}")

                final_px = project_point_to_pixel_pinhole(origin_m, node.rect_intr)
                draw_cross(overlay, final_px, color=(0,0,255), size=7, thickness=2)

                if all(v is not None for v in [x_axis, y_axis, z_axis]):
                    raw_rot_matrix = np.stack([x_axis, y_axis, z_axis], axis=1)
                    raw_rotation = SciRot.from_matrix(raw_rot_matrix)
                    yaw, pitch, roll = raw_rotation.as_euler('zyx', degrees=True)

                    # is_roll_high  = abs(roll)  >= DEAD_ZONE_DEG
                    # is_pitch_high = abs(pitch) >= DEAD_ZONE_DEG
                    # if is_roll_high and is_pitch_high:
                    #     if abs(roll) >= abs(pitch): pitch = 0.0
                    #     else: roll = 0.0
                    # else:
                    #     if not is_pitch_high: pitch = 0.0
                    #     if not is_roll_high:  roll  = 0.0

                    final_rotation = SciRot.from_euler('zyx', [yaw, pitch, roll], degrees=True)
                    final_rot_matrix = final_rotation.as_matrix()
                    final_quat = final_rotation.as_quat()
                    final_rpy = [roll, pitch, yaw]
                else:
                    final_rot_matrix = np.eye(3, dtype=np.float64)
                    final_quat = SciRot.from_matrix(final_rot_matrix).as_quat()
                    final_rpy = [0.0, 0.0, 0.0]

                origin_mm = origin_m * 1000.0
                final_matrix = np.eye(4, dtype=np.float64)
                final_matrix[:3,:3] = final_rot_matrix
                final_matrix[:3, 3] = origin_mm
                #print(f"origin_mm: {origin_mm}")

                def quat_delta_deg(q1, q2):
                    if q1 is None or q2 is None: return np.inf
                    dot = np.clip(np.abs(float(np.dot(q1, q2))), -1.0, 1.0)
                    return 2.0 * np.degrees(np.arccos(dot))

                # 안정화/쿨다운(원본 유지)
                cx, cy = chosen['cx'], chosen['cy']
                center_jump = 0.0 if node.last_center is None else math.hypot(cx-node.last_center[0], cy-node.last_center[1])
                dtheta = quat_delta_deg(node.last_quat, final_quat)
                same_target = (center_jump <= STAB_CENTER_JUMP_PX)
                if not same_target:
                    node.stable_count = 0
                elif dtheta < STAB_ANGLE_DEG:
                    node.stable_count += 1
                else:
                    node.stable_count = 0
                node.cooldown = max(0, node.cooldown - 1)
                node.last_quat = final_quat
                node.last_center = (cx, cy)

                if z_axis is not None:
                    cam_normal = np.array([0.0, 0.0, 1.0], dtype=np.float32)
                    zn = z_axis / (np.linalg.norm(z_axis) + 1e-9)
                    dot = float(np.clip(np.dot(zn, cam_normal), -1.0, 1.0))
                    angle_deg = float(np.degrees(np.arccos(dot))) 
                    final_rpy[1] = abs(angle_deg) # pitch 각도를 절대값으로 설정
                #     angle_txt = f"degree: {angle_deg:.1f}"
                #     cv2.putText(overlay, angle_txt, (x1, max(0, y1 - 40)), FONT, 0.6, (255,255,255), 3, cv2.LINE_AA)
                #     cv2.putText(overlay, angle_txt, (x1, max(0, y1 - 40)), FONT, 0.6, (255, 0, 0), 1, cv2.LINE_AA)
                # print(f"[Chosen] Label={label}, Origin(m)={origin_m}, RollPitchYaw(deg)=({final_rpy[0]:.1f}, {final_rpy[1]:.1f}, {final_rpy[2]:.1f})")

                # 퍼블리시(원본 유지)
                if node.detect_signal != "":
                    msg = Float32MultiArray()
                    rows, cols = 4, 4
                    msg.layout.dim.append(MultiArrayDimension(label='rows', size=rows, stride=cols))
                    msg.layout.dim.append(MultiArrayDimension(label='cols', size=cols, stride=1))
                    msg.data = final_matrix.flatten().astype(np.float32).tolist()
                    node.publisher_.publish(msg)
                    if node.detect_signal == "block1":
                        print(f"block1_color : {label}")
                    if node.detect_signal == "block2":
                        print(f"block2_color : {label}")
                        print("############################")
                    
                    if node.detect_signal == "block1":
                        msg2 = Float32MultiArray()
                        msg2.data = [final_rpy[0], final_rpy[1], final_rpy[2]]
                        node.pose_pubisher.publish(msg2)
                        node.publish_block(label if label is not None else "")
                        node.last_block1_color = (label if label is not None else None)
                        # <<< ADD: block1 발행 후 마지막 좌표/라벨 갱신 >>>
                        node.last_block1_coord_m = (origin_m.copy() if (origin_m is not None) else None)
                        node.last_block1_label   = (label if label is not None else None)

                    node.cooldown = PUBLISH_COOLDOWN_FRAMES
                    node.stable_count = 0
                    node.detect_signal = ""

                if final_px is not None:
                    info_key = "min XY" if mode == 'block1' else "min EE dist"
                    info1 = f"CHOSEN (class→area≥thr→{info_key})"
                    cv2.putText(overlay, info1, (max(0, final_px[0]-160), min(COLOR_H-8, final_px[1]+18)), FONT, 0.50, (255,255,255), 3, cv2.LINE_AA)
                    cv2.putText(overlay, info1, (max(0, final_px[0]-160), min(COLOR_H-8, final_px[1]+18)), FONT, 0.50, (0,0,255), 1, cv2.LINE_AA)

            # ==========================
            # EE 전용 UI 렌더링 (항상 표시)
            # ==========================
            try:
                ee_view = color.copy()

                # (a) EE 오프셋 적용 원점 픽셀 위치 계산: [x_off, y_off, 최근 평면 Z]
                ee_P = np.array([EE_OFFSET_M[0], EE_OFFSET_M[1], node.last_plane_depth_m], dtype=np.float32)
                ee_px = project_point_to_pixel_pinhole(ee_P, node.rect_intr)
                if ee_px is not None:
                    draw_axes(ee_view, ee_px, length=60, thickness=2)
                    cv2.putText(ee_view, "EE (0,0)", (ee_px[0]+8, ee_px[1]-8), FONT, 0.5, (255,255,255), 2, cv2.LINE_AA)
                    cv2.putText(ee_view, "EE (0,0)", (ee_px[0]+8, ee_px[1]-8), FONT, 0.5, (0,0,0), 1, cv2.LINE_AA)

                # (b) 후보들: 파란 점(크게)
                if candidates:
                    for c in candidates:
                        pt = c.get('rect_center_px', None)
                        if pt is None:
                            pt = project_point_to_pixel_pinhole(c.get('origin', None), node.rect_intr)
                        if pt is None:
                            continue
                        u, v = int(round(pt[0])), int(round(pt[1]))
                        cv2.circle(ee_view, (u, v), 9, (255, 0, 0), -1, cv2.LINE_AA)

                # (c) 최종 선택: 빨간 점(더 크게)
                if chosen is not None:
                    pt_sel = chosen.get('rect_center_px', None)
                    if pt_sel is None:
                        pt_sel = project_point_to_pixel_pinhole(chosen.get('origin', None), node.rect_intr)
                    if pt_sel is not None:
                        u, v = int(round(pt_sel[0])), int(round(pt_sel[1]))
                        cv2.circle(ee_view, (u, v), 11, (0, 0, 255), -1, cv2.LINE_AA)

                cv2.imshow("EE Distance View", ee_view)

            except Exception:
                pass

            # ==========================
            # 교체 끝
            # ==========================

            cv2.imshow("Detections (mask+box + centers)", overlay)
            cv2.imshow("Plane (Green) & InnerCore (Red)", plane_vis)
            # cv2.waitKey(1)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                ts = time.strftime("%Y%m%d_%H%M%S")
                path = os.path.join(CAP_DIR, f"{ts}_color.png")
                cv2.imwrite(path, color)
                print(f"[CAPTURE] {path}")
                

    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()