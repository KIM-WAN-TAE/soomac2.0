# -*- coding: utf-8 -*-

import os
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import warnings

warnings.filterwarnings("ignore")

import matplotlib
import math
def _safe_set_mpl_backend():
    try:
        if os.environ.get("DISPLAY", ""):
            try:
                matplotlib.use("TkAgg"); return "TkAgg"
            except Exception:
                try:
                    matplotlib.use("Qt5Agg"); return "Qt5Agg"
                except Exception:
                    matplotlib.use("Agg"); return "Agg"
        else:
            matplotlib.use("Agg"); return "Agg"
    except Exception:
        matplotlib.use("Agg"); return "Agg"
_MPL_BACKEND = _safe_set_mpl_backend()
print(f"[MPL] backend = {_MPL_BACKEND}")

import matplotlib.pyplot as plt

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

WEIGHTS    = "/home/pc/soomac_ws/src/zeus_vision/best_jjin_mak.pt"
DEVICE     = "0"       

CONF_DET   = 0.28       # YOLO 추론 최소 conf
CONF_PUB   = 0.50       # 대상: conf >= 0.5
IOU_TH     = 0.45
IMG_SIZE   = 640

COLOR_W, COLOR_H, COLOR_FPS = 640, 480, 30
DEPTH_W, DEPTH_H, DEPTH_FPS = 640, 480, 30
FONT = cv2.FONT_HERSHEY_SIMPLEX

# 포인트클라우드/평면 분할
PCL_STRIDE = 3
DEPTH_P_LOW, DEPTH_P_HIGH = 10, 90
PLANE_DIST_BASE, PLANE_RANSAC_N, PLANE_ITERS = 0.01, 3, 1000  # 1cm 허용
STAT_NB_NEIGHBORS, STAT_STD_RATIO = 20, 2.0
RAD_RADIUS, RAD_MIN_POINTS = 0.03, 10

# 제약/시각화
# Roll, Pitch 각도가 이 값보다 작으면 노이즈로 간주하여 0으로 처리
DEAD_ZONE_DEG = 3.0
MIN_AREA_PX   = 800
INLIER_RATIO_TH = 0.30    # plane_inlier / mask 비율
USE_DBSCAN_3D = True
DBSCAN_EPS_M  = 0.015     # 15mm
DBSCAN_MINPTS = 60

STAB_ANGLE_DEG = 1.0         # Δθ 임계각(도)
STAB_MIN_FRAMES = 20          # 연속 안정 프레임 수 임계
STAB_CENTER_JUMP_PX = 60      # 대상 변경/점프 감지용 중심 픽셀 허용 이동량
PUBLISH_COOLDOWN_FRAMES = 10  # 퍼블리시 후 재퍼블리시까지 쿨다운 프레임

# 세로 7.5cm, 가로 2.5cm → 긴 변 L=0.075, 짧은 변 W=0.025
BLOCK_LEN_M = 0.075
BLOCK_WID_M = 0.025

# 유틸 함수
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
    """
    P: (3,) in camera coords [X,Y,Z] (meters)
    return (u,v) pixel or None if Z<=0
    """
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
    """
    Open3D로 평면 분할 → z'(법선), x'(PCA 장축), y'(z×x), origin, inliers, plane_model
    """
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
    if z_axis[2] < 0:
        z_axis = -z_axis
    z_axis = z_axis / (np.linalg.norm(z_axis) + 1e-9)

    origin = inlier_cloud.get_center()

    _, cov = inlier_cloud.compute_mean_and_covariance()
    evals, evecs = np.linalg.eigh(cov)
    axes = evecs.T[np.argsort(evals)]
    x_axis = axes[2]  # PCA 장축

    # 카메라 X축을 평면에 사영하여 부호 정렬
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
    if inlier_points is None or len(inlier_points) == 0:
        return None
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(inlier_points))
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
    if labels.size == 0 or labels.max() < 0:
        return None
    uniq = [L for L in set(labels.tolist()) if L != -1]
    if not uniq:
        return None
    best = max(uniq, key=lambda L: int((labels == L).sum()))
    return inlier_points[labels == best]

# --------- [공용] 레이-평면 교점 (A안: 폴백) ---------
def ray_plane_intersect(cx, cy, intr, plane_model):
    """
    카메라 원점(0,0,0)에서 픽셀(cx,cy)로 나가는 레이와 평면(ax+by+cz+d=0)의 교점.
    성공 시 (3,), 실패 시 None
    """
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

# --------- minAreaRect + 실제 크기 스냅 기반 중심 보정 ---------
def _largest_contour(mask_bin):
    cnts, _ = cv2.findContours(mask_bin.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts: return None
    return max(cnts, key=cv2.contourArea)

def _expected_px_from_metric(L_m, W_m, z_m, fx, fy):
    w_px = L_m * fx / max(z_m, 1e-6)
    h_px = W_m * fy / max(z_m, 1e-6)
    return float(w_px), float(h_px)

def _rasterize_rotated_rect(shape_hw, rect):
    box = cv2.boxPoints(rect).astype(np.int32)
    mask = np.zeros(shape_hw, dtype=np.uint8)
    cv2.fillPoly(mask, [box], 1)
    return mask, box

def refine_center_minarearect_with_size(mask_bin, depth_frame, intr, depth_scale, L_m, W_m,
                                        plane_model, x_axis, y_axis):
    """
    [B] 기울기 보정 기대폭 + minAreaRect 스냅
    [A] 폴백: rect 중심 픽셀의 레이-평면 교점 사용
    반환: (refined_origin(3,), (cx,cy), z_med, rect_snap, rect_snap_box_pts, rect_orig_box_pts, origin_src)
    실패 시 (None, None, None, None, None, None, None)
    """
    H, W = mask_bin.shape[:2]
    cnt = _largest_contour(mask_bin)
    if cnt is None or cv2.contourArea(cnt) < 10:
        return None, None, None, None, None, None, None

    rect = cv2.minAreaRect(cnt)  # ((cx,cy),(w0,h0),angle)
    (cx, cy), (w0, h0), ang = rect
    rect_orig_box_pts = cv2.boxPoints(rect).astype(np.int32)  # 원본 박스 점

    ys, xs = np.where(mask_bin > 0)
    if xs.size == 0:
        return None, None, None, None, None, None, None

    depth = np.asanyarray(depth_frame.get_data())
    z_vals = depth[ys, xs].astype(np.float32) * depth_scale
    z_vals = z_vals[z_vals > 1e-6]
    if z_vals.size == 0:
        return None, None, None, None, None, None, None
    z_med = float(np.median(z_vals))

    # [B] 축 기울기 보정 스케일(화면 평면(XY) 투영 크기)
    scale_x = float(np.sqrt(x_axis[0]**2 + x_axis[1]**2)) if x_axis is not None else 1.0
    scale_y = float(np.sqrt(y_axis[0]**2 + y_axis[1]**2)) if y_axis is not None else 1.0

    fx, fy = intr.fx, intr.fy
    # (L,W) 가정
    w_exp1 = (L_m * fx / max(z_med, 1e-6)) * scale_x
    h_exp1 = (W_m * fy / max(z_med, 1e-6)) * scale_y
    # (W,L) 가정
    w_exp2 = (W_m * fx / max(z_med, 1e-6)) * scale_x
    h_exp2 = (L_m * fy / max(z_med, 1e-6)) * scale_y

    # 두 가설 중 오차가 작은 쪽 선택
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
        # [A] 폴백: rect 중심 픽셀의 레이-평면 교점
        pt = ray_plane_intersect(cx, cy, intr, plane_model)
        if pt is not None:
            return pt.astype(np.float32), (cx, cy), z_med, rect_snap, rect_snap_box_pts, rect_orig_box_pts, "A-ray-plane"
        # 최후의 폴백: depth 패치 median
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

# --------- [C] 평면 좌표계에서 중심 산출 후 3D 역변환 ---------
def plane_center_from_points(points_xyz, plane_origin, x_axis, y_axis):
    """
    in: 3D points (N,3), plane basis (origin, x_axis, y_axis)
    out: center_3d (3,)
    """
    if points_xyz is None or len(points_xyz) == 0:
        return None
    p_rel = points_xyz - plane_origin.reshape(1,3)
    x_coords = p_rel @ x_axis.reshape(3,1)  # (N,1)
    y_coords = p_rel @ y_axis.reshape(3,1)  # (N,1)
    mx = float(x_coords.mean()); my = float(y_coords.mean())
    center3d = plane_origin + mx * x_axis + my * y_axis
    return center3d.astype(np.float32)

class MPLLiveOne:
    def __init__(self):
        from matplotlib.lines import Line2D
        plt.ion()
        self.fig = plt.figure("Quaternion Visual Verification (Nearest Only)", figsize=(8, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel("X (right)"); self.ax.set_ylabel("Y (down)"); self.ax.set_zlabel("Z (forward)")
        self.ax.view_init(elev=25, azim=-120)
        self._arts = []
        proxies = [Line2D([0],[0], c='r', lw=3), Line2D([0],[0], c='g', lw=3),
                   Line2D([0],[0], c='b', lw=3), Line2D([0],[0], c='#00CCCC', lw=2.5)]
        self.ax.legend(proxies, ["Raw X'", "Raw Y'", "Raw Z'", "Final triad"], loc="lower left")
        self.text = self.ax.text2D(0.02, 0.98, "", transform=self.ax.transAxes, ha="left", va="top",
                                   fontsize=10, bbox=dict(boxstyle="round,pad=0.3", fc="yellow", ec="black", alpha=0.5))
        self._set_view()

    def _set_view(self):
        self.ax.set_xlim([-0.2, 0.2]); self.ax.set_ylim([-0.2, 0.2]); self.ax.set_zlim([0.3, 0.7])

    def _clear(self):
        for a in self._arts:
            try: a.remove()
            except Exception: pass
        self._arts.clear()

    def update(self, raw_pose, final_pose, inliers):
        self._clear()
        if final_pose is None or inliers is None:
            self.text.set_text("Detecting nearest...")
            self._set_view()
            self.fig.canvas.draw_idle(); plt.pause(0.001)
            return

        raw_pos, raw_R, _, _ = raw_pose
        pos_m, R_final, final_rpy, _ = final_pose

        sc = self.ax.scatter(inliers[:,0], inliers[:,1], inliers[:,2], s=1, alpha=0.4)
        self._arts.append(sc)

        arm = 0.1
        xr, yr, zr = raw_R[:,0], raw_R[:,1], raw_R[:,2]
        self._arts.append(self.ax.quiver(raw_pos[0],raw_pos[1],raw_pos[2], xr[0],xr[1],xr[2], length=arm, lw=3, color="r"))
        self._arts.append(self.ax.quiver(raw_pos[0],raw_pos[1],raw_pos[2], yr[0],yr[1],yr[2], length=arm, lw=3, color="g"))
        self._arts.append(self.ax.quiver(raw_pos[0],raw_pos[1],raw_pos[2], zr[0],zr[1],zr[2], length=arm, lw=3, color="b"))

        xv, yv, zv = R_final[:,0], R_final[:,1], R_final[:,2]
        self._arts.append(self.ax.quiver(pos_m[0],pos_m[1],pos_m[2], xv[0],xv[1],xv[2], length=arm*1.2, lw=2.5, color="#00CCCC"))
        self._arts.append(self.ax.quiver(pos_m[0],pos_m[1],pos_m[2], yv[0],yv[1],yv[2], length=arm*1.2, lw=2.5, color="#00CCCC"))
        self._arts.append(self.ax.quiver(pos_m[0],pos_m[1],pos_m[2], zv[0],zv[1],zv[2], length=arm*1.2, lw=2.5, color="#00CCCC"))


        c, s = pos_m, np.array([0.2, 0.2, 0.2], dtype=float)
        def _fix_lim(lim, pad=1e-3):
            a, b = float(lim[0]), float(lim[1])
            if not np.isfinite(a) or not np.isfinite(b): return None
            if a == b: a -= pad; b += pad
            if a > b: a, b = b, a
            return (a, b)
        
        xlim = _fix_lim([c[0]-s[0], c[0]+s[0]])
        ylim = _fix_lim([c[1]-s[1], c[1]+s[1]])
        zlim = _fix_lim([c[2]-s[2], c[2]+s[2]])
        if xlim and ylim and zlim:
            self.ax.set_xlim(xlim); self.ax.set_ylim(ylim); self.ax.set_zlim(zlim)
        else:
            self._set_view()

        roll, pitch, yaw = final_rpy
        self.text.set_text(f"Nearest Pose\nRPY: {roll:.1f}, {pitch:.1f}, {yaw:.1f}")
        self.fig.canvas.draw_idle(); plt.pause(0.001)


# ROS 2 노드 (퍼블리셔만 보유)
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
        self.intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        self.mpl = MPLLiveOne()

        # NEW(안정화): 퍼블리시 게이트 상태
        self.last_quat = None
        self.last_center = None
        self.stable_count = 0
        self.cooldown = 0
    
    def listener_callback(self, msg):
        # 필요하면 정규화: data = (msg.data or "").strip().lower()
        self.get_logger().info(f'Received order: {msg.data}')
        self.detect_signal = msg.data

    def publish_pose(self, pose):
        msg = Float32MultiArray()
        msg.data = [pose.roll, pose.pitch, pose.yaw]
        self.pose_pubisher.publish(msg)
        self.get_logger().info(f'Published Pose: {msg.data}'
        )
    def publish_block(self, label):
        msg = String()
        msg.data = label
        self.block_pubisher.publish(msg)
        self.get_logger().info(f'Published Block: {msg.data}')


# =========================
# 메인 루프(프레임 처리/퍼블리시)
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
            # --- 프레임 획득 ---
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

            # 모든 블록 plane inlier 누적 표시용(시각화용)
            plane_vis = (color_image * 0.3).astype(np.uint8)
            
            # 까만색 빈 이미지(마스크) 생성
            plane_accum_mask = np.zeros((COLOR_H, COLOR_W), dtype=np.uint8)

            # YOLO 
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

                    # 대략 중심
                    ys0, xs0 = np.where(mask_bin > 0)
                    if xs0.size > 0 and ys0.size > 0:
                        cx0, cy0 = int(xs0.mean()), int(ys0.mean())
                    else:
                        cx0, cy0 = int((x1 + x2)//2), int((y1 + y2)//2)

                    # (1) 2D 정제(최대 컴포넌트, 중심 바이어스)
                    mask_bin = connected_components_keep(mask_bin,
                                                         min_area_px=MIN_AREA_PX,
                                                         keep_largest=True,
                                                         center_bias=(cx0, cy0))

                    ys, xs = np.where(mask_bin > 0)
                    if xs.size == 0:
                        continue
                    cx, cy = int(xs.mean()), int(ys.mean())

                    # 오버레이: 검출 박스/텍스트
                    overlay = apply_mask_overlay(overlay, mask_bin, color=(0,255,255), alpha=0.35)
                    cv2.rectangle(overlay, (x1,y1), (x2,y2), (0,0,0), 2)
                    txt = f"{label} {conf_i:.2f}"
                    cv2.putText(overlay, txt, (x1, max(0,y1-8)), FONT, 0.6, (255,255,255), 3, cv2.LINE_AA)
                    cv2.putText(overlay, txt, (x1, max(0,y1-8)), FONT, 0.6, (0,0,0), 1, cv2.LINE_AA)

                    # (2) 3D 포인트/평면
                    pts, _, _ = get_points_from_mask(mask_bin, depth_frame, node.intr, node.depth_scale)
                    x_axis, y_axis, z_axis, origin_m, inliers, plane_model = segment_plane_and_axes(pts)
                    if origin_m is None:
                        continue

                    # plane inlier 2D 마스크 → 최대 컴포넌트
                    plane_inlier_mask = plane_inlier_mask_from_model(mask_bin, depth_frame, node.intr, node.depth_scale, plane_model)
                    plane_inlier_mask = keep_largest_component(plane_inlier_mask)
                    ratio = float(plane_inlier_mask.sum()) / (float(mask_bin.sum()) + 1e-9)
                    if ratio < INLIER_RATIO_TH:
                        continue

                    if plane_inlier_mask.any():
                        plane_accum_mask = np.clip(plane_accum_mask + plane_inlier_mask, 0, 1)

                    # (2.5) 실제 크기 기반 중심 보정 — [B] 기대폭 보정 + [A] 레이-평면 폴백
                    refined_origin, rect_center_px, z_med, rect_snap, rect_snap_box_pts, rect_orig_box_pts, origin_src = \
                        refine_center_minarearect_with_size(
                            plane_inlier_mask, depth_frame, node.intr, node.depth_scale,
                            BLOCK_LEN_M, BLOCK_WID_M,
                            plane_model, x_axis, y_axis
                        )
                    # minAreaRect 시각화: 원본(흰색), 스냅(노랑)
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

                    # (3) 3D DBSCAN 최대 군집 유지 + [C] 평면좌표계 중심 보정
                    if USE_DBSCAN_3D and inliers is not None and len(inliers) >= 50:
                        main = largest_cluster_dbscan(inliers, eps=DBSCAN_EPS_M, min_points=DBSCAN_MINPTS)
                        if main is not None and len(main) >= 50:
                            center3d = plane_center_from_points(main, origin_m, x_axis, y_axis)
                            if center3d is not None:
                                origin_m = center3d
                                origin_src = "C-plane-center"

                            # PCA로 x축 갱신(군집 기준)
                            cov = np.cov(main.T)
                            evals, evecs = np.linalg.eigh(cov)
                            axes = evecs.T[np.argsort(evals)]
                            x_axis = axes[2]
                            # x축 부호 정렬(평면 법선 기준)
                            a,b,c,d = plane_model
                            z_axis = np.array([a,b,c], dtype=np.float32)
                            if z_axis[2] < 0: z_axis = -z_axis
                            z_axis = z_axis/(np.linalg.norm(z_axis)+1e-9)
                            ex = np.array([1.0, 0.0, 0.0], dtype=np.float32)
                            r_x = ex - z_axis * float(ex @ z_axis)
                            if np.linalg.norm(r_x) > 1e-9 and float(x_axis @ r_x) < 0:
                                x_axis = -x_axis
                            x_axis = x_axis/(np.linalg.norm(x_axis)+1e-9)
                            y_axis = np.cross(z_axis, x_axis)
                            y_axis = y_axis/(np.linalg.norm(y_axis)+1e-9)
                            inliers = main

                    # [VIS] 후보 origin 픽셀 표시 (시안)
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
                        label=label, conf=conf_i, box=(x1,y1,x2,y2),
                        origin=origin_m, axes=(x_axis, y_axis, z_axis),
                        inliers=inliers, plane_model=plane_model, cx=cx, cy=cy,
                        src=origin_src or "-"
                    ))

            # PlaneFiltered 화면 합성(녹색)
            if plane_accum_mask.any():
                plane_vis = apply_mask_overlay(plane_vis, plane_accum_mask, color=(0,255,0), alpha=0.8)

            # ---------- 민재형 예시 ------
            def calculate_distance(point):
                return math.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            
            chosen = None; min_dis = +1e9
            for cand in candidates:
                if cand['origin'] is None: 
                    continue
            # min_z 대신 유클리디안 거리 계산
                dis = calculate_distance(cand['origin'])
                if dis < min_dis:
                    min_dis = dis
                    chosen = cand

            mpl_raw_pose = final_pose = None
            mpl_inliers = None

            if chosen is not None:
                x_axis, y_axis, z_axis = chosen['axes']
                origin_m = chosen['origin']

                cx, cy = chosen['cx'], chosen['cy']

                # [VIS] 최종 선택된 origin 빨간 십자
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
                    # 1) 원본 회전행렬/Raw Pose (시각화용)
                    raw_rot_matrix = np.stack([x_axis, y_axis, z_axis], axis=1)
                    mpl_raw_pose = (origin_m, raw_rot_matrix, None, None)  # (pos, R, rpy, quat)
                    mpl_inliers  = chosen['inliers']

                    # 2) 단일 프레임 기준 쿼터니언/이울러
                    raw_quat = SciRot.from_matrix(raw_rot_matrix).as_quat()
                    raw_rotation = SciRot.from_quat(raw_quat)
                    ypr = raw_rotation.as_euler('zyx', degrees=True)  # [yaw, pitch, roll]
                    yaw, pitch, roll = float(ypr[0]), float(ypr[1]), float(ypr[2])

                    # Dead-zone 및 제약조건 적용
                    is_roll_high  = abs(roll)  >= DEAD_ZONE_DEG
                    is_pitch_high = abs(pitch) >= DEAD_ZONE_DEG

                    # 둘 다 크면 큰 쪽만 살리고 나머지 0
                    if is_roll_high and is_pitch_high:
                        if abs(roll) >= abs(pitch):
                            pitch = 0.0
                        else:
                            roll = 0.0
                    else:
                        if not is_pitch_high: 
                            pitch = 0.0
                        if not is_roll_high:  
                            roll  = 0.0

                    final_rpy = (roll, pitch, yaw)

                    # 3) 최종 회전행렬/쿼터니언 재구성
                    final_rotation   = SciRot.from_euler('zyx', [yaw, pitch, roll], degrees=True)
                    final_rot_matrix = final_rotation.as_matrix()
                    final_quat       = final_rotation.as_quat()

                    # 4) 4x4 변환행렬 (시각화 m 유지 / 행렬엔 mm)
                    origin_mm = origin_m * 1000.0
                    final_matrix = np.eye(4, dtype=np.float64)
                    final_matrix[:3, :3] = final_rot_matrix
                    final_matrix[:3, 3]  = origin_mm  # translation [mm]

                    # ---------- 안정화 퍼블리시 게이트 ----------
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
                        if dot < 0.0:
                            dot = -dot  # 부호동치
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

                    # 게이트 간소 (사용자 요청 버전)
                    should_publish = (node.detect_signal != "")
                    if should_publish:
                        # 퍼블리시: 4x4 변환행렬
                        msg = Float32MultiArray()
                        rows, cols = 4, 4
                        msg.layout.dim.append(MultiArrayDimension(label='rows', size=rows, stride=cols))
                        msg.layout.dim.append(MultiArrayDimension(label='cols', size=cols, stride=1))
                        msg.layout.data_offset = 0
                        msg.data = final_matrix.flatten().astype(np.float32).tolist()
                        node.publisher_.publish(msg)
          
                        msg2 = Float32MultiArray()
                        msg2.data = [roll, pitch, yaw]
                        node.pose_pubisher.publish(msg2)
                        node.get_logger().info(f'Published Pose: {msg2.data}'
                        )
                        node.get_logger().info(
                            f"[STABLE PUBLISH] z={origin_m[2]:.3f} m | Δθ={dtheta:.3f}° | R:{roll:.1f} P:{pitch:.1f} Y:{yaw:.1f}"
                        )
                        node.publish_block(chosen['label'])
                        node.cooldown = PUBLISH_COOLDOWN_FRAMES
                        node.stable_count = 0
                        node.detect_signal = ""

                    # 2D 텍스트(상태 표시)
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

                    # Matplotlib 업데이트(최종 Pose 전달)
                    final_pose = (origin_m, final_rot_matrix, final_rpy, final_quat)

            # Matplotlib 3D
            node.mpl.update(mpl_raw_pose, final_pose, mpl_inliers)

            # 2D 시각화
            cv2.imshow("Detections (mask+box + centers)", overlay)
            cv2.imshow("PlaneFiltered (all blocks)", plane_vis)
            cv2.waitKey(1)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) received. Shutting down...")
    finally:
        node.pipeline.stop()
        cv2.destroyAllWindows()
        plt.ioff(); plt.close('all')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()