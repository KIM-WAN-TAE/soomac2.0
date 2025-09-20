# -*- coding: utf-8 -*-
import os
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from collections import deque
import warnings



warnings.filterwarnings("ignore")

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String

# Matplotlib
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

# Open3D
try:
    import open3d as o3d
    O3D_OK = True
except ImportError: O3D_OK = False

# SciPy (필수)
try:
    from scipy.spatial.transform import Rotation as SciRot
    SCIPY_OK = True
except ImportError: SCIPY_OK = False

WEIGHTS    = "/home/wt/zeus_wt_ws/src/vision_node/block_red.pt"
DEVICE     = "0"
CONF_TH, IOU_TH, IMG_SIZE = 0.28, 0.45, 960
COLOR_W, COLOR_H, COLOR_FPS = 640, 480, 30
DEPTH_W, DEPTH_H, DEPTH_FPS = 640, 480, 30
FONT = cv2.FONT_HERSHEY_SIMPLEX
PCL_STRIDE, CORE_ERODE_PX, CORE_DT_PX = 3, 1, 2
DEPTH_P_LOW, DEPTH_P_HIGH = 10, 90
PLANE_DIST_BASE, PLANE_RANSAC_N, PLANE_ITERS = 0.01, 3, 1000
STAT_NB_NEIGHBORS, STAT_STD_RATIO = 20, 2.0
RAD_RADIUS, RAD_MIN_POINTS = 0.03, 10
ROS_TOPIC_NAME = '/zeus/array/block_pose'

# =========================
# 유틸 함수 (변경 없음)
# =========================
def apply_mask_overlay(bgr, mask, alpha=0.4):
    H, W = bgr.shape[:2]; out = bgr.copy()
    if mask.dtype != np.uint8: mask = (mask > 0.5).astype(np.uint8)
    if mask.shape[:2] != (H, W): mask = cv2.resize(mask, (W, H), interpolation=cv2.INTER_NEAREST)
    color_overlay = np.zeros_like(out); color_overlay[:] = (0, 255, 255)
    m = mask.astype(bool); out[m] = cv2.addWeighted(out, 1 - alpha, color_overlay, alpha, 0)[m]
    return out

def get_points_from_mask(mask_bin, depth_frame, intr, depth_scale):
    depth = np.asanyarray(depth_frame.get_data())
    H, W = depth.shape[:2]
    if mask_bin.shape[:2] != (H, W): mask_bin = cv2.resize(mask_bin, (W, H), interpolation=cv2.INTER_NEAREST)
    ys, xs = np.where(mask_bin > 0)
    if ys.size == 0: return None
    xs, ys = xs[::PCL_STRIDE], ys[::PCL_STRIDE]
    zs = depth[ys, xs].astype(np.float32) * depth_scale
    valid = (zs > 1e-6)
    if not np.any(valid): return None
    xs, ys, zs = xs[valid], ys[valid], zs[valid]
    X = (xs - intr.ppx) * zs / intr.fx; Y = (ys - intr.ppy) * zs / intr.fy
    pts = np.stack([X, Y, zs], axis=1)
    if pts.shape[0] == 0: return None
    z = pts[:, 2]; lo, hi = np.percentile(z, [DEPTH_P_LOW, DEPTH_P_HIGH])
    return pts[(z >= lo) & (z <= hi)]

def get_plane_axes(pts):
    if not O3D_OK or pts is None or pts.shape[0] < 50: return [None] * 5
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=STAT_NB_NEIGHBORS, std_ratio=STAT_STD_RATIO)
    pcd, _ = pcd.remove_radius_outlier(nb_points=RAD_MIN_POINTS, radius=RAD_RADIUS)
    if len(pcd.points) < 50: return [None] * 5
    plane_model, inliers_idx = pcd.segment_plane(distance_threshold=PLANE_DIST_BASE, ransac_n=3, num_iterations=1000)
    if len(inliers_idx) < 50: return [None] * 5
    inlier_cloud = pcd.select_by_index(inliers_idx)
    a, b, c, d = plane_model
    z_axis = np.array([a, b, c], dtype=np.float32)
    if z_axis[2] < 0: z_axis = -z_axis
    origin = inlier_cloud.get_center()
    _, cov_matrix = inlier_cloud.compute_mean_and_covariance()
    evals, evecs = np.linalg.eigh(cov_matrix)
    axes = evecs.T[np.argsort(evals)]
    x_axis, y_axis = axes[2], axes[1]
    ex = np.array([1.0, 0.0, 0.0])
    r_x = ex - z_axis * (ex @ z_axis)
    if np.dot(x_axis, r_x) < 0: x_axis = -x_axis
    y_axis = np.cross(z_axis, x_axis)
    return x_axis, y_axis, z_axis, origin, np.asarray(inlier_cloud.points)

# =========================
# ROS2 Publisher 클래스 (변경 없음)
# =========================
class PosePublisher(Node):
    def __init__(self):
        super().__init__('block_pose_publisher')
        self.matrix_publisher = self.create_publisher(Float32MultiArray,  '/zeus/array/block_pose', 10)
        self.pose_pubisher = self.create_publisher(Float32MultiArray, '/zeus/xyzrpy/block_pose', 10)
        self.subscriber = self.create_subscription(String, '/zeus/string/block_order', self.listener_callback, 10)
        self.detect_signal = "None"  # 초기값 설정
    def publish_matrix(self, matrix):
        rows, cols = matrix.shape
        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label='rows', size=rows, stride=cols))
        msg.layout.dim.append(MultiArrayDimension(label='cols', size=cols, stride=1))
        msg.layout.data_offset = 0
        msg.data = matrix.flatten().tolist()
        self.matrix_publisher.publish(msg)

    def publish_pose(self, pose):
        msg = Float32MultiArray()
        msg.data = [pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw]
        self.pose_pubisher.publish(msg)
        self.get_logger().info(f'Published Pose: {msg.data}'
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received order: {msg.data}')
        self.detect_signal = msg.data

# =========================
# Matplotlib 시각화 클래스 (변경 없음)
# =========================
class MPLLive:
    def __init__(self):
        from matplotlib.lines import Line2D
        plt.ion(); self.fig = plt.figure("Quaternion Visual Verification", figsize=(8, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel("X (right)"); self.ax.set_ylabel("Y (down)"); self.ax.set_zlabel("Z (forward)")
        self.ax.view_init(elev=25, azim=-120)
        self.scatter = self.qx_raw = self.qy_raw = self.qz_raw = None
        self.qx_verify = self.qy_verify = self.qz_verify = None
        self.text = self.ax.text2D(0.02, 0.98, "", transform=self.ax.transAxes, ha="left", va="top", fontsize=10, bbox=dict(boxstyle="round,pad=0.3", fc="yellow", ec="black", alpha=0.5))
        proxies = [Line2D([0],[0], c='r', lw=3), Line2D([0],[0], c='g', lw=3), Line2D([0],[0], c='b', lw=3), Line2D([0],[0], c='#00FFFF', lw=2.5, ls='--')]
        self.ax.legend(proxies, ["Raw X'", "Raw Y'", "Raw Z'", "Stable Quat Verify Axes"], loc="lower left")
        self._set_view_to_default(); self.fig.tight_layout(); plt.pause(0.001)
    def _set_view_to_default(self):
        self.ax.set_xlim([-0.2, 0.2]); self.ax.set_ylim([-0.2, 0.2]); self.ax.set_zlim([0.3, 0.7])
    def _safe_remove(self, attr_name):
        art = getattr(self, attr_name, None)
        if art:
            try: art.remove()
            except (ValueError, AttributeError): pass
        setattr(self, attr_name, None)
    
    def update(self, raw_pose, stable_pose, inliers):
        for name in ["scatter", "qx_raw", "qy_raw", "qz_raw", "qx_verify", "qy_verify", "qz_verify"]: self._safe_remove(name)
        if stable_pose is None or inliers is None: self.text.set_text("Detecting..."); self._set_view_to_default(); self.fig.canvas.draw_idle(); plt.pause(0.001); return
        raw_pos, raw_rot_matrix, _, _ = raw_pose
        stable_pos, stable_rot_matrix, stable_rpy, stable_quat = stable_pose
        self.scatter = self.ax.scatter(inliers[:,0], inliers[:,1], inliers[:,2], s=1, c=inliers[:,2], cmap='viridis_r', alpha=0.5)
        arm = 0.1
        x_raw, y_raw, z_raw = raw_rot_matrix[:, 0], raw_rot_matrix[:, 1], raw_rot_matrix[:, 2]
        self.qx_raw = self.ax.quiver(raw_pos[0],raw_pos[1],raw_pos[2], x_raw[0],x_raw[1],x_raw[2], length=arm, lw=3, color="r")
        self.qy_raw = self.ax.quiver(raw_pos[0],raw_pos[1],raw_pos[2], y_raw[0],y_raw[1],y_raw[2], length=arm, lw=3, color="g")
        self.qz_raw = self.ax.quiver(raw_pos[0],raw_pos[1],raw_pos[2], z_raw[0],z_raw[1],z_raw[2], length=arm, lw=3, color="b")
        x_verify, y_verify, z_verify = stable_rot_matrix[:, 0], stable_rot_matrix[:, 1], stable_rot_matrix[:, 2]
        self.qx_verify = self.ax.quiver(stable_pos[0],stable_pos[1],stable_pos[2], x_verify[0],x_verify[1],x_verify[2], length=arm*1.2, lw=2.5, color="#00FFFF", linestyle='--')
        self.qy_verify = self.ax.quiver(stable_pos[0],stable_pos[1],stable_pos[2], y_verify[0],y_verify[1],y_verify[2], length=arm*1.2, lw=2.5, color="#00FFFF", linestyle='--')
        self.qz_verify = self.ax.quiver(stable_pos[0],stable_pos[1],stable_pos[2], z_verify[0],z_verify[1],z_verify[2], length=arm*1.2, lw=2.5, color="#00FFFF", linestyle='--')
        c, s = stable_pos, np.array([0.2, 0.2, 0.2])
        self.ax.set_xlim([c[0]-s[0], c[0]+s[0]]); self.ax.set_ylim([c[1]-s[1], c[1]+s[1]]); self.ax.set_zlim([c[2]-s[2], c[2]+s[2]])
        roll, pitch, yaw = stable_rpy
        text_content = (f"Position: {stable_pos[0]:.3f}, {stable_pos[1]:.3f}, {stable_pos[2]:.3f} m\n"
                      f"Final RPY: {roll:.1f}, {pitch:.1f}, {yaw:.1f}°\n"
                      f"Stable Quat: {stable_quat[0]:.3f}, {stable_quat[1]:.3f}, {stable_quat[2]:.3f}, {stable_quat[3]:.3f}")
        self.text.set_text(text_content); self.fig.canvas.draw_idle(); plt.pause(0.001)

# =========================
# 메인 함수
# =========================
def main():
    if not O3D_OK or not SCIPY_OK: print("[ERROR] Open3D와 SciPy가 모두 필요합니다."); return
    
    # ROS2 초기화
    rclpy.init()
    pose_publisher_node = PosePublisher()

    model = YOLO(WEIGHTS); print("YOLO 클래스:", model.names)
    pipeline = rs.pipeline(); config = rs.config()
    config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, COLOR_FPS)
    config.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, DEPTH_FPS)
    profile = pipeline.start(config); align = rs.align(rs.stream.color)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    
    mpl = MPLLive()
    quat_history = deque(maxlen=5)

    try:
        while rclpy.ok():
            frames = align.process(pipeline.wait_for_frames(timeout_ms=1000))
            depth, colorf = frames.get_depth_frame(), frames.get_color_frame()
            if not colorf or not depth: continue

            color = np.asanyarray(colorf.get_data()); overlay = color.copy()
            res = model(color, conf=CONF_TH, iou=IOU_TH, device=DEVICE, imgsz=IMG_SIZE, verbose=False)

            raw_pose, stable_pose, inliers_to_show = None, None, None

            if res and res[0].masks and res[0].boxes and len(res[0].masks.data) > 0:
                r = res[0]
                i = int(r.boxes.conf.cpu().numpy().argmax())
                mask = r.masks.data[i].cpu().numpy()
                overlay = apply_mask_overlay(overlay, mask)
                box = r.boxes.xyxy[i].cpu().numpy()
                ys, xs = np.where(mask > 0.5)
                cx, cy = int(xs.mean() if xs.size > 0 else (box[0]+box[2])/2), int(ys.mean() if ys.size > 0 else (box[1]+box[3])/2)
                
                pts = get_points_from_mask((mask > 0.5).astype(np.uint8), depth, intr, depth_scale)
                x_axis, y_axis, z_axis, origin, inliers = get_plane_axes(pts)
                
                if all(v is not None for v in [x_axis, y_axis, z_axis, origin]):
                    # 1. 포인트 클라우드에서 직접 계산된 '원본' 회전 행렬
                    raw_rot_matrix = np.stack([x_axis, y_axis, z_axis], axis=1)
                    raw_pose = (origin, raw_rot_matrix, None, None) # 시각화용 Raw Pose
                    inliers_to_show = inliers

                    # 2. 시간 필터링을 위한 Raw 쿼터니언 계산 및 저장
                    raw_quat = SciRot.from_matrix(raw_rot_matrix).as_quat()
                    quat_history.append(raw_quat)

                    # 3. 쿼터니언 평균을 통해 안정화된 방향 계산
                    q_sum = np.zeros(4)
                    for q in quat_history:
                        if np.dot(quat_history[0], q) < 0: q = -q
                        q_sum += q
                    stable_quat = q_sum / np.linalg.norm(q_sum)
                    
                    # 4. 안정화된 쿼터니언으로부터 안정화된 RPY 값 추출
                    stable_rotation = SciRot.from_quat(stable_quat)
                    stable_ypr = stable_rotation.as_euler('zyx', degrees=True)
                    stable_yaw, stable_pitch, stable_roll = stable_ypr[0], stable_ypr[1], stable_ypr[2]

                    # --- [MODIFIED] 최종 RPY 값에 제약조건 적용 ---
                    # 5. 안정화된 Roll/Pitch 값 중, 절댓값이 더 큰 쪽만 남기고 나머지는 0으로 처리
                    if abs(stable_roll) > abs(stable_pitch):
                        stable_pitch = 0.0
                    else:
                        stable_roll = 0.0
                    # -----------------------------------------------

                    # 6. 제약조건이 적용된 RPY로 최종 Pose 결정
                    final_rpy = (stable_roll, stable_pitch, stable_yaw)
                    
                    # 제약조건이 적용된 RPY로부터 최종 회전 행렬을 다시 계산
                    final_rotation = SciRot.from_euler('zyx', [stable_yaw, stable_pitch, stable_roll], degrees=True)
                    final_rot_matrix = final_rotation.as_matrix()
                    final_quat = final_rotation.as_quat()
                    
                    # === [수정] 4x4 변환행렬 생성 ===
                    final_matrix = np.eye(4)
                    final_matrix[:3, :3] = final_rot_matrix
                    origin[0]*= 1000  
                    origin[1]*= 1000 
                    origin[2]*= 1000 

                    final_matrix[:3, 3]  = origin  # translation (x,y,z)

                    stable_pose = (origin, final_rot_matrix, final_rpy, final_quat)
                    if pose_publisher_node.detect_signal == "block":
                    # --- ROS2 토픽으로 최종 회전 행렬 발행 ---
                        pose_publisher_node.publish_matrix(final_matrix)
                        pose_publisher_node.publish_pose(type('Pose', (object,), {
                            'x': origin[0], 'y': origin[1], 'z': origin[2],
                            'roll': stable_roll, 'pitch': stable_pitch, 'yaw': stable_yaw
                        })())
                        pose_publisher_node.detect_signal = "None"  # 한 번 발행 후 비활성화
                    # 터미널 및 HUD 출력
                    print(f"\nFinal RPY (deg): Roll={final_rpy[0]:.2f}, Pitch={final_rpy[1]:.2f}, Yaw={final_rpy[2]:.2f}")
                    cv2.putText(overlay, f"R:{final_rpy[0]:.1f} P:{final_rpy[1]:.1f} Y:{final_rpy[2]:.1f}", (cx-70, cy-20), FONT, 0.6, (255,255,255), 4, cv2.LINE_AA)
                    cv2.putText(overlay, f"R:{final_rpy[0]:.1f} P:{final_rpy[1]:.1f} Y:{final_rpy[2]:.1f}", (cx-70, cy-20), FONT, 0.6, (0,0,0), 2, cv2.LINE_AA)

            # 시각화 및 ROS 스핀
            mpl.update(raw_pose, stable_pose, inliers_to_show)
            cv2.imshow("Quaternion Visual Verification", overlay)
            rclpy.spin_once(pose_publisher_node, timeout_sec=0.001) # ROS 콜백 처리
            
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            
    except Exception as e:
        print(f"[ERROR] {e}"); import traceback; traceback.print_exc()
    finally:
        # 종료 처리
        pipeline.stop()
        cv2.destroyAllWindows()
        plt.ioff()
        plt.close('all')
        pose_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    if not O3D_OK or not SCIPY_OK:
        print("[ERROR] Open3D와 SciPy가 모두 필요합니다.")
    else:
        main()
