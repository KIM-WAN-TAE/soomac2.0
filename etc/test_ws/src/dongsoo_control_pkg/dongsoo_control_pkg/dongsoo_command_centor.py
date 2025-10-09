#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String
from dongsoo_interfaces.msg import DongSooCommand
import numpy as np
import threading, time

def deg_to_pulse(degree):
    p = int(np.round(-degree * (4096.0 / 360.0) + 2048))
    return max(0, min(4095, p))

def pulse_to_deg(pulse):
    pulse = max(0, min(4095, int(pulse)))
    return (pulse - 2048) * (360.0 / 4096.0)

def pos_as_T(P):
    T = np.eye(4)
    T[:3, 3] = np.asarray(P, float).reshape(3)
    return T

# 상태 상수
S_IDLE = 0
S_PREPOSE_DONE = 10
S_APPROACH_DONE = 20
S_DESCEND_OPEN_DONE = 30
S_CLOSE_DONE = 40
S_END = 50
S_END_DONE = 99

class CommandCentorNode(Node):
    def __init__(self):
        super().__init__('command_centor_node')
        self.get_logger().info('Command Centor is Ready!')

        self.sub_cb = ReentrantCallbackGroup()
        self.timer_cb = ReentrantCallbackGroup()

        self.lock = threading.RLock()
        self._start_ev = threading.Event()

        # 공유 상태
        self.cam_mat = np.eye(4, dtype=np.float32)
        self.obj_pos = None
        self.yaw_cam = None
        self.deg_1 = None
        self.move_done_msg = None
        self.state = S_IDLE
        self.wait_until = None
        self.approach_pos = None
        self._last_wrist = 0.0

        # 동작 옵션
        self.auto_start = True          # 실행 즉시 시작
        self.started_once = False
        self.use_move_done = False      # 상위 노드 'done' 신호 사용 여부
        self.md_timeout_s  = 6.0        # 신호 미수신 시 전이 타임아웃
        self._md_deadline  = None       # 타임아웃 마감 시각

        # 진행 로그 간격
        self._last_log_t = 0.0

        threading.Thread(target=self._input_loop, daemon=True).start()

        # 구독
        self.create_subscription(Float32MultiArray, '/info/array/target_obj_array',
                                 self.block_coordinate_callback, 10, callback_group=self.sub_cb)
        self.create_subscription(Float32MultiArray, '/aiot/matrix/camera',
                                 self.camera_mat_callback, 10, callback_group=self.sub_cb)
        self.create_subscription(Int32MultiArray, '/aiot/array/present_motor_pulse',
                                 self.read_pulse, 10, callback_group=self.sub_cb)
        self.create_subscription(String, '/info/string/movement_done',
                                 self.ik_state_callback, 10, callback_group=self.sub_cb)

        # 발행
        self.target_pose_pub = self.create_publisher(DongSooCommand, '/aiot/array/command_pose', 10)
        self.yaw_pose_pub    = self.create_publisher(Int32MultiArray, '/motor/command_dxl5_position', 10)
        self.tool_pub        = self.create_publisher(String, '/info/string/obj_name', 10)
        self.obj_coor_pub    = self.create_publisher(Float32MultiArray, '/info/array/obj_position', 10)
        self.grip_pub        = self.create_publisher(String, '/aiot/string/gripper_command', 10)

        # 타이머
        self.timer = self.create_timer(0.1, self.timer_callback, callback_group=self.timer_cb)  # 10 Hz

    # 입력 스레드
    def _input_loop(self):
        while rclpy.ok():
            if input('이동을 원하면 y를 입력하시오 : ').strip().lower() == 'y':
                self._start_ev.set()

    # 콜백들
    def ik_state_callback(self, msg: String):
        with self.lock:
            self.move_done_msg = msg.data

    def read_pulse(self, msg: Int32MultiArray):
        with self.lock:
            if msg.data:
                self.deg_1 = pulse_to_deg(msg.data[0])

    def camera_mat_callback(self, msg: Float32MultiArray):
        dims = msg.layout.dim
        if len(dims) < 2:
            self.get_logger().warn('잘못된 행렬 수신')
            return
        rows, cols = dims[0].size, dims[1].size
        if len(msg.data) != rows * cols:
            self.get_logger().warn(f'msg count error : {rows*cols}')
            return
        with self.lock:
            self.cam_mat = np.asarray(msg.data, dtype=np.float32).reshape(rows, cols)

    def block_coordinate_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warning('Wrong data length')
            return
        P = np.asarray(msg.data[:3], dtype=np.float32)   # camera frame
        T_CO = pos_as_T(P)
        with self.lock:
            T_BO = self.cam_mat @ T_CO
            self.obj_pos = T_BO[:3, 3].astype(np.float32)
            self.yaw_cam = float(msg.data[3])

        out = Float32MultiArray()
        out.data = self.obj_pos.tolist()
        self.obj_coor_pub.publish(out)

        self.get_logger().info(
            f'Position: [{self.obj_pos[0]:.3f}, {self.obj_pos[1]:.3f}, {self.obj_pos[2]:.3f}], yaw_cam: {self.yaw_cam:.3f}'
        )

    # 유틸
    def _publish_pose(self, pos_xyz, look='down', t=2.0, wrist=0.0):
        msg = DongSooCommand()
        msg.position = [float(pos_xyz[0]), float(pos_xyz[1]), float(pos_xyz[2])]
        msg.look = look
        msg.time = float(t)
        msg.wrist = float(wrist)
        self.target_pose_pub.publish(msg)
        # 타임아웃 전이를 쓰는 경우 deadline 갱신
        if not self.use_move_done:
            self._md_deadline = time.monotonic() + self.md_timeout_s

    def _set_tool(self, name: str):
        m = String(); m.data = name
        #self.tool_pub.publish(m)

    def _open_grip(self):
        m = String(); m.data = 'open'
        self.grip_pub.publish(m)

    def _close_grip(self):
        m = String(); m.data = 'close'
        self.grip_pub.publish(m)

    def _move_done(self) -> bool:
        now = time.monotonic()
        with self.lock:
            md = self.move_done_msg
            # 신호 기반
            if isinstance(md, str) and md.strip().lower() == 'done':
                self.move_done_msg = None
                self._md_deadline = None
                return True
        # 타임아웃 기반
        if not self.use_move_done:
            if self._md_deadline is None:
                self._md_deadline = now + self.md_timeout_s
            if now >= self._md_deadline:
                self._md_deadline = None
                return True
        return False

    def _delay(self, sec: float):
        self.wait_until = time.monotonic() + float(sec)

    def _waiting(self) -> bool:
        return self.wait_until is not None and time.monotonic() < self.wait_until

    # 상태기계
    def timer_callback(self):
        now = time.monotonic()
        if now - self._last_log_t >= 1.0:
            self._last_log_t = now
            self.get_logger().info(f'STATE={self.state}')

        if self._waiting():
            return
        else:
            self.wait_until = None

        with self.lock:
            state = self.state
            obj_pos = None if self.obj_pos is None else self.obj_pos.copy()
            yaw_cam = self.yaw_cam
            deg1 = self.deg_1

        # IDLE
        if state == S_IDLE:
            if (self.auto_start and not self.started_once) or self._start_ev.is_set():
                self._start_ev.clear()
                self.started_once = True
                self._publish_pose([0.25, 0.00, 0.25], look='down', t=2.0, wrist=0.0)
                with self.lock:
                    self.state = S_PREPOSE_DONE
            return

        # 프리포즈 완료 대기 → 접근
        if state == S_PREPOSE_DONE:
            if not self._move_done():
                return
            #self._set_tool('wire_stripper')
            if obj_pos is None or yaw_cam is None or deg1 is None:
                return
            detect = obj_pos.tolist()
            detect[0] += 0.03
            yaw = (yaw_cam - deg1) + np.rad2deg(np.arctan2(detect[1], detect[0]))
            self._last_wrist = float(yaw)
            self.approach_pos = detect
            self._publish_pose(detect, look='down', t=2.0, wrist=self._last_wrist)
            with self.lock:
                self.state = S_APPROACH_DONE
            return

        # 접근 완료 대기 → 하강 + 오픈
        if state == S_APPROACH_DONE:
            if not self._move_done():
                return
            if self.approach_pos is None:
                with self.lock:
                    self.state = S_END
                return
            descend = self.approach_pos.copy()
            descend[2] = float(descend[2]) - 0.03
            self._open_grip()
            self._delay(0.5)  # 오픈 안정화
            self._publish_pose(descend, look='down', t=2.0, wrist=self._last_wrist)
            with self.lock:
                self.state = S_DESCEND_OPEN_DONE
            return

        # 하강 완료 대기 → 클로즈
        if state == S_DESCEND_OPEN_DONE:
            if not self._move_done():
                return
            self._close_grip()
            self._delay(0.5)  # 파지 안정화
            with self.lock:
                self.state = S_CLOSE_DONE
            return

        
        # 종료
        if state == S_CLOSE_DONE:
            self._publish_pose([0.25, 0.00, 0.25], look='down', t=2.0, wrist=0.0)
            if not self._move_done():
                return
            self._delay(0.5)  # 파지 안정화
            #self.get_logger().info('동작 종료')
            with self.lock:
                self.state = S_END
            return

        if state == S_END:
            self._publish_pose([0.25, 0.03, 0.03], look='down', t=2.0, wrist=0.0)
            if not self._move_done():
                return
            self._delay(0.5)  # 파지 안정화
            with self.lock:
                self.state = S_END_DONE
            return


        # if state == S_END_DONE:
        #     if not self._move_done():
        #         return
        #     self._open_grip()
        #     self._delay(0.5)  # 파지 안정화
        #     print('111111111111111111111111111111111111111111111111')
        #     return
def main(args=None):
    rclpy.init(args=args)
    node = CommandCentorNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt로 종료')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
