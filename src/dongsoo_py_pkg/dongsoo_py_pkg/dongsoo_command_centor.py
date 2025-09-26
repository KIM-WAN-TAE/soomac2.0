#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String
from dongsoo_interfaces.msg import DongSooCommand

import numpy as np
import threading
import time

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
    
class CommandCentorNode(Node):
    def __init__(self):
        super().__init__('command_centor_node')
        self.get_logger().info('Command Centor is Ready!')
        
        self.sub_callback_gb = ReentrantCallbackGroup()
        self.timer_callback_gb = ReentrantCallbackGroup()
        
        self.lock = threading.RLock()
        self._start_ev = threading.Event()
        threading.Thread(target=self._input_loop, daemon=True).start()
        
        self.block_coor_sub = self.create_subscription(
            Float32MultiArray,
            '/info/array/target_obj_array',
            self.block_coordinate_callback,
            10,
            callback_group=self.sub_callback_gb)
        
        self.camera_mat_sub = self.create_subscription(
            Float32MultiArray,
            '/info/matrix/camera',
            self.camera_mat_callback,
            10,
            callback_group=self.sub_callback_gb)
        
        self.position_sub = self.create_subscription(
            Int32MultiArray,
            '/motor/position',
            self.read_pulse,
            10,
            callback_group=self.sub_callback_gb)
        
        self.deg_1 = None
        self.cam_mat = np.eye(4)
        
        self.target_pose_pub = self.create_publisher(
            DongSooCommand,
            '/command/array/pose',
            10)

        self.obj_pos = None

        self.yaw_pose_pub    = self.create_publisher(
            Int32MultiArray,
            '/motor/command_dxl5_position',
            10)
        
        self.tool_pub        = self.create_publisher(
            String,
            '/info/string/obj_name',
            10
        )
        
        self.ik_done_sub = self.create_subscription(
            String,
            '/info/string/movement_done',
            self.ik_state_callback,
            10
        )
        
        self.obj_coor_pub = self.create_publisher(
            Float32MultiArray,
            '/info/array/obj_position',
            10
        )
        
        TIMER_PERIOD = 1/5
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback, callback_group=self.timer_callback_gb)
        
        self.move_done_msg = None
        
        self.yaw = None
        self.state = 0
        
    def _input_loop(self):
        while rclpy.ok():
            ans = input('이동을 원하면 y를 입력하시오 : ')
            if ans.lower() == 'y':
                self._start_ev.set()
        
    def ik_state_callback(self, msg : String):
        self.move_done_msg = msg.data
        
    def read_pulse(self, msg : Int32MultiArray):
        pulse = msg.data
        with self.lock:
            self.deg_1 = pulse_to_deg(pulse[0])
        
    def camera_mat_callback(self, msg : Float32MultiArray):
        dims = msg.layout.dim
        
        if len(dims) < 2:
            self.get_logger().warn(' 잘못된 행렬 수신 ')
            return

        rows = dims[0].size
        cols = dims[1].size
        
        if len(msg.data) != rows * cols:
            self.get_logger().warn(f' msg count error : msg_count : {rows*cols}')
            return
        
        with self.lock:
            self.cam_mat = np.asarray(msg.data, dtype=np.float32).reshape(rows, cols)
            
    def block_coordinate_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warning('Wrong data length')
            return

        P = np.asarray(msg.data[:3], dtype=np.float32)   # [x,y,z] in camera frame
        T_CO = pos_as_T(P)                                # 4x4
        with self.lock:
            T_BO = self.cam_mat @ T_CO
            self.obj_pos = T_BO[:3, 3].astype(np.float32)
            self.yaw = float(msg.data[3])

        out = Float32MultiArray()
        out.data = self.obj_pos.tolist()
        self.obj_coor_pub.publish(out)

        self.get_logger().info(
            f'Position: [{self.obj_pos[0]:.2f}, {self.obj_pos[1]:.2f}, {self.obj_pos[2]:.2f}], Yaw: {self.yaw:.3f}'
        )       
        
    def timer_callback(self):
        with self.lock:
            state = self.state
            obj_pos = None if self.obj_pos is None else self.obj_pos.copy()
            if self.yaw is not None:
                yaw = self.yaw - self.deg_1
            move_done = self.move_done_msg

        tar_msg = DongSooCommand()
        tool_msg = String()

        if state == 0:
            if self._start_ev.is_set():
                self._start_ev.clear()
                tar_msg.position = [0.25, 0.0, 0.25]
                tar_msg.look = 'down'
                tar_msg.time = 3.0
                tar_msg.wrist = 0.0
                self.target_pose_pub.publish(tar_msg)
            if move_done == 'done':
                self.get_logger().info('move is done')
                with self.lock:
                    self.move_done_msg = None
                    self.state = 1

        elif state == 1:
            time.sleep(1)
            self.get_logger().info(f"{state}")
            tool_msg.data = 'wire_cutter'
            self.tool_pub.publish(tool_msg)
            if obj_pos is not None:
                with self.lock:
                    self.state = 2

        elif state == 2:
            time.sleep(1)
            self.get_logger().info(f"{state}")
            detect_position = obj_pos.tolist()
            detect_position[0] -= 0.08
            detect_position[2] = 0.25
            
            tar_msg.position = detect_position
            tar_msg.look = 'down'
            tar_msg.time = 3.0
            
            self.target_pose_pub.publish(tar_msg)
            with self.lock:
                self.state = 3

        elif state == 3:
            time.sleep(1)
            self.get_logger().info(f"{state}")
            if move_done == 'done':
                self.get_logger().info('move is done')
                with self.lock:
                    self.move_done_msg = None
                    self.obj_pos = None
                    self.state = 4
            else:
                self.get_logger().info('waiting for movement is done')

        elif state == 4:
            time.sleep(1)
            self.get_logger().info(f"{state}")
            if obj_pos is None:
                tool_msg.data = 'wire_cutter'
                self.tool_pub.publish(tool_msg)
                return
            self.get_logger().info(f'{obj_pos}, {yaw} 수신 완료')
            with self.lock:
                self.state = 5

        elif state == 5:
            time.sleep(0.5)
            self.get_logger().info(f"{state}")
            pick_position = obj_pos.tolist()
            pick_position_z = pick_position[2]
            pick_position[2] = pick_position_z - 0.005
            
            tar_msg.position = pick_position
            tar_msg.look = 'down'
            tar_msg.time = 5.0
            tar_msg.wrist = yaw
            self.target_pose_pub.publish(tar_msg)
            
            with self.lock:
                self.state = 6

        elif state == 6:
            time.sleep(0.5)
            self.get_logger().info(f"{state}")
            if move_done == 'done':
                self.get_logger().info('move is done')
                with self.lock:
                    self.move_done_msg = None
                    self.state = 7
            else:
                self.get_logger().info('waiting for movement is done')

        elif state == 7:
            self.get_logger().info(f"{state}")
            self.get_logger().info('동작 종료')
            return


def main(args=None):
    rclpy.init(args=args)
    node = CommandCentorNode()
    executor = MultiThreadedExecutor(num_threads=2)  # 필요시 4 등으로
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