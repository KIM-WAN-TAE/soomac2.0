#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dongsoo_interfaces.srv import DongSooExecutor
from dongsoo_interfaces.msg import DongSooCommand
import numpy as np

class DongsooClient(Node):
    def __init__(self):
        super().__init__('dongsoo_client')
        self.get_logger().info('[AIOT] DongSoo Service Client On! ')
        
        self.dongsoo_client = self.create_client(DongSooExecutor, 'dongsoo_executor')
        
        self.create_subscription(DongSooCommand, '/aiot/custom/command',
                                 self.target_pose_callback, 10)
        
        self.srv_pub = self.create_publisher(String, '/aiot/string/client_done', 10)

        self.last_position = np.array([0.0, 0.0, 0.0])  # 마지막 위치 저장

        self.target_position = np.array([])
        self.target_frame = None
        self.target_look  = None
        self.target_time  = None
        self.target_wrist = None
        self.target_flag  = False
        
        timer_period = 1/10
        self.client_timer = self.create_timer(timer_period, self.client_timer_callback)
        
    def target_pose_callback(self, msg : DongSooCommand):
        FRAME = msg.frame
        P     = np.array(msg.position, dtype=np.float32)
        LOOK  = msg.look
        TIME  = msg.time
        WRIST = msg.wrist

        self.target_frame    = FRAME
        self.target_position = P
        self.target_look     = LOOK
        self.target_time     = TIME
        self.target_wrist    = WRIST
        self.target_flag     = True
        
    def client_timer_callback(self):
        self.get_logger().info(f'[AIOT] Flag State : {self.target_flag}')

        if not self.target_flag:
            self.get_logger().info(' Waiting Target Pose... ')
            return

        if self.target_flag and self.target_position.size != 0:
            copy_frame  = self.target_frame
            copy_target = self.target_position
            copy_look   = self.target_look
            copy_time   = self.target_time
            copy_wrist  = self.target_wrist
            self.target_flag = False
            self.send_next_pose(copy_frame, copy_target, copy_look, copy_time, copy_wrist)
                
    def send_next_pose(self, frame, position, look, time, wrist):
        if position.size < 3:
            self.get_logger().warn(' Wrong Array Size Target Pose')
            return
        
        req = DongSooExecutor.Request()
        req.frame    = frame
        req.position = position
        req.look     = look
        req.time     = time
        req.wrist    = wrist
                
        future = self.dongsoo_client.call_async(req)
        future.add_done_callback(self.response_callback)
        
    def response_callback(self, future):
        srv_msg = String()
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'[AIOT] 서비스 호출 실패: {e}')
            return

        if res.success:
            self.get_logger().info(f'[AIOT] 동작 성공 ')
            # 동작 성공 시 마지막 위치 업데이트
            self.last_position = self.target_position.copy()
            self.get_logger().info(f'[AIOT] Last position updated: {self.last_position}')
            srv_msg.data = 'done'

        else:
            self.get_logger().warn(f'[AIOT] 동작 실패 ')
            srv_msg.data = 'fail'

        self.srv_pub.publish(srv_msg)
        self.target_flag = False
        
def main(args=None):
    rclpy.init(args=args)
    node = DongsooClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down Dongsoo Service Client...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()