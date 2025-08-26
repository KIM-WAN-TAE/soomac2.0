#!/usr/bin/env python3

import time, os
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from robotics_interfaces.srv import MotorExecutor
from std_msgs.msg import Float32MultiArray, Float32
from test_robotics_proj.Inverse_Kinematics import inverse_kinematics
from test_robotics_proj.Trajectory_Planner import TrajectoryPlanner
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSHistoryPolicy

def get_it():
    return int(1955)

def let_it():
    return int(2354)

class MotorExecutorServer(Node):
    def __init__(self):
        super().__init__('motor_executor_server')
        self.lock = threading.Lock()
        
        self.srv_cb_group = ReentrantCallbackGroup()
        self.sub_cb_group = ReentrantCallbackGroup()
        qos = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST)
        
        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            '/robotics/degree/target_theta',
            10
        )
        
        self.present_x, self.present_y, self.present_z = None, None, None
        self.position_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics/coordinate/position',
            self.position_callback,
            qos,                                # ← 여기
            callback_group=self.sub_cb_group, 
        )   
        
        self.present_th1, self.present_th2, self.present_th3, self.present_th4 = None, None, None, None
        self.angle_sub = self.create_subscription(
            Float32MultiArray,
            '/robotics/radian/theta',
            self.angle_callback,
            qos,                                # ← 여기
            callback_group=self.sub_cb_group, 
        )
        
        self.gripper_pub = self.create_publisher(
            Float32,
            '/robotics/gripper/command',
            10
        )
        self.motor_speed_pub = self.create_publisher(
            Float32MultiArray,
            '/robotics/abs/speed',
            10
        )
        self.srv = self.create_service(MotorExecutor, 'motor_executor', 
                                       self.service_response_callback,
                                       callback_group=self.srv_cb_group)
        
        self.timer_period = 0.5
        self.create_timer(self.timer_period, self.timer_callback)

    def angle_callback(self, msg=Float32MultiArray):
        with self.lock:
            if len(msg.data) != 4:
                return
            
            self.present_th1 = msg.data[0]
            self.present_th2 = msg.data[1]
            self.present_th3 = msg.data[2]
            self.present_th4 = msg.data[3]
            
    def position_callback(self, msg=Float32MultiArray):
        with self.lock:
            if len(msg.data) != 3:
                return
            
            self.present_x = msg.data[0]
            self.present_y = msg.data[1]
            self.present_z = msg.data[2]
            # print(f'x: {self.present_x:.2f} / y: {self.present_y:.2f} / z: {self.present_z:.2f}')
    
    def timer_callback(self):
        pass
        
    def service_response_callback(self, request, response):
        try:
            start_point = [self.present_x, self.present_y, self.present_z]
            end_point= [request.x, request.y, request.z]
            grab = request.grab
            radius = request.r
            task = request.task
            # worktime = request.time
            
            print(f'요청된 목표 좌표: x={end_point[0]}, y={end_point[1]}, z={end_point[2]}')
            print(f'요청된 작업: {task}, 반경: {radius}')
            
            if grab:
                theta_5 = 1955.0
            else:
                theta_5 = 2354.0
            
            if task == 'detailed_move':
                print(' Gripper가 이동합니다. ')
                self.motor_speed_pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0]))

                # path  = TrajectoryPlanner(start_point=start_point, end_point=end_point, num_points=900).plan()
                path  = np.linspace(start_point, end_point, num=500)
                
                q_matrix = inverse_kinematics(path, 
                                                initial_q_full=[0, self.present_th1, self.present_th2, self.present_th3, self.present_th4])
                
                q_matrix = np.array(q_matrix)
                q_matrix = np.rad2deg(q_matrix)

                # 6) 시간 스케줄 기반 joint‐level 명령 발행
                q_msg    = Float32MultiArray()
                for idx, q in enumerate(q_matrix):
                    
                    # 6-2) 목표 각도 변환 및 퍼블리시
                    q_msg.data = [q[0], q[1], q[2], q[3], float(theta_5)]
                    self.motor_pub.publish(q_msg)
                    
                    time.sleep(0.0035)
                    
                response.success = True
                self.get_logger().info('플래닝 및 IK 성공')
                
            elif task == 'fast_move':
                print(' Gripper가 이동합니다. ')
                self.motor_speed_pub.publish(Float32MultiArray(data=[70.0, 70.0, 70.0, 70.0]))
                
                path = np.vstack((start_point, end_point))
                
                q_matrix = inverse_kinematics(path, 
                                                initial_q_full=[0, self.present_th1, self.present_th2, self.present_th3, self.present_th4])
                
                q_matrix = np.array(q_matrix)
                q_matrix = np.rad2deg(q_matrix)

                # 6) 시간 스케줄 기반 joint‐level 명령 발행
                q_msg    = Float32MultiArray()
                for idx, q in enumerate(q_matrix):
                    
                    # 6-2) 목표 각도 변환 및 퍼블리시
                    q_msg.data = [q[0], q[1], q[2], q[3], float(theta_5)]
                    self.motor_pub.publish(q_msg)
                    
                while True:
                    current = [self.present_x, self.present_y, self.present_z]
                    e = [abs(e - c) for e, c in zip(end_point, current)]
                    
                    if all(err <= 5 for err in e):
                        break
                    time.sleep(0.001)
                
                response.success = True
                self.get_logger().info('플래닝 및 IK 성공')
                
            elif task == 'pick' or task == 'place':
                print(' Gripper를 열거나 닫습니다. ')
                
                gripper_msg = Float32()
                gripper_msg.data = theta_5
                self.gripper_pub.publish(gripper_msg)
                
                time.sleep(0.5)
                
                response.success = True
                self.get_logger().info('잡기 성공')
            
        except Exception as e:
            self.get_logger().error(f'플래닝/IK 실패: {e}')
            response.success = False

        return response
    

def main(args=None):
    rclpy.init(args=args)
    node = MotorExecutorServer()
    # 스레드를 2개로 늘립니다.
    exec = MultiThreadedExecutor(num_threads=2)
    exec.add_node(node)
    exec.spin()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()