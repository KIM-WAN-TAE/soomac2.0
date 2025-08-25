#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import time
import math


class TrajectoryTestNode(Node):
    """
    4자유도 Trajectory Planning 테스트를 위한 노드
    
    설정된 포인트 개수만큼 중간점을 생성하여
    /motor/command_position 토픽으로 발행 (Int32MultiArray 사용)
    
    시작점: [2048, 2048, 2048, 2048] (4자유도)
    종료점: [2048, 1800, 1800, 1800] (4자유도)
    """
    
    def __init__(self):
        super().__init__('trajectory_test_node')
        
        # 파라미터 선언 및 기본값 설정
        self.declare_parameter('num_points', 1000)  # 중간점 개수
        self.declare_parameter('interval_sec', 0.01)  # 각 포인트 간격 (초)
        self.declare_parameter('interpolation_type', 'linear')  # 'linear' 또는 'cubic'
        
        # 파라미터 읽기
        self.num_points = self.get_parameter('num_points').get_parameter_value().integer_value
        self.interval_sec = self.get_parameter('interval_sec').get_parameter_value().double_value
        self.interpolation_type = self.get_parameter('interpolation_type').get_parameter_value().string_value
        
        # 퍼블리셔 생성
        self.publisher = self.create_publisher(
            Int32MultiArray,
            '/motor/command_position',
            10
        )
        
        # 4자유도 궤적 정의: start -> 경유지 -> start (int 값으로 변경)
        self.start_pos = [2048, 2048, 2048, 2048]  # 시작점 (고정)
        
        # 경유지 위치 (사용자가 수정 가능)
        self.waypoint = [2048, 2780, 500, 1800]  # 2,3,4축 소폭 움직임
        
        # 타이머 생성 (interval_sec 간격으로 실행)
        self.timer = self.create_timer(
            self.interval_sec,
            self.timer_callback
        )
        
        # 상태 변수
        self.current_point = 0
        self.trajectory_points = []
        self.is_running = False
        
        # 로그
        self.get_logger().info(f'Trajectory Test Node 시작')
        self.get_logger().info(f'포인트 개수: {self.num_points}')
        self.get_logger().info(f'포인트 간격: {self.interval_sec} 초')
        self.get_logger().info(f'보간 방법: {self.interpolation_type}')
        self.get_logger().info(f'시작점: {self.start_pos}')
        self.get_logger().info(f'경유지: {self.waypoint}')
        self.get_logger().info(f'최종점: {self.start_pos} (시작점으로 복귀)')
        
        # 궤적 생성
        self.generate_trajectory()
        
        # 3초 후 시작
        self.get_logger().info('3초 후 궤적 실행 시작...')
        self.start_timer = self.create_timer(3.0, self.start_trajectory)
    
    def generate_trajectory(self):
        """궤적 포인트 생성: start -> waypoint -> start"""
        self.trajectory_points = []
        
        if self.interpolation_type == 'linear':
            self.trajectory_points = self.generate_linear_trajectory()
        elif self.interpolation_type == 'cubic':
            self.trajectory_points = self.generate_cubic_trajectory()
        else:
            self.get_logger().warn(f'알 수 없는 보간 방법: {self.interpolation_type}, linear로 설정')
            self.trajectory_points = self.generate_linear_trajectory()
        
        self.get_logger().info(f'{len(self.trajectory_points)}개의 궤적 포인트 생성 완료 (start->waypoint->start)')
    
    def generate_linear_trajectory(self):
        """선형 보간 궤적 생성: start -> waypoint -> start"""
        points = []
        
        # 전체 포인트를 반으로 나누어 두 구간으로 분할
        half_points = self.num_points // 2
        
        # 1구간: start -> waypoint
        for i in range(half_points):
            t = i / (half_points - 1) if half_points > 1 else 0  # 0~1 사이의 비율
            point = []
            for j in range(4):  # 4축
                # 선형 보간: start + t * (waypoint - start)
                pos = self.start_pos[j] + t * (self.waypoint[j] - self.start_pos[j])
                point.append(pos)
            points.append(point)
        
        # 2구간: waypoint -> start (waypoint는 중복 제거)
        for i in range(1, half_points + 1):
            t = i / half_points  # 0~1 사이의 비율
            point = []
            for j in range(4):  # 4축
                # 선형 보간: waypoint + t * (start - waypoint)
                pos = self.waypoint[j] + t * (self.start_pos[j] - self.waypoint[j])
                point.append(pos)
            points.append(point)
        
        return points
    
    def generate_cubic_trajectory(self):
        """3차 다항식 보간 궤적 생성 (S-curve): start -> waypoint -> start"""
        points = []
        
        # 전체 포인트를 반으로 나누어 두 구간으로 분할
        half_points = self.num_points // 2
        
        # 1구간: start -> waypoint
        for i in range(half_points):
            t = i / (half_points - 1) if half_points > 1 else 0  # 0~1 사이의 비율
            
            # S-curve: 3t^2 - 2t^3 (부드러운 가속/감속)
            s = 3 * t * t - 2 * t * t * t
            
            point = []
            for j in range(4):  # 4축
                pos = self.start_pos[j] + s * (self.waypoint[j] - self.start_pos[j])
                point.append(pos)
            points.append(point)
        
        # 2구간: waypoint -> start (waypoint는 중복 제거)
        for i in range(1, half_points + 1):
            t = i / half_points  # 0~1 사이의 비율
            
            # S-curve: 3t^2 - 2t^3 (부드러운 가속/감속)
            s = 3 * t * t - 2 * t * t * t
            
            point = []
            for j in range(4):  # 4축
                pos = self.waypoint[j] + s * (self.start_pos[j] - self.waypoint[j])
                point.append(pos)
            points.append(point)
        
        return points
    
    def start_trajectory(self):
        """궤적 실행 시작"""
        self.is_running = True
        self.current_point = 0
        self.start_timer.destroy()  # 시작 타이머 제거
        self.get_logger().info('궤적 실행 시작!')
    
    def timer_callback(self):
        """타이머 콜백 - 궤적 포인트 발행"""
        if not self.is_running:
            return
        
        if self.current_point < len(self.trajectory_points):
            # 현재 포인트 발행 (Int32MultiArray로 변경)
            msg = Int32MultiArray()
            msg.data = [int(round(x)) for x in self.trajectory_points[self.current_point]]
            self.publisher.publish(msg)
            
            self.get_logger().info(
                f'Point {self.current_point + 1}/{len(self.trajectory_points)}: '
                f'[{msg.data[0]}, {msg.data[1]}, {msg.data[2]}, {msg.data[3]}]'
            )
            
            self.current_point += 1
            
        else:
            # 궤적 완료
            self.is_running = False
            self.get_logger().info('궤적 실행 완료!')
            
            # 5초 후 다시 시작 (반복 실행)
            self.get_logger().info('5초 후 궤적 재시작...')
            # self.restart_timer = self.create_timer(5.0, self.restart_trajectory)
    
    def restart_trajectory(self):
        """궤적 재시작"""
        self.current_point = 0
        self.is_running = True
        self.restart_timer.destroy()
        self.get_logger().info('궤적 재시작!')


def main(args=None):
    rclpy.init(args=args)
    
    node = TrajectoryTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt로 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()