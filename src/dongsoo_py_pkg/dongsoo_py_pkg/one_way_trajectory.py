#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray


class OneWayTrajectoryNode(Node):
    """
    4자유도 편도 Trajectory Planning 노드
    
    설정된 포인트 개수만큼 중간점을 생성하여
    /motor/command_position 토픽으로 발행
    
    시작점에서 종료점까지 편도로만 이동
    """
    
    def __init__(self):
        super().__init__('one_way_trajectory_node')
        
        # 파라미터 선언 및 기본값 설정
        self.declare_parameter('num_points', 500)  # 중간점 개수
        self.declare_parameter('interval_sec', 0.01)  # 각 포인트 간격 (초)
        self.declare_parameter('interpolation_type', 'linear')  # 'linear' 또는 'cubic'
        self.declare_parameter('auto_repeat', False)  # 자동 반복 실행 여부
        self.declare_parameter('repeat_delay', 5.0)  # 반복 간격 (초)
        
        # 파라미터 읽기
        self.num_points = self.get_parameter('num_points').get_parameter_value().integer_value
        self.interval_sec = self.get_parameter('interval_sec').get_parameter_value().double_value
        self.interpolation_type = self.get_parameter('interpolation_type').get_parameter_value().string_value
        self.auto_repeat = self.get_parameter('auto_repeat').get_parameter_value().bool_value
        self.repeat_delay = self.get_parameter('repeat_delay').get_parameter_value().double_value
        
        # 퍼블리셔 생성
        self.publisher = self.create_publisher(
            Int32MultiArray,
            '/motor/command_position',
            10
        )
        
        # 현재 위치 수신을 위한 서브스크립션
        self.position_subscriber = self.create_subscription(
            Int32MultiArray,
            '/motor/position',
            self.position_callback,
            10
        )
        
        # 4자유도 궤적 정의: start -> end (편도)
        self.start_pos = [2048.0, 2048.0, 2048.0, 2048.0]  # 기본값 (현재 위치로 업데이트됨)
        self.end_pos = [2048.0, 2740.0, 700.0, 1700.0]     # 종료점 (목표 위치)
        self.current_position_received = False  # 현재 위치 수신 플래그
        
        # self.end_pos = [2048.0, 2048.0, 2048.0, 2048.0]  # 시작점 (중앙 위치)
        # self.start_pos = [2048.0, 2740.0, 700.0, 1700.0]
        
        # 타이머 생성 (interval_sec 간격으로 실행)
        self.timer = self.create_timer(
            self.interval_sec,
            self.timer_callback
        )
        
        # 상태 변수
        self.current_point = 0
        self.trajectory_points = []
        self.is_running = False
        self.repeat_timer = None
        
        # 로그
        self.get_logger().info(f'One-Way Trajectory Node 시작')
        self.get_logger().info(f'포인트 개수: {self.num_points}')
        self.get_logger().info(f'포인트 간격: {self.interval_sec} 초')
        self.get_logger().info(f'보간 방법: {self.interpolation_type}')
        self.get_logger().info(f'자동 반복: {self.auto_repeat}')
        if self.auto_repeat:
            self.get_logger().info(f'반복 간격: {self.repeat_delay} 초')
        self.get_logger().info(f'시작점: {self.start_pos}')
        self.get_logger().info(f'종료점: {self.end_pos}')
        
        # 궤적 생성
        self.generate_trajectory()
        
        # 현재 위치를 받을 때까지 대기 후 시작
        self.get_logger().info('현재 모터 위치를 기다리는 중...')
        self.wait_timer = self.create_timer(0.1, self.wait_for_position)
    
    def position_callback(self, msg):
        """현재 모터 위치 수신 콜백"""
        if not self.current_position_received and len(msg.data) >= 4:
            # 현재 위치를 시작점으로 설정 (4자유도만 사용)
            self.start_pos = [float(msg.data[i]) for i in range(4)]
            self.current_position_received = True
            self.get_logger().info(f'현재 위치를 시작점으로 설정: {self.start_pos}')
            
            # 새로운 시작점으로 궤적 재생성
            self.generate_trajectory()
    
    def wait_for_position(self):
        """현재 위치 수신 대기"""
        if self.current_position_received:
            self.wait_timer.destroy()
            self.get_logger().info('3초 후 편도 궤적 실행 시작...')
            self.start_timer = self.create_timer(3.0, self.start_trajectory)
    
    def generate_trajectory(self):
        """편도 궤적 포인트 생성: start -> end"""
        self.trajectory_points = []
        
        if self.interpolation_type == 'linear':
            self.trajectory_points = self.generate_linear_trajectory()
        elif self.interpolation_type == 'cubic':
            self.trajectory_points = self.generate_cubic_trajectory()
        elif self.interpolation_type == 'quintic':
            self.trajectory_points = self.generate_quintic_trajectory()
        else:
            self.get_logger().warn(f'알 수 없는 보간 방법: {self.interpolation_type}, linear로 설정')
            self.trajectory_points = self.generate_linear_trajectory()
        
        self.get_logger().info(f'{len(self.trajectory_points)}개의 편도 궤적 포인트 생성 완료 (start->end)')
    
    def generate_linear_trajectory(self):
        """선형 보간 궤적 생성: start -> end"""
        points = []
        
        for i in range(self.num_points):
            t = i / (self.num_points - 1) if self.num_points > 1 else 0  # 0~1 사이의 비율
            point = []
            for j in range(4):  # 4축
                # 선형 보간: start + t * (end - start)
                pos = self.start_pos[j] + t * (self.end_pos[j] - self.start_pos[j])
                point.append(pos)
            points.append(point)
        
        return points
    
    def generate_cubic_trajectory(self):
        """3차 다항식 보간 궤적 생성 (S-curve): start -> end"""
        points = []
        
        for i in range(self.num_points):
            t = i / (self.num_points - 1) if self.num_points > 1 else 0  # 0~1 사이의 비율
            
            # S-curve: 3t^2 - 2t^3 (부드러운 가속/감속)
            s = 3 * t * t - 2 * t * t * t
            
            point = []
            for j in range(4):  # 4축
                pos = self.start_pos[j] + s * (self.end_pos[j] - self.start_pos[j])
                point.append(pos)
            points.append(point)
        
        return points
    
    def generate_quintic_trajectory(self):
        """5차 다항식 보간 궤적 생성 (더 부드러운 S-curve): start -> end"""
        points = []
        
        for i in range(self.num_points):
            t = i / (self.num_points - 1) if self.num_points > 1 else 0  # 0~1 사이의 비율
            
            # 5차 다항식: 6t^5 - 15t^4 + 10t^3 (매우 부드러운 가속/감속)
            s = 6 * t**5 - 15 * t**4 + 10 * t**3
            
            point = []
            for j in range(4):  # 4축
                pos = self.start_pos[j] + s * (self.end_pos[j] - self.start_pos[j])
                point.append(pos)
            points.append(point)
        
        return points
    
    def start_trajectory(self):
        """궤적 실행 시작"""
        self.is_running = True
        self.current_point = 0
        self.start_timer.destroy()  # 시작 타이머 제거
        self.get_logger().info('편도 궤적 실행 시작!')
    
    def timer_callback(self):
        """타이머 콜백 - 궤적 포인트 발행"""
        if not self.is_running:
            return
        
        if self.current_point < len(self.trajectory_points):
            # 현재 포인트 발행
            msg = Int32MultiArray()
            msg.data = [int(x) for x in self.trajectory_points[self.current_point]]
            self.publisher.publish(msg)
            
            # 진행률 계산
            progress = (self.current_point + 1) / len(self.trajectory_points) * 100
            
            self.get_logger().info(
                f'Point {self.current_point + 1}/{len(self.trajectory_points)} ({progress:.1f}%): '
                f'[{msg.data[0]:.1f}, {msg.data[1]:.1f}, {msg.data[2]:.1f}, {msg.data[3]:.1f}]'
            )
            
            self.current_point += 1
            
        else:
            # 궤적 완료
            self.is_running = False
            self.get_logger().info('편도 궤적 실행 완료!')
            
            if self.auto_repeat:
                self.get_logger().info(f'{self.repeat_delay}초 후 궤적 재시작...')
                self.repeat_timer = self.create_timer(self.repeat_delay, self.restart_trajectory)
            else:
                self.get_logger().info('궤적 실행 종료 - 노드를 종료합니다')
                # 1초 후 노드 종료
                self.shutdown_timer = self.create_timer(1.0, self.shutdown_node)
    
    def restart_trajectory(self):
        """궤적 재시작 (자동 반복 모드)"""
        self.current_point = 0
        self.is_running = True
        if self.repeat_timer:
            self.repeat_timer.destroy()
            self.repeat_timer = None
        self.get_logger().info('편도 궤적 재시작!')
    
    def set_trajectory_points(self, start_pos, end_pos):
        """궤적 포인트 동적 설정 (외부 호출용)"""
        self.start_pos = start_pos.copy()
        self.end_pos = end_pos.copy()
        self.generate_trajectory()
        self.get_logger().info(f'궤적 포인트 업데이트: {start_pos} -> {end_pos}')
    
    def stop_trajectory(self):
        """궤적 실행 중단"""
        self.is_running = False
        if self.repeat_timer:
            self.repeat_timer.destroy()
            self.repeat_timer = None
        self.get_logger().info('궤적 실행 중단')
    
    def shutdown_node(self):
        """노드 종료"""
        self.get_logger().info('노드 종료 중...')
        self.stop_trajectory()  # 안전한 정리
        if hasattr(self, 'shutdown_timer'):
            self.shutdown_timer.destroy()
        
        # ROS2 노드 종료 요청
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    node = OneWayTrajectoryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt로 종료')
    finally:
        node.stop_trajectory()  # 안전한 종료
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()