#!/usr/bin/env python3
"""
motor_dummy.py
Dummy node that publishes the same topics as motor_connect.cpp
without requiring actual motor hardware connection.
This is useful for testing other nodes that consume these topics.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import random


class MotorDummyNode(Node):
    def __init__(self):
        super().__init__('motor_dummy_node')
        
        # Publishers - same topics as motor_connect.cpp
        self.publisher_current = self.create_publisher(
            Float32MultiArray, 'motor/current', 10)
        self.publisher_velocity = self.create_publisher(
            Float32MultiArray, 'motor/velocity', 10)
        self.publisher_position = self.create_publisher(
            Int32MultiArray, 'motor/position', 10)
        
        # Subscribers - to respond to position commands
        self.subscription_position = self.create_subscription(
            Int32MultiArray, 'motor/command_position', 
            self.position_callback, 10)
        self.subscription_dxl5_position = self.create_subscription(
            Int32MultiArray, 'motor/command_dxl5_position',
            self.dxl5_position_callback, 10)
        
        # Timer for publishing dummy data
        self.timer = self.create_timer(0.01, self.publish_dummy_data)  # 200Hz like motor_connect.cpp
        
        # Dummy motor state (5 motors: IDs 1,2,3,4,5)
        self.num_motors = 5
        self.current_positions = [0] * self.num_motors  # pulse counts
        self.target_positions = [0] * self.num_motors   # pulse counts
        self.velocities = [0.0] * self.num_motors       # velocity in counts/sec
        self.currents = [0.0] * self.num_motors         # current in raw counts
        
        # Simulation parameters
        self.position_gain = 0.1  # How fast motors reach target position
        self.noise_level = 2.0    # Noise amplitude
        
        # Initialize with some realistic values
        for i in range(self.num_motors):
            self.current_positions[i] = random.randint(-1000, 1000)
            self.target_positions[i] = self.current_positions[i]
        
        self.get_logger().info("Motor Dummy Node Started - Publishing dummy motor data")
        self.get_logger().info("Topics: /motor/current, /motor/velocity, /motor/position")
    
    def position_callback(self, msg):
        """Handle position commands for motors 1-4"""
        if len(msg.data) >= 4:
            for i in range(4):
                self.target_positions[i] = msg.data[i]
            self.get_logger().info(f"Position command received (motors 1-4): {msg.data[:4]}")
    
    def dxl5_position_callback(self, msg):
        """Handle position command for motor 5"""
        if len(msg.data) >= 1:
            self.target_positions[4] = msg.data[0]
            self.get_logger().info(f"DXL5 Position command received: {msg.data[0]}")
    
    def simulate_motor_movement(self):
        """Simulate realistic motor movement towards target positions"""
        dt = 0.005  # 5ms timestep
        
        for i in range(self.num_motors):
            # Calculate position error
            pos_error = self.target_positions[i] - self.current_positions[i]
            
            # Simple proportional control simulation
            velocity_command = pos_error * self.position_gain
            
            # Add some velocity limits
            max_velocity = 500.0  # counts/sec
            velocity_command = max(-max_velocity, min(max_velocity, velocity_command))
            
            # Update position and velocity
            self.velocities[i] = velocity_command
            self.current_positions[i] += velocity_command * dt
            
            # Add some noise to make it realistic
            noise = random.uniform(-self.noise_level, self.noise_level)
            self.current_positions[i] += noise
            
            # Simulate current based on effort (proportional to position error)
            effort_current = abs(pos_error) * 0.1  # Simplified current simulation
            self.currents[i] = effort_current + random.uniform(-5, 5)  # Add noise
            
            # Clamp current to realistic range
            self.currents[i] = max(-1000, min(1000, self.currents[i]))
    
    def publish_dummy_data(self):
        """Publish dummy motor data at regular intervals"""
        # Simulate motor movement
        self.simulate_motor_movement()
        
        # Create and publish current data
        current_msg = Float32MultiArray()
        current_msg.data = [float(c) for c in self.currents]
        self.publisher_current.publish(current_msg)
        
        # Create and publish velocity data  
        velocity_msg = Float32MultiArray()
        velocity_msg.data = [float(v) for v in self.velocities]
        self.publisher_velocity.publish(velocity_msg)
        
        # Create and publish position data (as integers)
        position_msg = Int32MultiArray()
        position_msg.data = [int(round(p)) for p in self.current_positions]
        self.publisher_position.publish(position_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = MotorDummyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Motor Dummy Node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()