#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial

class Led_Bridge(Node):
    def __init__(self, port='/dev/ttyACM0', baud=115200):
        super().__init__('led_bridge_node')
        self.ser = serial.Serial(port, baud, timeout=0.1)
        """_summary_
        """        self.create_subscription(Bool, '/aiot/bool/led_command', self.led_callback, 10)

    def led_callback(self, msg):
        self.ser.write(b'1\n' if msg.data else b'0\n')

def main():
    rclpy.init()
    node = Led_Bridge()
    rclpy.spin(node)

if __name__ == '__main__':
    main