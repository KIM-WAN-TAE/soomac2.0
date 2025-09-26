import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class GripperControllerNode(Node):
    def __init__(self):
        super().__init__('gripper_controller_node')

        self.subscription = self.create_subscription(
            String,
            '/zeus/string/gripper_command',
            self.listener_callback,
            10)

        try:
            # 1단계에서 확인한 아두이노 포트 이름을 여기에 적어주세요.
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            time.sleep(2) 
            self.get_logger().info('Serial port /dev/ttyACM0 opened successfully.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.ser = None

    def listener_callback(self, msg):
        if self.ser is None:
            self.get_logger().warn('Serial port not available. Command ignored.')
            return

        command = ''
        if msg.data == 'suction' or msg.data == 's':
            command = 's'
        elif msg.data == 'hold' or msg.data == 'h':
            command = 'h'
        elif msg.data == 'exhaust' or msg.data == 'e':
            command = 'e'
        else:
            self.get_logger().warn(f'Invalid command received: {msg.data}')
            return

        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f'Sent command "{command}" for ROS message "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = GripperControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()