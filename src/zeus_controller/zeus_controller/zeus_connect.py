#!/usr/bin/env python3

import socket, time, rclpy, json
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

DEFAULT_IP = '192.168.1.23'
DEFAULT_PORT = 5000
RECV_BUFSIZE = 4096
SOCKET_TIMEOUT = 5.0

class TCPClient:
    def __init__(self, ip, port, logger):
        self.ip, self.port, self.log = ip, port, logger
        self.sock, self._buf = None, b''

    def connect(self):
        self.close()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(SOCKET_TIMEOUT)
        self.log.info(f"[TCP] connecting to {self.ip}:{self.port} ...")
        s.connect((self.ip, self.port))
        s.settimeout(SOCKET_TIMEOUT)
        self.sock, self._buf = s, b''
        self.log.info("[TCP] connected")

    def close(self):
        if self.sock:
            try: self.sock.close()
            except: pass
            self.sock = None

    def send_line(self, line: str):
        if not self.sock: raise RuntimeError("socket not connected")
        self.sock.sendall((line + '\n').encode('utf-8'))

    def read_line(self) -> str:
        if not self.sock: raise RuntimeError("socket not connected")
        while True:
            nl = self._buf.find(b'\n')
            if nl != -1:
                line = self._buf[:nl]; self._buf = self._buf[nl+1:]
                return line.decode('utf-8', errors='replace').strip()
            chunk = self.sock.recv(RECV_BUFSIZE)
            if not chunk: raise RuntimeError("connection closed by peer")
            self._buf += chunk

    def request_until_done(self, cmd_with_opt_payload: str):
        self.send_line(cmd_with_opt_payload)
        lines = []
        while True:
            line = self.read_line()
            if line == 'done': return lines
            lines.append(line)


class ZeusConnectNode(Node):
    def __init__(self):
        super().__init__('zeus_connect_node')
        self.declare_parameter('server_ip', DEFAULT_IP)
        self.declare_parameter('server_port', DEFAULT_PORT)
        ip = self.get_parameter('server_ip').get_parameter_value().string_value
        port = int(self.get_parameter('server_port').get_parameter_value().integer_value)

        self.client = TCPClient(ip, port, self.get_logger())
        self._connect_with_retry()

        # start 핸드셰이크
        try:
            resp = self.client.request_until_done('start')
            for line in resp:
                self.get_logger().info(f"[TCP] start resp: {line}")
        except Exception as e:
            self.get_logger().warn(f"[TCP] start handshake failed: {e}")

        # 명령 수신 (한 줄에 'cmd [payload]' 형태)
        self.create_subscription(String, '/zeus/string/binary_command',
                                 self.order_callback, 10)
        
        self.str_pub = self.create_publisher(String, '/zeus/string/binary_responed', 10)
        
        # Float32MultiArray 퍼블리셔 생성
        self.xy_pub = self.create_publisher(Float32MultiArray, '/zeus/array/xy_state', 10)
        self.joint_pub = self.create_publisher(Float32MultiArray, '/zeus/array/joint_state', 10)
        
        # 10Hz 타이머로 좌표 데이터 주기적 요청 및 발행
        self.create_timer(0.1, self.publish_coordinates)

    def _connect_with_retry(self, tries=2, delay=1.0):
        for i in range(tries):
            try:
                self.client.connect(); return
            except Exception as e:
                self.get_logger().error(f"[TCP] connect fail({i+1}/{tries}): {e}")
                time.sleep(delay)
        self.client.connect()

    def order_callback(self, msg: String):
        text = (msg.data or '').strip()
        if not text: return

        # 'cmd [payload]' → 'cmd+payload'
        if ' ' in text:
            cmd, payload = text.split(' ', 1)
            wire = f"{cmd}+{payload.strip()}"
        else:
            wire = text  # payload 없는 명령 (start, jnt_coor, xy_coor 등)

        try:
            lines = self.client.request_until_done(wire)
            # 간단한 출력 규칙: 첫 줄이 CSV 값이면 예쁘게 찍고, 나머지는 로그
            if lines:
                # 값 1줄 + ok/ERR 등 추가 줄이 있을 수 있음
                self.get_logger().info(f"[TCP] resp[0]: {lines[0]}")
                
                # 첫 번째 응답을 토픽으로 발행
                response_msg = String()
                response_msg.data = lines[0]
                self.str_pub.publish(response_msg)
                
                for i, ln in enumerate(lines[1:], 1):
                    self.get_logger().info(f"[TCP] resp[{i}]: {ln}")
            else:
                self.get_logger().info("[TCP] (no payload before done)")
                # 빈 응답도 토픽으로 발행
                response_msg = String()
                response_msg.data = ""
                self.str_pub.publish(response_msg)
        except Exception as e:
            self.get_logger().error(f"[TCP] request failed: {e}")
            try:
                self._connect_with_retry()
                lines = self.client.request_until_done(wire)
                if lines:
                    # 재시도 성공 시에도 토픽으로 발행
                    response_msg = String()
                    response_msg.data = lines[0]
                    self.str_pub.publish(response_msg)
                    
                for i, ln in enumerate(lines):
                    self.get_logger().info(f"[TCP] resp[{i}]: {ln}")
            except Exception as e2:
                self.get_logger().error(f"[TCP] retry failed: {e2}")
                # 재시도 실패 시 에러 메시지 발행
                error_msg = String()
                error_msg.data = f"ERROR: {e2}"
                self.str_pub.publish(error_msg)

    def publish_coordinates(self):
        try:
            # xy_state 요청
            xy_lines = self.client.request_until_done('xy_state')
            if xy_lines:
                xy_data = self.parse_coordinate_data(xy_lines[0])
                if xy_data is not None:
                    xy_msg = Float32MultiArray()
                    xy_msg.data = xy_data
                    self.xy_pub.publish(xy_msg)
            
            # joint_state 요청
            joint_lines = self.client.request_until_done('joint_state')
            if joint_lines:
                joint_data = self.parse_coordinate_data(joint_lines[0])
                if joint_data is not None:
                    joint_msg = Float32MultiArray()
                    joint_msg.data = joint_data
                    self.joint_pub.publish(joint_msg)
                    
        except Exception as e:
            self.get_logger().debug(f"[TCP] coordinate request failed: {e}")
            try:
                self._connect_with_retry()
            except:
                pass
    
    def parse_coordinate_data(self, data_str: str):
        try:
            # CSV 형태의 문자열을 float 리스트로 변환
            values = [float(x.strip()) for x in data_str.split(',') if x.strip()]
            return values
        except:
            return None

    def destroy_node(self):
        try: self.client.close()
        except: pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZeusConnectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
