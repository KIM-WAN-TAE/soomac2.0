#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket, time, rclpy
from rclpy.node import Node
from std_msgs.msg import String

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


class ZeusConnectTestNode(Node):
    def __init__(self):
        super().__init__('zeus_connect_test_node')
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
        self.create_subscription(String, '/zeus/test_command',
                                 self.order_callback, 10)

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
                for i, ln in enumerate(lines[1:], 1):
                    self.get_logger().info(f"[TCP] resp[{i}]: {ln}")
            else:
                self.get_logger().info("[TCP] (no payload before done)")
        except Exception as e:
            self.get_logger().error(f"[TCP] request failed: {e}")
            try:
                self._connect_with_retry()
                lines = self.client.request_until_done(wire)
                for i, ln in enumerate(lines):
                    self.get_logger().info(f"[TCP] resp[{i}]: {ln}")
            except Exception as e2:
                self.get_logger().error(f"[TCP] retry failed: {e2}")

    def destroy_node(self):
        try: self.client.close()
        except: pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZeusConnectTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
