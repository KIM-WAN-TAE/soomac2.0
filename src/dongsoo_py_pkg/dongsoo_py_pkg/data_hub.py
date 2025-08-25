#!/usr/bin/env python3

import rclpy
import os, sys, threading, json, ast
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from dynamixel_sdk import *

# json이 존재하는 package를 ROS 표준 방식으로 찾기, 불가능 할 경우 준비한 대체 경로로 변경
try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None


def dh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa =     np.cos(alpha),     np.sin(alpha)
    
    T = np.array([[ ct, -st*ca,  st*sa, a*ct],
                  [ st,  ct*ca, -ct*sa, a*st],
                  [  0,      sa,     ca,    d],
                  [  0,       0,      0,    1]])
    return T

# JSON 파일을 읽어올 때 string 타입을 정확하게 읽어오는 클래스
class SafeExpr:
    ALLOWED = (ast.Expression, ast.BinOp, ast.UnaryOp, ast.Constant, ast.Name, ast.Add, ast.Sub,
               ast.Mult, ast.Div, ast.Pow, ast.UAdd, ast.USub, ast.Load)
    
    def __init__(self, varmap):
        self.varmap = varmap

    def eval(self, s):
        if isinstance(s, (int, float)):
            return float(s)
        if not isinstance(s, str):
            raise ValueError(f"Unsupported type: {type(s)}")
        tree = ast.parse(s, mode='eval')
        if not self._ok(tree):
            raise ValueError(f"Disallowed expr: {s}")
        return self._eval(tree.body)

    def _ok(self, node):
        if not isinstance(node, self.ALLOWED):
            return False
        if isinstance(node, ast.Call):
            return False
        return all(self._ok(c) for c in ast.iter_child_nodes(node))

    def _eval(self, node):
        # Python 3.8+ 에서는 숫자 리터럴이 ast.Constant로 들어옴
        if isinstance(node, ast.Constant):
            v = node.value
            # bool은 int의 서브클래스이므로 명시적으로 배제
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                raise ValueError("Only numeric constants allowed")
            return float(v)

        if isinstance(node, ast.Name):
            if node.id in self.varmap:
                return float(self.varmap[node.id])
            raise ValueError(f"Unknown name: {node.id}")

        if isinstance(node, ast.UnaryOp):
            v = self._eval(node.operand)
            if isinstance(node.op, ast.UAdd):
                return +v
            if isinstance(node.op, ast.USub):
                return -v
            raise ValueError("Unsupported unary op")

        if isinstance(node, ast.BinOp):
            l, r = self._eval(node.left), self._eval(node.right)
            if   isinstance(node.op, ast.Add):  return l + r
            elif isinstance(node.op, ast.Sub):  return l - r
            elif isinstance(node.op, ast.Mult): return l * r
            elif isinstance(node.op, ast.Div):  return l / r
            elif isinstance(node.op, ast.Pow):  return l ** r

        raise ValueError("Unsupported node")
    
def theta_value(entry, q_rad):
    varmap = {"pi": np.pi}
    for i, qi in enumerate(q_rad, start=1):
        varmap[f"q{i}"] = float(qi)
    parser = SafeExpr(varmap)
    
    theta_raw = entry.get("theta", 0.0)
    theta_off_raw = entry.get("theta_offset", 0.0)
    
    theta_main = parser.eval(theta_raw)
    theta_off  = parser.eval(theta_off_raw)
    
    return theta_main + theta_off

def load_joints_from_json(path):
    with open(path, 'r', encoding="utf-8") as f:
        data = json.load(f)
    return data["dh_parameters"]["joints"]
        
class DataHub(Node):
    def __init__(self):
        super().__init__('data_hub')
        self.get_logger().info(" Data Hub On! ")
        
        self.sub_position = self.create_subscription(
            Int32MultiArray,
            '/motor/position',
            self.present_position_callback,
            10  
        )
        
        # self.sub_current = self.create_subscription(
        #     Float32MultiArray,
        #     '/motor/current',
        #     self.present_current_callback,
        #     10
        # )
        
        # self.sub_velocity = self.create_subscription(
        #     Float32MultiArray,
        #     '/motor/velocity',
        #     self.present_velocity_callback,
        #     10
        # )
        
        self.pub_gripper_pose = self.create_publisher(
            Float32MultiArray,
            '/datahub/grip_pose',
            10
        )
        
        self.pub_camera_pose = self.create_publisher(
            Float32MultiArray,
            '/datahub/cam_pose',
            10
        )
        
        self.dh_cam, self.dh_grip = self.read_json()
        
        # # JSON 읽어오기 확인 코드
        # print(json.dumps({"dh_parameters": {"joints": self.dh_cam}},
        #          ensure_ascii=False, indent=2))
        # print(json.dumps({"dh_parameters": {"joints": self.dh_grip}},
        #          ensure_ascii=False, indent=2))
        
        self.q_rad = [0.0, 0.0, 0.0, 0.0, 0.0]
        
    def read_json(self):
        camera_file = "base_to_camera_dh.json"
        gripper_file = "base_to_gripper_dh.json"
        pkg_name = "dongsoo_description"
        config_dir = None
        paths_tried = []
        
        if get_package_share_directory is not None:
            try:
                share_dir = get_package_share_directory(pkg_name)
                config_dir = os.path.join(share_dir, "config")
            except Exception as e:
                self.get_logger().warn(f"[ament_index] share 탐색 실패 : {e}")
                
        if config_dir is None:
            here = os.path.dirname(os.path.abspath(__file__))
            config_dir = os.path.normpath(os.path.join(here, "..", "..", "..", "dongsoo_description", "config"))
        
        cam_path = os.path.join(config_dir, camera_file)
        grip_path = os.path.join(config_dir, gripper_file)
        
        paths_tried.extend([cam_path, grip_path])
        
        missing = [p for p in [cam_path, grip_path] if not os.path.isfile(p)]
    
        if missing:
            msg = " / ".join(missing)
            raise FileNotFoundError(f"DH JSON 파일을 찾을 수가 없습니다. 확인 경로 : {paths_tried}")
        
        dh_cam = load_joints_from_json(cam_path)
        dh_grip = load_joints_from_json(grip_path)
        
        self.get_logger().info(f"Loaded DH: camera({len(dh_cam)} joints), gripper({len(dh_grip)} joints)")
        return dh_cam, dh_grip
    
    def fk(self, joints, q_rad):
        T = np.eye(4)
        for j in joints:
            th = theta_value(j, q_rad)
            d  = float(j.get("d", 0.0))
            a  = float(j.get("a", 0.0))
            al = float(j.get("alpha", 0.0))
            T  = T @ dh_transform(th, d, a, al)
            
        return T
    
    def present_position_callback(self, msg: Int32MultiArray):
        pulses = list(msg.data)
        q = [0.0]*5
        for i in range(min(5, len(pulses))):
            q[i] = float(pulses[i])
        self.q_rad = q

        # 카메라/그리퍼 FK
        T_cam  = self.fk(self.dh_cam,  self.q_rad)
        T_grip = self.fk(self.dh_grip, self.q_rad)

        # 간단 퍼블리시: x,y,z만
        cam_msg  = Float32MultiArray(data=[float(T_cam[0,3]),  float(T_cam[1,3]),  float(T_cam[2,3])])
        grip_msg = Float32MultiArray(data=[float(T_grip[0,3]), float(T_grip[1,3]), float(T_grip[2,3])])
        self.pub_camera_pose.publish(cam_msg)
        self.pub_gripper_pose.publish(grip_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = DataHub()
    
    exec = MultiThreadedExecutor(num_threads=4)
    exec.add_node(node)
    
    exec.spin()
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()