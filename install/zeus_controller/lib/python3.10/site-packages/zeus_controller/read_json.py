#!/usr/bin/env python3

import os
import json
import ast
import numpy as np

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None


class SafeExpr:
    """JSON 파일의 string 타입을 안전하게 평가하는 클래스"""
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
        if isinstance(node, ast.Constant):
            v = node.value
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


class CameraDHParameters:
    """카메라 DH 파라미터를 관리하는 클래스"""
    
    def __init__(self):
        """
        카메라 DH 파라미터 초기화 (6-DOF)
        """
        self.joints_data = self._load_json()
        
    def _find_config_directory(self):
        """설정 파일 디렉토리를 찾는 메서드"""
        pkg_name = "zeus_description"
        config_dir = None
        
        if get_package_share_directory is not None:
            try:
                share_dir = get_package_share_directory(pkg_name)
                config_dir = os.path.join(share_dir, "config")
            except Exception:
                pass
                
        if config_dir is None:
            here = os.path.dirname(os.path.abspath(__file__))
            config_dir = os.path.normpath(os.path.join(here, "..", "..", "..", "zeus_description", "config"))
        
        return config_dir
    
    def _load_json(self):
        """base_to_camera_dh.json 파일을 로드하는 메서드"""
        config_dir = self._find_config_directory()
        json_file = "base_to_camera_dh.json"
        json_path = os.path.join(config_dir, json_file)
        
        if not os.path.isfile(json_path):
            raise FileNotFoundError(f"DH JSON 파일을 찾을 수 없습니다: {json_path}")
        
        with open(json_path, 'r', encoding="utf-8") as f:
            data = json.load(f)
        
        return data["dh_parameters"]["joints"]
    
    def get_joint_count(self):
        """조인트 개수를 반환"""
        return len(self.joints_data)
    
    def get_joint_parameter(self, joint_id, parameter_name):
        """특정 조인트의 특정 파라미터 값을 반환"""
        for joint in self.joints_data:
            if joint["joint_id"] == joint_id:
                return joint.get(parameter_name, 0.0)
        raise ValueError(f"Joint ID {joint_id}를 찾을 수 없습니다")
    
    def get_all_parameters(self, joint_id):
        """특정 조인트의 모든 파라미터를 딕셔너리로 반환"""
        for joint in self.joints_data:
            if joint["joint_id"] == joint_id:
                return joint.copy()
        raise ValueError(f"Joint ID {joint_id}를 찾을 수 없습니다")
    
    def get_parameter_list(self, parameter_name):
        """모든 조인트의 특정 파라미터 리스트를 반환"""
        return [joint.get(parameter_name, 0.0) for joint in self.joints_data]
    
    def get_dh_matrix_params(self, joint_id, q_values=None):
        """
        특정 조인트의 DH 변환 행렬 계산을 위한 파라미터들을 반환
        
        Args:
            joint_id (int): 조인트 ID
            q_values (list): 조인트 각도 값들 (라디안) - 6-DOF
        
        Returns:
            dict: theta, d, a, alpha 값들
        """
        joint_data = self.get_all_parameters(joint_id)
        
        if q_values is None:
            q_values = [0.0] * 6
        
        # theta 값 계산 (6-DOF)
        varmap = {"pi": np.pi}
        for i, qi in enumerate(q_values[:6], start=1):
            varmap[f"q{i}"] = float(qi)
        parser = SafeExpr(varmap)
        
        theta_raw = joint_data.get("theta", 0.0)
        theta_offset_raw = joint_data.get("theta_offset", 0.0)
        
        theta_main = parser.eval(theta_raw)
        theta_offset = parser.eval(theta_offset_raw)
        theta_total = theta_main + theta_offset
        
        # alpha 값 계산
        alpha_raw = joint_data.get("alpha", 0.0)
        alpha = parser.eval(alpha_raw)
        
        return {
            "theta": theta_total,
            "d": float(joint_data.get("d", 0.0)),
            "a": float(joint_data.get("a", 0.0)),
            "alpha": alpha
        }
    
    def get_all_dh_params(self, q_values=None):
        """모든 조인트의 DH 파라미터를 반환 (6-DOF)"""
        if q_values is None:
            q_values = [0.0] * 6
            
        all_params = []
        for joint in self.joints_data:
            joint_id = joint["joint_id"]
            params = self.get_dh_matrix_params(joint_id, q_values)
            params["joint_id"] = joint_id
            all_params.append(params)
        
        return all_params