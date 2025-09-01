# -*- coding: utf-8 -*-
"""
ros2_tool_chat_node.py
- ToolChatEngine 클래스를 import 하여 ROS2 노드에서 사용
- 입력:  /tool_chat/in        (std_msgs/String) : 사용자 자연어 명령/질문
- 출력1: /gemini_commands     (std_msgs/String) : {"mode": "...", "tool": "..."}
- 출력2: /tool_chat/out       (std_msgs/String) : 확인질문/설명/답변 등 텍스트 응답
"""
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os, sys
sys.path.append(os.path.dirname(__file__))
from llm_inference import ToolChatEngine

class ToolChatNode(Node):
    def __init__(self):
        super().__init__('tool_chat_node')

        # ── 파라미터 선언 ───────────────────────────────────
        self.declare_parameter('model_path', '/home/temp_id/LLM/gemma3-q4_k_m_budda.gguf')
        self.declare_parameter('rag_data_path', '/home/temp_id/SooMac/main_tool.json')
        self.declare_parameter('use_gpu_llama', False)
        self.declare_parameter('engine_verbose', True)

        model_path     = self.get_parameter('model_path').get_parameter_value().string_value
        rag_data_path  = self.get_parameter('rag_data_path').get_parameter_value().string_value
        use_gpu_llama  = self.get_parameter('use_gpu_llama').get_parameter_value().bool_value
        engine_verbose = self.get_parameter('engine_verbose').get_parameter_value().bool_value

        # ── 엔진 초기화(모델/RAG 1회 로드) ───────────────────
        self.engine = ToolChatEngine(
            model_path=model_path,
            rag_data_path=rag_data_path,
            use_gpu_llama=use_gpu_llama,
            verbose=engine_verbose
        )

        # ── 통신 설정 ────────────────────────────────────────
        self.sub_in   = self.create_subscription(String, '/tool_chat/in',  self.on_text, 10)
        self.pub_cmd  = self.create_publisher(String, '/control_commands', 10)
        self.pub_out  = self.create_publisher(String, '/tool_chat/out',   10)

        self.get_logger().info("ToolChatNode is up. Sub:/tool_chat/in  Pub:/gemini_commands, /tool_chat/out")

    # ── 콜백: 자연어 입력 수신 ───────────────────────────────
    def on_text(self, msg: String):
        user_text = msg.data.strip()
        if not user_text:
            return

        self.get_logger().info(f"[IN] {user_text}")

        # 1) 1차 파싱/추론
        parsed = self.engine.parse_and_infer(user_text)

        # 질문만 하는 경우: 답변 생성(RAG)
        if parsed.get("question_only", False):
            tool = parsed.get("tool_final")
            if tool and tool != "NONE":
                out = f"그 작업을 위한 공구는 [{tool}]입니다."
            else:
                try:
                    res = self.engine.rag_chain.invoke(user_text) if self.engine.rag_chain else {"result": "지식 베이스가 준비되지 않았습니다."}
                    out = (res.get('result', '') or '').strip()
                except Exception as e:
                    out = f"답변 생성 실패: {e}"
            self._publish_out(out)
            return

        # 인사/기타는 짧게 응답
        if parsed.get("intent") in ("인사말", "기타"):
            self._publish_out(self.engine.smalltalk_reply(user_text))
            return
        if parsed.get("intent") == "마무리":
            self._publish_out("세션을 종료합니다. 수고하셨어요!")
            return

        # 2) 실행 계획 산출(실행은 이 노드가 담당)
        plan = self.engine.plan_action(parsed, raw_text=user_text)

        # 확인질문 필요시
        if plan.get("ask_back") and plan.get("confirm_text"):
            self._publish_out(plan["confirm_text"])
            return

        # ── 여기서부터: 딱 mode/tool만 전송 ───────────────────
        # 실행 가능한 경우에만 /gemini_commands로 전달
        if plan.get("should_call_function"):
            payload = {
                "mode": plan.get("mode") or "NONE",
                "tool": plan.get("tool_final") or "NONE"
            }
            self._publish_cmd(payload)
            # 안내 로그
            self._publish_out(f"명령 전송: {payload}")
        else:
            # 실행 조건 부족 시 현재 상태 안내
            self._publish_out(f"모드={plan.get('mode')} / 도구={plan.get('tool_final')} → 추가 확인이 필요합니다.")

    # ── 퍼블리시 유틸 ────────────────────────────────────────
    def _publish_cmd(self, obj: dict):
        s = String()
        s.data = json.dumps(obj, ensure_ascii=False)  # {"mode":"...", "tool":"..."}
        self.pub_cmd.publish(s)
        self.get_logger().info(f"[CMD] {s.data}")

    def _publish_out(self, text: str):
        s = String(); s.data = text
        self.pub_out.publish(s)
        self.get_logger().info(f"[OUT] {text}")

def main():
    rclpy.init()
    node = ToolChatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
