# -*- coding: utf-8 -*-
"""
app_node.py
- Flask-SocketIO 웹 UI 서버
- ROS2 통합
- STT(Clova) → LLM → TTS
"""
from flask import Flask, request, render_template, jsonify
from flask_socketio import SocketIO, emit
import sys, threading, os, json, warnings, time, wave, requests, numpy as np, sounddevice as sd, re
from typing import Optional, Dict, Any
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from llm_inference import ToolChatEngine

warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", category=DeprecationWarning)
warnings.filterwarnings("ignore", category=FutureWarning)

# ----- Flask 앱 -----
BASE_DIR = Path("/home/liw/zeus_ws/src/llm_node/llm_node")
TEMPLATE_DIR = str(BASE_DIR / "templates")
STATIC_DIR   = str(BASE_DIR / "static")

app = Flask(__name__, template_folder=TEMPLATE_DIR, static_folder=STATIC_DIR, static_url_path="/static")
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins="*")

print("TEMPLATE_DIR =", TEMPLATE_DIR)
print("STATIC_DIR   =", STATIC_DIR)

# ----- STT 설정 -----
CLIENT_ID = os.getenv("CSR_CLIENT_ID", "ull29x0a8c")
CLIENT_SECRET = os.getenv("CSR_CLIENT_SECRET", "NpuNwNkx9Sr7ajeQJW434fS5xU3cGJcltWaVWbbj")

# ----- QoS -----
QOS_LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
QOS_RELIABLE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

# ----- cm 파싱 -----
_CM_UNITS = r'(?:cm|센치|센티|센티미터|센치미터)'
def _to_float_num(s: str) -> float: return float(s.replace(",", ""))
def parse_length_cm(text: str) -> Optional[float]:
    if text is None: return None
    t = str(text).lower().replace(" ", "").replace("점", ".")
    m = re.search(rf'(-?\d+(?:[.,]\d+)?)\s*(?:~|-)\s*(-?\d+(?:[.,]\d+)?)(?:\s*{_CM_UNITS})?$', t)
    if m: a, b = m.groups(); return (_to_float_num(a) + _to_float_num(b)) / 2.0
    m = re.search(rf'(-?\d+(?:[.,]\d+)?)(?:\s*{_CM_UNITS})?$', t)
    if m: return _to_float_num(m.group(1))
    return None

# ----- 도구/모드 -----
NIPPER_SYNS = ["니퍼","니뻐","니빠","롱노즈","롱노즈 플라이어","롱노우즈 플라이어","롱 플라이어","롱플라이어","needle nose","needle-nose","long nose","long-nose","nipper"]
TOOL_EN_MAP = {**{k: "nipper" for k in NIPPER_SYNS},
               "커터":"wire_cutter","와이어 스트리퍼":"wire_stripper","M3 나사":"M3","스탠드":"lamp",
               "wire_cutter":"wire_cutter","wire_stripper":"wire_stripper","M3":"M3","lamp":"lamp","NONE":"NONE"}
TOOL_KR_MAP = {"nipper":"니퍼","wire_cutter":"커터","wire_stripper":"와이어 스트리퍼","M3":"M3 나사","lamp":"스탠드","NONE":""}
ALLOWED_MODES = {"START","UP","DOWN","DELIVER","RETURN","DISASSEMBLE","MEASURE","FINISH"}
DIR_KR = {"left":"왼쪽","right":"오른쪽","front":"앞쪽","back":"뒤쪽","center":"정중앙","NONE":""}

def _tool_kr_for_speech(tool_en: str) -> str:
    kr = TOOL_KR_MAP.get(tool_en, tool_en or "")
    return "엠쓰리 나사" if kr.upper().startswith("M3") else kr
def _has_batchim(k: str) -> bool:
    if not k: return False
    ch = k.strip()[-1]; code = ord(ch) - 0xAC00
    return 0 <= code <= 11171 and (code % 28) != 0
def _eulreul(word: str) -> str: return "을" if _has_batchim(word) else "를"
def _delivery_speech(tool_en: str, target_en: str, direction: str) -> str:
    tool_k = _tool_kr_for_speech(tool_en)
    target_k = _tool_kr_for_speech(target_en) if target_en != "NONE" else ""
    dir_k = DIR_KR.get(direction, "")
    if target_en != "NONE" and direction != "NONE": return f"네, {tool_k}{_eulreul(tool_k)} {target_k} {dir_k}에 가져다 드릴게요."
    if target_en != "NONE": return f"네, {tool_k}{_eulreul(tool_k)} {target_k}에 가져다 드릴게요."
    if direction != "NONE": return f"네, {tool_k}{_eulreul(tool_k)} {dir_k}에 가져다 드릴게요."
    return f"네, {tool_k}{_eulreul(tool_k)} 바로 가져다 드릴게요."

# ----- STT 유틸 -----
def list_microphones():
    devices = []
    for i, d in enumerate(sd.query_devices()):
        if d.get("max_input_channels", 0) > 0:
            devices.append({'index': i,'name': d['name'],'channels': d['max_input_channels'],'samplerate': int(d.get('default_samplerate', 0))})
    return devices
def select_microphone(target_keyword="C270"):
    devices = sd.query_devices(); target_idx = None
    for i, d in enumerate(devices):
        if d.get("max_input_channels", 0) > 0 and target_keyword.lower() in d["name"].lower():
            target_idx = i; break
    if target_idx is None:
        try: target_idx = sd.default.device[0]
        except Exception:
            for i, d in enumerate(devices):
                if d.get("max_input_channels", 0) > 0: target_idx = i; break
    if target_idx is None: raise RuntimeError("입력 가능한 오디오 장치를 찾을 수 없습니다.")
    return target_idx, devices[target_idx]['name']
def record_audio_sd(filename="question.wav", seconds=5, device_index=None, samplerate=16000):
    sd.default.device = (device_index, None) if device_index is not None else (None, None)
    sd.default.dtype = "int16"
    data = sd.rec(int(seconds * samplerate), samplerate=samplerate, channels=1, blocking=True)
    raw = data.astype(np.int16).tobytes()
    with wave.open(filename, "wb") as wf:
        wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(samplerate); wf.writeframes(raw)
    return filename
def recognize_with_csr(audio_path, client_id, client_secret):
    url = "https://naveropenapi.apigw.ntruss.com/recog/v1/stt"
    headers = {"X-NCP-APIGW-API-KEY-ID": client_id,"X-NCP-APIGW-API-KEY": client_secret,"Content-Type": "application/octet-stream"}
    params = {"lang": "Kor", "completion": "sync"}
    with open(audio_path, "rb") as f: audio_data = f.read()
    resp = requests.post(url, headers=headers, params=params, data=audio_data)
    if resp.status_code == 200:
        try: return resp.json().get("text", "")
        except json.JSONDecodeError: return resp.text.strip()
    return ""

# ----- ROS2 통합 -----
class WebUINode(Node):
    def __init__(self, use_external_nodes=False):
        super().__init__('web_ui_node')
        self.use_external_nodes = use_external_nodes
        self._last_out_text = ""
        self._last_out_ts = 0.0
        self._stt_lock = threading.Lock()

        if not use_external_nodes:
            self.declare_parameter('model_path', '/home/liw/sllm/models/gemma3-q4_k_m_budda.gguf')
            self.declare_parameter('mode_rag_jsonl_path', '/home/temp_id/ros2_ws/src/vision_node/rule.jsonl')
            self.declare_parameter('faiss_index_path', './faiss_index_ros')
            self.declare_parameter('use_gpu_llama', True)
            self.declare_parameter('engine_verbose', True)
            model_path = self.get_parameter('model_path').get_parameter_value().string_value
            mode_rag_jsonl = self.get_parameter('mode_rag_jsonl_path').get_parameter_value().string_value
            faiss_index_path = self.get_parameter('faiss_index_path').get_parameter_value().string_value
            use_gpu_llama = self.get_parameter('use_gpu_llama').get_parameter_value().bool_value
            engine_verbose = self.get_parameter('engine_verbose').get_parameter_value().bool_value
            self.get_logger().info('[LLM] ToolChatEngine 초기화 중...')
            self.chat_engine = ToolChatEngine(model_path=model_path, mode_rag_jsonl_path=mode_rag_jsonl, faiss_index_path=faiss_index_path, use_gpu_llama=use_gpu_llama, verbose=engine_verbose)
            self.get_logger().info('[LLM] ToolChatEngine 초기화 완료')
        else:
            self.chat_engine = None
            self.get_logger().info('[외부 노드] ros2_chat_node/tts_node 활용')

        self.control_pub = self.create_publisher(String, '/zeus/string/llm_cmd', QOS_RELIABLE)
        self.chat_in_pub = self.create_publisher(String, '/tool_chat/in', QOS_LATCHED)
        self.pub_length = self.create_publisher(Float32, '/zeus/float/length_val', QOS_LATCHED)
        if not use_external_nodes:
            self.chat_out_pub = self.create_publisher(String, '/tool_chat/out', QOS_RELIABLE)

        if not use_external_nodes:
            self.chat_in_sub = self.create_subscription(String, '/tool_chat/in', self.chat_callback, QOS_LATCHED)

        self.chat_out_sub = self.create_subscription(String, '/tool_chat/out', self.chat_out_callback, QOS_RELIABLE)
        self.tool_detection_sub = self.create_subscription(String, '/tool_detections', self.tool_detection_callback, QOS_RELIABLE)
        self.tts_status_sub = self.create_subscription(String, '/tts/status', self.tts_status_callback, QOS_RELIABLE)

        self.last_tool_info = {}
        self.last_chat_response = ""
        self.tts_status = {"playing": False, "last_text": ""}

        self.stt_recording = False
        self.selected_microphone = None
        self.microphone_devices = []
        try:
            self.microphone_devices = list_microphones()
            if self.microphone_devices:
                self.selected_microphone, _ = select_microphone()
                self.get_logger().info(f'STT 마이크 선택됨: {self.selected_microphone}')
        except Exception as e:
            self.get_logger().warning(f'STT 마이크 초기화 실패: {e}')

    # --- ROS2 콜백 → UI 이벤트 ---
    def chat_callback(self, msg):
        if self.use_external_nodes: return
        user_input = (msg.data or "").strip()
        if not user_input: return
        self.get_logger().info(f"[IN] {user_input}")
        _ = self.process_chat_message(user_input)   # 여기서 emit 하지 않음

    def chat_out_callback(self, msg):
        now = time.time()
        text = msg.data or ""

        # 0.6초 이내 동일 문구 재수신 시 무시
        if text == self._last_out_text and (now - self._last_out_ts) < 0.6:
            return
        self._last_out_text, self._last_out_ts = text, now

        self.last_chat_response = text
        self.get_logger().info(f'[응답 수신] {text}')
        socketio.emit('external_chat_response', {'response': text, 'timestamp': now})

    def tool_detection_callback(self, msg):
        try:
            tool_data = json.loads(msg.data)
            self.last_tool_info = tool_data
            socketio.emit('tool_detection', tool_data)
        except Exception as e:
            self.get_logger().warning(f'도구 감지 데이터 파싱 실패: {e}')

    def tts_status_callback(self, msg):
        try:
            status_data = json.loads(msg.data)
            self.tts_status.update(status_data)
            socketio.emit('tts_status', {'status': self.tts_status,'timestamp': time.time()})
            self.get_logger().info(f'[TTS 상태] {self.tts_status}')
        except Exception as e:
            self.get_logger().warning(f'TTS 상태 데이터 파싱 실패: {e}')

    # --- 내부 유틸 ---
    def _to_en_tool(self, tool: str) -> str:
        if not tool: return "NONE"
        tool = tool.strip()
        return TOOL_EN_MAP.get(tool, tool)

    def _map_control_with_tool(self, control: str, mode: str, tool_en: str, direction: str, target_en: str) -> str:
        if mode not in ("DELIVER","RETURN"): return control or (mode or "")
        if not control:
            base = f"{tool_en}+{mode}" if tool_en != "NONE" else mode
            if mode == "DELIVER":
                if target_en and target_en != "NONE": base += f"+{target_en}"
                if direction and direction != "NONE": base += f"+{direction}"
            return base
        parts = control.split("+")
        if len(parts) >= 2:
            parts[0] = tool_en or parts[0]
            if mode == "DELIVER":
                if target_en and target_en != "NONE":
                    if len(parts) < 3: parts.append(target_en)
                    elif parts[2] not in TOOL_EN_MAP.values(): parts[2] = target_en
                if direction and direction != "NONE":
                    if len(parts) < 4: parts.append(direction)
                    else:
                        if parts[-1] not in ("left","right","front","back","center"):
                            parts.append(direction)
            return "+".join(parts)
        return control

    # --- 핵심 처리 ---
    def process_chat_message(self, user_input: str) -> Dict[str, Any]:
        val_cm = parse_length_cm(user_input)
        if val_cm is not None:
            out = Float32(); out.data = float(val_cm)
            self.pub_length.publish(out)
            self.get_logger().info(f"[LENGTH_CM] {out.data:.3f} cm")
            return {'mode':'LENGTH','tool':'NONE','target':'NONE','direction':'NONE','nlg': f'{val_cm:.1f}센치미터로 설정했습니다.','plan_action': f'LENGTH:{val_cm}cm'}

        if self.use_external_nodes:
            try:
                chat_msg = String(); chat_msg.data = user_input
                self.chat_in_pub.publish(chat_msg)
                self.get_logger().info(f'[외부 노드로 전송] {user_input}')
                return {'mode':'EXTERNAL','tool':'NONE','target':'NONE','direction':'NONE','nlg':'외부 노드에서 처리 중...','plan_action':'외부 처리'}
            except Exception:
                return {'mode':'ERROR','tool':'NONE','target':'NONE','direction':'NONE','nlg':'외부 노드 통신 오류가 발생했습니다.','plan_action':'오류 발생'}
        else:
            try:
                parsed = self.chat_engine.parse_and_infer(user_input)
                plan = self.chat_engine.plan_action(parsed, raw_text=user_input)

                if plan.get("ask_back") and plan.get("confirm_text"):
                    confirm_text = plan["confirm_text"]
                    if hasattr(self, "chat_out_pub"):
                        m = String(); m.data = confirm_text; self.chat_out_pub.publish(m)
                    return {'mode':'ASK_BACK','tool':'NONE','target':'NONE','direction':'NONE','nlg':confirm_text,'plan_action':'확인 필요'}

                if plan.get("should_call_function"):
                    mode = plan.get("mode","NONE")
                    tool_kr = plan.get("tool_final","NONE")
                    target_kr = plan.get("target_object","NONE")
                    direction = plan.get("direction") or "NONE"
                    control = plan.get("control_string") or ""

                    if mode not in ALLOWED_MODES:
                        error_msg = "가능한 Mode: START | UP | DOWN | DELIVER | RETURN | DISASSEMBLE | MEASURE | FINISH"
                        if hasattr(self, "chat_out_pub"):
                            m = String(); m.data = error_msg; self.chat_out_pub.publish(m)
                        return {'mode':'ERROR','tool':'NONE','target':'NONE','direction':'NONE','nlg':error_msg,'plan_action':'모드 오류'}

                    tool_en = self._to_en_tool(tool_kr)
                    target_en = self._to_en_tool(target_kr)
                    dir_kr = DIR_KR.get(direction, "")
                    control_mapped = self._map_control_with_tool(control, mode, tool_en, direction, target_en)

                    if mode == 'DELIVER':
                        payload_direction = direction if direction not in (None,"NONE") else ('straight' if tool_en=='M3' else 'down')
                    elif mode == 'RETURN':
                        payload_direction = 'down'
                    elif mode in ('UP','DOWN'):
                        payload_direction = 'straight'
                    else:
                        payload_direction = 'NONE'

                    if tool_en == target_en:
                        target_en = 'NONE'

                    payload = {
                        "mode": mode,
                        "tool": tool_en if mode in ("DELIVER","RETURN") else "NONE",
                        "direction": payload_direction if mode == "DELIVER" else "NONE",
                        "target": target_en if mode == "DELIVER" and target_en != "NONE" else "NONE",
                        "control": control_mapped,
                    }
                    msg = String(); msg.data = json.dumps(payload, ensure_ascii=False)
                    self.control_pub.publish(msg)
                    self.get_logger().info(f"[CONTROL] {control_mapped}")

                    try:
                        if mode == "DELIVER" and tool_en != "NONE":
                            speech = _delivery_speech(tool_en, target_en, direction)
                        else:
                            intent_details = f"모드: {mode}"
                            if mode in ("UP","DOWN"): intent_details += ", 도구: 스탠드"
                            if tool_en != "NONE" and mode in ("DELIVER","RETURN"):
                                intent_details += f", 도구: {_tool_kr_for_speech(tool_en)}"
                            if target_en != "NONE" and mode == "DELIVER":
                                intent_details += f", 배치 대상: {_tool_kr_for_speech(target_en)}"
                            if direction != "NONE" and mode == "DELIVER":
                                intent_details += f", 방향: {dir_kr}"
                            speech = self.chat_engine.generate_speech(task_description=intent_details, style="친절하고 간결하게")
                    except Exception:
                        speech = "알겠습니다. 지시에 따르겠습니다."

                    if hasattr(self, "chat_out_pub"):
                        m = String(); m.data = speech; self.chat_out_pub.publish(m)

                    # HTTP 경로로 호출돼도 UI에 동일 이벤트 송출
                    socketio.emit('ros2_response', {'input': user_input, 'response': speech, 'control': payload})

                    return {'mode':mode,'tool':tool_en,'target':target_en,'direction':payload_direction,'nlg':speech,'plan_action':control_mapped,'control':control_mapped}

                mode = plan.get("mode","NONE")
                error_msg = "모드는 인식했지만 작업에 필요한 정보가 더 필요합니다." if mode!="NONE" else "요청을 이해하지 못했습니다. 다시 한번 말씀해 주세요."
                if hasattr(self, "chat_out_pub"):
                    m = String(); m.data = error_msg; self.chat_out_pub.publish(m)
                return {'mode':mode,'tool':'NONE','target':'NONE','direction':'NONE','nlg':error_msg,'plan_action':'정보 부족'}

            except Exception:
                error_msg = '죄송합니다. 요청을 처리하는 중 오류가 발생했습니다.'
                if hasattr(self, "chat_out_pub"):
                    m = String(); m.data = error_msg; self.chat_out_pub.publish(m)
                socketio.emit('ros2_response', {'input': user_input, 'response': error_msg, 'control': None})
                return {'mode':'ERROR','tool':'NONE','target':'NONE','direction':'NONE','nlg':error_msg,'plan_action':'오류 발생'}

    def start_stt_recording(self, duration=5):
        if self.stt_recording: return {'success': False, 'error': '이미 녹음 중입니다.'}
        if self.selected_microphone is None: return {'success': False, 'error': '사용 가능한 마이크가 없습니다.'}
        try:
            self.stt_recording = True
            def record_and_recognize():
                with self._stt_lock:
                    try:
                        os.makedirs("/tmp/claude", exist_ok=True)
                        temp_file = f"/tmp/claude/stt_recording_{int(time.time())}.wav"
                        record_audio_sd(temp_file, duration, self.selected_microphone)
                        recognized_text = recognize_with_csr(temp_file, CLIENT_ID, CLIENT_SECRET) if (CLIENT_ID and CLIENT_SECRET) else ""
                    except Exception as e:
                        recognized_text = ""
                    finally:
                        try: os.remove(temp_file)
                        except: pass
                        self.stt_recording = False

                    # 결과 알림: 성공이면 텍스트, 실패면 빈 문자열
                    if recognized_text:
                        _ = self.process_chat_message(recognized_text)
                        socketio.emit('stt_result', {'recognized_text': recognized_text})
                    else:
                        socketio.emit('stt_result', {'recognized_text': ''})
            threading.Thread(target=record_and_recognize, daemon=True).start()
            return {'success': True, 'duration': duration}
        except Exception as e:
            self.stt_recording = False
            return {'success': False, 'error': str(e)}

    def get_microphone_info(self):
        return {'devices': self.microphone_devices,'selected': self.selected_microphone,'recording': self.stt_recording}

    def set_microphone(self, device_index):
        try:
            if device_index < 0 or device_index >= len(sd.query_devices()):
                return {'success': False, 'error': '잘못된 장치 인덱스입니다.'}
            device = sd.query_devices()[device_index]
            if device.get("max_input_channels", 0) <= 0:
                return {'success': False, 'error': '선택한 장치는 입력을 지원하지 않습니다.'}
            self.selected_microphone = device_index
            self.get_logger().info(f'마이크 변경됨: {device["name"]}')
            return {'success': True, 'device_name': device['name']}
        except Exception as e:
            return {'success': False, 'error': str(e)}

# ----- 전역 및 가드 -----
ros2_node: Optional[WebUINode] = None
ros2_executor: Optional[MultiThreadedExecutor] = None
ros2_thread: Optional[threading.Thread] = None
_ros_inited = False
_shutting_down = False

def init_ros2(use_external_nodes=False):
    global ros2_node, ros2_executor, ros2_thread, _ros_inited
    if _ros_inited:
        return True
    try:
        print(f"[ROS2] 초기화 중... (외부 노드 모드: {use_external_nodes})")
        rclpy.init()
        _ros_inited = True
        ros2_node = WebUINode(use_external_nodes=use_external_nodes)
        ros2_executor = MultiThreadedExecutor(); ros2_executor.add_node(ros2_node)
        def run_ros2():
            try: ros2_executor.spin()
            except Exception as e: print(f"[ROS2] 실행 중 오류: {e}")
        ros2_thread = threading.Thread(target=run_ros2, daemon=True); ros2_thread.start()
        print("[ROS2] 초기화 완료"); return True
    except Exception as e:
        print(f"[ROS2] 초기화 실패: {e}")
        import traceback; traceback.print_exc()
        _ros_inited = False
        return False

def shutdown_ros2():
    global ros2_node, ros2_executor, ros2_thread, _ros_inited, _shutting_down
    if _shutting_down:
        return
    _shutting_down = True
    try:
        if ros2_executor:
            try: ros2_executor.shutdown()
            except: pass
        if ros2_node:
            try: ros2_node.destroy_node()
            except: pass
        if _ros_inited and rclpy.ok():
            try: rclpy.shutdown()
            except: pass
        if ros2_thread and ros2_thread.is_alive():
            ros2_thread.join(timeout=1.0)
        print("[ROS2] 정리 완료")
    finally:
        ros2_node = None; ros2_executor = None; ros2_thread = None
        _ros_inited = False; _shutting_down = False

# ----- Flask 라우트 -----
@app.route("/")
def index(): return render_template("index.html")

@app.route("/chat", methods=["POST"])
def chat():
    data = request.get_json()
    user_input = data.get('message', '') if data else ''
    if not ros2_node: return jsonify({'reply': "ROS2 노드가 초기화되지 않았습니다."}), 500
    try:
        result = ros2_node.process_chat_message(user_input)
        return jsonify({'reply': result.get('nlg', '응답을 생성할 수 없습니다.'),'control': result})
    except Exception:
        return jsonify({'reply': "메시지 처리 오류가 발생했습니다."}), 500

@app.route("/status", methods=["GET"])
def status():
    microphone_info = ros2_node.get_microphone_info() if ros2_node else {}
    return jsonify({'ros2_initialized': ros2_node is not None,
                    'ros2_running': ros2_thread is not None and ros2_thread.is_alive(),
                    'last_tool_info': getattr(ros2_node, 'last_tool_info', {}) if ros2_node else {},
                    'stt_info': microphone_info})

@app.route("/send_ros2", methods=["POST"])
def send_ros2():
    data = request.get_json()
    message = data.get('message', '')
    if not ros2_node: return jsonify({'success': False, 'error': 'ROS2 노드가 초기화되지 않았습니다.'})
    try:
        result = ros2_node.process_chat_message(message)
        return jsonify({'success': True, 'result': result})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})

@app.route("/stt/start", methods=["POST"])
def start_stt():
    if not ros2_node: return jsonify({'success': False, 'error': 'ROS2 노드가 초기화되지 않았습니다.'}), 500
    data = request.get_json() or {}
    duration = data.get('duration', 5)
    result = ros2_node.start_stt_recording(duration)
    return jsonify(result)

@app.route("/stt/microphones", methods=["GET"])
def get_microphones():
    try:
        devices = list_microphones()
        return jsonify({'success': True, 'devices': devices})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})

@app.route("/stt/microphone", methods=["POST"])
def set_microphone():
    if not ros2_node: return jsonify({'success': False, 'error': 'ROS2 노드가 초기화되지 않았습니다.'}), 500
    data = request.get_json()
    device_index = data.get('device_index')
    if device_index is None: return jsonify({'success': False, 'error': '장치 인덱스가 필요합니다.'})
    result = ros2_node.set_microphone(device_index)
    return jsonify(result)

@app.route("/mode", methods=["POST"])
def switch_mode():
    data = request.get_json()
    use_external = data.get('use_external_nodes', False)
    try:
        shutdown_ros2()
        if init_ros2(use_external_nodes=use_external):
            return jsonify({'success': True,'mode': ("외부 노드" if use_external else "내장"),'use_external_nodes': use_external})
        else:
            return jsonify({'success': False, 'error': 'ROS2 재초기화 실패'})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})

# ----- SocketIO -----
@socketio.on('connect')
def handle_connect():
    emit('system_status', {'ros2_initialized': ros2_node is not None,
                           'ros2_running': ros2_thread is not None and ros2_thread.is_alive(),
                           'timestamp': time.time()})
    if ros2_node and hasattr(ros2_node, 'last_tool_info'):
        emit('tool_detection', ros2_node.last_tool_info)

@socketio.on('disconnect')
def handle_disconnect(): pass

@socketio.on('text_message')
def handle_text_message(data):
    user_input = data.get('message', '')
    if not ros2_node:
        emit('bot_reply', {'reply': 'ROS2 노드가 초기화되지 않았습니다.'}); return
    try:
        result = ros2_node.process_chat_message(user_input)
        emit('bot_reply', {'reply': result.get('nlg', '응답을 생성할 수 없습니다.'),'control': result})
    except Exception:
        emit('bot_reply', {'reply': '메시지 처리 중 오류가 발생했습니다.'})

@socketio.on('request_tool_info')
def handle_tool_info_request():
    emit('tool_detection', getattr(ros2_node, 'last_tool_info', {}) if ros2_node else {})

@socketio.on('send_control_command')
def handle_control_command(data):
    if not ros2_node:
        emit('command_result', {'success': False, 'error': 'ROS2 노드가 아직 아님'}); return
    try:
        control_msg = String(); control_msg.data = json.dumps(data, ensure_ascii=False)
        ros2_node.control_pub.publish(control_msg)
        emit('command_result', {'success': True, 'command': data})
    except Exception as e:
        emit('command_result', {'success': False, 'error': str(e)})

# ----- 엔트리포인트 -----
def main():
    print("[서버] ROS2 통합 Flask-SocketIO 서버를 시작합니다.")
    if not init_ros2(use_external_nodes=False):
        print("[오류] ROS2 초기화 실패"); sys.exit(1)
    try:
        socketio.run(app, host="0.0.0.0", port=5000, debug=True, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_ros2()

if __name__ == "__main__":
    main()
