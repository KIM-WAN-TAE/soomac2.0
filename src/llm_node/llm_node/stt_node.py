# -*- coding: utf-8 -*-
import os, wave, json, shutil, subprocess, requests
import numpy as np
import sounddevice as sd

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# ▼ 실제 키는 환경변수/시크릿에 두세요
# CLIENT_ID = os.getenv("CSR_CLIENT_ID", "")
# CLIENT_SECRET = os.getenv("CSR_CLIENT_SECRET", "")
CLIENT_ID = os.getenv("CSR_CLIENT_ID", "ull29x0a8c")
CLIENT_SECRET = os.getenv("CSR_CLIENT_SECRET", "NpuNwNkx9Sr7ajeQJW434fS5xU3cGJcltWaVWbbj")
def list_microphones():
    print("\n사용 가능한 마이크 장치 목록 (sounddevice):")
    for i, d in enumerate(sd.query_devices()):
        if d.get("max_input_channels", 0) > 0:
            print(f"[{i}] {d['name']} (in={d['max_input_channels']}, defaultSR={int(d.get('default_samplerate', 0))})")

def select_logitech_microphone(target_keyword="C270"):
    devices = sd.query_devices()
    target_idx = None
    for i, d in enumerate(devices):
        if d.get("max_input_channels", 0) > 0 and target_keyword.lower() in d["name"].lower():
            target_idx = i; break
    if target_idx is None:
        try:
            target_idx = sd.default.device[0]
        except Exception:
            for i, d in enumerate(devices):
                if d.get("max_input_channels", 0) > 0:
                    target_idx = i; break
    if target_idx is None:
        raise RuntimeError("입력 가능한 오디오 장치를 찾을 수 없습니다.")
    print(f"사용할 장치 [{target_idx}] {devices[target_idx]['name']}")
    return target_idx

def arecord_fallback(filename="question.wav", seconds=5):
    if shutil.which("arecord") is None:
        raise RuntimeError("arecord가 설치되어 있지 않습니다. `sudo apt-get install alsa-utils`")
    cmd = ["arecord","-f","S16_LE","-r","16000","-c","1","-d",str(seconds),"-t","wav","-q",filename]
    print("arecord 폴백으로 녹음:", " ".join(cmd))
    subprocess.run(cmd, check=True)
    return filename

def record_audio_sd(filename="question.wav", seconds=5, device_index=None, samplerate=16000):
    try:
        sd.default.device = (device_index, None) if device_index is not None else (None, None)
        sd.default.dtype = "int16"
        print(f"🎙️ 녹음 시작 (sounddevice, dev={device_index}, sr={samplerate}Hz, ch=1)...")
        data = sd.rec(int(seconds * samplerate), samplerate=samplerate, channels=1, blocking=True)
        raw = data.astype(np.int16).tobytes()
        with wave.open(filename, "wb") as wf:
            wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(samplerate); wf.writeframes(raw)
        print("녹음 종료")
        return filename
    except Exception as e:
        print(f"sounddevice 녹음 실패 → arecord 폴백 시도: {e}")
        return arecord_fallback(filename, seconds)

def recognize_with_csr(audio_path, client_id, client_secret):
    url = "https://naveropenapi.apigw.ntruss.com/recog/v1/stt"
    headers = {
        "X-NCP-APIGW-API-KEY-ID": client_id,
        "X-NCP-APIGW-API-KEY": client_secret,
        "Content-Type": "application/octet-stream",
    }
    params = {"lang": "Kor", "completion": "sync"}
    with open(audio_path, "rb") as f:
        audio_data = f.read()
    resp = requests.post(url, headers=headers, params=params, data=audio_data)
    if resp.status_code == 200:
        try:
            return resp.json().get("text", "")
        except json.JSONDecodeError:
            return resp.text.strip()
    else:
        print("CSR STT 오류:", resp.status_code); print(resp.text); return ""

class STTPublisher(Node):
    def __init__(self):
        super().__init__("stt_publisher")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, 
        )
        self.pub = self.create_publisher(String, "/tool_chat/in", qos)

    def wait_for_subscribers(self, timeout_sec=5.0, poll=0.1):
        elapsed = 0.0
        while self.pub.get_subscription_count() == 0 and elapsed < timeout_sec:
            rclpy.spin_once(self, timeout_sec=poll)
            elapsed += poll
        return self.pub.get_subscription_count() > 0

    def publish_text(self, text: str):
        msg = String(); msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f"[PUB -> /tool_chat/in] {text}")

def main(args=None):
    # 1) 장치 선택 및 녹음
    try:
        dev_idx = select_logitech_microphone("C270")
    except RuntimeError as e:
        print(e); return
    wav_file = record_audio_sd("question.wav", seconds=5, device_index=dev_idx, samplerate=16000)

    # 2) STT
    if not CLIENT_ID or not CLIENT_SECRET:
        print("CLIENT_ID / CLIENT_SECRET 환경변수(CSR_CLIENT_ID / CSR_CLIENT_SECRET)를 설정하세요.")
        recognized = ""
    else:
        recognized = recognize_with_csr(wav_file, CLIENT_ID, CLIENT_SECRET)
    print("인식된 질문:", recognized if recognized else "(없음)")

    # 3) ROS2 퍼블리시 (디스커버리 대기 + 송신 후 스핀)
    rclpy.init(args=args)
    node = STTPublisher()
    try:
        if recognized:
            # 구독자 매칭 대기
            matched = node.wait_for_subscribers(timeout_sec=5.0, poll=0.1)
            if not matched:
                node.get_logger().warn("구독자를 찾지 못했습니다. 그래도 1회 송신합니다.")
            node.publish_text(recognized)
            # 전달 보장 위해 잠깐 더 스핀
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            node.get_logger().warn("인식된 텍스트가 없어 퍼블리시하지 않았습니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
