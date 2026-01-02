import time
import cv2
import base64
import json
import requests
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactory
from unitree_sdk2py.go2.video.video_client import VideoClient

# --- 配置区域 ---
# --- 🚀 通用 API 配置 (以硅基流动为例) ---
# 这是一个通用的 OpenAI 格式配置，适用于 DeepSeek, Qwen, Moonshot 等
API_CONF = {
    "api_key": "sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx",  # 替换你的 SiliconFlow Key
    "base_url": "https://api.siliconflow.cn/v1/chat/completions",
    "model_vision": "Qwen/Qwen2-VL-72B-Instruct",    # 用于看图 (VLM)
    "model_logic": "Qwen/Qwen2.5-72B-Instruct",      # 用于纯逻辑/JSON生成
}

class SemanticBrain:
    def __init__(self, network_interface="eth0"):
        self.channel_factory = ChannelFactory()
        self.channel_factory.init(0, network_interface)
        
        # 初始化摄像头
        self.video_client = VideoClient()
        self.video_client.SetTimeout(3.0)
        self.video_client.Init(self.channel_factory)
        
        print(">>> 视觉语义中枢 (Semantic Brain) 已启动")
        print(">>> 等待指令交互...")

    def _get_frame(self):
        """获取并预处理一帧图像"""
        code, data = self.video_client.GetImageSample()
        if code == 0:
            image_data = np.frombuffer(data[:], dtype=np.uint8)
            frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            # 调整大小以减少 Token 消耗并加快传输
            return cv2.resize(frame, (640, 480))
        return None

    def _encode_image(self, image):
        _, buffer = cv2.imencode('.jpg', image)
        return base64.b64encode(buffer).decode('utf-8')

    def understand_scene(self, image, user_command):
        """
        核心 VLM 逻辑：将图像 + 指令 -> 转化为结构化导航数据
        """
        base64_image = self._encode_image(image)
        
        # System Prompt 设计：这是让 AI 拥有“生活逻辑”的关键
        # 我们强制 AI 输出 JSON 格式，以便程序直接解析
        system_prompt = """
        你是一个导盲机器狗的视觉中枢。你需要根据用户的指令和看到的画面，规划下一步行动。
        
        请输出严格的 JSON 格式，包含以下字段：
        1. "scene_description": 简短描述当前场景（如：繁忙的街道，空旷的走廊）。
        2. "target_detected": (true/false) 用户寻找的目标是否出现在视野中？
        3. "safety_assessment": (safe/caution/danger) 当前路径的安全等级。
        4. "action_vector": 建议的行动向量。
           - "move": "forward" | "stop" | "turn_left" | "turn_right" | "backward"
           - "speed": 0.0 到 1.0 (推荐速度)
        5. "reasoning": 做出该决策的生活化理由。

        示例场景：用户说“找个空座”。
        画面：左前方有一个没人的长椅，前方有人群。
        输出：{"target_detected": true, "action_vector": {"move": "turn_left", "speed": 0.4}, "reasoning": "左前方发现空长椅，避开前方人群"}
        """

        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {API_KEY}"
        }

        payload = {
            "model": "gpt-4o", # 建议使用具有强大多模态能力的模型
            "messages": [
                {
                    "role": "system", 
                    "content": system_prompt
                },
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": f"当前指令：{user_command}"},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}}
                    ]
                }
            ],
            "response_format": {"type": "json_object"}, # 强制 JSON 模式
            "max_tokens": 300
        }

        try:
            response = requests.post(API_URL, headers=headers, json=payload)
            result = response.json()['choices'][0]['message']['content']
            return json.loads(result)
        except Exception as e:
            print(f"VLM 推理错误: {e}")
            return None

    def execute_mission(self, command):
        """
        执行一个持续的寻找任务
        """
        print(f"\n[收到任务]: \"{command}\"")
        print("正在扫描环境...")
        
        mission_active = True
        
        while mission_active:
            frame = self._get_frame()
            if frame is None:
                continue

            # 调用 VLM 理解场景
            # 注意：实际部署时，为了实时性，可以每秒处理 1 帧，中间帧用传统 CV 算法补间
            decision = self.understand_scene(frame, command)
            
            if decision:
                print(f"\n--- AI 视野分析 ---")
                print(f"👀 场景: {decision['scene_description']}")
                print(f"🧠 思考: {decision['reasoning']}")
                print(f"🤖 决策: {decision['action_vector']['move']} (速度: {decision['action_vector']['speed']})")

                # --- 这里是与文件1的接口 ---
                # 实际上，你会在这里调用 HapticNavigation 的方法
                # 例如：
                # if decision['action_vector']['move'] == 'turn_left':
                #     haptic_nav.haptic_turn_guide("left")
                # elif decision['action_vector']['move'] == 'forward':
                #     haptic_nav.haptic_pull_forward()
                
                if decision['target_detected'] and decision['action_vector']['move'] == 'stop':
                    print("✅ 任务完成：目标已找到并抵达。")
                    mission_active = False
            
            # 模拟处理延迟
            time.sleep(2) 

if __name__ == "__main__":
    # 模拟运行
    brain = SemanticBrain(network_interface="eth0") # 记得根据实际情况改 wlan0
    
    # 测试场景：模拟用户发出模糊指令
    # 场景 1：寻找休息处
    brain.execute_mission("我很累，带我找个没人的地方坐一下")
    
    # 场景 2：社会化场景（寻找电梯）
    # brain.execute_mission("带我去电梯口，我要上楼")