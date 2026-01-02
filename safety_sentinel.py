import time
import json
import threading
import math
import requests
import os

# Unitree SDK2 依赖
from unitree_sdk2py.core.channel import ChannelFactory
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient

# 假设我们需要控制灯光来做声光报警
# 注意：具体控制灯光的 API 视固件版本可能有差异，这里模拟通用逻辑

class SafetySentinel:
    def __init__(self, network_interface="eth0", cloud_alert_url=None):
        self.channel_factory = ChannelFactory()
        self.channel_factory.init(0, network_interface)

        # 1. 运动控制客户端 (用于紧急刹车)
        self.sport_client = SportClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init(self.channel_factory)

        # 2. 机器人状态客户端 (获取 IMU 数据用于跌倒检测)
        self.state_client = RobotStateClient()
        self.state_client.SetTimeout(10.0)
        self.state_client.Init(self.channel_factory)

        self.cloud_alert_url = cloud_alert_url
        self.running = True
        
        # 模拟：上层应用传入的当前环境语义 (来自语义中枢)
        self.current_scene_context = "未知环境"
        
        print(">>> 智能安全中枢 (Safety Sentinel) 已启动")
        print(">>> 🛡️ 主动防御系统：在线")
        print(">>> 🚑 跌倒检测与 SOS：在线")

    def _get_imu_data(self):
        """获取姿态数据 (Roll, Pitch)"""
        # SDK2 中通常通过 GetRobotState 获取
        code, state = self.state_client.GetRobotState()
        if code == 0:
            # quaternion to euler or direct IMU reading
            # 这里简化假设直接获取到欧拉角 (弧度制)
            # 实际需根据 SDK 数据结构进行四元数转欧拉角运算
            return state.imu_state.rpy # [roll, pitch, yaw]
        return [0, 0, 0]

    def digital_whiskers_check(self):
        """
        [创新点]：数字胡须 (Digital Whiskers) - 高位风险预测
        逻辑：读取前置深度相机 '上半部分' 的数据，探测悬空障碍物（如树枝、路牌）。
        这通常是 Lidar 的盲区。
        """
        # 这里模拟读取深度相机的高位 ROI (Region of Interest)
        # 实际代码需调用 Intel RealSense SDK 或 Unitree VideoClient
        
        # 模拟：假设探测到前方 0.8米处，高度 1.5米处有障碍
        high_obstacle_detected = False 
        
        # 模拟偶尔触发
        # if random.random() < 0.01: high_obstacle_detected = True
        
        if high_obstacle_detected:
            print("⚠️ [数字胡须] 检测到高位悬挂物！触发阻拦式干预！")
            return True
        return False

    def detect_fall(self, rpy):
        """
        跌倒检测逻辑
        如果 Roll (横滚) 或 Pitch (俯仰) 超过 45度 (~0.78弧度)，视为跌倒
        """
        roll = abs(rpy[0])
        pitch = abs(rpy[1])
        threshold = 0.8 # 约 45 度
        
        if roll > threshold or pitch > threshold:
            return True
        return False

    def check_sos_button(self):
        """
        监听手柄/遥控器的 SOS 组合键
        """
        # 需通过 Unitree Joystick 接口读取
        # 模拟：检测到按键
        return False 

    def activate_emergency_protocol(self, trigger_type):
        """
        [核心]：一键响应与底线防御机制
        """
        print(f"\n!!! 🚨 紧急情况触发: {trigger_type} 🚨 !!!")
        
        # 1. 物理层：立刻锁死电机/急停
        print(">>> [动作] 紧急制动！")
        self.sport_client.Stop()
        self.sport_client.Damp() # 阻尼模式，防止溜车
        
        # 2. 本地交互层：声光报警
        # 模拟发出警报声 (需在机载电脑上安装 mpg123 或类似工具)
        print(">>> [本地] 开启高亮警示灯，播放 SOS 警报音...")
        # os.system("aplay /home/unitree/alert.wav &")
        
        # 3. 云端层：发送求救信号 (包含语义 + 坐标)
        self._send_cloud_alert(trigger_type)
        
        # 4. 保持报警状态直到人工复位
        time.sleep(5) 

    def _send_cloud_alert(self, trigger_type):
        """
        结合语义信息的智能求救
        """
        if not self.cloud_alert_url:
            print(">>> [云端] 未配置报警 webhook，仅本地记录。")
            return

        # 模拟 GPS 数据
        gps_location = {"lat": 39.9042, "lng": 116.4074} 
        
        payload = {
            "alert_type": "SOS_EMERGENCY",
            "trigger": trigger_type, # "FALL_DETECTED" or "MANUAL_SOS"
            "timestamp": time.time(),
            "location": gps_location,
            "environment_context": self.current_scene_context, # 关键：带上刚才 VLM 看到的画面描述
            "robot_id": "Go2_Edu_001",
            "message": f"用户在 {self.current_scene_context} 遭遇 {trigger_type}，请求立即支援！"
        }
        
        try:
            # 实际发送给你的后端或 Twilio/钉钉机器人
            # requests.post(self.cloud_alert_url, json=payload, timeout=2)
            print(f">>> [云端] 已向紧急联系人发送求救信息：\n{json.dumps(payload, indent=2, ensure_ascii=False)}")
        except Exception as e:
            print(f"云端报警失败: {e}")

    def start_protection_loop(self):
        """
        主循环：以极高频率 (50Hz) 运行
        """
        while self.running:
            # 1. 获取传感器数据
            rpy = self._get_imu_data()
            
            # 2. 检查数字胡须 (高位障碍)
            if self.digital_whiskers_check():
                # 这种情况下只刹车，不报警
                self.sport_client.Stop()
                # 这里可以结合文件1，发送一个“向后拉”的力反馈
                time.sleep(1) 
                continue

            # 3. 检查跌倒
            if self.detect_fall(rpy):
                self.activate_emergency_protocol("FALL_DETECTED (跌倒检测)")
                self.running = False # 停止循环，等待重启
                break

            # 4. 检查手动 SOS
            if self.check_sos_button():
                self.activate_emergency_protocol("MANUAL_SOS (用户手动求救)")
                self.running = False
                break
            
            # 维持心跳
            time.sleep(0.02) # 20ms

if __name__ == "__main__":
    # 模拟云端报警接口 (例如飞书/钉钉 webhook)
    webhook = "https://your-emergency-service.com/api/alert"
    
    sentinel = SafetySentinel(network_interface="eth0", cloud_alert_url=webhook)
    
    # 模拟更新环境语义 (实际应由文件2传入)
    sentinel.current_scene_context = "公园步道，周围光线昏暗"
    
    sentinel.start_protection_loop()