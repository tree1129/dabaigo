import time
import math
import threading
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactory
from unitree_sdk2py.go2.sport.sport_client import SportClient

class HapticNavigation:
    def __init__(self, network_interface="eth0"):
        """
        初始化灵觉牵引系统
        """
        self.channel_factory = ChannelFactory()
        # 默认使用 eth0，如果你已经连上 wifi，可以改为 wlan0
        self.channel_factory.init(0, network_interface)
        
        self.client = SportClient()
        self.client.SetTimeout(10.0)
        self.client.Init(self.channel_factory)

        self.running = True
        self.current_mode = "IDLE" 
        print(">>> 灵觉牵引系统 (Spiritual Traction) 已启动")
        print(">>> 请用户握紧牵引杆/牵引绳")

    def _apply_force(self, vx, vy, yaw_rate, body_height=0.0, roll=0.0, pitch=0.0):
        """
        底层驱动：将力矢量转化为机器人的运动指令
        vx: 前后速度 (模拟推背感/牵引力)
        vy: 左右速度 (模拟侧向引导)
        yaw_rate: 转向速度
        roll/pitch: 姿态角 (模拟重心偏移提示)
        """
        self.client.Move(vx, vy, yaw_rate)
        
        # 如果需要姿态反馈（例如震动或倾斜），需要调用 BalanceStand 或 Euler 模式
        # 这里为了简化，我们假设是在运动模式下叠加姿态（SDK2 某些模式支持）
        # 在实际 SDK2 中，Move 主要控制速度。若要控制姿态震动，可能需要切换到 Euler 模式或叠加。
        # 下面的逻辑主要通过微小的速度变化来模拟震动。

    def haptic_vibration(self, intensity=1.0, duration=0.5):
        """
        [创新点]：特定频率震动 - 用于提示障碍物或错误方向
        原理：通过快速交替的 Roll (横滚) 或 左右微移来产生震感传导到牵引杆
        """
        print(f"[触觉反馈] ⚠️ 震动提示 (强度: {intensity})")
        start_time = time.time()
        while time.time() - start_time < duration:
            # 通过快速切换左右横移速度产生抖动感
            # 频率约 10Hz
            self.client.Move(0.0, 0.2 * intensity, 0.0)
            time.sleep(0.05)
            self.client.Move(0.0, -0.2 * intensity, 0.0)
            time.sleep(0.05)
        
        # 归位
        self.client.Stop()

    def haptic_pull_forward(self, force_level=1.0):
        """
        [创新点]：灵觉前推 - 引导用户前进
        原理：持续、稳定的向前加速度，产生“牵引”感
        """
        print(f"[触觉反馈] ⬆️ 前向牵引 (等级: {force_level})")
        # 模拟 0.5m/s 的牵引速度，产生拉力
        target_vx = 0.5 * force_level
        self.client.Move(target_vx, 0.0, 0.0)

    def haptic_resistance_brake(self, urgency=1.0):
        """
        [创新点]：遇障碍回拉/阻尼 - 提示停止
        原理：机器人突然减速并稍稍后退，甚至抬头（Pitch Up）改变重心
        """
        print(f"[触觉反馈] 🛑 阻尼回拉 (紧急度: {urgency})")
        # 瞬间给一个向后的速度，模拟“撞墙”般的阻力
        self.client.Move(-0.3 * urgency, 0.0, 0.0)
        time.sleep(0.5) # 持续半秒的阻尼感
        self.client.Stop()

    def haptic_turn_guide(self, direction="left", strength=1.0):
        """
        [创新点]：侧向引导 - 引导转弯
        原理：不只是旋转，而是像导盲犬一样“侧身挤压”用户的腿或向一侧拉杆
        """
        vy = 0.3 * strength if direction == "left" else -0.3 * strength
        yaw = 0.5 * strength if direction == "left" else -0.5 * strength
        
        print(f"[触觉反馈] ↪️ 侧向引导: {direction}")
        # 混合 侧移(vy) 和 转向(yaw) 产生复合力场
        self.client.Move(0.2, vy, yaw)

    def calculate_interaction_vector(self, user_pos, target_pos, obstacles):
        """
        [AI 核心]：计算交互矢量
        这里模拟一个简单的势场法：目标产生引力，障碍产生斥力
        """
        # 1. 目标引力矢量 (简化计算)
        dx = target_pos[0] - user_pos[0]
        dy = target_pos[1] - user_pos[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        # 2. 障碍物斥力 (简化：假设检测到前方 1米有障碍)
        obstacle_force = 0.0
        if obstacles:
            obstacle_force = 1.0 # 触发强烈斥力
        
        return distance, dx, dy, obstacle_force

    def run_navigation_loop(self):
        """
        主循环：模拟从感知到力反馈的全过程
        """
        try:
            # 模拟：用户位置(0,0)，目标位置(10, 5)
            user_pos = [0, 0]
            target_pos = [10, 5]
            
            steps = 0
            
            while self.running:
                # --- 1. 感知阶段 (模拟) ---
                # 假设每 20 步遇到一个障碍
                has_obstacle = (steps % 50 == 40) 
                
                # --- 2. AI 计算阶段 ---
                dist, vec_x, vec_y, obs_force = self.calculate_interaction_vector(user_pos, target_pos, has_obstacle)
                
                # --- 3. 决策与力反馈执行阶段 ---
                
                if obs_force > 0.5:
                    # 场景 A: 遇到障碍 -> 产生“回拉”和“震动”
                    # 逻辑：先震动提醒，再产生阻尼
                    self.haptic_vibration(intensity=1.5, duration=0.4)
                    self.haptic_resistance_brake(urgency=1.0)
                    print(">>> 避障动作执行完毕，等待用户反应...")
                    time.sleep(1.0) 
                    
                elif dist < 0.5:
                    # 场景 B: 到达目标 -> 停止并轻微点头(这里用震动代替)
                    print(">>> 到达目的地")
                    self.client.Stop()
                    self.running = False
                    
                elif abs(vec_y) > 2.0 and steps % 10 == 0:
                    # 场景 C: 偏离路径需要转弯 -> 侧向力引导
                    # 模拟向左转
                    self.haptic_turn_guide(direction="left", strength=0.8)
                    
                else:
                    # 场景 D: 直行 -> 稳定的前向牵引力
                    # 根据距离调整拉力大小，距离越远拉力越大（但不超过阈值）
                    pull_strength = min(dist / 5.0, 1.0) 
                    self.haptic_pull_forward(force_level=pull_strength)
                    
                    # 模拟用户往前走了一点
                    user_pos[0] += 0.1
                    user_pos[1] += 0.05

                steps += 1
                time.sleep(0.1) # 控制循环频率 10Hz

        except KeyboardInterrupt:
            print("停止中...")
            self.client.Stop()

if __name__ == "__main__":
    # 确保你修改了网卡名称，如果是无线连接请改为 wlan0
    guide_dog = HapticNavigation(network_interface="eth0") 
    guide_dog.run_navigation_loop()