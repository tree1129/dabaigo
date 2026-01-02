import threading
import time
import queue
from datetime import datetime

# 导入我们之前写的三个模块
# 确保这三个 .py 文件在同一目录下
from spiritual_traction import HapticNavigation
from semantic_vision import SemanticBrain
from safety_sentinel import SafetySentinel

# --- 全局配置 ---
NETWORK_INTERFACE = "eth0"  # 如果用无线请改为 "wlan0"
USER_COMMAND = "带我找一个空闲的休息区" # 初始任务指令

# --- 1. 共享数据上下文 (线程安全) ---
class SharedContext:
    def __init__(self):
        self.lock = threading.Lock()
        
        # 导航状态
        self.nav_action = "stop"      # forward, turn_left, turn_right, stop
        self.nav_speed = 0.0
        self.target_detected = False
        
        # 环境语义 (用于安全报警时的描述)
        self.current_scene = "正在初始化视觉系统..."
        self.reasoning = ""
        
        # 安全状态 (最高优先级)
        self.emergency_triggered = False
        self.emergency_type = None

    def update_nav(self, action, speed, target_found, reasoning):
        with self.lock:
            self.nav_action = action
            self.nav_speed = speed
            self.target_detected = target_found
            self.reasoning = reasoning

    def update_scene(self, description):
        with self.lock:
            self.current_scene = description

    def trigger_emergency(self, type_str):
        with self.lock:
            self.emergency_triggered = True
            self.emergency_type = type_str

    def get_nav_state(self):
        with self.lock:
            return self.nav_action, self.nav_speed, self.emergency_triggered

    def get_scene_context(self):
        with self.lock:
            return self.current_scene

# 实例化全局共享对象
ctx = SharedContext()

# --- 2. 线程 A: 视觉感知 (大脑) ---
def vision_thread_func(brain):
    print(">>> [线程A] 视觉中枢已启动")
    while True:
        try:
            # 1. 如果处于紧急状态，暂停视觉分析以节省资源，或者专注寻找救援
            if ctx.emergency_triggered:
                time.sleep(1)
                continue

            # 2. 获取图像
            frame = brain._get_frame()
            if frame is None:
                time.sleep(0.5)
                continue

            # 3. 调用 VLM 进行分析 (这是一个耗时操作，约 1-3秒)
            # 注意：这里我们传入全局的 USER_COMMAND
            decision = brain.understand_scene(frame, USER_COMMAND)
            
            if decision:
                # 4. 更新共享数据
                ctx.update_nav(
                    action=decision.get('action_vector', {}).get('move', 'stop'),
                    speed=decision.get('action_vector', {}).get('speed', 0.0),
                    target_found=decision.get('target_detected', False),
                    reasoning=decision.get('reasoning', '')
                )
                
                ctx.update_scene(decision.get('scene_description', '未知环境'))
                
                print(f"[视觉] 👁️ {decision['scene_description']} -> 决策: {decision.get('action_vector', {}).get('move')}")

            # 控制帧率，避免 API 费用爆炸
            time.sleep(1.5) 

        except Exception as e:
            print(f"[视觉] 错误: {e}")
            time.sleep(1)

# --- 3. 线程 B: 安全守护 (神经反射) ---
def safety_thread_func(sentinel):
    print(">>> [线程B] 安全中枢已启动")
    while True:
        try:
            # 1. 实时读取传感器 (IMU)
            rpy = sentinel._get_imu_data()
            
            # 2. 检查数字胡须 (高位障碍)
            if sentinel.digital_whiskers_check():
                # 这是一个瞬时动作，不需要完全触发紧急停机，只需暂时阻拦
                # 这里我们可以选择直接覆盖 ctx 的状态，或者由 Control 线程处理
                # 为了简单，我们让 Sentinel 直接控制急停
                sentinel.sport_client.Stop()
                print("⚠️ [安全] 触发数字胡须阻拦！")
                time.sleep(1) # 暂停一秒
                continue

            # 3. 检查跌倒 (严重事故)
            if sentinel.detect_fall(rpy):
                ctx.trigger_emergency("FALL_DETECTED")
                # 获取当前环境语义用于报警
                current_scene = ctx.get_scene_context()
                sentinel.current_scene_context = current_scene # 更新 Sentinel 内部状态
                sentinel.activate_emergency_protocol("FALL_DETECTED")
                break # 跌倒后退出循环，等待人工重启

            # 4. 检查 SOS 按钮
            if sentinel.check_sos_button():
                ctx.trigger_emergency("MANUAL_SOS")
                current_scene = ctx.get_scene_context()
                sentinel.current_scene_context = current_scene
                sentinel.activate_emergency_protocol("MANUAL_SOS")
                break

            time.sleep(0.02) # 50Hz 高频扫描

        except Exception as e:
            print(f"[安全] 错误: {e}")
            time.sleep(0.1)

# --- 4. 线程 C: 运动控制 (小脑) ---
def control_thread_func(navigator):
    print(">>> [线程C] 灵觉牵引系统已启动")
    while True:
        try:
            # 1. 从共享上下文获取指令
            action, speed, is_emergency = ctx.get_nav_state()

            # 2. 优先级判断：如果是紧急状态，完全停止响应视觉指令
            if is_emergency:
                navigator.client.Stop()
                time.sleep(0.5)
                continue

            # 3. 执行“灵觉牵引”逻辑
            # 这里不再自己计算逻辑，而是翻译视觉指令为力反馈动作
            
            if action == "stop":
                navigator.client.Stop()
                
            elif action == "forward":
                # 调用牵引力模拟
                navigator.haptic_pull_forward(force_level=speed)
                
            elif action == "turn_left":
                # 调用侧向引导
                navigator.haptic_turn_guide("left", strength=speed)
                
            elif action == "turn_right":
                navigator.haptic_turn_guide("right", strength=speed)
                
            elif action == "backward" or action == "caution":
                # 遇到视觉识别出的危险 (非传感器触发) -> 阻尼反馈
                navigator.haptic_resistance_brake(urgency=0.8)

            # 控制频率 20Hz
            time.sleep(0.05)

        except Exception as e:
            print(f"[控制] 错误: {e}")
            navigator.client.Stop()
            time.sleep(1)

# --- 主程序入口 ---
def main():
    print("=========================================")
    print("   Unitree Go2 Edu - 具身智能导盲系统")
    print("   Initializing Modules...")
    print("=========================================")

    # 1. 初始化三大模块实例
    # 注意：这里我们复用之前写的类，但不再调用它们内部的死循环方法
    try:
        brain_module = SemanticBrain(network_interface=NETWORK_INTERFACE)
        sentinel_module = SafetySentinel(network_interface=NETWORK_INTERFACE, cloud_alert_url="YOUR_WEBHOOK_URL")
        nav_module = HapticNavigation(network_interface=NETWORK_INTERFACE)
    except Exception as e:
        print(f"初始化失败 (请检查网卡名称 {NETWORK_INTERFACE} 或 机器人连接): {e}")
        return

    # 2. 创建线程
    thread_vision = threading.Thread(target=vision_thread_func, args=(brain_module,), daemon=True)
    thread_safety = threading.Thread(target=safety_thread_func, args=(sentinel_module,), daemon=True)
    thread_control = threading.Thread(target=control_thread_func, args=(nav_module,), daemon=True)

    # 3. 启动线程
    thread_safety.start()  # 安全线程最先启动
    time.sleep(1)
    thread_control.start()
    thread_vision.start()

    print("\n✅ 所有系统已上线。按 Ctrl+C 终止程序。\n")

    # 4. 主线程保活与交互
    try:
        while True:
            # 这里可以用来接收键盘输入动态修改 USER_COMMAND
            # 或者单纯打印状态监控
            if ctx.emergency_triggered:
                print("!!! 系统处于紧急锁定状态 !!!")
                time.sleep(5)
            else:
                time.sleep(2)
    except KeyboardInterrupt:
        print("\n\n正在关闭系统...")
        # 线程是 daemon 模式，主线程退出它们也会退出
        # 但最好显式停止机器狗
        nav_module.client.Stop()
        print("已安全停机。")

if __name__ == "__main__":
    main()