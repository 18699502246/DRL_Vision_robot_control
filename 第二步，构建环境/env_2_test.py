import gymnasium as gym
import numpy as np
import sim  # CoppeliaSim Python API

# 导入优化后的 IRB360Env 类
from irb360_env_2 import IRB360Env

# from irb360_env import IRB360Env

def test_environment():
    # 创建环境对象
    env = IRB360Env()

    # 连接到 CoppeliaSim
    
    if not env.connect():
    # if not env.connect_to_server():    
        print("Failed to connect to CoppeliaSim. Exiting test.")
        return

    # 启动仿真
    
    if not env.start_simulation():
        print("Failed to start simulation. Exiting test.")
        env.close()
        return

    # 重置环境到初始状态
    initial_state, _ = env.reset()
    print("Initial State:", initial_state)

    # 执行一些动作并获取新的状态和奖励
    actions = [0, 1, 2, 3, 4, 5]  # 分别对应 moveUp, moveDown, moveLeft, moveRight, moveForward, moveBackward
    for action in actions:
        print(f"Executing action: {action}")
        state, reward, done, truncated, info = env.step(action)
        print(f"State: {state}, Reward: {reward}, Done: {done}, Truncated: {truncated}, Info: {info}")

        # 如果动作执行失败，停止仿真并退出测试
        if done or truncated:
            print("Environment ended or truncated. Stopping simulation.")
            break

    # 停止仿真
    if not env.stop_simulation():
        print("Failed to stop simulation.")

    # 关闭 CoppeliaSim 客户端
    env.close()

if __name__ == "__main__":
    test_environment()