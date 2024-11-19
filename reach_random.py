import gymnasium as gym
from Reach_env import IRB360Env
import numpy as np

def random_exploration_in_environment():
    # 创建环境实例
    env = IRB360Env(render_mode='human')

    # 重置环境以获得初始状态
    obs = env.reset()
    random_exploration_steps = 1000  # 随机探索的时间步数

    try:
        # 随机探索阶段
        for _ in range(random_exploration_steps):
            action = env.action_space.sample()  # 随机选择动作
            obs, reward, terminated,truncated, info = env.step(action)  # 执行动作
            
            # 输出反馈信息
            print(f"状态: {obs}, 奖励: {reward}, 完成标志: {terminated},中断标志:{truncated}, 信息: {info}")

            # 如果环境完成，重置
            if terminated:
                obs = env.reset()

    except Exception as e:
        print(f"探索过程中出现异常: {e}")

    finally:
        # 关闭环境
        env.close()

if __name__ == "__main__":
    random_exploration_in_environment()