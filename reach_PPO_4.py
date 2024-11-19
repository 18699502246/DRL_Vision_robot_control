import gymnasium as gym
from stable_baselines3 import PPO
from Reach_env import IRB360Env  # 确保您的环境在这个路径下可用
import numpy as np

def main():
    # 创建环境
    env = IRB360Env(render_mode='human')  # 如果需要可视化, 否则可将 render_mode 设为 None

    # 创建 PPO 模型
    model = PPO("MlpPolicy", env, verbose=1)

    # 训练智能体
    model.learn(total_timesteps=20000)  # 您可以根据需要调整训练步数

    # 测试智能体
    obs = env.reset()
    for _ in range(1000):  # 测试 1000 步
        action, _states = model.predict(obs)
        obs, rewards, dones, truncated, info = env.step(action)
        env.render()  # 可视化

    env.close()

if __name__ == "__main__":
    main()
