import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
from Reach_env_2 import IRB360Env
import numpy as np

# 创建多个环境实例
def make_env():
    def _thunk():
        env = IRB360Env()
        return env
    return _thunk

# 包装环境
def create_vec_env(num_envs=4):
    envs = [make_env() for _ in range(num_envs)]
    vec_env = SubprocVecEnv(envs)
    return vec_env

# 定义评估函数
def evaluate_model(model, env, n_eval_episodes=10):
    success_count = 0
    episode_rewards = []

    for episode in range(n_eval_episodes):
        episode_reward = 0
        obs = env.reset()
        done = False

        while not done:
            action, _ = model.predict(observation=obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            episode_reward += reward

            if done and info.get('is_success', True):
                success_count += 1

        episode_rewards.append(episode_reward)

    mean_reward = np.mean(episode_rewards)
    std_reward = np.std(episode_rewards)
    success_rate = success_count / n_eval_episodes

    return mean_reward, std_reward, success_rate

if __name__ == '__main__':
    # 初始化环境
    vec_env = create_vec_env()

    # 初始化PPO模型
    model = PPO('MlpPolicy', vec_env, verbose=1)

    # 训练模型
    total_timesteps = 10000
    model.learn(total_timesteps=total_timesteps)

    # 评估模型
    mean_reward, std_reward, success_rate = evaluate_model(model, vec_env.envs[0], n_eval_episodes=10)
    print(f"Mean reward: {mean_reward}, Std reward: {std_reward}, Success rate: {success_rate}")