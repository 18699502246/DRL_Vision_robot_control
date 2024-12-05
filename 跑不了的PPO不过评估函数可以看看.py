import os
import time
import numpy as np
import matplotlib.pyplot as plt
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.logger import configure
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import EvalCallback, CallbackList, BaseCallback, CheckpointCallback
from stable_baselines3.common.evaluation import evaluate_policy
from loguru import logger
from Reach_env_2 import IRB360Env
import torch


"""_summary_
环境设置：使用IRB360Env作为训练和评估环境。
回调函数：定义了一个CustomCallback用于记录训练过程中的奖励信息。
训练循环：在每个训练周期中，使用CheckpointCallback保存模型检查点，并使用EvalCallback进行模型评估。
评估函数：定义了一个evaluate_model函数用于评估模型性能，并记录评估结果。
"""
print(torch.cuda.is_available())

now = time.strftime('%m%d-%H%M%S', time.localtime())

# 设置参数
models_dir = f"models/{now}"
logs_dir = f"logs/{now}"
checkpoints = f"checkpoints/{now}"

# 确保目录存在
if not os.path.exists(models_dir):
    os.makedirs(models_dir)
if not os.path.exists(logs_dir):
    os.makedirs(logs_dir)
if not os.path.exists(checkpoints):
    os.makedirs(checkpoints)

def evaluate_model(model, env, n_eval_episodes=100):
    success_count = 0
    episode_rewards = []

    for episode in range(n_eval_episodes):
        episode_reward = 0
        obs, _ = env.reset()
        done = False

        while not done:
            action, _ = model.predict(observation=obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            episode_reward += reward

            if done and info.get('is_success', False):
                success_count += 1

        episode_rewards.append(episode_reward)

    mean_reward = np.mean(episode_rewards)
    std_reward = np.std(episode_rewards)
    success_rate = success_count / n_eval_episodes

    return mean_reward, std_reward, success_rate

class CustomCallback(BaseCallback):
    def __init__(self, verbose=0):
        super(CustomCallback, self).__init__(verbose)
        self.n_envs = 1  # 假设只有一个环境
        self.episode_rewards = [0.0 for _ in range(self.n_envs)]
        self.episode_lengths = [0 for _ in range(self.n_envs)]
        self.episode_counts = [0 for _ in range(self.n_envs)]
        self.log_interval = 5  # 每5个回合记录一次

    def _on_step(self) -> bool:
        # 遍历所有环境
        for i in range(len(self.locals['rewards'])):
            self.episode_rewards[i] += self.locals['rewards'][i]
            self.episode_lengths[i] += 1

            # 检查回合是否结束
            if self.locals['dones'][i]:
                self.episode_counts[i] += 1

                # 每5个回合记录一次平均奖励
                if self.episode_counts[i] % self.log_interval == 0:
                    avg_reward = self.episode_rewards[i] / self.log_interval
                    self.model.logger.record(f"reward/env_{i}", avg_reward, exclude="stdout")
                    self.model.logger.dump(step=self.episode_counts[i])

                    # 重置累积奖励和回合长度
                    self.episode_rewards[i] = 0.0
                    self.episode_lengths[i] = 0

        return True

def make_env():
    def _init():
        env = IRB360Env(render_mode='human', seed=7)
        env = Monitor(env, logs_dir)
        return env
    set_random_seed(0)
    return _init

# 创建环境
num_train = 1
env = SubprocVecEnv([make_env() for _ in range(num_train)])

# 配置日志记录
new_logger = configure(logs_dir, ["stdout", "csv", "tensorboard"])

# 定义和训练代理
model = PPO(
    policy="MlpPolicy",
    env=env,
    learning_rate=0.0003,
    n_steps=2048,
    batch_size=2048,
    n_epochs=10,
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
    clip_range_vf=None,
    normalize_advantage=True,
    ent_coef=0,
    vf_coef=0.5,
    max_grad_norm=0.5,
    use_sde=True,
    sde_sample_freq=-1,
    target_kl=None,
    stats_window_size=100,
    tensorboard_log=logs_dir,
    policy_kwargs=dict(normalize_images=False),
    verbose=1,
    seed=None,
    device="cuda",
    _init_setup_model=True
)

model.set_logger(new_logger)

# 创建测试环境
eval_env = IRB360Env(render_mode='human', seed=7)
eval_env = Monitor(eval_env, logs_dir)

# 定义回调
eval_callback = EvalCallback(
    eval_env,
    best_model_save_path=models_dir,
    log_path=logs_dir,
    eval_freq=3000,
    deterministic=True,
    render=True,
    n_eval_episodes=100
)

tensorboard_callback = CustomCallback()

TIMESTEPS = 10000  # 根据需要调整

for episode in range(1000):
    checkpoint_callback = CheckpointCallback(save_freq=1000, save_path=checkpoints)
    model.learn(
        total_timesteps=TIMESTEPS,
        tb_log_name=f"PPO-run-episode{episode}",
        reset_num_timesteps=False,
        callback=CallbackList([eval_callback, tensorboard_callback]),
        log_interval=10
    )

    # 测试模型
    mean_reward, std_reward, success_rate = evaluate_model(model, eval_env, n_eval_episodes=100)
    # 记录测试结果
    model.logger.record("eval/mean_reward", mean_reward)
    model.logger.record("eval/std_reward", std_reward)
    model.logger.record('eval/success_rate', success_rate)
    model.logger.dump(step=episode)

    # 保存模型
    model.save(models_dir + f"/PPO-run-episode{episode}")
    logger.info(f"**************episode--{episode} saved**************")