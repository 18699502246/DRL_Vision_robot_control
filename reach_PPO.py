import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import DummyVecEnv
from Reach_env import IRB360Env

"""
    当前不足：运行没有反馈信息，不知道运行到第几步了，需要添加进度条显示
            训练过程没有保存模型，需要添加保存模型功能
            
"""
def train_ppo_on_irb360():
    # 创建环境实例
    env = IRB360Env(render_mode='human')

    # 检查环境
    check_env(env, warn=True)

    # 使用DummyVecEnv包装环境
    vec_env = DummyVecEnv([lambda: env])

    # 配置PPO模型
    model = PPO(
        "MlpPolicy",  # 使用多层感知器策略
        vec_env, 
        verbose=1,  # 打印详细训练信息
        learning_rate=1e-3,  # 学习率
        n_steps=2048,  # 每次更新的步数
        batch_size=64,  # 批次大小
        n_epochs=10,  # 每次更新的训练轮数
        gamma=0.99,  # 折扣因子
        gae_lambda=0.95,  # 广义优势估计lambda
        clip_range=0.2  # PPO剪裁范围
    )

    try:
        # 开始训练
        model.learn(total_timesteps=100)

        # 保存模型
        model.save("ppo_irb360")
        print("训练完成，模型已保存")

    except Exception as e:
        print(f"训练过程出现异常: {e}")

    finally:
        # 关闭环境
        env.close()

if __name__ == "__main__":
    train_ppo_on_irb360()