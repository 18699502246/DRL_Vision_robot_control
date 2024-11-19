import gymnasium as gym
from stable_baselines3 import PPO
from Reach_env import IRB360Env  # 假设IRB360Env环境代码在irb360_env.py文件中

def main():
    # 创建环境
    env = IRB360Env(render_mode='human')  # 将渲染模式设置为 human
    
    # 创建 PPO 模型
    model = PPO("MlpPolicy", env, verbose=1)

    # 训练模型
    model.learn(total_timesteps=100000)  # 进行100,000个时间步的训练

    # 保存模型
    model.save("irb360_ppo_model")

    # 测试训练后的模型
    obs = env.reset()
    for _ in range(1000):  # 运行1000个时间步
        action, _states = model.predict(obs)  # 使用模型预测动作
        obs, rewards, done, truncated, info = env.step(action)  # 执行动作
        
        env.render()  # 渲染环境
        if done or truncated:
            break  # 如果完成或截断，退出循环

    env.close()  # 关闭环境

if __name__ == "__main__":
    main()