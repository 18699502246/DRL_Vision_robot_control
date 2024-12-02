import torch
import numpy as np
from Reach_env_2 import IRB360Env
from agents.dqn_agent import DQNAgent
import matplotlib.pyplot as plt
from datetime import datetime
import os

def train():
    # 创建保存目录
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    save_dir = f'training_results_{timestamp}'
    os.makedirs(save_dir, exist_ok=True)
    
    # 环境和智能体初始化
    env = IRB360Env(render_mode=None)
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.n
    agent = DQNAgent(state_dim, action_dim)
    
    # 训练参数
    episodes = 1000
    max_steps = 1000
    
    # 记录训练过程
    episode_rewards = []
    episode_lengths = []
    success_rate = []
    success_window = 100
    successes = []
    
    for episode in range(episodes):
        state, _ = env.reset()
        episode_reward = 0
        steps = 0
        
        for step in range(max_steps):
            # 选择动作
            action = agent.select_action(state)
            
            # 执行动作
            next_state, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            
            # 存储经验
            agent.memory.push(state, action, reward, next_state, done)
            
            # 训练智能体
            loss = agent.train()
            
            state = next_state
            episode_reward += reward
            steps += 1
            
            if done:
                break
        
        # 记录episode信息
        episode_rewards.append(episode_reward)
        episode_lengths.append(steps)
        successes.append(1 if terminated else 0)
        
        # 计算成功率
        if episode >= success_window:
            success_rate.append(sum(successes[-success_window:]) / success_window)
        else:
            success_rate.append(sum(successes) / (episode + 1))
        
        # 每100个episodes保存模型和绘制图表
        if (episode + 1) % 100 == 0:
            agent.save(f'{save_dir}/model_episode_{episode+1}.pth')
            
            # 绘制训练曲线
            plt.figure(figsize=(15, 5))
            
            plt.subplot(131)
            plt.plot(episode_rewards)
            plt.title('Episode Rewards')
            plt.xlabel('Episode')
            plt.ylabel('Reward')
            
            plt.subplot(132)
            plt.plot(episode_lengths)
            plt.title('Episode Lengths')
            plt.xlabel('Episode')
            plt.ylabel('Steps')
            
            plt.subplot(133)
            plt.plot(success_rate)
            plt.title('Success Rate')
            plt.xlabel('Episode')
            plt.ylabel('Rate')
            
            plt.tight_layout()
            plt.savefig(f'{save_dir}/training_curves_episode_{episode+1}.png')
            plt.close()
            
            print(f'Episode {episode+1}/{episodes}')
            print(f'Average Reward: {np.mean(episode_rewards[-100:])}')
            print(f'Success Rate: {success_rate[-1]}')
            print(f'Epsilon: {agent.epsilon}')
            print('-' * 50)

if __name__ == '__main__':
    train()