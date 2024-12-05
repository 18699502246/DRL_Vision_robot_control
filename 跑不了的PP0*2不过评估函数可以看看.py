import gymnasium as gym
from gymnasium import spaces
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter
from Reach_env_2 import IRB360Env
# 创建环境
env = IRB360Env()

# 定义强化学习模型
class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(ActorCritic, self).__init__()
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim)
        )
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 1)
        )

    def forward(self, state):
        action = self.actor(state)
        value = self.critic(state)
        return action, value

# 定义训练函数
def train(model, env, num_episodes, batch_size, learning_rate, gamma, ent_coef, vf_coef, max_grad_norm):
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)
    writer = SummaryWriter()

    for episode in range(num_episodes):
        states = []
        actions = []
        rewards = []
        values = []
        dones = []

        for _ in range(batch_size):
            state = env.reset()
            done = False
            rewards_episode = 0

            while not done:
                action, value = model(torch.tensor(state, dtype=torch.float32))
                next_state, reward, done, _ = env.step(action.item())
                rewards_episode += reward

                states.append(state)
                actions.append(action.item())
                rewards.append(reward)
                values.append(value.item())
                dones.append(done)

                state = next_state

            rewards.append(rewards_episode)

        states = torch.tensor(states, dtype=torch.float32)
        actions = torch.tensor(actions, dtype=torch.float32)
        rewards = torch.tensor(rewards, dtype=torch.float32)
        values = torch.tensor(values, dtype=torch.float32)
        dones = torch.tensor(dones, dtype=torch.float32)

        # 计算优势函数
        advantages = rewards - values

        # 计算损失函数
        policy_loss = -(advantages * actions).mean()
        value_loss = (rewards - values).pow(2).mean()
        entropy_loss = -(actions * actions.log()).mean()

        # 计算总损失函数
        loss = policy_loss + value_loss * vf_coef + entropy_loss * ent_coef

        # 更新模型参数
        optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(model.parameters(), max_grad_norm)
        optimizer.step()

        # 记录训练状况
        writer.add_scalar('Loss', loss.item(), episode)
        writer.add_scalar('Policy Loss', policy_loss.item(), episode)
        writer.add_scalar('Value Loss', value_loss.item(), episode)
        writer.add_scalar('Entropy Loss', entropy_loss.item(), episode)

    writer.close()

# 定义评估函数
def evaluate_model(model, env, num_episodes):
    rewards = []

    for _ in range(num_episodes):
        state = env.reset()
        done = False
        rewards_episode = 0

        while not done:
            action, _ = model(torch.tensor(state, dtype=torch.float32))
            next_state, reward, done, _ = env.step(action.item())
            rewards_episode += reward

            state = next_state

        rewards.append(rewards_episode)

    return np.mean(rewards)

# 训练模型
model = ActorCritic(env.observation_space.shape[0], env.action_space.shape[0])
train(model, env, 1000, 32, 0.001, 0.99, 0.01, 0.5, 0.5)

# 评估模型
rewards = evaluate_model(model, env, 100)
print('平均奖励：', rewards)