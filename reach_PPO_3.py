import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
import random
from collections import deque
from Reach_env import IRB360Env

# 2. 定义 DQN 模型
class DQN(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(input_dim, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc3 = nn.Linear(128, output_dim)
    
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x
    
    
# 3. 定义经验回放缓冲区
class ReplayBuffer:
    def __init__(self, capacity):
        self.buffer = deque(maxlen=capacity)
    
    def push(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))
    
    def sample(self, batch_size):
        state, action, reward, next_state, done = zip(*random.sample(self.buffer, batch_size))
        return np.array(state), np.array(action), np.array(reward), np.array(next_state), np.array(done)
    
    def __len__(self):
        return len(self.buffer)
    
    
# 4. 定义 DQN Agent
class DQNAgent:
    def __init__(self, env, learning_rate=0.001, gamma=0.99, buffer_capacity=10000, batch_size=64):
        self.env = env
        self.gamma = gamma
        self.batch_size = batch_size
        self.replay_buffer = ReplayBuffer(buffer_capacity)
        
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        self.policy_net = DQN(env.observation_space.shape[0], env.action_space.n).to(self.device)
        self.target_net = DQN(env.observation_space.shape[0], env.action_space.n).to(self.device)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()
        
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=learning_rate)
    
    def select_action(self, state, epsilon):
        if random.random() < epsilon:
            return self.env.action_space.sample()
        else:
            with torch.no_grad():
                state = torch.tensor([state], device=self.device, dtype=torch.float32)
                q_values = self.policy_net(state)
                return q_values.argmax().item()
    
    def update_policy(self):
        if len(self.replay_buffer) < self.batch_size:
            return
        
        state, action, reward, next_state, done = self.replay_buffer.sample(self.batch_size)
        
        state = torch.tensor(state, device=self.device, dtype=torch.float32)
        action = torch.tensor(action, device=self.device, dtype=torch.int64)
        reward = torch.tensor(reward, device=self.device, dtype=torch.float32)
        next_state = torch.tensor(next_state, device=self.device, dtype=torch.float32)
        done = torch.tensor(done, device=self.device, dtype=torch.float32)
        
        q_values = self.policy_net(state).gather(1, action.unsqueeze(1)).squeeze(1)
        next_q_values = self.target_net(next_state).max(1)[0].detach()
        expected_q_values = reward + self.gamma * next_q_values * (1 - done)
        
        loss = F.mse_loss(q_values, expected_q_values)
        
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
    
    def update_target_network(self):
        self.target_net.load_state_dict(self.policy_net.state_dict())
        
        
        
# 5. 训练循环
def train(agent, num_episodes=1000, max_steps=1000, epsilon_start=1.0, epsilon_end=0.01, epsilon_decay=0.995):
    epsilon = epsilon_start
    for episode in range(num_episodes):
        state, _ = agent.env.reset()
        episode_reward = 0
        for step in range(max_steps):
            action = agent.select_action(state, epsilon)
            next_state, reward, done, truncated, _ = agent.env.step(action)
            agent.replay_buffer.push(state, action, reward, next_state, done)
            agent.update_policy()
            
            state = next_state
            episode_reward += reward
            
            if done or truncated:
                break
        
        if episode % 10 == 0:
            agent.update_target_network()
            print(f"Episode {episode}, Reward: {episode_reward}, Epsilon: {epsilon:.4f}")
        
        epsilon = max(epsilon_end, epsilon * epsilon_decay)
        
        
# 6. 创建环境并训练       
        
if __name__ == "__main__":
    env = IRB360Env(tstep=0.05, distance_threshold=0.1, max_steps=1000)
    agent = DQNAgent(env)
    train(agent)
    
    
    
    
# 7. 保存和加载模型
def save_model(agent, filename="dqn_model.pth"):
    torch.save(agent.policy_net.state_dict(), filename)

def load_model(agent, filename="dqn_model.pth"):
    agent.policy_net.load_state_dict(torch.load(filename))
    agent.target_net.load_state_dict(agent.policy_net.state_dict())
    
   
# 8. 测试模型 
def test(agent, num_episodes=10):
    for episode in range(num_episodes):
        state, _ = agent.env.reset()
        episode_reward = 0
        for step in range(1000):
            action = agent.select_action(state, epsilon=0.0)  # Greedy policy
            next_state, reward, done, truncated, _ = agent.env.step(action)
            state = next_state
            episode_reward += reward
            
            if done or truncated:
                break
        print(f"Test Episode {episode}, Reward: {episode_reward}")
        
        
        
# 9.运行测试
if __name__ == "__main__":
    env = IRB360Env(tstep=0.05, distance_threshold=0.1, max_steps=1000)
    agent = DQNAgent(env)
    train(agent)
    save_model(agent)
    load_model(agent)
    test(agent)