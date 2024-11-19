import gym
from gym import spaces
import numpy as np
import math

class IRB360Env(gym.Env):
    def __init__(self):
        super(IRB360Env, self).__init__()
        # 定义动作空间：上下左右前后复位
        self.action_space = spaces.Discrete(7)  # 6个方向 + 1个复位
        # 状态空间：末端执行器的XYZ位置
        self.observation_space = spaces.Box(low=np.array([-1, -1, -1]), high=np.array([1, 1, 1]), dtype=np.float32)
        self.controller = IRB360Controller()  # 实例化控制器
        self.controller.connect()
        self.controller.start_simulation()
        self.initial_position = self.controller.initial_position

    def reset(self):
        """重置环境并返回初始状态"""
        self.controller.reset_to_initial_position()
        current_position = self.controller.get_current_position()
        return np.array(current_position, dtype=np.float32)

    def step(self, action):
        """执行动作并返回下一状态、奖励和是否完成"""
        if action == 0:  # 上
            self.controller.move('up')
        elif action == 1:  # 下
            self.controller.move('down')
        elif action == 2:  # 左
            self.controller.move('left')
        elif action == 3:  # 右
            self.controller.move('right')
        elif action == 4:  # 前
            self.controller.move('forward')
        elif action == 5:  # 后
            self.controller.move('backward')
        elif action == 6:  # 复位
            self.controller.reset_to_initial_position()

        current_position = self.controller.get_current_position()
        done, reward = self.calculate_reward(current_position)

        return np.array(current_position, dtype=np.float32), reward, done, {}

    def calculate_reward(self, position):
        """计算奖励"""
        target_position = np.array(self.initial_position)
        distance = np.linalg.norm(position - target_position)

        if distance < 0.1:  # 到达目标位置，给予奖励
            reward = 10.0
            done = True
        else:
            reward = -distance  # 距离越远，惩罚越大
            done = False

        return done, reward

    def render(self, mode='human'):
        """渲染环境，可选实现"""
        pass

    def close(self):
        """关闭环境"""
        self.controller.close()


def main():
    # 测试强化学习环境
    env = IRB360Env()
    observation = env.reset()
    for _ in range(100):
        action = env.action_space.sample()  # 随机选择动作
        observation, reward, done, _ = env.step(action)
        print(f"Observation: {observation}, Reward: {reward}")
        if done:
            print("Reached target position!")
            break
    env.close()

if __name__ == "__main__":
    main()
