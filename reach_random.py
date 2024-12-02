import gymnasium as gym
import numpy as np
import time
import pygame

# 导入自定义环境
from Reach_env_2 import IRB360Env

def main():
    # 初始化pygame
    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption("IRB360Env Control")

    # 创建环境实例
    env = IRB360Env(tstep=0.05, distance_threshold=0.1, max_steps=1000, render_mode='human')

    try:
        # 重置环境
        state, _ = env.reset()
        print(f"Initial state: {state}")

        running = True
        while running:
            # 显示当前状态
            print(f"Current state: {state}")

            # 处理事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_w:
                        action = 0
                    elif event.key == pygame.K_s:
                        action = 1
                    elif event.key == pygame.K_a:
                        action = 2
                    elif event.key == pygame.K_d:
                        action = 3
                    elif event.key == pygame.K_q:
                        action = 4
                    elif event.key == pygame.K_e:
                        action = 5
                    else:
                        continue

                    # 执行动作
                    state, reward, terminated, truncated, info = env.step(action)
                    print(f"Action: {action}, Reward: {reward}, Terminated: {terminated}, Truncated: {truncated}, Info: {info}")

                    # 检查是否需要重置环境
                    if terminated or truncated:
                        print("Episode ended. Resetting environment...")
                        state, _ = env.reset()

                    # 渲染环境
                    env.render()

                    # 适当延时以观察效果
                    time.sleep(0.1)

    except KeyboardInterrupt:
        print("Test interrupted by user.")

    finally:
        # 关闭环境
        env.close()
        pygame.quit()

if __name__ == "__main__":
    main()