# 测试代码文件名：test_IRB360Env.py
import pygame
import sys
from Reach_env_2 import IRB360Env
import logging
import time

# 初始化 Pygame
pygame.init()

# 设置窗口大小
WINDOW_WIDTH, WINDOW_HEIGHT = 800, 600
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("IRB360 Environment Testing")

# 定义颜色
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

def print_state_info(state, reward, terminated, truncated, info):
    """打印状态信息"""
    print("\n" + "="*50)
    print(f"末端执行器位置: {state[0:3]}")
    print(f"目标块位置: {state[3:6]}")
    print(f"相对位置: {state[6:9]}")
    print(f"速度: {state[9:12]}")
    print(f"姿态: {state[12:15]}")
    print(f"距离: {state[15]}")
    print(f"归一化步数: {state[16]}")
    print(f"奖励: {reward}")
    print(f"是否终止: {terminated}")
    print(f"是否截断: {truncated}")
    print(f"额外信息: {info}")
    print("="*50 + "\n")

def print_controls():
    """打印控制说明"""
    print("\n控制说明:")
    print("W - 向上移动")
    print("S - 向下移动")
    print("A - 向左移动")
    print("D - 向右移动")
    print("Q - 向前移动")
    print("E - 向后移动")
    print("R - 重置环境")
    print("ESC - 退出程序")
    print("="*50)

def main():
    # 设置日志级别
    logging.basicConfig(level=logging.INFO)
    
    # 创建环境
    env = IRB360Env(render_mode='human', seed=7)
    
    # 初始重置
    state, _ = env.reset()
    
    # 打印控制说明
    print_controls()
    
    # 动作映射
    key_to_action = {
        pygame.K_w: 0,  # 上
        pygame.K_s: 1,  # 下
        pygame.K_a: 2,  # 左
        pygame.K_d: 3,  # 右
        pygame.K_q: 4,  # 前
        pygame.K_e: 5   # 后
    }
    
    running = True
    while running:
        # 事件处理
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            if event.type == pygame.KEYDOWN:  # 只在按下时响应
                if event.key == pygame.K_ESCAPE:
                    running = False
                    continue
                    
                if event.key == pygame.K_r:
                    state, _ = env.reset()
                    print("\n环境已重置")
                    continue
                
                if event.key in key_to_action:
                    action = key_to_action[event.key]
                    state, reward, terminated, truncated, info = env.step(action)
                    
                    # 打印状态信息
                    print_state_info(state, reward, terminated, truncated, info)
                    
                    # 检查是否需要重置
                    if terminated or truncated:
                        print("\n回合结束，原因：", "完成任务" if terminated else "回合截断")
                        if info.get('episode_status') == 'unstable_ik':
                            print("逆运动学不稳定导致重置")
                        elif info.get('episode_status') == 'boundary_exceeded':
                            print("超出边界导致重置")
                        
                        state, _ = env.reset()
                        print("\n环境已重置")

        # 填充屏幕颜色
        screen.fill(WHITE)
        pygame.display.flip()

        # 小延时以避免过快响应
        time.sleep(0.1)
    
    # 关闭环境
    env.close()
    pygame.quit()
    print("\n测试结束")

if __name__ == "__main__":
    main()