import gymnasium as gym
import numpy as np
import sim  # CoppeliaSim Python API
import logging
import time

# 导入优化后的 IRB360Env 类
from Reach_env import IRB360Env

def test_environment():
    # 配置日志
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    try:
        # 创建环境对象
        env = IRB360Env(render_mode='human')  # 添加渲染模式

        # 连接到 CoppeliaSim
        try:
            env.connect()
            logger.info("成功连接到CoppeliaSim")
        except RuntimeError as e:
            logger.error(f"连接失败: {e}")
            return

        # 重置环境到初始状态
        try:
            initial_state, _ = env.reset()
            logger.info(f"初始状态: {initial_state}")
        except Exception as e:
            logger.error(f"环境重置失败: {e}")
            env.close()
            return

        # 执行测试动作序列
        actions = [0, 1, 2, 3, 4, 5]  # moveUp, moveDown, moveLeft, moveRight, moveForward, moveBackward
        
        for episode in range(2):  # 运行两个回合
            logger.info(f"开始第 {episode+1} 回合")
            
            for step, action in enumerate(actions):
                logger.info(f"执行动作: {action}")
                
                try:
                    # 执行动作并获取反馈
                    state, reward, done, truncated, info = env.step(action)
                    
                    # 渲染环境
                    env.render()
                    
                    # 打印详细信息
                    logger.info(f"步数: {step}")
                    logger.info(f"状态: {state}")
                    logger.info(f"奖励: {reward}")
                    logger.info(f"是否结束: {done}")
                    logger.info(f"额外信息: {info}")
                    
                    # 检查是否结束
                    if done or truncated:
                        logger.warning("环境已结束")
                        break
                
                except Exception as e:
                    logger.error(f"动作执行错误: {e}")
                    break
            
            # 重置环境继续下一回合
            env.reset()

    except Exception as e:
        logger.error(f"测试过程发生异常: {e}")
    
    finally:
        # 确保关闭环境
        env.close()
        logger.info("测试完成，环境已关闭")

def performance_test():
    """性能测试函数"""

    env = IRB360Env()
    start_time = time.time()
    
    total_steps = 0
    total_reward = 0

    try:
        for episode in range(5):  # 运行5个回合
            state, _ = env.reset()
            episode_reward = 0
            done = False
            
            while not done:
                action = env.action_space.sample()  # 随机动作
                state, reward, done, truncated, _ = env.step(action)
                
                episode_reward += reward
                total_steps += 1

                if done or truncated:
                    break
            
            total_reward += episode_reward
            print(f"Episode {episode+1} Reward: {episode_reward}")

    except Exception as e:
        print(f"性能测试发生错误: {e}")
    
    finally:
        env.close()

    end_time = time.time()
    print(f"总运行时间: {end_time - start_time:.2f}秒")
    print(f"总步数: {total_steps}")
    print(f"平均奖励: {total_reward/5:.2f}")

if __name__ == "__main__":
    # 运行标准测试
    test_environment()
    
    # 可选:运行性能测试
    # performance_test()