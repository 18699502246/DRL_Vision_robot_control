import gymnasium as gym
from gymnasium import spaces
import numpy as np
import random
import logging
import time
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class IRB360Env(gym.Env):
    def __init__(self, 
                 tstep=0.1,
                 distance_threshold=0.1, 
                 max_steps=1000,
                 render_mode=None,
                 seed=None):
        super().__init__()
        
        # 基础配置保持不变
        self.tstep = tstep
        self.distance_threshold = distance_threshold
        self.max_steps = max_steps
        self.render_mode = render_mode
        self.Boundary_size = [0.9, 0.9, 0.6]
        
        # 初始化ZMQ Remote API客户端
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        
        # 状态跟踪变量
        self.current_step = 0
        self.current_Pad = None
        self.current_Block = None
        self.current_Boundary = None
        self.last_Pad = None
        self.last_time = None
        self.velocity = np.zeros(3)
        self.distance = None
        self.relative_distance = None
        
        # 性能监控
        self.reward_history = []
        self.distance_history = []
        self.step_times = []
        
        # 动作和观察空间定义
        self.action_space = spaces.Discrete(6)
        self.observation_space = spaces.Box(
            low=-2.0, 
            high=2.0, 
            shape=(17,), 
            dtype=np.float32
        )
        
        # 初始化仿真环境
        self.setup_simulation()
        
        # 设置随机数生成器
        self.seed(seed)
        
    def setup_simulation(self):
        """设置仿真环境"""
        # 设置仿真步长
        self.sim.setFloatParameter(
            self.sim.floatparam_simulation_time_step, 
            self.tstep
        )
        
        # 获取对象句柄
        self.handles = {
            'block': self.sim.getObject('/Cuboid'),
            'ik_tip': self.sim.getObject('/irb360_ikTip'),
            'base': self.sim.getObject('/irb360Base'),
            'pad': self.sim.getObject('/suctionPadLoopClosureDummy2'),
            'boundary': self.sim.getObject('/Boundary')
        }
        
        # 获取边界位置
        self.current_Boundary = self.get_object_position(self.handles['boundary'])
        
    def get_object_position(self, handle):
        """获取对象位置"""
        return self.sim.getObjectPosition(handle, -1)
        
    def step(self, action):
        """执行动作并返回下一个状态"""
        # 更新位置信息
        self.last_Pad = self.current_Pad
        self.calculate_distances()
        self.current_step += 1
        
        # 执行动作
        action_map = {
            0: 'moveUp', 
            1: 'moveDown',
            2: 'moveLeft', 
            3: 'moveRight',
            4: 'moveForward', 
            5: 'moveBackward'
        }
        
        function_name = action_map.get(action)
        if function_name:
            # 使用脚本函数执行动作
            self.sim.callScriptFunction(
                f'irb360/{function_name}@irb360', 
                self.sim.scripttype_childscript
            )
            
        # 获取新状态
        self.current_Pad = self.get_object_position(self.handles['pad'])
        self.velocity = self._calculate_velocity()
        
        # 检查状态
        if not self.check_ik_stability():
            self.sim.callScriptFunction(
                'irb360/moveBackToSafePosition@irb360',
                self.sim.scripttype_childscript
            )
            return self.state, 0.0, False, True, {'status': 'unstable_ik'}
            
        if not self.check_within_boundary(self.current_Pad):
            self.sim.callScriptFunction(
                'irb360/moveBackToSafePosition@irb360',
                self.sim.scripttype_childscript
            )
            return self.state, 0.0, False, True, {'status': 'boundary_exceeded'}
            
        # 计算奖励和状态
        reward = self._calculate_reward(self.current_Pad)
        self.state = self._get_extended_observation()
        
        # 判断终止条件
        terminated = self._is_terminated(self.current_Pad)
        truncated = self.current_step >= self.max_steps
        
        info = {
            'distance': self.distance,
            'reward': reward,
            'terminated': terminated,
            'truncated': truncated
        }
        
        return self.state, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        """重置环境状态"""
        super().reset(seed=seed)
        
        # 重置内部状态
        self.current_step = 0
        self.current_Pad = None
        self.last_time = None
        self.reward_history = []
        self.distance_history = []
        self.step_times = []
        
        # 重置机器人位置
        self.sim.callScriptFunction(
            'irb360/moveBack@irb360',
            self.sim.scripttype_childscript
        )
        
        # 随机化物块位置
        self.current_Block = self.randomize_block_position()
        
        # 获取初始观测
        self.state = self._get_extended_observation()
        
        return self.state, {}

    # 其他辅助方法保持基本不变,只需将sim.simx相关调用替换为self.sim的对应方法
    
    def close(self):
        """关闭环境"""
        # 保存性能数据
        np.save('reward_history.npy', self.reward_history)
        np.save('distance_history.npy', self.distance_history)
        
        # 关闭ZMQ连接
        self.client.close()