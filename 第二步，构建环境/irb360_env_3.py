import gymnasium as gym
from gymnasium import spaces
import numpy as np
import random
import sim  # CoppeliaSim Python API
import logging
import time
import matplotlib.pyplot as plt

class IRB360Env(gym.Env):
    def __init__(self, 
                 tstep=0.05, 
                 distance_threshold=0.1,
                 max_steps=1000,
                 render_mode=None):
        super().__init__()
        
        # 添加性能监控
        self.reward_history = []
        self.distance_history = []
        self.step_times = []
        
        # 添加速度计算所需的历史位置
        self.last_position = None
        self.last_time = None
        
        # 配置参数
        self.tstep = tstep
        self.distance_threshold = distance_threshold
        self.max_steps = max_steps
        self.current_step = 0
        self.render_mode = render_mode
        
        # 日志配置
        logging.basicConfig(level=logging.INFO)
        
        # 仿真连接和初始化
        self.client_id = -1
        self.robot_name = 'irb360'
        self.script_type = sim.sim_scripttype_childscript
        
        # 连接并设置仿真环境
        self.connect()
        
        # 动作和观测空间的定义
        self.action_space = spaces.Discrete(6)
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(9,), dtype=np.float32)
        
        # 初始化句柄和状态
        self.handles = {
            'block': None,
            'ik_tip': None,
            'base': None,
            'pad': None
        }
        self.state = None
        
        # 设置仿真
        self.setup_simulation()
        
        # 随机数生成器
        self.np_random = np.random.default_rng()
    
    def connect(self):
        """连接到仿真环境"""
        sim.simxFinish(-1)
        self.client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
        
        if self.client_id == -1:
            logging.error('Failed to connect to remote API server')
            raise RuntimeError("Simulation connection failed")
        
        logging.info('Connected to remote API server')
        return True
    def setup_simulation(self):
        """设置仿真环境参数"""
        sim.simxSetFloatingParameter(self.client_id, sim.sim_floatparam_simulation_time_step, self.tstep, sim.simx_opmode_oneshot)
        sim.simxSynchronous(self.client_id, True)
        
        handle_names = {
            'block': 'Cuboid',
            'ik_tip': 'irb360_ikTip', 
            'base': 'irb360Base',
            'pad': 'suctionPadLoopClosureDummy2'
        }
        
        for key, name in handle_names.items():
            self.handles[key] = sim.simxGetObjectHandle(self.client_id, name, sim.simx_opmode_oneshot_wait)[1]
    
    def reset(self, seed=None, options=None):
        """重置环境"""
        super().reset(seed=seed)
        
        result = self.call_script_function('moveBack', opmode=sim.simx_opmode_oneshot_wait)
        
        if result == sim.simx_return_ok:
            self.randomize_block_position()
            current_position = self.get_current_position(self.handles['pad'])
            current_block_position = self.get_current_position(self.handles['block'])
            logging.info(f"Current End Effector Position: {current_position}")
            logging.info(f"Current Block Position: {current_block_position}")
            self.current_step = 0
            self.state = self._get_extended_observation()
            return self.state, {}
        else:
            logging.warning("Failed to reset to initial position.")
            return None, {}
    
    def step(self, action):
        """执行动作"""
        self.current_step += 1
        logging.info(f"Step: {self.current_step}, Action: {action}")

        action_map = {
            0: 'moveUp', 1: 'moveDown', 
            2: 'moveLeft', 3: 'moveRight', 
            4: 'moveForward', 5: 'moveBackward'
        }
        
        function_name = action_map.get(action)
        if not function_name:
            logging.error("Invalid action")
            return None, 0.0, True, False, {}
        
        res = self.call_script_function(function_name)
        
        new_pad_position = self.get_current_position(self.handles['pad'])
        reward = self._calculate_reward(new_pad_position)
        
        done = self._is_done(new_pad_position)
        truncated = self.current_step >= self.max_steps
        
        self.state = self._get_extended_observation()
        distance = self.state[3]
        
        info = {
            'distance_to_block': distance,
            'reward': reward,
            'done': done
        }
        return self.state, reward, done, truncated, info
    
    
    
    def _get_extended_observation(self):
        """获取扩展观测"""
        pad_pos = self.get_current_position(self.handles['pad'])
        block_pos = self.get_current_position(self.handles['block'])
        
        # 扩展观测
        relative_pos = np.array(block_pos) - np.array(pad_pos)
        distance = np.linalg.norm(relative_pos)
        # velocity = self._calculate_velocity()  # 速度信息
        velocity = []
        
        return np.concatenate([
            pad_pos,           # 末端位置
            block_pos,         # 目标位置
            relative_pos,      # 相对位置
            velocity,          # 速度信息
            [distance],        # 距离
            [self.current_step / self.max_steps]  # 归一化步数
        ])
    def render(self):
        """可视化环境状态"""
        if self.render_mode == 'human':
            pad_pos = self.get_current_position(self.handles['pad'])
            block_pos = self.get_current_position(self.handles['block'])
            
            plt.clf()
            plt.scatter(pad_pos[0], pad_pos[1], c='red', label='Pad')
            plt.scatter(block_pos[0], block_pos[1], c='blue', label='Block')
            plt.title(f'Step: {self.current_step}')
            plt.legend()
            plt.pause(0.1)
    
    def close(self):
        """关闭 CoppeliaSim 客户端"""
        sim.simxFinish(self.client_id)

    def call_script_function(self, function_name: str, ints=None, floats=None, strings=None, bytes=None, opmode=sim.simx_opmode_oneshot) -> int:
        """调用仿真环境中的脚本函数"""
        ints = ints or []
        floats = floats or []
        strings = strings or []
        bytes = bytes or bytearray()

        res, ret_ints, ret_floats, ret_strings, ret_bytes = sim.simxCallScriptFunction(
            self.client_id, self.robot_name, self.script_type,
            function_name, ints, floats, strings, bytes, opmode
        )
        if res != sim.simx_return_ok:
            logging.error(f"Error calling script function {function_name}: {res}")
        return res
    
    def get_current_position(self, handle):
        """获取当前位置"""
        result, position = sim.simxGetObjectPosition(self.client_id, handle, -1, sim.simx_opmode_oneshot_wait)

        if result == sim.simx_return_ok:
            return position
        else:
            logging.warning(f"Failed to get current position: {result}")
            return None
    
    def randomize_block_position(self):
        """随机化物块位置"""
        current_position = self.get_current_position(self.handles['block'])
        logging.info(f"Before Randomize Block Position: {current_position}")
        if current_position is not None:
            workspace_center = [0, 0, 0]
            workspace_size = [0.4, 0.4, 0.2]
            new_position = [
                current_position[i] + self.np_random.uniform(-workspace_size[i]/2, workspace_size[i]/2)
                for i in range(3)
            ]
            sim.simxSetObjectPosition(self.client_id, self.handles['block'], -1, new_position, sim.simx_opmode_oneshot)
        else:
            logging.warning("Failed to get current position of the block.")
    

    def _calculate_velocity(self):
        """计算末端执行器速度"""
        current_position = self.get_current_position(self.handles['pad'])
        current_time = time.time()
        
        if self.last_position is not None and self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                velocity = (np.array(current_position) - np.array(self.last_position)) / dt
                self.last_position = current_position
                self.last_time = current_time
                return velocity
        
        self.last_position = current_position
        self.last_time = current_time
        return np.zeros(3)

    def _calculate_orientation_reward(self):
        """计算姿态奖励"""
        result, orientation = sim.simxGetObjectOrientation(
            self.client_id, 
            self.handles['pad'], 
            -1, 
            sim.simx_opmode_oneshot_wait
        )
        
        if result == sim.simx_return_ok:
            # 计算与期望姿态的差异
            target_orientation = [0, 0, 0]  # 可以根据需求调整目标姿态
            orientation_error = np.linalg.norm(
                np.array(orientation) - np.array(target_orientation)
            )
            return 1.0 / (1.0 + orientation_error)
        return 0.0
    
    def _calculate_reward(self, new_position):
        """计算奖励函数"""
        block_position = self.get_current_position(self.handles['block'])
        distance = np.linalg.norm(np.array(new_position) - np.array(block_position))
        
        proximity_reward = 1.0 / (1.0 + distance)
        step_penalty = 0.01 * self.current_step
        goal_reward = 10.0 if distance < self.distance_threshold else 0
        
        total_reward = proximity_reward + goal_reward - step_penalty
        
        return total_reward
    
    def _calculate_reward(self, new_position):
        block_position = self.get_current_position(self.handles['block'])
        distance = np.linalg.norm(np.array(new_position) - np.array(block_position))
        
        # 多目标奖励
        proximity_reward = 1.0 / (1.0 + distance)  # 距离奖励
        # orientation_reward = self._calculate_orientation_reward()  # 姿态奖励
        orientation_reward = 0          # 还未设置姿态奖励
        efficiency_reward = 1.0 / (1.0 + self.current_step)  # 效率奖励
        
        total_reward = (
            proximity_reward * 0.5 + 
            orientation_reward * 0.3 + 
            efficiency_reward * 0.2
        )
        
        return total_reward
    
    def _is_done(self, new_position):
        """判断是否完成"""
        block_position = self.get_current_position(self.handles['block'])
        distance = np.linalg.norm(np.array(new_position) - np.array(block_position))
        
        return distance < self.distance_threshold