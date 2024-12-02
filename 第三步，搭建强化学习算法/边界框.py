import gymnasium as gym
from gymnasium import spaces
import numpy as np
import random
import sim  # CoppeliaSim Python API
import logging
import time
import matplotlib.pyplot as plt

class IRB360Env(gym.Env):
    """IRB360 环境类，继承自 gym.Env，负责机器人控制和仿真环境交互
        观察空间

            定义：观察空间定义为 Box(low=-2.0, high=2.0, shape=(17,), dtype=np.float32)，包含了末端执行器位置、目标块位置、相对坐标、速度信息、姿态信息、距离和归一化步数等。
            合理性：观察空间的设计是合理的，涵盖了机器人控制任务中的关键信息。

        动作空间

            定义：动作空间定义为 Discrete(6)，表示机械臂可以执行六种离散动作（上、下、左、右、前、后）。
            合理性：对于简单的机器人控制任务，这个动作空间是足够的。

        奖励机制

            多目标奖励：奖励函数考虑了距离奖励、姿态奖励、效率奖励和速度惩罚。
            合理性：这个奖励函数设计是合理的，鼓励机器人快速、准确地完成任务，同时避免过大的速度。

        结束和终止条件

            结束条件：任务结束时，距离阈值小于指定值。
            终止条件：步数达到最大步数或其他特殊情况（如超出工作范围）。
            合理性：这些条件是合理的，确保任务有明确的结束和终止标准。


        存在的不足：  没有每一步的详细信息，如末端执行器的速度、末端执行器的姿态、末端执行器的力、末端执行器的扭矩等。
                    训练过程无法看到完整流程
                    末端执行器边界设定：最好能获取一个末端执行器的运动区域，以便在训练过程中控制末端执行器的运动范围。或是关节角度，哪个关节角度偏转过于严重，导致末端执行器无法运动。就设定重启
    
    
        修改：逆运动状态监测的添加，step函数内状态终止的判断
    
    """
    
    def __init__(self, 
                 tstep=0.05, 
                 distance_threshold=0.1,
                 max_steps=1000,
                 render_mode=None,
                 seed=None):
        """初始化环境参数和连接仿真"""
        super().__init__()
        
        # 添加性能监控
        self.reward_history = []
        self.distance_history = []
        self.step_times = []
        
        # 添加速度计算所需的历史位置
        self.current_Pad = None  # 末端当前位置
        self.current_Block = None   # 物块位置
        self.current_Boundary = None  # 边界框的中心位置
        self.last_Pad = None    # 上一次的位置
        self.distance = None         # 当前距离
        self.relative_distance = None  # 相对距离
        self.last_time = None
        
        # 配置参数
        self.tstep = tstep
        self.distance_threshold = distance_threshold
        self.max_steps = max_steps
        self.current_step = 0
        self.render_mode = render_mode
        self.seed(seed)
        self.Boundary_size = [0.9, 0.9, 0.6]  # 边界框的尺寸

        
        # 日志配置
        logging.basicConfig(level=logging.INFO)
        
        # 仿真连接和初始化
        self.client_id = -1
        self.robot_name = 'irb360'
        self.script_type = sim.sim_scripttype_childscript
        
        # 连接并设置仿真环境
        
        self.connect()
        # self.stop_simulation()
        time.sleep(2)
        self.start_simulation()
        time.sleep(2)
        # 动作和观测空间的定义
        self.action_space = spaces.Discrete(6)
        self.observation_space = spaces.Box(low=-2.0, high=2.0, shape=(17,), dtype=np.float32)
        
        # 初始化句柄和状态
        self.handles = {
            'block': None,
            'ik_tip': None,
            'base': None,
            'pad': None,
            'boundary': None
        }
        self.state = None
        
        # 设置仿真
        self.setup_simulation()
        
        # 随机数生成器
        self.np_random = np.random.default_rng()
    
    
    def step(self, action):
        """执行动作
            返回值如下所示：
            
            状态(state):当前观测
            奖励(reward):当前奖励值
            是否结束(terminated):表示环境是否已结束的布尔值
            是否截断(truncated):表示是否因超出最大步数而截断的布尔值,
                                或，是否因末端坐标超出移动区间，
                                或，关节超出限制角度范围而终止
            额外信息(info):字典,包含其他可能有用的信息

            """
        # 检查结束或截断状态，重置环境判定（进入下一个循环）
        if self.terminated or self.truncated:
            self.reset()
            return self.state, 0.0, True, True, {'episode_status': 'reset'}
        # 位置信息更新    

        self.last_Pad = self.current_Pad
        # 计算距离
        self.calculate_distances()
        # 时间步计数器 
        self.current_step += 1
        # 动作映射
        action_map = {
            0: 'moveUp', 1: 'moveDown', 
            2: 'moveLeft', 3: 'moveRight', 
            4: 'moveForward', 5: 'moveBackward'
        }
        
        function_name = action_map.get(action)
        if not function_name:
            logging.error("Invalid action")
            return self.state, 0.0, self.terminated, self.truncated, {}
        # 执行动作
        res = self.call_script_function(function_name)
        
        # 获取新状态
        self.current_Pad = self.get_current_position(self.handles['pad'])
        
        # 检查逆运动学稳定性
        ik_status = self.check_ik_stability()
        if not ik_status:
            logging.warning("IK is not stable, truncating current episode and resetting environment.")
            self.truncated = True
            return self.state, 0.0, self.terminated, self.truncated, {'episode_status': 'unstable_ik'}
        
        # 检查末端执行器是否在边界内
        if not self.check_within_boundary(self.current_Pad):
            logging.warning("End effector out of boundary, resetting environment.")
            self.truncated = True  # 或者设置 terminated = True
            return self.reset()[0], 0.0, False, True, {'episode_status': 'boundary_exceeded'}

        
        # 计算奖励
        reward = float(self._calculate_reward(self.current_Pad))
        
        # 判断是否结束
        self.terminated = bool(self._is_terminated(self.current_Pad))
        self.truncated = bool(self.current_step >= self.max_steps)
        
        # 更新状态
        self.state = self._get_extended_observation()
        
        # 获取距离
        distance = self.state[15]  # 距离在索引6的位置
        
        info = {
            'distance_to_block': distance,
            'reward': reward,
            'terminated':self.terminated,
            'truncated': self.truncated
        }
        
        return self.state, reward,self.terminated, self.truncated, info
    
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
            'pad': 'suctionPadLoopClosureDummy2',
            'boundary':'Boundary'
        }
        
        for key, name in handle_names.items():
            self.handles[key] = sim.simxGetObjectHandle(self.client_id, name, sim.simx_opmode_oneshot_wait)[1]
        self.current_Boundary = self.get_current_position(self.handles['boundary'])
        logging.info(f"Boundary center position: {self.current_Boundary}")
    def reset(self, seed=None, options=None):
        """重置环境
            目标随机化，末端执行器位置初始化
        """
        super().reset(seed=seed)
        
        # 重置内部状态
        self.current_step = 0
        self.current_Pad = None
        self.last_time = None
        self.reward_history = []
        self.distance_history = []
        self.step_times = []
        self.terminated = False
        self.truncated = False
        
        # 重置机器人位置
        
        # time.sleep(0.1)
        # self.call_script_function('resetPosition')
        # self.call_script_function('resetPad')
        self.call_script_function('moveBack')
        

        # 随机化物块位置
        self.current_Block = self.randomize_block_position()
        
        # 获取初始观测
        self.state = self._get_extended_observation()
        
        return self.state, {}
    
    def _get_extended_observation(self):
        """
        精简的观测空间 - 17维
        包括：
        - 末端执行器位置    (3)
        - 目标块位置        (3)
        - 相对坐标          (3)
        - 速度信息          (3)
        - 姿态信息          (3)
        - 距离              (1)
        - 归一化步数            (1)
        """
        """获取观测，并确保返回float32类型"""
        
        pad_pos = np.array(self.current_Pad, dtype=np.float32)
        
        block_pos = np.array(self.current_Block, dtype=np.float32)
        relative_pos = np.array(block_pos) - np.array(pad_pos)
        
        self.calculate_distances()
        velocity = self._calculate_velocity()
        
        # 获取姿态信息
        _, orientation = sim.simxGetObjectOrientation(
            self.client_id, 
            self.handles['pad'], 
            -1, 
            sim.simx_opmode_oneshot_wait            
        )
        orientation = np.array(orientation, dtype=np.float32)
        observation = np.concatenate([
            pad_pos,           # 末端 位置 (3)
            block_pos,         # 目标位置 (3)
            relative_pos,      # 相对位置 (3)
            velocity,          # 速度信息 (3)
            orientation,       # 姿态信息 (3)
            [self.distance],        # 距离 (1)
            [np.float32(self.current_step) / self.max_steps]  # 归一化步数 (1)
             ]).astype(np.float32)  # 确保最终结果为float32
        return observation    
    
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
        """获取指定句柄当前的位置"""
        # result, position = sim.simxGetObjectPosition(self.client_id, handle, -1, sim.simx_opmode_oneshot)
        result, position = sim.simxGetObjectPosition(self.client_id, handle, -1, sim.simx_opmode_oneshot_wait)

        if result == sim.simx_return_ok:
            return position
        else:
            logging.warning(f"Failed to get current position: {result}")
            return None
    
    def randomize_block_position(self):
        """随机化物块位置"""
        self.current_Pad = self.get_current_position(self.handles['pad'])
        self.current_Boundary = self.get_current_position(self.handles['boundary'])
        logging.info(f"Before Randomize Block Position: {self.current_Block}")
        # 随机化物块位置
        
        if self.current_Boundary is not None:
            # workspace_center = [0, 0, 0]
            workspace_size = [0.4, 0.4, 0.2]
            new_position = [
                self.current_Boundary[i] + self.np_random.uniform(-workspace_size[i]/2, workspace_size[i]/2)
                for i in range(3)
            ]
            # 让物块落地
            sim.simxSetObjectPosition(self.client_id, self.handles['block'], -1, new_position, sim.simx_opmode_oneshot)
            self.current_Block = self.get_current_position(self.handles['block'])
            return new_position 
        else:
            logging.warning("Failed to get current position of the block.")
            
    def _calculate_velocity(self):
        """计算末端执行器速度"""
        current_position = self.current_Pad
        current_time = time.time()
        
        if self.last_Pad is not None and self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                velocity = (np.array(current_position) - np.array(self.last_Pad)) / dt
                self.last_Pad = current_position
                self.last_time = current_time
                return velocity
        
        self.last_Pad = current_position
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
        """改进的奖励计算"""
        block_position = self.current_Block
        # distance = np.linalg.norm(np.array(new_position) - np.array(block_position))
        
        # 多目标奖励
        proximity_reward = 1.0 / (1.0 + self.distance)  # 距离奖励
        orientation_reward = self._calculate_orientation_reward()  # 姿态奖励
        efficiency_reward = 1.0 / (1.0 + self.current_step)  # 效率奖励
        
        # 添加速度惩罚
        velocity = self._calculate_velocity()
        velocity_penalty = -0.1 * np.linalg.norm(velocity)  # 防止过大的速度
        
        total_reward = (
            proximity_reward * 0.4 + 
            orientation_reward * 0.3 + 
            efficiency_reward * 0.2 +
            velocity_penalty * 0.1
        )
        
        # 记录历史数据
        self.reward_history.append(total_reward)
        self.distance_history.append(self.distance)
        
        return total_reward
    
    def _is_terminated(self, new_position):
        """判断是否达到完成条件"""
        block_position = self.get_current_position(self.handles['block'])
        if block_position is None or new_position is None:
            return False    
        distance = np.linalg.norm(np.array(new_position) - np.array(block_position))
        
        return distance < self.distance_threshold
    
    def calculate_distances(self):
        """计算并更新间距和相对间距"""
        if self.current_Pad is not None and self.current_Block is not None:
            self.relative_distance = np.array(self.current_Block) - np.array(self.current_Pad)
            self.distance = np.linalg.norm(self.relative_distance)

    def check_ik_stability(self):
        """
        检查机械臂的逆运动学求解状态
        
        Returns:
        - is_stable: 布尔值，表示机械臂是否稳定
        - result: IK求解器状态码
        - message: 状态描述
        """
        result, ints_ret, _, _, _ = sim.simxCallScriptFunction(
            clientID=self.client_id, 
            scriptDescription=self.robot_name,
            options=self.script_type,
            functionName='checkIKStatus',
            inputInts=[],
            inputFloats=[],
            inputStrings=[],
            inputBuffer=bytearray(),
            # operationMode=sim.simx_opmode_oneshot_wait
            operationMode=sim.simx_opmode_oneshot

        )

        if result == 0 and ints_ret:
            is_stable = ints_ret[0] == 1
            logging.debug(f"IK Stability Check: Stable: {is_stable}")
            return is_stable
        else:
            logging.warning("Failed to check IK stability")
            return False


    def check_within_boundary(self, position):
        """检查末端执行器是否在边界范围内"""
        if self.current_Boundary is None:
            return True  # 如果未定义边界，则不限制

        # 计算边界范围
        boundary_limits = [
            (self.current_Boundary[i] - self.Boundary_size[i] / 2, self.current_Boundary[i] + self.Boundary_size[i] / 2)
            for i in range(3)
        ]

        # 检查每个维度是否在边界范围内
        for i in range(3):
            if not (boundary_limits[i][0] <= position[i] <= boundary_limits[i][1]):
                return False
        return True
        
    def render(self):
        """增强的可视化功能"""
        if self.render_mode == 'human':
            plt.ion()  # 开启交互模式
            plt.figure(figsize=(12, 8))
            
            # 位置图
            plt.subplot(2, 2, 1)
            plt.cla()  # 清除当前坐标轴
            pad_pos = self.get_current_position(self.handles['pad'])
            block_pos = self.get_current_position(self.handles['block'])
            plt.scatter(pad_pos[0], pad_pos[1], c='red', label='Pad')
            plt.scatter(block_pos[0], block_pos[1], c='blue', label='Block')
            plt.title(f'Position (Step: {self.current_step})')
            plt.legend()
            
            # 奖励历史
            plt.subplot(2, 2, 2)
            plt.cla()  # 清除当前坐标轴
            plt.plot(self.reward_history)
            plt.title('Reward History')
            
            # 距离历史
            plt.subplot(2, 2, 3)
            plt.cla()  # 清除当前坐标轴
            plt.plot(self.distance_history)
            plt.title('Distance History')
            
            plt.tight_layout()
            plt.pause(0.1)  # 刷新图像

    def stop_simulation(self) -> bool:
        """停止仿真"""
        logging.info("Stopping simulation...")
        return sim.simxStopSimulation(self.client_id, sim.simx_opmode_oneshot_wait) == sim.simx_return_ok
    
    def start_simulation(self) -> bool:
        """启动仿真"""
        logging.info("Starting simulation...")
        return sim.simxStartSimulation(self.client_id, sim.simx_opmode_oneshot_wait) == sim.simx_return_ok

    def close(self):
        """增强的关闭功能并保存性能数据"""
        # 保存性能数据
        np.save('reward_history.npy', self.reward_history)
        np.save('distance_history.npy', self.distance_history)
        
        # 关闭连接
        sim.simxFinish(self.client_id)
        
    def seed(self,seed = None):
        """设置随机数种子"""
        """设置随机数生成器的种子"""
        self.np_random = np.random.default_rng(seed)
        return [seed]