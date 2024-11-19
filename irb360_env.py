import gymnasium as gym
from gymnasium import spaces
import numpy as np
import sim  # CoppeliaSim Python API
import logging
import time
logging.basicConfig(level=logging.INFO)



class IRB360Env(gym.Env):
    """IRB360 环境类,继承自 gym.Env"""
    
    def __init__(self, tstep=0.05, distance_threshold=0.1):
        super().__init__()
        self.client_id = -1
        self.robot_name = 'irb360'
        self.script_type = sim.sim_scripttype_childscript
        self.connect()
        self.action_space = spaces.Discrete(6)
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        self.state = None
        self.initial_position = [0, 0, 0]
        self.ik_tip_handle = -1
        self.base_handle = -1
        self.block_handle = -1
        self.Pad_handle = -1
        self.tstep = tstep
        self.distance_threshold = distance_threshold
        self.setup_simulation()
    
    
    def connect(self) -> bool:
        """连接到仿真环境"""
        sim.simxFinish(-1)
        self.client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
        if self.client_id != -1:
            # print('Connected to remote API server')
            logging.info('Connected to remote API server')
            return True
        else:
            logging.info('Failed to connect to remote API server')
            return False
        
    def setup_simulation(self):
        """设置仿真环境"""
        # 设置仿真时间步长
        self.tstep = 0.05  # 设定适当的时间步长,比如0.05秒
        sim.simxSetFloatingParameter(self.client_id, sim.sim_floatparam_simulation_time_step, self.tstep, sim.simx_opmode_oneshot)
        # 打开同步模式
        sim.simxSynchronous(self.client_id, True)
        # 获取机械臂末端执行器和基座的句柄
        self.block_handle = sim.simxGetObjectHandle(self.client_id, 'Cuboid', sim.simx_opmode_oneshot_wait)[1]  # 获取物块句柄
        self.ik_tip_handle = sim.simxGetObjectHandle(self.client_id, 'irb360_ikTip', sim.simx_opmode_oneshot_wait)[1]
        self.base_handle = sim.simxGetObjectHandle(self.client_id, 'irb360Base', sim.simx_opmode_oneshot_wait)[1]
        self.Pad_handle = sim.simxGetObjectHandle(self.client_id, 'suctionPadLoopClosureDummy2', sim.simx_opmode_oneshot_wait)[1]
        
    def start_simulation(self) -> bool:
        """启动仿真"""
        logging.info("Starting simulation...")
        return sim.simxStartSimulation(self.client_id, sim.simx_opmode_oneshot_wait) == sim.simx_return_ok

    def stop_simulation(self) -> bool:
        """停止仿真"""
        logging.info("Stopping simulation...")
        return sim.simxStopSimulation(self.client_id, sim.simx_opmode_oneshot_wait) == sim.simx_return_ok

    def reset(self, seed=None, options=None):
        """重置环境到初始状态"""
        # 调用 CoppeliaSim 中的 reset 脚本
        result = self.call_script_function('moveBack',opmode=sim.simx_opmode_oneshot_wait)
        # current_block_position = self.get_current_position(self.block_handle)
        # logging.info(f"Current Block Position: {current_block_position}")
        # print(result,sim.simx_return_ok)
        if result == sim.simx_return_ok:
            self.randomize_block_position()
            # 获取初始位置
            current_position = self.get_current_position(self.Pad_handle)
            current_block_position = self.get_current_position(self.block_handle)
            logging.info(f"Current End Effector Position: {current_position}")
            logging.info(f"Current Block Position: {current_block_position}")
            self.state = np.array(current_position)
            return self.state, dict()
        else:
            logging.warning("Failed to reset to initial position.")
            return None, dict()

    # def step(self, action):
    #     """执行一步行动并返回新的状态、奖励、完成标志等信息"""
    #     # 执行动作
    #     function_name = {
    #         0: 'moveUp',
    #         1: 'moveDown',
    #         2: 'moveLeft',
    #         3: 'moveRight',
    #         4: 'moveForward',
    #         5: 'moveBackward'
    #     }.get(action)
    #     if function_name is None:
    #         logging.error("Invalid action.")
    #         return None, 0.0, False, False, {}
        
    #     res = self.call_script_function(function_name)
    #     if res == sim.simx_return_ok:
    #         # 获取新的状态
    #         new_position = self.get_current_position(self.ik_tip_handle)
    #         # 计算奖励和是否结束
    #         reward = self.calculate_reward(new_position)
    #         done = self.is_done(new_position)
    #         truncated = False  # 根据具体任务定义截断条件
    #         self.state = np.array(new_position)
    #         return self.state, reward, done, truncated, dict()
    #     else:
    #         print(f"Error moving: {res}")
    #         return None, 0.0, False, False, dict()


    def step(self, action):
        """执行一步行动并返回新的状态、奖励、完成标志等信息"""
        # 执行动作
        function_name = {
            0: 'moveUp',
            1: 'moveDown',
            2: 'moveLeft',
            3: 'moveRight',
            4: 'moveForward',
            5: 'moveBackward'
        }.get(action)
        if function_name is None:
            logging.error("Invalid action.")
            return None, 0.0, False, False, {}
        max_attempts = 10  # 最大尝试次数
        wait_time = 1  # 每次尝试之间的等待时间（秒）
        for attempt in range(max_attempts):
            res = self.call_script_function(function_name)
            if res == sim.simx_return_ok:
                # 获取新的状态
                new_position = self.get_current_position(self.Pad_handle)
                # _, new_position = sim.simxGetObjectPosition(self.client_id, self.ik_tip_handle, self.base_handle, sim.simx_opmode_oneshot_wait)
                # 计算奖励和是否结束
                reward = self.calculate_reward(new_position)
                done = self.is_done(new_position)
                truncated = False  # 根据具体任务定义截断条件
                self.state = np.array(new_position)
                return self.state, reward, done, truncated, dict()
            else:
                logging.warning(f"Waiting... (Attempt {attempt + 1}/{max_attempts})")
                time.sleep(wait_time)

        logging.info(f"Error moving: {res} after {max_attempts} attempts")
        return None, 0.0, False, False, dict()
    def render(self, mode='human'):
        """渲染环境"""
        # 如果需要渲染,可以在这里实现
        pass

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
        # sim.simx_opmode_oneshot      适用于不关注立即反馈的情况
        # sim.simx_opmode_oneshot_wait 则是在有需要立即获得确认或返回值的情况下使用
        if res != sim.simx_return_ok:
            print(f"Error calling script function {function_name}: {res}")
        return res
    
    def get_current_position(self, handle):
        """获取当前位置"""
        # sim.simxPauseCommunication(self.client_id, True) 
        result, position = sim.simxGetObjectPosition(self.client_id, handle, -1, sim.simx_opmode_oneshot_wait)
        # sim.simxPauseCommunication(self.client_id, False) 

        if result == sim.simx_return_ok:
            return position
        else:
            logging.warning(f"Failed to get current position: {result}")
            return None
    
    
        # 随机化物块位置的函数
    def randomize_block_position(self):
        """随机化物块位置，通过在当前坐标上增加随机增量"""
        # 获取当前物块位置
        current_position = self.get_current_position(self.block_handle)
        if current_position is not None:  # 确保当前坐标获取成功
            # 定义物块位置的范围
            low = np.array([-0.2, -0.2, 0])
            high = np.array([0.2, 0.2, 0.2])
            # 随机化增量
            delta = self.np_random.uniform(low, high)
            # 计算新位置
            new_position = current_position + delta
            # 设置物块的新位置
            res = sim.simxSetObjectPosition(self.client_id, self.block_handle, -1, new_position, sim.simx_opmode_oneshot)
        else:
            print("Failed to get current position of the block.")           
        
    def calculate_reward(self, new_position):
        # 计算机械臂末端执行器与物块之间的距离
        block_position = self.get_current_position(self.block_handle)
        distance = np.linalg.norm(np.array(new_position) - np.array(block_position))
        
        return -distance  # 奖励是距离的负值，鼓励智能体减小距离
    
    def is_done(self, new_position):
        _, block_position = sim.simxGetObjectPosition(self.client_id, self.block_handle, self.base_handle, sim.simx_opmode_oneshot_wait)
        if _ == sim.simx_return_ok:
            distance = np.linalg.norm(np.array(new_position) - np.array(block_position))
            return distance < 0.1
        else:
            print("Failed to get block position.")
            return False