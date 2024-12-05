import sim
import time
import sys

class IRB360Controller:
    # 初始化类实例，设置默认参数
    def __init__(self):
        self.clientID = -1
        self.robot_name = 'irb360'
        self.script_type = sim.sim_scripttype_childscript

    # 连接到远程API服务器
    def connect(self):
        sim.simxFinish(-1)  # 关闭所有连接
        self.clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
        if self.clientID != -1:
            print('Connected to remote API server')
            return True
        else:
            print('Failed to connect to remote API server')
            return False

    # 启动仿真
    def start_simulation(self):
        return sim.simxStartSimulation(self.clientID, sim.simx_opmode_oneshot_wait)

    # 停止仿真
    def stop_simulation(self):
        return sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot_wait)

    # 调用脚本函数
    def call_script_function(self, function_name, ints=[], floats=[], strings=[], bytes=bytearray(), opmode=sim.simx_opmode_oneshot_wait):
        return sim.simxCallScriptFunction(self.clientID, self.robot_name, self.script_type, function_name, ints, floats, strings, bytes, opmode)
    # 获取对象位置
    def get_object_position(self, object_name):
        result, handle = sim.simxGetObjectHandle(self.clientID, object_name, sim.simx_opmode_oneshot_wait)
        if result != sim.simx_return_ok:
            print(f"Error getting object handle for {object_name}")
            return None
        result, position = sim.simxGetObjectPosition(self.clientID, handle, -1, sim.simx_opmode_oneshot_wait)
        if result != sim.simx_return_ok:
            print(f"Error getting position for {object_name}")
            return None
        return position

    # 移动机器人
    def move(self, direction):
        function_name = {
            'up': 'moveUp',
            'down': 'moveDown',
            'left': 'moveLeft',
            'right': 'moveRight',
            'forward': 'moveForward',
            'backward': 'moveBackward'
        }.get(direction.lower())

        if function_name:
            return self.call_script_function(function_name)
        else:
            print(f"Invalid direction: {direction}")
            return None

    # 移动机器人到指定位置
    def move_to_position(self, target_position, steps=None):
        """
        Move the robot's end-effector to a specified target position using cubic interpolation.

        Args:
            target_position (list): A list of three floats representing the target position [x, y, z].
            steps (int, optional): Number of interpolation steps. Defaults to None, which will be determined based on the distance.
        """
        # Ensure target_position is a list of three floats
        if not isinstance(target_position, list) or len(target_position) != 3:
            raise ValueError("target_position must be a list of three floats [x, y, z].")

        # Convert target_position to floats
        target_position = [float(coord) for coord in target_position]

        # Ensure steps is an integer if provided
        if steps is not None and not isinstance(steps, int):
            raise ValueError("steps must be an integer.")

        # Call the Lua script function 'moveToPosition'
        res, ret_ints, ret_floats, ret_strings, ret_bytes = self.call_script_function(
            function_name='moveToPosition',
            ints=[steps] if steps is not None else [],
            floats=target_position,
            opmode=sim.simx_opmode_oneshot_wait
        )

        if res != sim.simx_return_ok:
            print(f"Error calling script function 'moveToPosition': {res}")
            return False

        return True
    
 
    def move_to_direction(self, target_direction, steps=None):
        """
        Move the robot's end-effector to a specified target position using cubic interpolation.

        Args:
            target_position (list): A list of three floats representing the target position [x, y, z].
            steps (int, optional): Number of interpolation steps. Defaults to None, which will be determined based on the distance.
        """
        # Ensure target_position is a list of three floats
        if not isinstance(target_direction, list) or len(target_direction) != 3:
            raise ValueError("target_position must be a list of three floats [x, y, z].")

        # Convert target_position to floats
        target_direction = [float(coord) for coord in target_direction]

        # Ensure steps is an integer if provided
        if steps is not None and not isinstance(steps, int):
            raise ValueError("steps must be an integer.")

         # Call the Lua script function 'move'
        res, ret_ints, ret_floats, ret_strings, ret_bytes = sim.simxCallScriptFunction(
            clientID=self.clientID, 
            scriptDescription=self.robot_name,
            options=self.script_type,
            functionName='move_to_direction',
            inputInts=[steps] if steps is not None else [],
            inputFloats=target_direction,
            inputStrings=[],
            inputBuffer=bytearray(),
            operationMode=sim.simx_opmode_oneshot_wait
        )

        if res != sim.simx_return_ok:
            print(f"Error calling script function 'moveToPosition': {res}")
            return False

        return True
    
    def check_ik_stability(self):
        """
        检查机械臂的逆运动学求解状态
        
        Returns:
        - is_stable: 布尔值，表示机械臂是否稳定
        - result: IK求解器状态码
        - message: 状态描述
        """
        result, ints_ret, floats_ret, strings_ret, bytes_ret = sim.simxCallScriptFunction(
            clientID=self.clientID, 
            scriptDescription=self.robot_name,
            options=self.script_type,
            functionName='checkIKStatus',
            inputInts=[],
            inputFloats=[],
            inputStrings=[],
            inputBuffer=bytearray(),
            operationMode=sim.simx_opmode_oneshot
        )

        if ints_ret:
            ik_status = {
                'is_stable': ints_ret[0] == 1,
                'result': ints_ret[0],
                'message': strings_ret[0] if strings_ret else 'No message'
            }
            
            print("IK Stability Check:")
            print(f"Stable: {ik_status['is_stable']}")
            print(f"Result Code: {ik_status['result']}")
            print(f"Message: {ik_status['message']}")
            
            return ik_status
        else:
            print("Failed to check IK stability")
            return {
                'is_stable': False,
                'result': None,
                'message': 'Check failed'
            }

    # 重置机器人位置
    def reset_position(self):
        return self.call_script_function('resetToInitialPosition')

    # 控制吸盘
    def control_suction_pad(self, activate):
        function_name = 'activateSuctionPad' if activate else 'deactivateSuctionPad'
        return self.call_script_function(function_name)

    # 设置逆运动学模式
    def set_ik_mode(self, enable):
        function_name = 'setIkMode' if enable else 'setFkMode'
        return self.call_script_function(function_name)

    # 关闭连接
    def close(self):
        sim.simxFinish(self.clientID)

    def reset_to_initial_position(self):
        print("Resetting to initial position...")

        # 调用Lua脚本中的resetToInitialPosition函数
        result = self.call_script_function('resetToInitialPosition')

        if result == sim.simx_return_ok:  # 检查Lua函数返回值
            print("Reset command sent successfully.")

            # 等待机械臂移动完成
            time.sleep(5)  # 可能需要调整等待时间

            # 验证最终位置
            current_position = self.get_current_position()
            print(f"Current position after reset: {current_position}")

            # 检查是否达到了预期的初始位置
            tolerance = 0.001  # 允许的误差范围
            if all(abs(current - initial) < tolerance for current, initial in zip(current_position, self.initial_position)):
                print("Successfully reset to initial position.")
                return True
            else:
                print("Warning: Final position does not match initial position.")
                return False
        else:
            print("Failed to reset to initial position.")
            return False

# 主函数，演示如何使用IRB360Controller类
def main():
    controller = IRB360Controller()
    if not controller.connect():
        sys.exit("Connection failed")

    try:
        controller.start_simulation()
        # time.sleep(1)
        controller.set_ik_mode(True)

        # 示例操作序列
        controller.move('right')
        # time.sleep(1)
        move_result = controller.move('up')
        print(move_result)
        controller.control_suction_pad(True)
        controller.move('right')
        controller.control_suction_pad(False)
        controller.set_ik_mode(False)
        controller.set_ik_mode(True)
        controller.move('left')
        controller.move('left')
        controller.move('left')
        controller.move('up')
        controller.reset_position()
        time.sleep(1)
        controller.move('up')
        controller.move('up')
        controller.move('up')
        controller.move('up')
        controller.move('up')
        controller.move('up')
        controller.move('up')
        controller.call_script_function('moveBack')
        controller.move('up')
        time.sleep(1)
        controller.set_ik_mode(False)
        controller.set_ik_mode(True)

        # 使用新的 move_to_position 方法
        target_position = [0.1, 0.2, -0.8]  # 目标位置 [x, y, z]
        steps = 500  # 步数
        controller.move_to_position(target_position, steps)
        # time.sleep(5)  # 等待移动完成
        direction = [0.1, 0, 0]  # 向右移动 0.01 单位
        steps = 50
        controller.move_to_direction(direction, steps)
        # 获取末端执行器位置
        end_effector_position = controller.get_object_position('irb360_ikTip')
        if end_effector_position:
            print(f"End effector position: {end_effector_position}")

        controller.reset_position()
        time.sleep(2)

    finally:
        controller.stop_simulation()
        controller.close()

if __name__ == "__main__":
    main()