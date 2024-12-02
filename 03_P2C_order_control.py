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
    def call_script_function(self, function_name, ints=[], floats=[], strings=[], bytes=bytearray()):
        return sim.simxCallScriptFunction(self.clientID, self.robot_name, self.script_type, function_name, ints, floats, strings, bytes, sim.simx_opmode_oneshot_wait)

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
        # function_name = 'enableIkMode' if enable else 'disableIkMode'
        return self.call_script_function(function_name)

    
        
    # 关闭连接
    def close(self):
        sim.simxFinish(self.clientID)
    def reset_to_initial_position(self):
        print("Resetting to initial position...")

        # 调用Lua脚本中的resetToInitialPosition函数
        result = self.call_script_function('resetPosition')

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
        # controller.set_ik_mode(True)

        # 示例操作序列
        controller.move('right')
        # controller.check_ik_stability()
        # controller.check_ik_stability()
        # controller.check_ik_stability()

        # print(controller.move('left'))
        time.sleep(1)
        move_result = controller.move('up')
        print(move_result)
        # controller.call_script_function( 'move', ints=[1], floats=[0,0,0.05], strings=[], bytes=bytearray())
        # controller.call_script_function(function_name='move',floats= [0, 0, 0.05])
        time.sleep(1)
        controller.control_suction_pad(True)
        time.sleep(1)
        controller.move('right')
        time.sleep(1)
        controller.control_suction_pad(False)
        time.sleep(1)
        controller.set_ik_mode(False)
        
        # controller.check_ik_stability()
        controller.set_ik_mode(True)
        controller.move('left')
        controller.move('left')
        controller.move('left')
        time.sleep(1)
        controller.move('up')
        controller.reset_position()
        # controller.call_script_function('resetPosition')

        # controller.move2('up')
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
        # controller.reset_position()
        # controller.reset_to_initial_position()
        controller.set_ik_mode(True)
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