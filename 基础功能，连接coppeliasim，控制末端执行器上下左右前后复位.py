# 代码缺陷1. reset函数调用时无法使机械臂回到初始位置，原因未知。
# 代码缺陷2. 机械臂运动方向与键盘方向不一致，原因未知。
# 代码缺陷3. 机械臂运动速度过慢，原因未知。。  
# 代码缺陷4. 对lua的内irb360的限位控制不到位，当机械臂因异常飞车时，应提前发现异常并实现复位操作



import math
import sim
import sys
import cv2
import numpy as np
import pygame
import signal
import time
from Camera import ImageHandler, create_directories, get_camera_handles

class IRB360Controller:
    def __init__(self):
        self.client_id = -1
        self.robot_name = 'irb360'
        self.script_type = sim.sim_scripttype_childscript
        self.image_handler = None
        self.screen = None
        self.resolution = (640, 480)
        self.running = False
        self.initial_position = [0, 0, 0]  # 确保这与Lua脚本中的初始位置一致
        self.ik_tip_handle = -1
        self.base_handle = -1
        
    def connect(self) -> bool:
        """Connect to the simulation environment"""
        sim.simxFinish(-1)
        self.client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
        if self.client_id != -1:
            print('Connected to remote API server')
            # 设置仿真时间步长
            self.tstep = 0.05  # 设定适当的时间步长，比如0.05秒
            sim.simxSetFloatingParameter(self.client_id, sim.sim_floatparam_simulation_time_step, self.tstep, sim.simx_opmode_oneshot)
            # 打开同步模式
            sim.simxSynchronous(self.client_id, True)

            
            cameraRGBHandles, cameraDepthHandles = get_camera_handles(self.client_id)  
            self.image_handler = ImageHandler(self.client_id, cameraRGBHandles, cameraDepthHandles, self.resolution[0], self.resolution[1])
            # 获取机械臂末端执行器和基座的句柄
            self.ik_tip_handle = sim.simxGetObjectHandle(self.client_id, 'irb360_ikTip', sim.simx_opmode_oneshot_wait)[1]
            self.base_handle = sim.simxGetObjectHandle(self.client_id, 'irb360Base', sim.simx_opmode_oneshot_wait)[1]
            return True
        else:
            print('Failed to connect to remote API server')
            return False

    def start_simulation(self) -> bool:
        """Start the simulation"""
        return sim.simxStartSimulation(self.client_id, sim.simx_opmode_oneshot_wait) == sim.simx_return_ok

    def stop_simulation(self) -> bool:
        """Stop the simulation"""
        return sim.simxStopSimulation(self.client_id, sim.simx_opmode_oneshot_wait) == sim.simx_return_ok

    def call_script_function(self, function_name: str, ints=None, floats=None, strings=None, bytes=None) -> int:
        """Call a script function in the simulation environment"""
        ints = ints or []
        floats = floats or []
        strings = strings or []
        bytes = bytes or bytearray()

        res, ret_ints, ret_floats, ret_strings, ret_bytes = sim.simxCallScriptFunction(
            self.client_id, self.robot_name, self.script_type,
            function_name, ints, floats, strings, bytes, sim.simx_opmode_oneshot_wait
        )
        if res != sim.simx_return_ok:
            print(f"Error calling script function {function_name}: {res}")
        return res

   
 
    def get_current_position(self):
        # 获取当前位置
        result, position = sim.simxGetObjectPosition(self.client_id, self.ik_tip_handle, self.base_handle, sim.simx_opmode_oneshot_wait)
        if result == sim.simx_return_ok:
            return position
        else:
            print(f"Failed to get current position: {result}")
            return None

    def move(self, direction: str) -> None:
        """Move the robot in a specific direction"""
        function_name = {
            'up': 'moveUp',
            'down': 'moveDown',
            'left': 'moveLeft',
            'right': 'moveRight',
            'forward': 'moveForward',
            'backward': 'moveBackward'
        }.get(direction.lower())

        if function_name:
            res = self.call_script_function(function_name)
            if res == sim.simx_return_ok:
                print(f"Robot moved {direction}")
            else:
                print(f"Error moving {direction}: {res}")
        else:
            print(f"Invalid direction: {direction}")

    def initialize_pygame(self) -> None:
        """Initialize the Pygame window"""
        pygame.init()
        self.screen = pygame.display.set_mode((self.resolution[0] * 2, self.resolution[1] * 2))
        pygame.display.set_caption("IRB360 Camera Feed")

    def display_camera_feed(self) -> None:
        """Display the camera feed in the Pygame window"""
        if self.image_handler and self.screen:
            rgb_images, depth_images = self.image_handler.get_images_from_all_cameras()

            for i, (rgb_image, depth_image) in enumerate(zip(rgb_images, depth_images)):
                if rgb_image is not None and depth_image is not None:
                    rgb_surface = pygame.surfarray.make_surface(rgb_image.swapaxes(0, 1))
                    depth_surface = pygame.surfarray.make_surface(depth_image.swapaxes(0, 1))

                    x, y = (0, 0) if i == 0 else (0, self.resolution[1])
                    self.screen.blit(rgb_surface, (x, y))
                    x, y = (self.resolution[0], 0) if i == 0 else (self.resolution[0], self.resolution[1])
                    self.screen.blit(depth_surface, (x, y))

            pygame.display.flip()

    def handle_input(self) -> None:
        """Handle user input"""
        move_commands = {
            pygame.K_UP: 'up',
            pygame.K_DOWN: 'down',
            pygame.K_LEFT: 'left',
            pygame.K_RIGHT: 'right',
            pygame.K_w: 'forward',
            pygame.K_s: 'backward'
        }

        key_state = {key: False for key in move_commands.keys()}  # 用于记录按键状态

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in move_commands:
                    key_state[event.key] = True
                elif event.key == pygame.K_r:
                    # self.reset_to_initial_position()
                    self.call_script_function('moveBack')
                elif event.key == pygame.K_SPACE:
                    self.call_script_function('activateSuctionPad')
                elif event.key == pygame.K_LCTRL:
                    self.call_script_function('deactivateSuctionPad')
                elif event.key == pygame.K_p:
                    position = self.get_end_effector_position()
                    if position:
                        print(f"End effector position: {position}")
                elif event.key == pygame.K_ESCAPE:
                    self.terminate()
            elif event.type == pygame.KEYUP:
                if event.key in move_commands:
                    key_state[event.key] = False

        # 在 run 循环中持续检查按键状态并执行移动
        for key, direction in move_commands.items():
            if key_state[key]:
                self.move(direction)

    def run(self) -> None:
        """Run the main loop"""
        self.running = True
        clock = pygame.time.Clock()
        while self.running:
            self.handle_input()
            self.display_camera_feed()
            clock.tick(30)  # Limit the frame rate to 30 FPS


    def get_end_effector_position(self) -> list:
        """Get the position of the end effector"""
        result, handle = sim.simxGetObjectHandle(self.client_id, 'irb360_ikTip', sim.simx_opmode_oneshot_wait)
        if result != sim.simx_return_ok:
            print("Error getting object handle for end effector")
            return None
        result, position = sim.simxGetObjectPosition(self.client_id, handle, -1, sim.simx_opmode_oneshot_wait)
        if result != sim.simx_return_ok:
            print("Error getting position for end effector")
            return None
        return position



    def close(self) -> None:
        """Close the Pygame window and simulation"""
        pygame.quit()
        sim.simxFinish(self.client_id)

    def terminate(self) -> None:
        """Terminate the process"""
        print("Terminating the process...")
        self.running = False
        self.stop_simulation()
        self.close()
        sys.exit()

def signal_handler(sig, frame) -> None:
    """Handle signals"""
    print("Signal received, terminating the process...")
    pygame.quit()
    sim.simxFinish(-1)
    sys.exit(0)

def main() -> None:
    """Main function"""
    signal.signal(signal.SIGINT, signal_handler)
    controller = IRB360Controller()
    if not controller.connect():
        sys.exit("Connection failed")

    controller.initialize_pygame()
    if not controller.start_simulation():
        sys.exit("Failed to start simulation")

    try:
        controller.run()
    finally:
        controller.stop_simulation()
        controller.close()

if __name__ == "__main__":
    main()