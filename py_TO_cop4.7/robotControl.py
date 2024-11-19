# -*- coding:utf-8 -*-

"""
keyboard Instructions:
    robot moving velocity: <=5(advise)
    Q,W: joint 0
    A,S: joint 1
    Z,X: joint 2
    E,R: joint 3
    D,F: joint 4
    C,V: joint 5
    P: exit()
    T: close RG2
    Y: open RG2
    L: reset robot
    SPACE: save image
"""
"""
控制机械臂的各个关节旋转
实现RG2的打开与闭合
将摄像头拍到的图片导出来
以及其它一些小功能
"""
# 1.导入相关模块
import os
import cv2
import sys
import math
import time
import random
import string
import pygame
import sim
import numpy as np


class UR5_RG2:
    # 2.常量，变量，句柄的获取
    # 2-1 摄像头图像大小
    resolutionX = 640  # Camera resolution: 640*480
    resolutionY = 480
    # 2-2 机械臂关节角度
    joint_angle = [0, 0, 0, 0, 0, 0]  # each angle of joint
    RAD2DEG = 180 / math.pi  # transform radian to degrees

    # 2-3 句柄
    jointNum = 6
    baseName = 'UR5'
    rgName = 'RG2'
    jointName = 'UR5_joint'
    camera_rgb_Name = 'kinect_rgb'
    camera_depth_Name = 'kinect_depth'


    # 3.函数
    # 3-1 communication and read the handles
    def __init__(self):
        jointNum = self.jointNum
        baseName = self.baseName
        rgName = self.rgName
        jointName = self.jointName
        camera_rgb_Name = self.camera_rgb_Name
        camera_depth_Name = self.camera_depth_Name

        print('仿真开始')
        sim.simxFinish(-1)  # 关闭潜在的连接
        # 每隔0.2s检测一次, 直到连接上V-rep
        while True:
            # simxStart的参数分别为：服务端IP地址(连接本机用127.0.0.1);端口号;是否等待服务端开启;连接丢失时是否尝试再次连接;超时时间(ms);数据传输间隔(越小越快)
            # 返回一个类似bool的类型变量
            clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

            if clientID > -1:
                print("Connection success!")
                break
            else:
                time.sleep(0.2)
                print("Failed connecting to remote API server!")
                print("Maybe you forget to run the simulation on sim...")
        
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)  # 仿真初始化

        # 3-2 ，获取句柄
        # 获取了句柄，就可以通过句柄向被控对象输入指令或者获取数据。
        # UR5-关节-句柄
        # jointHandle = np.zeros((jointNum, 1), dtype=np.int)

        jointHandle = np.zeros(jointNum, dtype=int)

        for i in range(jointNum):
            # 第三个参数是通讯方式:阻塞模式，函数会等待直到收到服务器的响应才会继续执行
            _, returnHandle = sim.simxGetObjectHandle(clientID, jointName + str(i + 1), sim.simx_opmode_blocking)
            jointHandle[i] = returnHandle
            print("jointHandle" + str(i + 1) + ": " + str(jointHandle[i]))
        # UR5、RG2、CAMERA的句柄/接口
        _, baseHandle = sim.simxGetObjectHandle(clientID, baseName, sim.simx_opmode_blocking)
        print("baseHandle: " + str(baseHandle))
        _, rgHandle = sim.simxGetObjectHandle(clientID, rgName, sim.simx_opmode_blocking)
        _, cameraRGBHandle = sim.simxGetObjectHandle(clientID, camera_rgb_Name, sim.simx_opmode_blocking)
        _, cameraDepthHandle = sim.simxGetObjectHandle(clientID, camera_depth_Name, sim.simx_opmode_blocking)

        # 3-3 关节角度
        # simxGetJointPosition（） 获取关节初始角度
        # 在类的实例被创建时，初始化关节配置 jointConfig。这个配置保存了机械臂启动时各个关节的初始角度，为后续的关节控制提供起始点。
        jointConfig = np.zeros((jointNum, 1))
        for i in range(jointNum):
            _, jpos = sim.simxGetJointPosition(clientID, jointHandle[i], sim.simx_opmode_blocking)
            jointConfig[i] = jpos

        # 句柄的传递，把函数获取的句柄传递给整个类，当该类实例化式，句柄已经得到。简单的说，把自己的变成大家的
        self.clientID = clientID
        self.jointHandle = jointHandle
        self.rgHandle = rgHandle
        self.cameraRGBHandle = cameraRGBHandle
        self.cameraDepthHandle = cameraDepthHandle
        self.jointConfig = jointConfig

    # 功能函数1 断开连接
    def __del__(self):
        clientID = self.clientID
        sim.simxFinish(clientID)
        print('Simulation end')

    # 功能函数2 展示句柄 便于理解代码
    def showHandles(self):

        RAD2DEG = self.RAD2DEG
        jointNum = self.jointNum
        clientID = self.clientID
        jointHandle = self.jointHandle
        rgHandle = self.rgHandle
        cameraRGBHandle = self.cameraRGBHandle
        cameraDepthHandle = self.cameraDepthHandle

        print('Handles available!')
        print("==============================================")
        print("Handles:  ")
        for i in range(len(jointHandle)):
            print("jointHandle" + str(i + 1) + ": " + jointHandle[i])
        print("rgHandle:" + rgHandle)
        print("cameraRGBHandle:" + cameraRGBHandle)
        print("cameraDepthHandle:" + cameraDepthHandle)
        print("===============================================")

    # 功能函数3 获取UR5的六个关节角度--当前
    def showJointAngles(self):
        RAD2DEG = self.RAD2DEG
        jointNum = self.jointNum
        clientID = self.clientID
        jointHandle = self.jointHandle

        for i in range(jointNum):
            _, jpos = sim.simxGetJointPosition(clientID, jointHandle[i], sim.simx_opmode_blocking)
            print(round(float(jpos) * RAD2DEG, 2), end=' ')
        print('\n')

    # 功能函数 4  获取RGB图像

    def getImageRGB(self):
        clientID = self.clientID
        cameraRGBHandle = self.cameraRGBHandle
        # 获得像素
        resolutionX = self.resolutionX
        resolutionY = self.resolutionY

        # sim.simxGetVisionSensorImage获取图像数据
        res1, resolution1, image_rgb = sim.simxGetVisionSensorImage(clientID, cameraRGBHandle, 0,
                                                                    sim.simx_opmode_blocking)

        image_rgb_r = [image_rgb[i] for i in range(0, len(image_rgb), 3)]
        image_rgb_r = np.array(image_rgb_r)
        image_rgb_r = image_rgb_r.reshape(resolutionY, resolutionX)
        image_rgb_r = image_rgb_r.astype(np.uint8)

        image_rgb_g = [image_rgb[i] for i in range(1, len(image_rgb), 3)]
        image_rgb_g = np.array(image_rgb_g)
        image_rgb_g = image_rgb_g.reshape(resolutionY, resolutionX)
        image_rgb_g = image_rgb_g.astype(np.uint8)

        image_rgb_b = [image_rgb[i] for i in range(2, len(image_rgb), 3)]
        image_rgb_b = np.array(image_rgb_b)
        image_rgb_b = image_rgb_b.reshape(resolutionY, resolutionX)
        image_rgb_b = image_rgb_b.astype(np.uint8)

        # 合并颜色通道
        result_rgb = cv2.merge([image_rgb_b, image_rgb_g, image_rgb_r])
        # 镜像翻转, opencv在这里返回的是一张翻转的图
        # coppeliasim返回的图像通常上下颠倒，
        result_rgb = cv2.flip(result_rgb, 0)
        return result_rgb

    # 功能函数 5 获取深度图像
    def getImageDepth(self):
        clientID = self.clientID
        cameraDepthHandle = self.cameraDepthHandle
        resolutionX = self.resolutionX
        resolutionY = self.resolutionY

        res2, resolution2, image_depth = sim.simxGetVisionSensorImage(clientID, cameraDepthHandle, 0,
                                                                      sim.simx_opmode_blocking)

        image_depth_r = [image_depth[i] for i in range(0, len(image_depth), 3)]
        image_depth_r = np.array(image_depth_r)
        image_depth_r = image_depth_r.reshape(resolutionY, resolutionX)
        image_depth_r = image_depth_r.astype(np.uint8)

        image_depth_g = [image_depth[i] for i in range(1, len(image_depth), 3)]
        image_depth_g = np.array(image_depth_g)
        image_depth_g = image_depth_g.reshape(resolutionY, resolutionX)
        image_depth_g = image_depth_g.astype(np.uint8)

        image_depth_b = [image_depth[i] for i in range(2, len(image_depth), 3)]
        image_depth_b = np.array(image_depth_b)
        image_depth_b = image_depth_b.reshape(resolutionY, resolutionX)
        image_depth_b = image_depth_b.astype(np.uint8)

        result_depth = cv2.merge([image_depth_b, image_depth_g, image_depth_r])
        # 镜像翻转, opencv在这里返回的是一张翻转的图
        result_depth = cv2.flip(result_depth, 0)

        # 黑白取反
        height, width, channels = result_depth.shape
        for row in range(height):
            for list in range(width):
                for c in range(channels):
                    pv = result_depth[row, list, c]
                    result_depth[row, list, c] = 255 - pv

        return result_depth

    # 功能函数 6 控制机械手RG2的开合
    # open rg2
    def openRG2(self):
        rgName = self.rgName
        clientID = self.clientID
        # simxCallScriptFunction() 调用嵌入式脚本，一个用于与模拟环境中的脚本进行交互的函数，允许你从外部程序控制模拟环境中的对象和执行脚本功能。
        # sim_scripttype_childscript：是一个枚举值，表示要调用的是一个子脚本（child script）
        res, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(clientID, rgName, \
                                                                                    sim.sim_scripttype_childscript,
                                                                                    'rg2Open', [], [], [], b'',
                                                                  sim.simx_opmode_blocking)
        print(res)

    # 功能函数 7 控制机械手 RG2的闭合
    def closeRG2(self):
        rgName = self.rgName
        clientID = self.clientID
        res, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(clientID, rgName, \
                                                                                    sim.sim_scripttype_childscript,
                                                                                    'rg2Close', [], [], [], b'',
                                                                                    sim.simx_opmode_blocking)

    # 功能函数 8 控制机械臂各关节旋转到joint_angle设定的角度。
    # joint_angle是这种形式: [0,0,0,0,0,0], 所有的关节都旋转到对应的角度
    def rotateAllAngle(self, joint_angle):
        clientID = self.clientID
        jointNum = self.jointNum
        RAD2DEG = self.RAD2DEG
        jointHandle = self.jointHandle

        # 暂停通信，用于存储所有控制命令一起发送
        sim.simxPauseCommunication(clientID, True)
        for i in range(jointNum):
            sim.simxSetJointTargetPosition(clientID, jointHandle[i], joint_angle[i] / RAD2DEG,
                                           sim.simx_opmode_oneshot)
        sim.simxPauseCommunication(clientID, False)
        # 调用此函数，即改变当前关节角度
        self.jointConfig = joint_angle

    # 功能函数 9 将第num个关节正转angle度
    # 与 8 区别：
    # rotateAllAngle 将机械臂关节转动到对应角度，转动30°的话就是：0--》30，45--》30，-15--》30
    # rotateCertainAnglePositive，是将机械臂在当前角度的基础上再转动对应角度
    def rotateCertainAnglePositive(self, num, angle):
        clientID = self.clientID
        RAD2DEG = self.RAD2DEG
        jointHandle = self.jointHandle
        jointConfig = self.jointConfig

        sim.simxSetJointTargetPosition(clientID, jointHandle[num], (jointConfig[num] + angle) / RAD2DEG,
                                       sim.simx_opmode_oneshot)
        jointConfig[num] = jointConfig[num] + angle

        self.jointConfig = jointConfig

    # 将第num个关节反转angle度
    def rotateCertainAngleNegative(self, num, angle):
        clientID = self.clientID
        RAD2DEG = self.RAD2DEG
        jointHandle = self.jointHandle
        jointConfig = self.jointConfig

        sim.simxSetJointTargetPosition(clientID, jointHandle[num], (jointConfig[num] - angle) / RAD2DEG,
                                       sim.simx_opmode_oneshot)
        jointConfig[num] = jointConfig[num] - angle

        self.jointConfig = jointConfig

    # convert array from sim to image
    def arrayToImage(self):
        path = "imgTemp\\frame.jpg"
        if os.path.exists(path):
            os.remove(path)
        ig = self.getImageRGB()
        cv2.imwrite(path, ig)

    # convert array from sim to depth image
    def arrayToDepthImage(self):
        path = "imgTempDep\\frame.jpg"
        if os.path.exists(path):
            os.remove(path)
        ig = self.getImageDepth()
        cv2.imwrite(path, ig)

# 控制程序
# 使用pygame模块控制机械臂，由于pygame是一个开发游戏的工具，所以使用pygame可以实现用键盘按键操控机械臂运动。
def main():
    robot = UR5_RG2()
    resolutionX = robot.resolutionX
    resolutionY = robot.resolutionY

    # angle = float(eval(input("please input velocity: ")))
    angle = 1

    pygame.init()
    # 1.创建一个游戏窗口，基于arrayToImage()
    screen = pygame.display.set_mode((resolutionX, resolutionY))
    screen.fill((255, 255, 255))
    pygame.display.set_caption("sim yolov3 ddpg pytorch")
    # 循环事件，按住一个键可以持续移动
    pygame.key.set_repeat(200, 50)

    while True:
        # RGB
        robot.arrayToImage()
        ig = pygame.image.load("imgTemp\\frame.jpg")
        # Depth
        # robot.arrayToDepthImage()
        # ig = pygame.image.load("imgTempDep\\frame.jpg")
        screen.blit(ig, (0, 0))
        pygame.display.update()

        key_pressed = pygame.key.get_pressed()
        for event in pygame.event.get():
            # 关闭程序
            if event.type == pygame.QUIT:
                sys.exit()
            if event.type == pygame.KEYDOWN:
                # p 退出程序
                if event.key == pygame.K_p:
                    sys.exit()
                # joinit 0
                # q /w 关节1正反转
                elif event.key == pygame.K_q:
                    robot.rotateCertainAnglePositive(0, angle)
                elif event.key == pygame.K_w:
                    robot.rotateCertainAngleNegative(0, angle)
                # joinit 1
                # a/s
                elif event.key == pygame.K_a:
                    robot.rotateCertainAnglePositive(1, angle)
                elif event.key == pygame.K_s:
                    robot.rotateCertainAngleNegative(1, angle)
                # joinit 2
                # z/x
                elif event.key == pygame.K_z:
                    robot.rotateCertainAnglePositive(2, angle)
                elif event.key == pygame.K_x:
                    robot.rotateCertainAngleNegative(2, angle)
                # joinit 3
                # e/r
                elif event.key == pygame.K_e:
                    robot.rotateCertainAnglePositive(3, angle)
                elif event.key == pygame.K_r:
                    robot.rotateCertainAngleNegative(3, angle)
                # joinit 4
                # d/f
                elif event.key == pygame.K_d:
                    robot.rotateCertainAnglePositive(4, angle)
                elif event.key == pygame.K_f:
                    robot.rotateCertainAngleNegative(4, angle)
                # joinit 5
                # c/v
                elif event.key == pygame.K_c:
                    robot.rotateCertainAnglePositive(5, angle)
                elif event.key == pygame.K_v:
                    robot.rotateCertainAngleNegative(5, angle)
                # close RG2
                # t关闭
                elif event.key == pygame.K_t:
                    robot.closeRG2()
                # # open RG2
                # y打开
                elif event.key == pygame.K_y:
                    robot.openRG2()
                # save Images
                # 空格保存图像
                elif event.key == pygame.K_SPACE:
                    rgbImg = robot.getImageRGB()
                    depthImg = robot.getImageDepth()
                    # 随机生成8位ascii码和数字作为文件名
                    ran_str = ''.join(random.sample(string.ascii_letters + string.digits, 8))
                    cv2.imwrite("saveImg\\rgbImg\\" + ran_str + "_rgb.jpg", rgbImg)
                    cv2.imwrite("saveImg\\depthImg\\" + ran_str + "_depth.jpg", depthImg)
                    print("save image")
                # reset angle
                elif event.key == pygame.K_l:
                    robot.rotateAllAngle([0, 0, 0, 0, 0, 0])
                    angle = float(eval(input("please input velocity: ")))
                else:
                    print("Invalid input, no corresponding function for this key!")


if __name__ == '__main__':
    main()
