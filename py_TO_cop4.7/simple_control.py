# -*- coding:utf-8 -*-

"""
控制机械臂的各个关节旋转
"""

import os
import sys
import math
import time
import pygame
import sim
import numpy as np

# 在main函数开始之前添加以下代码qq
if not os.path.exists("saveImg"):
    os.makedirs("saveImg")

# 定义Delta机械臂控制类
class DeltaArm:
    # 初始化函数
    def __init__(self):
        print('仿真开始')
        sim.simxFinish(-1)  # 关闭潜在的连接
        # 连接到V-rep
        self.clientID = self.connect_to_simulator()
        self.joint_handles = self.get_joint_handles()
        self.joint_angles = self.get_initial_joint_angles()

    # 连接到模拟器
    def connect_to_simulator(self):
        while True:
            clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
            if clientID > -1:
                print("Connection success!")
                break
            else:
                time.sleep(0.2)
                print("Failed connecting to remote API server!")
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
        return clientID

    # 获取关节句柄
    def get_joint_handles(self):
        joint_handles = []
        for i in range(3):  # 假设Delta机械臂有3个关节
            _, handle = sim.simxGetObjectHandle(self.clientID, f'DrivingJoint{i+1}', sim.simx_opmode_blocking)
            joint_handles.append(handle)
        return joint_handles

    # 获取初始关节角度
    def get_initial_joint_angles(self):
        joint_angles = []
        for handle in self.joint_handles:
            _, angle = sim.simxGetJointPosition(self.clientID, handle, sim.simx_opmode_blocking)
            joint_angles.append(angle)
        return joint_angles

    # 控制单个关节角度
    def set_joint_angle(self, joint_index, angle):
        joint_handle = self.joint_handles[joint_index]
        # sim.simxSetJointTargetPosition(self.clientID, joint_handle, self.joint_angles[joint_index] + angle*math.pi/180, sim.simx_opmode_oneshot)
        sim.simxSetJointPosition(self.clientID, joint_handle, self.joint_angles[joint_index] + angle*math.pi/180, sim.simx_opmode_oneshot)
        self.joint_angles[joint_index] += angle

    # 重置关节角度
    def reset_robot(self):
        for i, handle in enumerate(self.joint_handles):
            sim.simxSetJointTargetPosition(self.clientID, handle, 0, sim.simx_opmode_oneshot)
        self.joint_angles = [0] * len(self.joint_handles)
 
    # 断开连接
    def __del__(self):
        sim.simxFinish(self.clientID)
        print('Simulation end')

# 控制程序
def main():
    robot = DeltaArm()

    pygame.init()
    pygame.key.set_repeat(200, 50)  # 设置按键重复间隔，首次延迟200毫秒，之后每50毫秒检测一次按键状态
    # 1.创建一个游戏窗口，基于arrayToImage()
    screen = pygame.display.set_mode((480, 640))
    screen.fill((255, 255, 255))
    pygame.display.set_caption("sim yolov3 ddpg pytorch")
    clock = pygame.time.Clock()

    angle = 1

    while True:
        key_pressed = pygame.key.get_pressed()  # 获取当前所有按键的状态
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p:
                    sys.exit()
                elif event.key == pygame.K_q:
                    print('q键按下')
                    robot.set_joint_angle(0, 30)
                    
                elif event.key == pygame.K_w:
                    robot.set_joint_angle(0, -angle)
                elif event.key == pygame.K_a:
                    robot.set_joint_angle(1, angle)
                elif event.key == pygame.K_s:
                    robot.set_joint_angle(1, -angle)
                elif event.key == pygame.K_z:
                    robot.set_joint_angle(2, angle)
                elif event.key == pygame.K_x:
                    robot.set_joint_angle(2, -angle)
                elif event.key == pygame.K_l:
                    robot.reset_robot()

        clock.tick(60)

if __name__ == '__main__':
    main()