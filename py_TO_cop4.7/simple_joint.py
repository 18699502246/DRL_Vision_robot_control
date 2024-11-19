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