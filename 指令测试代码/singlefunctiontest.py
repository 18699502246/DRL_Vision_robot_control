import sim

# 导入CoppeliaSim的API库
sim = sim

sim.simxRMLMoveToJointPositions()
def resetJoints(self):
    clientID = self.clientID
    RAD2DEG = self.RAD2DEG
    fkDrivingJoints = self.fkDrivingJoints
    initialJointPositions = {0,0,0,0}
    
    for i in range(len(fkDrivingJoints)):
        sim.simxSetJointTargetPosition(clientID, fkDrivingJoints[i], initialJointPositions[i]/RAD2DEG, vrep.simx_opmode_oneshot)
    
    # 等待运动完成
    sim.simxSleep(2)
# 连接到CoppeliaSim
sim.simxFinish(-1)  # 关闭所有连接以保证连接干净
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 2000, 5)  # 启动连接

if clientID != -1:
    print("Connected to CoppeliaSim")
    
    # 启动模拟
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)
    
    # 等待一段时间确保模拟已经启动和脚本初始化完成
    import time
    time.sleep(1)
    
    # 调用resetPosition函数
    result, ret_ints, ret_floats, ret_strings, ret_bytes = sim.simxCallScriptFunction(clientID, 'irb360', sim.sim_scripttype_childscript,
                                         'sysCall_init', [], [], [], bytearray(), sim.simx_opmode_blocking)
    if result == sim.simx_return_ok:
        print("resetPosition function called successfully")
    else:
        print("Error calling resetPosition function:", result)
    
    # 停止模拟
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    
    # 结束连接
    sim.simxFinish(clientID)
else:
    print("Failed to connect to CoppeliaSim")
