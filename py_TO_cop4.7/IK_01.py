import sim

def main():
    # 连接到 CoppeliaSim
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID == -1:
        print('Failed connecting to remote API server')
        return

    # 获取对象句柄
    simTipHandle = sim.simxGetObjectHandle(clientID, './tip', sim.simx_opmode_oneshot_wait)
    simTargetHandle = sim.simxGetObjectHandle(clientID, './target', sim.simx_opmode_oneshot_wait)
    modelBaseHandle = sim.simxGetObjectHandle(clientID, '.', sim.simx_opmode_oneshot_wait)

    # 创建 IK 环境
    ikEnv = sim.simxCreateIkEnvironment(clientID, sim.simx_opmode_oneshot_wait)
    ikGroup = sim.simxCreateIkGroup(clientID, ikEnv, sim.simx_opmode_oneshot_wait)

    # 设置 IK 组的计算方法
    sim.simxSetIkGroupProperties(clientID, ikEnv, ikGroup, sim.sim_ik_pseudo_inverse, 0, 6, sim.simx_opmode_oneshot_wait)

    # 从场景中添加 IK 元素
    sim.simxAddIkElementFromScene(clientID, ikEnv, ikGroup, modelBaseHandle, simTipHandle, simTargetHandle, sim.sim_ik_constraint_pose, sim.simx_opmode_oneshot_wait)

    # 应用 IK 环境到场景中
    sim.simxApplyIkEnvironment(clientID, ikEnv, ikGroup, True, sim.simx_opmode_oneshot_wait)

    # 保持连接直到用户中断
    while True:
        pass

if __name__ == '__main__':
    main()