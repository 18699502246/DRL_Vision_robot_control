from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

# 使用coppeliasim_zmqremoteapi_client建立连接通道
client = RemoteAPIClient()
sim = client.getObject('sim')

# 使用sim.py文件提供的API进行仿真控制
def start_simulation():
    sim.startSimulation()

def stop_simulation():
    sim.stopSimulation()

def move_to_config(goal_config, info, max_velocity=None, max_acceleration=None, max_jerk=None):
    # 设置默认值
    max_velocity = max_velocity or info['maxAngVel']
    max_acceleration = max_acceleration or info['maxAngAccel']
    max_jerk = max_jerk or info['maxAngJerk']

    # 获取关节对象
    joints = info['fkDrivingJoints'] if not info['ikMode'] else info['ikDrivingJoints']

    # 获取当前关节位置
    start_config = [sim.getJointPosition(joint) for joint in joints]

    # 准备参数
    params = {
        'pos': start_config,
        'targetPos': goal_config,
        'maxVel': max_velocity,
        'maxAccel': max_acceleration,
        'maxJerk': max_jerk,
        'callback': move_to_config_callback,
        'auxData': info
    }

    # 调用moveToConfig函数
    sim.moveToConfig(params)

def move_to_config_callback(data):
    # 处理回调函数
    pass

if __name__ == "__main__":
    # 示例调用
    info = {
        'robotHandle': sim.getObject('IRB360'),  # 假设机械臂的名称为IRB360
        'ikMode': False,
        'ikEnv': simIK.createEnvironment(),
        'mainIkGroup': simIK.createGroup(ikEnv),
        'platformIkElement': simIK.addElementFromScene(ikEnv, ikGroup_main, base, ikTip, ikTarget, simIK.constraint_position),
        'axisIkGroup': simIK.createGroup(ikEnv),
        'bridgeIkGroups': [simIK.createGroup(ikEnv) for _ in range(6)],
        'fkDrivingJoints': [sim.getObject('joint1'), sim.getObject('joint2'), sim.getObject('joint3'), sim.getObject('joint4')],  # 假设关节名称
        'ikDrivingJoints': [sim.getObject('cartesianX'), sim.getObject('cartesianY'), sim.getObject('cartesianZ'), sim.getObject('joint4')],  # 假设关节名称
        'fkDrivingJoints_inIkEnv': [mapping[fkDrivingJoints[1]], mapping[fkDrivingJoints[2]], mapping[fkDrivingJoints[3]]]
    }
    goal_config = [-80 * math.pi / 180, 0 * math.pi / 180, 0 * math.pi / 180, 45 * math.pi / 180]

    # 启动仿真
    start_simulation()

    # 控制机械臂运动
    move_to_config(goal_config, info)

    # 等待一段时间
    time.sleep(5)

    # 停止仿真
    stop_simulation()
