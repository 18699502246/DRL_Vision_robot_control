import sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient #引入 CoppeliaSim 的零MQ远程API客户端库，用于与仿真环境进行通信。

# CartPole simulation model for VREP

# 2. 定义类
class CartPoleSimModel():
    # 3.初始化方法
    def __init__(self, name='CartPole'):
        """
        :param: name: string
            name of objective
        """
        super(self.__class__, self).__init__()
        self.name = name
        self.client_ID = None
        self.sim = None

        self.prismatic_joint_handle = None
        self.revolute_joint_handle = None

    # 5.初始化仿真模型，获取关节句柄
    def initializeSimModel(self, sim):

        self.sim = sim

        self.prismatic_joint_handle = self.sim.getObjectHandle('prismatic_joint')
        print('get object prismatic joint ok.')

        self.revolute_joint_handle = self.sim.getObjectHandle('revolute_joint')
        print('get object revolute joint ok.')

        # Get the joint position
        # q = vrep_sim.simxGetJointPosition(self.client_ID, self.prismatic_joint_handle, vrep_sim.simx_opmode_streaming)
        # q = vrep_sim.simxGetJointPosition(self.client_ID, self.revolute_joint_handle, vrep_sim.simx_opmode_streaming)

        # Set the initialized position for each joint
        # 初始化关节的扭矩，设置为0
        self.setJointTorque(0)

    # 6.获取关节位置
    def getJointPosition(self, joint_name):
        """
        :param: joint_name: string
        """
        q = 0
        if joint_name == 'prismatic_joint':
            q = self.sim.getJointPosition(self.prismatic_joint_handle)
        elif joint_name == 'revolute_joint':
            q = self.sim.getJointPosition(self.revolute_joint_handle)
        else:
            print('Error: joint name: \' ' + joint_name + '\' can not be recognized.')

        return q

# 7.获取关节速度
    def getJointVelocity(self, joint_name):
        """
        :param: joint_name: string
        """
        v = 0
        if joint_name == 'prismatic_joint':
            v = self.sim.getJointVelocity(self.prismatic_joint_handle)
        elif joint_name == 'revolute_joint':
            v = self.sim.getJointVelocity(self.revolute_joint_handle)
        else:
            print('Error: joint name: \' ' + joint_name + '\' can not be recognized.')

        return v

    def setJointPosition(self, joint_name, pos):
        """
        :param: joint_name: string
        """
        if joint_name == 'prismatic_joint':
            self.sim.setJointPosition(self.prismatic_joint_handle, pos)
        elif joint_name == 'revolute_joint':
            self.sim.setJointPosition(self.revolute_joint_handle, pos)
        else:
            print('Error: joint name: \' ' + joint_name + '\' can not be recognized.')

        return 0

    def setJointTorque(self, torque):
        if torque >= 0:
            self.sim.setJointTargetVelocity(self.prismatic_joint_handle, 1000)
        else:
            self.sim.setJointTargetVelocity(self.prismatic_joint_handle, -1000)

        self.sim.setJointMaxForce(self.prismatic_joint_handle, abs(torque))
