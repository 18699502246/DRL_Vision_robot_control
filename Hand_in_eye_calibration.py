# Make sure to have the add-on "ZMQ remote API" running in
# CoppeliaSim. Do not launch simulation, but run this script

from mpl_toolkits.mplot3d import Axes3D
from src.coppeliasim_zmqremoteapi_client import RemoteAPIClient
import depth_image_encoding as encoding
import cv2
import numpy as np
from matplotlib import pyplot as plt
import math
import struct
import time
import matplotlib
matplotlib.use('TkAgg')


# 设置棋盘格规格（内角点的数量）
chessboard_size = (10, 7)

# 3D 世界坐标的点准备（标定板的真实物理尺寸）
square_size = 0.015  # 假设每个格子的边长是15毫米（可以根据实际情况调整）
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0],
                       0:chessboard_size[1]].T.reshape(-1, 2) * square_size


# 储存所有图片的3D点和2D点
objpoints = []  # 3D 点 (真实世界空间坐标)
imgpoints = []  # 2D 点 (图像平面中的点)


print('Program started')

client = RemoteAPIClient()
sim = client.require('sim')

robot_poses_R = []  # 10 个旋转矩阵
robot_poses_t = []  # 10 个平移向量

camera_poses_R = []  # 10 个相机的旋转矩阵
camera_poses_t = []  # 10 个相机的平移向量


targetjoinPos1 = [0 * math.pi / 180, 45 * math.pi / 180, -60 *
                  math.pi / 180, 0 * math.pi / 180, 90 * math.pi / 180, 0 * math.pi / 180]
targetPos = []


targetPos1 = [-0.1100547923916193, -0.01595811170705489, 0.8466254222789784,
              0.560910589456253, -0.5609503047415633, 0.4305463900323013, 0.43051582116844406]
targetPos2 = [-0.16595811170705488, -0.01595811170705487, 0.8216254222789784,
              0.5417925804901749, -0.4495188049772251, 0.575726166263469, 0.415852167455231]
targetPos3 = [-0.1100547923916193, -0.01595811170705487, 0.8216254222789784,
              3.21292597227468e-05, 0.7932742610364036, -0.6088644721541127, 1.7127530004424877e-05]
targetPos4 = [-0.1100547923916193, -0.01595811170705489, 0.9216254222789784,
              1.4077336111962496e-05, 0.7932754374087434, -0.608862940314109, 1.085602215830202e-05]
targetPos5 = [-0.1100547923916193, -0.015958111707054884, 0.8216254222789784,
              0.579152193496431, -0.5416388515007398, 0.4546027806731813, 0.40564319680902283]
targetPos6 = [-0.1100547923916193, -0.015958111707054884, 0.8216254222789784,
              0.5415953148716405, -0.5791980963813808, 0.4056650408034863, 0.45457667640033966]
targetPos7 = [-0.2100547923916193, -0.015958111707054898, 0.8816254222789784,
              0.5212515544329563, -0.5212938037943111, 0.4777945964218841, 0.47776763259657323]
targetPos8 = [-0.15000547923916193, -0.015958111707054898, 0.8016254222789784,
              0.5212515544329563, -0.5212938037943111, 0.4777945964218841, 0.47776763259657323]
targetPos9 = [-0.15000547923916193, -0.015958111707054898, 0.8016254222789784,
              0.5212515544329563, -0.5212938037943111, 0.4777945964218841, 0.47776763259657323]
targetPos10 = [-0.1010000547923916193, -0.015958111707054898, 0.8016254222789784,
               0.5212515544329563, -0.5212938037943111, 0.4777945964218841, 0.47776763259657323]
targetPos11 = []
targetPos12 = []


targetPos.append(targetPos1)
targetPos.append(targetPos2)
targetPos.append(targetPos3)
targetPos.append(targetPos4)
targetPos.append(targetPos5)
targetPos.append(targetPos6)
targetPos.append(targetPos7)
targetPos.append(targetPos8)
targetPos.append(targetPos9)
targetPos.append(targetPos10)


def moveToConfig(robotColor, handles, maxVel, maxAccel, maxJerk, targetConf):
    sim = robotColor
    currentConf = []
    for i in range(len(handles)):
        currentConf.append(sim.getJointPosition(handles[i]))
    params = {}
    params['joints'] = handles
    params['targetPos'] = targetConf
    params['maxVel'] = maxVel
    params['maxAccel'] = maxAccel
    params['maxJerk'] = maxJerk
    # one could also use sim.moveToConfig_init, sim.moveToConfig_step and sim.moveToConfig_cleanup
    sim.moveToConfig(params)


def MovetoPose(robothandle, maxVel, maxAccel, maxJerk, targetpost):
    sim = robothandle

    params['ik'] = {'tip': tipHandle, 'target': targetHandle}
    # params['object'] = targetHandle
    params['targetPose'] = targetpost
    params['maxVel'] = maxVel
    params['maxAccel'] = maxAccel
    params['maxJerk'] = maxJerk

    sim.moveToPose(params)


def draw_axes(img, camera_matrix, dist_coeffs, rvec, tvec, axis_length=0.15, origin=[0, 0, 0]):
    """
    在图像中绘制标定板的中心点和 X、Y 轴。
    :param img: 要绘制的图像
    :param camera_matrix: 相机内参矩阵
    :param dist_coeffs: 畸变系数
    :param rvec: 相机相对于标定板的旋转向量
    :param tvec: 相机相对于标定板的平移向量
    :param axis_length: 坐标轴的长度（单位：米）
    :return: 绘制了坐标轴的图像
    """
    # 定义坐标轴的3D点: (0, 0, 0) 是原点, X 轴 (axis_length, 0, 0), Y 轴 (0, axis_length, 0), Z 轴 (0, 0, axis_length)
    axis_points_3d = np.float32([origin, [axis_length, origin[1], origin[2]], [
                                origin[0], axis_length, origin[2]], [origin[0], origin[1], axis_length]]).reshape(-1, 3)

    # 使用 cv2.projectPoints 将 3D 坐标投影到 2D 图像平面
    axis_points_2d, _ = cv2.projectPoints(
        axis_points_3d, rvec, tvec, camera_matrix, dist_coeffs)

    # 提取 2D 点并转换为整数类型
    origin = tuple(map(int, axis_points_2d[0].ravel()))  # 原点
    x_axis = tuple(map(int, axis_points_2d[1].ravel()))  # X 轴
    y_axis = tuple(map(int, axis_points_2d[2].ravel()))  # Y 轴
    z_axis = tuple(map(int, axis_points_2d[3].ravel()))  # Z 轴

    # 绘制中心点（原点）
    cv2.circle(img, origin, 5, (0, 0, 255), -1)  # 红色圆圈表示原点

    # 绘制 X 轴 (红色)
    cv2.line(img, origin, x_axis, (255, 0, 0), 2)

    # 绘制 Y 轴 (绿色)
    cv2.line(img, origin, y_axis, (0, 255, 0), 2)
    # 绘制 Z 轴 (蓝色)
    cv2.line(img, origin, z_axis, (0, 0, 255), 2)

    return img


def pixel_to_camera_coordinates(u, v, Z, camera_matrix):
    # 从相机内参矩阵中提取焦距和主点
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    # 将像素坐标转换为相机坐标系下的三维点
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

    # 返回三维坐标
    return np.array([X, Y, Z])


class ImageStreamDisplay:
    def __init__(self, resolution):
        # 创建一个包含两个子图的图形
        self.fig, (self.ax1, self.ax2) = plt.subplots(
            1, 2, figsize=(10, 6))  # 1行2列
        # 设置子图的标题
        self.ax1.set_title('RGB img')
        self.ax2.set_title('Depth img')
        # 初始化两个图像显示对象，使用零数组作为占位符，并设置animated=True以启用动画
        self.im_display1 = self.ax1.imshow(
            np.zeros([resolution[1], resolution[0], 3]), animated=True)
        self.im_display2 = self.ax2.imshow(np.zeros(
            [resolution[1], resolution[0]]), animated=True, cmap='gray', vmin=0, vmax=3.5)

        # 自动调整子图布局以避免重叠
        self.fig.tight_layout()
        # 开启交互模式
        plt.ion()
        # 显示图形
        plt.show()

    def displayUpdated(self, rgbBuffer1, rgbBuffer2):
        # 更新两个图像显示对象
        # 注意：对于imshow，通常使用set_data而不是set_array
        self.im_display1.set_data(rgbBuffer1)
        self.im_display2.set_data(rgbBuffer2)

        # plt.colorbar()
        # 刷新事件并暂停以更新显示
        self.fig.canvas.flush_events()
        plt.pause(0.01)  # 暂停一小段时间以允许GUI更新


targetHandle = sim.getObject('/UR5/Target')
tipHandle = sim.getObject('/UR5/Tip')
robotHandle = sim.getObject('/UR5')

visionSensorHandle = sim.getObject('/UR5/3D_camera')
deepSensorHandle = sim.getObject('/UR5/3D_camera/depth')
chessboardHandle = sim.getObject('/Plane')

jointHandles = []
for i in range(6):
    joint_handle=sim.getObject('/UR5/joint', {'index': i})
    jointHandles.append(joint_handle)
    cyclic, interval = sim.getJointInterval(joint_handle)

    if cyclic:
        print(f"关节 {joint_handle} 是循环关节，其角度在 -180° 到 +180° 之间循环。")
    else:
        lower_limit_deg = math.degrees(interval[0])             # 最小角度值
        upper_limit_deg = math.degrees(interval[0] + interval[1])  # 最大角度值
        print(f"关节 {joint_handle} 的运动范围是：{lower_limit_deg:.2f}° 到 {upper_limit_deg:.2f}°")

jvel = 310 * math.pi / 180
jaccel = 100 * math.pi / 180
jjerk = 80 * math.pi / 180

jmaxVel = [jvel, jvel, jvel, jvel, jvel, jvel]
jmaxAccel = [jaccel, jaccel, jaccel, jaccel, jaccel, jaccel]
jmaxJerk = [jjerk, jjerk, jjerk, jjerk, jjerk, jjerk]

vel = 30
accel = 1.0
jerk = 1

maxVel = [vel, vel, vel, vel]
maxAccel = [accel, accel, accel, accel]
maxJerk = [jerk, jerk, jerk, jerk]

sim.startSimulation()
# cv2.namedWindow('Camera calibration')
display = ImageStreamDisplay([640, 480])

print('------Perform camera calibration------')
# 初始拍照位置
moveToConfig(sim, jointHandles, jmaxVel, jmaxAccel, jmaxJerk, targetjoinPos1)
index = 0
for i in range(10):

    goalTr = targetPos[i].copy()
    params = {}
    params['ik'] = {'tip': tipHandle, 'target': targetHandle}
    # params['object'] = targetHandle
    params['targetPose'] = goalTr
    params['maxVel'] = maxVel
    params['maxAccel'] = maxAccel
    params['maxJerk'] = maxJerk

    sim.moveToPose(params)

    # 这里是尝试转动角度的

    # EulerAngles=sim.getObjectOrientation(targetHandle,tipHandle)

    # # EulerAngles[1]+=15* math.pi / 180
    # EulerAngles[0]+=25* math.pi / 180

    # sim.setObjectOrientation(targetHandle,EulerAngles,tipHandle)
    # goalTr=sim.getObjectPose(targetHandle,robotHandle)
    # # goalTr[0]=goalTr[1]-0.15

    # params = {}
    # params['ik'] = {'tip': tipHandle, 'target': targetHandle}
    # # params['object'] = targetHandle
    # params['targetPose'] = goalTr
    # params['maxVel'] = maxVel
    # params['maxAccel'] = maxAccel
    # params['maxJerk'] = maxJerk

    # sim.moveToPose(params)

    img, [resX, resY] = sim.getVisionSensorImg(visionSensorHandle)
    img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)

    # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
    # (consistent with the axes of vision sensors, pointing Z outwards, Y up)
    # and color format is RGB triplets, whereas OpenCV uses BGR:
    img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
    # img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 找到棋盘格的角点
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    # 如果找到角点，添加到点集
    if ret == True:
        objpoints.append(objp)
        imgpoints.append(corners)
        # 显示角点
        img = cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        # 使用 solvePnP 获取旋转向量和平移向量
        # ret, rvec, tvec = cv2.solvePnP(objpoints, imgpoints, camera_matrix, dist_coeffs)

    # 获取深度图像
    Deepdate = sim.getVisionSensorDepth(deepSensorHandle, 1)
    # 解包为浮点数数组
    num_floats = Deepdate[1][0] * Deepdate[1][1]
    depth_data = struct.unpack(f'{num_floats}f', Deepdate[0])

    # 将数据转为 numpy 数组
    depth_array = np.array(depth_data)
    depth_image = depth_array.reshape((Deepdate[1][1],  Deepdate[1][0]))
    depth_image = np.flipud(depth_image)
    deepcolor = encoding.FloatArrayToRgbImage(
        depth_image, scale_factor=256*3.5*1000)

    display.displayUpdated(img, depth_image)

# 进行相机标定
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

# 计算标定的误差
total_error = 0
errors = []
# 遍历每张图像
for objp, imgp, rvec, tvec in zip(objpoints, imgpoints, rvecs, tvecs):
    # 将三维物体点转换为相机坐标系下的二维图像点
    projected_imgpoints, _ = cv2.projectPoints(objp, rvec, tvec, mtx, dist)

    # 计算原始 imgpoints 和 projected_imgpoints 之间的误差
    error = cv2.norm(imgp, projected_imgpoints, cv2.NORM_L2) / \
        len(projected_imgpoints)
    errors.append(error)
    total_error += error

# 计算所有图像的平均误差
mean_error = total_error / len(objpoints)

# 打印每幅图像的误差
for i, error in enumerate(errors):
    print(f"图像 {i+1} 的误差: {error}")

# 打印平均误差
print(f"所有图像的平均误差: {mean_error}")


# 打印相机内参和畸变系数
print("Camera Matrix:\n", mtx)
print("Distortion Coefficients:\n", dist)
print('camera carlibraed Done')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 定义坐标轴的箭头长度
arrow_length = 0.5

# 绘制坐标轴箭头
ax.quiver(0, 0, 0, arrow_length, 0, 0, color='r',
          length=arrow_length, label='X axis')
ax.quiver(0, 0, 0, 0, arrow_length, 0, color='g',
          length=arrow_length, label='Y axis')
ax.quiver(0, 0, 0, 0, 0, arrow_length, color='b',
          length=arrow_length, label='Z axis')

for objp, rvec, tvec in zip(objpoints, rvecs, tvecs):
    # 将 rvec 转换为旋转矩阵
    R, _ = cv2.Rodrigues(rvec)

    # 计算每个物体点在相机坐标系下的位置
    board_points_camera = np.dot(R, objp.T).T + tvec.T

    # 绘制标定板的三维物体点
    # ax.scatter(objp[:, 0], objp[:, 1], objp[:, 2], c='r', marker='o', label="Object Points")

    # 绘制标定板的三维点在相机坐标系下的投影点
    ax.scatter(board_points_camera[:, 0], board_points_camera[:, 1],
               board_points_camera[:, 2], c='b', marker='x', label="Projected Points in Camera")

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.title('Calibration Board Points in 3D')
# plt.legend()
plt.show()


print('------Perform hand-eye calibration------')
# 初始拍照位置
moveToConfig(sim, jointHandles, jmaxVel, jmaxAccel, jmaxJerk, targetjoinPos1)


for i in range(10):
    goalTr = targetPos[i].copy()
    # goalTr[2] = goalTr[2] - 0.2
    params = {}
    params['ik'] = {'tip': tipHandle, 'target': targetHandle}
    # params['object'] = targetHandle
    params['targetPose'] = goalTr
    params['maxVel'] = maxVel
    params['maxAccel'] = maxAccel
    params['maxJerk'] = maxJerk
    sim.moveToPose(params)

    img, [resX, resY] = sim.getVisionSensorImg(visionSensorHandle)
    img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)

    # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
    # (consistent with the axes of vision sensors, pointing Z outwards, Y up)
    # and color format is RGB triplets, whereas OpenCV uses BGR:
    img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
    # img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 找到棋盘格的角点
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    # 如果找到角点，添加到点集
    if ret == True:
        # 使用 solvePnP 获取旋转向量和平移向量
        ret, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)
        # 将 rvec 转换为旋转矩阵
        R, _ = cv2.Rodrigues(rvec)

        camera_tvec_matrix = tvec.reshape(-1)
        # 获取到机器人夹爪的姿态
        Targetpost = sim.getObjectPose(tipHandle)
        pose_matrix = sim.poseToMatrix(Targetpost)
        # 提取旋转矩阵 (3x3)
        robot_rotation_matrix = np.array([
            [pose_matrix[0], pose_matrix[1], pose_matrix[2]],
            [pose_matrix[4], pose_matrix[5], pose_matrix[6]],
            [pose_matrix[8], pose_matrix[9], pose_matrix[10]]
        ])
        # 提取平移向量 (P0, P1, P2)
        robot_tvec_matrix = np.array(
            [pose_matrix[3], pose_matrix[7], pose_matrix[11]])

        # 加入到手眼标定数据
        robot_poses_R.append(robot_rotation_matrix)
        robot_poses_t.append(robot_tvec_matrix)
        camera_poses_R.append(R)
        camera_poses_t.append(camera_tvec_matrix)

        img = draw_axes(img, mtx, dist, rvec, tvec, 0.015*7)

        # 获取深度图像
        Deepdate = sim.getVisionSensorDepth(deepSensorHandle, 1)
        # 解包为浮点数数组
        num_floats = Deepdate[1][0] * Deepdate[1][1]
        depth_data = struct.unpack(f'{num_floats}f', Deepdate[0])

        # 将数据转为 numpy 数组
        depth_array = np.array(depth_data)
        depth_image = depth_array.reshape((Deepdate[1][1],  Deepdate[1][0]))
        depth_image = np.flipud(depth_image)
        deepcolor = encoding.FloatArrayToRgbImage(
            depth_image, scale_factor=256*3.5*1000)

        display.displayUpdated(img, depth_image)


print("开始手眼标定...")
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    robot_poses_R, robot_poses_t, camera_poses_R, camera_poses_t, method=cv2.CALIB_HAND_EYE_TSAI
)

camera_pose_intip = sim.getObjectPose(visionSensorHandle, tipHandle)
camera_matrix_intip = sim.poseToMatrix(camera_pose_intip)
eulerAngles = sim.getEulerAnglesFromMatrix(camera_matrix_intip)


print("相机到机器人末端的旋转矩阵：\n", R_cam2gripper)
print("相机到机器人末端的平移向量：\n", t_cam2gripper)

# 保存手眼标定的结果
# np.savez('hand_eye_calibration.dat', R_cam2gripper=R_cam2gripper, t_cam2gripper=t_cam2gripper)
# print("手眼标定结果已保存到 hand_eye_calibration.npz")
print("手眼标定完成")
print('------Test hand-eye calibration------')
# 测试标定的结果：

# 初始拍照位置
moveToConfig(sim, jointHandles, jmaxVel, jmaxAccel, jmaxJerk, targetjoinPos1)

# 10个拍照位置的验证
# for i in range(10):
while True:
    goalTr = targetPos[i].copy()
    # goalTr[2] = goalTr[2] - 0.2
    params = {}
    params['ik'] = {'tip': tipHandle, 'target': targetHandle}
    # params['object'] = targetHandle
    params['targetPose'] = goalTr
    params['maxVel'] = maxVel
    params['maxAccel'] = maxAccel
    params['maxJerk'] = maxJerk
    sim.moveToPose(params)

    img, [resX, resY] = sim.getVisionSensorImg(visionSensorHandle)
    img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)

    # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
    # (consistent with the axes of vision sensors, pointing Z outwards, Y up)
    # and color format is RGB triplets, whereas OpenCV uses BGR:
    img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
    # img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # 获取深度图像
    Deepdate = sim.getVisionSensorDepth(deepSensorHandle, 1)
    num_floats = Deepdate[1][0] * Deepdate[1][1]
    depth_data = struct.unpack(f'{num_floats}f', Deepdate[0])
    depth_array = np.array(depth_data)
    depth_image = depth_array.reshape((Deepdate[1][1],  Deepdate[1][0]))
    depth_image = np.flipud(depth_image)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 找到棋盘格的角点
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    # 如果找到角点，使用第一个角点来转换为机器人的坐标
    if ret == True:
        u = corners[0][0][0]
        v = corners[0][0][1]
        Z = depth_image[int(v), int(u)]

        img = cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        ret, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)

        # 计算相机坐标系下的三维点
        P_cam = pixel_to_camera_coordinates(u, v, Z, mtx)

        print("相机坐标系下的三维点 P_cam:", P_cam)
        # t_cam2gripper=t_cam2gripper.reshape(-1)
        # 计算物体在手爪坐标系中的位置

        # 计算点在末端坐标系下的坐标 P_end
        P_end = np.dot(R_cam2gripper, P_cam) + t_cam2gripper.reshape(-1)
        # sim.setObjectPosition(targetHandle, P_end,tipHandle)

        # 计算点在基座坐标系下的坐标 P_base
        tip_matrix = sim.getObjectMatrix(tipHandle)
        # 提取旋转矩阵 R_end_to_base (3x3)
        R_end_to_base = np.array([
            [tip_matrix[0], tip_matrix[1], tip_matrix[2]],
            [tip_matrix[4], tip_matrix[5], tip_matrix[6]],
            [tip_matrix[8], tip_matrix[9], tip_matrix[10]]
        ])

        # 提取平移向量 t_end_to_base (3x1)
        t_end_to_base = np.array([
            [tip_matrix[3]],
            [tip_matrix[7]],
            [tip_matrix[11]]
        ])

        P_base = np.dot(R_end_to_base, P_end) + t_end_to_base.reshape(-1)
        # sim.setObjectPosition(targetHandle, P_base)
        Tip_pose = sim.getObjectPose(tipHandle)

        # 将旋转向量转换为旋转矩阵
        R_board_to_camera, _ = cv2.Rodrigues(rvec)

        # 计算标定板相对于末端的旋转矩阵和平移向量
        R_board_to_end = R_cam2gripper @ R_board_to_camera
        t_board_to_end = R_cam2gripper @ tvec.flatten() + t_cam2gripper.flatten()

        # 计算标定板相对于世界坐标系的旋转矩阵和平移向量
        R_board_to_world = R_end_to_base @ R_board_to_end
        t_board_to_world = R_end_to_base @ t_board_to_end + t_end_to_base.flatten()
        chessboard_matrix = sim.getObjectMatrix(targetHandle)

        cal_chessboard_matrix = np.array([R_board_to_world[0][0], R_board_to_world[0][1], R_board_to_world[0][2], t_board_to_world[0],
                                          R_board_to_world[1][0], R_board_to_world[1][1], R_board_to_world[1][2], t_board_to_world[1],
                                          R_board_to_world[2][0], R_board_to_world[2][1], R_board_to_world[2][2], t_board_to_world[2]])

        sim.setObjectMatrix(targetHandle,  cal_chessboard_matrix)

        goalTr = Tip_pose.copy()
        goalTr[0]=P_base[0]
        goalTr[1]=P_base[1]
        goalTr[2]=P_base[2]
        params = {}
        params['ik'] = {'tip': tipHandle, 'target': targetHandle}
        # params['object'] = targetHandle
        params['targetPose'] = goalTr
        params['maxVel'] = maxVel
        params['maxAccel'] = maxAccel
        params['maxJerk'] = maxJerk
        sim.moveToPose(params)

    display.displayUpdated(img, depth_image)

sim.stopSimulation()
