import os
import random
import string
import time
import numpy as np
import cv2
import sim

# 创建基础文件夹
def create_directories():
    """
    创建用于保存图像的文件夹。
    """
    dir_paths = [
        "saveImg",
        os.path.join("saveImg", "rgbImg"),
        os.path.join("saveImg", "depthImg"),
        os.path.join("saveImg", "rgbImg", "Air"),
        os.path.join("saveImg", "rgbImg", "Floor"),
        os.path.join("saveImg", "depthImg", "Air"),
        os.path.join("saveImg", "depthImg", "Floor")
    ]
    for path in dir_paths:
        if not os.path.exists(path):
            os.makedirs(path)

class ImageHandler:
    def __init__(self, clientID, cameraRGBHandles, cameraDepthHandles, resolutionX, resolutionY):
        """
        初始化图像处理类。

        :param clientID: CoppeliaSim客户端ID
        :param cameraRGBHandles: RGB相机句柄列表
        :param cameraDepthHandles: 深度相机句柄列表
        :param resolutionX: 图像宽度
        :param resolutionY: 图像高度
        """
        self.clientID = clientID
        self.cameraRGBHandles = cameraRGBHandles
        self.cameraDepthHandles = cameraDepthHandles
        self.resolutionX = resolutionX
        self.resolutionY = resolutionY

    def getImageRGB(self, cameraRGBHandle):
        """
        从指定的RGB相机获取图像。

        :param cameraRGBHandle: RGB相机句柄
        :return: RGB图像数据
        """
        clientID = self.clientID
        resolutionX = self.resolutionX
        resolutionY = self.resolutionY

        # 获取RGB图像数据
        res1, resolution1, image_rgb = sim.simxGetVisionSensorImage(clientID, cameraRGBHandle, 0, sim.simx_opmode_blocking)

        if res1 != sim.simx_return_ok:
            raise Exception(f"Failed to get RGB image from camera with handle {cameraRGBHandle}")

        # 将原始数据转换为可处理的格式
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

        # 合并颜色通道并翻转图像
        result_rgb = cv2.merge([image_rgb_b, image_rgb_g, image_rgb_r])
        result_rgb = cv2.flip(result_rgb, 0)
        return result_rgb

    def getImageDepth(self, cameraDepthHandle):
        """
        从指定的深度相机获取图像。

        :param cameraDepthHandle: 深度相机句柄
        :return: 深度图像数据
        """
        clientID = self.clientID
        resolutionX = self.resolutionX
        resolutionY = self.resolutionY

        # 获取深度图像数据
        res2, resolution2, image_depth = sim.simxGetVisionSensorImage(clientID, cameraDepthHandle, 0, sim.simx_opmode_blocking)

        if res2 != sim.simx_return_ok:
            raise Exception(f"Failed to get depth image from camera with handle {cameraDepthHandle}")

        # 将原始数据转换为可处理的格式
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

        # 合并颜色通道并翻转图像
        result_depth = cv2.merge([image_depth_b, image_depth_g, image_depth_r])
        result_depth = cv2.flip(result_depth, 0)

        # 黑白取反
        result_depth = cv2.bitwise_not(result_depth)
        return result_depth

    def getImagesFromAllCameras(self):
        """
        从所有相机获取RGB和深度图像。

        :return: RGB图像列表和深度图像列表
        """
        rgb_images = []
        depth_images = []

        for cameraRGBHandle in self.cameraRGBHandles:
            rgb_image = self.getImageRGB(cameraRGBHandle)
            rgb_images.append(rgb_image)

        for cameraDepthHandle in self.cameraDepthHandles:
            depth_image = self.getImageDepth(cameraDepthHandle)
            depth_images.append(depth_image)

        return rgb_images, depth_images

    def saveImages(self, rgb_images, depth_images, folder_name):
        """
        保存图像到指定文件夹。

        :param rgb_images: RGB图像列表
        :param depth_images: 深度图像列表
        :param folder_name: 保存文件夹名称（Air或Floor）
        """
        for i, (rgb_image, depth_image) in enumerate(zip(rgb_images, depth_images)):
            ran_str = ''.join(random.sample(string.ascii_letters + string.digits, 8))
            cv2.imwrite(os.path.join("saveImg", "rgbImg", folder_name, ran_str + "_rgb.jpg"), rgb_image)
            cv2.imwrite(os.path.join("saveImg", "depthImg", folder_name, ran_str + "_depth.jpg"), depth_image)
        print("Images saved")

def connect_to_simulation():
    """
    连接到CoppeliaSim客户端。

    :return: 客户端ID
    """
    print('仿真开始')
    sim.simxFinish(-1)  # 关闭潜在的连接
    while True:
        clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
        if clientID > -1:
            print("Connection success!")
            return clientID
        else:
            time.sleep(0.2)
            print("Failed connecting to remote API server!")

def get_camera_handles(clientID):
    """
    获取相机句柄。

    :param clientID: 客户端ID
    :return: RGB相机句柄列表和深度相机句柄列表
    """
    kinect_names = ['kinect1', 'kinect2']
    cameraRGBHandles = []
    cameraDepthHandles = []

    for i, kinect in enumerate(kinect_names):
        rgb_name = f'kinect_rgb{i+1}'
        _, cameraRGBHandle = sim.simxGetObjectHandle(clientID, rgb_name, sim.simx_opmode_blocking)
        cameraRGBHandles.append(cameraRGBHandle)
        print(f'{kinect} RGB Handle:', cameraRGBHandle)

        depth_name = f'kinect_depth{i+1}'
        _, cameraDepthHandle = sim.simxGetObjectHandle(clientID, depth_name, sim.simx_opmode_blocking)
        cameraDepthHandles.append(cameraDepthHandle)
        print(f'{kinect} Depth Handle:', cameraDepthHandle)

    return cameraRGBHandles, cameraDepthHandles


# 创建基础文件夹
create_directories()

# 连接到仿真和获取相机句柄
clientID = connect_to_simulation()
sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)  # 仿真初始化    
cameraRGBHandles, cameraDepthHandles = get_camera_handles(clientID)

# 使用ImageHandler类
resolutionX = 640  # 图像宽度
resolutionY = 480  # 图像高度

imageHandler = ImageHandler(clientID, cameraRGBHandles, cameraDepthHandles, resolutionX, resolutionY)



# 设置程序运行时间
start_time = time.time()
duration = 30 # 秒

while time.time() - start_time < duration:
    rgb_images, depth_images = imageHandler.getImagesFromAllCameras()

    # 显示图像
    for i, (rgb_image, depth_image) in enumerate(zip(rgb_images, depth_images)):
        cv2.imshow(f"RGB Image {i+1}", rgb_image)
        cv2.imshow(f"Depth Image {i+1}", depth_image)

    # 保存图像
    imageHandler.saveImages(rgb_images, depth_images, "Air")
    imageHandler.saveImages(rgb_images, depth_images, "Floor")
    # 检查按键事件以提前退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()