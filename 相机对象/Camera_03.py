import os
import random
import string
import time
import numpy as np
import cv2
import sim

# rgb图像更加立体了
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

        # 使用流模式获取图像，提高效率
        for cameraRGBHandle in self.cameraRGBHandles:
            sim.simxGetVisionSensorImage(clientID, cameraRGBHandle, 0, sim.simx_opmode_streaming)
        for cameraDepthHandle in self.cameraDepthHandles:
            sim.simxGetVisionSensorImage(clientID, cameraDepthHandle, 0, sim.simx_opmode_streaming)


    
    def get_image(self, camera_handle):
        """
        从指定的相机获取图像。

        :param camera_handle: 相机句柄
        :return: 图像数据
        """
        ret, resolution, image = sim.simxGetVisionSensorImage(self.clientID, camera_handle, 0, sim.simx_opmode_buffer)
        if ret == sim.simx_return_ok:
            # 将图像数据转换为NumPy数组并确保数据在uint8范围内
            image = np.array(image, dtype=np.int32)  # 使用int32以避免溢出
            min_value = np.min(image)  # 找到最小值
            
            # 将所有值偏移，使最小值变为0
            image = image - min_value
            image = np.clip(image, 0, 255)  # 将数据限制在0到255范围内
            image = image.astype(np.uint8)  # 转换为uint8类型
            
            image.resize([resolution[1], resolution[0], 3])
            # 转换颜色通道顺序并翻转图像
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = cv2.flip(image, 0)
            return image
        else:
            return None
    def get_rgb_image(self, cameraRGBHandle):
        """
        从指定的RGB相机获取图像。

        :param cameraRGBHandle: RGB相机句柄
        :return: RGB图像数据
        """
        return self.get_image(cameraRGBHandle)

    def get_depth_image(self, cameraDepthHandle):
        """
        从指定的深度相机获取图像。

        :param cameraDepthHandle: 深度相机句柄
        :return: 深度图像数据
        """
        depth_image = self.get_image(cameraDepthHandle)
        if depth_image is not None:
            # 黑白取反
            depth_image = cv2.bitwise_not(depth_image)
        return depth_image

    def getImagesFromAllCameras(self):
        """
        从所有相机获取RGB和深度图像。

        :return: RGB图像列表和深度图像列表
        """
        rgb_images = [self.get_rgb_image(cameraRGBHandle) for cameraRGBHandle in self.cameraRGBHandles]
        depth_images = [self.get_depth_image(cameraDepthHandle) for cameraDepthHandle in self.cameraDepthHandles]
        return rgb_images, depth_images

    def saveImages(self, rgb_images, depth_images, folder_name):
        """
        保存图像到指定文件夹。

        :param rgb_images: RGB图像列表
        :param depth_images: 深度图像列表
        :param folder_name: 保存文件夹名称（Air或Floor）
        """
        for i, (rgb_image, depth_image) in enumerate(zip(rgb_images, depth_images)):
            if rgb_image is not None and depth_image is not None:
                ran_str = ''.join(random.sample(string.ascii_letters + string.digits, 8))
                cv2.imwrite(os.path.join("saveImg", "rgbImg", folder_name, ran_str + "_rgb.jpg"), rgb_image)
                cv2.imwrite(os.path.join("saveImg", "depthImg", folder_name, ran_str + "_depth.jpg"), depth_image)
        print("Images saved")

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

# 示例使用
if __name__ == "__main__":
    create_directories()

    clientID = connect_to_simulation()
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)  # 仿真初始化
    cameraRGBHandles, cameraDepthHandles = get_camera_handles(clientID)

    resolutionX = 640  # 图像宽度
    resolutionY = 480  # 图像高度

    imageHandler = ImageHandler(clientID, cameraRGBHandles, cameraDepthHandles, resolutionX, resolutionY)

    # 示例：实时获取和显示图像
    while True:
        rgb_images, depth_images = imageHandler.getImagesFromAllCameras()

        # 显示图像
        for i, (rgb_image, depth_image) in enumerate(zip(rgb_images, depth_images)):
            if rgb_image is not None and depth_image is not None:
                cv2.imshow(f'RGB Camera {i+1}', rgb_image)
                cv2.imshow(f'Depth Camera {i+1}', depth_image)

        # 按下 's' 键保存图像
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            imageHandler.saveImages(rgb_images, depth_images, "Air")
            imageHandler.saveImages(rgb_images, depth_images, "Floor")

        # 按下 'q' 键退出
        if key == ord('q'):
            break

    cv2.destroyAllWindows()
    sim.simxFinish(clientID)  # 结束连接