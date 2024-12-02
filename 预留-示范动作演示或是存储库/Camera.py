import os
import random
import string
import time
import numpy as np
import cv2
import sim

class ImageHandler:
    def __init__(self, clientID, cameraRGBHandles, cameraDepthHandles, resolutionX, resolutionY):
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

    def get_image(self, camera_handle, max_retries=10):
        retries = 0
        while retries < max_retries:
            ret, resolution, image = sim.simxGetVisionSensorImage(self.clientID, camera_handle, 0, sim.simx_opmode_buffer)
            if ret == sim.simx_return_ok:
                image = np.array(image, dtype=np.int16)
                image = np.clip(image, -128, 127).astype(np.uint8)
                image = image.reshape([resolution[1], resolution[0], 3])
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                image = cv2.flip(image, 0)
                return image
            else:
                print(f"Attempt {retries + 1} to get image from camera handle {camera_handle} failed, return code: {ret}")
                retries += 1
                time.sleep(0.1)
        print(f"Failed to get image from camera handle {camera_handle} after {max_retries} retries.")
        return None

    def get_rgb_image(self, cameraRGBHandle):
        return self.get_image(cameraRGBHandle)

    def get_depth_image(self, cameraDepthHandle):
        depth_image = self.get_image(cameraDepthHandle)
        if depth_image is not None:
            depth_image = cv2.bitwise_not(depth_image)
        return depth_image

    def get_images_from_all_cameras(self):
        rgb_images = [self.get_rgb_image(cameraRGBHandle) for cameraRGBHandle in self.cameraRGBHandles]
        depth_images = [self.get_depth_image(cameraDepthHandle) for cameraDepthHandle in self.cameraDepthHandles]
        return rgb_images, depth_images

    def save_images(self, rgb_images, depth_images, folder_name):
        for i, (rgb_image, depth_image) in enumerate(zip(rgb_images, depth_images)):
            if rgb_image is not None and depth_image is not None:
                ran_str = ''.join(random.sample(string.ascii_letters + string.digits, 8))
                cv2.imwrite(os.path.join("saveImg", "rgbImg", folder_name, ran_str + "_rgb.jpg"), rgb_image)
                cv2.imwrite(os.path.join("saveImg", "depthImg", folder_name, ran_str + "_depth.jpg"), depth_image)
        print("Images saved")

def create_directories():
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
    kinect_names = ['kinect1', 'kinect2']
    cameraRGBHandles = []
    cameraDepthHandles = []

    for i, kinect in enumerate(kinect_names):
        rgb_name = f'kinect_rgb{i+1}'
        _, cameraRGBHandle = sim.simxGetObjectHandle(clientID, rgb_name, sim.simx_opmode_blocking)
        if _ != sim.simx_return_ok:
            print(f"Failed to get handle for {rgb_name}")
        cameraRGBHandles.append(cameraRGBHandle)

        depth_name = f'kinect_depth{i+1}'
        _, cameraDepthHandle = sim.simxGetObjectHandle(clientID, depth_name, sim.simx_opmode_blocking)
        if _ != sim.simx_return_ok:
            print(f"Failed to get handle for {depth_name}")
        cameraDepthHandles.append(cameraDepthHandle)

    return cameraRGBHandles, cameraDepthHandles

if __name__ == "__main__":
    create_directories()

    clientID = connect_to_simulation()
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)  # 仿真初始化
    cameraRGBHandles, cameraDepthHandles = get_camera_handles(clientID)

    resolutionX = 640  # 图像宽度
    resolutionY = 480  # 图像高度

    imageHandler = ImageHandler(clientID, cameraRGBHandles, cameraDepthHandles, resolutionX, resolutionY)

    try:
        while True:
            rgb_images, depth_images = imageHandler.get_images_from_all_cameras()

            for i, (rgb_image, depth_image) in enumerate(zip(rgb_images, depth_images)):
                if rgb_image is not None and depth_image is not None:
                    cv2.imshow(f'RGB Camera {i+1}', rgb_image)
                    cv2.imshow(f'Depth Camera {i+1}', depth_image)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                imageHandler.save_images(rgb_images, depth_images, "Air")
                imageHandler.save_images(rgb_images, depth_images, "Floor")

            if key == ord('q'):
                break

    finally:
        cv2.destroyAllWindows()
        sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
        sim.simxFinish(clientID)