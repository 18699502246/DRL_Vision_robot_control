import torch
import os
os.environ['CUDA_VISIBLE_DEVICES'] ='0'

print("CUDA 可用性:", torch.cuda.is_available())
print("CUDA 设备数量:", torch.cuda.device_count())

if torch.cuda.is_available():
    print("当前使用的设备:", torch.cuda.current_device())
    print("设备名称:", torch.cuda.get_device_name(torch.cuda.current_device()))
else:
    print("CUDA 不可用")