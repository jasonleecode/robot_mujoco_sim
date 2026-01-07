# scripts/vla_bridge.py
import sys
import os
import time

# === 关键步骤：把子模块路径加入 sys.path ===
# 这样你就可以直接 import openvla 里的模块了
current_dir = os.path.dirname(os.path.abspath(__file__))
vla_path = os.path.join(current_dir, "../models/openvla")
sys.path.append(vla_path)

# 现在可以导入 OpenVLA 相关的库了
# (注意：OpenVLA 官方示例通常直接使用 transformers 的 AutoModelForVision2Seq)
from transformers import AutoModelForVision2Seq, AutoProcessor
from PIL import Image
import torch

# 假设你有 Python 版的 DDS 库 (CycloneDDS python binding)
# from cyclonedds.domain import DomainParticipant
# from cyclonedds.topic import Topic
# from cyclonedds.sub import DataReader
# from cyclonedds.pub import DataWriter

def main():
    print("Loading OpenVLA model on Mac M3 (MPS)...")
    
    # 1. 加载模型 (使用 4-bit 量化以适应 Mac 内存，需安装 bitsandbytes，或加载 float16)
    # 注意：Mac 上 bitsandbytes 支持尚不完善，建议先尝试 float16
    processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
    
    # device_map="auto" 在 Mac 上可能不完美，建议手动指定 device
    # model = AutoModelForVision2Seq.from_pretrained(
    #     "openvla/openvla-7b", 
    #     torch_dtype=torch.bfloat16, 
    #     trust_remote_code=True
    # ).to("mps") 
    
    print("Model loaded!")

    # 2. 循环：接收图像 -> 推理 -> 发送指令
    while True:
        # TODO: 从 DDS 接收图像数据 (Bytes -> PIL Image)
        # raw_img = dds_reader.take()
        # image = Image.frombytes(...)
        
        prompt = "A robot dog walking forward"
        
        # inputs = processor(prompt, image, return_tensors="pt").to("mps")
        # action = model.predict_action(**inputs)
        
        # TODO: 将 action 发送回 C++ 控制器
        # dds_writer.write(action)
        
        time.sleep(0.1) # VLA 推理较慢，控制频率

if __name__ == "__main__":
    main()