# 1. 建议使用 Conda 创建一个隔离环境
conda create -n spot_vla python=3.10
conda activate spot_vla

# 2. 安装 PyTorch (Mac M3 优化版)
pip install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cpu
# 或者直接 pip install torch (现在的稳定版对 M3 支持也不错)

# 3. 安装 OpenVLA 的依赖
# 进入子模块目录
cd models/openvla
# 修改 requirements (如果需要): 
# OpenVLA 的原始 requirements 可能包含针对 NVIDIA 的库 (如 flash-attn)，在 Mac 上安装会失败。
# 你可能需要手动注释掉 requirements.txt 里的 'flash-attn'。
pip install -r requirements.txt

# 4. 安装 transformers 和 diffusers (VLA 核心库)
pip install transformers diffusers