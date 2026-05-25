#!/bin/bash
# Ubuntu 20.04 + RTX 3080 (CUDA 12.1) 训练环境配置
# 使用独立 conda 环境避免与现有 PyTorch 冲突

set -e

ENV_NAME="mjx_train"

echo "=== 创建 conda 环境: ${ENV_NAME} (Python 3.11) ==="
conda create -n ${ENV_NAME} python=3.11 -y
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate ${ENV_NAME}

echo "=== 安装 JAX (CUDA 12) ==="
# jax[cuda12] 自动匹配 CUDA 12.x，兼容 RTX 30/40 系列
pip install --upgrade pip
pip install "jax[cuda12]" -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html

echo "=== 安装 MuJoCo / MJX / Brax ==="
pip install mujoco mujoco-mjx brax

echo "=== 安装导出工具（CPU PyTorch，仅用于 export_to_torch.py）==="
pip install torch --index-url https://download.pytorch.org/whl/cpu

echo "=== 安装其他依赖 ==="
pip install numpy scipy tqdm tensorboard

echo ""
echo "=== 安装完成 ==="
echo "激活环境: conda activate ${ENV_NAME}"
echo "验证 JAX GPU: python -c \"import jax; print(jax.devices())\""
