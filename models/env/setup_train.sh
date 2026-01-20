# 创建名为 mjx_env 的虚拟环境
python3 -m venv mjx_env

# 激活环境
source mjx_env/bin/activate

# 升级 pip 到最新版
pip install --upgrade pip

# 安装 JAX 基础库
pip install jaxlib jax

# 安装 Apple 的 Metal 插件 (使 JAX 支持 M4 GPU)
pip install jax-metal

# 安装 MuJoCo 相关
pip install mujoco mujoco-mjx brax