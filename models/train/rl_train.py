import jax
import jax.numpy as jnp
from datetime import datetime
import pickle
import os

from brax.io import html, model
from brax.training.agents.ppo import train as ppo

# 假设你之前定义的 SpotEnv 已经导入
# from your_env_file import SpotEnv 

# 1. 创建保存目录
save_dir = f"train_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
os.makedirs(save_dir, exist_ok=True)

# 2. 定义训练配置
print(f"[{datetime.now()}] 正在编译训练函数 (CPU 编译可能需要 1-2 分钟)...")
train_fn = ppo.train(
    environment=env,
    num_timesteps=1_000_000, 
    num_envs=128,            
    learning_rate=3e-4,
    unroll_length=20,
    batch_size=32,
    num_minibatches=16,      # 增加 minibatch 提升 CPU 稳定性
    seed=42
)

# 使用 jax.jit 包裹整个训练过程（可选，ppo.train 内部已有部分优化）
# 注意：第一次运行 params, metrics = train_fn() 会触发真正的编译

# 3. 开始训练
print(f"[{datetime.now()}] 训练正式开始...")
params, metrics = train_fn()
print(f"[{datetime.now()}] 训练结束！最终奖励值: {metrics['eval/episode_reward'][-1]:.2f}")

# --- 功能增加：保存模型 ---
model_path = os.path.join(save_dir, "spot_params.pkl")
with open(model_path, "wb") as f:
    # 这里的 params 包含了神经网络所有的权重
    pickle.dump(params, f)
print(f"✅ 模型参数已保存至: {model_path}")

# --- 功能增加：导出可视化 HTML ---
print("正在生成可视化录像...")

# 重新初始化一个环境进行推理测试
jit_env_reset = jax.jit(env.reset)
jit_env_step = jax.jit(env.step)
jit_inference_fn = jax.jit(ppo.make_inference_fn(env.observation_size, env.action_size, params))

states = []
rng = jax.random.PRNGKey(0)
state = jit_env_reset(rng)

# 录制 200 帧（大约 10 秒的动作）
for _ in range(200):
    states.append(state.pipeline_state)
    action, _ = jit_inference_fn(state.obs, rng)
    state = jit_env_step(state, action)

# 导出为 HTML 页面
html_path = os.path.join(save_dir, "spot_animation.html")
with open(html_path, "w") as f:
    f.write(html.render(env.sys.tree(), states))

print(f"🎬 可视化视频已导出至: {html_path}")
print("你可以将此文件拷贝到 Mac，直接用 Chrome 浏览器打开查看。")