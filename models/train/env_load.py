import jax
import jax.numpy as jnp
from mujoco import mjx
import mujoco
from brax.envs.base import PipelineEnv, State
from brax.io import html # 用于导出可视化结果

class SpotEnv(PipelineEnv):
    def __init__(self, model_path):
        # 加载 MuJoCo 模型
        sys_model = mujoco.MjModel.from_xml_path(model_path)
        # 将其转化为 MJX 格式用于并行计算
        sys_mjx = mjx.put_model(sys_model)
        
        # 初始化 PipelineEnv (Brax 的封装)
        super().__init__(sys_mjx, backend='mjx', n_frames=4)

    def reset(self, rng):
        # 初始状态：让 Spot 站在空中
        pipeline_state = self.pipeline_init(self.sys.qpos0, self.sys.qvel0)
        obs = self._get_obs(pipeline_state)
        reward, done = jnp.zeros(2)
        return State(pipeline_state, obs, reward, done, {})

    def step(self, state, action):
        # 执行物理仿真步进
        pipeline_state = self.pipeline_step(state.pipeline_state, action)
        obs = self._get_obs(pipeline_state)
        
        # 核心：定义奖励函数 (这里简化为向前走的奖励)
        forward_reward = pipeline_state.qvel[0] # X 轴速度
        healthy_reward = 1.0 # 只要不倒下就给分
        reward = forward_reward + healthy_reward
        
        # 终止条件 (如果翻车就结束)
        done = jnp.where(pipeline_state.qpos[2] < 0.2, 1.0, 0.0)
        
        return state.replace(pipeline_state=pipeline_state, obs=obs, reward=reward, done=done)

    def _get_obs(self, pipeline_state):
        # 简单观察空间：关节位置 + 关节速度
        return jnp.concatenate([pipeline_state.qpos, pipeline_state.qvel])

# 实例化环境
env = SpotEnv(model_path='robot/boston_dynamics_spot/scene.xml')
print("✅ Spot 强化学习环境在 Ubuntu CPU 上构建成功！")