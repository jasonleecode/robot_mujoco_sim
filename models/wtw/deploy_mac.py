import torch
import torch.nn as nn

# === 1. 定义模型结构 (从源码复刻并简化) ===
class ActorCritic(nn.Module):
    def __init__(self, 
                 num_obs=48,              # 机器人当前状态维度
                 num_privileged_obs=48,   # 训练时的作弊信息 (推理时不需要，但为了加载权重得占位)
                 num_obs_history=30*48,   # 历史观测 (关键输入!)
                 num_actions=12):         # 12个电机
        super().__init__()

        # --- 配置参数 (来自原代码 AC_Args) ---
        actor_hidden_dims = [512, 256, 128]
        critic_hidden_dims = [512, 256, 128]
        activation = nn.ELU()
        
        # 适应性模块参数 (Adaptation Module)
        # 输入: 历史数据 -> 输出: 隐变量 (Latent)
        adaptation_branch_hidden_dims = [256, 32] 
        latent_dim = 18  # env_factor_encoder_branch_latent_dims

        # --- A. 适应性模块 (Student 的直觉) ---
        # 负责从历史数据中“猜”出地形信息
        adaptation_layers = []
        adaptation_layers.append(nn.Linear(num_obs_history, adaptation_branch_hidden_dims[0]))
        adaptation_layers.append(activation)
        adaptation_layers.append(nn.Linear(adaptation_branch_hidden_dims[0], adaptation_branch_hidden_dims[1])) # 32
        # 注意：原代码逻辑里，最后一层输出的是 latent_dim (18)
        # 这里需要仔细对齐原代码的循环逻辑:
        # if l == len(branch_hidden_dims) - 1: linear(..., branch_latent_dim)
        adaptation_layers.append(nn.Linear(adaptation_branch_hidden_dims[1], latent_dim)) 
        self.adaptation_module = nn.Sequential(*adaptation_layers)

        # --- B. 策略网络 (Actor Body) ---
        # 负责根据 (当前状态 + 猜测的地形) 输出动作
        actor_layers = []
        # 输入是: 当前状态(num_obs) + 隐变量(latent_dim)
        actor_layers.append(nn.Linear(num_obs + latent_dim, actor_hidden_dims[0]))
        actor_layers.append(activation)
        for i in range(len(actor_hidden_dims) - 1):
            actor_layers.append(nn.Linear(actor_hidden_dims[i], actor_hidden_dims[i+1]))
            actor_layers.append(activation)
        actor_layers.append(nn.Linear(actor_hidden_dims[-1], num_actions))
        self.actor_body = nn.Sequential(*actor_layers)

        # --- C. 占位模块 (为了防止 load_state_dict 报错) ---
        # 环境编码器 (Teacher)
        self.env_factor_encoder = nn.Sequential(
            nn.Linear(18, 256), activation, # 假设输入是18
            nn.Linear(256, 128), activation,
            nn.Linear(128, 18)
        )
        # 价值网络 (Critic)
        self.critic_body = nn.Sequential(nn.Linear(num_obs+latent_dim, 1))
        
        # 动作标准差 (RL 训练参数)
        self.std = nn.Parameter(torch.ones(num_actions))

    def forward(self, obs, obs_history):
        """
        推理时的前向传播 (Student Mode)
        Args:
            obs: 当前时刻的观测 [Batch, num_obs]
            obs_history: 历史观测 [Batch, num_obs_history]
        """
        # 1. 用历史数据猜 Latent
        latent = self.adaptation_module(obs_history)
        
        # 2. 拼接 当前状态 + Latent
        input_vec = torch.cat((obs, latent), dim=-1)
        
        # 3. 输出动作
        actions = self.actor_body(input_vec)
        return actions

# === 2. 加载与测试函数 ===
def load_and_run(pt_path):
    print(f"🔄 正在加载: {pt_path}")
    
    # 加载权重
    checkpoint = torch.load(pt_path, map_location='cpu')
    state_dict = checkpoint['model_state_dict'] if 'model_state_dict' in checkpoint else checkpoint

    # --- 自动探测维度 (重要!) ---
    # 尝试从权重中推断 input size，防止 config 不匹配
    try:
        # adaptation_module.0.weight 的 shape 是 [256, input_dim]
        hist_input_dim = state_dict['adaptation_module.0.weight'].shape[1]
        print(f"🕵️ 探测到历史输入维度 (History Dim): {hist_input_dim}")
        
        # actor_body.0.weight 的 shape 是 [512, obs_dim + latent_dim]
        # 我们已知 latent_dim 通常是 18 (或者 16/32)
        # 这里假设 latent_dim = 18 (根据原代码 default)
        actor_first_layer_dim = state_dict['actor_body.0.weight'].shape[1]
        obs_dim = actor_first_layer_dim - 18 
        print(f"🕵️ 探测到观测维度 (Obs Dim): {obs_dim}")
        
    except KeyError:
        print("⚠️ 无法自动探测维度，使用默认值...")
        hist_input_dim = 30 * 48 # 假设
        obs_dim = 48

    # 实例化模型
    model = ActorCritic(num_obs=obs_dim, num_obs_history=hist_input_dim)
    
    # 加载权重 (使用 strict=False 忽略掉不匹配的层，比如 critic 结构可能有细微差异)
    try:
        model.load_state_dict(state_dict, strict=False)
        print("✅ 权重加载成功！")
    except Exception as e:
        print(f"❌ 权重加载失败: {e}")
        return

    # 设置为评估模式
    model.eval()
    
    # --- 模拟推理 ---
    # 构造假数据
    dummy_obs = torch.randn(1, obs_dim)
    dummy_history = torch.randn(1, hist_input_dim)
    
    with torch.no_grad():
        action = model(dummy_obs, dummy_history)
        
    print("\n🎉 推理成功！")
    print(f"输入 Obs 形状: {dummy_obs.shape}")
    print(f"输入 History 形状: {dummy_history.shape}")
    print(f"输出 Action 形状: {action.shape}")
    print("输出值示例:", action[0][:4].numpy(), "...")

if __name__ == "__main__":
    # 替换成你的 .pt 文件路径
    load_and_run("unitree_go1.pt")