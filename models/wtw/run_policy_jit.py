import torch

def run_policy():
    print("🚀 正在加载 JIT 模型...")
    
    try:
        actor = torch.jit.load("body_latest.jit", map_location='cpu')
        adapter = torch.jit.load("adaptation_module_latest.jit", map_location='cpu')
        print("✅ 模型加载成功！")
    except Exception as e:
        print(f"❌ 加载失败: {e}")
        return

    # === 关键参数 ===
    # 原始观测维度 (包含了关节位置、速度、重力向量、动作指令等)
    obs_dim = 70  
    # 历史长度 (Walk-These-Ways 默认是 30 帧)
    history_len = 30
    # 历史总维度
    num_history = history_len * obs_dim  # 2100

    # 构造假数据
    # 注意：在真实部署中，你需要维护一个 buffer，把最新的 obs 不断 push 进去
    dummy_history = torch.randn(1, num_history)
    
    print(f"\n📊 维度检查:")
    print(f"   History Input: {dummy_history.shape} (应为 1x2100)")

    with torch.no_grad():
        # 1. 适应性模块 (History -> Latent)
        latent = adapter(dummy_history)
        print(f"   Latent Output: {latent.shape} (应为 1x2)")

        # 2. 策略网络 (History + Latent -> Action)
        # ❌ 错误写法: torch.cat((dummy_obs, latent), dim=-1)
        # ✅ 正确写法: 拼接 历史(2100) + Latent(2) = 2102
        input_vec = torch.cat((dummy_history, latent), dim=-1)
        
        print(f"   Actor Input:   {input_vec.shape} (应为 1x2102)")
        
        # 3. 推理
        actions = actor(input_vec)

    print("\n🎉 推理成功！")
    print(f"🤖 机器人动作输出 (12个电机指令):")
    print(actions[0].numpy())

if __name__ == "__main__":
    run_policy()