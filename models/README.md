# Models — Training & Deployment Pipeline

## Architecture

Train with JAX/Brax on GPU → export to TorchScript → load in C++ via LibTorch.

```
models/
├── env/
│   ├── setup_train.sh          # Mac M4 conda env (JAX Metal)
│   └── setup_train_ubuntu.sh   # Ubuntu 20.04 + RTX 3080 conda env (JAX CUDA 12)
├── train/
│   ├── go2_env.py              # Go2 Brax/MJX environment (48-dim Isaac Lab obs)
│   ├── go2_train.py            # Brax PPO training → go2_checkpoint.pkl + actor_go2.pt
│   └── export_to_torch.py      # Standalone: checkpoint.pkl → TorchScript actor.pt
├── legged_gym/
│   ├── model_300.pt            # Legged_gym checkpoint (300 iters, undertrained)
│   ├── actor_300.pt            # TorchScript fallback (48→128→64→32→12)
│   └── actor_go2.pt            # [generated] Brax-trained TorchScript actor
└── options/
    └── wtw/                    # Walk-These-Ways reference (obs=70, history×30, latent=2)
```

## Quickstart (Ubuntu 20.04 + RTX 3080)

```bash
# 1. Create conda environment (once)
bash models/env/setup_train_ubuntu.sh
conda activate mjx_train

# 2. Train Go2 (~1-2h on RTX 3080, 50M steps)
cd <repo_root>
python models/train/go2_train.py
# → models/legged_gym/go2_checkpoint.pkl
# → models/legged_gym/actor_go2.pt

# 3. (Optional) Re-export from existing checkpoint
python models/train/export_to_torch.py \
    models/legged_gym/go2_checkpoint.pkl \
    models/legged_gym/actor_go2.pt

# 4. Build C++ sim with LibTorch
cmake .. -DUSE_TORCH=ON -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.1/bin/nvcc
make -j$(nproc)
```

## Observation Space (48-dim, Isaac Lab standard)

| idx    | content                     | scale  |
|--------|-----------------------------|--------|
| 0-2    | base linear vel (body)      | ×2.0   |
| 3-5    | base angular vel (body)     | ×0.25  |
| 6-8    | projected gravity           | ×1.0   |
| 9-11   | command [vx, vy, wz]        | scaled |
| 12-23  | dof_pos − nominal           | ×1.0   |
| 24-35  | dof_vel                     | ×0.05  |
| 36-47  | last_action                 | ×1.0   |

## PD Control (C++ deployment)

The C++ sim uses `go2.xml` (torque actuators). The PD layer converts policy output to torques:

```
τ = kp × (action × 0.25 + nominal − q) + kd × (0 − dq)
kp = 50.0,  kd = 0.5   (matches scene_mjx.xml training config)
```

Joint order: FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf,
             RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf

Nominal: [0, 0.9, -1.8] per leg.

## Notes

- `scene_mjx.xml` has built-in position servo (kp=50, kd=0.5, forcerange=±24 Nm)
- Training action is clipped to [-1, 1]; effective joint offset range = ±0.25 rad
- `normalize_observations=False` — the env already applies Isaac Lab scales and clips to [-5, 5]
- Spot policy model: train separately, place at `models/legged_gym/actor_spot.pt`
