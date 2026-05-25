"""Train Go2 locomotion policy with Brax PPO on GPU (JAX + MJX).

Usage (from repo root, inside mjx_train conda env):
    python models/train/go2_train.py

Outputs:
    models/legged_gym/go2_checkpoint.pkl    -- Brax PPO params
    models/legged_gym/actor_go2.pt          -- TorchScript actor for C++ deployment
"""
import functools
import os
import pickle
import time
from datetime import datetime
from pathlib import Path

import jax
import jax.numpy as jnp

# ---------------------------------------------------------------------------
# Validate GPU is available before doing anything expensive
# ---------------------------------------------------------------------------
print(f"JAX version: {jax.__version__}")
print(f"JAX devices: {jax.devices()}")
if not any(str(d).startswith("cuda") or str(d).startswith("gpu") or "cuda" in str(d).lower()
           for d in jax.devices()):
    print("[WARNING] No GPU detected. Training on CPU will be very slow.")

from brax.training.agents.ppo import train as ppo
from brax.training.agents.ppo import networks as ppo_networks
from brax.training import distribution

from go2_env import Go2Env

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
REPO_ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = str(REPO_ROOT / "robot/unitree_go2/scene_mjx.xml")
OUT_DIR = REPO_ROOT / "models/legged_gym"
OUT_DIR.mkdir(parents=True, exist_ok=True)

TRAIN_CFG = dict(
    num_timesteps=80_000_000,   # ~40min — stability-first run
    num_evals=16,
    reward_scaling=1.0,
    episode_length=1000,
    normalize_observations=False,  # obs already scaled in env
    action_repeat=1,
    unroll_length=32,    # covers >1 full gait cycle (25 steps @ 0.5s period)
    num_minibatches=32,
    num_updates_per_batch=4,
    discounting=0.99,
    learning_rate=3e-4,
    entropy_cost=1e-2,   # increased from 5e-3 to encourage more exploration
    num_envs=2048,       # parallel envs on GPU
    batch_size=1024,
    max_grad_norm=1.0,   # gradient clipping prevents NaN propagation
    seed=8,
)

# Policy network: 48 → 256 → 128 → 12 (× 2 for mean+logstd in PPO)
POLICY_HIDDEN = (256, 128)

# ---------------------------------------------------------------------------
# Network factory — matches architecture we'll export to TorchScript
# ---------------------------------------------------------------------------

def make_networks(
    observation_size: int,
    action_size: int,
    preprocess_observations_fn=None,
    **kwargs,
):
    """Custom PPO network factory: smaller actor for faster C++ inference."""
    import flax.linen as nn
    from brax.training.types import PreprocessObservationFn

    # ppo.train passes preprocess_observations_fn as the obs normalizer;
    # when normalize_observations=False it's the identity function.
    kwargs_net = {}
    if preprocess_observations_fn is not None:
        kwargs_net["preprocess_observations_fn"] = preprocess_observations_fn

    return ppo_networks.make_ppo_networks(
        observation_size=observation_size,
        action_size=action_size,
        policy_hidden_layer_sizes=POLICY_HIDDEN,
        value_hidden_layer_sizes=(256, 256),
        activation=nn.elu,
        distribution_type="tanh_normal",  # squashes actions to (-1, 1)
        **kwargs_net,
    )


# ---------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------

def train():
    env = Go2Env(model_path=MODEL_PATH)
    eval_env = Go2Env(model_path=MODEL_PATH)

    # TensorBoard writer — view with: tensorboard --logdir models/runs
    from torch.utils.tensorboard import SummaryWriter
    run_name = datetime.now().strftime("%m%d_%H%M")
    writer = SummaryWriter(log_dir=str(OUT_DIR.parent / f"runs/go2_{run_name}"))

    progress_log = []

    def progress_cb(num_steps, metrics):
        ts = datetime.now().strftime("%H:%M:%S")
        rew        = float(metrics.get("eval/episode_reward", 0.0))
        rew_vel    = float(metrics.get("eval/episode_reward_lin_vel", 0.0))
        rew_sym    = float(metrics.get("eval/episode_reward_symmetry", 0.0))
        rew_air    = float(metrics.get("eval/episode_reward_airtime", 0.0))
        rew_psync  = float(metrics.get("eval/episode_reward_phase_sync", 0.0))
        ep_len     = float(metrics.get("eval/avg_episode_length", 0.0))
        progress_log.append((num_steps, rew))
        print(f"[{ts}] steps={num_steps:,}  reward={rew:.1f}"
              f"  vel={rew_vel:.2f}  sym={rew_sym:.2f}  air={rew_air:.2f}"
              f"  psync={rew_psync:.2f}  ep_len={ep_len:.0f}")
        writer.add_scalar("eval/reward_total",      rew,      num_steps)
        writer.add_scalar("eval/reward_lin_vel",    rew_vel,  num_steps)
        writer.add_scalar("eval/reward_symmetry",   rew_sym,  num_steps)
        writer.add_scalar("eval/reward_airtime",    rew_air,  num_steps)
        writer.add_scalar("eval/reward_phase_sync", rew_psync, num_steps)
        writer.add_scalar("eval/episode_length",    ep_len,   num_steps)
        writer.flush()

    print(f"\n=== Go2 PPO Training ===")
    print(f"  model:    {MODEL_PATH}")
    print(f"  envs:     {TRAIN_CFG['num_envs']}")
    print(f"  total_ts: {TRAIN_CFG['num_timesteps']:,}")
    print(f"  policy:   52 → {POLICY_HIDDEN} → 12\n")
    print(f"  run:      8 (kernel=-16, moderate gait, strong stability)\n")

    t0 = time.time()
    make_inference_fn, params, metrics = ppo.train(
        environment=env,
        eval_env=eval_env,
        network_factory=make_networks,
        progress_fn=progress_cb,
        **TRAIN_CFG,
    )
    elapsed = time.time() - t0
    writer.close()
    print(f"\nTraining done in {elapsed/60:.1f} min")
    print(f"Final eval reward: {float(metrics['eval/episode_reward']):.2f}")

    # Save Brax checkpoint
    ckpt_path = OUT_DIR / "go2_checkpoint.pkl"
    with open(ckpt_path, "wb") as f:
        pickle.dump(
            {
                "params": jax.tree_util.tree_map(lambda x: x.tolist(), params),
                "hidden_sizes": POLICY_HIDDEN,
                "obs_dim": 48,
                "action_dim": 12,
            },
            f,
        )
    print(f"Checkpoint saved → {ckpt_path}")

    # Immediately export to TorchScript
    export_torchscript(params, make_inference_fn, env)


# ---------------------------------------------------------------------------
# Export: JAX params → TorchScript actor
# ---------------------------------------------------------------------------

def export_torchscript(params, make_inference_fn, env):
    """Extract policy weights and export deterministic actor as TorchScript."""
    import numpy as np
    import torch
    import torch.nn as nn

    print("\n=== Exporting to TorchScript ===")

    # Extract policy weights from Brax FrozenDict params
    # Params structure: (normalizer_params, policy_params) when normalize_observations=True
    # or just policy_params when normalize_observations=False
    # In Brax PPO the training_state.params is (normalizer_params, (policy_params, value_params))
    # The make_inference_fn takes the full params tuple as returned by train()
    policy_weights = _extract_policy_weights(params)

    # Build PyTorch model with same architecture
    layers = []
    in_dim = policy_weights[0][0].shape[1]  # derive input dim from first layer weights
    for w, b in policy_weights[:-1]:
        out_dim = w.shape[0]
        linear = nn.Linear(in_dim, out_dim)
        linear.weight.data = torch.tensor(w, dtype=torch.float32)
        linear.bias.data = torch.tensor(b, dtype=torch.float32)
        layers += [linear, nn.ELU()]
        in_dim = out_dim

    # Last layer: take only mean outputs (first action_dim rows), discard log_std.
    # Then apply tanh to match tanh_normal distribution used during training.
    w_last, b_last = policy_weights[-1]
    action_dim = w_last.shape[0] // 2
    linear_out = nn.Linear(in_dim, action_dim)
    linear_out.weight.data = torch.tensor(w_last[:action_dim], dtype=torch.float32)
    linear_out.bias.data = torch.tensor(b_last[:action_dim], dtype=torch.float32)
    layers.append(linear_out)
    layers.append(nn.Tanh())  # tanh_normal: action = tanh(mean)

    actor = nn.Sequential(*layers)
    actor.eval()

    # Verify with random input (use actual obs_dim from trained weights)
    obs_dim = policy_weights[0][0].shape[1]
    test_input = torch.zeros(1, obs_dim)
    with torch.no_grad():
        out = actor(test_input)
    assert out.shape == (1, 12), f"Expected (1,12), got {out.shape}"
    print(f"  forward check: input={test_input.shape} → output={out.shape} ✓")

    # Export as TorchScript
    scripted = torch.jit.script(actor)
    out_path = OUT_DIR / "actor_go2.pt"
    scripted.save(str(out_path))
    print(f"  TorchScript actor saved → {out_path}")

    # Quick load-back verification
    loaded = torch.jit.load(str(out_path))
    with torch.no_grad():
        out2 = loaded(test_input)
    assert torch.allclose(out, out2), "Load-back verification failed"
    print("  Load-back verification ✓")


def _extract_policy_weights(params):
    """Extract (weight, bias) pairs for each Dense layer of the policy network.

    Brax (GitHub main) PPO params after training:
        params = (normalizer_params, policy_params, value_params)   # 3-tuple

    policy_params: {'params': {'hidden_0': {'kernel': (in,out), 'bias': (out,)}, ...}}
    Flax kernel layout: (in, out) — transpose to (out, in) for PyTorch Linear.
    """
    import numpy as np

    # New Brax (GitHub main): 3-tuple (normalizer, policy, value)
    if len(params) == 3:
        policy_params = params[1]
    else:
        # Legacy 2-tuple (normalizer, (policy, value)) or (normalizer, policy)
        _, policy_and_value = params
        policy_params = policy_and_value[0] if isinstance(policy_and_value, tuple) else policy_and_value

    # Unwrap Flax FrozenDict / nested dict
    p = _to_dict(policy_params)
    if "params" in p:
        p = p["params"]

    # Try Dense_N naming first, then hidden_N
    weights = []
    for prefix in ("Dense_", "hidden_"):
        i = 0
        while f"{prefix}{i}" in p:
            layer = p[f"{prefix}{i}"]
            w = np.array(layer["kernel"]).T    # Flax (in, out) → PyTorch (out, in)
            b = np.array(layer["bias"])
            weights.append((w, b))
            i += 1
        if weights:
            break

    assert weights, f"Could not find Dense layers in params keys: {list(p.keys())}"
    print(f"  Extracted {len(weights)} layers:")
    for idx, (w, b) in enumerate(weights):
        print(f"    Layer {idx}: {w.shape[1]} → {w.shape[0]}")
    return weights


def _to_dict(x):
    """Recursively convert Flax FrozenDict to plain dict."""
    try:
        from flax.core import FrozenDict
        if isinstance(x, FrozenDict):
            return {k: _to_dict(v) for k, v in x.items()}
    except ImportError:
        pass
    if hasattr(x, "items"):
        return {k: _to_dict(v) for k, v in x.items()}
    return x


if __name__ == "__main__":
    train()
