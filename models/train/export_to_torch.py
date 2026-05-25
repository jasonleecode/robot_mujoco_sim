"""Export a saved Brax PPO checkpoint to TorchScript for C++ deployment.

Usage (from repo root, inside mjx_train conda env):
    python models/train/export_to_torch.py [checkpoint.pkl] [output.pt]

Defaults:
    checkpoint : models/legged_gym/go2_checkpoint.pkl
    output     : models/legged_gym/actor_go2.pt

The output TorchScript model accepts a float32 tensor of shape (1, 48) and
returns a float32 tensor of shape (1, 12) — identical interface to the
existing actor_300.pt used by the C++ TorchBackend.
"""
import argparse
import pickle
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn


def load_checkpoint(path: str) -> dict:
    with open(path, "rb") as f:
        ckpt = pickle.load(f)
    return ckpt


def extract_weights(params, hidden_sizes=(256, 128)):
    """Extract Dense layer weights from Brax FrozenDict params.

    Returns list of (weight, bias) as numpy arrays in PyTorch convention (out, in).
    """
    def to_plain(x):
        try:
            from flax.core import FrozenDict
            if isinstance(x, FrozenDict):
                return {k: to_plain(v) for k, v in x.items()}
        except ImportError:
            pass
        if hasattr(x, "items"):
            return {k: to_plain(v) for k, v in x.items()}
        return x

    # Brax PPO params structure: (normalizer_params, (policy_params, value_params))
    _, policy_and_value = params
    if isinstance(policy_and_value, tuple):
        policy_params = policy_and_value[0]
    else:
        policy_params = policy_and_value

    p = to_plain(policy_params)
    if "params" in p:
        p = p["params"]

    # Collect Dense_0, Dense_1, ... in order
    weights = []
    i = 0
    while f"Dense_{i}" in p:
        layer = p[f"Dense_{i}"]
        # Flax stores kernel as (in_features, out_features); PyTorch expects (out, in)
        w = np.array(layer["kernel"]).T
        b = np.array(layer["bias"])
        weights.append((w, b))
        i += 1

    if not weights:
        raise ValueError(
            f"No Dense_N layers found in params. Keys: {list(p.keys())}"
        )
    return weights


def build_actor(weights, action_dim=12) -> nn.Module:
    """Build PyTorch actor that outputs only the mean (first action_dim outputs).

    Brax PPO policy outputs 2*action_dim (mean + log_std). We keep only mean.
    """
    layers = []
    in_dim = weights[0][0].shape[1]

    for i, (w, b) in enumerate(weights[:-1]):
        out_dim = w.shape[0]
        linear = nn.Linear(in_dim, out_dim)
        linear.weight.data = torch.tensor(w, dtype=torch.float32)
        linear.bias.data = torch.tensor(b, dtype=torch.float32)
        layers += [linear, nn.ELU()]
        in_dim = out_dim

    # Final layer: take only first action_dim rows (the mean outputs)
    w_last, b_last = weights[-1]
    linear_out = nn.Linear(in_dim, action_dim)
    linear_out.weight.data = torch.tensor(
        w_last[:action_dim], dtype=torch.float32
    )
    linear_out.bias.data = torch.tensor(
        b_last[:action_dim], dtype=torch.float32
    )
    layers.append(linear_out)

    return nn.Sequential(*layers)


def export(ckpt_path: str, out_path: str, action_dim: int = 12) -> None:
    print(f"Loading checkpoint: {ckpt_path}")
    ckpt = load_checkpoint(ckpt_path)

    # ckpt may store weights as plain lists (saved via .tolist()) or as JAX arrays
    params = ckpt["params"]
    hidden_sizes = ckpt.get("hidden_sizes", (256, 128))
    obs_dim = ckpt.get("obs_dim", 48)

    print(f"  obs_dim={obs_dim}  hidden_sizes={hidden_sizes}  action_dim={action_dim}")

    weights = extract_weights(params, hidden_sizes)

    print("  Layer shapes:")
    for i, (w, b) in enumerate(weights):
        print(f"    Dense_{i}: {w.shape[1]} → {w.shape[0]}")

    actor = build_actor(weights, action_dim=action_dim)
    actor.eval()

    # Sanity check forward pass
    test_in = torch.zeros(1, obs_dim)
    with torch.no_grad():
        test_out = actor(test_in)
    assert test_out.shape == (1, action_dim), (
        f"Expected output shape (1, {action_dim}), got {test_out.shape}"
    )
    print(f"  Forward check: {test_in.shape} → {test_out.shape} ✓")

    # TorchScript export
    scripted = torch.jit.script(actor)
    scripted.save(out_path)
    print(f"  Saved TorchScript actor → {out_path}")

    # Verify load-back
    reloaded = torch.jit.load(out_path)
    with torch.no_grad():
        verify_out = reloaded(test_in)
    assert torch.allclose(test_out, verify_out), "Load-back mismatch!"
    print("  Load-back verification ✓")


def main():
    parser = argparse.ArgumentParser(description="Export Brax checkpoint → TorchScript")
    parser.add_argument(
        "checkpoint",
        nargs="?",
        default="models/legged_gym/go2_checkpoint.pkl",
        help="Path to go2_checkpoint.pkl",
    )
    parser.add_argument(
        "output",
        nargs="?",
        default="models/legged_gym/actor_go2.pt",
        help="Output TorchScript .pt path",
    )
    parser.add_argument("--action-dim", type=int, default=12)
    args = parser.parse_args()

    ckpt = Path(args.checkpoint)
    if not ckpt.exists():
        print(f"Error: checkpoint not found: {ckpt}", file=sys.stderr)
        sys.exit(1)

    export(str(ckpt), args.output, action_dim=args.action_dim)


if __name__ == "__main__":
    main()
