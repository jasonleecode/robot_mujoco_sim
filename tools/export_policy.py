#!/usr/bin/env python3
"""Export a legged_gym/Isaac Lab actor checkpoint to a TorchScript file.

Usage:
    python3 tools/export_policy.py models/legged_gym/model_300.pt

The script auto-detects the actor hidden-layer sizes from the state_dict
and writes an "actor_<N>.pt" TorchScript file beside the input checkpoint.

Architecture assumed: Linear -> ELU -> ... -> Linear (no final activation).
Input dim and output dim are inferred from the first and last linear layers.
"""
import sys
import os
import torch
import torch.nn as nn
from pathlib import Path


def build_actor(in_dim: int, hidden: list[int], out_dim: int) -> nn.Sequential:
    layers: list[nn.Module] = []
    prev = in_dim
    for h in hidden:
        layers.append(nn.Linear(prev, h))
        layers.append(nn.ELU())
        prev = h
    layers.append(nn.Linear(prev, out_dim))
    return nn.Sequential(*layers)


class ActorWrapper(nn.Module):
    def __init__(self, seq: nn.Sequential):
        super().__init__()
        self.actor = seq

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.actor(obs)


def export(ckpt_path: str) -> str:
    ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False)

    # Support both plain state_dict and dict with 'model_state_dict' key
    if isinstance(ckpt, dict) and "model_state_dict" in ckpt:
        sd = ckpt["model_state_dict"]
        iteration = ckpt.get("iter", "?")
    else:
        sd = ckpt
        iteration = "?"

    # Extract actor weights only
    actor_sd = {k[len("actor."):]: v for k, v in sd.items() if k.startswith("actor.")}
    if not actor_sd:
        raise ValueError("No 'actor.*' keys found in checkpoint. "
                         "Is this a legged_gym/Isaac Lab checkpoint?")

    # Infer layer sizes from weight shapes
    linear_layers = [(k, v) for k, v in actor_sd.items() if k.endswith(".weight")]
    hidden_sizes = [v.shape[0] for _, v in linear_layers[:-1]]
    in_dim  = linear_layers[0][1].shape[1]
    out_dim = linear_layers[-1][1].shape[0]

    print(f"  Architecture: {in_dim} -> {hidden_sizes} -> {out_dim}")
    print(f"  Training iter: {iteration}")

    seq = build_actor(in_dim, hidden_sizes, out_dim)
    seq.load_state_dict(actor_sd)
    model = ActorWrapper(seq)
    model.eval()

    # Sanity-check
    with torch.no_grad():
        dummy = torch.zeros(1, in_dim)
        out = model(dummy)
    assert out.shape == (1, out_dim), f"Unexpected output shape: {out.shape}"
    print(f"  Forward pass OK: {dummy.shape} -> {out.shape}")

    # Export
    out_path = Path(ckpt_path).with_name(
        "actor_" + Path(ckpt_path).stem.split("_")[-1] + ".pt"
    )
    scripted = torch.jit.script(model)
    scripted.save(str(out_path))

    # Verify reload
    reloaded = torch.jit.load(str(out_path))
    with torch.no_grad():
        out2 = reloaded(dummy)
    assert torch.allclose(out, out2), "Reload mismatch!"
    print(f"  Saved: {out_path}")
    return str(out_path)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    path = sys.argv[1]
    if not os.path.exists(path):
        print(f"Error: file not found: {path}")
        sys.exit(1)
    print(f"Exporting: {path}")
    result = export(path)
    print(f"Done: {result}")
