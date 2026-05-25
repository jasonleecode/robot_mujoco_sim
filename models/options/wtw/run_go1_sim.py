"""
Run WTW (Walk These Ways) policy on Unitree Go1 in MuJoCo.

Architecture:
  adaptation_module_latest.jit : history(2100=30×70) → latent(2)
  body_latest.jit              : history(2100) + latent(2) = 2102 → actions(12)

Observation (70-dim per frame):
  [0:3]   lin_vel_body  × 2.0
  [3:6]   ang_vel_body  × 0.25
  [6:9]   projected_gravity
  [9:12]  commands [vx×2, vy×2, wz×0.25]
  [12:24] dof_pos - nominal
  [24:36] dof_vel × 0.05
  [36:48] last_action
  [48:56] gait_clock [sin×4, cos×4] per foot (FL,FR,RL,RR)
  [56:65] gait_params [freq, swing_h, body_h, width, length, aux×4]
  [65:69] foot_contact [FL,FR,RL,RR] 0/1
  [69:70] body_height
"""
import math
import os
import tempfile
import time
import numpy as np
import mujoco
import mujoco.viewer
import torch
from pathlib import Path

# ── paths ──────────────────────────────────────────────────────────────────
SCENE_XML  = str(Path(__file__).resolve().parents[3] / "robot/unitree_go1/scene.xml")

# ── mode flags (mutually exclusive) ────────────────────────────────────────
USE_BALL    = False   # put robot on a rolling sphere
BALL_RADIUS = 0.65

USE_TERRAIN = True    # add gravel / walls / stairs ahead of the robot
ACTOR_JIT  = "body_latest.jit"
ADAPT_JIT  = "adaptation_module_latest.jit"

# ── Go1 constants ──────────────────────────────────────────────────────────
# MuJoCo joint order (go1.xml): FR, FL, RR, RL  (each: hip, thigh, calf)
# WTW policy order:             FL, FR, RL, RR
# Permutation go1→WTW and WTW→go1 (symmetric): [3,4,5, 0,1,2, 9,10,11, 6,7,8]
GO1_TO_WTW = np.array([3,4,5, 0,1,2, 9,10,11, 6,7,8])

# WTW nominal joint angles in WTW order (FL, FR, RL, RR):
# FL/RL hip: +0.1  FR/RR hip: -0.1  front thigh: 0.8  rear thigh: 1.0  calf: -1.5
NOMINAL_WTW = np.array([
     0.1, 0.8, -1.5,   # FL
    -0.1, 0.8, -1.5,   # FR
     0.1, 1.0, -1.5,   # RL
    -0.1, 1.0, -1.5,   # RR
], dtype=np.float32)

ACTION_SCALE = 0.25

# ── WTW gait parameters (trot defaults) ───────────────────────────────────
CMD_VX        = 0.0    # m/s forward  (0 for ball balancing)
CMD_VY        = 0.0
CMD_WZ        = 0.0
CLOCK_HZ      = 3.0    # gait clock advancement rate (Hz) — must be >0 for clock to move
GAIT_FREQ     = 3.0    # gait_params[0]: must match CLOCK_HZ so obs is consistent
SWING_HEIGHT  = 0.05   # foot lift height (m)
BODY_HEIGHT   = 0.3    # desired body height (m)
STANCE_WIDTH  = 0.3    # stance width (m)
STANCE_LENGTH = 0.35   # stance length (m)
AUX_PARAMS    = [0.0, 0.0, 0.0, 0.0]

# Trot phase offsets per foot (FL, FR, RL, RR)
# FL+RR swing together, FR+RL swing together
PHASE_OFFSETS = np.array([0.0, math.pi, math.pi, 0.0], dtype=np.float32)

CTRL_DT    = 0.02   # 50 Hz policy
HISTORY_LEN = 30
OBS_DIM     = 70


def quat_rotate_inverse(q, v):
    """Rotate world-frame vector v into body frame. q = [w, x, y, z]."""
    w, x, y, z = q
    vx, vy, vz = v
    rx = (1-2*y*y-2*z*z)*vx + (2*x*y+2*w*z)*vy   + (2*x*z-2*w*y)*vz
    ry = (2*x*y-2*w*z)*vx   + (1-2*x*x-2*z*z)*vy + (2*y*z+2*w*x)*vz
    rz = (2*x*z+2*w*y)*vx   + (2*y*z-2*w*x)*vy   + (1-2*x*x-2*y*y)*vz
    return np.array([rx, ry, rz], dtype=np.float32)


def get_foot_geom_ids(m):
    """Return geom IDs for feet in WTW order [FL, FR, RL, RR]."""
    ids = {}
    for gid in range(m.ngeom):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, gid)
        if name in ("FL", "FR", "RL", "RR"):
            ids[name] = gid
    return [ids["FL"], ids["FR"], ids["RL"], ids["RR"]]


def build_obs(d, m, last_action, phase, gait_params, foot_geom_ids=None):
    q = d.qpos[3:7]   # [w, x, y, z]
    v_world = d.qvel[0:3]
    w_world = d.qvel[3:6]

    lin_vel_b  = quat_rotate_inverse(q, v_world)
    ang_vel_b  = quat_rotate_inverse(q, w_world)
    proj_grav  = quat_rotate_inverse(q, np.array([0., 0., -1.]))

    # reorder from go1.xml order (FR,FL,RR,RL) → WTW order (FL,FR,RL,RR)
    dof_pos = d.qpos[7:19].astype(np.float32)[GO1_TO_WTW] - NOMINAL_WTW
    dof_vel = d.qvel[6:18].astype(np.float32)[GO1_TO_WTW]

    cmd_obs = np.array([CMD_VX * 2.0, CMD_VY * 2.0, CMD_WZ * 0.25], dtype=np.float32)

    phi = phase + PHASE_OFFSETS
    clock = np.concatenate([np.sin(phi), np.cos(phi)])  # [sin×4, cos×4]

    body_height = np.array([d.qpos[2]], dtype=np.float32)

    # foot contact via MuJoCo contact list (works on flat ground AND ball surface)
    foot_contact = np.zeros(4, dtype=np.float32)
    if foot_geom_ids is not None:
        in_contact = set()
        for ci in range(d.ncon):
            g1 = d.contact[ci].geom1
            g2 = d.contact[ci].geom2
            for fi, gid in enumerate(foot_geom_ids):
                if g1 == gid or g2 == gid:
                    in_contact.add(fi)
        for fi in in_contact:
            foot_contact[fi] = 1.0
    elif m.nsite >= 6:
        for i, s in enumerate([3, 2, 5, 4]):
            foot_contact[i] = 1.0 if d.site_xpos[s, 2] < 0.08 else 0.0

    obs = np.concatenate([
        lin_vel_b  * 2.0,          # [0:3]
        ang_vel_b  * 0.25,         # [3:6]
        proj_grav,                 # [6:9]
        cmd_obs,                   # [9:12]
        dof_pos,                   # [12:24]
        dof_vel    * 0.05,         # [24:36]
        last_action,               # [36:48]
        clock,                     # [48:56]
        gait_params,               # [56:65]
        foot_contact,              # [65:69]
        body_height,               # [69:70]
    ]).astype(np.float32)

    return np.clip(obs, -5.0, 5.0)


def make_ball_scene(scene_xml_path: str, ball_radius: float) -> str:
    """Inject a free-rolling sphere into the scene and return a temp XML path."""
    scene_dir = os.path.dirname(scene_xml_path)
    with open(scene_xml_path) as f:
        xml = f.read()

    ball_xml = f"""
    <body name="ball" pos="0 0 {ball_radius:.4f}">
      <freejoint name="ball_joint"/>
      <geom type="sphere" size="{ball_radius:.4f}"
            mass="5.0" friction="1.5 0.01 0.0001"
            condim="6" solref="0.01 1" solimp="0.95 0.99 0.001"
            rgba="1.0 0.5 0.1 0.95"/>
    </body>"""

    # Insert just before </worldbody>
    xml = xml.replace("</worldbody>", ball_xml + "\n  </worldbody>")

    # Write next to scene.xml so relative <include> still works
    tmp_path = os.path.join(scene_dir, "_scene_ball_tmp.xml")
    with open(tmp_path, "w") as f:
        f.write(xml)
    return tmp_path


def make_terrain_scene(scene_xml_path: str) -> tuple[str, np.ndarray]:
    """
    Build a corridor scene with:
      - Flat start zone            x = 0  … 1.5 m
      - Gravel (bumpy heightfield) x = 1.5 … 4.5 m
      - Stairs (5 steps × 8 cm)   x = 5.0 … 6.75 m  →  40 cm total rise
      - Elevated platform          x = 6.75 … 9.0 m
      - Corridor walls             y = ±1.3 m  along full length

    Returns (tmp_xml_path, hfield_heights_float32)
    """
    scene_dir = os.path.dirname(scene_xml_path)
    with open(scene_xml_path) as f:
        xml = f.read()

    # ── layout constants ──────────────────────────────────────────────────
    corridor_w  = 2.6    # full width between walls (m)
    wall_h      = 0.6    # wall height (m)
    total_len   = 9.0

    # gravel heightfield: covers x=[1.5, 4.5], y=[−1.3, 1.3]
    gravel_x0, gravel_x1 = 1.5, 4.5
    gravel_hx = (gravel_x1 - gravel_x0) / 2    # half-size x = 1.5
    gravel_hy = corridor_w / 2                  # half-size y = 1.3
    gravel_nx, gravel_ny = 80, 50               # resolution (rows × cols)
    gravel_max_h = 0.025                        # 2.5 cm peak bump

    # stairs
    n_steps    = 5
    step_d     = 0.35     # depth of each tread (m)
    step_h     = 0.08     # rise per step (m)
    stair_x0   = 5.0
    plat_len   = 2.25     # platform length after stairs
    plat_h     = n_steps * step_h   # 0.40 m

    # ── heightfield asset ─────────────────────────────────────────────────
    gravel_asset = (
        f'<hfield name="gravel" nrow="{gravel_nx}" ncol="{gravel_ny}" '
        f'size="{gravel_hx:.3f} {gravel_hy:.3f} {gravel_max_h:.4f} 0.01"/>'
    )

    # ── stairs geoms ──────────────────────────────────────────────────────
    stair_geoms = []
    hw = corridor_w / 2
    for i in range(n_steps):
        sx    = stair_x0 + i * step_d + step_d / 2
        sh    = (i + 1) * step_h
        stair_geoms.append(
            f'<geom name="stair{i}" type="box" '
            f'size="{step_d/2:.4f} {hw:.3f} {sh/2:.4f}" '
            f'pos="{sx:.4f} 0 {sh/2:.4f}" '
            f'rgba="0.55 0.60 0.80 1"/>'
        )
    # platform
    plat_x = stair_x0 + n_steps * step_d + plat_len / 2
    stair_geoms.append(
        f'<geom name="platform" type="box" '
        f'size="{plat_len/2:.4f} {hw:.3f} {plat_h/2:.4f}" '
        f'pos="{plat_x:.4f} 0 {plat_h/2:.4f}" '
        f'rgba="0.55 0.60 0.80 1"/>'
    )

    # ── walls ─────────────────────────────────────────────────────────────
    wy    = corridor_w / 2 + 0.15    # wall centre y
    wlen  = total_len / 2
    wx    = total_len / 2
    wall_geoms = (
        f'<geom name="wall_l" type="box" '
        f'size="{wlen:.2f} 0.15 {wall_h/2:.3f}" '
        f'pos="{wx:.2f}  {wy:.3f} {wall_h/2:.3f}" '
        f'rgba="0.85 0.82 0.74 1"/>\n'
        f'    <geom name="wall_r" type="box" '
        f'size="{wlen:.2f} 0.15 {wall_h/2:.3f}" '
        f'pos="{wx:.2f} {-wy:.3f} {wall_h/2:.3f}" '
        f'rgba="0.85 0.82 0.74 1"/>'
    )

    # ── gravel geom (references hfield) ──────────────────────────────────
    gravel_cx = (gravel_x0 + gravel_x1) / 2   # = 3.0
    gravel_geom = (
        f'<geom name="gravel_geom" type="hfield" hfield="gravel" '
        f'pos="{gravel_cx:.2f} 0 0" '
        f'friction="0.7 0.005 0.0001" rgba="0.72 0.58 0.40 1"/>'
    )

    # ── inject into XML ───────────────────────────────────────────────────
    xml = xml.replace("</asset>", f"    {gravel_asset}\n  </asset>")
    terrain_block = (
        f"\n    {gravel_geom}\n"
        f"    {wall_geoms}\n"
        + "".join(f"    {g}\n" for g in stair_geoms)
    )
    xml = xml.replace("</worldbody>", terrain_block + "  </worldbody>")

    tmp_path = os.path.join(scene_dir, "_scene_terrain_tmp.xml")
    with open(tmp_path, "w") as f:
        f.write(xml)

    # ── pre-compute heightfield data (returned; applied after model load) ─
    rng = np.random.default_rng(7)
    x  = np.linspace(0, 2 * np.pi * 6, gravel_nx)
    y  = np.linspace(0, 2 * np.pi * 4, gravel_ny)
    X, Y = np.meshgrid(y, x)
    h  = (0.5
          + 0.25 * np.sin(X) * np.cos(Y)
          + 0.15 * np.sin(2.3 * X + 0.7) * np.cos(1.7 * Y + 1.1)
          + 0.10 * rng.uniform(-1, 1, (gravel_nx, gravel_ny)))
    h  = np.clip(h, 0.0, 1.0).astype(np.float32).flatten()
    return tmp_path, h


def main():
    print("Loading models...")
    actor   = torch.jit.load(ACTOR_JIT,  map_location="cpu").eval()
    adapter = torch.jit.load(ADAPT_JIT,  map_location="cpu").eval()
    print("  body_latest.jit       ✓")
    print("  adaptation_module.jit ✓")

    print("Loading MuJoCo scene...")
    hfield_data = None
    if USE_BALL:
        scene_path = make_ball_scene(SCENE_XML, BALL_RADIUS)
        print(f"  Ball mode: radius={BALL_RADIUS} m")
    elif USE_TERRAIN:
        scene_path, hfield_data = make_terrain_scene(SCENE_XML)
        print("  Terrain mode: gravel + walls + stairs")
    else:
        scene_path = SCENE_XML

    m = mujoco.MjModel.from_xml_path(scene_path)
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)

    # Apply gravel heightfield heights
    if hfield_data is not None:
        hid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_HFIELD, "gravel")
        if hid >= 0:
            adr = m.hfield_adr[hid]
            n   = m.hfield_nrow[hid] * m.hfield_ncol[hid]
            m.hfield_data[adr:adr + n] = hfield_data

    if USE_BALL:
        trunk_z = BALL_RADIUS * 2 + 0.31
        d.qpos[2] = trunk_z
    else:
        d.qpos[2] = 0.33         # WTW Go1 stand height
    d.qpos[3] = 1.0          # quaternion w=1 (upright)
    # set joints to WTW nominal in go1.xml order (FR,FL,RR,RL)
    nominal_go1 = NOMINAL_WTW[GO1_TO_WTW]
    d.qpos[7:19] = nominal_go1
    d.ctrl[:] = nominal_go1
    # Match WTW training PD gains: Kp=20, Kd=0.5  (go1.xml defaults are 100/2)
    m.actuator_gainprm[:, 0] = 20.0
    m.actuator_biasprm[:, 1] = -20.0
    m.dof_damping[6:18] = 0.5   # only robot joints, not ball freejoint

    # Precompute foot geom IDs for contact detection (works flat AND on ball)
    foot_geom_ids = get_foot_geom_ids(m)

    mujoco.mj_forward(m, d)
    print(f"  Go1 loaded, {m.nu} actuators, {m.nsite} sites  (Kp=20, Kd=0.5)")
    print(f"  Foot geom IDs [FL,FR,RL,RR]: {foot_geom_ids}")

    # gait params tensor (9-dim)
    gait_params = np.array(
        [GAIT_FREQ, SWING_HEIGHT, BODY_HEIGHT, STANCE_WIDTH, STANCE_LENGTH] + AUX_PARAMS,
        dtype=np.float32,
    )

    # history buffer: ring of HISTORY_LEN obs vectors
    history = np.zeros((HISTORY_LEN, OBS_DIM), dtype=np.float32)

    last_action  = np.zeros(12, dtype=np.float32)
    last_ctrl_t  = -1.0
    phase        = 0.0
    step_count   = 0

    print(f"\nStarting simulation  (cmd_vx={CMD_VX} m/s, clock={CLOCK_HZ} Hz, gait_freq={GAIT_FREQ})")
    print("Close the viewer window to exit.\n")

    if USE_BALL:
        cam_lookat = [0, 0, BALL_RADIUS * 2 + 0.3]
        cam_dist, cam_elev = 2.5, -20
    elif USE_TERRAIN:
        cam_lookat = [3.0, 0, 0.3]   # look at the middle of the course
        cam_dist, cam_elev = 6.0, -15
    else:
        cam_lookat = [0, 0, 0.27]
        cam_dist, cam_elev = 2.5, -20

    with mujoco.viewer.launch_passive(m, d) as viewer:
        viewer.cam.lookat[:] = cam_lookat
        viewer.cam.distance  = cam_dist
        viewer.cam.elevation = cam_elev

        while viewer.is_running():
            t = d.time

            if t - last_ctrl_t >= CTRL_DT:
                # advance gait phase using CLOCK_HZ (independent of gait_params[0])
                phase = math.fmod(t * 2.0 * math.pi * CLOCK_HZ, 2.0 * math.pi)

                obs = build_obs(d, m, last_action, phase, gait_params, foot_geom_ids)

                # push new obs into history (shift left, append right)
                history = np.roll(history, -1, axis=0)
                history[-1] = obs

                hist_flat = torch.tensor(history.flatten()[None], dtype=torch.float32)

                with torch.no_grad():
                    latent  = adapter(hist_flat)            # (1, 2)
                    inp     = torch.cat([hist_flat, latent], dim=-1)  # (1, 2102)
                    actions = actor(inp).squeeze(0).numpy() # (12,)

                actions = np.clip(actions, -1.0, 1.0)
                last_action = actions.copy()  # store in WTW order for next obs

                # convert WTW order → go1.xml order for ctrl
                actions_go1 = actions[GO1_TO_WTW]
                nominal_go1 = NOMINAL_WTW[GO1_TO_WTW]
                ctrl = nominal_go1 + ACTION_SCALE * actions_go1
                d.ctrl[:] = ctrl

                last_ctrl_t = t
                step_count += 1

                if step_count % 50 == 0:
                    q = d.qpos[3:7]
                    zproj = 1.0 - 2.0*(q[1]**2 + q[2]**2)
                    ball_str = f"  ball_z={d.qpos[19+2]:.3f}" if USE_BALL else ""
                    print(f"[t={t:.1f}]  x={d.qpos[0]:.3f}  body_z={d.qpos[2]:.3f}"
                          f"  zproj={zproj:.3f}  action_norm={np.linalg.norm(actions):.3f}"
                          + ball_str)

            mujoco.mj_step(m, d)
            viewer.sync()


if __name__ == "__main__":
    import os
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    main()
