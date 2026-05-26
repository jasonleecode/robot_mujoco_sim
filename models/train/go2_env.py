"""Go2 locomotion environment for Brax/MJX training (Ubuntu 20.04 + RTX 3080).

Observation (52-dim, Isaac Lab standard + phase clock):
  [0:3]   lin_vel_body  × 2.0
  [3:6]   ang_vel_body  × 0.25
  [6:9]   projected_gravity
  [9:12]  command [vx, vy, wz] scaled
  [12:24] dof_pos - nominal
  [24:36] dof_vel × 0.05
  [36:48] last_action
  [48:52] gait phase clock [sin(φ), cos(φ), sin(φ+π), cos(φ+π)]

Action (12-dim): position offsets from nominal, scaled by action_scale=0.25.
The MJX model (scene_mjx.xml) has built-in PD: kp=50, kd=0.5.
"""
import os
from typing import Any, Dict, Tuple

import jax
import jax.numpy as jnp
import mujoco
from brax.envs.base import PipelineEnv, State
from mujoco import mjx

GAIT_PERIOD = 0.5   # trot cycle duration (s); phase advances 2π every GAIT_PERIOD s
CTRL_DT     = 0.02  # policy control timestep (s), must match __init__ ctrl_dt

# Joint order in Go2: FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf,
#                     RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf
NOMINAL_QPOS = jnp.array([
    0.0, 0.9, -1.8,  # FL
    0.0, 0.9, -1.8,  # FR
    0.0, 0.9, -1.8,  # RL
    0.0, 0.9, -1.8,  # RR
])

# Command sampling ranges [vx, vy, wz]
# Force forward-only to eliminate the stand-still local optimum during initial training
CMD_LIN_VEL_X = (0.3, 1.0)
CMD_LIN_VEL_Y = (-0.5, 0.5)
CMD_ANG_VEL_Z = (-1.0, 1.0)


def _quat_rotate_inverse(q: jax.Array, v: jax.Array) -> jax.Array:
    """Rotate vector v from world frame to body frame using quaternion q=(w,x,y,z)."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    vx, vy, vz = v[0], v[1], v[2]
    rx = (1 - 2*y*y - 2*z*z)*vx + (2*x*y + 2*w*z)*vy + (2*x*z - 2*w*y)*vz
    ry = (2*x*y - 2*w*z)*vx + (1 - 2*x*x - 2*z*z)*vy + (2*y*z + 2*w*x)*vz
    rz = (2*x*z + 2*w*y)*vx + (2*y*z - 2*w*x)*vy + (1 - 2*x*x - 2*y*y)*vz
    return jnp.array([rx, ry, rz])


class Go2Env(PipelineEnv):
    """Go2 locomotion environment compatible with Brax PPO.

    Uses scene_mjx.xml which has position-servo actuators (kp=50, kd=0.5).
    The ctrl input to pipeline_step is desired joint position (not torque).
    """

    def __init__(
        self,
        model_path: str = "robot/unitree_go2/scene_mjx.xml",
        action_scale: float = 0.25,
        ctrl_dt: float = 0.02,
        sim_dt: float = 0.004,
        episode_length: int = 1000,
        **kwargs: Any,
    ):
        mj_model = mujoco.MjModel.from_xml_path(model_path)
        mj_model.opt.timestep = sim_dt
        sys = mjx.put_model(mj_model)
        n_frames = int(ctrl_dt / sim_dt)  # = 5
        super().__init__(sys=sys, backend="mjx", n_frames=n_frames, **kwargs)

        self._action_scale = action_scale
        self._episode_length = episode_length

    @property
    def observation_size(self) -> int:
        return 52

    @property
    def action_size(self) -> int:
        return 12

    def reset(self, rng: jax.Array) -> State:
        rng, rng_qpos, rng_qvel, rng_cmd, rng_phase = jax.random.split(rng, 5)

        # Start from nominal pose with small random perturbation
        qpos = jnp.zeros(self.sys.nq)
        qpos = qpos.at[0:3].set(jnp.array([0.0, 0.0, 0.27]))
        qpos = qpos.at[3].set(1.0)  # w=1, identity quaternion
        qpos = qpos.at[7:19].set(NOMINAL_QPOS)
        qpos = qpos.at[7:19].add(
            jax.random.uniform(rng_qpos, (12,), minval=-0.1, maxval=0.1)
        )
        qvel = jax.random.uniform(rng_qvel, (self.sys.nv,), minval=-0.05, maxval=0.05)

        pipeline_state = self.pipeline_init(qpos, qvel)
        command = self._sample_command(rng_cmd)
        last_action = jnp.zeros(12)
        # Randomise initial phase so each env starts at a different point in the gait cycle
        phase = jax.random.uniform(rng_phase, (), minval=0.0, maxval=2.0 * jnp.pi)

        obs = self._get_obs(pipeline_state, last_action, command, phase)
        metrics: Dict[str, jax.Array] = {
            "reward_lin_vel": jnp.zeros(()),
            "reward_symmetry": jnp.zeros(()),
            "reward_airtime": jnp.zeros(()),
            "reward_phase_sync": jnp.zeros(()),
            "step": jnp.zeros(()),
        }
        info = {"last_action": last_action, "command": command, "step": jnp.zeros(()), "phase": phase}
        return State(pipeline_state, obs, jnp.zeros(()), jnp.zeros(()), metrics, info)

    def step(self, state: State, action: jax.Array) -> State:
        command = state.info["command"]
        step = state.info["step"]
        phase = state.info["phase"]
        phase_new = (phase + 2.0 * jnp.pi * CTRL_DT / GAIT_PERIOD) % (2.0 * jnp.pi)

        # Clip and apply action scale → desired joint position
        action = jnp.clip(action, -1.0, 1.0)
        q_target = action * self._action_scale + NOMINAL_QPOS

        pipeline_state = self.pipeline_step(state.pipeline_state, q_target)

        lin_vel_b = self._lin_vel_body(pipeline_state)
        ang_vel_b = self._ang_vel_body(pipeline_state)
        proj_grav = self._projected_gravity(pipeline_state)

        # Velocity tracking rewards (kernel=-16: standing still gives only 24% of max reward)
        r_lin_x = jnp.exp(-16.0 * jnp.square(lin_vel_b[0] - command[0]))
        r_lin_y = jnp.exp(-16.0 * jnp.square(lin_vel_b[1] - command[1]))
        r_ang_z = jnp.exp(-16.0 * jnp.square(ang_vel_b[2] - command[2]))
        r_vel = 2.5 * r_lin_x + 0.5 * r_lin_y + 0.5 * r_ang_z

        # Stability penalties — strengthened to prevent aggressive/unstable gaits
        r_roll_pitch = -0.5 * jnp.sum(jnp.square(ang_vel_b[:2]))
        r_height = -2.0 * jnp.square(pipeline_state.qpos[2] - 0.27)
        r_action = -0.02 * jnp.sum(jnp.square(action))
        r_action_diff = -0.05 * jnp.sum(jnp.square(action - state.info["last_action"]))
        r_gravity = -2.0 * jnp.sum(jnp.square(proj_grav[:2]))
        r_lin_z = -2.0 * jnp.square(pipeline_state.qvel[2])
        r_dof_vel = -0.001 * jnp.sum(jnp.square(pipeline_state.qvel[6:18]))

        # Structural symmetry for trot gait.
        # Action order: [FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf,
        #                RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf]
        #
        # 1. Hip abduction: always anti-symmetric L/R (holds at every timestep)
        r_symmetry = -0.05 * (jnp.square(action[0] + action[3])   # FL_hip + FR_hip = 0
                             + jnp.square(action[6] + action[9]))  # RL_hip + RR_hip = 0

        # 2. Diagonal trot pairing: FL+RR one diagonal, FR+RL the other.
        #    Thigh and calf within each diagonal pair move in sync.
        #    (Replaces instantaneous L/R thigh symmetry which contradicts trot anti-phase.)
        r_diagonal = -0.1 * (
            jnp.square(action[1] - action[10])   # FL_thigh = RR_thigh
            + jnp.square(action[2] - action[11])  # FL_calf  = RR_calf
            + jnp.square(action[4] - action[7])   # FR_thigh = RL_thigh
            + jnp.square(action[5] - action[8])   # FR_calf  = RL_calf
        )

        # Foot clearance reward: reward each foot for lifting during its designated swing phase.
        # r_airtime removed (exploitable: policy could collect it without real locomotion).
        # r_foot_clearance is phase-synchronized so harder to exploit — only rewards
        # lifting the right feet at the right phase, not random hopping.
        # Foot order: FL(0), FR(1), RL(2), RR(3); trot diagonals: FL+RR share phase, FR+RL share phase+π
        foot_z = pipeline_state.site_xpos[jnp.array([1, 2, 3, 4]), 2]
        moving = jnp.abs(command[0]) + jnp.abs(command[1]) + jnp.abs(command[2]) > 0.3
        foot_phase_offsets = jnp.array([0.0, jnp.pi, jnp.pi, 0.0])
        foot_phases = phase + foot_phase_offsets
        swing_weight = jnp.maximum(0.0, jnp.sin(foot_phases))  # 0→1, peaks at mid-swing
        foot_z_clipped = jnp.clip(foot_z, 0.0, 0.10)           # clip to reasonable clearance
        r_airtime = 0.0
        r_foot_clearance = jnp.where(
            moving,
            0.5 * jnp.sum(swing_weight * foot_z_clipped),
            0.0,
        )

        reward = (r_vel + r_roll_pitch + r_height + r_action + r_action_diff
                  + r_gravity + r_lin_z + r_dof_vel + r_symmetry + r_diagonal
                  + r_airtime + r_foot_clearance)

        # Clip reward to prevent extreme negatives from dominating critic
        reward = jnp.clip(reward, -10.0, 10.0)

        # Termination: fell over or height too low
        height = pipeline_state.qpos[2]
        up = -proj_grav[2]  # 1.0 when upright, <0 when flipped
        done = jnp.where(height < 0.18, 1.0, 0.0)
        done = jnp.where(up < 0.85, 1.0, done)  # terminate at ~32° tilt (was 60°)
        done = jnp.where(step >= self._episode_length - 1, 1.0, done)
        # NaN guard: physics divergence → terminate and zero reward
        any_nan = jnp.any(jnp.isnan(pipeline_state.qpos)) | jnp.any(jnp.isnan(pipeline_state.qvel))
        reward = jnp.where(any_nan, 0.0, reward)
        done = jnp.where(any_nan, 1.0, done)

        obs = self._get_obs(pipeline_state, action, command, phase_new)
        obs = jnp.nan_to_num(obs, nan=0.0, posinf=5.0, neginf=-5.0)
        # Update metrics in-place to preserve any keys added by Brax wrappers
        # (e.g. 'reward' added by EpisodeWrapper) — avoids pytree structure mismatch.
        metrics = dict(state.metrics)
        metrics.update({
            "reward_lin_vel": jnp.nan_to_num(r_vel, nan=0.0),
            "reward_symmetry": r_symmetry,
            "reward_airtime": r_airtime,
            "reward_phase_sync": r_foot_clearance,
            "step": step,
        })
        info = state.info.copy()
        info.update({"last_action": action, "command": command, "step": step + 1, "phase": phase_new})

        return state.replace(
            pipeline_state=pipeline_state,
            obs=obs,
            reward=reward,
            done=done,
            metrics=metrics,
            info=info,
        )

    # -------------------------------------------------------------------------
    # Observation
    # -------------------------------------------------------------------------

    def _get_obs(
        self,
        pipeline_state: mjx.Data,
        last_action: jax.Array,
        command: jax.Array,
        phase: jax.Array,
    ) -> jax.Array:
        lin_vel_b = self._lin_vel_body(pipeline_state)
        ang_vel_b = self._ang_vel_body(pipeline_state)
        proj_grav = self._projected_gravity(pipeline_state)

        dof_pos = pipeline_state.qpos[7:19] - NOMINAL_QPOS
        dof_vel = pipeline_state.qvel[6:18]

        # Commands with Isaac Lab scales: [vx×2, vy×2, wz×0.25]
        cmd_obs = command * jnp.array([2.0, 2.0, 0.25])

        # Gait phase clock: two unit-circle signals, 180° apart for trot diagonals
        phi_opp = phase + jnp.pi
        phase_obs = jnp.array([
            jnp.sin(phase), jnp.cos(phase),
            jnp.sin(phi_opp), jnp.cos(phi_opp),
        ])

        obs = jnp.concatenate([
            lin_vel_b * 2.0,    # 3
            ang_vel_b * 0.25,   # 3
            proj_grav,          # 3
            cmd_obs,            # 3
            dof_pos,            # 12
            dof_vel * 0.05,     # 12
            last_action,        # 12
            phase_obs,          # 4
        ])
        return jnp.clip(obs, -5.0, 5.0)

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------

    def _base_quat(self, pipeline_state: mjx.Data) -> jax.Array:
        # xquat[1] = base body (index 0 = world)
        return pipeline_state.xquat[1]

    def _lin_vel_body(self, pipeline_state: mjx.Data) -> jax.Array:
        q = self._base_quat(pipeline_state)
        v = pipeline_state.qvel[0:3]  # world-frame linear vel
        return _quat_rotate_inverse(q, v)

    def _ang_vel_body(self, pipeline_state: mjx.Data) -> jax.Array:
        q = self._base_quat(pipeline_state)
        w = pipeline_state.qvel[3:6]  # world-frame angular vel
        return _quat_rotate_inverse(q, w)

    def _projected_gravity(self, pipeline_state: mjx.Data) -> jax.Array:
        q = self._base_quat(pipeline_state)
        return _quat_rotate_inverse(q, jnp.array([0.0, 0.0, -1.0]))

    def _sample_command(self, rng: jax.Array) -> jax.Array:
        rng_vx, rng_vy, rng_wz = jax.random.split(rng, 3)
        vx = jax.random.uniform(rng_vx, minval=CMD_LIN_VEL_X[0], maxval=CMD_LIN_VEL_X[1])
        vy = jax.random.uniform(rng_vy, minval=CMD_LIN_VEL_Y[0], maxval=CMD_LIN_VEL_Y[1])
        wz = jax.random.uniform(rng_wz, minval=CMD_ANG_VEL_Z[0], maxval=CMD_ANG_VEL_Z[1])
        return jnp.array([vx, vy, wz])
