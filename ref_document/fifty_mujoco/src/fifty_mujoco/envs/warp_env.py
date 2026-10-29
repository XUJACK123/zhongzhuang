"""GPU-vectorised environment: MuJoCo Warp physics + torch control/obs/reward.

The CPU env (:class:`fifty_mujoco.envs.fifty_env.FiftyVecEnv`) runs a Python
loop per environment, and a profile of it shows that ``mj_step`` does not even
appear in the hot list: rewards (47 %), the PD/spring write (29 %) and the
privileged observation (13 %) dominate.  Moving only the physics to the GPU
would therefore buy almost nothing.  This module moves *everything* that runs
per environment into batched torch code on the device, so the numbers below
are an end-to-end speed-up rather than a physics-only one.

Contract is identical to the CPU env (doc sections 5.2 - 5.6):

* obs 25 (ang_vel*0.25, projected_gravity, command*[3,0.25,5],
  (q_legs-default), qd*0.05, last_action)
* history 5 x 25, oldest first
* privileged 141 + 4 per gas spring
* rewards clipped per term and scaled by ``policy_dt`` exactly once

Deliberate scope limits (documented, not hidden):

* no action delay (the CPU env implements it; the GPU env asserts it is 0)
* no domain randomisation yet
* ``mass_bias``/``com_bias``/``friction``/``restitution`` privileged channels
  are the neutral constants the CPU env also uses
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
import torch

from ..config import Config
from ..contracts import names
from ..control.gas_spring import reference_force
from ..control.mixed_pd import PdGains
from ..control.spawn import load_spawn_spec, resolve_spawn_height
from ..envs.cpu_env import FiftyCpuEnv
from ..mdp import rewards as reward_lib
from ..mdp import terminations as term_lib

LEG, WHEEL = list(names.POLICY_LEG_INDICES), list(names.POLICY_WHEEL_INDICES)

#: Spring feature scaling, identical to ``mdp.observations.spring_features``.
SPRING_LENGTH_SCALE = 5.0
SPRING_FORCE_SCALE = 1.0 / 300.0


@dataclass
class WarpEnvOptions:
    num_envs: int = 2048
    device: str = "cuda:0"
    seed: int = 0
    episode_length_s: float = 20.0
    action_delay_steps: int = 0
    nconmax_per_env: int = 24
    njmax_per_env: int = 48
    command_ranges: tuple[tuple[float, float], ...] = (
        (-0.6, 0.6),
        (-0.4, 0.4),
        (-0.03, 0.03),
    )
    gains: PdGains | None = None
    gravity: float = 9.81
    randomize_initial: bool = False
    #: When set, every env gets this exact command (parity tests, evaluation).
    fixed_command: tuple[float, float, float] | None = None


class WarpVecEnv:
    """Batched environment whose whole control stack runs as torch kernels."""

    def __init__(self, config: Config, options: WarpEnvOptions | None = None):
        import mujoco
        import mujoco_warp as mjw
        import warp as warp

        from ..paths import assets_dir

        self.config = config
        self.options = options or WarpEnvOptions()
        if self.options.action_delay_steps != 0:
            raise NotImplementedError("the GPU env does not implement action delay")
        self.warp = warp
        self.mjw = mjw
        self.device = torch.device(self.options.device)
        self.num_envs = int(self.options.num_envs)
        self.generator = torch.Generator(device=self.device).manual_seed(self.options.seed)

        xml = assets_dir() / "mjcf" / "fifty_train.xml"
        self.cpu = FiftyCpuEnv(xml, config)  # name maps, spring list, spawn audit
        self.model = self.cpu.model
        self.spawn_z = resolve_spawn_height(self.cpu, load_spawn_spec(config))[0]
        self.nominal_height = self.spawn_z - float(config.spawn["spawn"]["wheel_clearance_m"])

        # ------------------------------------------------------------- constants
        self.default_angles = np.array(
            [config.default_angles[name] for name in names.MODEL_JOINT_ORDER], dtype=np.float64
        )
        self.gains = self.options.gains or PdGains(
            leg_kp=float(config.robot["control"]["leg_kp"]),
            leg_kd=float(config.robot["control"]["leg_kd"]),
            wheel_kp=float(config.robot["control"]["wheel_kp"]),
            wheel_kd=float(config.robot["control"]["wheel_kd"]),
            torque_limit_nm=config.effort_limit("rf0_Joint"),
            wheel_torque_limit_nm=config.effort_limit("r_wheel_Joint"),
            leg_position_scale=float(config.robot["control"]["leg_position_scale"]),
            wheel_velocity_scale=float(config.robot["control"]["wheel_velocity_scale"]),
        )
        self.springs = self.cpu.springs
        self.spring_count = len(self.springs)
        self.num_obs = names.ACTOR_OBS_DIM
        self.num_privileged_obs, self.critic_input_dim = names.privileged_dim(self.spring_count)
        self.num_actions = names.NUM_ACTIONS
        self.history_length = names.HISTORY_LENGTH
        self.decimation = int(round(
            float(config.robot["control"]["policy_dt_s"])
            / float(config.robot["simulation"]["timestep_s"])
        ))
        self.timestep = float(self.model.opt.timestep)
        self.dt = self.decimation * self.timestep
        self.termination = term_lib.TerminationConfig(
            episode_length_s=self.options.episode_length_s
        )

        # ------------------------------------------------------------ physics
        mjm = mujoco.MjModel.from_xml_path(str(xml))
        mjd = mujoco.MjData(mjm)
        self._spawn_qpos = np.zeros(mjm.nq)
        self._spawn_qpos[2] = self.spawn_z
        self._spawn_qpos[3] = 1.0
        for joint_name, angle in config.default_angles.items():
            self._spawn_qpos[self.cpu.qpos_adr[joint_name]] = angle
        mjd.qpos[:] = self._spawn_qpos
        mjd.qvel[:] = 0.0
        mjd.ctrl[:] = 0.0
        for name, entry in self.cpu.spring_state().items():
            mjd.ctrl[self.cpu.spring_motor_ids[name]] = float(entry["force_n"])
        mujoco.mj_forward(mjm, mjd)

        self.mjm = mjm
        self.mj = mjw.put_model(mjm)
        # nconmax / njmax are PER WORLD in mujoco_warp (measured: one nominal
        # standing pose needs ~43 constraints), and efc.J is allocated as
        # (nworld, njmax_pad, nv_pad), so passing totals explodes into GBs.
        self.d = mjw.put_data(
            mjm,
            mjd,
            nworld=self.num_envs,
            nconmax=self.options.nconmax_per_env,
            njmax=self.options.njmax_per_env,
        )
        mjw.step(self.mj, self.d)  # first touch: compile kernels, fill derived arrays
        warp.synchronize()

        # --------------------------------------------------- device-side views
        self.qpos = warp.to_torch(self.d.qpos)
        self.qvel = warp.to_torch(self.d.qvel)
        self.ctrl = warp.to_torch(self.d.ctrl)
        self.xpos = warp.to_torch(self.d.xpos)
        self.ten_length = warp.to_torch(self.d.ten_length)
        # Contacts live in ONE flat array shared by all worlds (world i's contacts
        # are packed after world 0's, so a fixed-stride view would mis-attribute
        # them).  Use the flat arrays plus ``worldid`` and scatter instead.
        self.contact_geom = warp.to_torch(self.d.contact.geom)
        self.contact_dist = warp.to_torch(self.d.contact.dist)
        self.contact_world = warp.to_torch(self.d.contact.worldid)
        #: ``nacon`` is the *global* number of active contacts; the valid entries are
        #: packed into the first ``nacon`` slots.  Slots beyond it keep whatever the
        #: previous step wrote there, which look like real contacts, so they must be
        #: masked out or every env reports phantom illegal contacts.
        self.nacon = warp.to_torch(self.d.nacon)
        self.slot_index = torch.arange(
            self.contact_geom.shape[0], dtype=torch.int32, device=self.device
        )

        dev = self.device
        self.qpos_idx = torch.tensor(
            [self.cpu.qpos_adr[name] for name in names.MODEL_JOINT_ORDER], device=dev
        )
        self.dof_idx = torch.tensor(
            [self.cpu.dof_adr[name] for name in names.MODEL_JOINT_ORDER], device=dev
        )
        self.ctrl_idx = torch.tensor(
            [self.cpu.joint_motor_ids[name] for name in names.MODEL_JOINT_ORDER], device=dev
        )
        self.spring_ctrl_idx = torch.tensor(
            [self.cpu.spring_motor_ids[spring.name] for spring in self.springs], device=dev
        )
        self.spring_ten_idx = torch.tensor(
            [self.cpu.tendon_ids[spring.name] for spring in self.springs], device=dev, dtype=torch.long
        )
        self.body_ids = torch.tensor(
            [self.cpu.body_ids[name] for name in names.LINK_NAMES], device=dev
        )
        self.hip_body = torch.tensor(
            [self.cpu.body_ids["r0_link"], self.cpu.body_ids["l0_link"]], device=dev
        )
        self.wheel_body = torch.tensor(
            [self.cpu.body_ids["right_wheel"], self.cpu.body_ids["left_wheel"]], device=dev
        )

        self.default = torch.tensor(self.default_angles, dtype=torch.float32, device=dev)
        self.pos_scale = torch.tensor(
            [self.gains.leg_position_scale if i in LEG else 0.0 for i in range(6)],
            dtype=torch.float32, device=dev,
        )
        self._zero_i = torch.tensor(LEG, device=dev)
        self._vel_i = torch.tensor(WHEEL, device=dev)
        leg_mask = torch.tensor([i in LEG for i in range(6)], device=dev)
        wheel_mask = ~leg_mask
        kp = torch.tensor(
            [self.gains.leg_kp if i in LEG else self.gains.wheel_kp for i in range(6)],
            dtype=torch.float32, device=dev,
        )
        kd = torch.tensor(
            [self.gains.leg_kd if i in LEG else self.gains.wheel_kd for i in range(6)],
            dtype=torch.float32, device=dev,
        )
        self.kp = kp
        self.kd = kd
        self.leg_mask = leg_mask
        self.wheel_mask = wheel_mask
        limits = self.gains.limits().astype(np.float32)
        self.torque_limit = torch.tensor(limits, dtype=torch.float32, device=dev)
        self.vel_scale = torch.tensor(self.gains.wheel_velocity_scale, dtype=torch.float32, device=dev)
        self.spring_force = torch.tensor(
            [float(self.cpu.spring_state()[s.name]["force_n"]) for s in self.springs],
            dtype=torch.float32, device=dev,
        )
        # Batched gas-spring model (same F(l, dl/dt) law as ``control.gas_spring``).
        # The force must be recomputed every physics sub-step, exactly like the
        # CPU env does; a constant force would make the two backends disagree
        # whenever the leg moves.
        if self.spring_count:
            def _col(fn) -> torch.Tensor:
                return torch.tensor(
                    [float(fn(spring)) for spring in self.springs],
                    dtype=torch.float32,
                    device=dev,
                )

            self.spring_f0 = _col(lambda s: s.force_ref_n)
            self.spring_k = _col(lambda s: s.stiffness_n_per_m)
            self.spring_l_ref = _col(lambda s: s.length_ref_m if s.length_ref_m is not None else 0.0)
            self.spring_has_stiffness = _col(
                lambda s: 1.0 if (s.stiffness_n_per_m and s.length_ref_m is not None) else 0.0
            )
            self.spring_damping = _col(lambda s: s.damping_n_s_per_m)
            self.spring_friction = _col(lambda s: s.friction_n)
            self.spring_smoothing = _col(
                lambda s: s.velocity_smoothing_m_s if s.velocity_smoothing_m_s else 1.0
            )
            self.spring_force_min = _col(lambda s: s.force_min_n)
            self.spring_force_max = _col(lambda s: s.force_max_n)
            self.spring_ref_force = torch.tensor(
                [reference_force(spring) for spring in self.springs],
                dtype=torch.float32,
                device=dev,
            )
        else:
            self.spring_ref_force = torch.zeros(0, dtype=torch.float32, device=dev)

        lower = np.full(6, -np.inf, dtype=np.float32)
        upper = np.full(6, np.inf, dtype=np.float32)
        for k, joint_name in enumerate(names.MODEL_JOINT_ORDER):
            limit = config.joint_limits[joint_name]
            if limit is not None:
                lower[k], upper[k] = limit
        self.q_lower = torch.tensor(lower, device=dev)
        self.q_upper = torch.tensor(upper, device=dev)

        self.command_scale = torch.tensor(
            [3.0, 0.25, 5.0], dtype=torch.float32, device=dev
        )
        self.command_mean = torch.tensor(
            [float(np.mean(r)) for r in self.options.command_ranges],
            dtype=torch.float32, device=dev,
        )
        self.command_half = torch.tensor(
            [float(0.5 * (r[1] - r[0])) for r in self.options.command_ranges],
            dtype=torch.float32, device=dev,
        )

        self.weights = {
            name: float(value) for name, value in reward_lib.DEFAULT_WEIGHTS.items()
        }
        # contact geometry -> body ids, prepared once
        self.geom_body = torch.tensor(
            np.asarray(mjm.geom_bodyid, dtype=np.int64), device=dev
        )
        wheel_geom_bodies = set(int(self.cpu.body_ids[n]) for n in ("right_wheel", "left_wheel"))
        self.wheel_bodies = torch.tensor(sorted(wheel_geom_bodies), device=dev)
        self.history = torch.zeros(
            (self.num_envs, self.history_length, self.num_obs),
            dtype=torch.float32, device=dev,
        )
        self.obs = torch.zeros((self.num_envs, self.num_obs), dtype=torch.float32, device=dev)
        self.privileged = torch.zeros(
            (self.num_envs, self.num_privileged_obs), dtype=torch.float32, device=dev
        )
        self.actions = torch.zeros((self.num_envs, 6), dtype=torch.float32, device=dev)
        self.previous_actions = torch.zeros_like(self.actions)
        self.previous_previous_actions = torch.zeros_like(self.actions)
        self.previous_velocity = torch.zeros_like(self.actions)
        self.torques = torch.zeros_like(self.actions)
        self.joint_acceleration = torch.zeros_like(self.actions)
        self.commands = torch.zeros((self.num_envs, 3), dtype=torch.float32, device=dev)
        self.episode_length = torch.zeros(self.num_envs, dtype=torch.float32, device=dev)
        self.episode_return = torch.zeros(self.num_envs, dtype=torch.float32, device=dev)
        self.tilt_time = torch.zeros(self.num_envs, dtype=torch.float32, device=dev)
        self.height_time = torch.zeros(self.num_envs, dtype=torch.float32, device=dev)
        self.terminal_privileged = torch.zeros_like(self.privileged)
        self.terminal_history = torch.zeros(
            (self.num_envs, names.HISTORY_DIM), dtype=torch.float32, device=dev
        )
        self.dones = torch.zeros(self.num_envs, dtype=torch.bool, device=dev)
        self.time_outs = torch.zeros(self.num_envs, dtype=torch.bool, device=dev)
        self.stats = {
            "episodes": 0,
            "returns": [],
            "lengths": [],
            "tilt": 0,
            "height": 0,
            "illegal_contact": 0,
            "nonfinite": 0,
            "time_out": 0,
        }
        self._running_length = torch.zeros(self.num_envs, dtype=torch.float32, device=dev)

        # ------------------------------------------------------------------ reset
        self.spawn_row = torch.tensor(self._spawn_qpos, dtype=torch.float32, device=dev)
        self.ctrl_row = torch.zeros(self.ctrl.shape[1], dtype=torch.float32, device=dev)
        if len(self.springs):
            self.ctrl_row[self.spring_ctrl_idx] = self.spring_force
        self.action_in = torch.zeros_like(self.actions)

        # ------------------------------------------- spawn-state obs / privileged
        # The reset state is fixed, so its observation is constant except for the
        # command channels, which are patched per env inside the masked reset.
        # The privileged offsets below follow ``mdp.observations.privileged``:
        # 0:3 lin_vel, 3:28 obs, 28:34 a(t-1), 34:40 a(t-2), 40:46 acc,
        # 46:123 heights, 123:129 torque, 129 mass, 130:133 com,
        # 133:139 default bias, 139 friction, 140 restitution, 141: springs.
        spawn_rotation = self._quat_to_mat(self.spawn_row[3:7].reshape(1, 4))[0]
        self.spawn_obs_base = torch.zeros(self.num_obs, dtype=torch.float32, device=dev)
        self.spawn_obs_base[3:6] = -spawn_rotation[2, :]
        self.spawn_privileged = torch.zeros(
            self.num_privileged_obs, dtype=torch.float32, device=dev
        )
        self.spawn_privileged[3 : 3 + self.num_obs] = self.spawn_obs_base
        self.spawn_privileged[46:123] = float(
            np.clip(self.spawn_z - 0.5, -1.0, 1.0)
        ) * 5.0
        self.spawn_privileged[139] = 1.0
        self.spawn_privileged[140] = 0.5
        if self.spring_count:
            state = self.cpu.spring_state()
            lengths = np.array(
                [float(state[spring.name]["length_m"]) for spring in self.springs], dtype=np.float32
            )
            forces = np.array(
                [float(state[spring.name]["force_n"]) for spring in self.springs], dtype=np.float32
            )
            self.spawn_spring_length = torch.tensor(lengths, device=dev)
            self.spawn_privileged[names.BASE_PRIVILEGED_DIM :] = torch.tensor(
                np.concatenate(
                    [
                        lengths * SPRING_LENGTH_SCALE,
                        np.zeros_like(lengths),
                        self.spring_ref_force.cpu().numpy() * SPRING_FORCE_SCALE,
                        forces * SPRING_FORCE_SCALE,
                    ]
                ),
                dtype=torch.float32,
                device=dev,
            )
        else:
            self.spawn_spring_length = torch.zeros(0, dtype=torch.float32, device=dev)
        #: full tendon-length row at the spawn pose (spring columns patched)
        self.ten_row = torch.zeros(self.ten_length.shape[1], dtype=torch.float32, device=dev)
        if self.spring_count:
            self.ten_row[self.spring_ten_idx] = self.spawn_spring_length
        self.ten_prev_length = self.spawn_spring_length.repeat(self.num_envs, 1)
        self.ten_obs_length = self.spawn_spring_length.repeat(self.num_envs, 1)
        self.spring_actual_force = torch.zeros_like(self.ten_prev_length)
        #: Commands drawn outside the captured graph and consumed by the next
        #: episode reset, so relaxing them costs no RNG call inside the graph.
        self.command_pool = torch.zeros((self.num_envs, 3), dtype=torch.float32, device=dev)

        self.reward = torch.zeros(self.num_envs, dtype=torch.float32, device=dev)
        self.stat_episodes = torch.zeros((), dtype=torch.float32, device=dev)
        self.stat_return_sum = torch.zeros((), dtype=torch.float32, device=dev)
        self.stat_length_sum = torch.zeros((), dtype=torch.float32, device=dev)
        self.stat_tilt = torch.zeros((), dtype=torch.float32, device=dev)
        self.stat_height = torch.zeros((), dtype=torch.float32, device=dev)
        self.stat_contact = torch.zeros((), dtype=torch.float32, device=dev)
        self.stat_nonfinite = torch.zeros((), dtype=torch.float32, device=dev)
        self.stat_timeout = torch.zeros((), dtype=torch.float32, device=dev)
        self.graph: Any | None = None

        self.ctrl[:, self.spring_ctrl_idx] = self.spring_force
        self.reset(resample_commands=True)
        self._sync()

    # ------------------------------------------------------------------ utils
    def _sync(self) -> None:
        self.warp.synchronize()

    def _quat_to_mat(self, q: torch.Tensor) -> torch.Tensor:
        w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
        norm = torch.sqrt(w * w + x * x + y * y + z * z).clamp_min(1e-9)
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
        row0 = torch.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1)
        row1 = torch.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1)
        row2 = torch.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)
        return torch.stack([row0, row1, row2], 1)

    # ------------------------------------------------------------------ reset
    def reset(
        self, env_ids: torch.Tensor | None = None, resample_commands: bool = True
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Explicit reset (evaluation, start-up).  Not capture-safe by design."""
        ids = (
            torch.arange(self.num_envs, device=self.device)
            if env_ids is None
            else env_ids.to(self.device)
        )
        n = ids.numel()
        self.qpos[ids] = self.spawn_row
        self.qvel[ids] = 0.0
        # ``ctrl[ids][:, idx] = 0`` would write into the temporary copy produced by
        # the first advanced index and silently leave the real control untouched.
        self.ctrl[ids.unsqueeze(1), self.ctrl_idx.unsqueeze(0)] = 0.0
        self.actions[ids] = 0.0
        self.previous_actions[ids] = 0.0
        self.previous_previous_actions[ids] = 0.0
        self.previous_velocity[ids] = 0.0
        self.torques[ids] = 0.0
        self.joint_acceleration[ids] = 0.0
        self.episode_length[ids] = 0.0
        self.episode_return[ids] = 0.0
        self.tilt_time[ids] = 0.0
        self.height_time[ids] = 0.0
        self._running_length[ids] = 0.0
        self.dones[ids] = False
        self.time_outs[ids] = False
        if self.spring_count:
            self.ten_length[ids] = self.ten_row
            self.ten_prev_length[ids] = self.spawn_spring_length
            self.ten_obs_length[ids] = self.spawn_spring_length
            self.spring_actual_force[ids] = self.spring_force
        if resample_commands and self.options.fixed_command is None:
            noise = torch.rand((n, 3), generator=self.generator, device=self.device) * 2.0 - 1.0
            command = self.command_mean + noise * self.command_half
            command[:, 2] = self.nominal_height + command[:, 2]
            self.commands[ids] = command
            self.command_pool[ids] = command
        elif resample_commands:
            self.commands[ids] = torch.as_tensor(
                self.options.fixed_command, dtype=torch.float32, device=self.device
            )
            self.command_pool[ids] = self.commands[ids]
        if self.options.randomize_initial:
            half = torch.rand((n, 6), generator=self.generator, device=self.device) - 0.5
            rng = torch.tensor([0.06, 0.06, 0.0, 0.06, 0.06, 0.0], device=self.device)
            self.qpos[ids.unsqueeze(1), self.qpos_idx.unsqueeze(0)] += half * rng * 2.0

        self._compute_observation(ids)
        self.history[ids] = self.obs[ids].unsqueeze(1).expand(n, self.history_length, self.num_obs)
        self._compute_privileged(ids)
        return self.obs[ids].clone(), self.privileged[ids].clone()

    def resample_commands(self) -> None:
        """New commands for every env; call outside the captured graph.

        The pool is what the *next* episode reset consumes, so a finished env
        gets a fresh command without a random draw inside the graph.  The
        observation command channels are refreshed immediately (doc 5.2): the
        next action must not run on the previous command.
        """
        if self.options.fixed_command is not None:
            self.commands[:] = torch.as_tensor(
                self.options.fixed_command, dtype=torch.float32, device=self.device
            )
            self.command_pool.copy_(self.commands)
            self._refresh_command_channels()
            return
        noise = torch.rand((self.num_envs, 3), generator=self.generator, device=self.device)
        command = self.command_mean + (noise * 2.0 - 1.0) * self.command_half
        command[:, 2] = self.nominal_height + command[:, 2]
        self.command_pool.copy_(command)
        self.commands.copy_(command)
        self._refresh_command_channels()

    def _refresh_command_channels(self) -> None:
        """Rewrite the command channels in place, without touching physics/history."""
        scaled = self.commands * self.command_scale
        self.obs[:, 6:9] = scaled
        self.history[:, -1, 6:9] = scaled
        self.privileged[:, 9:12] = scaled

    # ---------------------------------------------------------- observations
    def _compute_observation(self, ids: torch.Tensor | slice | None = None) -> None:
        sel = slice(None) if ids is None else ids
        qpos = self.qpos[sel]
        qvel = self.qvel[sel]
        rotation = self._quat_to_mat(qpos[:, 3:7])
        lin_world = qvel[:, 0:3]
        lin_body = torch.einsum("bij,bj->bi", rotation.transpose(1, 2), lin_world)
        ang_body = qvel[:, 3:6]  # free joint: angular part is already body-local
        projected_gravity = -rotation[:, 2, :]
        position = qpos[:, self.qpos_idx]
        velocity = qvel[:, self.dof_idx]

        obs = torch.empty((qpos.shape[0], self.num_obs), dtype=torch.float32, device=self.device)
        obs[:, 0:3] = ang_body * 0.25
        obs[:, 3:6] = projected_gravity
        obs[:, 6:9] = self.commands[sel] * self.command_scale
        obs[:, 9:13] = position[:, self._zero_i] - self.default[self._zero_i]
        obs[:, 13:19] = velocity * 0.05
        obs[:, 19:25] = self.actions[sel]
        self.obs[sel] = obs
        self.joint_acceleration[sel] = (velocity - self.previous_velocity[sel]) / self.dt
        self.previous_velocity[sel] = velocity

    def _compute_privileged(self, ids: torch.Tensor | slice | None = None) -> None:
        sel = slice(None) if ids is None else ids
        qpos = self.qpos[sel]
        qvel = self.qvel[sel]
        rotation = self._quat_to_mat(qpos[:, 3:7])
        lin_body = torch.einsum("bij,bj->bi", rotation.transpose(1, 2), qvel[:, 0:3])
        n = qpos.shape[0]
        heights = (qpos[:, 2] - 0.5).clamp(-1.0, 1.0).mul(5.0).unsqueeze(1).expand(n, 77)
        zeros3 = torch.zeros((n, 3), dtype=torch.float32, device=self.device)
        zeros6 = torch.zeros((n, 6), dtype=torch.float32, device=self.device)
        ones = torch.ones((n, 1), dtype=torch.float32, device=self.device)
        parts = [
            lin_body * 2.0,
            self.obs[sel],
            self.previous_actions[sel],
            self.previous_previous_actions[sel],
            self.joint_acceleration[sel] * 0.0025,
            heights,
            self.torques[sel] * 0.05,
            torch.zeros((n, 1), dtype=torch.float32, device=self.device),
            zeros3,
            zeros6,
            ones,
            ones * 0.5,
        ]
        if self.spring_count:
            length = self.ten_length[sel][:, self.spring_ten_idx]
            rate = (length - self.ten_obs_length[sel]) / self.dt
            actual = self._spring_forces(length, rate)
            self.ten_obs_length[sel] = length
            self.spring_actual_force[sel] = actual
            # Same field order and scaling as ``mdp.observations.spring_features``:
            # length*5, length rate, reference force/300, actual force/300.
            parts.append(
                torch.cat(
                    [
                        length * SPRING_LENGTH_SCALE,
                        rate,
                        self.spring_ref_force.expand(n, -1) * SPRING_FORCE_SCALE,
                        actual * SPRING_FORCE_SCALE,
                    ],
                    1,
                )
            )
        self.privileged[sel] = torch.cat(parts, 1)

    def get_observations(self) -> tuple[torch.Tensor, torch.Tensor]:
        return self.obs.clone(), self.history.reshape(self.num_envs, -1)

    def get_privileged_observations(self) -> torch.Tensor:
        return self.privileged.clone()

    # ------------------------------------------------------------------- step
    def _spring_forces(self, length: torch.Tensor, rate: torch.Tensor) -> torch.Tensor:
        """Batched ``control.gas_spring.spring_force`` (same law and clamping)."""
        force = self.spring_f0 + self.spring_has_stiffness * self.spring_k * (
            self.spring_l_ref - length
        )
        force = force - self.spring_damping * rate
        force = force - self.spring_friction * torch.tanh(rate / self.spring_smoothing)
        return torch.max(torch.min(force, self.spring_force_max), self.spring_force_min)

    def _write_controls(self, actions: torch.Tensor) -> None:
        """Batched ``decode_action`` + ``compute_torques`` (same law as the CPU env)."""
        position = self.qpos[:, self.qpos_idx]
        velocity = self.qvel[:, self.dof_idx]
        pos_target = self.default.expand_as(position) + self.pos_scale * actions
        vel_target = self.vel_scale * actions * self.wheel_mask
        torque = self.kp * (pos_target - position) + self.kd * (vel_target - velocity)
        torque = torch.max(torch.min(torque, self.torque_limit), -self.torque_limit)
        # in-place: a captured CUDA graph replays fixed addresses, so rebinding
        # ``self.torques = torque`` would silently freeze the value.
        self.torques.copy_(torque)
        self.ctrl[:, self.ctrl_idx] = torque
        if self.spring_count:
            # The gas spring is a physical element: its force follows the current
            # length every sub-step (doc 5.3), never a value written once at reset.
            length = self.ten_length[:, self.spring_ten_idx]
            rate = (length - self.ten_prev_length) / self.timestep
            force = self._spring_forces(length, rate)
            self.ten_prev_length.copy_(length)
            self.spring_actual_force.copy_(force)
            self.ctrl[:, self.spring_ctrl_idx] = force

    def step(self, actions: torch.Tensor) -> tuple[
        torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, dict[str, Any], torch.Tensor
    ]:
        """One policy step.  With a captured graph this is a single launch."""
        clean = torch.nan_to_num(
            actions.to(self.device), nan=0.0, posinf=100.0, neginf=-100.0
        ).clamp(-100.0, 100.0).reshape(self.num_envs, self.num_actions)
        if self.graph is not None:
            with torch.cuda.stream(self._torch_stream):
                self.action_in.copy_(clean)
                self.warp.capture_launch(self.graph)
            self._done_event.record(self._torch_stream)
            torch.cuda.current_stream().wait_event(self._done_event)
        else:
            self.action_in.copy_(clean)
            self._step_core()
        return (
            self.obs,
            self.privileged,
            self.reward,
            self.dones,
            {
                "time_outs": self.time_outs,
                "episode_returns": self.episode_return,
                "done_mask": self.dones,
                "terminal_privileged": self.terminal_privileged,
                "terminal_history": self.terminal_history,
            },
            self.history.reshape(self.num_envs, -1),
        )

    def _step_core(self) -> None:
        actions = self.action_in
        self.previous_previous_actions.copy_(self.previous_actions)
        self.previous_actions.copy_(self.actions)
        self.actions.copy_(actions)
        for _ in range(self.decimation):
            self._write_controls(actions)
            self.mjw.step(self.mj, self.d)
        self._compute_observation()
        self._compute_privileged()
        _reward = self._compute_reward_and_termination()
        self.reward.copy_(_reward)
        self._masked_reset_and_history()

    def _masked_reset_and_history(self) -> None:
        """Snapshot terminals, push history and reset done envs without a host sync.

        Order matters (doc 5.1): the newest history frame must be the terminal
        observation when the episode ends, and the observation returned for the
        next policy step must describe the *reset* state, not the state the
        episode died in.
        """
        done = self.dones
        mask_q = done.unsqueeze(1)

        # 1) append the new observation to the history for every env ...
        appended = torch.cat([self.history[:, 1:], self.obs[:, None, :]], 1)
        # 2) ... then snapshot it: terminal history ends with the terminal obs.
        self.terminal_history.copy_(
            torch.where(done[:, None], appended.reshape(self.num_envs, -1), self.terminal_history)
        )
        self.terminal_privileged.copy_(
            torch.where(mask_q, self.privileged, self.terminal_privileged)
        )

        # 3) the finished envs draw their new command from the pre-sampled pool.
        self.commands.copy_(torch.where(mask_q, self.command_pool, self.commands))
        command_channels = self.commands * self.command_scale
        spawn_obs = self.spawn_obs_base.expand_as(self.obs).clone()
        spawn_obs[:, 6:9] = command_channels
        spawn_priv = self.spawn_privileged.expand_as(self.privileged).clone()
        spawn_priv[:, 9:12] = command_channels
        spawn_history = spawn_obs[:, None, :].expand_as(self.history)

        self.qpos.copy_(torch.where(mask_q, self.spawn_row.expand_as(self.qpos), self.qpos))
        self.qvel.copy_(torch.where(mask_q, torch.zeros_like(self.qvel), self.qvel))
        self.ctrl.copy_(torch.where(mask_q, self.ctrl_row.expand_as(self.ctrl), self.ctrl))
        self.actions.copy_(torch.where(mask_q, torch.zeros_like(self.actions), self.actions))
        self.previous_actions.copy_(
            torch.where(mask_q, torch.zeros_like(self.actions), self.previous_actions)
        )
        self.previous_previous_actions.copy_(
            torch.where(mask_q, torch.zeros_like(self.actions), self.previous_previous_actions)
        )
        self.previous_velocity.copy_(
            torch.where(mask_q, torch.zeros_like(self.actions), self.previous_velocity)
        )
        self.torques.copy_(torch.where(mask_q, torch.zeros_like(self.actions), self.torques))
        self.joint_acceleration.copy_(
            torch.where(mask_q, torch.zeros_like(self.actions), self.joint_acceleration)
        )
        if self.spring_count:
            self.ten_length.copy_(
                torch.where(mask_q, self.ten_row.expand_as(self.ten_length), self.ten_length)
            )
            self.ten_prev_length.copy_(
                torch.where(
                    mask_q, self.spawn_spring_length.expand_as(self.ten_prev_length), self.ten_prev_length
                )
            )
            self.ten_obs_length.copy_(
                torch.where(
                    mask_q, self.spawn_spring_length.expand_as(self.ten_obs_length), self.ten_obs_length
                )
            )
            self.spring_actual_force.copy_(
                torch.where(
                    mask_q, self.spring_force.expand_as(self.spring_actual_force), self.spring_actual_force
                )
            )
        self.episode_return.copy_(
            torch.where(done, torch.zeros_like(self.episode_return), self.episode_return)
        )
        self.tilt_time.copy_(torch.where(done, torch.zeros_like(self.tilt_time), self.tilt_time))
        self.height_time.copy_(torch.where(done, torch.zeros_like(self.height_time), self.height_time))
        self._running_length.copy_(
            torch.where(done, torch.zeros_like(self._running_length), self._running_length)
        )
        # 4) the returned observation / privileged / history of a finished env is
        #    the reset state, so the next action is taken from the spawn pose.
        self.obs.copy_(torch.where(mask_q, spawn_obs, self.obs))
        self.privileged.copy_(torch.where(mask_q, spawn_priv, self.privileged))
        self.history.copy_(torch.where(done[:, None, None], spawn_history, appended))
        self.episode_length.copy_(self._running_length)

    #: Buffers that must survive the graph warm-up steps untouched.
    _STATEFUL = (
        "obs",
        "privileged",
        "history",
        "actions",
        "previous_actions",
        "previous_previous_actions",
        "previous_velocity",
        "torques",
        "joint_acceleration",
        "commands",
        "command_pool",
        "episode_length",
        "episode_return",
        "tilt_time",
        "height_time",
        "_running_length",
        "terminal_privileged",
        "terminal_history",
        "dones",
        "time_outs",
        "ten_prev_length",
        "ten_obs_length",
        "spring_actual_force",
    )

    def _snapshot(self) -> dict[str, torch.Tensor]:
        saved = {name: getattr(self, name).clone() for name in self._STATEFUL}
        saved["qpos"] = self.qpos.clone()
        saved["qvel"] = self.qvel.clone()
        saved["ctrl"] = self.ctrl.clone()
        saved["ten_length"] = self.ten_length.clone()
        return saved

    def _restore(self, saved: dict[str, torch.Tensor]) -> None:
        for name in self._STATEFUL:
            getattr(self, name).copy_(saved[name])
        self.qpos.copy_(saved["qpos"])
        self.qvel.copy_(saved["qvel"])
        self.ctrl.copy_(saved["ctrl"])
        self.ten_length.copy_(saved["ten_length"])

    def capture(self, warmup: int = 3) -> bool:
        """Capture one policy step into a CUDA graph.

        Eager execution of one step costs ~100 ms here because a single step
        issues O(120) kernel launches (warp + torch) at ~0.3-0.7 ms of host
        time each, which is latency-bound rather than throughput-bound: the cost
        is the same for 1 world and for 2000.  Recording the step once and
        replaying it collapses that to one launch.

        ``torch.cuda.graph`` cannot be used directly: warp's default stream is
        not a capture-legal stream, so torch invalidates the capture.  Warp's
        own capture works, and because CUDA capture is stream based it also
        picks up the torch kernels as long as torch runs on the same stream.

        The warm-up steps really advance the simulation, so the state is
        snapshotted and restored: a captured env must start where the eager env
        would.
        """
        if self.graph is not None:
            return True
        self.action_in.zero_()
        stream = self.warp.get_stream()
        self._torch_stream = self.warp.stream_to_torch(stream)
        self._done_event = torch.cuda.Event()
        saved = self._snapshot()
        try:
            with torch.cuda.stream(self._torch_stream):
                for _ in range(warmup):
                    self._step_core()
            self.warp.synchronize()
            self._restore(saved)
            with self.warp.ScopedCapture() as capture:
                with torch.cuda.stream(self._torch_stream):
                    self._step_core()
            self.warp.synchronize()
            self._restore(saved)
        except Exception as error:  # pragma: no cover - depends on the driver
            self.capture_error = f"{type(error).__name__}: {error}"
            self.graph = None
            self._restore(saved)
            return False
        self.graph = capture.graph
        self.capture_error = None
        return True

    def _illegal_contact_count(self) -> torch.Tensor:
        """Contacts between the floor and anything except the wheels.

        Fixed-shape arithmetic plus ``index_add_`` only: no ``nonzero``, no
        boolean indexing, so it can run inside a captured graph.
        """
        geom = self.contact_geom
        body_a = self.geom_body[geom[:, 0].long()]
        body_b = self.geom_body[geom[:, 1].long()]
        robot = torch.where(body_a == 0, body_b, body_a)
        floor = (body_a == 0) | (body_b == 0)
        is_wheel = (robot == self.wheel_bodies[0]) | (robot == self.wheel_bodies[1])
        touching = self.contact_dist <= 1e-5
        valid = self.slot_index < self.nacon[0].to(torch.int32)
        illegal = floor & (robot != 0) & ~is_wheel & touching & valid
        flags = illegal.to(torch.float32)
        counts = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)
        counts.index_add_(0, self.contact_world.long().clamp(0, self.num_envs - 1), flags)
        return counts

    def _theta0(self) -> torch.Tensor:
        """In-plane leg angle per side, body frame, as in the CPU env."""
        rotation = self._quat_to_mat(self.qpos[:, 3:7])
        hip = self.xpos[:, self.hip_body]
        wheel = self.xpos[:, self.wheel_body]
        offset = torch.einsum("bij,bnj->bni", rotation.transpose(1, 2), wheel - hip)
        return torch.atan2(-offset[..., 2], offset[..., 0])

    def _compute_reward_and_termination(self) -> torch.Tensor:
        qpos = self.qpos
        qvel = self.qvel
        rotation = self._quat_to_mat(qpos[:, 3:7])
        lin_body = torch.einsum("bij,bj->bi", rotation.transpose(1, 2), qvel[:, 0:3])
        ang_body = qvel[:, 3:6]
        projected_gravity = -rotation[:, 2, :]
        position = qpos[:, self.qpos_idx]
        velocity = qvel[:, self.dof_idx]
        command = self.commands
        speed_sq = (command[:, 0] - lin_body[:, 0]) ** 2
        yaw_sq = (command[:, 1] - ang_body[:, 2]) ** 2
        theta = self._theta0()
        illegal_count = self._illegal_contact_count()

        terms = {
            "tracking_lin_vel": torch.exp(-speed_sq / 0.25),
            "tracking_lin_vel_enhance": torch.exp(-speed_sq / 2.5) - 1.0,
            "tracking_ang_vel": torch.exp(-yaw_sq / 0.25),
            "tracking_ang_vel_enhance": torch.exp(-yaw_sq / 2.5) - 1.0,
            "base_height": torch.exp(-((qpos[:, 2] - command[:, 2]) ** 2) / 0.001),
            "nominal_state": (theta[:, 1] - theta[:, 0]) ** 2,
            "lin_vel_z": lin_body[:, 2] ** 2,
            "ang_vel_xy": ang_body[:, 0] ** 2 + ang_body[:, 1] ** 2,
            "orientation": projected_gravity[:, 0] ** 2 + projected_gravity[:, 1] ** 2,
            "dof_vel": (velocity[:, self._zero_i] ** 2).sum(-1),
            "dof_acc": (self.joint_acceleration**2).sum(-1),
            "torques": (self.torques**2).sum(-1),
            "action_rate": ((self.actions - self.previous_actions) ** 2).sum(-1),
            "action_smooth": (
                (self.actions - 2.0 * self.previous_actions + self.previous_previous_actions) ** 2
            ).sum(-1),
            "collision": illegal_count,
            "dof_pos_limits": (
                (self.q_lower + 0.01 - position).clamp_min(0.0)
                + (position - (self.q_upper - 0.01)).clamp_min(0.0)
            ).sum(-1),
        }
        total = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)
        for name, value in terms.items():
            weight = self.weights.get(name, 0.0)
            total = total + (weight * value).clamp(-1.0, 1.0) * self.dt
        self.episode_return.add_(total)

        # ---------------------------------------------------------- terminations
        tilt = rotation[:, 2, 2] < self.termination.tilt_cos
        self.tilt_time.copy_(
            torch.where(tilt, self.tilt_time + self.dt, torch.zeros_like(self.tilt_time))
        )
        failed_tilt = self.tilt_time >= self.termination.tilt_grace_s

        low = qpos[:, 2] < self.termination.height_ratio * self.nominal_height
        self.height_time.copy_(
            torch.where(low, self.height_time + self.dt, torch.zeros_like(self.height_time))
        )
        failed_height = self.height_time >= self.termination.height_grace_s

        failed_contact = illegal_count > 0
        nonfinite = ~(
            torch.isfinite(qpos).all(-1)
            & torch.isfinite(qvel).all(-1)
            & torch.isfinite(self.obs).all(-1)
            & torch.isfinite(self.actions).all(-1)
        )
        self._running_length.add_(self.dt)
        timeout = self._running_length >= self.termination.episode_length_s - 1e-9
        failed = failed_tilt | failed_height | failed_contact | nonfinite
        self.dones.copy_(failed | timeout)
        self.time_outs.copy_(timeout & ~failed)

        # device-side episode bookkeeping: no host reads inside the graph
        mask = self.dones.to(torch.float32)
        # the terminal observation/history snapshots happen in
        # ``_masked_reset_and_history`` *after* the new frame is appended.
        self.stat_episodes.add_(mask.sum())
        self.stat_return_sum.add_((self.episode_return * mask).sum())
        self.stat_length_sum.add_((self._running_length * mask).sum())
        self.stat_tilt.add_((failed_tilt.to(torch.float32) * mask).sum())
        self.stat_height.add_((failed_height.to(torch.float32) * mask).sum())
        self.stat_contact.add_((failed_contact.to(torch.float32) * mask).sum())
        self.stat_nonfinite.add_((nonfinite.to(torch.float32) * mask).sum())
        self.stat_timeout.add_((self.time_outs.to(torch.float32) * mask).sum())
        return total

    # ------------------------------------------------------------- utilities
    def drain_stats(self, reset: bool = True) -> dict[str, Any]:
        values = [
            self.stat_episodes,
            self.stat_return_sum,
            self.stat_length_sum,
            self.stat_tilt,
            self.stat_height,
            self.stat_contact,
            self.stat_nonfinite,
            self.stat_timeout,
        ]
        episodes, ret_sum, len_sum, tilt, height, contact, nonfinite, timeout = (
            float(v) for v in torch.stack(values).cpu()
        )
        if reset:
            for tensor in values:
                tensor.zero_()
        return {
            "episodes": int(episodes),
            "return_mean": ret_sum / episodes if episodes else float("nan"),
            "length_mean": len_sum / episodes if episodes else float("nan"),
            "tilt": int(tilt),
            "height": int(height),
            "illegal_contact": int(contact),
            "nonfinite": int(nonfinite),
            "time_out": int(timeout),
        }
