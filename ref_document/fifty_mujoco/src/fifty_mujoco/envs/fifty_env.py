"""Vectorised single-robot MuJoCo environment implementing the PPO protocol.

Protocol (doc section 5.6)::

    get_observations()            -> obs, history
    get_privileged_observations() -> privileged
    reset()                       -> obs, privileged
    step(actions)                 -> obs, privileged, rewards, dones, infos, history
    set_commands(values)          -> refresh the command channels, no physics

Every physics sub-step refreshes the gas-spring geometry, the (delayed) action,
the PD torque and the motor limit.  Rewards, terminations and observations are
all computed after the last sub-step, from the same state.

Step order (doc 5.1) is fixed:

1. build the action, advance physics,
2. compute the new obs / privileged / reward / termination,
3. append the new observation to the history,
4. snapshot the terminal observation and *its* history (last frame == terminal
   observation) for the timeout bootstrap,
5. reset the finished envs and refill their history with the new initial frame,
6. return the reset observation for the next policy inference and the terminal
   snapshot separately in ``infos``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import mujoco
import numpy as np

from ..config import Config
from ..contracts import names
from ..control.gas_spring import load_springs, reference_force, spring_force, spring_geometry
from ..control.mixed_pd import PdGains, compute_torques, decode_action
from ..control.spawn import SpawnSpec, load_spawn_spec, resolve_spawn_height
from ..envs.cpu_env import FiftyCpuEnv
from ..mdp import observations as obs_lib
from ..mdp import rewards as reward_lib
from ..mdp import terminations as term_lib
from ..mdp.commands import Commands, CommandRanges, fixed_command, sample_commands


@dataclass
class EnvOptions:
    num_envs: int = 1
    variant: str = "train"
    seed: int = 0
    episode_length_s: float = 20.0
    action_delay_steps: int = 0
    motor_enabled: bool = True
    compensation_mode: str = "off"
    noise: bool = False
    command_ranges: CommandRanges = field(default_factory=CommandRanges)
    fixed_command: tuple[float, float, float] | None = None
    gains: PdGains | None = None
    randomize: bool = False


class FiftyVecEnv:
    def __init__(self, config: Config, options: EnvOptions | None = None):
        self.config = config
        self.options = options or EnvOptions()
        if self.options.num_envs < 1:
            raise ValueError("num_envs must be >= 1")

        from ..paths import assets_dir

        xml = assets_dir() / "mjcf" / f"fifty_{self.options.variant}.xml"
        self.single = FiftyCpuEnv(xml, config)  # name maps, spring list, summaries
        self.model = self.single.model
        self.num_envs = int(self.options.num_envs)

        self.data = [mujoco.MjData(self.model) for _ in range(self.num_envs)]
        self.rng = np.random.default_rng(self.options.seed)

        self.spawn: SpawnSpec = load_spawn_spec(config)
        self.spawn_z = resolve_spawn_height(self.single, self.spawn)[0]

        self.default_angles = np.array(
            [config.default_angles[name] for name in names.MODEL_JOINT_ORDER], dtype=float
        )
        self.nominal_height = self.spawn_z - float(config.spawn["spawn"]["wheel_clearance_m"])
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
        self.decimation = int(round(
            float(config.robot["control"]["policy_dt_s"]) / float(config.robot["simulation"]["timestep_s"])
        ))
        self.dt = self.decimation * float(self.model.opt.timestep)

        self.springs = load_springs(config) if config.gas_spring_configured else []
        self.spring_count = len(self.springs)
        #: Static force at the design reference length, per spring.  Kept apart
        #: from the instantaneous force so the critic feature vector never mixes
        #: "what the model asks for" with "what the spring currently delivers".
        self.spring_reference_forces = np.array(
            [reference_force(spring) for spring in self.springs], dtype=float
        )
        self.num_obs = names.ACTOR_OBS_DIM
        self.num_privileged_obs, self.critic_input_dim = names.privileged_dim(self.spring_count)
        self.num_actions = names.NUM_ACTIONS
        self.obs_history_len = names.HISTORY_LENGTH

        self.termination = term_lib.TerminationConfig(episode_length_s=self.options.episode_length_s)
        self.wheel_body_ids = {
            self.single.body_ids[name] for name in ("right_wheel", "left_wheel")
        }
        self.hip_joint_ids = {
            "r": self.single.joint_ids["rf0_Joint"],
            "l": self.single.joint_ids["lf0_Joint"],
        }
        self.wheel_body_ids_by_side = {
            "r": self.single.body_ids["right_wheel"],
            "l": self.single.body_ids["left_wheel"],
        }

        n = self.num_envs
        self.obs = np.zeros((n, self.num_obs))
        self.privileged = np.zeros((n, self.num_privileged_obs))
        self.history = np.zeros((n, self.obs_history_len, self.num_obs))
        self.rewards = np.zeros(n)
        self.dones = np.zeros(n, dtype=bool)
        self.time_outs = np.zeros(n, dtype=bool)
        self.actions = np.zeros((n, self.num_actions))
        self.previous_actions = np.zeros((n, self.num_actions))
        self.previous_previous_actions = np.zeros((n, self.num_actions))
        self.previous_joint_velocity = np.zeros((n, self.num_actions))
        self.motor_torques = np.zeros((n, self.num_actions))
        self.joint_acceleration = np.zeros((n, self.num_actions))
        self.episode_length = np.zeros(n)
        self.tilt_fail_time = np.zeros(n)
        self.height_fail_time = np.zeros(n)
        self.terminal_obs = np.zeros((n, self.num_privileged_obs))
        self.terminal_history = np.zeros((n, names.HISTORY_DIM))
        self.episode_returns = np.zeros(n)
        self.episode_reward_terms: dict[str, float] = {}
        self.spring_lengths = np.zeros((n, max(1, self.spring_count)))
        self.spring_forces = np.zeros((n, max(1, self.spring_count)))
        self.termination_reason = np.full(n, "", dtype=object)
        #: Per-step quantities captured *before* the reset, so an evaluator never
        #: scores the spawn state of the next episode as if it were the failure.
        self.step_stats = {
            "height": np.zeros(n),
            "tilt_deg": np.zeros(n),
            "lin_vel_body": np.zeros((n, 3)),
            "ang_vel_body": np.zeros((n, 3)),
            "root_xy": np.zeros((n, 2)),
            "torque": np.zeros((n, self.num_actions)),
            "alive": np.zeros(n, dtype=bool),
        }

        self.delay_buffer = np.zeros((n, max(1, self.options.action_delay_steps), self.num_actions))
        self.delay_index = np.zeros(n, dtype=int)
        self.commands = self._make_commands()

        self._reset_all()

    # ------------------------------------------------------------------ helpers
    def _make_commands(self) -> Commands:
        if self.options.fixed_command is not None:
            vx, wz, height = self.options.fixed_command
            return fixed_command(self.num_envs, vx, wz, height)
        return sample_commands(self.num_envs, self.nominal_height, self.options.command_ranges, self.rng)

    def _joint_state(self, index: int) -> tuple[np.ndarray, np.ndarray]:
        data = self.data[index]
        position = np.array([data.qpos[self.single.qpos_adr[name]] for name in names.MODEL_JOINT_ORDER])
        velocity = np.array([data.qvel[self.single.dof_adr[name]] for name in names.MODEL_JOINT_ORDER])
        return position, velocity

    def _base_velocity(self, index: int) -> tuple[np.ndarray, np.ndarray]:
        """Body-frame (angular, linear) velocity of the base link origin."""
        data = self.data[index]
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(
            self.model, data, mujoco.mjtObj.mjOBJ_BODY, self.single.body_ids["base_link"], velocity, 1
        )
        return velocity[:3].copy(), velocity[3:].copy()

    def _base_rotation(self, index: int) -> np.ndarray:
        data = self.data[index]
        quat = np.asarray(data.qpos[3:7], dtype=float)
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, quat / np.linalg.norm(quat))
        return rotation.reshape(3, 3)

    def _theta0(self, index: int) -> dict[str, float]:
        data = self.data[index]
        rotation = self._base_rotation(index)
        result = {}
        for side, wheel_body in self.wheel_body_ids_by_side.items():
            hip_id = self.hip_joint_ids[side]
            hip = np.asarray(data.xanchor[hip_id], dtype=float)
            wheel = np.asarray(data.xpos[wheel_body], dtype=float)
            offset = rotation.T @ (wheel - hip)
            result[side] = float(np.arctan2(-offset[2], offset[0]))
        return result

    def _spring_state(self, index: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """lengths, rates, forces, generalized joint torque (6,) at the current state."""
        lengths = np.zeros(self.spring_count)
        rates = np.zeros(self.spring_count)
        forces = np.zeros(self.spring_count)
        joint_torque = np.zeros(6)
        for k, spring in enumerate(self.springs):
            proxy = _SlotView(self.single, self.data[index])
            geometry = spring_geometry(proxy, spring)
            force = spring_force(spring, geometry["length_m"], geometry["length_rate_m_s"])
            lengths[k] = geometry["length_m"]
            rates[k] = geometry["length_rate_m_s"]
            forces[k] = force
            joint_torque += geometry["jacobian"][6:] * force
        return lengths, rates, forces, joint_torque

    def _write_controls(self, index: int, action: np.ndarray) -> None:
        data = self.data[index]
        position, velocity = self._joint_state(index)
        targets = decode_action(action, self.config.default_angles, self.gains)

        spring_torque = None
        lengths = rates = forces = None
        if self.spring_count:
            lengths, rates, forces, spring_torque = self._spring_state(index)

        estimated = spring_torque if self.options.compensation_mode == "modeled_feedforward" else None
        torque = compute_torques(
            targets,
            position,
            velocity,
            self.gains,
            estimated_spring_torque=estimated,
            motor_enabled=self.options.motor_enabled,
        )
        self.motor_torques[index] = torque
        for joint_name, value in zip(names.MODEL_JOINT_ORDER, torque, strict=True):
            data.ctrl[self.single.joint_motor_ids[joint_name]] = float(value)

        if self.spring_count:
            self.spring_lengths[index, : self.spring_count] = lengths
            self.spring_forces[index, : self.spring_count] = forces
            for k, spring in enumerate(self.springs):
                data.ctrl[self.single.spring_motor_ids[spring.name]] = float(forces[k])

    # -------------------------------------------------------------------- reset
    def _reset_env(self, index: int) -> None:
        data = self.data[index]
        mujoco.mj_resetData(self.model, data)
        data.qpos[0:3] = (self.spawn.root_xy_m[0], self.spawn.root_xy_m[1], self.spawn_z)
        data.qpos[3:7] = self.spawn.root_quat_wxyz
        for joint_name, angle in self.config.default_angles.items():
            data.qpos[self.single.qpos_adr[joint_name]] = angle
        data.ctrl[:] = 0.0
        self.actions[index] = 0.0
        self.previous_actions[index] = 0.0
        self.previous_previous_actions[index] = 0.0
        self.previous_joint_velocity[index] = 0.0
        self.motor_torques[index] = 0.0
        self.joint_acceleration[index] = 0.0
        self.episode_length[index] = 0.0
        self.tilt_fail_time[index] = 0.0
        self.height_fail_time[index] = 0.0
        self.episode_returns[index] = 0.0
        self.delay_buffer[index] = 0.0
        self.delay_index[index] = 0
        self.dones[index] = False
        self.time_outs[index] = False
        self.termination_reason[index] = ""
        mujoco.mj_forward(self.model, data)

        if self.options.fixed_command is None:
            command = sample_commands(1, self.nominal_height, self.options.command_ranges, self.rng)
            self.commands.values[index] = command.values[0]
        self.commands.resample_timer[index] = self.options.command_ranges.resample_time_s

        self._compute_observation(index)
        obs_lib.fill_history(self.history, np.array([index]), self.obs)

    def _reset_all(self) -> None:
        for index in range(self.num_envs):
            self._reset_env(index)
        self._compute_privileged()

    def reset(self, env_ids: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        ids = np.arange(self.num_envs) if env_ids is None else np.asarray(env_ids, dtype=int)
        for index in ids:
            self._reset_env(int(index))
        self._compute_privileged(ids)
        return self.obs[ids].copy(), self.privileged[ids].copy()

    # ---------------------------------------------------------------- commands
    def set_commands(self, values: np.ndarray, env_ids: np.ndarray | None = None) -> None:
        """Overwrite commands and refresh the observation immediately.

        The refresh does not advance physics, does not shift the history and
        does not recompute the joint acceleration: only the command channels
        (obs 6:9 and the newest history frame) change, so the very next action
        already sees the new command.
        """
        ids = np.arange(self.num_envs) if env_ids is None else np.asarray(env_ids, dtype=int)
        values = np.asarray(values, dtype=float)
        if values.ndim == 1:
            values = np.tile(values, (ids.size, 1))
        if values.shape != (ids.size, 3):
            raise ValueError(f"commands must be (N, 3), got {values.shape}")
        self.commands.values[ids] = values
        self._refresh_command_channels(ids)

    def _refresh_command_channels(self, ids: np.ndarray) -> None:
        scaled = self.commands.values[ids] * obs_lib.COMMAND_SCALE
        self.obs[ids, 6:9] = scaled
        self.history[ids, -1, 6:9] = scaled
        self._compute_privileged(ids)

    def _resample_expired_commands(self, index: int) -> None:
        """Per-env command resampling on the *episode* clock (doc 5.2)."""
        if self.options.fixed_command is not None:
            return
        self.commands.resample_timer[index] -= self.dt
        if self.commands.resample_timer[index] > 0.0:
            return
        command = sample_commands(1, self.nominal_height, self.options.command_ranges, self.rng)
        self.commands.values[index] = command.values[0]
        self.commands.resample_timer[index] = self.options.command_ranges.resample_time_s
        self._refresh_command_channels(np.array([index]))

    # -------------------------------------------------------------- observation
    def _compute_observation(self, index: int) -> None:
        data = self.data[index]
        ang_vel, _lin_vel = self._base_velocity(index)
        rotation = self._base_rotation(index)
        projected_gravity = rotation.T @ np.array([0.0, 0.0, -1.0])
        position, velocity = self._joint_state(index)
        self.obs[index] = obs_lib.proprio(
            ang_vel,
            projected_gravity,
            self.commands.values[index],
            position,
            velocity,
            self.actions[index],
            self.default_angles,
        )
        self.joint_acceleration[index] = (
            velocity - self.previous_joint_velocity[index]
        ) / self.dt
        self.previous_joint_velocity[index] = velocity

    def _compute_privileged(self, env_ids: np.ndarray | None = None) -> None:
        ids = np.arange(self.num_envs) if env_ids is None else np.asarray(env_ids, dtype=int)
        for index in ids:
            index = int(index)
            data = self.data[index]
            _ang_vel, lin_vel = self._base_velocity(index)
            heights = np.full(77, float(np.clip(data.qpos[2] - 0.5, -1.0, 1.0)) * 5.0)
            base = obs_lib.privileged(
                lin_vel,
                self.obs[index],
                self.previous_actions[index],
                self.previous_previous_actions[index],
                self.joint_acceleration[index],
                heights,
                self.motor_torques[index],
                mass_bias=0.0,
                com_bias=np.zeros(3),
                default_angle_bias=np.zeros(6),
                friction=1.0,
                restitution=0.5,
            )
            if self.spring_count:
                lengths, rates, forces, _ = self._spring_state(index)
                extra = obs_lib.spring_features(
                    lengths, rates, self.spring_reference_forces, forces
                )
                self.privileged[index] = np.concatenate([base, extra])
            else:
                self.privileged[index] = base

    def get_observations(self) -> tuple[np.ndarray, np.ndarray]:
        """Read-only: never advances the history buffer."""
        return self.obs.copy(), obs_lib.flatten_history(self.history)

    def get_privileged_observations(self) -> np.ndarray:
        return self.privileged.copy()

    # --------------------------------------------------------------------- step
    def step(self, actions: np.ndarray) -> tuple[
        np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, Any], np.ndarray
    ]:
        actions = np.clip(np.asarray(actions, dtype=float).reshape(self.num_envs, self.num_actions), -100.0, 100.0)
        self.previous_previous_actions[:] = self.previous_actions
        self.previous_actions[:] = self.actions
        self.actions[:] = actions

        applied = self.actions
        if self.options.action_delay_steps > 0:
            self.delay_buffer[np.arange(self.num_envs), self.delay_index] = self.actions
            applied = self.delay_buffer[np.arange(self.num_envs), (self.delay_index + 1) % self.options.action_delay_steps]
            self.delay_index = (self.delay_index + 1) % self.options.action_delay_steps

        for _ in range(self.decimation):
            for index in range(self.num_envs):
                self._write_controls(index, applied[index])
                mujoco.mj_step(self.model, self.data[index])

        for index in range(self.num_envs):
            self._compute_observation(index)

        # Privileged observations must describe the *current* state before the
        # termination bookkeeping copies them as the terminal observation.
        self._compute_privileged()
        rewards, infos = self._compute_rewards_and_terminations()

        # Snapshot ``dones`` before the reset clears it: the learner needs the
        # done flags of *this* step, not of the state it is about to start.
        dones = self.dones.copy()
        env_ids = np.nonzero(dones)[0]
        if env_ids.size:
            infos["terminal_obs"] = self.terminal_obs[env_ids].copy()
            infos["terminal_history"] = self.terminal_history[env_ids].copy()
            infos["reset_env_ids"] = env_ids.copy()
            self.reset(env_ids)

        return (
            self.obs.copy(),
            self.privileged.copy(),
            rewards,
            dones,
            infos,
            obs_lib.flatten_history(self.history),
        )

    def _compute_rewards_and_terminations(self) -> tuple[np.ndarray, dict[str, Any]]:
        rewards = np.zeros(self.num_envs)
        infos: dict[str, Any] = {"time_outs": np.zeros(self.num_envs, dtype=bool)}
        terminations: dict[str, int] = {}

        for index in range(self.num_envs):
            data = self.data[index]
            rotation = self._base_rotation(index)
            ang_vel, lin_vel = self._base_velocity(index)
            position, velocity = self._joint_state(index)
            projected_gravity = rotation.T @ np.array([0.0, 0.0, -1.0])
            theta0 = self._theta0(index)
            command = self.commands.values[index]

            illegal = term_lib.illegal_contacts(self.model, data, self.wheel_body_ids)
            lower = np.full(6, -np.inf)
            upper = np.full(6, np.inf)
            for k, name in enumerate(names.MODEL_JOINT_ORDER):
                limit = self.config.joint_limits[name]
                if limit is not None:
                    lower[k], upper[k] = limit

            self.step_stats["height"][index] = float(data.qpos[2])
            self.step_stats["tilt_deg"][index] = term_lib.tilt_angle_deg(rotation)
            self.step_stats["lin_vel_body"][index] = lin_vel
            self.step_stats["ang_vel_body"][index] = ang_vel
            self.step_stats["root_xy"][index] = data.qpos[0:2]
            self.step_stats["torque"][index] = self.motor_torques[index]
            self.step_stats["alive"][index] = True

            terms = {
                "tracking_lin_vel": reward_lib.tracking_lin_vel(command[0], lin_vel[0]),
                "tracking_lin_vel_enhance": reward_lib.tracking_lin_vel_enhance(command[0], lin_vel[0]),
                "tracking_ang_vel": reward_lib.tracking_ang_vel(command[1], ang_vel[2]),
                "tracking_ang_vel_enhance": reward_lib.tracking_ang_vel_enhance(command[1], ang_vel[2]),
                "base_height": reward_lib.base_height(float(data.qpos[2]), command[2]),
                "nominal_state": reward_lib.nominal_state(theta0["l"], theta0["r"]),
                "lin_vel_z": reward_lib.lin_vel_z(lin_vel[2]),
                "ang_vel_xy": reward_lib.ang_vel_xy(ang_vel[0], ang_vel[1]),
                "orientation": reward_lib.orientation(projected_gravity[0], projected_gravity[1]),
                "dof_vel": reward_lib.dof_vel(velocity[list(names.POLICY_LEG_INDICES)]),
                "dof_acc": reward_lib.dof_acc(self.joint_acceleration[index]),
                "torques": reward_lib.torques(self.motor_torques[index]),
                "action_rate": reward_lib.action_rate(self.actions[index], self.previous_actions[index]),
                "action_smooth": reward_lib.action_smooth(
                    self.actions[index], self.previous_actions[index], self.previous_previous_actions[index]
                ),
                "collision": reward_lib.collision(len(illegal)),
                "dof_pos_limits": reward_lib.dof_pos_limits(position, lower, upper),
            }
            reward, contribution = reward_lib.combine(terms, self.dt)
            rewards[index] = reward
            self.episode_returns[index] += reward

            # ------------------------------------------------------- terminations
            failed = False
            if term_lib.orientation_failed(rotation, self.termination):
                self.tilt_fail_time[index] += self.dt
            else:
                self.tilt_fail_time[index] = 0.0
            if self.tilt_fail_time[index] >= self.termination.tilt_grace_s:
                failed = True
                terminations["tilt"] = terminations.get("tilt", 0) + 1

            if term_lib.height_failed(float(data.qpos[2]), self.nominal_height, self.termination):
                self.height_fail_time[index] += self.dt
            else:
                self.height_fail_time[index] = 0.0
            if self.height_fail_time[index] >= self.termination.height_grace_s:
                failed = True
                terminations["height"] = terminations.get("height", 0) + 1

            if illegal:
                failed = True
                terminations["illegal_contact"] = terminations.get("illegal_contact", 0) + 1

            if term_lib.nonfinite_failed(
                qpos=data.qpos, qvel=data.qvel, obs=self.obs[index], action=self.actions[index]
            ):
                failed = True
                terminations["nonfinite"] = terminations.get("nonfinite", 0) + 1

            self.episode_length[index] += self.dt
            timeout = self.episode_length[index] >= self.termination.episode_length_s - 1e-9
            if timeout:
                terminations["time_out"] = terminations.get("time_out", 0) + 1

            self.dones[index] = bool(failed or timeout)
            self.time_outs[index] = bool(timeout and not failed)
            if failed:
                if term_lib.orientation_failed(rotation, self.termination):
                    reason = "tilt"
                elif term_lib.height_failed(float(data.qpos[2]), self.nominal_height, self.termination):
                    reason = "height"
                elif illegal:
                    reason = "illegal_contact"
                else:
                    reason = "nonfinite"
            elif timeout:
                reason = "time_out"
            else:
                reason = ""
            self.termination_reason[index] = reason

            # History first, then the terminal snapshot: the newest history frame
            # and the terminal observation must come from the same instant.
            obs_lib.push_history(self.history, self.obs, np.array([index]))
            if self.dones[index]:
                # Value bootstrap must use the state *before* the reset.
                self.terminal_obs[index] = self.privileged[index]
                self.terminal_history[index] = obs_lib.flatten_history(self.history[index])

            # Command resampling runs on the episode clock, never for an env
            # that is about to be reset (its command is drawn in ``_reset_env``).
            if not self.dones[index]:
                self._resample_expired_commands(index)

        infos["time_outs"] = self.time_outs.copy()
        infos["terminations"] = terminations
        infos["termination_reason"] = list(self.termination_reason)
        infos["episode_returns"] = self.episode_returns.copy()
        infos["step_stats"] = {key: value.copy() for key, value in self.step_stats.items()}
        return rewards, infos

    # ------------------------------------------------------------------ helpers
    @property
    def observation_space_shape(self) -> tuple[int, ...]:
        return (self.num_obs,)

    def close(self) -> None:
        self.data.clear()


class _SlotView:
    """Minimal adapter so ``spring_geometry`` can run on one env slot."""

    def __init__(self, single: FiftyCpuEnv, data: mujoco.MjData):
        self.model = single.model
        self.data = data
        self.xml_path = single.xml_path
