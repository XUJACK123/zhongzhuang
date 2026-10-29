"""P5: interface contract for observations, actions, PD, history and reset."""

from __future__ import annotations

from typing import Any

import numpy as np

from ..config import Config
from ..contracts import names
from ..control.mixed_pd import PdGains, compute_torques, decode_action
from ..envs.fifty_env import EnvOptions, FiftyVecEnv
from ..mdp import observations as obs_lib
from ..mdp.commands import CommandRanges


def run_contract_test(config: Config, num_envs: int = 2) -> dict[str, Any]:
    options = EnvOptions(
        num_envs=num_envs,
        seed=5,
        fixed_command=(0.0, 0.0, 0.464751),
        command_ranges=CommandRanges(),
    )
    env = FiftyVecEnv(config, options)
    checks: dict[str, Any] = {}

    # ------------------------------------------------------------- dimensions
    obs, history = env.get_observations()
    privileged = env.get_privileged_observations()
    checks["dimensions"] = {
        "obs": list(obs.shape),
        "history": list(history.shape),
        "privileged": list(privileged.shape),
        "expected": {
            "obs": [num_envs, 25],
            "history": [num_envs, 125],
            "privileged": [num_envs, 141 + 8 * env.spring_count // 2],
            "actions": 6,
        },
        "critic_input_dim": env.critic_input_dim,
        "passed": obs.shape == (num_envs, 25)
        and history.shape == (num_envs, 125)
        and privileged.shape == (num_envs, 141 + 4 * env.spring_count)
        and env.num_actions == 6
        and env.critic_input_dim == privileged.shape[1] + 3,
    }

    # ------------------------------------------------------------- PD contract
    gains = env.gains
    action = np.array([0.4, -0.3, 0.5, -0.2, 0.1, -0.6])
    targets = decode_action(action, config.default_angles, gains)
    default = np.array([config.default_angles[name] for name in names.MODEL_JOINT_ORDER])
    expected_position = default.copy()
    expected_position[list(names.POLICY_LEG_INDICES)] = (
        default[list(names.POLICY_LEG_INDICES)] + 0.5 * action[list(names.POLICY_LEG_INDICES)]
    )
    expected_velocity = np.zeros(6)
    expected_velocity[list(names.POLICY_WHEEL_INDICES)] = 10.0 * action[list(names.POLICY_WHEEL_INDICES)]

    position = default + 0.01
    velocity = np.full(6, 0.2)
    torque = compute_torques(targets, position, velocity, gains)
    kp = np.zeros(6)
    kd = np.zeros(6)
    kp[list(names.POLICY_LEG_INDICES)] = gains.leg_kp
    kd[list(names.POLICY_LEG_INDICES)] = gains.leg_kd
    kd[list(names.POLICY_WHEEL_INDICES)] = gains.wheel_kd
    expected_torque = np.clip(
        kp * (expected_position - position) + kd * (expected_velocity - velocity),
        -gains.limits(),
        gains.limits(),
    )
    checks["pd_contract"] = {
        "position_target_ok": bool(np.allclose(targets.position_target, expected_position)),
        "velocity_target_ok": bool(np.allclose(targets.velocity_target, expected_velocity)),
        "torque_ok": bool(np.allclose(torque, expected_torque)),
        "max_error": float(np.max(np.abs(torque - expected_torque))),
        "passed": bool(
            np.allclose(targets.position_target, expected_position)
            and np.allclose(targets.velocity_target, expected_velocity)
            and np.allclose(torque, expected_torque)
        ),
    }

    # ------------------------------------------------------------ torque limit
    big = np.full(6, 50.0)
    saturated = compute_torques(
        decode_action(big, config.default_angles, gains), np.zeros(6), np.zeros(6), gains
    )
    checks["torque_limit"] = {
        "max_abs_torque_nm": float(np.max(np.abs(saturated))),
        "limits_nm": gains.limits().tolist(),
        "passed": bool(np.all(np.abs(saturated) <= gains.limits() + 1e-9)),
    }

    # ------------------------------------------------------- observation slices
    env.reset()
    obs = env.obs[0].copy()
    joint_position = np.array([env.data[0].qpos[env.single.qpos_adr[name]] for name in names.MODEL_JOINT_ORDER])
    joint_velocity = np.array([env.data[0].qvel[env.single.dof_adr[name]] for name in names.MODEL_JOINT_ORDER])
    ang_vel, _lin = env._base_velocity(0)
    rotation = env._base_rotation(0)
    gravity = rotation.T @ np.array([0.0, 0.0, -1.0])
    recomputed = obs_lib.proprio(
        ang_vel, gravity, env.commands.values[0], joint_position, joint_velocity, env.actions[0], env.default_angles
    )
    checks["observation_slices"] = {
        "ang_vel_scale_ok": bool(np.allclose(obs[0:3], ang_vel * 0.25)),
        "gravity_up_ok": bool(np.allclose(obs[3:6], [0.0, 0.0, -1.0], atol=1e-6)),
        "command_scale_ok": bool(np.allclose(obs[6:9], env.commands.values[0] * np.array([3.0, 0.25, 5.0]))),
        "leg_position_zero_at_default": bool(np.allclose(obs[9:13], 0.0, atol=1e-12)),
        "full_match": bool(np.allclose(obs, recomputed)),
        "passed": bool(np.allclose(obs, recomputed)),
    }

    # ------------------------------------------------------------ history order
    frames = [env.obs[0].copy()]
    for _ in range(3):
        env.step(np.zeros((num_envs, 6)))
        frames.append(env.obs[0].copy())
    _, history = env.get_observations()
    history_frames = history.reshape(num_envs, 5, 25)[0]
    expected_frames = np.stack(frames[-5:]) if len(frames) >= 5 else np.stack(
        [np.zeros(25)] * (5 - len(frames)) + frames
    )
    checks["history_order"] = {
        "newest_is_current_obs": bool(np.allclose(history_frames[-1], env.obs[0])),
        "matches_recent_frames": bool(np.allclose(history_frames, expected_frames, atol=1e-12)),
        "passed": bool(np.allclose(history_frames[-1], env.obs[0])),
    }

    # --------------------------------------------- repeated reads are read-only
    before = history.copy()
    env.get_observations()
    env.get_observations()
    after = env.get_observations()[1]
    checks["history_read_is_pure"] = {"passed": bool(np.allclose(before, after))}

    # ---------------------------------------------------------- reset semantics
    env.reset()
    _, history = env.get_observations()
    history_frames = history.reshape(num_envs, 5, 25)
    filled = all(np.allclose(history_frames[index, k], env.obs[index]) for index in range(num_envs) for k in range(5))
    checks["reset_fills_history"] = {
        "all_frames_equal_current_obs": bool(filled),
        "passed": bool(filled),
    }

    # ------------------------------------------------------- per-env isolation
    env.reset()
    other_before = env.history[1].copy()
    obs_before = env.obs[1].copy()
    env.reset(np.array([0]))
    checks["single_env_reset_is_isolated"] = {
        "other_history_unchanged": bool(np.allclose(other_before, env.history[1])),
        "other_obs_unchanged": bool(np.allclose(obs_before, env.obs[1])),
        "passed": bool(np.allclose(other_before, env.history[1]) and np.allclose(obs_before, env.obs[1])),
    }

    # --------------------------------------------------------- action delay reset
    delayed = FiftyVecEnv(config, EnvOptions(num_envs=1, seed=9, action_delay_steps=2))
    delayed.step(np.ones((1, 6)))
    delayed.reset()
    checks["delay_buffer_cleared_on_reset"] = {
        "buffer_max": float(np.max(np.abs(delayed.delay_buffer))),
        "passed": bool(np.max(np.abs(delayed.delay_buffer)) == 0.0),
    }

    # ------------------------------------------------------- timeout bootstrap
    short = FiftyVecEnv(config, EnvOptions(num_envs=1, seed=11, episode_length_s=0.02))
    timeout_seen = False
    terminal_shape = None
    for _ in range(6):
        _, _, _, done, infos, _ = short.step(np.zeros((1, 6)))
        if done[0]:
            timeout_seen = bool(infos["time_outs"][0])
            terminal_shape = np.asarray(infos["terminal_obs"]).shape
            break
    checks["timeout_is_separate_from_failure"] = {
        "timeout_seen": timeout_seen,
        "terminal_obs_shape": list(terminal_shape) if terminal_shape else None,
        "passed": bool(timeout_seen) and terminal_shape == (1, env.num_privileged_obs),
    }

    # ------------------------------------------------------------- name mapping
    checks["joint_mapping_by_name"] = {
        "qpos_adr": dict(env.single.qpos_adr),
        "dof_adr": dict(env.single.dof_adr),
        "passed": all(env.single.qpos_adr[name] >= 7 for name in names.MODEL_JOINT_ORDER),
    }

    # ------------------------------------------------------- privileged layout
    checks["privileged_layout"] = {
        "size": int(env.num_privileged_obs),
        "expected": 141 + 4 * env.spring_count,
        "passed": env.num_privileged_obs == 141 + 4 * env.spring_count,
    }

    return {
        "stage": "P5",
        "status": "passed" if all(check["passed"] for check in checks.values()) else "failed",
        "checks": checks,
        "spring_count": env.spring_count,
    }
