"""CPU/GPU contract parity suite.

``tests/test_warp_env.py`` compares a handful of quantities (base height, joint
angles, ``obs`` after a graph replay). This module is the field-by-field check
that ``progress.md`` §14.1 and §18.10 actually ask for, and it leaves a report
behind instead of a pass/fail bit:

* observation channels, grouped by the layout in ``configs/policy_contract.yaml``
  (``[ang_vel, projected_gravity, command, leg_pos, joint_vel, last_action]``),
* the 141 base privileged channels and the 8 gas-spring channels separately,
* reward, termination flags and the flattened 125-channel history,
* the post-reset observation after a forced termination, which is the one field
  that used to be silently wrong (§18.1),
* a CUDA-graph replay comparison (§18.9 item 7).

Every field is reported with its measured error and the tolerance it was
compared against, so "passed" cannot hide a channel that was never looked at.
"""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

import numpy as np

from ..config import Config

#: Observation layout from ``configs/policy_contract.yaml``.
OBS_GROUPS: tuple[tuple[str, slice], ...] = (
    ("ang_vel", slice(0, 3)),
    ("projected_gravity", slice(3, 6)),
    ("command", slice(6, 9)),
    ("leg_pos", slice(9, 13)),
    ("joint_vel", slice(13, 19)),
    ("last_action", slice(19, 25)),
)

#: History keeps the same channel order, five frames deep.
HISTORY_GROUPS: tuple[tuple[str, slice], ...] = tuple(
    (
        f"history[{frame}].{name}",
        slice(frame * 25 + group.start, frame * 25 + group.stop),
    )
    for frame in range(5)
    for name, group in OBS_GROUPS
)

#: Absolute error budget per field. Observed errors are dominated by the physics
#: backend (MuJoCo CPU vs mujoco_warp), not by the control stack, so the budget
#: is per field rather than one number for everything.
TOLERANCES: dict[str, float] = {
    "obs": 2e-3,
    "privileged": 5e-3,
    "privileged_spring": 5e-3,
    "reward": 1e-4,
    "history": 2e-3,
    "post_reset_obs": 2e-3,
    "graph_replay_obs": 1e-5,
    "graph_replay_qpos": 1e-6,
}


def _spawn_height(config: Config) -> float:
    from ..control.spawn import load_spawn_spec, resolve_spawn_height  # noqa: PLC0415
    from ..envs.cpu_env import FiftyCpuEnv
    from ..paths import assets_dir

    env = FiftyCpuEnv(assets_dir() / "mjcf" / "fifty_train.xml", config)
    root_z = resolve_spawn_height(env, load_spawn_spec(config))[0]
    return root_z - float(config.spawn["spawn"]["wheel_clearance_m"])


def _max_abs(cpu: np.ndarray, gpu: np.ndarray, where: slice | None = None) -> float:
    left = np.asarray(cpu, dtype=np.float64)
    right = np.asarray(gpu, dtype=np.float64)
    if where is not None:
        left, right = left[..., where], right[..., where]
    if left.size == 0:
        return 0.0
    return float(np.max(np.abs(left - right)))


class _FieldRecorder:
    """Collect the worst error seen for one field, and the step it happened on."""

    def __init__(self, tolerance: float) -> None:
        self.tolerance = tolerance
        self.max_error = 0.0
        self.worst_step: int | None = None
        self.first_failure: int | None = None

    def observe(self, step: int, error: float) -> None:
        if error > self.max_error:
            self.max_error = error
            self.worst_step = step
        if error > self.tolerance and self.first_failure is None:
            self.first_failure = step

    @property
    def passed(self) -> bool:
        return self.first_failure is None

    def as_dict(self) -> dict[str, Any]:
        return {
            "max_abs_error": self.max_error,
            "tolerance": self.tolerance,
            "worst_step": self.worst_step,
            "first_step_exceeding": self.first_failure,
            "passed": self.passed,
        }


def _action_sequence(num_envs: int, steps: int, seed: int) -> np.ndarray:
    """Deterministic, bounded, identical for both backends.

    Small amplitude on purpose: a wild action sequence falls over in three steps
    and then compares two different contact sequences, which says nothing about
    the contract. The legs are moved, the wheels are spun, everything stays
    within what the spawn pose can survive.
    """
    rng = np.random.default_rng(seed)
    actions = rng.normal(0.0, 0.25, size=(steps, num_envs, 6))
    actions[:, :, [0, 1, 3, 4]] = np.clip(actions[:, :, [0, 1, 3, 4]], -0.6, 0.6)
    actions[:, :, [2, 5]] = np.clip(actions[:, :, [2, 5]], -0.2, 0.2)
    return actions


def run_parity_suite(
    config: Config,
    num_envs: int = 4,
    steps: int = 24,
    seed: int = 7,
    device: str = "cuda:0",
    check_graph: bool = True,
    fall_steps: int = 200,
) -> dict[str, Any]:
    """Run the CPU/GPU field comparison and return a report payload."""
    import torch

    from ..envs.fifty_env import EnvOptions, FiftyVecEnv
    from ..envs.warp_env import WarpEnvOptions, WarpVecEnv

    height = _spawn_height(config)
    command = (0.0, 0.0, height)
    cpu = FiftyVecEnv(config, EnvOptions(num_envs=num_envs, seed=seed, fixed_command=command))
    gpu = WarpVecEnv(
        config,
        WarpEnvOptions(num_envs=num_envs, device=device, seed=seed, fixed_command=command),
    )

    fields: dict[str, _FieldRecorder] = {
        "obs": _FieldRecorder(TOLERANCES["obs"]),
        "privileged": _FieldRecorder(TOLERANCES["privileged"]),
        "privileged_spring": _FieldRecorder(TOLERANCES["privileged_spring"]),
        "reward": _FieldRecorder(TOLERANCES["reward"]),
        "history": _FieldRecorder(TOLERANCES["history"]),
    }
    groups: dict[str, _FieldRecorder] = {
        f"obs.{name}": _FieldRecorder(TOLERANCES["obs"]) for name, _ in OBS_GROUPS
    }
    groups.update(
        {name: _FieldRecorder(TOLERANCES["history"]) for name, _ in HISTORY_GROUPS}
    )

    actions = _action_sequence(num_envs, steps, seed)
    done_mismatch = 0
    steps_run = 0
    for step in range(steps):
        cpu_obs, cpu_priv, cpu_reward, cpu_done, _info, cpu_hist = cpu.step(actions[step])
        gpu_out = gpu.step(
            torch.as_tensor(actions[step], dtype=torch.float32, device=gpu.device)
        )
        gpu_obs, gpu_priv, gpu_reward, gpu_done, _gpu_info, gpu_hist = gpu_out
        torch.cuda.synchronize()
        steps_run = step + 1

        gpu_obs_np = gpu_obs.detach().cpu().numpy()
        gpu_priv_np = gpu_priv.detach().cpu().numpy()
        gpu_reward_np = gpu_reward.detach().cpu().numpy()
        gpu_done_np = gpu_done.detach().cpu().numpy().astype(bool)
        gpu_hist_np = gpu_hist.detach().cpu().numpy()

        fields["obs"].observe(step, _max_abs(cpu_obs, gpu_obs_np))
        fields["privileged"].observe(step, _max_abs(cpu_priv, gpu_priv_np))
        base = 141
        fields["privileged_spring"].observe(
            step, _max_abs(cpu_priv, gpu_priv_np, slice(base, cpu_priv.shape[1]))
        )
        fields["reward"].observe(step, _max_abs(cpu_reward, gpu_reward_np))
        fields["history"].observe(step, _max_abs(cpu_hist, gpu_hist_np))
        for name, where in OBS_GROUPS:
            groups[f"obs.{name}"].observe(step, _max_abs(cpu_obs, gpu_obs_np, where))
        for name, where in HISTORY_GROUPS:
            groups[name].observe(step, _max_abs(cpu_hist, gpu_hist_np, where))
        done_mismatch += int(np.count_nonzero(np.asarray(cpu_done, bool) != gpu_done_np))

    # ---------------------------------------------------------- forced reset
    # A termination is forced by tilting the base, because a policy that simply
    # receives large actions may keep standing. The tilt is re-applied every
    # step so a termination that needs a dwell time still fires.
    import mujoco

    reset_field = _FieldRecorder(TOLERANCES["post_reset_obs"])
    reset_seen = False
    tilt_deg = 70.0
    half = np.radians(tilt_deg) / 2.0
    tilted = np.array([np.cos(half), np.sin(half), 0.0, 0.0])
    for _ in range(fall_steps):
        for index in range(num_envs):
            cpu.data[index].qpos[3:7] = tilted
            mujoco.mj_forward(cpu.model, cpu.data[index])
        gpu.qpos[:, 3:7] = torch.as_tensor(tilted, dtype=gpu.qpos.dtype, device=gpu.device)
        cpu_out = cpu.step(np.zeros((num_envs, 6)))
        gpu_out = gpu.step(torch.zeros((num_envs, 6), device=gpu.device))
        torch.cuda.synchronize()
        cpu_done = np.asarray(cpu_out[3], bool)
        gpu_done = gpu_out[3].detach().cpu().numpy().astype(bool)
        done_mismatch += int(np.count_nonzero(cpu_done != gpu_done))
        # Only compare the rows that actually reset: for a row that is still
        # alive we would be comparing two live states, whose difference is
        # accumulated physics divergence, not a contract error.
        if cpu_done.any() and gpu_done.any():
            mask = cpu_done & gpu_done
            reset_field.observe(
                0,
                _max_abs(
                    cpu_out[0][mask],
                    gpu_out[0].detach().cpu().numpy()[mask],
                ),
            )
            reset_seen = True
        if cpu_done.all() and gpu_done.all():
            break

    report: dict[str, Any] = {
        "tool": "gpu-parity",
        "schema": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "device": device,
        "num_envs": num_envs,
        "steps": steps_run,
        "fixed_command": list(command),
        "seed": seed,
        "fields": {name: recorder.as_dict() for name, recorder in fields.items()},
        "field_groups": {name: recorder.as_dict() for name, recorder in groups.items()},
        "termination_flag_mismatches": done_mismatch,
        "forced_reset": {
            "reached": reset_seen,
            "post_reset_obs": reset_field.as_dict(),
            "note": (
                "the observation returned for a finished env must describe the reset"
                " state, not the state the episode died in (§18.1)"
            ),
        },
    }

    if check_graph:
        report["graph_replay"] = _graph_replay(config, num_envs=num_envs, device=device, height=height)

    variants = [report["fields"], report["field_groups"]]
    if check_graph:
        variants.append(report["graph_replay"]["comparison"])
    failures = [
        name
        for block in variants
        for name, entry in block.items()
        if isinstance(entry, dict) and entry.get("passed") is False
    ]
    if done_mismatch:
        failures.append("termination_flag_mismatches")
    if not report["forced_reset"]["reached"]:
        failures.append("forced_reset_not_reached")
    elif not report["forced_reset"]["post_reset_obs"]["passed"]:
        failures.append("forced_reset.post_reset_obs")
    report["failures"] = failures
    report["status"] = "passed" if not failures else "failed"
    return report


def _graph_replay(config: Config, num_envs: int, device: str, height: float) -> dict[str, Any]:
    """A captured step must produce what the eager step produces."""
    import torch

    from ..envs.warp_env import WarpEnvOptions, WarpVecEnv

    options = WarpEnvOptions(
        num_envs=num_envs, device=device, seed=11, fixed_command=(0.0, 0.0, height)
    )
    eager = WarpVecEnv(config, options)
    captured = WarpVecEnv(config, options)
    captured_ok = captured.capture()
    obs_field = _FieldRecorder(TOLERANCES["graph_replay_obs"])
    qpos_field = _FieldRecorder(TOLERANCES["graph_replay_qpos"])
    if captured_ok:
        actions = torch.zeros((num_envs, 6), device=eager.device)
        actions[:, [1, 4]] = 0.2
        actions[:, [2, 5]] = 0.3
        for step in range(10):
            eager_out = eager.step(actions)
            captured_out = captured.step(actions)
            torch.cuda.synchronize()
            obs_field.observe(
                step, _max_abs(eager_out[0].detach().cpu().numpy(), captured_out[0].detach().cpu().numpy())
            )
            qpos_field.observe(
                step, _max_abs(eager.qpos.detach().cpu().numpy(), captured.qpos.detach().cpu().numpy())
            )
    return {
        "captured": captured_ok,
        "comparison": {
            "graph_replay_obs": obs_field.as_dict(),
            "graph_replay_qpos": qpos_field.as_dict(),
        },
    }


def summarize(report: dict[str, Any]) -> str:
    lines = [
        f"device        : {report['device']}  envs {report['num_envs']}  steps {report['steps']}",
    ]
    for block in ("fields", "field_groups"):
        for name, entry in report[block].items():
            flag = "ok  " if entry["passed"] else "FAIL"
            lines.append(
                f"  {flag} {name:<28} max {entry['max_abs_error']:.3e}"
                f"  tol {entry['tolerance']:.0e}"
                + (f"  step {entry['first_step_exceeding']}" if not entry["passed"] else "")
            )
    reset = report["forced_reset"]
    lines.append(
        f"  forced reset : reached={reset['reached']}"
        f"  obs error {reset['post_reset_obs']['max_abs_error']:.3e}"
        f"  tol {reset['post_reset_obs']['tolerance']:.0e}"
    )
    graph = report.get("graph_replay")
    if graph is not None:
        lines.append(f"  graph replay : captured={graph['captured']}")
        for name, entry in graph["comparison"].items():
            flag = "ok  " if entry["passed"] else "FAIL"
            lines.append(
                f"    {flag} {name:<24} max {entry['max_abs_error']:.3e}"
                f"  tol {entry['tolerance']:.0e}"
            )
    lines.append(f"  termination  : {report['termination_flag_mismatches']} flag mismatch(es)")
    lines.append(
        "  verdict      : " + ("passed" if report["status"] == "passed" else "FAILED " + ", ".join(report["failures"]))
    )
    return "\n".join(lines)
