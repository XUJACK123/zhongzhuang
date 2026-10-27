#!/usr/bin/env python3
"""Standalone Isaac Gym task adapter for the generated `fifty_training` model.

Modes
-----
--validate-only   Dependency-free structural / kinematic check.  Does not
                  import Isaac Gym or torch.
--smoke-test      Load one environment with all randomization disabled and run
                  100 zero-action steps.  Prints DOF order, body count, masses,
                  contact indices, torque limits and a finiteness verdict.
--joint-test      Fixed base, one environment.  Sends a small command to every
                  joint in turn and records the resulting motion, so that the
                  index -> joint -> physical direction mapping is verified
                  before any training run.
(default)         Train with the Fudan PPO implementation.  The reference repo
                  is never modified; the task is registered at runtime as
                  ``fifty_wheel_legged``.

The model, configuration and this script are copied into the run directory so
that a checkpoint can always be traced back to the exact URDF that produced it.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
PARAMS = json.loads((PACKAGE / "config" / "model_parameters.json").read_text(encoding="utf-8"))
URDF = PACKAGE / "urdf" / "fifty_training.urdf"
TASK = "fifty_wheel_legged"

# Provisional: both leg motors sit in the chassis (five-bar parallel leg), so
# the virtual knee has almost no rotor inertia in this equivalent model.  Fill
# in the reflected inertia once the motor / gearbox data is available.
DOF_ARMATURE = {"f0": 0.0, "f1": 0.0, "wheel": 0.0}


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--fudan-root", type=Path,
        default=PACKAGE.parent.parent / "ref_document/fudan_rl_wheel_leg-main")
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--smoke-test", action="store_true")
    parser.add_argument("--joint-test", action="store_true")
    local, remaining = parser.parse_known_args()

    if local.validate_only:
        sys.path.insert(0, str(PACKAGE / "integration"))
        from validate_model import main as validate_main
        return validate_main()

    plane = local.fudan_root.resolve() / "plane"
    if not (plane / "wheel_legged_gym/envs/base/legged_robot.py").is_file():
        raise SystemExit("Pass --fudan-root pointing at fudan_rl_wheel_leg-main.")
    sys.path.insert(0, str(plane))
    sys.path.insert(0, str(plane / "rsl_rl"))
    sys.argv = [sys.argv[0]] + remaining

    import isaacgym  # noqa: F401  (must precede torch)
    import torch
    from wheel_legged_gym.envs.base.legged_robot import LeggedRobot
    from wheel_legged_gym.envs.wheel_legged.wheel_legged_config import (
        WheelLeggedCfg, WheelLeggedCfgPPO)
    from wheel_legged_gym.utils import get_args, task_registry
    from wheel_legged_gym.utils.helpers import class_to_dict

    stance = PARAMS["spawn_height_m"] - 0.010

    class FiftyCfg(WheelLeggedCfg):
        class env(WheelLeggedCfg.env):
            num_envs = 4096

        class init_state(WheelLeggedCfg.init_state):
            pos = [0.0, 0.0, PARAMS["spawn_height_m"]]
            rot = [0.0, 0.0, 0.0, 1.0]
            default_joint_angles = dict(PARAMS["default_joint_angles_rad"])

        class control(WheelLeggedCfg.control):
            pos_action_scale = 0.5
            vel_action_scale = 10.0
            stiffness = {"f0": 20.0, "f1": 20.0, "wheel": 0.0}
            damping = {"f0": 1.0, "f1": 1.0, "wheel": 0.2}

        class asset(WheelLeggedCfg.asset):
            file = str(URDF)
            name = "fifty"
            foot_name = "wheel"
            l1 = PARAMS["leg_dimensions"]["l"]["thigh_planar_m"]
            l2 = PARAMS["leg_dimensions"]["l"]["shin_planar_m"]
            offset = 0.0
            dof_armature = dict(DOF_ARMATURE)
            replace_cylinder_with_capsule = False
            flip_visual_attachments = False
            fix_base_link = False
            self_collisions = 1
            collapse_fixed_joints = False

        class commands(WheelLeggedCfg.commands):
            curriculum = False

            class ranges(WheelLeggedCfg.commands.ranges):
                lin_vel_x = [-0.3, 0.3]
                ang_vel_yaw = [-0.4, 0.4]
                height = [stance - 0.03, stance + 0.01]

        class rewards(WheelLeggedCfg.rewards):
            base_height_target = stance

    class FiftyCfgPPO(WheelLeggedCfgPPO):
        class runner(WheelLeggedCfgPPO.runner):
            experiment_name = "fifty_wheel_legged"
            run_name = "six_dof"
            resume = False

    class FiftyRobot(LeggedRobot):
        def _create_envs(self):
            super()._create_envs()
            expected = PARAMS["expected_runtime_dof_order"]
            if list(self.dof_names) != expected:
                raise RuntimeError(
                    "Importer DOF order mismatch; refusing to train with silently "
                    f"misrouted actions. expected={expected} actual={list(self.dof_names)}")
            if self.num_bodies != 7:
                raise RuntimeError(f"Expected 7 rigid bodies, got {self.num_bodies}.")
            if int(self.feet_indices.numel()) != 2:
                raise RuntimeError("Both wheel bodies must be identified by foot_name='wheel'.")
            props = self.gym.get_actor_rigid_body_properties(self.sim, self.envs[0])
            mass = sum(p.mass for p in props)
            self.fifty_mass = mass
            self.fifty_body_names = list(self.gym.get_actor_rigid_body_names(self.sim, self.envs[0]))
            if abs(mass - PARAMS["source_mass_kg"]) > 1e-3:
                raise RuntimeError(
                    "Loaded mass %.6f kg differs from the URDF %.6f kg; Isaac Gym is not "
                    "using the URDF inertials (check asset density / override flags)."
                    % (mass, PARAMS["source_mass_kg"]))

    cfg, train_cfg = FiftyCfg(), FiftyCfgPPO()
    args = get_args()
    args.task = TASK

    if local.smoke_test or local.joint_test:
        args.num_envs = 1
        cfg.noise.add_noise = False
        for name in vars(type(cfg.domain_rand)):
            if name.startswith("randomize_"):
                setattr(cfg.domain_rand, name, False)
        for name in ["randomize_friction", "randomize_restitution", "randomize_base_mass",
                     "randomize_inertia", "randomize_base_com", "randomize_Kp",
                     "randomize_Kd", "randomize_motor_torque", "randomize_default_dof_pos",
                     "randomize_action_delay", "push_robots"]:
            setattr(cfg.domain_rand, name, False)
        cfg.commands.ranges.lin_vel_x = [0.0, 0.0]
        cfg.commands.ranges.ang_vel_yaw = [0.0, 0.0]
        cfg.commands.ranges.height = [stance, stance]
        if local.joint_test:
            cfg.asset.fix_base_link = True

    task_registry.register(TASK, FiftyRobot, cfg, train_cfg)
    env, env_cfg = task_registry.make_env(name=TASK, args=args)
    print("DOF order        :", list(env.dof_names))
    print("Body names       :", env.fifty_body_names)
    print("Loaded mass [kg] :", round(env.fifty_mass, 9))
    print("Torque limits    :", env.torque_limits.tolist())
    print("Kp / Kd          :", env.p_gains[0].tolist(), env.d_gains[0].tolist())
    print("Feet indices     :", env.feet_indices.tolist())

    if local.smoke_test:
        zeros = torch.zeros((env.num_envs, 6), device=env.device)
        initial_z = float(env.root_states[0, 2])
        resets = 0
        for _ in range(100):
            obs, _, rew, dones, _ = env.step(zeros)
            if not torch.isfinite(env.root_states).all() or not torch.isfinite(env.dof_state).all():
                raise RuntimeError("Non-finite physics state during the smoke test.")
            if not torch.isfinite(obs).all() or not torch.isfinite(rew).all():
                raise RuntimeError("Non-finite observation/reward during the smoke test.")
            resets += int(dones.sum().item())
        contact = env.contact_forces[0, env.feet_indices, :]
        print(json.dumps({
            "smoke_steps": 100,
            "resets": resets,
            "finite_states": True,
            "initial_base_height_m": initial_z,
            "final_base_height_m": float(env.root_states[0, 2]),
            "wheel_contact_force_norm": [round(float(x), 2) for x in contact.norm(dim=-1)],
            "expected_weight_n": round(env.fifty_mass * 9.81, 2),
            "stable_standing_verified": False,
        }, indent=2))
        env.gym.destroy_sim(env.sim)
        return 0

    if local.joint_test:
        zeros = torch.zeros((env.num_envs, 6), device=env.device)
        rows = []
        for idx, name in enumerate(env.dof_names):
            env.reset_idx(torch.arange(env.num_envs, device=env.device))
            for _ in range(20):                    # settle back at the default pose
                env.step(zeros)
            q0 = env.dof_pos[0].clone()
            qd0 = env.dof_vel[0].clone()
            action = zeros.clone()
            action[0, idx] = 0.2 if "wheel" not in name else 1.0
            for _ in range(50):                    # 0.5 s
                env.step(action)
            dq = float(env.dof_pos[0, idx] - q0[idx])
            dqd = float(env.dof_vel[0, idx] - qd0[idx])
            rows.append({"index": idx, "joint": name,
                         "delta_q": round(dq, 4), "delta_qd": round(dqd, 4),
                         "sign_ok": (dq > 0) if "wheel" not in name else (dqd > 0)})
        print(json.dumps({"joint_test": rows,
                          "note": "wheel rows compare delta_qd, leg rows compare delta_q"},
                         indent=2))
        for idx, name in enumerate(env.dof_names):
            env.reset_idx(torch.arange(env.num_envs, device=env.device))
            for _ in range(20):
                env.step(zeros)
            action = zeros.clone()
            action[0, idx] = -0.2 if "wheel" not in name else -1.0
            for _ in range(50):
                env.step(action)
            print(f"negative command {name:14s} -> dof_pos={env.dof_pos[0, idx].item():+.4f} "
                  f"dof_vel={env.dof_vel[0, idx].item():+.4f}")
        env.gym.destroy_sim(env.sim)
        return 0

    runner, train_cfg = task_registry.make_alg_runner(env=env, name=TASK, args=args)
    log_dir = Path(task_registry.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    for name, obj in [("env_cfg.json", env_cfg), ("train_cfg.json", train_cfg)]:
        (log_dir / name).write_text(json.dumps(class_to_dict(obj), indent=2, default=str),
                                    encoding="utf-8")
    for path in [Path(__file__), URDF, PACKAGE / "config/model_parameters.json",
                 PACKAGE / "config/build_report.json",
                 PACKAGE / "docs/CONVERSION_NOTES.md"]:
        shutil.copy2(path, log_dir / path.name)
    runner.learn(num_learning_iterations=train_cfg.runner.max_iterations,
                 init_at_random_ep_len=True)
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
