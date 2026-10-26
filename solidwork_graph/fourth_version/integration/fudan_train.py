"""Standalone task adapter for the checked local Fudan plane training tree.

Run --validate-only without Isaac Gym, or --smoke-test in the Isaac Gym env.
Does not edit the reference repository or replace its wheel_legged task.
"""
import argparse
import json
import math
import shutil
import sys
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
PARAMS = json.loads((PACKAGE / 'config/model_parameters.json').read_text(encoding='utf-8'))


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--fudan-root', type=Path, default=PACKAGE.parent.parent / 'ref_document/fudan_rl_wheel_leg-main')
    parser.add_argument('--smoke-test', action='store_true')
    parser.add_argument('--validate-only', action='store_true')
    local, remaining = parser.parse_known_args()
    if local.validate_only:
        from validate_model import validate
        print(json.dumps(validate(), ensure_ascii=False, indent=2))
        return

    plane = local.fudan_root.resolve() / 'plane'
    if not (plane / 'wheel_legged_gym/envs/base/legged_robot.py').is_file():
        raise SystemExit('Pass --fudan-root with the fudan_rl_wheel_leg-main directory.')
    sys.path.insert(0, str(plane))
    sys.path.insert(0, str(plane / 'rsl_rl'))
    sys.argv = [sys.argv[0]] + remaining

    # Isaac Gym requires import before torch.
    import isaacgym  # noqa: F401
    import torch
    from wheel_legged_gym.envs.base.legged_robot import LeggedRobot
    from wheel_legged_gym.envs.wheel_legged.wheel_legged_config import WheelLeggedCfg, WheelLeggedCfgPPO
    from wheel_legged_gym.utils import get_args, task_registry
    from wheel_legged_gym.utils.helpers import class_to_dict

    class FourthVersionCfg(WheelLeggedCfg):
        class env(WheelLeggedCfg.env):
            num_envs = 1024

        class init_state(WheelLeggedCfg.init_state):
            pos = [0.0, 0.0, PARAMS['spawn_height_m']]
            rot = [0.0, 0.0, 0.0, 1.0]
            default_joint_angles = dict(PARAMS['default_joint_angles_rad'])

        class asset(WheelLeggedCfg.asset):
            file = str(PACKAGE / 'urdf/fourth_version.urdf')
            name = 'fourth_version'
            foot_name = 'wheel'
            l1 = PARAMS['leg_dimensions']['l']['thigh_planar_m']
            l2 = PARAMS['leg_dimensions']['l']['shin_planar_m']
            offset = 0.0
            replace_cylinder_with_capsule = False
            flip_visual_attachments = False
            fix_base_link = False
            self_collisions = 1

        class commands(WheelLeggedCfg.commands):
            curriculum = False
            class ranges(WheelLeggedCfg.commands.ranges):
                lin_vel_x = [-0.3, 0.3]
                ang_vel_yaw = [-0.4, 0.4]
                height = [PARAMS['spawn_height_m'] - 0.03, PARAMS['spawn_height_m'] + 0.01]

    class FourthVersionCfgPPO(WheelLeggedCfgPPO):
        class runner(WheelLeggedCfgPPO.runner):
            experiment_name = 'fourth_version_proxy'
            run_name = 'six_dof'
            resume = False

    class FourthVersionRobot(LeggedRobot):
        def _create_envs(self):
            super()._create_envs()
            expected = PARAMS['expected_runtime_dof_order']
            if list(self.dof_names) != expected:
                raise RuntimeError('Importer DOF order mismatch; refuse silent action misrouting. '
                                   f'Expected {expected}, got {list(self.dof_names)}. '
                                   'Do not train until the importer/order mapping is corrected.')
            if self.num_bodies != 7:
                raise RuntimeError(f'The current Fudan observation layout expects 7 bodies, got {self.num_bodies}.')
            if int(self.feet_indices.numel()) != 2:
                raise RuntimeError('Both wheel contact bodies must be identified.')

    task_name = 'fourth_version_proxy'
    cfg, train_cfg = FourthVersionCfg(), FourthVersionCfgPPO()
    args = get_args()
    args.task = task_name
    if local.smoke_test:
        args.num_envs = 1
        cfg.noise.add_noise = False
        for name in vars(type(cfg.domain_rand)):
            if name.startswith('randomize_'):
                setattr(cfg.domain_rand, name, False)
        # The class inherits randomization fields, so also explicitly disable all
        # known flags in the inspected repository.
        for name in ['randomize_friction', 'randomize_restitution', 'randomize_base_mass',
                     'randomize_inertia', 'randomize_base_com', 'randomize_Kp',
                     'randomize_Kd', 'randomize_motor_torque', 'randomize_default_dof_pos',
                     'randomize_action_delay', 'push_robots']:
            setattr(cfg.domain_rand, name, False)
        cfg.commands.ranges.lin_vel_x = [0.0, 0.0]
        cfg.commands.ranges.ang_vel_yaw = [0.0, 0.0]
        h = PARAMS['spawn_height_m'] - 0.01
        cfg.commands.ranges.height = [h, h]

    task_registry.register(task_name, FourthVersionRobot, cfg, train_cfg)
    env, env_cfg = task_registry.make_env(name=task_name, args=args)
    print('Loaded DOF order:', list(env.dof_names))
    print('PD gains:', env.p_gains[0].tolist(), env.d_gains[0].tolist())
    if local.smoke_test:
        reset_count = 0
        for _ in range(100):
            result = env.step(torch.zeros((env.num_envs, 6), device=env.device))
            if not torch.isfinite(env.root_states).all() or not torch.isfinite(env.dof_state).all():
                raise RuntimeError('Non-finite physics state during smoke test.')
            if not torch.isfinite(result[0]).all() or not torch.isfinite(result[2]).all():
                raise RuntimeError('Non-finite observation/reward during smoke test.')
            reset_count += int(result[3].sum().item())
        print(json.dumps({'smoke_steps': 100, 'resets': reset_count,
                          'finite_states': True, 'stable_standing_verified': False}, indent=2))
        env.gym.destroy_sim(env.sim)
        return

    runner, train_cfg = task_registry.make_alg_runner(env=env, name=task_name, args=args)
    log_dir = Path(task_registry.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    # The upstream save_cfgs assumes a matching source folder for the task name.
    # Save our effective configuration and exact model here instead.
    for name, obj in [('env_cfg.json', env_cfg), ('train_cfg.json', train_cfg)]:
        (log_dir / name).write_text(json.dumps(class_to_dict(obj), indent=2, default=str), encoding='utf-8')
    for path in [Path(__file__), PACKAGE / 'urdf/fourth_version.urdf', PACKAGE / 'config/model_parameters.json', PACKAGE / 'MODEL_NOTES.md']:
        shutil.copy2(path, log_dir / path.name)
    runner.learn(num_learning_iterations=train_cfg.runner.max_iterations, init_at_random_ep_len=True)


if __name__ == '__main__':
    main()
