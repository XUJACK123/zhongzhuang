#!/usr/bin/env python3
"""Run bounded trajectory candidates through CPU MuJoCo and its C observer."""

import argparse
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path

import numpy as np
import torch

from fifty_mujoco.contracts.getup31 import Phase
from fifty_mujoco.envs.sample_env31_supported import SupportedGetupEnv31
from realdata_train import SamplePolicy31, digest, verified_artifact


def replay(task):
    profile, checkpoint, rise_checkpoint, params, ticks, gain, seed, index, destination, feedback, brake, *optional = task
    torch.set_num_threads(1)
    policy = SamplePolicy31(verified_artifact('model_3050.pt'), 'cpu')
    policy.load_state_dict(torch.load(checkpoint, map_location='cpu', weights_only=True)['model'])
    rise = None
    if rise_checkpoint:
        rise = SamplePolicy31(verified_artifact('model_3050.pt'), 'cpu')
        rise.load_state_dict(torch.load(rise_checkpoint, map_location='cpu', weights_only=True)['model'])
    follow = policy
    if optional and optional[0] is not None:
        follow = SamplePolicy31(verified_artifact('model_3050.pt'), 'cpu')
        follow.load_state_dict(torch.load(optional[0], map_location='cpu', weights_only=True)['model'])
    learner = None
    learner_mix = optional[3] if len(optional) > 3 else 0.
    if learner_mix:
        learner = SamplePolicy31(verified_artifact('model_3050.pt'), 'cpu')
        learner.load_state_dict(torch.load(optional[2], map_location='cpu', weights_only=True)['model'])
    env = SupportedGetupEnv31(policy.actor, config=profile, settle_seconds=1.)
    trace = {key: [] for key in ('qpos', 'qvel', 'metrics', 'phase', 'motor_command')}
    rows = {'observation': [], 'action': []}
    advance = env._advance

    def record(action, scale):
        phase = env.controller.phase.value
        observation, info, state = advance(action, scale)
        trace['qpos'].append(env.data.qpos.copy())
        trace['qvel'].append(env.data.qvel.copy())
        trace['metrics'].append([state.time_s, state.clearance_m, state.displacement_m,
                                 *state.roll_pitch_deg, state.forward_velocity_m_s,
                                 *env.wheel_load_n.values(), state.body_contact,
                                 env.support_peak_n, env.support_peak_nm])
        trace['phase'].append(phase)
        trace['motor_command'].append(env.env.fast_actuators.command.copy())
        return observation, info, state

    env._advance = record
    observation = env.reset(seed)
    env.env.fast_actuators.gain[[0, 1, 3, 4]] = gain
    switched = rise is None
    capture_x = None
    slew = optional[1] if len(optional) > 1 else 1.
    previous_action = np.zeros(6, dtype=np.float32)
    for step in range(env.max_steps):
        if step // ticks < len(params):
            hip, knee, wheel = params[step // ticks]
            action = np.array([hip, knee, 2*wheel, -hip, -knee, 2*wheel], dtype=np.float32)
            if feedback:
                correction = feedback*(4*observation[1] + 4*observation[3])
                action[[2, 5]] = np.clip(action[[2, 5]] + correction, -2., 2.)
            previous_action += slew*(action-previous_action)
            action = previous_action.copy()
        else:
            state = env.last_sample
            switched |= state.clearance_m >= .12 and max(abs(x) for x in state.roll_pitch_deg) < 15
            actor = follow.actor if switched else rise.actor
            teacher_obs = np.array(observation, dtype=np.float32, copy=True)
            if switched and brake:
                if capture_x is None:
                    capture_x = min(float(env.data.qpos[0]) + .02, float(env.origin[0]) + .20)
                teacher_obs[25] = np.clip(5*(capture_x-env.data.qpos[0]), -2., 2.)
                teacher_obs[29] = 2.*env.data.qvel[0]
            with torch.inference_mode():
                action = actor(torch.as_tensor(teacher_obs)[None])[0].numpy()
            if switched and brake:
                action[[2, 5]] = np.clip(action[[2, 5]] + brake*env.data.qvel[0], -2., 2.)
        if env.decision.phase in (Phase.GETUP, Phase.HANDOFF):
            rows['observation'].append(np.array(observation, dtype=np.float32))
            rows['action'].append(action.copy())
            if learner is not None:
                with torch.inference_mode():
                    student = learner.actor(torch.as_tensor(observation, dtype=torch.float32)[None])[0].numpy()
                bound = np.array([1., 1., 2., 1., 1., 2.])
                action = (1-learner_mix)*np.clip(action, -bound, bound) + learner_mix*np.clip(student, -bound, bound)
        observation, _, done, truncated, info = env.step(action)
        if done or truncated:
            break

    metrics = np.asarray(trace['metrics'])
    within = (metrics[:, 2] <= .3)
    file = Path(destination)/f'candidate_{index:03d}_seed_{seed}_gain_{gain:.2f}.npz'
    np.savez_compressed(file, **trace, **rows)
    return {'candidate': index, 'seed': seed, 'gain': gain, 'trace': file.name,
            'success': bool(info['success']), 'failure': info['failure'],
            'phase_counts': info['phase_counts'], 'balance_ticks': info['balance_ticks'],
            'peak_clearance_while_within_30cm_m': float(metrics[within, 1].max()),
            'body_off_ground_ticks_while_within_30cm': int((within & (metrics[:, 8] == 0)).sum()),
            'max_displacement_m': float(metrics[:, 2].max()),
            'max_tilt_deg': float(abs(metrics[:, 3:5]).max()),
            'support_peak_force_n': float(metrics[:, 9].max()), 'support_peak_moment_nm': float(metrics[:, 10].max())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--balance', type=Path, required=True)
    parser.add_argument('--rise', type=Path)
    parser.add_argument('--follow', type=Path, help='teacher tracking policy before the frozen handoff')
    parser.add_argument('--learner', type=Path)
    parser.add_argument('--learner-mix', type=float, default=0.)
    parser.add_argument('--candidates', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--gains', type=float, nargs='+', default=[1., .8])
    parser.add_argument('--workers', type=int, default=8)
    parser.add_argument('--seed-base', type=int, default=910000)
    parser.add_argument('--seeds', type=int, default=2)
    parser.add_argument('--feedback', type=float, default=0.)
    parser.add_argument('--brake', type=float, default=0., help='teacher-only capture-point feedback from simulation truth')
    args = parser.parse_args()
    if not 0. <= args.learner_mix <= 1. or (args.learner_mix and args.learner is None):
        parser.error('learner mixing requires a checkpoint and a weight between 0 and 1')
    profile = json.loads(args.config.read_text())
    candidates = json.loads(args.candidates.read_text())
    args.out.mkdir(parents=True, exist_ok=False)
    tasks = [(profile, args.balance, args.rise, params, round(candidates['segment_seconds']/.01), gain, seed, index, args.out,
              args.feedback, args.brake, args.follow, candidates.get('slew', 1.), args.learner, args.learner_mix)
             for index, params in enumerate(candidates['candidates']) for gain in args.gains
             for seed in range(args.seed_base, args.seed_base+args.seeds)]
    with ProcessPoolExecutor(max_workers=args.workers) as pool, (args.out/'rows.jsonl').open('w') as stream:
        results = []
        for result in pool.map(replay, tasks):
            results.append(result)
            stream.write(json.dumps(result)+'\n')
            stream.flush()
            print(json.dumps(result), flush=True)
    report = {'configuration': profile, 'balance_sha256': digest(args.balance),
              'rise_sha256': digest(args.rise) if args.rise else None,
              'follow_sha256': digest(args.follow) if args.follow else None,
              'learner_sha256': digest(args.learner) if args.learner else None, 'learner_mix': args.learner_mix,
              'teacher_brake_gain': args.brake,
              'candidates_sha256': digest(args.candidates), 'rows': results}
    (args.out/'report.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
