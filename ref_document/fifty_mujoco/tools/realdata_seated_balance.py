#!/usr/bin/env python3
"""Fit a stationary handoff initializer under the updated seated-task dynamics."""

import argparse
import itertools
import json
from pathlib import Path
import time

import torch

from realdata_linear_teacher import encode
from realdata_train import SamplePolicy31, equilibrium_action, save_checkpoint, verified_artifact


def run(args):
    from fifty_mujoco.envs.realdata_warp import RealDataWarpEnv

    config = json.loads(args.config.read_text())
    limit = args.trials
    device = torch.device('cuda')
    grid = list(itertools.product([8., 12., 18., 24.], [.6, 1.2, 1.8, 2.5], [1.2, 2.5, 4., 6.], [0.5, 2., 4.]))
    env = RealDataWarpEnv(limit, config=config, episode_seconds=args.eval_seconds+1, capture=True)
    base = equilibrium_action(env.cpu)
    initial = torch.as_tensor(base, device=device, dtype=torch.float32)
    args.out.mkdir(parents=True, exist_ok=False)
    best = []
    start = time.monotonic()
    torch.manual_seed(810300)
    position = env.spawn.clone()
    policy = SamplePolicy31(verified_artifact('model_3050.pt'), 'cpu')
    for group in range(args.groups):
        if group*limit < len(grid):
            batch = grid[group*limit:(group+1)*limit]
            batch += [batch[-1]]*(limit-len(batch))
            coefficients = torch.tensor(batch, device=device)
        else:
            centers = torch.tensor([row['gains'] for row in best[:min(len(best), 12)]], device=device)
            offsets = torch.randn((limit, 4), device=device)*initial.new_tensor([3., .35, .75, .5])
            coefficients = (centers[torch.arange(limit, device=device)%len(centers)] + offsets).clamp(min=0.)
            coefficients[:len(centers)] = centers
        kp, kd, kv, kx = coefficients.unbind(-1)
        env.reset()
        env.qpos[:] = position
        env.qvel[:, 0] = (.006 if group%2==0 else -.006)
        bad = torch.zeros(limit, dtype=torch.bool, device=device)
        duration = torch.zeros(limit, device=device)
        strict = torch.zeros_like(duration)
        softness = torch.zeros_like(duration)
        with torch.no_grad():
            for _ in range(round(args.eval_seconds/.01)):
                obs = env.obs
                action = initial.expand(limit, 6).clone()
                action[:, 0] -= 2*obs[:, 3]
                action[:, 3] += 2*obs[:, 3]
                roll = -config['teacher_feedback']['roll_kp']*obs[:, 4] + 4*config['teacher_feedback']['roll_kd']*obs[:, 0]
                action[:, 1] += roll
                action[:, 4] += roll
                wheel = kp*obs[:, 3] + 4*kd*obs[:, 1] + kv*obs[:, 29]/2 - kx*obs[:, 25]/5
                turn = .6*obs[:, 2] - .15*obs[:, 27]
                action[:, 2] = wheel - turn
                action[:, 5] = wheel + turn
                env.step(action)
                bad.logical_or_(env.failure != 0)
                error = (env.clearance - config['standing_clearance_m']).abs()
                displacement = torch.linalg.vector_norm(env.qpos[:, :2], dim=-1)
                pitch = env.rpy[:, :2].abs().amax(-1)
                speed = env.observer.velocity[:, 0].abs()
                leg = env.obs[:, 9:13].abs().amax(-1)
                duration.add_(~bad)
                strict.add_(~bad & (error <= .02) & (displacement <= .3) & (pitch <= .1396263) & (speed <= .08) & (leg <= .15))
                softness.add_(torch.where(~bad, error + pitch + speed + displacement.clamp(max=1) + leg, 0))
        quality = strict + .1*duration - .25*softness
        for index in torch.nan_to_num(quality, nan=-1.e6).topk(12).indices.tolist():
            best.append({'gains': coefficients[index].tolist(), 'score': float(quality[index]),
                         'duration_s': float(duration[index])*.01, 'strict_seconds': float(strict[index])*.01})
        best.sort(key=lambda row: row['score'], reverse=True)
        best = best[:32]
        print(json.dumps({'group': group+1, 'best': best[0], 'wall_s': time.monotonic()-start}), flush=True)
    seen = set()
    rows = []
    for row in best:
        key = tuple(row['gains'])
        if key in seen:
            continue
        seen.add(key)
        pitch_kp, pitch_kd, velocity, position_gain = key
        feedback = config['teacher_feedback'] | {'pitch_kp': pitch_kp, 'pitch_kd': pitch_kd, 'velocity': velocity}
        encode(policy, base, feedback, position_gain)
        name = f'rank{len(rows):02d}.pt'
        save_checkpoint(args.out/name, {'schema': 'seated_getup_balance_initializer_v1',
                                        'configuration': config, 'model': policy.state_dict(), 'search_row': row})
        rows.append(row | {'checkpoint': name})
    (args.out/'search.json').write_text(json.dumps(rows, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--eval-seconds', type=float, default=10.)
    parser.add_argument('--trials', type=int, default=128)
    parser.add_argument('--groups', type=int, default=8)
    arguments = parser.parse_args()
    torch.set_num_threads(1)
    run(arguments)
